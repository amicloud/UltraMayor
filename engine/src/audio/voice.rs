use std::{collections::HashMap, f32::consts::PI, sync::Arc};

use bevy_ecs::entity::Entity;
use glam::Vec3;

use crate::audio::audio_mixer::ListenerInfo;

const ITD_DELAY_BUFFER_SIZE: usize = 128;
const PAN_SMOOTH_TIME_SECONDS: f32 = 0.05;
const BACK_LPF_MIX_MULT: f32 = 0.8;
const LPF_CUTOFF_HZ: f32 = 400.0;

#[derive(Debug)]
pub(crate) struct Voice {
    samples: Arc<[f32]>,
    sample_rate: f32,
    cursor: usize,
    volume: f32,
    looping: bool,
    pub(crate) channels: usize,
    pub(crate) buffer: Vec<f32>,
    source: Option<Entity>,
    source_channels: usize,
    location: Option<Vec3>,
    itd_delay: ItdDelay,
    lpf_left: LowPassFilter,
    lpf_right: LowPassFilter,
    pan_smoothed: f32,
}

#[derive(Debug)]
struct ItdDelay {
    buffer: [f32; ITD_DELAY_BUFFER_SIZE],
    write_idx: usize,
    mask: usize,  // buffer.len() - 1 (power-of-two size)
    range: usize, // Maximum delay in samples
}

#[derive(Debug)]
struct LowPassFilter {
    z: f32, // previous output
    alpha: f32,
}

impl LowPassFilter {
    fn process(&mut self, input: f32) -> f32 {
        let out = self.z + self.alpha * (input - self.z);
        self.z = out;
        out
    }
}

impl Voice {
    pub(crate) fn channels(&self) -> usize {
        self.channels
    }

    pub(crate) fn new(
        samples: Arc<[f32]>,
        sample_rate: f32,
        volume: f32,
        looping: bool,
        source: Option<Entity>,
        location: Option<Vec3>,
        source_channels: usize,
        required_buffer_size: usize,
    ) -> Self {
        let itd_scale = 1.0; // Scale factor for ITD effect, for demonstration purposes
        let itd_max_time_seconds = 0.67 / 1000.0; //0.67 ms converted to seconds

        // Calculate the maximum ITD delay in samples based on the desired time and sample rate, scaled by the ITD effect strength
        // We clamp it between 1 and the buffer size to avoid division by zero and ensure we do not overrun the buffer
        let itd_max_samples = ((itd_max_time_seconds * sample_rate * itd_scale) as usize)
            .clamp(1, ITD_DELAY_BUFFER_SIZE - 1); // Max delay in samples

        // Low pass filter coefficient for head shadow effect
        let alpha = 1.0 - (-2.0 * PI * LPF_CUTOFF_HZ / sample_rate).exp();
        Self {
            samples,
            cursor: 0,
            sample_rate,
            volume,
            looping,
            channels: 2, // We always output stereo from the voice, even if the source is mono. The mixer will handle downmixing if necessary.
            buffer: vec![0.0; required_buffer_size], // stereo output buffer
            source,
            source_channels,
            itd_delay: ItdDelay {
                buffer: [0.0; ITD_DELAY_BUFFER_SIZE],
                write_idx: 0,
                mask: ITD_DELAY_BUFFER_SIZE - 1, // buffer.len() - 1 (power-of-two size)
                range: itd_max_samples,
            },
            lpf_left: LowPassFilter { z: 0.0, alpha },
            lpf_right: LowPassFilter { z: 0.0, alpha },
            pan_smoothed: 0.0,
            location,
        }
    }

    pub(crate) fn next_block(
        &mut self,
        listener_info: Option<&ListenerInfo>,
        required_frames: usize,
        source_map: &HashMap<Entity, Vec3>,
    ) -> bool {
        let total_frames = self.samples.len() / self.source_channels;
        let frames_to_fill = (total_frames - self.cursor).min(required_frames);

        let mut location = self.location;
        // Simple pan based spatialization
        let mut pan = 0.0; // -1.0 = full left, 0.0 = center, 1.0 = full right
        if let Some(source) = self.source
            && let Some(_location) = source_map.get(&source)
        {
            location = Some(*_location);
        }

        let distance_attenuation =
            if let (Some(location), Some((listener_pos, _))) = (location, listener_info) {
                let distance = location.distance(*listener_pos);
                // Raw inverse square attenuation feels too harsh.
                // Perhaps this should be tweaked or made configurable, but for now
                // we'll just use a modified inverse square that falls off more gently.
                1.0 / (1.0 + (distance.powi(2) / 5.0))
            } else {
                1.0
            };
        let mut back_strength = 0.0;
        if let Some(location) = location
            && let Some((listener_pos, listener_rot)) = listener_info
        {
            let world_dir = (location - listener_pos).normalize_or_zero();

            let listener_right = listener_rot.mul_vec3(Vec3::X).normalize_or_zero();
            let listener_forward = listener_rot.mul_vec3(Vec3::Z).normalize_or_zero();

            // Pan from right-axis alignment, behind from forward-axis alignment
            pan = world_dir.dot(listener_right).clamp(-1.0, 1.0);
            let front_back = world_dir.dot(listener_forward).clamp(-1.0, 1.0);
            back_strength = (front_back).max(0.0);
        }

        let pan_smooth_alpha =
            1.0 - (-(required_frames as f32) / (self.sample_rate * PAN_SMOOTH_TIME_SECONDS)).exp();
        self.pan_smoothed += pan_smooth_alpha * (pan - self.pan_smoothed);

        let ild_strength = 0.33;
        let pan_scaled = self.pan_smoothed * ild_strength;

        let pan_rad = (pan_scaled + 1.0) * 0.25 * PI;
        let left_gain = pan_rad.cos();
        let right_gain = pan_rad.sin();

        // Interaural Time Difference
        // Even though one side will always have a 0 sample delay let's just do the math for both sides to avoid
        // doing a ton of if statement evaluations each frame.
        let delay_signed = self.pan_smoothed * self.itd_delay.range as f32;
        // let delay_signed:f32 = 0.0;
        let left_delay = delay_signed.max(0.0);
        let right_delay = (-delay_signed).max(0.0);
        let itd_range_f32 = self.itd_delay.range as f32;
        // The required delay can be fractional and we can achieve better quality with linear interpolation
        let left_delay_samples = left_delay as usize;
        let left_interpolation_factor = left_delay - left_delay_samples as f32;
        let right_delay_samples = right_delay as usize;
        let right_interpolation_factor = right_delay - right_delay_samples as f32;

        let combined_volume = self.volume * distance_attenuation;

        // Head shadow effect
        let behind_lpf_mix = BACK_LPF_MIX_MULT * back_strength;
        let left_itd_shadow = (left_delay / itd_range_f32).clamp(0.0, 1.0);
        let right_itd_shadow = (right_delay / itd_range_f32).clamp(0.0, 1.0);
        let left_shadow_mix =
            (left_itd_shadow + (1.0 - left_itd_shadow) * behind_lpf_mix).clamp(0.0, 1.0);
        let right_shadow_mix =
            (right_itd_shadow + (1.0 - right_itd_shadow) * behind_lpf_mix).clamp(0.0, 1.0);

        // Match outside of the loop for a tiny performance boost
        match self.source_channels {
            1 => {
                for frame in 0..frames_to_fill {
                    let sample_idx = self.cursor * self.source_channels;
                    let mono = self.samples[sample_idx] * combined_volume;

                    // +++ ITD +++
                    self.itd_delay.buffer[self.itd_delay.write_idx] = mono;

                    let left_read_base = self
                        .itd_delay
                        .write_idx
                        .wrapping_sub(left_delay_samples + 1)
                        & self.itd_delay.mask;
                    let left_read_next = (left_read_base + 1) & self.itd_delay.mask;

                    // Linear interpolation for fractional delay
                    let left_s0 = self.itd_delay.buffer[left_read_base];
                    let left_s1 = self.itd_delay.buffer[left_read_next];
                    let mut left_sample = left_s0 * (1.0 - left_interpolation_factor)
                        + left_s1 * left_interpolation_factor;

                    let right_read_base = self
                        .itd_delay
                        .write_idx
                        .wrapping_sub(right_delay_samples + 1)
                        & self.itd_delay.mask;
                    let right_read_next = (right_read_base + 1) & self.itd_delay.mask;

                    // Linear interpolation for fractional delay
                    let right_s0 = self.itd_delay.buffer[right_read_base];
                    let right_s1 = self.itd_delay.buffer[right_read_next];
                    let mut right_sample = right_s0 * (1.0 - right_interpolation_factor)
                        + right_s1 * right_interpolation_factor;
                    // --- ITD ---

                    // Simulating head shadow effect with low pass filter.
                    let left_filtered = self.lpf_left.process(left_sample);
                    let right_filtered = self.lpf_right.process(right_sample);

                    // Blend continuously to avoid discontinuities when crossing center.
                    left_sample =
                        left_sample * (1.0 - left_shadow_mix) + left_filtered * left_shadow_mix;
                    right_sample =
                        right_sample * (1.0 - right_shadow_mix) + right_filtered * right_shadow_mix;

                    self.buffer[frame * 2] = left_sample * left_gain;
                    self.buffer[frame * 2 + 1] = right_sample * right_gain;

                    self.itd_delay.write_idx = (self.itd_delay.write_idx + 1) & self.itd_delay.mask;

                    self.cursor += 1;
                }
            }
            2 => {
                for frame in 0..frames_to_fill {
                    let sample_idx = self.cursor * self.source_channels;
                    // Stereo source, apply panning and distance attenuation to each channel
                    // but not ITD since the source is already stereo and that would really mess things up
                    // Stereo voices should ideally not be used with spatialization but we should still support it in some way?
                    let left_sample = self.samples[sample_idx] * combined_volume;
                    let right_sample = self.samples[sample_idx + 1] * combined_volume;
                    self.buffer[frame * 2] = left_sample * left_gain; // Left channel
                    self.buffer[frame * 2 + 1] = right_sample * right_gain; // Right channel

                    self.cursor += 1;
                }
            }
            _ => {
                log::error!(
                    "Unsupported channel count: {}, expected 1 or 2",
                    self.source_channels
                );
                return false;
            }
        }

        // Zero the rest of the block if we ran out of frames
        for frame in frames_to_fill..required_frames {
            for ch in 0..self.channels {
                self.buffer[frame * self.channels + ch] = 0.0;
            }
        }
        if self.cursor >= total_frames {
            if self.looping {
                self.cursor = 0;
            } else {
                return false;
            }
        }
        self.looping || self.cursor < total_frames
    }
}
