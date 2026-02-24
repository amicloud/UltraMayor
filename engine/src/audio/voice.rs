use std::{collections::HashMap, f32::consts::PI, sync::Arc};

use bevy_ecs::entity::Entity;
use glam::Vec3;

use crate::audio::audio_mixer::ListenerInfo;

const DEFAULT_SAMPLE_RATE_HZ: f32 = 44_100.0;
const PAN_SMOOTH_TIME_SECONDS: f32 = 0.05;

#[derive(Debug)]
pub(crate) struct Voice {
    samples: Arc<[f32]>,
    cursor: usize,
    volume: f32,
    looping: bool,
    pub(crate) channels: usize,
    pub(crate) buffer: Vec<f32>,
    source: Option<Entity>,
    source_channels: usize,
    itd_delay: ItdDelay,
    lpf_left: LowPassFilter,
    lpf_right: LowPassFilter,
    pan_smoothed: f32,
}

const ITD_DELAY_BUFFER_SIZE: usize = 64; // Must be a power of two for efficient wrapping
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
        volume: f32,
        looping: bool,
        source: Option<Entity>,
        source_channels: usize,
        required_buffer_size: usize,
    ) -> Self {
        let itd_scale = 1.0; // Scale factor for ITD effect, for demonstration purposes
        let itd_max_time_seconds = 0.67 / 1000.0; //0.67 ms converted to seconds

        let sample_rate = 44100.0; // This should come in as an argument
        let itd_range = (itd_max_time_seconds * sample_rate * itd_scale) as usize; // Max delay in samples

        let low_pass_frequency_cutoff = 1500.0;
        let alpha = 1.0 - (-2.0 * PI * low_pass_frequency_cutoff / sample_rate).exp();

        Self {
            samples,
            cursor: 0,
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
                range: itd_range,
            },
            lpf_left: LowPassFilter { z: 0.0, alpha },
            lpf_right: LowPassFilter { z: 0.0, alpha },
            pan_smoothed: 0.0,
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

        let mut location = None;
        // Simple pan based spatialization
        let mut pan = 0.0; // -1.0 = full left, 0.0 = center, 1.0 = full right
        if let Some(source) = self.source {
            if let Some(_location) = source_map.get(&source) {
                location = Some(*_location);
            }
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

        if let Some(location) = location {
            if let Some((listener_pos, listener_rot)) = listener_info {
                let world_dir = (location - listener_pos).normalize_or_zero();

                // transform world direction into listener local space
                let local_dir = listener_rot.conjugate().mul_vec3(world_dir);

                // now pan is just local X
                pan = local_dir.x.clamp(-1.0, 1.0);
            }
        }

        let pan_smooth_alpha =
            1.0 - (-(required_frames as f32) / (DEFAULT_SAMPLE_RATE_HZ * PAN_SMOOTH_TIME_SECONDS)).exp();
        self.pan_smoothed += pan_smooth_alpha * (pan - self.pan_smoothed);

        let ild_strength = 0.33;
        let pan_scaled = self.pan_smoothed * ild_strength;

        let pan_rad = (pan_scaled + 1.0) * 0.25 * PI;
        let left_gain = pan_rad.cos();
        let right_gain = pan_rad.sin();

        let delay_signed = self.pan_smoothed * self.itd_delay.range as f32;
        let left_delay = delay_signed.max(0.0);
        let right_delay = (-delay_signed).max(0.0);
        let itd_range_f32 = self.itd_delay.range.max(1) as f32;
        let left_shadow_mix = (left_delay / itd_range_f32).clamp(0.0, 1.0);
        let right_shadow_mix = (right_delay / itd_range_f32).clamp(0.0, 1.0);
        let left_int = left_delay.floor() as usize;
        let left_frac = left_delay - left_int as f32;
        let right_int = right_delay.floor() as usize;
        let right_frac = right_delay - right_int as f32;

        let combined_volume = self.volume * distance_attenuation;

        for frame in 0..frames_to_fill {
            let sample_idx = self.cursor * self.source_channels;
            match self.source_channels {
                1 => {
                    let mono = self.samples[sample_idx] * combined_volume;
                    self.itd_delay.buffer[self.itd_delay.write_idx] = mono;

                    let left_read_base =
                        self.itd_delay.write_idx.wrapping_sub(left_int + 1) & self.itd_delay.mask;

                    let left_read_next = (left_read_base + 1) & self.itd_delay.mask;

                    let left_s0 = self.itd_delay.buffer[left_read_base];
                    let left_s1 = self.itd_delay.buffer[left_read_next];

                    let mut left_sample = left_s0 * left_frac + left_s1 * (1.0 - left_frac);

                    let right_read_base =
                        self.itd_delay.write_idx.wrapping_sub(right_int + 1) & self.itd_delay.mask;

                    let right_read_next = (right_read_base + 1) & self.itd_delay.mask;

                    let right_s0 = self.itd_delay.buffer[right_read_base];
                    let right_s1 = self.itd_delay.buffer[right_read_next];

                    let mut right_sample = right_s0 * right_frac + right_s1 * (1.0 - right_frac);

                    // Simulating head shadow effect with low pass filter.
                    // Blend continuously to avoid discontinuities when crossing center.
                    let left_filtered = self.lpf_left.process(left_sample);
                    let right_filtered = self.lpf_right.process(right_sample);
                    left_sample = left_sample * (1.0 - left_shadow_mix) + left_filtered * left_shadow_mix;
                    right_sample =
                        right_sample * (1.0 - right_shadow_mix) + right_filtered * right_shadow_mix;

                    self.buffer[frame * 2] = left_sample * left_gain;
                    self.buffer[frame * 2 + 1] = right_sample * right_gain;

                    self.itd_delay.write_idx = (self.itd_delay.write_idx + 1) & self.itd_delay.mask;
                }
                2 => {
                    // Stereo source, apply panning and distance attenuation to each channel
                    // but not ITD since the source is already stereo and that would really mess things up
                    let left_sample = self.samples[sample_idx] * combined_volume;
                    let right_sample = self.samples[sample_idx + 1] * combined_volume;
                    self.buffer[frame * 2] = left_sample * left_gain; // Left channel
                    self.buffer[frame * 2 + 1] = right_sample * right_gain; // Right channel
                }
                _ => {
                    // Unsupported channel count, silence output
                    if frame == 0 {
                        // Log this only for the first frame to avoid spamming the console
                        eprintln!(
                            "Unsupported channel count: {}, expected 1 or 2",
                            self.source_channels
                        );
                    }
                    self.buffer[frame * 2] = 0.0;
                    self.buffer[frame * 2 + 1] = 0.0;
                }
            }
            self.cursor += 1;
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
        return self.looping || self.cursor < total_frames;
    }
}
