use std::{collections::HashMap, f32::consts::PI, sync::Arc};

use bevy_ecs::entity::Entity;
use glam::Vec3;

use crate::audio::audio_mixer::ListenerInfo;

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
    delay_buffer_left: [f32; 64],  // for ITD
    delay_buffer_right: [f32; 64], // for ITD
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
        Self {
            samples,
            cursor: 0,
            volume,
            looping,
            channels: 2, // We always output stereo from the voice, even if the source is mono. The mixer will handle downmixing if necessary.
            buffer: vec![0.0; required_buffer_size], // stereo output buffer
            source,
            source_channels,
            delay_buffer_left: [0.0; 64], // More than enough samples for ITD
            delay_buffer_right: [0.0; 64],
        }
    }

    pub(crate) fn next_block(
        &mut self,
        listener_info: Option<&ListenerInfo>,
        required_frames: usize,
        source_map: &HashMap<Entity, Vec3>,
    ) -> bool {
        let total_frames = self.samples.len() / self.channels;
        let frames_to_fill = (total_frames - self.cursor).min(required_frames);
        let itd_scale = 1.0; // Scale factor for ITD effect, for demonstration purposes
        let itd_max_time_seconds = 0.67 / 1000.0; //0.67 ms converted to seconds

        let sample_rate = 44100.0; // This should ideally come from the audio context
        let itd_range = (itd_max_time_seconds * sample_rate * itd_scale) as usize; // Max delay in samples

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
                let dir = (location - listener_pos).normalize_or_zero();
                let right = listener_rot.mul_vec3(Vec3::X);

                // project direction onto listener's horizontal plane (XY)
                let dir_horizontal = Vec3::new(dir.x, dir.y, 0.0).normalize_or_zero();

                // compute signed pan: dot with right vector
                pan = right.dot(dir_horizontal).clamp(-1.0, 1.0);
            }
        }
        let pan_rad = (pan + 1.0) * 0.25 * PI; // map -1..1 -> 0..π/2
        let left_gain = pan_rad.cos().max(0.1);
        let right_gain = pan_rad.sin().max(0.1);

        let itd_signed = pan * itd_range as f32;
        let mut left_delay = 0;
        let mut right_delay = 0;
        let max_allowable_delay = self.cursor.min(itd_range);

        if itd_signed > 0.0 {
            right_delay = (max_allowable_delay as f32 * pan_rad.sin() )as usize;
        }
        
        if itd_signed < 0.0 {
            left_delay = (max_allowable_delay as f32 * pan_rad.cos() )as usize;
        }

        for frame in 0..frames_to_fill {
            let sample_idx = self.cursor * self.source_channels;
            match self.source_channels {
                1 => {
                    // Mono source, apply same sample to both channels with panning and distance attenuation
                    let left_sample = self.samples[sample_idx - left_delay] * self.volume * distance_attenuation;
                    let right_sample =
                        self.samples[sample_idx - right_delay] * self.volume * distance_attenuation;
                    self.buffer[frame * 2] = left_sample * left_gain; // Left channel
                    self.buffer[frame * 2 + 1] = right_sample * right_gain; // Right channel
                }
                2 => {
                    // Stereo source, apply panning and distance attenuation to each channel
                    // but not ITD since the source is already stereo and that would really mess things up
                    let left_sample = self.samples[sample_idx ] * self.volume * distance_attenuation;
                    let right_sample =
                        self.samples[sample_idx + 1] * self.volume * distance_attenuation;
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
