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
    delay_buffer_left: Vec<f32>,  // for ITD
    delay_buffer_right: Vec<f32>, // for ITD
}

impl Voice {
    pub(crate) fn channels(&self) -> usize {
        self.channels
    }

    pub(crate) fn new(
        samples: Arc<[f32]>,
        volume: f32,
        looping: bool,
        channels: usize,
        source: Option<Entity>,
        required_buffer_size: usize,
    ) -> Self {
        Self {
            samples,
            cursor: 0,
            volume,
            looping,
            channels,
            buffer: vec![0.0; required_buffer_size], // stereo output buffer
            source,
            delay_buffer_left: vec![0.0; 128], // More than enough samples for ITD
            delay_buffer_right: vec![0.0; 128],
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

        let mut location = None;
        // Simple pan based spatialization
        let mut pan = 0.0;
        if let Some(source) = self.source {
            if let Some(_location) = source_map.get(&source) {
                location = Some(*_location);
            }
        }

        let distance_attenuation =
            if let (Some(location), Some((listener_pos, _))) = (location, listener_info) {
                let distance = location.distance(*listener_pos);
                (1.0 / (1.0 + distance * distance)).min(0.01) 
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
        let left_gain = pan_rad.cos().max(0.1); // avoid complete silence in one ear
        let right_gain = pan_rad.sin().max(0.1); // avoid complete silence in one ear

        for frame in 0..frames_to_fill {
            let sample_idx = self.cursor * self.channels;
            if self.channels == 2 {
                self.buffer[frame * 2] =
                    self.samples[sample_idx] * self.volume * distance_attenuation * left_gain;
                self.buffer[frame * 2 + 1] =
                    self.samples[sample_idx + 1] * self.volume * distance_attenuation * right_gain;
            } else if self.channels == 1 {
                let sample = self.samples[self.cursor] * self.volume * distance_attenuation;
                self.buffer[frame * 2] = sample * left_gain;
                self.buffer[frame * 2 + 1] = sample * right_gain;
            } else {
                for ch in 0..self.channels {
                    self.buffer[frame * self.channels + ch] =
                        self.samples[sample_idx + ch] * self.volume * distance_attenuation;
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

        return self.cursor < total_frames || self.looping;
    }
}
