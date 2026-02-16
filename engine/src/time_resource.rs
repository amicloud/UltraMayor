use std::time::Instant;

use bevy_ecs::prelude::*;
#[derive(Resource)]
pub struct TimeResource {
    dt: f32,
    simulation_fixed_dt: f32,
    total_time: f64,
    frame_count: u64,
    target_frame_duration: f32,
    last_frame_time: Instant
}

impl Default for TimeResource {
    fn default() -> Self {
        TimeResource {
            dt: 0.0,
            simulation_fixed_dt: 1.0 / 60.0, // Default to 60 Hz
            total_time: 0.0,
            frame_count: 0,
            target_frame_duration: 1.0 / 60.0, // Default to 60 FPS max,
            last_frame_time: Instant::now(),
        }
    }
}

#[allow(dead_code)]
impl TimeResource {
    pub fn new(target_frame_rate: u32, simulation_fixed_dt: f32) -> Self {
        let target_frame_time = 1.0 / target_frame_rate as f32;
        TimeResource {
            dt: 0.0,
            total_time: 0.0,
            frame_count: 0,
            target_frame_duration: target_frame_time,
            simulation_fixed_dt,
            last_frame_time: Instant::now(),
        }
    }

    pub fn set_simulation_fixed_dt(&mut self, fixed_dt: f32) {
        self.simulation_fixed_dt = fixed_dt;
    }

    pub fn simulation_fixed_dt(&self) -> f32 {
        self.simulation_fixed_dt
    }

    /// Alias for simulation_fixed_dt, for convenience
    pub fn fixed_dt(&self) -> f32 {
        self.simulation_fixed_dt
    }

    pub fn update_frame_dt(&mut self, delta_time: f32) {
        self.dt = delta_time;
        self.total_time += delta_time as f64;
        self.frame_count += 1;
    }

    pub fn frame_delta_time(&self) -> f32 {
        self.dt
    }

    pub fn total_time(&self) -> f64 {
        self.total_time
    }

    pub fn frame_count(&self) -> u64 {
        self.frame_count
    }

    pub fn target_frame_duration(&self) -> f32 {
        self.target_frame_duration
    }

    pub fn update_time_resource(mut time: ResMut<TimeResource>) {
        let now = Instant::now();
        let frame_time = now.duration_since(time.last_frame_time).as_secs_f32();
        time.last_frame_time = now;
        time.update_frame_dt(frame_time);
    }
}
