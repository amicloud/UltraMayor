use std::time::{Duration, Instant};

use bevy_ecs::prelude::*;
#[derive(Resource)]
pub struct TimeResource {
    dt: f32,
    simulation_fixed_dt: Duration,
    total_time: f64,
    frame_count: u64,
    target_frame_duration: Duration,
    last_frame_time: Instant
}

impl Default for TimeResource {
    fn default() -> Self {
        TimeResource {
            dt: 0.0,
            simulation_fixed_dt: Duration::from_secs_f32(1.0 / 60.0), // Default to 60 Hz
            total_time: 0.0,
            frame_count: 0,
            target_frame_duration: Duration::from_secs_f32(1.0 / 60.0), // Default to 60 FPS max
            last_frame_time: Instant::now(),
        }
    }
}

#[allow(dead_code)]
impl TimeResource {
    pub fn new(target_frame_rate: u32, simulation_rate: u32) -> Self {
        let target_frame_time = Duration::from_secs_f32(1.0 / target_frame_rate as f32);
        let simulation_fixed_dt = Duration::from_secs_f32(1.0 / simulation_rate as f32);
        TimeResource {
            dt: 0.0,
            total_time: 0.0,
            frame_count: 0,
            target_frame_duration: target_frame_time,
            simulation_fixed_dt,
            last_frame_time: Instant::now(),
        }
    }

    pub fn set_simulation_fixed_dt(&mut self, fixed_dt: Duration) {
        self.simulation_fixed_dt = fixed_dt;
    }

    pub fn simulation_fixed_dt(&self) -> Duration {
        self.simulation_fixed_dt
    }

    /// Alias for simulation_fixed_dt, for convenience
    pub fn fixed_dt(&self) -> Duration {
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

    pub fn target_frame_duration(&self) -> Duration {
        self.target_frame_duration
    }

    pub fn update_time_resource(mut time: ResMut<TimeResource>) {
        let now = Instant::now();
        let frame_time = now.duration_since(time.last_frame_time).as_secs_f32();
        time.last_frame_time = now;
        time.update_frame_dt(frame_time);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy_ecs::schedule::Schedule;

    fn assert_f32_close(left: f32, right: f32, epsilon: f32) {
        assert!(
            (left - right).abs() <= epsilon,
            "expected {left} to be within {epsilon} of {right}"
        );
    }

    fn assert_f64_close(left: f64, right: f64, epsilon: f64) {
        assert!(
            (left - right).abs() <= epsilon,
            "expected {left} to be within {epsilon} of {right}"
        );
    }

    #[test]
    fn default_values_match_expected_60hz_defaults() {
        let time = TimeResource::default();

        assert_f32_close(time.frame_delta_time(), 0.0, 1e-6);
        assert_eq!(
            time.simulation_fixed_dt(),
            Duration::from_secs_f32(1.0 / 60.0)
        );
        assert_eq!(time.fixed_dt(), time.simulation_fixed_dt());
        assert_f64_close(time.total_time(), 0.0, 1e-12);
        assert_eq!(time.frame_count(), 0);
        assert_eq!(
            time.target_frame_duration(),
            Duration::from_secs_f32(1.0 / 60.0)
        );
    }

    #[test]
    fn new_initializes_requested_frame_and_simulation_rates() {
        let time = TimeResource::new(144, 120);

        assert_eq!(
            time.target_frame_duration(),
            Duration::from_secs_f32(1.0 / 144.0)
        );
        assert_eq!(
            time.simulation_fixed_dt(),
            Duration::from_secs_f32(1.0 / 120.0)
        );
        assert_eq!(time.fixed_dt(), time.simulation_fixed_dt());
        assert_f32_close(time.frame_delta_time(), 0.0, 1e-6);
        assert_f64_close(time.total_time(), 0.0, 1e-12);
        assert_eq!(time.frame_count(), 0);
    }

    #[test]
    fn set_simulation_fixed_dt_updates_both_fixed_dt_accessors() {
        let mut time = TimeResource::default();
        let updated = Duration::from_millis(20);

        time.set_simulation_fixed_dt(updated);

        assert_eq!(time.simulation_fixed_dt(), updated);
        assert_eq!(time.fixed_dt(), updated);
    }

    #[test]
    fn update_frame_dt_tracks_latest_dt_total_time_and_frame_count() {
        let mut time = TimeResource::default();

        time.update_frame_dt(0.016);
        assert_f32_close(time.frame_delta_time(), 0.016, 1e-6);
        assert_f64_close(time.total_time(), 0.016, 1e-9);
        assert_eq!(time.frame_count(), 1);

        time.update_frame_dt(0.033);
        assert_f32_close(time.frame_delta_time(), 0.033, 1e-6);
        assert_f64_close(time.total_time(), 0.049, 1e-9);
        assert_eq!(time.frame_count(), 2);
    }

    #[test]
    fn update_time_resource_system_updates_time_from_elapsed_in_world() {
        let mut world = World::new();
        let mut time = TimeResource::default();
        time.last_frame_time = Instant::now() - Duration::from_millis(25);
        world.insert_resource(time);

        let mut schedule = Schedule::default();
        schedule.add_systems(TimeResource::update_time_resource);
        schedule.run(&mut world);

        let time = world
            .get_resource::<TimeResource>()
            .expect("TimeResource should exist after system run");

        assert!(time.frame_delta_time() > 0.0);
        assert!(time.frame_delta_time() >= 0.015);
        assert!(time.frame_delta_time() < 1.0);
        assert_eq!(time.frame_count(), 1);
        assert!(time.total_time() > 0.0);
    }

    #[test]
    fn update_time_resource_system_accumulates_over_multiple_runs() {
        let mut world = World::new();
        let mut time = TimeResource::default();
        time.last_frame_time = Instant::now() - Duration::from_millis(5);
        world.insert_resource(time);

        let mut schedule = Schedule::default();
        schedule.add_systems(TimeResource::update_time_resource);

        schedule.run(&mut world);
        {
            let mut time = world
                .get_resource_mut::<TimeResource>()
                .expect("TimeResource should exist between runs");
            time.last_frame_time = Instant::now() - Duration::from_millis(7);
        }
        schedule.run(&mut world);

        let time = world
            .get_resource::<TimeResource>()
            .expect("TimeResource should exist after multiple runs");

        assert_eq!(time.frame_count(), 2);
        assert!(time.total_time() > 0.0);
        assert!(time.frame_delta_time() >= 0.004);
        assert!(time.frame_delta_time() < 1.0);
    }
}
