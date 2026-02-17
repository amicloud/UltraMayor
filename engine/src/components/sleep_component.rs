use bevy_ecs::component::Component;

#[derive(Component, Debug, Clone, Copy)]
pub struct SleepComponent {
    pub is_sleeping: bool,
    pub sleep_timer: f32,
    pub linear_threshold: f32,
    pub angular_threshold: f32,
    pub time_to_sleep: f32,
}

impl Default for SleepComponent {
    fn default() -> Self {
        Self {
            is_sleeping: false,
            sleep_timer: 0.0,
            linear_threshold: 0.05,
            angular_threshold: 0.05,
            time_to_sleep: 0.5,
        }
    }
}
