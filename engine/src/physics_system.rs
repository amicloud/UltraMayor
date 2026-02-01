use crate::movement_system::MovementSystem;
use crate::physics_resource::PhysicsResource;
use crate::velocity_component::VelocityComponent;
use crate::WorldBasis;
use crate::{physics_component::PhysicsComponent, transform_component::TransformComponent};
use bevy_ecs::prelude::*;
pub struct PhysicsSystem {}

impl PhysicsSystem {
    pub fn update_bodies(
        mut query: Query<(
            &mut TransformComponent,
            &mut VelocityComponent,
            &PhysicsComponent,
        )>,
    ) {
        let delta_time = 1.0 / 60.0; // Assuming a fixed time step of 1/60 seconds
        let g = WorldBasis::gravity_vector();
        for (mut transform, mut velocity, physics) in query.iter_mut() {
            Self::update_body(&mut transform, &mut velocity, physics, delta_time, g);
        }
    }

    pub fn apply_impulses(
        mut query: Query<(&mut VelocityComponent, &PhysicsComponent)>,
        mut phys: ResMut<PhysicsResource>, // (entity, linear_impulse, angular_impulse)
    ) {
        for impulse in phys.impulses.drain(..) {
            println!("Applying impulse to entity {:?}", impulse.entity);
            if let Ok((mut velocity, physics)) = query.get_mut(impulse.entity) {
                Self::apply_impulse(&mut velocity, physics, impulse.linear, impulse.angular);
            }
        }
    }

    fn update_body(
        transform: &mut TransformComponent,
        velocity: &mut VelocityComponent,
        physics: &PhysicsComponent,
        delta_time: f32,
        gravity: glam::Vec3,
    ) {
        // Integrate gravity
        velocity.translational += gravity * delta_time;

        // Update position and rotation
        transform.position += velocity.translational * delta_time;
        transform.rotation =
            MovementSystem::apply_rotation(&transform.rotation, &velocity.angular, delta_time);

        // Update velocity based on drag (simple linear drag model)
        let drag_force = -velocity.translational * physics.drag_coefficient;
        let angular_drag_force = -velocity.angular * physics.angular_drag_coefficient;

        velocity.translational += (drag_force / physics.mass) * delta_time;
        velocity.angular += (angular_drag_force / physics.mass) * delta_time;
    }

    fn apply_impulse(
        velocity: &mut VelocityComponent,
        physics: &PhysicsComponent,
        linear: glam::Vec3,
        angular: glam::Vec3,
    ) {
        velocity.translational += linear / physics.mass;
        velocity.angular += angular / physics.mass;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use glam::{Quat, Vec3};

    fn physics_component() -> PhysicsComponent {
        PhysicsComponent {
            physics_type: crate::physics_component::PhysicsType::Dynamic,
            mass: 1.0,
            friction: 0.0,
            drag_coefficient: 0.1,
            angular_drag_coefficient: 0.2,
            restitution: 0.0,
        }
    }

    #[test]
    fn update_body_applies_gravity_and_drag() {
        let mut transform = TransformComponent::default();
        let mut velocity = VelocityComponent {
            translational: Vec3::new(1.0, 0.0, 0.0),
            angular: Vec3::ZERO,
        };
        let physics = physics_component();
        let delta_time = 1.0;
        let gravity = WorldBasis::gravity_vector();

        PhysicsSystem::update_body(&mut transform, &mut velocity, &physics, delta_time, gravity);

        let expected_position = Vec3::new(1.0, 0.0, -9.81);
        let expected_velocity = Vec3::new(0.9, 0.0, -8.829);
        assert_relative_eq!(transform.position.x, expected_position.x, epsilon = 1e-6);
        assert_relative_eq!(transform.position.y, expected_position.y, epsilon = 1e-6);
        assert_relative_eq!(transform.position.z, expected_position.z, epsilon = 1e-6);
        assert_relative_eq!(
            velocity.translational.x,
            expected_velocity.x,
            epsilon = 1e-6
        );
        assert_relative_eq!(
            velocity.translational.y,
            expected_velocity.y,
            epsilon = 1e-6
        );
        assert_relative_eq!(
            velocity.translational.z,
            expected_velocity.z,
            epsilon = 1e-6
        );
    }

    #[test]
    fn update_body_applies_rotation_from_angular_velocity() {
        let mut transform = TransformComponent::default();
        let mut velocity = VelocityComponent {
            translational: Vec3::ZERO,
            angular: Vec3::new(0.0, 0.0, 1.0),
        };
        let physics = physics_component();
        let delta_time = 1.0;

        PhysicsSystem::update_body(
            &mut transform,
            &mut velocity,
            &physics,
            delta_time,
            Vec3::ZERO,
        );

        let expected = Quat::from_axis_angle(Vec3::Z, 1.0);
        assert_relative_eq!(transform.rotation.x, expected.x, epsilon = 1e-6);
        assert_relative_eq!(transform.rotation.y, expected.y, epsilon = 1e-6);
        assert_relative_eq!(transform.rotation.z, expected.z, epsilon = 1e-6);
        assert_relative_eq!(transform.rotation.w, expected.w, epsilon = 1e-6);
    }

    #[test]
    fn apply_impulse_scales_by_mass() {
        let mut velocity = VelocityComponent::default();
        let mut physics = physics_component();
        physics.mass = 2.0;

        PhysicsSystem::apply_impulse(
            &mut velocity,
            &physics,
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 4.0, 0.0),
        );

        assert_relative_eq!(velocity.translational.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(velocity.translational.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(velocity.translational.z, 0.0, epsilon = 1e-6);
        assert_relative_eq!(velocity.angular.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(velocity.angular.y, 2.0, epsilon = 1e-6);
        assert_relative_eq!(velocity.angular.z, 0.0, epsilon = 1e-6);
    }
}
