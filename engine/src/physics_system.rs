use std::collections::HashMap;

use crate::movement_system::MovementSystem;
use crate::physics_component::PhysicsType;
use crate::physics_resource::{Contact, PhysicsResource};
use crate::velocity_component::VelocityComponent;
use crate::WorldBasis;
use crate::{
    physics_component::PhysicsComponent, sleep_component::SleepComponent,
    transform_component::TransformComponent,
};
use bevy_ecs::prelude::*;
use glam::Vec3;

#[derive(Clone)]
struct ContactManifold {
    normal: Vec3,
    contacts: Vec<Contact>,
}
pub struct PhysicsSystem {}

struct ContactConstraint {
    entity_a: Entity,
    entity_b: Entity,
    normal: Vec3,
    penetration: f32,
    accumulated_lambda: f32,
    accumulated_tangent_lambda: f32,
}

impl PhysicsSystem {
    pub fn update_bodies(
        mut query: Query<(
            &mut TransformComponent,
            &mut VelocityComponent,
            &PhysicsComponent,
            Option<&mut SleepComponent>,
        )>,
    ) {
        let delta_time = 1.0 / 60.0; // Assuming a fixed time step of 1/60 seconds
        let g = WorldBasis::gravity_vector();
        for (mut transform, mut velocity, physics, mut sleep) in query.iter_mut() {
            if !matches!(
                physics.physics_type,
                crate::physics_component::PhysicsType::Dynamic
            ) {
                continue;
            }

            if let Some(sleep) = sleep.as_deref_mut() {
                if sleep.is_sleeping {
                    velocity.translational = glam::Vec3::ZERO;
                    velocity.angular = glam::Vec3::ZERO;
                    continue;
                }
            }

            Self::update_body(&mut transform, &mut velocity, physics, delta_time, g);

            if let Some(sleep) = sleep.as_deref_mut() {
                let linear_speed = velocity.translational.length();
                let angular_speed = velocity.angular.length();
                if linear_speed < sleep.linear_threshold && angular_speed < sleep.angular_threshold
                {
                    sleep.sleep_timer += delta_time;
                    if sleep.sleep_timer >= sleep.time_to_sleep {
                        sleep.is_sleeping = true;
                        velocity.translational = glam::Vec3::ZERO;
                        velocity.angular = glam::Vec3::ZERO;
                    }
                } else {
                    sleep.sleep_timer = 0.0;
                }
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

    fn generate_manifolds_from_contacts(contacts: &Vec<Contact>) -> Vec<ContactManifold> {
        let mut manifolds: HashMap<(Entity, Entity), ContactManifold> = HashMap::new();

        for contact in contacts {
            let key = (contact.entity_a, contact.entity_b);

            if let Some(manifold) = manifolds.get_mut(&key) {
                manifold.contacts.push(*contact);
                manifold.normal += contact.normal;
                continue;
            }

            let reversed_key = (contact.entity_b, contact.entity_a);
            if let Some(manifold) = manifolds.get_mut(&reversed_key) {
                let mut flipped = *contact;
                flipped.entity_a = contact.entity_b;
                flipped.entity_b = contact.entity_a;
                flipped.normal = -contact.normal;
                manifold.contacts.push(flipped);
                manifold.normal += flipped.normal;
                continue;
            }

            manifolds.insert(
                key,
                ContactManifold {
                    normal: contact.normal,
                    contacts: vec![*contact],
                },
            );
        }

        manifolds
            .into_values()
            .map(|mut manifold| {
                if manifold.normal.length_squared() > f32::EPSILON {
                    manifold.normal = manifold.normal.normalize();
                } else if let Some(first) = manifold.contacts.first() {
                    manifold.normal = first.normal;
                } else {
                    manifold.normal = Vec3::ZERO;
                }
                manifold
            })
            .collect()
    }

    fn manifold_to_constraints(manifold: &ContactManifold) -> Vec<ContactConstraint> {
        manifold
            .contacts
            .iter()
            .map(|contact| ContactConstraint {
                entity_a: contact.entity_a,
                entity_b: contact.entity_b,
                normal: manifold.normal,
                penetration: contact.penetration,
                accumulated_lambda: 0.0,
                accumulated_tangent_lambda: 0.0,
            })
            .collect()
    }

    fn solve_constraint(
        constraint: &mut ContactConstraint,
        query: &mut Query<(Option<&mut VelocityComponent>, Option<&PhysicsComponent>)>,
    ) {
        let Ok([mut a, mut b]) = query.get_many_mut([constraint.entity_a, constraint.entity_b])
        else {
            return;
        };

        let (vel_a_opt, phys_a_opt) = (&mut a.0, a.1);
        let (vel_b_opt, phys_b_opt) = (&mut b.0, b.1);

        let (inv_mass_a, restitution_a, friction_a) = physics_props(phys_a_opt);
        let (inv_mass_b, restitution_b, friction_b) = physics_props(phys_b_opt);

        let inv_mass_sum = inv_mass_a + inv_mass_b;
        if inv_mass_sum <= f32::EPSILON {
            return;
        }

        let normal = if constraint.normal.length_squared() > f32::EPSILON {
            constraint.normal.normalize()
        } else {
            return;
        };

        let mut v_a = vel_a_opt
            .as_ref()
            .map(|v| v.translational)
            .unwrap_or(Vec3::ZERO);
        let mut v_b = vel_b_opt
            .as_ref()
            .map(|v| v.translational)
            .unwrap_or(Vec3::ZERO);

        let mut rv = v_b - v_a;
        let rvn = rv.dot(normal);

        let restitution_threshold = 0.5;
        let restitution = if rvn < -restitution_threshold {
            restitution_a.min(restitution_b)
        } else {
            0.0
        };

        let baumgarte = 0.08;
        let slop = 0.005;
        let max_correction = 0.2;
        let penetration_correction = (constraint.penetration - slop).clamp(0.0, max_correction);
        let bias = (baumgarte / fixed_dt()) * penetration_correction;

        let impulse_mag = (-(1.0 + restitution) * rvn + bias) / inv_mass_sum;
        let new_lambda = (constraint.accumulated_lambda + impulse_mag).max(0.0);
        let delta_lambda = new_lambda - constraint.accumulated_lambda;
        constraint.accumulated_lambda = new_lambda;

        let impulse = normal * delta_lambda;
        if let Some(vel_a) = vel_a_opt.as_mut() {
            vel_a.translational -= impulse * inv_mass_a;
            v_a = vel_a.translational;
        } else {
            v_a -= impulse * inv_mass_a;
        }
        if let Some(vel_b) = vel_b_opt.as_mut() {
            vel_b.translational += impulse * inv_mass_b;
            v_b = vel_b.translational;
        } else {
            v_b += impulse * inv_mass_b;
        }

        rv = v_b - v_a;
        let mut tangent = rv - normal * rv.dot(normal);
        let tangent_len = tangent.length();
        if tangent_len <= f32::EPSILON {
            return;
        }
        tangent /= tangent_len;

        let friction = (friction_a * friction_b).sqrt();
        if friction <= 0.0 {
            return;
        }

        let jt = -rv.dot(tangent) / inv_mass_sum;
        let max_friction = friction * constraint.accumulated_lambda;
        let new_tangent_lambda =
            (constraint.accumulated_tangent_lambda + jt).clamp(-max_friction, max_friction);
        let delta_tangent = new_tangent_lambda - constraint.accumulated_tangent_lambda;
        constraint.accumulated_tangent_lambda = new_tangent_lambda;

        let friction_impulse = tangent * delta_tangent;
        if let Some(vel_a) = vel_a_opt.as_mut() {
            vel_a.translational -= friction_impulse * inv_mass_a;
        }
        if let Some(vel_b) = vel_b_opt.as_mut() {
            vel_b.translational += friction_impulse * inv_mass_b;
        }
    }

    /// Resolves contacts from the collision system with PGS solver.
    pub fn pgs_solver(
        mut query: Query<(Option<&mut VelocityComponent>, Option<&PhysicsComponent>)>,
        mut phys: ResMut<PhysicsResource>,
    ) {
        let iterations = 8;
        let manifolds = Self::generate_manifolds_from_contacts(&phys.contacts);
        let mut constraints: Vec<Vec<ContactConstraint>> = manifolds
            .iter()
            .map(|m| Self::manifold_to_constraints(m))
            .collect();

        for _ in 0..iterations {
            for constraint_set in &mut constraints {
                for constraint in constraint_set.iter_mut() {
                    Self::solve_constraint(constraint, &mut query);
                }
            }
        }

        phys.contacts.clear();
    }
}

fn physics_props(physics: Option<&PhysicsComponent>) -> (f32, f32, f32) {
    let Some(physics) = physics else {
        return (0.0, 0.0, 0.0);
    };

    let inv_mass = if matches!(physics.physics_type, PhysicsType::Dynamic) && physics.mass > 0.0 {
        1.0 / physics.mass
    } else {
        0.0
    };

    (inv_mass, physics.restitution, physics.friction)
}

fn fixed_dt() -> f32 {
    1.0 / 60.0
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
}
