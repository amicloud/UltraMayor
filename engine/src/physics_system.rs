use std::collections::HashMap;

use crate::WorldBasis;
use crate::movement_system::MovementSystem;
use crate::physics_resource::{CollisionFrameData, ContactManifold, PhysicsFrameData};
use crate::time_resource::TimeResource;
use crate::velocity_component::VelocityComponent;
use crate::{
    physics_component::PhysicsComponent, sleep_component::SleepComponent,
    transform_component::TransformComponent,
};
use bevy_ecs::prelude::*;
use glam::{Mat3, Vec3};
pub struct PhysicsSystem {}

pub struct ContactConstraint {
    entity_a: Entity,
    entity_b: Entity,
    normal: Vec3,
    penetration: f32,
    accumulated_tangent_lambda: f32,
    accumulated_normal_lambda: f32,
    contact_point: Vec3, // world-space contact
}

const PGS_ITERATIONS: usize = 8;
const ENABLE_RESTING_STABILIZATION: bool = false;

impl PhysicsSystem {
    pub fn integrate_motion(
        mut query: Query<(
            &mut TransformComponent,
            &mut VelocityComponent,
            &PhysicsComponent,
            Option<&mut SleepComponent>,
        )>,
        time: Res<TimeResource>,
        world: Res<WorldBasis>,
    ) {
        let delta_time = time.simulation_fixed_dt().as_secs_f32();
        let g = world.gravity_vector();
        for (mut transform, mut velocity, physics, mut sleep) in query.iter_mut() {
            if !matches!(
                physics.physics_type,
                crate::physics_component::PhysicsType::Dynamic
            ) {
                continue;
            }

            if let Some(sleep) = sleep.as_deref_mut() {
                if sleep.is_sleeping {
                    velocity.translational = Vec3::ZERO;
                    velocity.angular = Vec3::ZERO;
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
                        velocity.translational = Vec3::ZERO;
                        velocity.angular = Vec3::ZERO;
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
        gravity: Vec3,
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

    fn manifold_to_constraints(manifold: &ContactManifold) -> Vec<ContactConstraint> {
        manifold
            .contacts
            .iter()
            .map(|contact| {
                let normal = if contact.normal.length_squared() > f32::EPSILON {
                    contact.normal
                } else {
                    manifold.normal
                };

                ContactConstraint {
                    entity_a: contact.entity_a,
                    entity_b: contact.entity_b,
                    normal,
                    penetration: contact.penetration,
                    accumulated_tangent_lambda: 0.0,
                    accumulated_normal_lambda: 0.0,
                    contact_point: contact.contact_point,
                }
            })
            .collect()
    }

    fn solve_constraint(
        constraint: &mut ContactConstraint,
        query: &mut Query<(
            &mut TransformComponent,
            Option<&mut VelocityComponent>,
            Option<&PhysicsComponent>,
        )>,
    ) {
        let Ok([mut a, mut b]) = query.get_many_mut([constraint.entity_a, constraint.entity_b])
        else {
            return;
        };

        let (transform_a, mut vel_a_opt, phys_a_opt) = (&mut a.0, a.1, a.2);
        let (transform_b, mut vel_b_opt, phys_b_opt) = (&mut b.0, b.1, b.2);

        // --- Physics properties ---
        let props_a = physics_props(phys_a_opt);
        let props_b = physics_props(phys_b_opt);

        let inv_mass_sum = props_a.inv_mass + props_b.inv_mass;
        if inv_mass_sum <= f32::EPSILON {
            return;
        }

        // --- Normal ---
        let normal = {
            let n2 = constraint.normal.length_squared();
            if n2 <= f32::EPSILON {
                return;
            }
            constraint.normal / n2.sqrt()
        };

        // --- Fetch velocities ---
        let mut v_a = vel_a_opt
            .as_ref()
            .map(|v| v.translational)
            .unwrap_or(Vec3::ZERO);
        let mut omega_a = vel_a_opt.as_ref().map(|v| v.angular).unwrap_or(Vec3::ZERO);
        let mut v_b = vel_b_opt
            .as_ref()
            .map(|v| v.translational)
            .unwrap_or(Vec3::ZERO);
        let mut omega_b = vel_b_opt.as_ref().map(|v| v.angular).unwrap_or(Vec3::ZERO);

        // --- Contact offsets ---
        let ra = constraint.contact_point - transform_a.position;
        let rb = constraint.contact_point - transform_b.position;

        // --- Relative velocity at contact ---
        let mut rv = (v_b + omega_b.cross(rb)) - (v_a + omega_a.cross(ra));
        let rvn = rv.dot(normal);
        if rvn > 0.0 {
            return;
        }

        // --- Restitution ---
        let restitution_threshold = 0.1;
        let restitution = if rvn < -restitution_threshold {
            // ((restitution_a.sqrt() + restitution_b.sqrt()) * 0.5).powi(2)
            f32::min(props_a.restitution, props_b.restitution)
        } else {
            0.0
        };

        // --- Effective mass for normal impulse ---
        let ra_cross_n = ra.cross(normal);
        let rb_cross_n = rb.cross(normal);
        let k = inv_mass_sum
            + normal.dot((props_a.inv_inertia * ra_cross_n).cross(ra))
            + normal.dot((props_b.inv_inertia * rb_cross_n).cross(rb));
        if k <= f32::EPSILON {
            return;
        }

        // --- Normal impulse ---
        let normal_impulse = -(1.0 + restitution) * rvn / k;
        let new_normal_lambda = (constraint.accumulated_normal_lambda + normal_impulse).max(0.0);
        let delta_normal = new_normal_lambda - constraint.accumulated_normal_lambda;
        constraint.accumulated_normal_lambda = new_normal_lambda;
        let impulse = normal * delta_normal;

        if let Some(vel_a) = vel_a_opt.as_mut() {
            vel_a.translational -= impulse * props_a.inv_mass;
            vel_a.angular -= props_a.inv_inertia * ra.cross(impulse);
            v_a = vel_a.translational;
            omega_a = vel_a.angular;
        }
        if let Some(vel_b) = vel_b_opt.as_mut() {
            vel_b.translational += impulse * props_b.inv_mass;
            vel_b.angular += props_b.inv_inertia * rb.cross(impulse);
            v_b = vel_b.translational;
            omega_b = vel_b.angular;
        }

        // --- Friction ---
        rv = (v_b + omega_b.cross(rb)) - (v_a + omega_a.cross(ra));
        let mut tangent = rv - normal * rv.dot(normal);
        let tangent_len = tangent.length();
        if tangent_len <= f32::EPSILON {
            return;
        }
        tangent /= tangent_len;

        let friction = (props_a.friction * props_b.friction).sqrt();
        if friction <= 0.0 {
            return;
        }

        // Effective mass for friction
        let ra_cross_t = ra.cross(tangent);
        let rb_cross_t = rb.cross(tangent);
        let k_t = inv_mass_sum
            + tangent.dot((props_a.inv_inertia * ra_cross_t).cross(ra))
            + tangent.dot((props_b.inv_inertia * rb_cross_t).cross(rb));
        if k_t <= f32::EPSILON {
            return;
        }

        let jt = -rv.dot(tangent) / k_t;
        let max_friction = friction * constraint.accumulated_normal_lambda;
        let new_tangent_lambda =
            (constraint.accumulated_tangent_lambda + jt).clamp(-max_friction, max_friction);
        let delta_tangent = new_tangent_lambda - constraint.accumulated_tangent_lambda;
        constraint.accumulated_tangent_lambda = new_tangent_lambda;
        let friction_impulse = tangent * delta_tangent;

        if let Some(vel_a) = vel_a_opt.as_mut() {
            vel_a.translational -= friction_impulse * props_a.inv_mass;
            vel_a.angular -= props_a.inv_inertia * ra.cross(friction_impulse);
        }
        if let Some(vel_b) = vel_b_opt.as_mut() {
            vel_b.translational += friction_impulse * props_b.inv_mass;
            vel_b.angular += props_b.inv_inertia * rb.cross(friction_impulse);
        }
    }

    fn positional_correction(
        contacts: &[ContactConstraint],
        query: &mut Query<(
            &mut TransformComponent,
            Option<&mut VelocityComponent>,
            Option<&PhysicsComponent>,
        )>,
    ) {
        // Parameters
        let slop = 0.025;
        let percent = 0.45;
        let max_correction = 2.0;

        // Track accumulated corrections per entity
        let mut corrections: HashMap<Entity, Vec3> = HashMap::new();

        for constraint in contacts {
            let Ok([mut a, mut b]) = query.get_many_mut([constraint.entity_a, constraint.entity_b])
            else {
                continue;
            };

            let (_, _, phys_a) = (&mut a.0, a.1, a.2);
            let (_, _, phys_b) = (&mut b.0, b.1, b.2);

            let props_a = physics_props(phys_a);
            let props_b = physics_props(phys_b);
            let inv_mass_sum = props_a.inv_mass + props_b.inv_mass;
            if inv_mass_sum <= f32::EPSILON {
                continue;
            }

            let normal = constraint.normal;
            let penetration = (constraint.penetration - slop).max(0.0);
            if penetration <= 0.0 {
                continue;
            }

            let correction_mag = (penetration * percent) / inv_mass_sum;
            let correction = normal * correction_mag;

            // Accumulate corrections
            corrections
                .entry(constraint.entity_a)
                .and_modify(|v| *v += -correction * props_a.inv_mass)
                .or_insert(-correction * props_a.inv_mass);

            corrections
                .entry(constraint.entity_b)
                .and_modify(|v| *v += correction * props_b.inv_mass)
                .or_insert(correction * props_b.inv_mass);
        }

        // Apply clamped corrections
        for (entity, delta) in corrections {
            let Ok(mut entry) = query.get_mut(entity) else {
                continue;
            };
            let clamped = delta.clamp_length_max(max_correction);
            entry.0.position += clamped;
        }
    }

    /// Applies extra damping for bodies that are in resting contact on support surfaces
    /// to reduce micro-jitter and slow spin accumulation.
    fn stabilize_resting_contacts(
        collision_frame_data: &CollisionFrameData,
        query: &mut Query<(
            &mut TransformComponent,
            Option<&mut VelocityComponent>,
            Option<&PhysicsComponent>,
        )>,
        gravity: Vec3,
    ) {
        use crate::physics_component::PhysicsType;

        let up = if gravity.length_squared() > f32::EPSILON {
            -gravity.normalize()
        } else {
            Vec3::Z
        };

        let support_dot_threshold = 0.5;
        let linear_rest_threshold = 1.0;
        let angular_rest_threshold = 2.0;

        for manifold in collision_frame_data.manifolds.values() {
            if manifold.contacts.is_empty() {
                continue;
            }

            let normal = if manifold.normal.length_squared() > f32::EPSILON {
                manifold.normal.normalize()
            } else {
                continue;
            };

            // Only apply this for support-like contacts (roughly aligned with up/down).
            if normal.dot(up).abs() < support_dot_threshold {
                continue;
            }

            for contact in &manifold.contacts {
                for entity in [contact.entity_a, contact.entity_b] {
                    let Ok((_, vel_opt, phys_opt)) = query.get_mut(entity) else {
                        continue;
                    };

                    let Some(physics) = phys_opt else {
                        continue;
                    };
                    if !matches!(physics.physics_type, PhysicsType::Dynamic) {
                        continue;
                    }

                    let Some(mut vel) = vel_opt else {
                        continue;
                    };

                    let linear_speed = vel.translational.length();
                    let angular_speed = vel.angular.length();

                    // Only stabilize nearly-resting bodies so active motion is unaffected.
                    if linear_speed < linear_rest_threshold
                        && angular_speed < angular_rest_threshold
                    {
                        // Weak rolling/spin damping while supported.
                        vel.angular *= 0.99;

                        let vertical_speed = vel.translational.dot(up);
                        if vertical_speed.abs() < 0.5 {
                            vel.translational -= up * vertical_speed;
                        }

                        // Dampen tiny horizontal drift that keeps contacts chattering.
                        let horizontal = vel.translational - up * vel.translational.dot(up);
                        if horizontal.length() < 0.8 {
                            vel.translational -= horizontal * 0.5;
                        }

                        // Hard lock very small residual motion so bodies fully settle.
                        if vel.angular.length() < 0.01 {
                            vel.angular = Vec3::ZERO;
                        }
                        if vel.translational.length() < 0.01 {
                            vel.translational = Vec3::ZERO;
                        }
                    }
                }
            }
        }
    }

    /// Resolves contacts from the collision system with PGS solver.
    pub fn physics_solver(
        mut query: Query<(
            &mut TransformComponent,
            Option<&mut VelocityComponent>,
            Option<&PhysicsComponent>,
        )>,
        collision_frame_data: Res<CollisionFrameData>,
        mut physics_frame_data: ResMut<PhysicsFrameData>,
        world: Res<WorldBasis>,
    ) {
        for manifold in collision_frame_data.manifolds.values() {
            physics_frame_data
                .constraints
                .extend(Self::manifold_to_constraints(manifold));
        }

        for _ in 0..=PGS_ITERATIONS {
            for constraint in &mut physics_frame_data.constraints {
                Self::solve_constraint(constraint, &mut query);
            }
        }

        Self::positional_correction(&physics_frame_data.constraints, &mut query);
        if ENABLE_RESTING_STABILIZATION {
            Self::stabilize_resting_contacts(
                &collision_frame_data,
                &mut query,
                world.gravity_vector(),
            );
        }
        physics_frame_data.clear();
    }
}

fn physics_props(physics: Option<&PhysicsComponent>) -> PhysicsProps {
    use crate::physics_component::PhysicsType;

    let Some(physics) = physics else {
        return PhysicsProps {
            inv_mass: 0.0,
            restitution: 0.0,
            friction: 0.0,
            inv_inertia: Mat3::ZERO,
        };
    };

    // --- Inverse mass ---
    let inv_mass = if matches!(physics.physics_type, PhysicsType::Dynamic) && physics.mass > 0.0 {
        1.0 / physics.mass
    } else {
        0.0
    };

    // --- Inverse inertia ---
    let inv_inertia = if matches!(physics.physics_type, PhysicsType::Dynamic) {
        // Avoid dividing by zero
        physics.local_inertia.inverse_or_zero()
    } else {
        Mat3::ZERO
    };

    PhysicsProps {
        inv_mass,
        restitution: physics.restitution,
        friction: physics.friction,
        inv_inertia,
    }
}

struct PhysicsProps {
    inv_mass: f32,
    restitution: f32,
    friction: f32,
    inv_inertia: Mat3,
}

#[cfg(test)]
mod tests {
    use crate::physics_component::PhysicsType;

    use super::*;
    use approx::assert_relative_eq;
    use glam::{Quat, Vec3};

    fn physics_component() -> PhysicsComponent {
        PhysicsComponent {
            physics_type: PhysicsType::Dynamic,
            mass: 1.0,
            friction: 0.0,
            drag_coefficient: 0.1,
            angular_drag_coefficient: 0.2,
            restitution: 0.0,
            local_inertia: glam::Mat3::IDENTITY,
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
        let gravity = WorldBasis::normal_gravity();

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
