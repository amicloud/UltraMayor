use bevy_ecs::prelude::{Changed, Entity, Query, Res, ResMut};
use glam::{Mat4, Vec3};
use std::collections::HashMap;

use crate::{
    TransformComponent,
    collider_component::{Collider, ConvexCollider, ConvexShape, MeshCollider, Triangle},
    mesh::AABB,
    physics_component::{PhysicsComponent, PhysicsType},
    physics_resource::{Contact, Impulse, PhysicsResource},
    render_resource_manager::RenderResourceManager,
    velocity_component::VelocityComponent,
};

#[derive(Default)]
pub struct CollisionSystem {}

impl CollisionSystem {
    pub fn update_world_aabb_cache(
        query: Query<
            (
                Entity,
                &TransformComponent,
                Option<&ConvexCollider>,
                Option<&MeshCollider>,
            ),
            Changed<TransformComponent>,
        >,
        render_resources: Res<RenderResourceManager>,
        mut phys: ResMut<PhysicsResource>,
    ) {
        for (entity, transform, convex_collider, mesh_collider) in &query {
            if let Some(mesh_collider) = mesh_collider {
                if let Some(local_aabb) = render_body_local_aabb(
                    mesh_collider.render_body_id,
                    &render_resources,
                ) {
                    let world_aabb = transform_aabb(local_aabb, transform);
                    phys.world_aabbs.insert(entity, world_aabb);
                    continue;
                }
            }

            if let Some(convex_collider) = convex_collider {
                let world_aabb = convex_collider.aabb(&transform.to_mat4());
                phys.world_aabbs.insert(entity, world_aabb);
            }
        }
    }

    pub fn generate_contacts(
        moving_query: Query<
            (
                Entity,
                &TransformComponent,
                Option<&VelocityComponent>,
                Option<&ConvexCollider>,
                Option<&MeshCollider>,
            ),
            Changed<TransformComponent>,
        >,
        all_query: Query<(
            Entity,
            &TransformComponent,
            Option<&VelocityComponent>,
            Option<&ConvexCollider>,
            Option<&MeshCollider>,
        )>,
        render_resources: Res<RenderResourceManager>,
        mut phys: ResMut<PhysicsResource>,
    ) {
        // Collect moving entities
        let moving_entities: Vec<Entity> =
            moving_query.iter().map(|(e, _, _, _, _)| e).collect();
        let mut contacts = Vec::new();

        // Iterate over moving entities only
        for &entity_a in &moving_entities {
            let Some(aabb_a) = phys.world_aabbs.get(&entity_a) else {
                continue;
            };

            let Ok((_, transform_a, velocity_a, convex_a, mesh_a)) =
                moving_query.get(entity_a)
            else {
                continue;
            };

            // Compare against all colliders (moving + static)
            for (entity_b, transform_b, velocity_b, convex_b, mesh_b) in &all_query {
                if entity_a == entity_b {
                    continue;
                } // skip self

                let Some(aabb_b) = phys.world_aabbs.get(&entity_b) else {
                    continue;
                };

                let mut aabb_a_swept = *aabb_a;
                if let Some(velocity) = velocity_a {
                    let delta = velocity.translational * fixed_dt();
                    if delta.length_squared() > 0.0 {
                        aabb_a_swept = swept_aabb(aabb_a, delta);
                    }
                }

                if aabb_intersects(&aabb_a_swept, aabb_b) {
                    if let (Some(convex_a), Some(convex_b)) = (convex_a, convex_b) {
                        if matches!(convex_a.shape, ConvexShape::Cuboid { .. })
                            && matches!(convex_b.shape, ConvexShape::Cuboid { .. })
                        {
                            if let Some(contact) =
                                box_box_contact(entity_a, aabb_a, entity_b, aabb_b)
                            {
                                contacts.push(contact);
                            }
                            continue;
                        }
                    }

                    if let (Some(convex_a), Some(mesh_b)) = (convex_a, mesh_b) {
                        if matches!(convex_a.shape, ConvexShape::Sphere { .. }) {
                            if let Some(contact) = sphere_mesh_contact(
                                entity_a,
                                convex_a,
                                transform_a,
                                velocity_a,
                                entity_b,
                                mesh_b,
                                transform_b,
                                &render_resources,
                            ) {
                                contacts.push(contact);
                            }
                            continue;
                        }

                        if matches!(convex_a.shape, ConvexShape::Cuboid { .. }) {
                            if let Some(contact) = box_mesh_contact(
                                entity_a,
                                convex_a,
                                transform_a,
                                velocity_a,
                                entity_b,
                                mesh_b,
                                transform_b,
                                &render_resources,
                            ) {
                                contacts.push(contact);
                            }
                            continue;
                        }
                    }

                    if let (Some(mesh_a), Some(convex_b)) = (mesh_a, convex_b) {
                        if matches!(convex_b.shape, ConvexShape::Sphere { .. }) {
                            if let Some(contact) = sphere_mesh_contact(
                                entity_b,
                                convex_b,
                                transform_b,
                                velocity_b,
                                entity_a,
                                mesh_a,
                                transform_a,
                                &render_resources,
                            ) {
                                contacts.push(contact);
                            }
                            continue;
                        }

                        if matches!(convex_b.shape, ConvexShape::Cuboid { .. }) {
                            if let Some(contact) = box_mesh_contact(
                                entity_b,
                                convex_b,
                                transform_b,
                                velocity_b,
                                entity_a,
                                mesh_a,
                                transform_a,
                                &render_resources,
                            ) {
                                contacts.push(contact);
                            }
                            continue;
                        }
                    }
                }
            }
        }

        for contact in contacts {
            phys.add_contact(contact);
        }
    }

    pub fn resolve_contacts(
        query: Query<(Option<&VelocityComponent>, Option<&PhysicsComponent>)>,
        mut phys: ResMut<PhysicsResource>,
        mut transforms: Query<&mut TransformComponent>,
    ) {
        let mut impulses = Vec::new();
        let mut corrections: HashMap<Entity, Vec3> = HashMap::new();
        for contact in phys.contacts.iter() {
            let (vel_a, phys_a) = query
                .get(contact.entity_a)
                .map(|(v, p)| (v.map(|v| v.translational), p))
                .unwrap_or((None, None));
            let (vel_b, phys_b) = query
                .get(contact.entity_b)
                .map(|(v, p)| (v.map(|v| v.translational), p))
                .unwrap_or((None, None));

            let velocity_a = vel_a.unwrap_or(Vec3::ZERO);
            let velocity_b = vel_b.unwrap_or(Vec3::ZERO);

            let inv_mass_a = phys_a
                .filter(|p| matches!(p.physics_type, PhysicsType::Dynamic))
                .map(|p| if p.mass > 0.0 { 1.0 / p.mass } else { 0.0 })
                .unwrap_or(0.0);
            let inv_mass_b = phys_b
                .filter(|p| matches!(p.physics_type, PhysicsType::Dynamic))
                .map(|p| if p.mass > 0.0 { 1.0 / p.mass } else { 0.0 })
                .unwrap_or(0.0);
            let inv_mass_sum = inv_mass_a + inv_mass_b;

            if inv_mass_sum == 0.0 {
                continue;
            }

            let restitution = match (phys_a, phys_b) {
                (Some(a), Some(b)) => a.restitution.min(b.restitution),
                (Some(a), None) => a.restitution,
                (None, Some(b)) => b.restitution,
                (None, None) => 0.0,
            };

            let relative_velocity = velocity_b - velocity_a;
            let vel_along_normal = relative_velocity.dot(contact.normal);

            let penetration_slop = 0.01;
            let resting_threshold = 0.2;
            if vel_along_normal.abs() < resting_threshold
                && contact.penetration <= penetration_slop
            {
                continue;
            }
            let effective_restitution = if vel_along_normal.abs() < resting_threshold {
                0.0
            } else {
                restitution
            };

            let mut normal_impulse = 0.0;
            if vel_along_normal < 0.0 {
                normal_impulse = (-(1.0 + effective_restitution) * vel_along_normal) / inv_mass_sum;
                let impulse = contact.normal * normal_impulse;
                impulses.push(Impulse {
                    entity: contact.entity_a,
                    linear: -impulse,
                    angular: Vec3::ZERO,
                });
                impulses.push(Impulse {
                    entity: contact.entity_b,
                    linear: impulse,
                    angular: Vec3::ZERO,
                });
            }

            if normal_impulse > 0.0 {
                let tangent = relative_velocity - vel_along_normal * contact.normal;
                let tangent_len = tangent.length();
                if tangent_len > 1e-6 {
                    let tangent_dir = tangent / tangent_len;
                    let jt = -relative_velocity.dot(tangent_dir) / inv_mass_sum;

                    let friction = match (phys_a, phys_b) {
                        (Some(a), Some(b)) => (a.friction * b.friction).sqrt(),
                        (Some(a), None) => a.friction,
                        (None, Some(b)) => b.friction,
                        (None, None) => 0.0,
                    };

                    let max_friction = friction * normal_impulse;
                    let friction_impulse = jt.clamp(-max_friction, max_friction);
                    let impulse = tangent_dir * friction_impulse;
                    impulses.push(Impulse {
                        entity: contact.entity_a,
                        linear: -impulse,
                        angular: Vec3::ZERO,
                    });
                    impulses.push(Impulse {
                        entity: contact.entity_b,
                        linear: impulse,
                        angular: Vec3::ZERO,
                    });
                }
            }

            let correction_percent = 0.2;
            let penetration = (contact.penetration - penetration_slop).max(0.0);
            if penetration > 0.0 {
                let correction =
                    contact.normal * (penetration * correction_percent / inv_mass_sum);

                if inv_mass_a > 0.0 {
                    let entry = corrections.entry(contact.entity_a).or_insert(Vec3::ZERO);
                    *entry -= correction * inv_mass_a;
                }
                if inv_mass_b > 0.0 {
                    let entry = corrections.entry(contact.entity_b).or_insert(Vec3::ZERO);
                    *entry += correction * inv_mass_b;
                }
            }
        }
        for impulse in impulses {
            phys.add_impulse(impulse.entity, impulse.linear, impulse.angular);
        }
        for (entity, correction) in corrections {
            if let Ok(mut transform) = transforms.get_mut(entity) {
                transform.position += correction;
            }
        }
        phys.contacts.clear();
    }
}

fn transform_aabb(local: AABB, transform: &TransformComponent) -> AABB {
    let matrix = transform.to_mat4();
    transform_aabb_with_mat4(local, &matrix)
}

fn transform_aabb_with_mat4(local: AABB, transform: &Mat4) -> AABB {
    let min = local.min;
    let max = local.max;

    let corners = [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(max.x, max.y, max.z),
    ];

    let mut world_min = transform.transform_point3(corners[0]);
    let mut world_max = world_min;

    for corner in corners.iter().skip(1) {
        let world = transform.transform_point3(*corner);
        world_min = world_min.min(world);
        world_max = world_max.max(world);
    }

    AABB {
        min: world_min,
        max: world_max,
    }
}

fn aabb_intersects(a: &AABB, b: &AABB) -> bool {
    (a.min.x <= b.max.x && a.max.x >= b.min.x)
        && (a.min.y <= b.max.y && a.max.y >= b.min.y)
        && (a.min.z <= b.max.z && a.max.z >= b.min.z)
}

fn box_box_contact(
    entity_a: Entity,
    aabb_a: &AABB,
    entity_b: Entity,
    aabb_b: &AABB,
) -> Option<Contact> {
    let delta = (aabb_b.min + aabb_b.max) * 0.5 - (aabb_a.min + aabb_a.max) * 0.5;
    let overlap_x = (aabb_a.max.x - aabb_a.min.x + aabb_b.max.x - aabb_b.min.x) * 0.5
        - delta.x.abs();
    let overlap_y = (aabb_a.max.y - aabb_a.min.y + aabb_b.max.y - aabb_b.min.y) * 0.5
        - delta.y.abs();
    let overlap_z = (aabb_a.max.z - aabb_a.min.z + aabb_b.max.z - aabb_b.min.z) * 0.5
        - delta.z.abs();

    let (penetration, normal) = if overlap_x < overlap_y && overlap_x < overlap_z {
        (overlap_x, Vec3::new(delta.x.signum(), 0.0, 0.0))
    } else if overlap_y < overlap_z {
        (overlap_y, Vec3::new(0.0, delta.y.signum(), 0.0))
    } else {
        (overlap_z, Vec3::new(0.0, 0.0, delta.z.signum()))
    };

    Some(Contact {
        entity_a,
        entity_b,
        normal,
        penetration,
    })
}

fn box_mesh_contact(
    box_entity: Entity,
    box_collider: &ConvexCollider,
    box_transform: &TransformComponent,
    box_velocity: Option<&VelocityComponent>,
    mesh_entity: Entity,
    mesh_collider: &MeshCollider,
    mesh_transform: &TransformComponent,
    render_resources: &RenderResourceManager,
) -> Option<Contact> {
    let Some(_) = box_collider.as_cuboid() else {
        return None;
    };
    let render_body = render_resources
        .render_body_manager
        .get_render_body(mesh_collider.render_body_id)?;

    let box_aabb_world = box_collider.aabb(&box_transform.to_mat4());
    let box_world = box_transform.to_mat4();
    let mesh_entity_world = mesh_transform.to_mat4();

    for part in &render_body.parts {
        let mesh = render_resources.mesh_manager.get_mesh(part.mesh_id)?;
        let bvh = mesh.bvh.as_ref()?;

        let mesh_world = mesh_entity_world * part.local_transform;
        let mesh_world_inv = mesh_world.try_inverse()?;
        if let Some(contact) = box_mesh_contact_at_transform(
            box_entity,
            mesh_entity,
            box_collider,
            box_transform,
            box_world,
            &mesh_world,
            &mesh_world_inv,
            bvh,
        ) {
            return Some(contact);
        }

        if let Some(velocity) = box_velocity {
            let delta = velocity.translational * fixed_dt();
            let distance = delta.length();
            if distance > 0.0 {
                let step = (box_aabb_world.max - box_aabb_world.min)
                    .abs()
                    .min_element()
                    .max(0.01)
                    * 0.5;
                let steps = ((distance / step).ceil() as i32).clamp(1, 20);

                for i in 1..=steps {
                    let t = i as f32 / steps as f32;
                    let swept_position = box_transform.position + delta * t;
                    let swept_transform = TransformComponent {
                        position: swept_position,
                        rotation: box_transform.rotation,
                        scale: box_transform.scale,
                    };
                    let swept_world = swept_transform.to_mat4();
                    if let Some(contact) = box_mesh_contact_at_transform(
                        box_entity,
                        mesh_entity,
                        box_collider,
                        &swept_transform,
                        swept_world,
                        &mesh_world,
                        &mesh_world_inv,
                        bvh,
                    ) {
                        return Some(contact);
                    }
                }
            }
        }
    }

    None
}

fn sphere_mesh_contact(
    sphere_entity: Entity,
    sphere_collider: &ConvexCollider,
    sphere_transform: &TransformComponent,
    sphere_velocity: Option<&VelocityComponent>,
    mesh_entity: Entity,
    mesh_collider: &MeshCollider,
    mesh_transform: &TransformComponent,
    render_resources: &RenderResourceManager,
) -> Option<Contact> {
    let Some(radius) = sphere_collider.as_sphere_radius() else {
        return None;
    };
    let render_body = render_resources
        .render_body_manager
        .get_render_body(mesh_collider.render_body_id)?;

    let sphere_world = sphere_transform.to_mat4();
    let mesh_entity_world = mesh_transform.to_mat4();

    for part in &render_body.parts {
        let mesh = render_resources.mesh_manager.get_mesh(part.mesh_id)?;
        let bvh = mesh.bvh.as_ref()?;

        let mesh_world = mesh_entity_world * part.local_transform;
        let mesh_world_inv = mesh_world.try_inverse()?;
        if let Some(contact) = sphere_mesh_contact_at_transform(
            sphere_entity,
            mesh_entity,
            radius,
            sphere_transform,
            sphere_world,
            &mesh_world,
            &mesh_world_inv,
            bvh,
        ) {
            return Some(contact);
        }

        if let Some(velocity) = sphere_velocity {
            let delta = velocity.translational * fixed_dt();
            let distance = delta.length();
            if distance > 0.0 {
                let step = (radius * sphere_transform.scale.max_element())
                    .max(0.01)
                    * 0.5;
                let steps = ((distance / step).ceil() as i32).clamp(1, 20);

                for i in 1..=steps {
                    let t = i as f32 / steps as f32;
                    let swept_position = sphere_transform.position + delta * t;
                    let swept_transform = TransformComponent {
                        position: swept_position,
                        rotation: sphere_transform.rotation,
                        scale: sphere_transform.scale,
                    };
                    let swept_world = swept_transform.to_mat4();
                    if let Some(contact) = sphere_mesh_contact_at_transform(
                        sphere_entity,
                        mesh_entity,
                        radius,
                        &swept_transform,
                        swept_world,
                        &mesh_world,
                        &mesh_world_inv,
                        bvh,
                    ) {
                        return Some(contact);
                    }
                }
            }
        }
    }

    None
}

fn sphere_mesh_contact_at_transform(
    sphere_entity: Entity,
    mesh_entity: Entity,
    radius: f32,
    sphere_transform: &TransformComponent,
    sphere_world: Mat4,
    mesh_world: &Mat4,
    mesh_world_inv: &Mat4,
    bvh: &crate::collider_component::BVHNode,
) -> Option<Contact> {
    let sphere_aabb_mesh = sphere_aabb_in_mesh_space(radius, sphere_transform, mesh_world_inv);
    let mut candidates = Vec::new();
    bvh_collect_triangles(bvh, &sphere_aabb_mesh, &mut candidates);
    if candidates.is_empty() {
        return None;
    }

    let sphere_center_world = sphere_world.transform_point3(Vec3::ZERO);
    let radius_world = radius * max_scale(&sphere_world);
    if radius_world <= f32::EPSILON {
        return None;
    }

    let mut best_contact: Option<Contact> = None;
    let mut best_penetration = 0.0;
    for tri in candidates {
        let v0 = mesh_world.transform_point3(tri.v0);
        let v1 = mesh_world.transform_point3(tri.v1);
        let v2 = mesh_world.transform_point3(tri.v2);

        let closest = closest_point_on_triangle_world(sphere_center_world, v0, v1, v2);
        let delta = closest - sphere_center_world;
        let dist_sq = delta.length_squared();
        let radius_sq = radius_world * radius_world;
        if dist_sq > radius_sq {
            continue;
        }

        let dist = dist_sq.sqrt();
        let mut normal = if dist > f32::EPSILON {
            delta / dist
        } else {
            let n = (v1 - v0).cross(v2 - v0);
            if n.length_squared() > f32::EPSILON {
                n.normalize()
            } else {
                Vec3::Z
            }
        };

        let tri_point_world = v0;
        if (sphere_center_world - tri_point_world).dot(normal) > 0.0 {
            normal = -normal;
        }

        let contact = Contact {
            entity_a: sphere_entity,
            entity_b: mesh_entity,
            normal,
            penetration: radius_world - dist,
        };
        if contact.penetration > best_penetration {
            best_penetration = contact.penetration;
            best_contact = Some(contact);
        }
    }

    best_contact
}

fn box_mesh_contact_at_transform(
    box_entity: Entity,
    mesh_entity: Entity,
    box_collider: &ConvexCollider,
    box_transform: &TransformComponent,
    box_world: Mat4,
    mesh_world: &Mat4,
    mesh_world_inv: &Mat4,
    bvh: &crate::collider_component::BVHNode,
) -> Option<Contact> {
    let box_aabb_world = box_collider.aabb(&box_transform.to_mat4());
    let collider_in_mesh_space = *mesh_world_inv * box_world;
    let mut hits = Vec::new();
    bvh.query_collider(box_collider, &collider_in_mesh_space, &mut hits);
    if hits.is_empty() {
        return None;
    }

    let mut best_contact: Option<Contact> = None;
    let mut best_penetration = 0.0;
    for (tri, hit) in hits {
        let mut normal_world = mesh_world.transform_vector3(hit.normal);
        let normal_len = normal_world.length();
        if normal_len <= f32::EPSILON {
            continue;
        }
        let penetration = hit.penetration * normal_len;
        normal_world /= normal_len;

        let box_center_world = (box_aabb_world.min + box_aabb_world.max) * 0.5;
        let tri_point_world = mesh_world.transform_point3(tri.v0);
        if (box_center_world - tri_point_world).dot(normal_world) > 0.0 {
            normal_world = -normal_world;
        }

        let contact = Contact {
            entity_a: box_entity,
            entity_b: mesh_entity,
            normal: normal_world,
            penetration,
        };

        if contact.penetration > best_penetration {
            best_penetration = contact.penetration;
            best_contact = Some(contact);
        }
    }

    best_contact
}

fn swept_aabb(aabb: &AABB, delta: Vec3) -> AABB {
    let moved_min = aabb.min + delta;
    let moved_max = aabb.max + delta;
    AABB {
        min: aabb.min.min(moved_min),
        max: aabb.max.max(moved_max),
    }
}

fn fixed_dt() -> f32 {
    1.0 / 60.0
}

fn bvh_collect_triangles(
    node: &crate::collider_component::BVHNode,
    query: &AABB,
    out: &mut Vec<Triangle>,
) {
    if !aabb_intersects(&node.aabb, query) {
        return;
    }

    if node.left.is_none() && node.right.is_none() {
        out.extend(node.triangles.iter().cloned());
        return;
    }

    if let Some(left) = &node.left {
        bvh_collect_triangles(left, query, out);
    }
    if let Some(right) = &node.right {
        bvh_collect_triangles(right, query, out);
    }
}

fn sphere_aabb_in_mesh_space(
    radius: f32,
    sphere_transform: &TransformComponent,
    mesh_world_inv: &Mat4,
) -> AABB {
    let sphere_world = sphere_transform.to_mat4();
    let center = sphere_world.transform_point3(Vec3::ZERO);
    let scale = max_scale(&sphere_world);
    let radius = radius * scale;
    let world_aabb = AABB {
        min: center - Vec3::splat(radius),
        max: center + Vec3::splat(radius),
    };
    let corners = aabb_corners(&world_aabb);
    let mut min = mesh_world_inv.transform_point3(corners[0]);
    let mut max = min;
    for corner in corners.iter().skip(1) {
        let p = mesh_world_inv.transform_point3(*corner);
        min = min.min(p);
        max = max.max(p);
    }
    AABB { min, max }
}

fn aabb_corners(aabb: &AABB) -> [Vec3; 8] {
    let min = aabb.min;
    let max = aabb.max;
    [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(max.x, max.y, max.z),
    ]
}

fn max_scale(transform: &Mat4) -> f32 {
    let x = transform.x_axis.truncate().length();
    let y = transform.y_axis.truncate().length();
    let z = transform.z_axis.truncate().length();
    x.max(y).max(z)
}

fn closest_point_on_triangle_world(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab * v;
    }

    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac * w;
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a + ab * v + ac * w
}

fn render_body_local_aabb(
    render_body_id: crate::handles::RenderBodyHandle,
    render_resources: &RenderResourceManager,
) -> Option<AABB> {
    let render_body = render_resources
        .render_body_manager
        .get_render_body(render_body_id)?;

    let mut combined: Option<AABB> = None;
    for part in &render_body.parts {
        let mesh = render_resources.mesh_manager.get_mesh(part.mesh_id)?;
        let part_aabb = transform_aabb_with_mat4(mesh.aabb, &part.local_transform);
        combined = Some(match combined {
            Some(existing) => union_aabb(existing, part_aabb),
            None => part_aabb,
        });
    }

    combined
}

fn union_aabb(a: AABB, b: AABB) -> AABB {
    AABB {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}
