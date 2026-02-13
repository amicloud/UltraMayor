use bevy_ecs::{
    lifecycle::RemovedComponents,
    prelude::{Changed, Entity, Query, Res, ResMut},
};
use glam::{Mat4, Vec3};
use rayon::prelude::*;
use std::collections::HashMap;

use crate::{
    TransformComponent,
    collider_component::{
        BVHNode, Collider, ConvexCollider, ConvexShape, MeshCollider, Triangle,
        closest_point_on_triangle,
    },
    epa::epa,
    gjk::{GjkResult, gjk_intersect},
    mesh::AABB,
    physics_resource::{CollisionFrameData, Contact, ContactManifold, PhysicsResource},
    physics_system::delta_time,
    render_resource_manager::RenderResourceManager,
    velocity_component::VelocityComponent,
};

#[derive(Default)]
pub struct CollisionSystem {}

impl CollisionSystem {
    pub fn update_world_dynamic_tree(
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
            // --- 1. Compute world AABB ---
            let world_aabb = if let Some(mesh_collider) = mesh_collider {
                if let Some(local_aabb) =
                    render_body_local_aabb(mesh_collider.render_body_id, &render_resources)
                {
                    transform_aabb(local_aabb, transform)
                } else {
                    continue;
                }
            } else if let Some(convex_collider) = convex_collider {
                convex_collider.aabb(&transform.to_mat4())
            } else {
                continue;
            };

            // --- 2. Store world AABB ---
            phys.world_aabbs.insert(entity, world_aabb);

            // --- 3. Sync dynamic tree ---
            match phys.entity_node.get(&entity).copied() {
                Some(node_id) => {
                    // Existing object → update
                    phys.broadphase.update(node_id, world_aabb);
                }
                None => {
                    // New object → allocate leaf node
                    let node_id = phys.broadphase.allocate_leaf(entity, world_aabb);
                    phys.entity_node.insert(entity, node_id);
                }
            }
        }
    }

    pub fn cleanup_removed_entities(
        _phys: ResMut<PhysicsResource>,
        _removed: RemovedComponents<TransformComponent>,
    ) {
        todo!("Handle removed entities in collision system");
        // for entity in removed.iter() {
        //     if let Some(node_id) = phys.entity_node.remove(&entity) {
        //         phys.broadphase.remove(node_id);
        //     }

        //     phys.world_aabbs.remove(&entity);
        // }
    }

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
                if let Some(local_aabb) =
                    render_body_local_aabb(mesh_collider.render_body_id, &render_resources)
                {
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

    fn deduplicate_pairs(pairs: &mut Vec<(Entity, Entity)>) {
        for (a, b) in pairs.iter_mut() {
            if *a > *b {
                std::mem::swap(a, b);
            }
        }

        pairs.sort_unstable();
        pairs.dedup();
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
        physics_world: Res<PhysicsResource>,
        mut frame: ResMut<CollisionFrameData>,
    ) {
        frame.clear();

        for (entity, _transform, velocity, _convex, _mesh) in &moving_query {
            let base_aabb = match physics_world.world_aabbs.get(&entity) {
                Some(aabb) => *aabb,
                None => continue,
            };

            // --- Build swept AABB ---
            let swept = if let Some(velocity) = velocity {
                let delta = velocity.translational * delta_time();
                if delta.length_squared() > 0.0 {
                    swept_aabb(&base_aabb, delta)
                } else {
                    base_aabb
                }
            } else {
                base_aabb
            };

            // --- Query dynamic tree ---
            physics_world.broadphase.query(swept, |other_entity| {
                if other_entity != entity {
                    frame.candidate_pairs.push((entity, other_entity));
                }
            });
        }

        // deduplicate pairs (important!)
        Self::deduplicate_pairs(&mut frame.candidate_pairs);

        let narrowphase_results: Vec<((Entity, Entity), ContactManifold)> = frame
            .candidate_pairs
            .par_iter()
            .filter_map(|(entity_a, entity_b)| {
                let (.., transform_a, velocity_a, convex_a, mesh_a) =
                    all_query.get(*entity_a).ok()?;
                let (.., transform_b, velocity_b, convex_b, mesh_b) =
                    all_query.get(*entity_b).ok()?;

                if let (Some(convex_a), Some(convex_b)) = (convex_a, convex_b) {
                    let pair = ordered_pair(*entity_a, *entity_b);
                    let mut new_contacts = Vec::new();
                    if let Some(contact) = convex_convex_contact(
                        *entity_a,
                        &convex_a,
                        &transform_a,
                        *entity_b,
                        &convex_b,
                        &transform_b,
                    ) {
                        new_contacts.push(orient_contact_to_pair(contact, pair));
                    }
                    let merge_distance = manifold_merge_distance_pair_map(
                        &physics_world.world_aabbs,
                        pair.0,
                        pair.1,
                    );
                    let merged = merge_contact_manifold(
                        frame.manifolds.get(&pair),
                        &new_contacts,
                        merge_distance,
                        0.95,
                        4,
                    );
                    if merged.contacts.is_empty() {
                        return None;
                    }
                    return Some((pair, merged));
                }

                if let (Some(convex_a), Some(mesh_b)) = (convex_a, mesh_b) {
                    let pair = (*entity_b, *entity_a); // (mesh, convex)
                    let mesh_contacts = convex_mesh_contact(
                        *entity_a,
                        &convex_a,
                        &transform_a,
                        velocity_a,
                        *entity_b,
                        &mesh_b,
                        &transform_b,
                        &render_resources,
                    );
                    let merge_distance = manifold_merge_distance_pair_map(
                        &physics_world.world_aabbs,
                        *entity_a,
                        *entity_b,
                    );
                    let merged = merge_contact_manifold(
                        frame.manifolds.get(&pair),
                        &mesh_contacts,
                        merge_distance,
                        0.9,
                        8,
                    );
                    if merged.contacts.is_empty() {
                        return None;
                    }
                    return Some((pair, merged));
                }

                if let (Some(mesh_a), Some(convex_b)) = (mesh_a, convex_b) {
                    let pair = (*entity_a, *entity_b);
                    let mesh_contacts = convex_mesh_contact(
                        *entity_b,
                        &convex_b,
                        &transform_b,
                        velocity_b,
                        *entity_a,
                        &mesh_a,
                        &transform_a,
                        &render_resources,
                    );
                    let merge_distance = manifold_merge_distance_pair_map(
                        &physics_world.world_aabbs,
                        *entity_b,
                        *entity_b,
                    );
                    let merged = merge_contact_manifold(
                        frame.manifolds.get(&pair),
                        &mesh_contacts,
                        merge_distance,
                        0.9,
                        8,
                    );
                    if merged.contacts.is_empty() {
                        return None;
                    }
                    return Some((pair, merged));
                }

                None
            })
            .collect();

        for (pair, manifold) in narrowphase_results {
            frame
                .manifolds
                .entry(pair)
                .and_modify(|existing| {
                    if manifold.contacts.len() > existing.contacts.len() {
                        *existing = manifold.clone();
                    } else if manifold.contacts.len() == existing.contacts.len()
                        && manifold_max_penetration(&manifold) > manifold_max_penetration(existing)
                    {
                        *existing = manifold.clone();
                    }
                })
                .or_insert(manifold.clone());
        }

        let mut merged_contacts = Vec::new();
        for manifold in frame.manifolds.values() {
            merged_contacts.extend_from_slice(&manifold.contacts);
        }
        frame.contacts.extend(merged_contacts);
    }
}

fn manifold_merge_distance_pair_map(
    world_aabbs: &HashMap<Entity, AABB>,
    a: Entity,
    b: Entity,
) -> f32 {
    let extent_a = world_aabbs
        .get(&a)
        .map(|aabb| (aabb.max - aabb.min).length())
        .unwrap_or(0.0);
    let extent_b = world_aabbs
        .get(&b)
        .map(|aabb| (aabb.max - aabb.min).length())
        .unwrap_or(0.0);
    let extent = extent_a.max(extent_b).max(0.01);
    let merge_strenth = 0.5;
    extent * merge_strenth
}

fn ordered_pair(a: Entity, b: Entity) -> (Entity, Entity) {
    if a.to_bits() <= b.to_bits() {
        (a, b)
    } else {
        (b, a)
    }
}

fn orient_contact_to_pair(mut contact: Contact, pair: (Entity, Entity)) -> Contact {
    if contact.entity_a == pair.0 {
        return contact;
    }

    contact.normal = -contact.normal;
    let old_a = contact.entity_a;
    contact.entity_a = contact.entity_b;
    contact.entity_b = old_a;
    contact
}

fn manifold_max_penetration(manifold: &ContactManifold) -> f32 {
    manifold
        .contacts
        .iter()
        .map(|c| c.penetration)
        .fold(0.0_f32, f32::max)
}

fn merge_contact_manifold(
    previous: Option<&ContactManifold>,
    new_contacts: &[Contact],
    merge_distance: f32,
    normal_cos_threshold: f32,
    max_contacts: usize,
) -> ContactManifold {
    if new_contacts.is_empty() {
        return ContactManifold {
            contacts: Vec::new(),
            normal: Vec3::ZERO,
        };
    }

    let merge_distance_sq = merge_distance * merge_distance;
    let mut merged: Vec<Contact> = Vec::new();
    let mut used_new = vec![false; new_contacts.len()];

    if let Some(prev) = previous {
        for prev_contact in &prev.contacts {
            let mut best_idx: Option<usize> = None;
            let mut best_dist_sq = f32::INFINITY;

            for (i, new_contact) in new_contacts.iter().enumerate() {
                if used_new[i] {
                    continue;
                }
                if prev_contact.normal.dot(new_contact.normal) < normal_cos_threshold {
                    continue;
                }
                let dist_sq =
                    (prev_contact.contact_point - new_contact.contact_point).length_squared();
                if dist_sq <= merge_distance_sq && dist_sq < best_dist_sq {
                    best_dist_sq = dist_sq;
                    best_idx = Some(i);
                }
            }

            if let Some(i) = best_idx {
                merged.push(new_contacts[i]);
                used_new[i] = true;
            }
        }
    }

    for (i, contact) in new_contacts.iter().enumerate() {
        if used_new[i] {
            continue;
        }

        let mut merged_into_existing = false;
        for existing in &mut merged {
            if existing.normal.dot(contact.normal) < normal_cos_threshold {
                continue;
            }
            let dist_sq = (existing.contact_point - contact.contact_point).length_squared();
            if dist_sq <= merge_distance_sq {
                if contact.penetration > existing.penetration {
                    *existing = *contact;
                }
                merged_into_existing = true;
                break;
            }
        }

        if !merged_into_existing {
            merged.push(*contact);
        }
    }

    if merged.len() > max_contacts {
        let mut scored: Vec<(Contact, f32)> = merged
            .into_iter()
            .map(|contact| {
                let distance_to_prev = previous
                    .map(|prev| {
                        prev.contacts
                            .iter()
                            .map(|p| (p.contact_point - contact.contact_point).length())
                            .fold(f32::INFINITY, f32::min)
                    })
                    .unwrap_or(0.0);
                (contact, distance_to_prev)
            })
            .collect();

        scored.sort_by(|a, b| {
            a.1.total_cmp(&b.1)
                .then_with(|| b.0.penetration.total_cmp(&a.0.penetration))
        });

        scored.truncate(max_contacts);
        merged = scored.into_iter().map(|(contact, _)| contact).collect();
    }

    let normal = merged
        .iter()
        .fold(Vec3::ZERO, |acc, contact| {
            acc + contact.normal * contact.penetration
        })
        .try_normalize()
        .unwrap_or(Vec3::ZERO);

    ContactManifold {
        contacts: merged,
        normal,
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

fn sphere_sphere_contact(
    entity_a: Entity,
    collider_a: &ConvexCollider,
    transform_a: &TransformComponent,
    entity_b: Entity,
    collider_b: &ConvexCollider,
    transform_b: &TransformComponent,
) -> Option<Contact> {
    let radius_a = collider_a.as_sphere_radius()?;
    let radius_b = collider_b.as_sphere_radius()?;

    let center_a = transform_a.position;
    let center_b = transform_b.position;
    let ab = center_b - center_a;
    let distance_sq = ab.length_squared();
    let radius_sum = radius_a + radius_b;
    let radius_sum_sq = radius_sum * radius_sum;

    if distance_sq >= radius_sum_sq {
        return None;
    }

    let distance = distance_sq.sqrt();
    let penetration = radius_sum - distance;

    let normal = if distance > f32::EPSILON {
        ab / distance
    } else {
        Vec3::X
    };

    let contact_point = center_a + normal * radius_a;

    Some(Contact {
        entity_a,
        entity_b,
        normal,
        penetration,
        contact_point,
    })
}

fn cuboid_cuboid_contact(
    entity_a: Entity,
    collider_a: &ConvexCollider,
    transform_a: &TransformComponent,
    entity_b: Entity,
    collider_b: &ConvexCollider,
    transform_b: &TransformComponent,
) -> Option<Contact> {
    // SAT is generally more efficient for box-box collisions than GJK+EPA, so we can use it as a fast path here.
    let (a_len, a_wid, a_hei) = collider_a.as_cuboid()?;
    let (b_len, b_wid, b_hei) = collider_b.as_cuboid()?;

    let a_center = transform_a.position;
    let b_center = transform_b.position;

    let a_axes = [
        transform_a.rotation * Vec3::X,
        transform_a.rotation * Vec3::Y,
        transform_a.rotation * Vec3::Z,
    ];
    let b_axes = [
        transform_b.rotation * Vec3::X,
        transform_b.rotation * Vec3::Y,
        transform_b.rotation * Vec3::Z,
    ];

    let a_extents = Vec3::new(a_len * 0.5, a_wid * 0.5, a_hei * 0.5);
    let b_extents = Vec3::new(b_len * 0.5, b_wid * 0.5, b_hei * 0.5);

    let t = b_center - a_center;

    let mut r = [[0.0_f32; 3]; 3];
    let mut abs_r = [[0.0_f32; 3]; 3];
    let eps = 1e-6_f32;

    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a_axes[i].dot(b_axes[j]);
            abs_r[i][j] = r[i][j].abs() + eps;
        }
    }

    let t_a = Vec3::new(t.dot(a_axes[0]), t.dot(a_axes[1]), t.dot(a_axes[2]));

    let mut min_penetration = f32::INFINITY;
    let mut best_axis = Vec3::ZERO;

    let mut test_axis = |axis: Vec3, separation: f32| -> bool {
        let penetration = -separation;
        if penetration < min_penetration {
            min_penetration = penetration;
            best_axis = axis;
        }
        true
    };

    // Test axes L = A0, A1, A2
    for i in 0..3 {
        let ra = a_extents[i];
        let rb = b_extents.x * abs_r[i][0] + b_extents.y * abs_r[i][1] + b_extents.z * abs_r[i][2];
        let separation = t_a[i].abs() - (ra + rb);
        if separation > 0.0 {
            return None;
        }
        let mut axis = a_axes[i];
        if t_a[i] < 0.0 {
            axis = -axis;
        }
        test_axis(axis, separation);
    }

    // Test axes L = B0, B1, B2
    for j in 0..3 {
        let ra = a_extents.x * abs_r[0][j] + a_extents.y * abs_r[1][j] + a_extents.z * abs_r[2][j];
        let rb = b_extents[j];
        let t_b = t_a.x * r[0][j] + t_a.y * r[1][j] + t_a.z * r[2][j];
        let separation = t_b.abs() - (ra + rb);
        if separation > 0.0 {
            return None;
        }
        let mut axis = b_axes[j];
        if t_b < 0.0 {
            axis = -axis;
        }
        test_axis(axis, separation);
    }

    // Test axis L = Ai x Bj
    for i in 0..3 {
        for j in 0..3 {
            let axis = a_axes[i].cross(b_axes[j]);
            let axis_len_sq = axis.length_squared();
            if axis_len_sq < eps {
                continue;
            }

            let ra = a_extents[(i + 1) % 3] * abs_r[(i + 2) % 3][j]
                + a_extents[(i + 2) % 3] * abs_r[(i + 1) % 3][j];
            let rb = b_extents[(j + 1) % 3] * abs_r[i][(j + 2) % 3]
                + b_extents[(j + 2) % 3] * abs_r[i][(j + 1) % 3];
            let t_term =
                t_a[(i + 2) % 3] * r[(i + 1) % 3][j] - t_a[(i + 1) % 3] * r[(i + 2) % 3][j];
            let separation = t_term.abs() - (ra + rb);
            if separation > 0.0 {
                return None;
            }

            let mut axis_world = axis / axis_len_sq.sqrt();
            if axis_world.dot(t) < 0.0 {
                axis_world = -axis_world;
            }
            test_axis(axis_world, separation);
        }
    }

    if best_axis.length_squared() <= eps {
        return None;
    }

    let normal = best_axis.normalize();
    let a_support = collider_a.support(transform_a.to_mat4(), normal);
    let b_support = collider_b.support(transform_b.to_mat4(), -normal);
    let contact_point = (a_support + b_support) * 0.5;

    Some(Contact {
        entity_a,
        entity_b,
        normal,
        penetration: min_penetration,
        contact_point,
    })
}

fn convex_convex_contact(
    entity_a: Entity,
    collider_a: &ConvexCollider,
    transform_a: &TransformComponent,
    entity_b: Entity,
    collider_b: &ConvexCollider,
    transform_b: &TransformComponent,
) -> Option<Contact> {
    // Fast path for sphere-sphere collisions
    match (collider_a.shape, collider_b.shape) {
        (ConvexShape::Sphere { .. }, ConvexShape::Sphere { .. }) => {
            return sphere_sphere_contact(
                entity_a,
                collider_a,
                transform_a,
                entity_b,
                collider_b,
                transform_b,
            );
        }
        (ConvexShape::Cuboid { .. }, ConvexShape::Cuboid { .. }) => {
            return cuboid_cuboid_contact(
                entity_a,
                collider_a,
                transform_a,
                entity_b,
                collider_b,
                transform_b,
            );
        }
        _ => {}
    }

    let contact = gjk_epa_contact_generator(collider_a, transform_a, collider_b, transform_b)?;
    Some(Contact {
        entity_a,
        entity_b,
        normal: contact.normal,
        penetration: contact.penetration_depth,
        contact_point: contact.contact_point,
    })
}

struct GjkEpaResult {
    normal: Vec3,
    penetration_depth: f32,
    contact_point: Vec3,
}

fn gjk_epa_contact_generator(
    collider_a: &ConvexCollider,
    transform_a: &TransformComponent,
    collider_b: &ConvexCollider,
    transform_b: &TransformComponent,
) -> Option<GjkEpaResult> {
    let a_world = transform_a.to_mat4();
    let b_world = transform_b.to_mat4();

    let result = gjk_intersect(collider_a, a_world, collider_b, b_world);
    let simplex = match result {
        GjkResult::Intersection(hit) => hit.simplex,
        GjkResult::NoIntersection => return None,
    };

    let epa_result = epa(collider_a, a_world, collider_b, b_world, &simplex)?;

    let mut normal = epa_result.normal;
    let a_center = a_world.transform_point3(Vec3::ZERO);
    let b_center = b_world.transform_point3(Vec3::ZERO);
    let ab = b_center - a_center;
    if ab.length_squared() > f32::EPSILON && normal.dot(ab) < 0.0 {
        normal = -normal;
    }

    let a_support = collider_a.support(a_world, normal);
    let b_support = collider_b.support(b_world, -normal);
    let contact_point = (a_support + b_support) * 0.5;
    Some(GjkEpaResult {
        normal,
        penetration_depth: epa_result.penetration_depth,
        contact_point,
    })
}

fn convex_mesh_contact(
    convex_entity: Entity,
    convex_collider: &ConvexCollider,
    convex_transform: &TransformComponent,
    convex_velocity: Option<&VelocityComponent>,
    mesh_entity: Entity,
    mesh_collider: &MeshCollider,
    mesh_transform: &TransformComponent,
    render_resources: &RenderResourceManager,
) -> Vec<Contact> {
    let Some(render_body) = render_resources
        .render_body_manager
        .get_render_body(mesh_collider.render_body_id)
    else {
        return Vec::new();
    };

    let convex_world = convex_transform.to_mat4();
    let mesh_entity_world = mesh_transform.to_mat4();

    let convex_aabb_world = convex_collider.aabb(&convex_world);
    let mut candidates: Vec<ContactCandidate> = Vec::new();

    for part in &render_body.parts {
        let Some(mesh) = render_resources.mesh_manager.get_mesh(part.mesh_id) else {
            continue;
        };
        let Some(bvh) = mesh.bvh.as_ref() else {
            continue;
        };

        let mesh_world = mesh_entity_world * part.local_transform;
        let Some(mesh_world_inv) = mesh_world.try_inverse() else {
            continue;
        };

        candidates.extend(convex_mesh_contact_at_transform(
            convex_collider,
            convex_world,
            &mesh_world,
            &mesh_world_inv,
            bvh,
        ));

        if let Some(velocity) = convex_velocity {
            let delta = velocity.translational * delta_time();
            let distance = delta.length();
            if distance > 0.0 {
                let step = (convex_aabb_world.max - convex_aabb_world.min)
                    .abs()
                    .min_element()
                    .max(0.01)
                    * 0.5;
                let steps = ((distance / step).ceil() as i32).clamp(1, 20);

                for i in 1..=steps {
                    let t = i as f32 / steps as f32;
                    let swept_position = convex_transform.position + delta * t;
                    let swept_transform = TransformComponent {
                        position: swept_position,
                        rotation: convex_transform.rotation,
                        scale: convex_transform.scale,
                    };
                    let swept_world = swept_transform.to_mat4();
                    candidates.extend(convex_mesh_contact_at_transform(
                        convex_collider,
                        swept_world,
                        &mesh_world,
                        &mesh_world_inv,
                        bvh,
                    ));
                }
            }
        }
    }

    reduce_contact_candidates(mesh_entity, convex_entity, candidates, convex_aabb_world)
}

fn convex_mesh_contact_at_transform(
    convex_collider: &ConvexCollider,
    convex_world: Mat4,
    mesh_world: &Mat4,
    mesh_world_inv: &Mat4,
    bvh: &BVHNode,
) -> Vec<ContactCandidate> {
    let collider_in_mesh_space = *mesh_world_inv * convex_world;
    let _convex_center_mesh = collider_in_mesh_space.transform_point3(Vec3::ZERO);
    let convex_aabb_mesh = convex_collider.aabb(&collider_in_mesh_space);

    let mut triangles = Vec::new();
    collect_triangles_in_aabb(bvh, &convex_aabb_mesh, &mut triangles);
    if triangles.is_empty() {
        return Vec::new();
    }

    let convex_center_world = convex_world.transform_point3(Vec3::ZERO);

    let mut candidates = Vec::new();
    for tri in triangles {
        let tri_world = Triangle {
            v0: mesh_world.transform_point3(tri.v0),
            v1: mesh_world.transform_point3(tri.v1),
            v2: mesh_world.transform_point3(tri.v2),
        };

        let closest_world = closest_point_on_triangle(convex_center_world, &tri_world);

        let face_normal_world = {
            let n = (tri_world.v1 - tri_world.v0).cross(tri_world.v2 - tri_world.v0);
            if n.length_squared() > f32::EPSILON {
                Some(n.normalize())
            } else {
                None
            }
        };

        // The easy sphere case
        if let ConvexShape::Sphere { radius } = convex_collider.shape {
            let delta = convex_center_world - closest_world;
            let dist2 = delta.length_squared();

            if dist2 <= radius * radius {
                let dist = dist2.sqrt();
                let normal = if dist > f32::EPSILON {
                    delta / dist
                } else {
                    // fallback: triangle normal or arbitrary up
                    face_normal_world.unwrap_or(Vec3::Z)
                };

                let penetration = radius - dist;

                candidates.push(ContactCandidate {
                    point: closest_world,
                    normal,
                    penetration,
                });
            }
            continue;
        }

        // Use GJK/EPA to get the contact all other convex colliders for now
        let triangle_collider =
            ConvexCollider::triangle(tri.v0, tri.v1, tri.v2, convex_collider.layer);

        let result = gjk_intersect(
            &triangle_collider,
            *mesh_world,
            convex_collider,
            convex_world,
        );

        let simplex = match result {
            GjkResult::Intersection(hit) => hit.simplex,
            GjkResult::NoIntersection => continue,
        };

        let Some(epa_result) = epa(
            &triangle_collider,
            *mesh_world,
            convex_collider,
            convex_world,
            &simplex,
        ) else {
            continue;
        };

        let mut normal = epa_result.normal;
        let tri_normal = face_normal_world.unwrap();
        if tri_normal.dot(normal) > 0.95 {
            normal = tri_normal;
        }
        let tri_center_local = (tri.v0 + tri.v1 + tri.v2) / 3.0;
        let tri_center_world = mesh_world.transform_point3(tri_center_local);
        let convex_center_world = convex_world.transform_point3(Vec3::ZERO);
        let ab = convex_center_world - tri_center_world;
        if ab.length_squared() > f32::EPSILON && normal.dot(ab) < 0.0 {
            normal = -normal;
        }

        let support = convex_collider.support(convex_world, -normal);
        // project support point onto triangle plane
        let plane_point = tri_world.v0;
        let plane_normal = normal;
        let d = (support - plane_point).dot(plane_normal);
        let contact_point = support - plane_normal * d;

        candidates.push(ContactCandidate {
            point: contact_point,
            normal,
            penetration: epa_result.penetration_depth,
        });
    }
    candidates
}

#[derive(Debug, Clone, Copy)]
struct ContactCandidate {
    point: Vec3,
    normal: Vec3,
    penetration: f32,
}

fn reduce_contact_candidates(
    mesh_entity: Entity,
    convex_entity: Entity,
    mut candidates: Vec<ContactCandidate>,
    convex_aabb_world: AABB,
) -> Vec<Contact> {
    // Filter out degenerate contacts
    candidates.retain(|c| c.penetration > 0.0 && c.normal.length_squared() > f32::EPSILON);
    if candidates.is_empty() {
        return Vec::new();
    }

    // Sort by penetration depth (descending), then stable tie-breakers
    candidates.sort_by(|a, b| {
        b.penetration
            .total_cmp(&a.penetration)
            .then_with(|| a.point.x.total_cmp(&b.point.x))
            .then_with(|| a.point.y.total_cmp(&b.point.y))
            .then_with(|| a.point.z.total_cmp(&b.point.z))
            .then_with(|| a.normal.x.total_cmp(&b.normal.x))
            .then_with(|| a.normal.y.total_cmp(&b.normal.y))
            .then_with(|| a.normal.z.total_cmp(&b.normal.z))
    });

    // Compute cluster distance
    let extent = convex_aabb_world.max - convex_aabb_world.min;
    let cluster_distance = extent.length().max(0.01) * 0.1;
    let normal_epsilon = 0.01;

    // Select contacts
    let mut selected: Vec<ContactCandidate> = Vec::new();
    for candidate in candidates {
        let mut skip = false;
        for c in &selected {
            let close = (c.point - candidate.point).length() < cluster_distance;
            let same_normal = c.normal.dot(candidate.normal) > 1.0 - normal_epsilon;

            // Only skip if BOTH too close AND normals are almost identical
            if close && same_normal {
                skip = true;
                break;
            }
        }

        if skip {
            continue;
        }

        selected.push(candidate);
        if selected.len() >= 4 {
            break;
        }
    }

    // Convert to Contact
    selected
        .into_iter()
        .map(|candidate| Contact {
            entity_a: mesh_entity,
            entity_b: convex_entity,
            normal: candidate.normal.normalize(),
            penetration: candidate.penetration,
            contact_point: candidate.point,
        })
        .collect()
}

fn collect_triangles_in_aabb(bvh: &BVHNode, target: &AABB, out: &mut Vec<Triangle>) {
    if !aabb_intersects(&bvh.aabb, target) {
        return;
    }

    if bvh.left.is_none() && bvh.right.is_none() {
        for tri in &bvh.triangles {
            let tri_aabb = triangle_aabb(tri);
            if aabb_intersects(&tri_aabb, target) {
                out.push(tri.clone());
            }
        }
        return;
    }

    if let Some(left) = &bvh.left {
        collect_triangles_in_aabb(left, target, out);
    }
    if let Some(right) = &bvh.right {
        collect_triangles_in_aabb(right, target, out);
    }
}

fn triangle_aabb(tri: &Triangle) -> AABB {
    let min = tri.v0.min(tri.v1).min(tri.v2);
    let max = tri.v0.max(tri.v1).max(tri.v2);
    AABB { min, max }
}

fn swept_aabb(aabb: &AABB, delta: Vec3) -> AABB {
    let moved_min = aabb.min + delta;
    let moved_max = aabb.max + delta;
    AABB {
        min: aabb.min.min(moved_min),
        max: aabb.max.max(moved_max),
    }
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

#[cfg(test)]
mod tests {
    use crate::collider_component::CollisionLayer;

    use super::*;
    use approx::assert_relative_eq;
    use glam::{Mat4, Quat, Vec3};

    fn make_transform(position: Vec3, rotation: Quat, scale: Vec3) -> TransformComponent {
        TransformComponent {
            position,
            rotation,
            scale,
        }
    }

    fn make_triangle() -> Triangle {
        Triangle {
            v0: Vec3::new(0.0, 0.0, 0.0),
            v1: Vec3::new(1.0, 0.0, 0.0),
            v2: Vec3::new(0.0, 1.0, 0.0),
        }
    }

    #[test]
    fn convex_mesh_contact_at_transform_hits_triangle() {
        let tri = make_triangle();
        let bvh = BVHNode::build(vec![tri], 4);

        let convex_collider = ConvexCollider::sphere(1.0, CollisionLayer::Default);
        let convex_transform = TransformComponent {
            position: Vec3::new(1.5, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        };
        let convex_world = convex_transform.to_mat4();

        let mesh_world = Mat4::IDENTITY;
        let mesh_world_inv = mesh_world.inverse();

        let contacts = convex_mesh_contact_at_transform(
            &convex_collider,
            convex_world,
            &mesh_world,
            &mesh_world_inv,
            &bvh,
        );

        assert_eq!(contacts.len(), 1);
        let contact = contacts[0];
        assert_relative_eq!(contact.normal.x, 1.0, epsilon = 1e-4);
        assert_relative_eq!(contact.normal.y, 0.0, epsilon = 1e-4);
        assert_relative_eq!(contact.normal.z, 0.0, epsilon = 1e-4);
        assert_relative_eq!(contact.penetration, 0.5, epsilon = 1e-4);
    }

    #[test]
    fn convex_mesh_contact_at_transform_flips_normal_toward_convex() {
        let tri = make_triangle();
        let bvh = BVHNode::build(vec![tri], 4);

        let convex_collider = ConvexCollider::sphere(1.0, CollisionLayer::Default);
        let convex_transform = TransformComponent {
            position: Vec3::new(0.2, 0.2, -0.5),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        };
        let convex_world = convex_transform.to_mat4();

        let mesh_world = Mat4::from_scale(Vec3::new(2.0, 1.0, 0.5));
        let mesh_world_inv = mesh_world.inverse();

        let contacts = convex_mesh_contact_at_transform(
            &convex_collider,
            convex_world,
            &mesh_world,
            &mesh_world_inv,
            &bvh,
        );

        assert_eq!(contacts.len(), 1);
        let contact = contacts[0];
        assert!(contact.normal.z < 0.0);
        assert_relative_eq!(contact.normal.length(), 1.0, epsilon = 1e-4);
        assert!(contact.penetration > 0.0);
    }

    #[test]
    fn reduce_contact_candidates_caps_to_four() {
        let mesh_entity = Entity::from_bits(1);
        let convex_entity = Entity::from_bits(2);
        let convex_aabb_world = AABB {
            min: Vec3::splat(-1.0),
            max: Vec3::splat(1.0),
        };

        let candidates = vec![
            ContactCandidate {
                point: Vec3::new(0.0, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 6.0,
            },
            ContactCandidate {
                point: Vec3::new(10.0, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 5.0,
            },
            ContactCandidate {
                point: Vec3::new(20.0, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 4.0,
            },
            ContactCandidate {
                point: Vec3::new(30.0, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 3.0,
            },
            ContactCandidate {
                point: Vec3::new(40.0, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 2.0,
            },
            ContactCandidate {
                point: Vec3::new(50.0, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 1.0,
            },
        ];

        let contacts =
            reduce_contact_candidates(mesh_entity, convex_entity, candidates, convex_aabb_world);

        assert_eq!(contacts.len(), 4);
        assert_relative_eq!(contacts[0].penetration, 6.0, epsilon = 1e-6);
        assert_relative_eq!(contacts[1].penetration, 5.0, epsilon = 1e-6);
        assert_relative_eq!(contacts[2].penetration, 4.0, epsilon = 1e-6);
        assert_relative_eq!(contacts[3].penetration, 3.0, epsilon = 1e-6);
    }

    #[test]
    fn reduce_contact_candidates_clusters() {
        let mesh_entity = Entity::from_bits(1);
        let convex_entity = Entity::from_bits(2);
        let convex_aabb_world = AABB {
            min: Vec3::splat(-1.0),
            max: Vec3::splat(1.0),
        };

        // Points 0.0, 0.05, 0.1 are very close → should cluster to 1 contact
        // Points 1.0, 1.05 are far enough → separate cluster
        let candidates = vec![
            ContactCandidate {
                point: Vec3::new(0.0, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 6.0,
            },
            ContactCandidate {
                point: Vec3::new(0.05, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 5.5,
            },
            ContactCandidate {
                point: Vec3::new(0.1, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 5.0,
            },
            ContactCandidate {
                point: Vec3::new(1.0, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 4.0,
            },
            ContactCandidate {
                point: Vec3::new(1.05, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 3.5,
            },
            ContactCandidate {
                point: Vec3::new(2.0, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 3.0,
            },
            ContactCandidate {
                point: Vec3::new(3.0, 0.0, 0.0),
                normal: Vec3::Z,
                penetration: 2.0,
            },
        ];

        let contacts =
            reduce_contact_candidates(mesh_entity, convex_entity, candidates, convex_aabb_world);

        // Only one from each cluster should be chosen, respecting the max of 4 contacts
        assert_eq!(contacts.len(), 4);

        // First cluster picks the deepest penetration
        assert_relative_eq!(contacts[0].penetration, 6.0, epsilon = 1e-6);

        // Second cluster picks next deepest (1.0 point cluster)
        assert_relative_eq!(contacts[1].penetration, 4.0, epsilon = 1e-6);

        // Third cluster (2.0 point)
        assert_relative_eq!(contacts[2].penetration, 3.0, epsilon = 1e-6);

        // Fourth cluster (3.0 point)
        assert_relative_eq!(contacts[3].penetration, 2.0, epsilon = 1e-6);
    }

    #[test]
    fn cuboid_cuboid_contact_scaling_does_not_change_overlap_state() {
        let entity_a = Entity::from_bits(10);
        let entity_b = Entity::from_bits(11);

        let collider_a = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let collider_b = collider_a;

        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let mut transform_b = make_transform(Vec3::new(2.1, 0.0, 0.0), Quat::IDENTITY, Vec3::ONE);

        let initial = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );
        assert!(initial.is_none());

        transform_b.scale = Vec3::new(1.2, 1.0, 1.0);
        let scaled = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );

        assert_eq!(initial.is_some(), scaled.is_some());
    }

    #[test]
    fn cuboid_cuboid_contact_rotation_and_scaling_match_overlap_state() {
        let entity_a = Entity::from_bits(20);
        let entity_b = Entity::from_bits(21);

        let collider_a = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let collider_b = collider_a;

        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let mut transform_b = make_transform(Vec3::new(2.1, 0.0, 0.0), Quat::IDENTITY, Vec3::ONE);

        let initial = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );
        assert!(initial.is_none());

        transform_b.rotation = Quat::from_rotation_z(45.0_f32.to_radians());
        let rotated = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );

        transform_b.scale = Vec3::splat(1.4);
        let rotated_scaled = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );

        assert_eq!(rotated.is_some(), rotated_scaled.is_some());
    }

    #[test]
    fn cuboid_cuboid_contact_rotated_without_scaling_intersect() {
        let entity_a = Entity::from_bits(22);
        let entity_b = Entity::from_bits(23);

        let collider_a = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let collider_b = collider_a;

        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let mut transform_b = make_transform(Vec3::new(2.1, 0.0, 0.0), Quat::IDENTITY, Vec3::ONE);

        let initial = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );
        assert!(initial.is_none());

        transform_b.rotation = Quat::from_rotation_z(45.0_f32.to_radians());
        let rotated = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        )
        .expect("Expected overlap after rotation");

        assert!(rotated.penetration > 0.0);
        assert_relative_eq!(rotated.normal.length(), 1.0, epsilon = 1e-4);
    }

    #[test]
    fn sphere_sphere_contact_scaling_does_not_change_overlap_state() {
        let entity_a = Entity::from_bits(30);
        let entity_b = Entity::from_bits(31);

        let radius = 1.0;
        let collider_a = ConvexCollider::sphere(radius, CollisionLayer::Default);
        let collider_b = collider_a;

        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let mut transform_b = make_transform(Vec3::new(2.1, 0.0, 0.0), Quat::IDENTITY, Vec3::ONE);

        let initial = sphere_sphere_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );
        assert!(initial.is_none());

        transform_b.scale = Vec3::splat(1.2);
        let scaled = sphere_sphere_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );

        assert_eq!(initial.is_some(), scaled.is_some());
    }

    #[test]
    fn sphere_sphere_contact_rotation_and_scaling_match_overlap_state() {
        let entity_a = Entity::from_bits(32);
        let entity_b = Entity::from_bits(33);

        let radius = 1.0;
        let collider_a = ConvexCollider::sphere(radius, CollisionLayer::Default);
        let collider_b = collider_a;

        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let mut transform_b = make_transform(Vec3::new(2.1, 0.0, 0.0), Quat::IDENTITY, Vec3::ONE);

        let initial = sphere_sphere_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );
        assert!(initial.is_none());

        transform_b.rotation = Quat::from_rotation_y(30.0_f32.to_radians());
        let rotated = sphere_sphere_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );

        transform_b.scale = Vec3::splat(1.3);
        let rotated_scaled = sphere_sphere_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );

        assert_eq!(rotated.is_some(), rotated_scaled.is_some());
    }

    #[test]
    fn cuboid_cuboid_contact_unequal_lengths_intersect() {
        let entity_a = Entity::from_bits(40);
        let entity_b = Entity::from_bits(41);

        let collider_a = ConvexCollider::cuboid(Vec3::new(1.0, 1.0, 1.0), CollisionLayer::Default);
        let collider_b = ConvexCollider::cuboid(Vec3::new(2.0, 1.0, 1.0), CollisionLayer::Default);

        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let transform_b = make_transform(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY, Vec3::ONE);

        let contact = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        )
        .expect("Expected overlap for unequal cuboids");

        assert_relative_eq!(contact.penetration, 0.5, epsilon = 1e-4);
    }

    #[test]
    fn cuboid_cuboid_contact_unequal_lengths_no_intersection() {
        let entity_a = Entity::from_bits(42);
        let entity_b = Entity::from_bits(43);

        let collider_a = ConvexCollider::cuboid(Vec3::new(1.0, 1.0, 1.0), CollisionLayer::Default);
        let collider_b = ConvexCollider::cuboid(Vec3::new(0.5, 1.0, 1.0), CollisionLayer::Default);

        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let transform_b = make_transform(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY, Vec3::ONE);

        let contact = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );

        assert!(contact.is_none());
    }
}
