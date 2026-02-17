use bevy_ecs::{
    lifecycle::RemovedComponents,
    prelude::{Changed, Entity, Query, Res, ResMut},
};
use glam::{Mat4, Vec3};
use rayon::prelude::*;
use std::{collections::HashMap, time::Duration};

use crate::{
    TransformComponent,
    components::collider_component::{
        BVHNode, Collider, ConvexCollider, ConvexShape, MeshCollider, Triangle,
        closest_point_on_triangle,
    },
    components::velocity_component::VelocityComponent,
    mesh::Aabb,
    physics,
    render::render_resource_manager::RenderResourceManager,
    time_resource::TimeResource,
};

use physics::{
    epa::epa,
    gjk::{GjkResult, gjk_intersect},
    physics_resource::{CollisionFrameData, Contact, ContactManifold, PhysicsResource},
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

            phys.world_aabbs.insert(entity, world_aabb);

            // Sync dynamic tree
            match phys.entity_node.get(&entity).copied() {
                Some(node_id) => {
                    // Existing object -> update
                    phys.broadphase.update(node_id, world_aabb);
                }
                None => {
                    // New object -> allocate leaf node
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
            if let Some(mesh_collider) = mesh_collider
                && let Some(local_aabb) =
                    render_body_local_aabb(mesh_collider.render_body_id, &render_resources)
            {
                let world_aabb = transform_aabb(local_aabb, transform);
                phys.world_aabbs.insert(entity, world_aabb);
                continue;
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
        time: Res<TimeResource>,
    ) {
        let old_manifolds = std::mem::take(&mut frame.manifolds);
        let delta_t = time.simulation_fixed_dt();
        frame.clear();

        for (entity, _transform, velocity, _convex, _mesh) in &moving_query {
            let base_aabb = match physics_world.world_aabbs.get(&entity) {
                Some(aabb) => *aabb,
                None => continue,
            };

            // --- Build swept AABB ---
            let swept = if let Some(velocity) = velocity {
                let delta = velocity.translational * delta_t.as_secs_f32();
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

                let pair = ordered_pair(*entity_a, *entity_b);
                let previous_manifold = old_manifolds.get(&pair);

                if let (Some(convex_a), Some(convex_b)) = (convex_a, convex_b) {
                    return convex_convex_pair_manifold(
                        *entity_a,
                        convex_a,
                        transform_a,
                        velocity_a,
                        *entity_b,
                        convex_b,
                        transform_b,
                        velocity_b,
                        &physics_world.world_aabbs,
                        previous_manifold,
                        delta_t,
                    )
                    .map(|merged| (pair, merged));
                }

                if let (Some(convex_a), Some(mesh_b)) = (convex_a, mesh_b) {
                    return convex_mesh_pair_manifold(
                        *entity_a,
                        convex_a,
                        transform_a,
                        velocity_a,
                        *entity_b,
                        mesh_b,
                        transform_b,
                        &render_resources,
                        &physics_world.world_aabbs,
                        previous_manifold,
                        delta_t,
                    )
                    .map(|merged| (pair, merged));
                }

                if let (Some(mesh_a), Some(convex_b)) = (mesh_a, convex_b) {
                    return convex_mesh_pair_manifold(
                        *entity_b,
                        convex_b,
                        transform_b,
                        velocity_b,
                        *entity_a,
                        mesh_a,
                        transform_a,
                        &render_resources,
                        &physics_world.world_aabbs,
                        previous_manifold,
                        delta_t,
                    )
                    .map(|merged| (pair, merged));
                }

                None
            })
            .collect();

        for (pair, manifold) in narrowphase_results {
            frame
                .manifolds
                .entry(pair)
                .and_modify(|existing| {
                    if manifold.contacts.len() > existing.contacts.len()
                        || (manifold.contacts.len() == existing.contacts.len()
                            && manifold_max_penetration(&manifold)
                                > manifold_max_penetration(existing))
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
    world_aabbs: &HashMap<Entity, Aabb>,
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
    let extent = extent_a.min(extent_b);
    extent * 0.01
}

/// Delta_t will be used for toi/sweep
fn convex_convex_pair_manifold(
    entity_a: Entity,
    collider_a: &ConvexCollider,
    transform_a: &TransformComponent,
    velocity_a: Option<&VelocityComponent>,
    entity_b: Entity,
    collider_b: &ConvexCollider,
    transform_b: &TransformComponent,
    velocity_b: Option<&VelocityComponent>,
    world_aabbs: &HashMap<Entity, Aabb>,
    previous_manifold: Option<&ContactManifold>,
    _delta_t: Duration,
) -> Option<ContactManifold> {
    let pair = ordered_pair(entity_a, entity_b);
    let contacts = convex_convex_contact(
        entity_a,
        collider_a,
        transform_a,
        velocity_a,
        entity_b,
        collider_b,
        transform_b,
        velocity_b,
        previous_manifold,
    );
    let oriented_contacts: Vec<Contact> = contacts
        .into_iter()
        .map(|contact| orient_contact_to_pair(contact, pair))
        .collect();

    let merge_distance = manifold_merge_distance_pair_map(world_aabbs, pair.0, pair.1);
    let merged = merge_contact_manifold(
        previous_manifold,
        &oriented_contacts,
        merge_distance,
        0.95,
        4,
    );

    if merged.contacts.is_empty() {
        None
    } else {
        Some(merged)
    }
}

fn convex_mesh_pair_manifold(
    convex_entity: Entity,
    convex_collider: &ConvexCollider,
    convex_transform: &TransformComponent,
    convex_velocity: Option<&VelocityComponent>,
    mesh_entity: Entity,
    mesh_collider: &MeshCollider,
    mesh_transform: &TransformComponent,
    render_resources: &RenderResourceManager,
    world_aabbs: &HashMap<Entity, Aabb>,
    previous_manifold: Option<&ContactManifold>,
    delta_t: Duration,
) -> Option<ContactManifold> {
    let pair = ordered_pair(convex_entity, mesh_entity);
    let mesh_contacts = convex_mesh_contact(
        convex_entity,
        convex_collider,
        convex_transform,
        convex_velocity,
        mesh_entity,
        mesh_collider,
        mesh_transform,
        render_resources,
        previous_manifold,
        delta_t,
    );

    let oriented_contacts: Vec<Contact> = mesh_contacts
        .into_iter()
        .map(|contact| orient_contact_to_pair(contact, pair))
        .collect();

    let merge_distance = manifold_merge_distance_pair_map(world_aabbs, pair.0, pair.1);
    let merged = merge_contact_manifold(
        previous_manifold,
        &oriented_contacts,
        merge_distance,
        0.9,
        8,
    );

    if merged.contacts.is_empty() {
        None
    } else {
        Some(merged)
    }
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
    std::mem::swap(&mut contact.entity_a, &mut contact.entity_b);
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

fn transform_aabb(local: Aabb, transform: &TransformComponent) -> Aabb {
    let matrix = transform.to_mat4();
    transform_aabb_with_mat4(local, &matrix)
}

fn transform_aabb_with_mat4(local: Aabb, transform: &Mat4) -> Aabb {
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

    Aabb {
        min: world_min,
        max: world_max,
    }
}

fn aabb_intersects(a: &Aabb, b: &Aabb) -> bool {
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
) -> Vec<Contact> {
    let radius_a = collider_a.as_sphere_radius().unwrap();
    let radius_b = collider_b.as_sphere_radius().unwrap();

    let center_a = transform_a.position;
    let center_b = transform_b.position;
    let ab = center_b - center_a;
    let distance_sq = ab.length_squared();
    let radius_sum = radius_a + radius_b;
    let radius_sum_sq = radius_sum * radius_sum;

    if distance_sq >= radius_sum_sq {
        return Vec::new();
    }

    let distance = distance_sq.sqrt();
    let penetration = radius_sum - distance;

    let normal = if distance > f32::EPSILON {
        ab / distance
    } else {
        Vec3::X
    };

    let contact_point = center_a + normal * radius_a;

    vec![Contact {
        entity_a,
        entity_b,
        normal,
        penetration,
        contact_point,
    }]
}

fn cuboid_cuboid_contact(
    entity_a: Entity,
    collider_a: &ConvexCollider,
    transform_a: &TransformComponent,
    entity_b: Entity,
    collider_b: &ConvexCollider,
    transform_b: &TransformComponent,
) -> Vec<Contact> {
    // Extract half-sizes of cuboids
    let Some((a_len, a_wid, a_hei)) = collider_a.as_cuboid() else {
        return Vec::new();
    };
    let Some((b_len, b_wid, b_hei)) = collider_b.as_cuboid() else {
        return Vec::new();
    };

    // Build world transforms that include scale.
    let a_mat = transform_a.to_mat4();
    let b_mat = transform_b.to_mat4();

    let a_center = a_mat.transform_point3(Vec3::ZERO);
    let b_center = b_mat.transform_point3(Vec3::ZERO);

    // Extract world-space axes. These include scale in their lengths.
    let a_axis_raw = [
        a_mat.transform_vector3(Vec3::X),
        a_mat.transform_vector3(Vec3::Y),
        a_mat.transform_vector3(Vec3::Z),
    ];
    let b_axis_raw = [
        b_mat.transform_vector3(Vec3::X),
        b_mat.transform_vector3(Vec3::Y),
        b_mat.transform_vector3(Vec3::Z),
    ];

    // Axis lengths encode the entity scale; multiply collider half-extents
    // by these lengths to get true world-space half-extents, then normalize
    // the axes to unit length for the SAT.
    let a_scale = Vec3::new(
        a_axis_raw[0].length(),
        a_axis_raw[1].length(),
        a_axis_raw[2].length(),
    );
    let b_scale = Vec3::new(
        b_axis_raw[0].length(),
        b_axis_raw[1].length(),
        b_axis_raw[2].length(),
    );

    let a_axes = [
        if a_scale.x > f32::EPSILON {
            a_axis_raw[0] / a_scale.x
        } else {
            Vec3::X
        },
        if a_scale.y > f32::EPSILON {
            a_axis_raw[1] / a_scale.y
        } else {
            Vec3::Y
        },
        if a_scale.z > f32::EPSILON {
            a_axis_raw[2] / a_scale.z
        } else {
            Vec3::Z
        },
    ];
    let b_axes = [
        if b_scale.x > f32::EPSILON {
            b_axis_raw[0] / b_scale.x
        } else {
            Vec3::X
        },
        if b_scale.y > f32::EPSILON {
            b_axis_raw[1] / b_scale.y
        } else {
            Vec3::Y
        },
        if b_scale.z > f32::EPSILON {
            b_axis_raw[2] / b_scale.z
        } else {
            Vec3::Z
        },
    ];

    let a_extents = Vec3::new(
        a_len * 0.5 * a_scale.x,
        a_wid * 0.5 * a_scale.y,
        a_hei * 0.5 * a_scale.z,
    );
    let b_extents = Vec3::new(
        b_len * 0.5 * b_scale.x,
        b_wid * 0.5 * b_scale.y,
        b_hei * 0.5 * b_scale.z,
    );

    let t = b_center - a_center;

    // Rotation matrix from A to B
    let mut r = [[0.0_f32; 3]; 3];
    let mut abs_r = [[0.0_f32; 3]; 3];
    let eps = 1e-6_f32;
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a_axes[i].dot(b_axes[j]);
            abs_r[i][j] = r[i][j].abs() + eps;
        }
    }

    // Translation in A's frame
    let t_a = Vec3::new(t.dot(a_axes[0]), t.dot(a_axes[1]), t.dot(a_axes[2]));

    let mut min_penetration = f32::INFINITY;
    let mut best_axis = Vec3::ZERO;

    // SAT helper
    let mut check_axis = |axis: Vec3, penetration: f32| {
        if penetration < min_penetration {
            min_penetration = penetration;
            best_axis = axis;
        }
        penetration > 0.0
    };

    // Test axes: A0, A1, A2
    for i in 0..3 {
        let ra = a_extents[i];
        let rb = b_extents.x * abs_r[i][0] + b_extents.y * abs_r[i][1] + b_extents.z * abs_r[i][2];
        let sep = t_a[i].abs() - (ra + rb);
        if sep > 0.0 {
            return Vec::new();
        }
        let mut axis = a_axes[i];
        if t_a[i] < 0.0 {
            axis = -axis;
        }
        check_axis(axis, -sep);
    }

    // Test axes: B0, B1, B2
    for j in 0..3 {
        let ra = a_extents.x * abs_r[0][j] + a_extents.y * abs_r[1][j] + a_extents.z * abs_r[2][j];
        let rb = b_extents[j];
        let t_b = t_a.x * r[0][j] + t_a.y * r[1][j] + t_a.z * r[2][j];
        let sep = t_b.abs() - (ra + rb);
        if sep > 0.0 {
            return Vec::new();
        }
        let mut axis = b_axes[j];
        if t_b < 0.0 {
            axis = -axis;
        }
        check_axis(axis, -sep);
    }

    // Test cross axes: Ai x Bj
    for i in 0..3 {
        for j in 0..3 {
            let axis = a_axes[i].cross(b_axes[j]);
            if axis.length_squared() < eps {
                continue; // skip degenerate
            }

            let ra = a_extents[(i + 1) % 3] * abs_r[(i + 2) % 3][j]
                + a_extents[(i + 2) % 3] * abs_r[(i + 1) % 3][j];
            let rb = b_extents[(j + 1) % 3] * abs_r[i][(j + 2) % 3]
                + b_extents[(j + 2) % 3] * abs_r[i][(j + 1) % 3];
            let t_term =
                t_a[(i + 2) % 3] * r[(i + 1) % 3][j] - t_a[(i + 1) % 3] * r[(i + 2) % 3][j];
            let sep = t_term.abs() - (ra + rb);
            if sep > 0.0 {
                return Vec::new();
            }

            let mut axis_world = axis.normalize();
            if axis_world.dot(t) < 0.0 {
                axis_world = -axis_world;
            }
            check_axis(axis_world, -sep);
        }
    }

    if best_axis.length_squared() <= eps {
        return Vec::new();
    }

    // Build up to 4 contact points from vertices inside the opposing OBB.
    let normal = best_axis.normalize();
    let a_vertices = cuboid_world_vertices(a_center, a_axes, a_extents);
    let b_vertices = cuboid_world_vertices(b_center, b_axes, b_extents);

    let mut candidate_points: Vec<Vec3> = Vec::new();
    for point in a_vertices {
        if point_inside_obb(point, b_center, b_axes, b_extents, 1e-4) {
            candidate_points.push(point);
        }
    }
    for point in b_vertices {
        if point_inside_obb(point, a_center, a_axes, a_extents, 1e-4) {
            candidate_points.push(point);
        }
    }

    if candidate_points.is_empty() {
        let a_support = collider_a.support(transform_a.to_mat4(), normal);
        let b_support = collider_b.support(transform_b.to_mat4(), -normal);
        candidate_points.push((a_support + b_support) * 0.5);
    }

    candidate_points.sort_by(|a, b| {
        a.x.total_cmp(&b.x)
            .then_with(|| a.y.total_cmp(&b.y))
            .then_with(|| a.z.total_cmp(&b.z))
    });

    let mut unique_points: Vec<Vec3> = Vec::new();
    for point in candidate_points {
        let already_present = unique_points
            .iter()
            .any(|existing| (*existing - point).length_squared() <= 1e-6);
        if !already_present {
            unique_points.push(point);
        }
    }

    if unique_points.len() > 4 {
        unique_points.truncate(4);
    }

    unique_points
        .into_iter()
        .map(|contact_point| Contact {
            entity_a,
            entity_b,
            normal,
            penetration: min_penetration,
            contact_point,
        })
        .collect()
}

/// This should return a Vec<Contact> just like convex_mesh_contact, need to update (TODO)
fn convex_convex_contact(
    entity_a: Entity,
    collider_a: &ConvexCollider,
    transform_a: &TransformComponent,
    _velocity_a: Option<&VelocityComponent>,
    entity_b: Entity,
    collider_b: &ConvexCollider,
    transform_b: &TransformComponent,
    _velocity_b: Option<&VelocityComponent>,
    previous_manifold: Option<&ContactManifold>,
) -> Vec<Contact> {
    match (collider_a.shape, collider_b.shape) {
        (ConvexShape::Sphere { .. }, ConvexShape::Sphere { .. }) => sphere_sphere_contact(
            entity_a,
            collider_a,
            transform_a,
            entity_b,
            collider_b,
            transform_b,
        ),
        (ConvexShape::Cuboid { .. }, ConvexShape::Cuboid { .. }) => cuboid_cuboid_contact(
            entity_a,
            collider_a,
            transform_a,
            entity_b,
            collider_b,
            transform_b,
        ),
        _ => {
            let contact = gjk_epa(
                collider_a,
                transform_a,
                collider_b,
                transform_b,
                previous_manifold,
            );

            contact
                .map(|contact| Contact {
                    entity_a,
                    entity_b,
                    normal: contact.normal,
                    penetration: contact.penetration_depth,
                    contact_point: contact.contact_point,
                })
                .into_iter()
                .collect()
        }
    }
}

fn point_inside_obb(
    point: Vec3,
    center: Vec3,
    axes: [Vec3; 3],
    extents: Vec3,
    epsilon: f32,
) -> bool {
    let local = point - center;
    local.dot(axes[0]).abs() <= extents.x + epsilon
        && local.dot(axes[1]).abs() <= extents.y + epsilon
        && local.dot(axes[2]).abs() <= extents.z + epsilon
}

fn cuboid_world_vertices(center: Vec3, axes: [Vec3; 3], extents: Vec3) -> [Vec3; 8] {
    [
        center + axes[0] * extents.x + axes[1] * extents.y + axes[2] * extents.z,
        center + axes[0] * extents.x + axes[1] * extents.y - axes[2] * extents.z,
        center + axes[0] * extents.x - axes[1] * extents.y + axes[2] * extents.z,
        center + axes[0] * extents.x - axes[1] * extents.y - axes[2] * extents.z,
        center - axes[0] * extents.x + axes[1] * extents.y + axes[2] * extents.z,
        center - axes[0] * extents.x + axes[1] * extents.y - axes[2] * extents.z,
        center - axes[0] * extents.x - axes[1] * extents.y + axes[2] * extents.z,
        center - axes[0] * extents.x - axes[1] * extents.y - axes[2] * extents.z,
    ]
}

struct GjkEpaResult {
    normal: Vec3,
    penetration_depth: f32,
    contact_point: Vec3,
}

fn gjk_epa(
    collider_a: &ConvexCollider,
    transform_a: &TransformComponent,
    collider_b: &ConvexCollider,
    transform_b: &TransformComponent,
    previous_manifold: Option<&ContactManifold>,
) -> Option<GjkEpaResult> {
    let a_world = transform_a.to_mat4();
    let b_world = transform_b.to_mat4();

    let result = gjk_intersect(collider_a, a_world, collider_b, b_world);
    let simplex = match result {
        GjkResult::Intersection(hit) => hit.simplex,
        GjkResult::NoIntersection => return None,
    };

    let epa_result = epa(
        collider_a,
        a_world,
        collider_b,
        b_world,
        &simplex,
        previous_manifold,
    )?;

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
    previous_manifold: Option<&ContactManifold>,
    delta_t: Duration,
) -> Vec<Contact> {
    let Some(render_body) = render_resources
        .render_body_manager
        .get_render_body(mesh_collider.render_body_id)
    else {
        return Vec::new();
    };

    let convex_world = convex_transform.to_mat4();
    let sweep_delta = convex_velocity
        .map(|v| v.translational * delta_t.as_secs_f32())
        .unwrap_or(Vec3::ZERO);
    let has_sweep = sweep_delta.length_squared() > 0.0;
    let swept_transform = TransformComponent {
        position: convex_transform.position + sweep_delta,
        rotation: convex_transform.rotation,
        scale: convex_transform.scale,
    };
    let swept_world = swept_transform.to_mat4();
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
            previous_manifold,
        ));

        if has_sweep {
            candidates.extend(convex_mesh_swept_contact_at_transform(
                convex_collider,
                convex_world,
                swept_world,
                &mesh_world,
                &mesh_world_inv,
                bvh,
            ));
        }
    }
    reduce_contact_candidates(mesh_entity, convex_entity, candidates, convex_aabb_world)
}

/// Continuous convex-vs-mesh candidate generation using swept support-plane TOI.
/// This is more reliable for fast-moving bodies than pure substep sampling.
fn convex_mesh_swept_contact_at_transform(
    convex_collider: &ConvexCollider,
    start_world: Mat4,
    end_world: Mat4,
    mesh_world: &Mat4,
    mesh_world_inv: &Mat4,
    bvh: &BVHNode,
) -> Vec<ContactCandidate> {
    let collider_start_mesh = *mesh_world_inv * start_world;
    let collider_end_mesh = *mesh_world_inv * end_world;
    let start_aabb_mesh = convex_collider.aabb(&collider_start_mesh);
    let end_aabb_mesh = convex_collider.aabb(&collider_end_mesh);
    let swept_aabb_mesh = union_aabb(start_aabb_mesh, end_aabb_mesh);

    let mut triangles = Vec::new();
    collect_triangles_in_aabb(bvh, &swept_aabb_mesh, &mut triangles);
    if triangles.is_empty() {
        return Vec::new();
    }

    let mut candidates = Vec::new();
    for tri in triangles {
        let tri_world = Triangle {
            v0: mesh_world.transform_point3(tri.v0),
            v1: mesh_world.transform_point3(tri.v1),
            v2: mesh_world.transform_point3(tri.v2),
        };

        let n_unnorm = (tri_world.v1 - tri_world.v0).cross(tri_world.v2 - tri_world.v0);
        if n_unnorm.length_squared() <= f32::EPSILON {
            continue;
        }
        let mut normal = n_unnorm.normalize();

        // Keep normal pointing from mesh towards the moving convex center at start.
        let tri_center = (tri_world.v0 + tri_world.v1 + tri_world.v2) / 3.0;
        let convex_center_start = start_world.transform_point3(Vec3::ZERO);
        if normal.dot(convex_center_start - tri_center) < 0.0 {
            normal = -normal;
        }

        // Extreme point on convex towards triangle plane at start/end.
        let support_start = convex_collider.support(start_world, -normal);
        let support_end = convex_collider.support(end_world, -normal);

        let d0 = (support_start - tri_world.v0).dot(normal);
        let d1 = (support_end - tri_world.v0).dot(normal);

        // We only care about crossing from positive distance to non-positive distance.
        if !(d0 > 0.0 && d1 <= 0.0) {
            continue;
        }

        let denom = d0 - d1;
        if denom.abs() <= f32::EPSILON {
            continue;
        }

        let toi = (d0 / denom).clamp(0.0, 1.0);
        let support_t = support_start.lerp(support_end, toi);

        // Contact point on plane, then clamp to triangle.
        let signed = (support_t - tri_world.v0).dot(normal);
        let projected = support_t - normal * signed;
        let closest = closest_point_on_triangle(projected, &tri_world);

        // Reject far-off projections (outside triangle neighborhood).
        let lateral_error = (closest - projected).length_squared();
        if lateral_error > 0.25 {
            continue;
        }

        let penetration = (-d1).max(0.001);
        candidates.push(ContactCandidate {
            point: closest,
            normal,
            penetration,
        });
    }

    candidates
}

fn convex_mesh_contact_at_transform(
    convex_collider: &ConvexCollider,
    convex_world: Mat4,
    mesh_world: &Mat4,
    mesh_world_inv: &Mat4,
    bvh: &BVHNode,
    previous_manifold: Option<&ContactManifold>,
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

        let face_normal_world = {
            let n = (tri_world.v1 - tri_world.v0).cross(tri_world.v2 - tri_world.v0);
            if n.length_squared() > f32::EPSILON {
                Some(n.normalize())
            } else {
                None
            }
        };

        match convex_collider.shape {
            ConvexShape::Sphere { radius } => {
                let closest_world = closest_point_on_triangle(convex_center_world, &tri_world);
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
            }
            _ => {
                // Use a tiny-thickness triangle prism so GJK/EPA operates on a full 3D
                // convex polytope instead of a degenerate 2D triangle.
                let half_thickness = 1e-3;
                let triangle_collider = ConvexCollider::triangle_prism(
                    tri.v0,
                    tri.v1,
                    tri.v2,
                    half_thickness,
                    convex_collider.layer,
                );

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

                let epa_result = epa(
                    &triangle_collider,
                    *mesh_world,
                    convex_collider,
                    convex_world,
                    &simplex,
                    previous_manifold,
                );

                let penetration_depth = match epa_result {
                    Some(result) => result.penetration_depth,
                    None => {
                        continue;
                    }
                };

                let Some(tri_normal) = face_normal_world else {
                    continue;
                };
                let tri_center_local = (tri.v0 + tri.v1 + tri.v2) / 3.0;
                let tri_center_world = mesh_world.transform_point3(tri_center_local);
                let convex_center_world = convex_world.transform_point3(Vec3::ZERO);
                let ab = convex_center_world - tri_center_world;
                // For mesh triangle contacts, use the oriented triangle face
                // normal for projection and depth. This avoids inflated
                // penetration from skewed EPA normals on prism proxies.
                let mut normal = tri_normal;
                if ab.length_squared() > f32::EPSILON && normal.dot(ab) < 0.0 {
                    normal = -normal;
                }

                let support = convex_collider.support(convex_world, -normal);
                // project support point onto triangle plane
                let plane_point = tri_world.v0;
                let plane_normal = normal;
                let d = (support - plane_point).dot(plane_normal);
                let projected = support - plane_normal * d;
                let contact_point = closest_point_on_triangle(projected, &tri_world);
                let lateral_error = (contact_point - projected).length_squared();
                let edge0 = (tri_world.v1 - tri_world.v0).length();
                let edge1 = (tri_world.v2 - tri_world.v1).length();
                let edge2 = (tri_world.v0 - tri_world.v2).length();
                let tri_extent = edge0.max(edge1).max(edge2).max(0.01);
                let lateral_tolerance = tri_extent * 0.25 + 0.05;
                if lateral_error > lateral_tolerance * lateral_tolerance {
                    continue;
                }
                // Use triangle-plane penetration for final manifold depth to avoid
                // overestimation from the prism proxy thickness and axis selection.
                let plane_penetration = (-d).max(0.0);
                if plane_penetration <= f32::EPSILON {
                    continue;
                }

                let penetration = plane_penetration.min(penetration_depth);
                candidates.push(ContactCandidate {
                    point: contact_point,
                    normal,
                    penetration,
                });
            }
        }
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
    convex_aabb_world: Aabb,
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

fn collect_triangles_in_aabb(bvh: &BVHNode, target: &Aabb, out: &mut Vec<Triangle>) {
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

fn triangle_aabb(tri: &Triangle) -> Aabb {
    let min = tri.v0.min(tri.v1).min(tri.v2);
    let max = tri.v0.max(tri.v1).max(tri.v2);
    Aabb { min, max }
}

fn swept_aabb(aabb: &Aabb, delta: Vec3) -> Aabb {
    let moved_min = aabb.min + delta;
    let moved_max = aabb.max + delta;
    Aabb {
        min: aabb.min.min(moved_min),
        max: aabb.max.max(moved_max),
    }
}

fn render_body_local_aabb(
    render_body_id: crate::handles::RenderBodyHandle,
    render_resources: &RenderResourceManager,
) -> Option<Aabb> {
    let render_body = render_resources
        .render_body_manager
        .get_render_body(render_body_id)?;

    let mut combined: Option<Aabb> = None;
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

fn union_aabb(a: Aabb, b: Aabb) -> Aabb {
    Aabb {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}

#[cfg(test)]
mod tests {
    use crate::components::collider_component::CollisionLayer;

    use super::*;
    use approx::assert_relative_eq;
    use glam::{Mat4, Quat, Vec3};
    use std::path::Path;

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

    fn load_obj_triangles(obj_path: &str) -> Vec<Triangle> {
        let (models, _) = tobj::load_obj(
            Path::new(obj_path),
            &tobj::LoadOptions {
                triangulate: true,
                single_index: true,
                ..Default::default()
            },
        )
        .expect("Failed to load OBJ");

        let mut triangles = Vec::new();
        for model in models {
            let mesh = model.mesh;
            for tri_idx in (0..mesh.indices.len()).step_by(3) {
                let i0 = mesh.indices[tri_idx] as usize;
                let i1 = mesh.indices[tri_idx + 1] as usize;
                let i2 = mesh.indices[tri_idx + 2] as usize;

                let v0 = Vec3::new(
                    mesh.positions[i0 * 3],
                    mesh.positions[i0 * 3 + 1],
                    mesh.positions[i0 * 3 + 2],
                );
                let v1 = Vec3::new(
                    mesh.positions[i1 * 3],
                    mesh.positions[i1 * 3 + 1],
                    mesh.positions[i1 * 3 + 2],
                );
                let v2 = Vec3::new(
                    mesh.positions[i2 * 3],
                    mesh.positions[i2 * 3 + 1],
                    mesh.positions[i2 * 3 + 2],
                );

                triangles.push(Triangle { v0, v1, v2 });
            }
        }

        triangles
    }

    fn snapshot_best_penetration(convex_transform: TransformComponent) -> f32 {
        let mesh_transform = TransformComponent {
            position: Vec3::new(0.0, 0.0, -20.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::splat(10.0),
        };

        let convex_collider =
            ConvexCollider::cuboid(Vec3::new(2.0, 2.0, 2.000001), CollisionLayer::Player);
        let ground_obj = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("./test_resources/test_ground/test_ground.obj");
        let triangles = load_obj_triangles(ground_obj.to_str().expect("Invalid UTF-8 path"));
        let bvh = BVHNode::build(triangles, 8);

        let mesh_world = mesh_transform.to_mat4();
        let mesh_world_inv = mesh_world.inverse();
        let convex_world = convex_transform.to_mat4();

        let candidates = convex_mesh_contact_at_transform(
            &convex_collider,
            convex_world,
            &mesh_world,
            &mesh_world_inv,
            &bvh,
            None,
        );

        assert!(
            !candidates.is_empty(),
            "Expected at least one convex-vs-mesh contact candidate"
        );

        let contacts = reduce_contact_candidates(
            Entity::from_bits(3),
            Entity::from_bits(2),
            candidates,
            convex_collider.aabb(&convex_world),
        );

        assert!(
            !contacts.is_empty(),
            "Expected reduced contact set to be non-empty"
        );

        contacts
            .iter()
            .map(|c| c.penetration)
            .fold(0.0_f32, f32::max)
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
            None,
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
            None,
        );

        assert_eq!(contacts.len(), 1);
        let contact = contacts[0];
        assert!(contact.normal.z < 0.0);
        assert_relative_eq!(contact.normal.length(), 1.0, epsilon = 1e-4);
        assert!(contact.penetration > 0.0);
    }

    #[test]
    fn convex_mesh_contact_at_transform_cuboid_hits_triangle() {
        let tri = make_triangle();
        let bvh = BVHNode::build(vec![tri], 4);

        let convex_collider = ConvexCollider::cuboid(Vec3::splat(1.0), CollisionLayer::Default);
        let convex_transform = TransformComponent {
            position: Vec3::new(0.25, 0.25, 0.4),
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
            None,
        );

        assert!(
            !contacts.is_empty(),
            "Expected cuboid-mesh contact candidates"
        );
        let best = contacts
            .iter()
            .max_by(|a, b| a.penetration.total_cmp(&b.penetration))
            .unwrap();
        assert!(best.penetration > 0.0);
        // Triangle is on Z=0 with +Z normal; cuboid center is above, so normal should be upward.
        assert!(best.normal.z > 0.0);
    }

    #[test]
    fn reduce_contact_candidates_caps_to_four() {
        let mesh_entity = Entity::from_bits(1);
        let convex_entity = Entity::from_bits(2);
        let convex_aabb_world = Aabb {
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
        let convex_aabb_world = Aabb {
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
    fn cuboid_cuboid_contact_scaling_changes_overlap_state() {
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
        // At scale 1: A spans [-1,1], B spans [1.1,3.1] → no overlap
        assert!(initial.is_empty());

        // At scale 1.2x: B spans [2.1-1.2, 2.1+1.2] = [0.9, 3.3] → overlaps A
        transform_b.scale = Vec3::new(1.2, 1.0, 1.0);
        let scaled = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );

        assert!(!scaled.is_empty(), "Scaling should cause overlap");
        let max_penetration = scaled
            .iter()
            .map(|contact| contact.penetration)
            .fold(0.0_f32, f32::max);
        assert!(max_penetration > 0.0);
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
        assert!(initial.is_empty());

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

        assert_eq!(rotated.is_empty(), rotated_scaled.is_empty());
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
        assert!(initial.is_empty());

        transform_b.rotation = Quat::from_rotation_z(45.0_f32.to_radians());
        let rotated = cuboid_cuboid_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );

        assert!(!rotated.is_empty(), "Expected overlap after rotation");
        let deepest = rotated
            .iter()
            .max_by(|a, b| a.penetration.total_cmp(&b.penetration))
            .unwrap();
        assert!(deepest.penetration > 0.0);
        assert_relative_eq!(deepest.normal.length(), 1.0, epsilon = 1e-4);
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
        assert!(initial.is_empty());

        transform_b.scale = Vec3::splat(1.2);
        let scaled = sphere_sphere_contact(
            entity_a,
            &collider_a,
            &transform_a,
            entity_b,
            &collider_b,
            &transform_b,
        );

        assert_eq!(initial.is_empty(), scaled.is_empty());
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
        assert!(initial.is_empty());

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

        assert_eq!(rotated.is_empty(), rotated_scaled.is_empty());
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
        );

        assert!(!contact.is_empty(), "Expected overlap for unequal cuboids");
        let deepest = contact
            .iter()
            .map(|c| c.penetration)
            .fold(0.0_f32, f32::max);
        assert_relative_eq!(deepest, 0.5, epsilon = 1e-4);
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

        assert!(contact.is_empty());
    }

    #[test]
    fn cuboid_cuboid_contact_scaled_cuboid_overlaps() {
        // A cube(2) at origin: spans [-1,1]. B cube(2) at x=1.5 with scale 2:
        // B spans [1.5-2, 1.5+2] = [-0.5, 3.5] → overlap of 1.5
        let entity_a = Entity::from_bits(50);
        let entity_b = Entity::from_bits(51);

        let collider = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let transform_b =
            make_transform(Vec3::new(1.5, 0.0, 0.0), Quat::IDENTITY, Vec3::splat(2.0));

        let contact = cuboid_cuboid_contact(
            entity_a,
            &collider,
            &transform_a,
            entity_b,
            &collider,
            &transform_b,
        );

        assert!(!contact.is_empty(), "Expected overlap with scaled cuboid");
        let deepest = contact
            .iter()
            .map(|c| c.penetration)
            .fold(0.0_f32, f32::max);
        assert_relative_eq!(deepest, 1.5, epsilon = 0.05);
    }

    #[test]
    fn cuboid_cuboid_contact_non_uniform_scale_y() {
        // A cube(2) at origin spans [-1,1] on all axes.
        // B cube(2) at y=1.5 with scale (1,2,1): B y-extent becomes 2 → spans [-0.5, 3.5]
        // Overlap on y = 1.5, overlap on x and z = 2.0 each. Min penetration = 1.5 on y.
        let entity_a = Entity::from_bits(52);
        let entity_b = Entity::from_bits(53);

        let collider = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let transform_b = make_transform(
            Vec3::new(0.0, 1.5, 0.0),
            Quat::IDENTITY,
            Vec3::new(1.0, 2.0, 1.0),
        );

        let contact = cuboid_cuboid_contact(
            entity_a,
            &collider,
            &transform_a,
            entity_b,
            &collider,
            &transform_b,
        );

        assert!(
            !contact.is_empty(),
            "Expected overlap with non-uniform y-scaled cuboid"
        );
        let best = contact
            .iter()
            .max_by(|a, b| a.penetration.total_cmp(&b.penetration))
            .unwrap();
        assert!(best.penetration > 0.0);
        assert!(best.normal.y.abs() > 0.5);
    }

    #[test]
    fn cuboid_cuboid_contact_non_uniform_scale_no_overlap() {
        // A cube(2) at origin spans [-1,1].
        // B cube(2) at x=3.5 with scale (1,1,1): B spans [2.5, 4.5] → no overlap
        let entity_a = Entity::from_bits(54);
        let entity_b = Entity::from_bits(55);

        let collider = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let transform_b = make_transform(Vec3::new(3.5, 0.0, 0.0), Quat::IDENTITY, Vec3::ONE);

        let contact = cuboid_cuboid_contact(
            entity_a,
            &collider,
            &transform_a,
            entity_b,
            &collider,
            &transform_b,
        );

        assert!(contact.is_empty());
    }

    #[test]
    fn cuboid_cuboid_contact_rotated_and_scaled_overlap() {
        // Two cubes, B rotated 45° around Z and scaled by 1.5.
        // At x=2.0, a 45° rotated cube(2) scaled 1.5 has diagonal reach of
        // sqrt(2) * 1.5 ≈ 2.12 which should overlap A's edge at 1.0.
        let entity_a = Entity::from_bits(56);
        let entity_b = Entity::from_bits(57);

        let collider = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let transform_b = make_transform(
            Vec3::new(2.0, 0.0, 0.0),
            Quat::from_rotation_z(45.0_f32.to_radians()),
            Vec3::splat(1.5),
        );

        let contact = cuboid_cuboid_contact(
            entity_a,
            &collider,
            &transform_a,
            entity_b,
            &collider,
            &transform_b,
        );

        assert!(
            !contact.is_empty(),
            "Expected overlap for rotated+scaled cuboid"
        );
        let max_penetration = contact
            .iter()
            .map(|c| c.penetration)
            .fold(0.0_f32, f32::max);
        assert!(max_penetration > 0.0);
    }

    #[test]
    fn convex_convex_pair_manifold_cuboid_face_face_generates_four_contacts() {
        let entity_a = Entity::from_bits(70);
        let entity_b = Entity::from_bits(71);

        let collider_a = ConvexCollider::cube(2.0, CollisionLayer::Default);
        let collider_b = ConvexCollider::cube(2.0, CollisionLayer::Default);

        let transform_a = make_transform(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);
        let transform_b = make_transform(Vec3::new(0.0, 1.5, 0.0), Quat::IDENTITY, Vec3::ONE);

        let mut world_aabbs = std::collections::HashMap::new();
        world_aabbs.insert(entity_a, collider_a.aabb(&transform_a.to_mat4()));
        world_aabbs.insert(entity_b, collider_b.aabb(&transform_b.to_mat4()));

        let manifold = convex_convex_pair_manifold(
            entity_a,
            &collider_a,
            &transform_a,
            None,
            entity_b,
            &collider_b,
            &transform_b,
            None,
            &world_aabbs,
            None,
            Duration::from_secs_f32(1.0 / 60.0),
        )
        .expect("Expected cuboid face-face manifold");

        assert_eq!(manifold.contacts.len(), 4);

        for contact in &manifold.contacts {
            assert_eq!(contact.entity_a, entity_a);
            assert_eq!(contact.entity_b, entity_b);
            assert!(contact.penetration > 0.0);
            assert!(contact.normal.y > 0.5);
        }

        for i in 0..manifold.contacts.len() {
            for j in (i + 1)..manifold.contacts.len() {
                let dist_sq = (manifold.contacts[i].contact_point
                    - manifold.contacts[j].contact_point)
                    .length_squared();
                assert!(dist_sq > 1e-6, "Expected unique manifold points");
            }
        }
    }

    #[test]
    fn target_penetration_bound_for_main_scene_snapshot() {
        let convex_transform = TransformComponent {
            position: Vec3::new(0.14915955, 0.91336507, -17.742033),
            rotation: Quat::from_xyzw(-0.0040421374, -0.003234554, 0.33637854, 0.9417117),
            scale: Vec3::ONE,
        };
        let best_penetration = snapshot_best_penetration(convex_transform);

        assert!(
            best_penetration <= 0.50,
            "Penetration bound violated for snapshot: {}",
            best_penetration
        );
    }

    #[test]
    fn target_penetration_bound_for_main_scene_snapshot_2() {
        let convex_transform = TransformComponent {
            position: Vec3::new(0.12149104, -1.2194518, -17.7922),
            rotation: Quat::from_xyzw(6.6116976e-5, 0.0007644149, 0.6084883, 0.7935627),
            scale: Vec3::ONE,
        };

        let best_penetration = snapshot_best_penetration(convex_transform);

        assert!(
            best_penetration <= 0.50,
            "Penetration bound violated for snapshot_2: {}",
            best_penetration
        );
    }

    #[test]
    /// Contact information gathered from a snapshot of the main scene,
    /// where two entities had 4 contact points with identical normals and penetrations.
    /// This tests that the merge_contact_manifold function correctly retains all 4
    /// contacts when they are identical, rather than erroneously merging them into fewer contacts.
    fn merge_contact_manifold_should_return_full_manifold() {
        let entity_a = Entity::from_bits(420);
        let entity_b = Entity::from_bits(69);
        let contacts = vec![
            Contact {
                entity_a: entity_a,
                entity_b: entity_b,
                normal: Vec3 {
                    x: 0.0,
                    y: -0.0,
                    z: 1.0,
                },
                penetration: 0.035182,
                contact_point: Vec3 {
                    x: 1.9607796e-5,
                    y: -0.9804878,
                    z: 1.2450399,
                },
            },
            Contact {
                entity_a: entity_a,
                entity_b: entity_b,
                normal: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                },
                penetration: 0.035182,
                contact_point: Vec3 {
                    x: 1.0174599,
                    y: 1.9593022e-5,
                    z: 1.2450399,
                },
            },
            Contact {
                entity_a: entity_a,
                entity_b: entity_b,
                normal: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                },
                penetration: 0.035182,
                contact_point: Vec3 {
                    x: 1.0174636,
                    y: -0.9804802,
                    z: 1.2450399,
                },
            },
            Contact {
                entity_a: entity_a,
                entity_b: entity_b,
                normal: Vec3 {
                    x: -0.0,
                    y: 0.0,
                    z: 1.0,
                },
                penetration: 0.03251047,
                contact_point: Vec3 {
                    x: 2e-5,
                    y: 2e-5,
                    z: 1.2450399,
                },
            },
        ];

        let merged = merge_contact_manifold(None, &contacts, 0.1, 0.9, 8);
        assert_eq!(merged.contacts.len(), 4);

        let merged_2 = merge_contact_manifold(Some(&merged), &contacts, 0.1, 0.9, 8);
        assert_eq!(merged_2.contacts.len(), 4);
    }
}
