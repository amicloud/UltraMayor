use bevy_ecs::prelude::*;
use bevy_ecs::schedule::{IntoScheduleConfigs, Schedule};
use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;

use engine::physics_resource::{CollisionFrameData, PhysicsResource};
use engine::{
    CollisionLayer, CollisionSystem, ConvexCollider, RenderResourceManager, TransformComponent,
};
use glam::{Quat, Vec3};

fn spawn_convex_grid(world: &mut World, count: usize, spacing: f32, radius: f32) {
    let side = (count as f32).cbrt().ceil() as usize;
    let mut spawned = 0usize;

    for z in 0..side {
        for y in 0..side {
            for x in 0..side {
                if spawned >= count {
                    return;
                }

                let position =
                    Vec3::new(x as f32 * spacing, y as f32 * spacing, z as f32 * spacing);

                world.spawn((
                    TransformComponent {
                        position,
                        rotation: Quat::IDENTITY,
                        scale: Vec3::splat(1.0),
                    },
                    ConvexCollider::sphere(radius, CollisionLayer::Default),
                ));

                spawned += 1;
            }
        }
    }
}

fn jiggle_transforms(world: &mut World, delta: f32) {
    let mut query = world.query::<&mut TransformComponent>();
    for mut transform in query.iter_mut(world) {
        transform.position.x += delta;
    }
}

fn setup_world(count: usize, spacing: f32, radius: f32) -> World {
    let mut world = World::new();
    world.insert_resource(RenderResourceManager::new());
    world.insert_resource(PhysicsResource::default());
    spawn_convex_grid(&mut world, count, spacing, radius);
    world
}

fn bench_broadphase_update(c: &mut Criterion) {
    let mut world = setup_world(1024, 3.0, 1.0);
    let mut schedule = Schedule::default();
    schedule.add_systems(CollisionSystem::update_world_dynamic_tree);

    schedule.run(&mut world);

    let mut flip = false;
    c.bench_function("collision/broadphase_update_1024", |b| {
        b.iter(|| {
            let delta = if flip { 0.01 } else { -0.01 };
            flip = !flip;
            jiggle_transforms(&mut world, delta);
            schedule.run(&mut world);
            black_box(
                world
                    .get_resource::<PhysicsResource>()
                    .expect("PhysicsResource missing")
                    .world_aabbs
                    .len(),
            );
        })
    });
}

fn bench_generate_contacts(c: &mut Criterion) {
    let mut world = setup_world(512, 1.5, 1.0);
    let mut schedule = Schedule::default();
    schedule.add_systems(
        (
            CollisionSystem::update_world_dynamic_tree,
            CollisionSystem::generate_contacts,
        )
            .chain(),
    );

    schedule.run(&mut world);

    let mut flip = false;
    c.bench_function("collision/contacts_convex_512", |b| {
        b.iter(|| {
            let delta = if flip { 0.005 } else { -0.005 };
            flip = !flip;
            jiggle_transforms(&mut world, delta);
            schedule.run(&mut world);
            black_box(
                world
                    .get_resource::<CollisionFrameData>()
                    .expect("CollisionFrameData missing")
                    .contacts
                    .len(),
            );
        })
    });
}

fn bench_generate_contacts_touching(c: &mut Criterion) {
    let mut world = setup_world(512, 0.9, 1.0);
    let mut schedule = Schedule::default();
    schedule.add_systems(
        (
            CollisionSystem::update_world_dynamic_tree,
            CollisionSystem::generate_contacts,
        )
            .chain(),
    );

    schedule.run(&mut world);

    let mut flip = false;
    c.bench_function("collision/contacts_convex_512_touching", |b| {
        b.iter(|| {
            let delta = if flip { 0.005 } else { -0.005 };
            flip = !flip;
            jiggle_transforms(&mut world, delta);
            schedule.run(&mut world);
            black_box(
                world
                    .get_resource::<CollisionFrameData>()
                    .expect("CollisionFrameData missing")
                    .contacts
                    .len(),
            );
        })
    });
}

criterion_group!(
    benches,
    bench_broadphase_update,
    bench_generate_contacts,
    bench_generate_contacts_touching
);
criterion_main!(benches);
