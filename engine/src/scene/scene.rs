use bevy_ecs::prelude::*;

use crate::{
    ActiveCamera, Gravity, TimeResource, WorldBasis,
    audio::audio_control::AudioControl,
    input::InputStateResource,
    physics::physics_resource::{CollisionFrameData, PhysicsFrameData, PhysicsResource},
    render::render_queue::RenderQueue,
    scene::{scene_changer_resource::SceneChangerResource, scene_services::SceneServices},
};

pub struct Scene {
    pub world: World,
    pub game_frame_schedule: Schedule,
    pub game_simulation_schedule: Schedule,
}

impl Scene {
    pub fn new(services: &SceneServices) -> Self {
        let mut world = World::new();

        world.insert_resource(services.clone());

        // Insert services
        world.insert_resource(services.meshes.clone());
        world.insert_resource(services.textures.clone());
        world.insert_resource(services.shaders.clone());
        world.insert_resource(services.sounds.clone());
        world.insert_resource(services.bodies.clone());
        world.insert_resource(services.materials.clone());

        world.insert_resource(RenderQueue::default());
        world.insert_resource(ActiveCamera::default());
        world.insert_resource(InputStateResource::default());
        world.insert_resource(WorldBasis::canonical());
        world.insert_resource(PhysicsResource::default());
        world.insert_resource(CollisionFrameData::default());
        world.insert_resource(PhysicsFrameData::default());
        world.insert_resource(TimeResource::new(60, 120));
        world.insert_resource(Gravity::default());
        world.insert_resource(AudioControl::default());
        world.insert_resource(SceneChangerResource::default());

        let game_frame_schedule = Schedule::default();
        let game_simulation_schedule = Schedule::default();

        Scene {
            world,
            game_frame_schedule,
            game_simulation_schedule,
        }
    }
}
