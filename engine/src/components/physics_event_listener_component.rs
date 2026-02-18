use bevy_ecs::prelude::*;

#[derive(Component, Debug)]
/// A marker component for entities that want to listen to physics events.
/// Entities with this component will receive `PhysicsEvent` events when they occur.
/// This allows us to not have to trigger events for every entity in the world, only those that are interested.
pub struct PhysicsEventListenerComponent;
