# Copilot instructions for UltraMayor

## Project layout & architecture
- Cargo workspace with two crates: engine (lib) and game (bin). The engine owns the SDL2 window + OpenGL context and runs the Bevy ECS `World` + `Schedule`; the game crate wires gameplay systems on top. See [Cargo.toml](../Cargo.toml), [engine/src/lib.rs](../engine/src/lib.rs), [game/src/main.rs](../game/src/main.rs).
- Engine systems run in a fixed chain inside `Engine::new()`; game code extends the same `Schedule` via `engine.schedule.add_systems(...)`. Keep this structure instead of creating a separate schedule. See [engine/src/lib.rs](../engine/src/lib.rs).

## ECS & gameplay patterns
- Inputs are captured in the engine loop and stored in `InputStateResource`. Gameplay systems should read from that resource; avoid adding SDL2 event handling in game code. See [engine/src/input.rs](../engine/src/input.rs) and [engine/src/lib.rs](../engine/src/lib.rs).
- Cameras are ECS entities. The active render camera is stored in the `ActiveCamera` resource and read during render extraction; game camera entities should always have a CameraComponent component, and camera controllers in the game operate by mutating transforms, velocities, and other relevant components on their respective camera entity and should set the active camera in the `ActiveCamera` resource to a camera entity. See [game/src/camera_controller.rs](../game/src/camera_controller.rs).

## Rendering & assets
- Rendering is a two-step pipeline: `RenderSystem::extract_render_data` builds a `RenderQueue` of `RenderInstance`s from `TransformComponent` + `RenderBodyComponent`, then `Renderer::render` consumes it with `RenderResourceManager`. See [engine/src/render_system.rs](../engine/src/render_system.rs) and [engine/src/renderer.rs](../engine/src/renderer.rs).
- To render new models, use `Engine::load_model(...)` (supports glTF/glb/obj) which returns a `RenderBodyHandle`, then spawn an entity with `TransformComponent` + `RenderBodyComponent`. See [engine/src/model_loader.rs](../engine/src/model_loader.rs) and [engine/src/render_body_component.rs](../engine/src/render_body_component.rs).
- Materials/shaders are managed centrally in `RenderResourceManager`; PBR shaders live in resources/shaders. See [engine/src/render_resource_manager.rs](../engine/src/render_resource_manager.rs) and resources/shaders.

## Physics & collisions
- Physics is currently fixed-step at 1/60s in `PhysicsSystem::update_bodies`; impulses should be added via `Engine::add_impulse` (queues into `PhysicsResource`). See [engine/src/physics_system.rs](../engine/src/physics_system.rs) and [engine/src/lib.rs](../engine/src/lib.rs).
- Collisions are AABB-based. Use `Engine::collider_from_render_body(...)` to build colliders from render data; collisions use cached world AABBs in `PhysicsResource`. See [engine/src/collision_system.rs](../engine/src/collision_system.rs).
- The physics system is largely unfinished. Collision -> Impulse resolution is not yet implemented.

## Settings & tests
- User settings live in TOML under the OS config dir and are loaded/saved by `Settings` (tests rely on temp dirs + `serial_test`). See [game/src/settings.rs](../game/src/settings.rs).

## Common workflows
- Run the game: `cargo run -p UltraMayor` (from workspace root).
- Run tests: `cargo test -p engine` or `cargo test -p UltraMayor`.
