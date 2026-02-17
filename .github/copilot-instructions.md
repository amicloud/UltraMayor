# Copilot instructions for UltraMayor

## Project layout & architecture
- Cargo workspace with two crates: engine (lib) and game (bin). The engine owns the SDL2 window + OpenGL context and runs the Bevy ECS `World` + `Schedule`; the game crate wires gameplay systems on top. See [Cargo.toml](../Cargo.toml), [engine/src/lib.rs](../engine/src/lib.rs), [game/src/main.rs](../game/src/main.rs).

## Common workflows
- Run the game: `cargo run -p UltraMayor` (from workspace root).
- Run tests: `cargo test -p engine` or `cargo test -p UltraMayor`.
