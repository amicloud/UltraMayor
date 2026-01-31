#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct MeshHandle(pub u32);

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct MaterialHandle(pub u32);

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct TextureHandle(pub u32);

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct ShaderHandle(pub u32);

/// A handle to a RenderBody in the engine.
/// RenderBody represents a collection of meshes, materials, and associated data used for rendering a 3D model.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct RenderBodyHandle(pub u32);
