#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct MeshHandle(pub u32);

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct MaterialHandle(pub u32);

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct TextureHandle(pub u32);

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShaderHandle(pub u32);

/// A handle to a RenderBody in the engine.
/// RenderBody represents a collection of meshes, materials, and associated data used for rendering a 3D model.
#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct RenderBodyHandle(pub u32);
