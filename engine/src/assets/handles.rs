use slotmap::new_key_type;

new_key_type! { 
    pub struct MeshHandle; 
    pub struct MaterialHandle;
}

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct TextureHandle(pub u32);

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShaderHandle(pub u32);

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct SoundHandle(pub u32);

/// A handle to a RenderBody in the engine.
/// RenderBody represents a collection of meshes, materials, and associated data used for rendering a 3D model.
#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct RenderBodyHandle(pub u32);