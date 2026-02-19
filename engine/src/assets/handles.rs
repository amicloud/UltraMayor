use slotmap::new_key_type;

new_key_type! { 
    pub struct MeshHandle; 
    pub struct MaterialHandle;
    pub struct TextureHandle;
    pub struct ShaderHandle;
    pub struct SoundHandle;
    pub struct RenderBodyHandle;
}