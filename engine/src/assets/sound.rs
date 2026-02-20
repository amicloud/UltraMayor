use crate::{Engine, SoundHandle, assets::sound_resource};

pub struct Sound {
    pub sample_rate: u32,
    pub channels: u16,
    pub data: Vec<f32>,
}

impl Sound {
    pub fn new(sample_rate: u32, channels: u16, data: Vec<f32>) -> Self {
        Self {
            sample_rate,
            channels,
            data,
        }
    }

    pub fn from_wav(path: &str) -> Self {
        let mut reader = hound::WavReader::open(path).unwrap();
        let samples = reader.samples::<i16>();
        let data = samples.map(|s| s.unwrap() as f32 / i16::MAX as f32).collect();
        Self {
            sample_rate: 44100,
            channels: 2,
            data,
        }
    }
}

impl Engine {
    pub fn load_wav(&mut self, path: &str) -> Result<SoundHandle, String> {
        let sound = Sound::from_wav(path);
        let mut sound_resource = self.world.get_resource_mut::<sound_resource::SoundResource>().unwrap();
        let handle = sound_resource.add_sound(sound);
        Ok(handle)
    }
}