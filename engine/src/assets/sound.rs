use std::path::Path;

use crate::{Engine, SoundHandle, assets::sound_resource::SoundResource};

pub struct Sound {
    pub sample_rate: u32,
    pub channels: usize,
    pub data: Vec<f32>,
}

impl Sound {
    pub fn new(sample_rate: u32, channels: usize, data: Vec<f32>) -> Self {
        Self {
            sample_rate,
            channels,
            data,
        }
    }

    pub fn from_wav(path: &str, sample_rate: u32) -> Self {
        let mut reader = hound::WavReader::open(path).unwrap();
        let spec = reader.spec();
        if spec.channels == 1 {
            let samples = reader
                .samples::<i16>()
                .map(|f| f.unwrap())
                .collect::<Vec<i16>>();
            let data = Self::resample_mono(&samples, spec.sample_rate, sample_rate);
            Self {
                sample_rate,
                channels: spec.channels as usize,
                data,
            }
        } else if spec.channels == 2 {
            let samples = reader
                .samples::<i16>()
                .map(|f| f.unwrap())
                .collect::<Vec<i16>>();
            let data = Self::resample_stereo(&samples, spec.sample_rate, sample_rate);
            Self {
                sample_rate,
                channels: spec.channels as usize,
                data,
            }
        } else {
            panic!("Unsupported number of channels: {}", spec.channels);
        }
    }

    fn resample_mono(samples: &[i16], src_rate: u32, dst_rate: u32) -> Vec<f32> {
        if src_rate == dst_rate {
            return samples.iter().map(|&s| Self::i16_to_f32(s)).collect();
        }

        let len_out = (samples.len() as u64 * dst_rate as u64 / src_rate as u64) as usize;
        let mut out = Vec::with_capacity(len_out);

        for i in 0..len_out {
            let t = i as f32 * src_rate as f32 / dst_rate as f32;
            let idx = t.floor() as usize;
            let frac = t - t.floor();
            let s0 = samples[idx];
            let s1 = samples.get(idx + 1).copied().unwrap_or(s0);
            out.push(Self::i16_to_f32(s0) * (1.0 - frac) + Self::i16_to_f32(s1) * frac);
        }

        out
    }
    fn i16_to_f32(sample: i16) -> f32 {
        (sample as f32) / 32768.0
    }

    fn resample_stereo(samples: &[i16], src_rate: u32, dst_rate: u32) -> Vec<f32> {
        if src_rate == dst_rate {
            return samples.iter().map(|&s| Self::i16_to_f32(s)).collect();
        }

        assert!(samples.len().is_multiple_of(2));
        let len_in = samples.len() / 2;
        let len_out = (len_in as u64 * dst_rate as u64 / src_rate as u64) as usize;

        let mut out = Vec::with_capacity(len_out * 2);

        for i in 0..len_out {
            let t = i as f32 * src_rate as f32 / dst_rate as f32;
            let idx = t.floor() as usize;
            let frac = t - t.floor();

            let l0 = samples[2 * idx];
            let r0 = samples[2 * idx + 1];
            let l1 = samples.get(2 * (idx + 1)).copied().unwrap_or(l0);
            let r1 = samples.get(2 * (idx + 1) + 1).copied().unwrap_or(r0);

            out.push(Self::i16_to_f32(l0) * (1.0 - frac) + Self::i16_to_f32(l1) * frac);
            out.push(Self::i16_to_f32(r0) * (1.0 - frac) + Self::i16_to_f32(r1) * frac);
        }

        out
    }
}

impl Engine {
    pub fn load_wav(&mut self, path: &str) -> Result<SoundHandle, String> {
        let sample_rate = self.audio_mixer.sample_rate;
        let sound = Sound::from_wav(path, sample_rate);
        let mut sound_resource = self.world.get_resource_mut::<SoundResource>().unwrap();
        let name = Path::new(path)
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown")
            .to_string();
        let handle = sound_resource.add_sound(sound, name);
        Ok(handle)
    }
}
