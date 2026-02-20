use std::sync::{Arc, Mutex};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use rtrb::Producer;

use crate::assets::sound_resource::SoundResource;

use super::audio_queue::AudioInstance;

pub struct AudioMixer {
    device: cpal::Device,
    stream: Option<cpal::Stream>,
    // producer: Option<Producer<f32>>,
    sample_rate: cpal::SampleRate,
    channels: usize,
    voices: Arc<Mutex<Vec<Voice>>>,
}

struct Voice {
    samples: Arc<[f32]>,
    cursor: usize,
    volume: f32,
    looping: bool,
}

impl Voice {
    fn next_sample(&mut self) -> f32 {
        if self.cursor >= self.samples.len() {
            if self.looping {
                self.cursor = 0;
            } else {
                return 0.0;
            }
        }
        let sample = self.samples[self.cursor] * self.volume;
        self.cursor += 1;
        sample
    }

    fn sine_test(&mut self, frequency: f32, sample_rate: u32) -> f32 {
        let sample = (2.0 * std::f32::consts::PI * frequency * self.cursor as f32
            / sample_rate as f32)
            .sin()
            * self.volume;
        self.cursor += 1;
        sample
    }
}

impl Default for AudioMixer {
    fn default() -> Self {
        let host = cpal::default_host();
        let device = host
            .default_output_device()
            .expect("no output device available");
        let sample_rate = device.default_output_config().unwrap().sample_rate();
        let channels = device.default_output_config().unwrap().channels();
        let sample = [0.0; 1024]; // Placeholder, should be actual audio data
        let voice_1 = Voice {
            samples: Arc::new(sample), // Placeholder, should be actual audio data
            cursor: 0,
            volume: 0.5,
            looping: true,
        };
        let config = device.default_output_config().unwrap();
        let channels = config.channels() as usize;

        let voices = Arc::new(Mutex::new(vec![voice_1]));
        let stream_voices = voices.clone();
        let stream = 
            device
            .build_output_stream(
                &config.into(),
                move |output: &mut [f32], _: &cpal::OutputCallbackInfo| {
                    let mut voices = stream_voices.lock().unwrap();
                    for frame in output.chunks_mut(channels) {
                        let mut mixed = 0.0;

                        for voice in voices.iter_mut() {
                            mixed += voice.next_sample();
                        }
                        for sample_out in frame.iter_mut() {
                            *sample_out = mixed;
                        }
                    }
                },
                move |err| eprintln!("Stream error: {}", err),
                None, // None=blocking, Some(Duration)=timeout
            )
            .expect("failed to build output stream");
        Self {
            device,
            stream: Some(stream),
            // producer: None,
            sample_rate,
            channels,
            voices,
        }
    }
}

impl AudioMixer {
    fn play(&self) {
        self.stream
            .as_ref()
            .expect("No stream.")
            .play()
            .expect("failed to play stream");
    }

    fn pause(&self) {
        self.stream
            .as_ref()
            .expect("No stream.")
            .pause()
            .expect("failed to pause stream");
    }

    pub fn handle_audio_queue(&mut self, queue: &[AudioInstance], sound_resource: &SoundResource) {
        for instance in queue.iter() {
            if let Some(sound) = sound_resource.get_sound(instance.sound) {
                self.add_voice(Arc::from(sound.data.clone()), instance.volume, instance.looping);
            } else {
                eprintln!("Sound with ID {:?} not found in SoundResource", instance.sound);
            }
        }
    }

    pub fn add_voice(&mut self, samples: Arc<[f32]>, volume: f32, looping: bool) {
        let voice = Voice {
            samples,
            cursor: 0,
            volume,
            looping,
        };
        self.voices.lock().unwrap().push(voice);
    }
}
