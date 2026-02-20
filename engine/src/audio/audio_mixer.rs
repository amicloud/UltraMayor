use std::sync::{Arc, Mutex};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use rtrb::Producer;

use crate::assets::sound_resource::SoundResource;

use super::audio_queue::AudioInstance;

pub struct AudioMixer {
    device: cpal::Device,
    stream: Option<cpal::Stream>,
    // producer: Option<Producer<f32>>,
    pub sample_rate: cpal::SampleRate,
    voices: Arc<Mutex<Vec<Voice>>>,
}

struct Voice {
    samples: Arc<[f32]>,
    cursor: usize,
    volume: f32,
    looping: bool,
    channels: usize,
}

impl Voice {
    fn next_frame(&mut self) -> Vec<f32> {
        let mut frame = vec![0.0; self.channels];
        for c in 0..self.channels {
            let idx = self.cursor * self.channels + c;
            if idx == self.samples.len() - 1 {
                if self.looping {
                    self.cursor = 0;
                } else {
                    return frame; // zeroes
                }
            }
            frame[c] = self.samples[idx] * self.volume;
        }
        self.cursor += 1;
        frame
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
        dbg!(sample_rate);
        let config = device.default_output_config().unwrap();
        let channels = config.channels() as usize;

        let voices: Arc<Mutex<Vec<Voice>>> = Arc::new(Mutex::new(Vec::new()));
        let stream_voices = voices.clone();
        let stream = device
            .build_output_stream(
                &config.into(),
                move |output: &mut [f32], _: &cpal::OutputCallbackInfo| {
                    let mut voices = stream_voices.lock().unwrap();
                    for frame_out in output.chunks_mut(channels) {
                        let mut mixed = vec![0.0; channels];
                        for voice in voices.iter_mut() {
                            let v_frame = voice.next_frame();
                            for c in 0..channels {
                                mixed[c] += v_frame[c];
                            }
                        }
                        // clamp to -1..1 to avoid clipping
                        for c in 0..channels {
                            frame_out[c] = mixed[c].clamp(-1.0, 1.0);
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
                self.add_voice(
                    Arc::from(sound.data.clone()),
                    instance.volume,
                    instance.looping,
                    sound.channels,
                );
            } else {
                eprintln!(
                    "Sound with ID {:?} not found in SoundResource",
                    instance.sound
                );
            }
        }
    }

    pub fn add_voice(&mut self, samples: Arc<[f32]>, volume: f32, looping: bool, channels: usize) {
        let voice = Voice {
            samples,
            cursor: 0,
            volume,
            looping,
            channels, 
        };
        self.voices.lock().unwrap().push(voice);
    }
}
