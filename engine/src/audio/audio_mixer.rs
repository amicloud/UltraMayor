use std::sync::{Arc, Mutex};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};

use crate::{
    SoundHandle, assets::sound_resource::SoundResource, audio::command_queue::AudioCommand,
};

pub struct AudioMixer {
    device: cpal::Device,
    stream: Option<cpal::Stream>,
    // producer: Option<Producer<f32>>,
    pub sample_rate: cpal::SampleRate,
    tracks: Arc<Mutex<Vec<Track>>>,
    mix_buffer: Vec<f32>,
    paused: bool,
}

pub struct Track {
    volume: f32,
    playing: bool,
    voices: Vec<Voice>,
    buffer: Vec<f32>,
    channels: usize,
    finished_indices_buffer: Vec<usize>,
    muted: bool,
}

impl Track {
    pub fn next_frame(&mut self) {
        self.finished_indices_buffer.clear();
        self.buffer.fill(0.0);
        if !self.playing {
            return;
        }
        for (i, voice) in self.voices.iter_mut().enumerate() {
            if voice.next_frame() {
                for c in 0..self.channels {
                    // If the voice is mono, use the first channel for all output channels
                    let src_channel = if voice.channels == 1 { 0 } else { c };
                    self.buffer[c] +=
                        voice.buffer[src_channel] * self.volume * !self.muted as u8 as f32;
                }
            } else {
                self.finished_indices_buffer.push(i);
            }
        }
        for &index in self.finished_indices_buffer.iter().rev() {
            log::info!("Voice {} finished, removing from track", index);
            self.voices.remove(index);
        }
    }
}

struct Voice {
    samples: Arc<[f32]>,
    cursor: usize,
    volume: f32,
    looping: bool,
    channels: usize,
    buffer: Vec<f32>,
}

impl Voice {
    /// Returns false when the voice is out of sampels (has no next frame)
    fn next_frame(&mut self) -> bool {
        let total_frames = self.samples.len() / self.channels;

        if self.cursor >= total_frames {
            if self.looping {
                self.cursor = 0;
            } else {
                self.buffer.fill(0.0);
                return false; // finished
            }
        }

        for c in 0..self.channels {
            let idx = self.cursor * self.channels + c;
            self.buffer[c] = self.samples[idx] * self.volume;
        }

        self.cursor += 1;
        true
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

        let tracks: Arc<Mutex<Vec<Track>>> = Arc::new(Mutex::new(Vec::new()));
        tracks.lock().unwrap().push(Track {
            volume: 1.0,
            playing: true,
            voices: Vec::new(),
            buffer: vec![0.0; channels],
            channels,
            finished_indices_buffer: Vec::new(),
            muted: false,
        });
        let stream_tracks = tracks.clone();
        let stream = device
            .build_output_stream(
                &config.into(),
                move |output: &mut [f32], _: &cpal::OutputCallbackInfo| {
                    // if self.paused {
                    //     for frame_out in output.chunks_mut(channels) {
                    //         for c in 0..channels {
                    //             frame_out[c] = 0.0;
                    //         }
                    //     }
                    //     return;
                    // }
                    let mut tracks = stream_tracks.lock().unwrap();
                    let mut mixed = vec![0.0; channels];

                    for frame_out in output.chunks_mut(channels) {
                        mixed.fill(0.0);

                        for track in tracks.iter_mut() {
                            // Fill the track buffer
                            track.next_frame();
                            for c in 0..channels {
                                // If the track is mono, use the first channel for all output channels
                                let src_channel = if track.channels == 1 { 0 } else { c };
                                mixed[c] += track.buffer[src_channel];
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
        stream.play().expect("failed to play stream");
        Self {
            device,
            stream: Some(stream),
            // producer: None,
            sample_rate,
            tracks: tracks.clone(),
            mix_buffer: vec![0.0; channels],
            paused: false,
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

    pub fn add_track(&mut self, volume: f32, channels: usize) {
        self.tracks.lock().unwrap().push(Track {
            volume,
            playing: true,
            voices: Vec::new(),
            buffer: vec![0.0; channels],
            channels,
            finished_indices_buffer: Vec::new(),
            muted: false,
        });
    }

    pub fn set_track_volume(&mut self, track: usize, volume: f32) {
        if let Some(track) = self.tracks.lock().unwrap().get_mut(track) {
            track.volume = volume;
        } else {
            eprintln!("Track {} does not exist", track);
        }
    }

    pub fn mute_track(&mut self, track: usize, mute: bool) {
        if let Some(track) = self.tracks.lock().unwrap().get_mut(track) {
            track.muted = mute;
        } else {
            eprintln!("Track {} does not exist", track);
        }
    }

    pub fn process_commands(&mut self, commands: &[AudioCommand], sound_resource: &SoundResource) {
        for command in commands.iter() {
            match command {
                AudioCommand::PlaySound {
                    track,
                    sound,
                    volume,
                    looping,
                } => {
                    if let Some(sound) = sound_resource.get_sound(*sound) {
                        self.add_voice_to_track(
                            Arc::from(sound.data.clone()),
                            *track,
                            *volume,
                            *looping,
                            sound.channels as usize,
                        );
                    } else {
                        eprintln!("Sound with ID {:?} not found in SoundResource", sound);
                    }
                }
                AudioCommand::PauseTrack { track } => {
                    if let Some(track) = self.tracks.lock().unwrap().get_mut(*track) {
                        track.playing = false;
                    } else {
                        eprintln!("Track {} does not exist", track);
                    }
                }
                AudioCommand::ResumeTrack { track } => {
                    if let Some(track) = self.tracks.lock().unwrap().get_mut(*track) {
                        track.playing = true;
                    } else {
                        eprintln!("Track {} does not exist", track);
                    }
                }
                AudioCommand::PauseMix => {
                    self.paused = true;
                }
                AudioCommand::ResumeMix => {
                    self.paused = false;
                }
            }
        }
    }

    pub fn add_voice_to_track(
        &mut self,
        samples: Arc<[f32]>,
        track: usize,
        volume: f32,
        looping: bool,
        channels: usize,
    ) {
        let voice = Voice {
            samples,
            cursor: 0,
            volume,
            looping,
            channels,
            buffer: vec![0.0; channels],
        };
        if let Some(track) = self.tracks.lock().unwrap().get_mut(track) {
            track.voices.push(voice);
        } else {
            eprintln!("Track {} does not exist", track);
        }
    }
}
