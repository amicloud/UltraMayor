use cpal::{
    Device, Stream, SupportedStreamConfig,
    traits::{DeviceTrait, HostTrait, StreamTrait},
};
use glam::{Quat, Vec3};
use rtrb::{Consumer, Producer, RingBuffer};
use std::sync::Arc;

use crate::{assets::sound_resource::SoundResource, audio::command_queue::AudioCommand};
pub struct AudioMixer {
    stream: Option<Stream>,
    pub sample_rate: cpal::SampleRate,
    producer: Producer<MixerCommand>,
}

#[derive(Debug)]
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
    pub fn fill_buffer_from_voices(&mut self, listener_info: ListenerInfo, required_frames: usize) {
        self.finished_indices_buffer.clear();
        self.buffer.fill(0.0);
        if !self.playing {
            return;
        }
        let mute_gain = if self.muted { 0.0 } else { 1.0 };
        for (i, voice) in self.voices.iter_mut().enumerate() {
            if voice.next_block(listener_info, required_frames) {
                for frame in 0..required_frames {
                    for ch in 0..self.channels {
                        let src_ch = if voice.channels == 1 { 0 } else { ch };
                        self.buffer[frame * self.channels + ch] +=
                            voice.buffer[frame * voice.channels + src_ch] * self.volume * mute_gain;
                    }
                }
            } else {
                self.finished_indices_buffer.push(i);
            }
        }
        for &index in self.finished_indices_buffer.iter().rev() {
            dbg!("Voice {} finished, removing from track", index);
            self.voices.swap_remove(index);
        }
    }
}

#[derive(Debug)]
struct Voice {
    samples: Arc<[f32]>,
    cursor: usize,
    volume: f32,
    looping: bool,
    channels: usize,
    buffer: Vec<f32>,
    location: Option<Vec3>,
    delay_buffer_left: Vec<f32>,  // for ITD
    delay_buffer_right: Vec<f32>, // for ITD
}

pub type ListenerInfo = Option<(Vec3, Quat)>; // position, rotation

impl Voice {
    fn next_block(&mut self, listener_info: ListenerInfo, required_frames: usize) -> bool {
        let distance_attenuation =
            if let (Some(location), Some((listener_pos, _))) = (self.location, listener_info) {
                let distance = location.distance(listener_pos);
                // Simple linear attenuation with distance, clamped to a minimum of 0.1 to avoid complete silence
                (1.0 - distance / 100.0).max(0.1)
            } else {
                1.0
            };
        let total_frames = self.samples.len() / self.channels;
        let frames_to_fill = (total_frames - self.cursor).min(required_frames);

        for frame in 0..frames_to_fill {
            let sample_idx = self.cursor * self.channels;
            if self.channels == 2 {
                self.buffer[frame * 2] =
                    self.samples[sample_idx] * self.volume * distance_attenuation;
                self.buffer[frame * 2 + 1] =
                    self.samples[sample_idx + 1] * self.volume * distance_attenuation;
            } else if self.channels == 1 {
                let sample = self.samples[self.cursor] * self.volume * distance_attenuation;
                self.buffer[frame * 2] = sample;
                self.buffer[frame * 2 + 1] = sample;
            } else {
                for ch in 0..self.channels {
                    self.buffer[frame * self.channels + ch] =
                        self.samples[sample_idx + ch] * self.volume * distance_attenuation;
                }
            }
            self.cursor += 1;
        }

        // Zero the rest of the block if we ran out of frames
        for frame in frames_to_fill..required_frames {
            for ch in 0..self.channels {
                self.buffer[frame * self.channels + ch] = 0.0;
            }
        }

        return self.cursor < total_frames || self.looping;
    }
}

enum MixerCommand {
    AddVoice {
        track: usize,
        samples: Arc<[f32]>,
        volume: f32,
        looping: bool,
        channels: usize,
        location: Option<Vec3>,
    },
    PauseMix,
    ResumeMix,
    MuteMix,
    UnmuteMix,
    MuteTrack {
        track: usize,
    },
    UnmuteTrack {
        track: usize,
    },
    UpdateListenerInfo {
        listener_info: ListenerInfo,
    },
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

        let mut tracks = Vec::with_capacity(32);
        tracks.push(Track {
            volume: 1.0,
            playing: true,
            voices: Vec::new(),
            buffer: vec![0.0; 5096 * 2], 
            channels,
            finished_indices_buffer: Vec::new(),
            muted: false,
        });

        let paused = false;
        let muted = false;
        let (producer, consumer) = RingBuffer::<MixerCommand>::new(1024);
        let mut s = Self {
            stream: None,
            producer,
            sample_rate,
        };

        let listener_info = None; // position, rotation

        s.stream = Some(s.build_stream(
            &device,
            config,
            tracks,
            paused,
            consumer,
            muted,
            listener_info,
        ));
        s
    }
}

impl AudioMixer {
    fn build_stream(
        &mut self,
        device: &Device,
        config: SupportedStreamConfig,
        mut tracks: Vec<Track>,
        mut paused: bool,
        mut consumer: Consumer<MixerCommand>,
        mut muted: bool,
        mut listener_info: ListenerInfo,
    ) -> Stream {
        let channels = config.channels() as usize;
            let stream = device
            .build_output_stream(
                &config.into(),
                move |output: &mut [f32], _: &cpal::OutputCallbackInfo| {
                    Self::process_mixer_commands(
                        &mut consumer,
                        &mut tracks,
                        &mut paused,
                        &mut muted,
                        &mut listener_info,
                        output.len(),
                    );
                    if paused {
                        for frame_out in output.chunks_mut(channels) {
                            for c in frame_out.iter_mut().take(channels) {
                                *c = 0.0;
                            }
                        }
                        return;
                    }

                    let required_frames = output.len() / channels;

                    for track in tracks.iter_mut() {
                        track.fill_buffer_from_voices(listener_info, required_frames);
                    }

                    for frame in 0..required_frames {
                        for ch in 0..channels {
                            let mut sample = 0.0;
                            for track in tracks.iter() {
                                let src_ch = if track.channels == 1 { 0 } else { ch };
                                sample += track.buffer[frame * track.channels + src_ch];
                            }
                            let mute_gain = if muted { 0.0 } else { 1.0 };
                            output[frame * channels + ch] = sample.clamp(-1.0, 1.0) * mute_gain;
                        }
                    }
                },
                move |err| eprintln!("Stream error: {}", err),
                None, // None=blocking, Some(Duration)=timeout
            )
            .expect("failed to build output stream");
        stream.play().expect("failed to play stream");
        stream
    }

    fn process_mixer_commands(
        consumer: &mut Consumer<MixerCommand>,
        tracks: &mut Vec<Track>,
        paused: &mut bool,
        muted: &mut bool,
        listener_info: &mut ListenerInfo,
        required_buffer_size_for_voices: usize,
    ) {
        while let Some(command) = consumer.pop().ok() {
            match command {
                MixerCommand::AddVoice {
                    track,
                    samples,
                    volume,
                    looping,
                    channels,
                    location,
                } => {
                    if let Some(track) = tracks.get_mut(track) {
                        track.voices.push(Voice {
                            samples: samples,
                            cursor: 0,
                            volume: volume,
                            looping: looping,
                            channels: channels,
                            buffer: vec![0.0; required_buffer_size_for_voices],
                            location,
                            delay_buffer_left: vec![0.0; required_buffer_size_for_voices],
                            delay_buffer_right: vec![0.0; required_buffer_size_for_voices],
                        });
                    } else {
                        eprintln!("Track {} does not exist", track);
                    }
                }
                MixerCommand::PauseMix => {
                    eprintln!("Pausing mix");
                    *paused = true;
                }
                MixerCommand::ResumeMix => {
                    eprintln!("Resuming mix");
                    *paused = false;
                }
                MixerCommand::MuteMix => {
                    eprintln!("Muting mix");
                    *muted = true;
                }
                MixerCommand::UnmuteMix => {
                    eprintln!("Unmuting mix");
                    *muted = false;
                }
                MixerCommand::MuteTrack { track } => {
                    if let Some(track) = tracks.get_mut(track) {
                        eprintln!("Muting track {:?}", track);
                        track.muted = true;
                    } else {
                        eprintln!("Track {} does not exist", track);
                    }
                }
                MixerCommand::UnmuteTrack { track } => {
                    if let Some(track) = tracks.get_mut(track) {
                        eprintln!("Unmuting track {:?}", track);
                        track.muted = false;
                    } else {
                        eprintln!("Track {} does not exist", track);
                    }
                }
                MixerCommand::UpdateListenerInfo { listener_info: l } => {
                    *listener_info = l;
                }
            }
        }
    }

    pub fn make_mixer_commands(
        &mut self,
        commands: &[AudioCommand],
        sound_resource: &SoundResource,
    ) {
        const MIXER_FULL_ERROR_MESSAGE: &str = "Audio mixer command queue is full! Sorry.";
        for command in commands.iter() {
            // dbg!(command);
            match command {
                AudioCommand::PlaySound {
                    track,
                    sound,
                    volume,
                    looping,
                    location,
                } => {
                    if let Some(sound) = sound_resource.get_sound(*sound) {
                        self.producer
                            .push(MixerCommand::AddVoice {
                                track: *track,
                                samples: sound.data.clone(),
                                volume: *volume,
                                looping: *looping,
                                channels: sound.channels,
                                location: *location,
                            })
                            .expect(MIXER_FULL_ERROR_MESSAGE);
                    } else {
                        eprintln!("Sound ID {:?} not found", sound);
                    }
                }
                AudioCommand::PauseMix => {
                    self.producer
                        .push(MixerCommand::PauseMix)
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
                AudioCommand::ResumeMix => {
                    self.producer
                        .push(MixerCommand::ResumeMix)
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
                AudioCommand::PauseTrack { track } => {
                    self.producer
                        .push(MixerCommand::MuteTrack { track: *track })
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
                AudioCommand::ResumeTrack { track } => {
                    self.producer
                        .push(MixerCommand::UnmuteTrack { track: *track })
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
                AudioCommand::MuteMix => {
                    self.producer
                        .push(MixerCommand::MuteMix)
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
                AudioCommand::UnmuteMix => {
                    self.producer
                        .push(MixerCommand::UnmuteMix)
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
                AudioCommand::UpdateListenerInfo { listener_info } => {
                    self.producer
                        .push(MixerCommand::UpdateListenerInfo {
                            listener_info: *listener_info,
                        })
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
            }
        }
    }
}
