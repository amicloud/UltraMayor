use bevy_ecs::entity::Entity;
use cpal::{
    Device, Stream, SupportedStreamConfig,
    traits::{DeviceTrait, HostTrait, StreamTrait},
};
use glam::{Quat, Vec3};
use rtrb::{Consumer, Producer, RingBuffer};
use std::{collections::HashMap, sync::Arc};

use crate::{
    assets::sound_resource::SoundResource,
    audio::{audio_control::AudioCommand, track::Track, voice::Voice},
};
pub struct AudioMixer {
    stream: Option<Stream>,
    pub sample_rate: cpal::SampleRate,
    producer: Producer<MixerCommand>,
}

pub(crate) type ListenerInfo = (Vec3, Quat); // position, rotation
pub(crate) type SourceInfo = Vec3; // position

enum MixerCommand {
    AddVoice {
        track: u8,
        samples: Arc<[f32]>,
        sample_rate: f32,
        volume: f32,
        looping: bool,
        source_channels: u16,
        source: Option<Entity>,
        location: Option<Vec3>,
    },
    PauseMix,
    ResumeMix,
    MuteMix,
    UnmuteMix,
    MuteTrack {
        track: u8,
    },
    UnmuteTrack {
        track: u8,
    },
    UpdateListenerInfo {
        info: ListenerInfo,
    },
    UpdateSourceInfo {
        entity: Entity,
        info: SourceInfo,
    },
    RemoveSourceInfo {
        entity: Entity,
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
        let channels = config.channels() as u16;

        let tracks: [Track; 32] = core::array::from_fn(|_| Track {
            volume: 1.0,
            playing: true,
            voices: Vec::with_capacity(256),
            buffer: vec![0.0; 4096 * channels as usize],
            channels,
            finished_indices_buffer: Vec::with_capacity(256),
            muted: false,
        });

        let paused = false;
        let muted = false;
        let (producer, consumer) = RingBuffer::<MixerCommand>::new(4096);
        let mut s = Self {
            stream: None,
            producer,
            sample_rate,
        };

        let listener_info = None; // position, rotation

        // Should probably not use a hashmap here but it works for now. We can optimize later if needed.
        let source_map: HashMap<Entity, Vec3> = HashMap::with_capacity(256);

        s.stream = Some(s.build_stream(
            &device,
            config,
            tracks,
            paused,
            consumer,
            muted,
            listener_info,
            source_map,
        ));
        s
    }
}

impl AudioMixer {
    #[allow(clippy::too_many_arguments)]
    fn build_stream(
        &mut self,
        device: &Device,
        config: SupportedStreamConfig,
        mut tracks: [Track; 32],
        mut paused: bool,
        mut consumer: Consumer<MixerCommand>,
        mut muted: bool,
        mut listener_info: Option<ListenerInfo>,
        mut source_map: HashMap<Entity, Vec3>,
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
                        &mut source_map,
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
                        track.fill_buffer_from_voices(
                            listener_info.as_ref(),
                            required_frames,
                            &source_map,
                        );
                    }

                    for frame in 0..required_frames {
                        for ch in 0..channels {
                            let mut sample = 0.0;
                            for track in tracks.iter() {
                                let src_ch = if track.channels == 1 { 0 } else { ch };
                                sample += track.buffer[frame * track.channels as usize + src_ch];
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
        tracks: &mut [Track],
        paused: &mut bool,
        muted: &mut bool,
        listener_info: &mut Option<ListenerInfo>,
        required_buffer_size_for_voices: usize,
        source_map: &mut HashMap<Entity, Vec3>,
    ) {
        while let Ok(command) = consumer.pop() {
            match command {
                MixerCommand::AddVoice {
                    track,
                    samples,
                    sample_rate,
                    volume,
                    looping,
                    source_channels,
                    source,
                    location,
                } => {
                    if let Some(track) = tracks.get_mut(track as usize) {
                        track.voices.push(Voice::new(
                            samples,
                            sample_rate,
                            volume,
                            looping,
                            source,
                            location,
                            source_channels,
                            required_buffer_size_for_voices,
                        ));
                        if let Some(source) = source {
                            source_map.insert(source, Vec3::ZERO); // Default location
                        }
                    }
                }
                MixerCommand::PauseMix => {
                    *paused = true;
                }
                MixerCommand::ResumeMix => {
                    *paused = false;
                }
                MixerCommand::MuteMix => {
                    *muted = true;
                }
                MixerCommand::UnmuteMix => {
                    *muted = false;
                }
                MixerCommand::MuteTrack { track } => {
                    if let Some(track) = tracks.get_mut(track as usize) {
                        track.muted = true;
                    }
                }
                MixerCommand::UnmuteTrack { track } => {
                    if let Some(track) = tracks.get_mut(track as usize) {
                        track.muted = false;
                    }
                }
                MixerCommand::UpdateListenerInfo { info: l } => {
                    *listener_info = Some(l);
                }
                MixerCommand::UpdateSourceInfo {
                    entity,
                    info: position,
                } => {
                    source_map.insert(entity, position);
                }
                MixerCommand::RemoveSourceInfo { entity } => {
                    source_map.remove(&entity);
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
            match command {
                AudioCommand::SpawnSpatialEmitter {
                    track,
                    sound,
                    volume,
                    looping,
                    source,
                } => {
                    if let Some(sound) = sound_resource.get_sound(*sound) {
                        self.producer
                            .push(MixerCommand::AddVoice {
                                track: *track,
                                samples: sound.data.clone(),
                                sample_rate: sound.sample_rate as f32,
                                volume: *volume,
                                looping: *looping,
                                source_channels: sound.channels,
                                source: Some(*source),
                                location: None,
                            })
                            .expect(MIXER_FULL_ERROR_MESSAGE);
                    } else {
                        eprintln!("Sound ID {:?} not found", sound);
                    }
                }
                AudioCommand::PlayOneShotAtLocation {
                    track,
                    sound,
                    volume,
                    location,
                } => {
                    if let Some(sound) = sound_resource.get_sound(*sound) {
                        self.producer
                            .push(MixerCommand::AddVoice {
                                track: *track,
                                samples: sound.data.clone(), // Cloning an Arc
                                sample_rate: sound.sample_rate as f32,
                                volume: *volume,
                                looping: false,
                                source_channels: sound.channels,
                                source: None,
                                location: Some(*location),
                            })
                            .expect(MIXER_FULL_ERROR_MESSAGE);
                    } else {
                        eprintln!("Sound ID {:?} not found", sound);
                    }
                }
                AudioCommand::PlayOneShot {
                    track,
                    sound,
                    volume,
                } => {
                    if let Some(sound) = sound_resource.get_sound(*sound) {
                        self.producer
                            .push(MixerCommand::AddVoice {
                                track: *track,
                                samples: sound.data.clone(), // Cloning an Arc
                                sample_rate: sound.sample_rate as f32,
                                volume: *volume,
                                looping: false,
                                source_channels: sound.channels,
                                source: None,
                                location: None,
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
                AudioCommand::MuteTrack { track } => {
                    self.producer
                        .push(MixerCommand::MuteTrack { track: *track })
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
                AudioCommand::UnmuteTrack { track } => {
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
                AudioCommand::UpdateListenerInfo {
                    info: listener_info,
                } => {
                    self.producer
                        .push(MixerCommand::UpdateListenerInfo {
                            info: *listener_info,
                        })
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
                AudioCommand::UpdateSourceInfo {
                    entity,
                    info: source_info,
                } => {
                    self.producer
                        .push(MixerCommand::UpdateSourceInfo {
                            entity: *entity,
                            info: *source_info,
                        })
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
                AudioCommand::RemoveSourceInfo { entity } => {
                    self.producer
                        .push(MixerCommand::RemoveSourceInfo { entity: *entity })
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
            }
        }
    }
}
