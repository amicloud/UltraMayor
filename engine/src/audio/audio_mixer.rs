use cpal::{
    Device, Stream, SupportedStreamConfig,
    traits::{DeviceTrait, HostTrait, StreamTrait},
};
use glam::Vec3;
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
    pub fn fill_buffer_from_voices(&mut self) {
        self.finished_indices_buffer.clear();
        self.buffer.fill(0.0);
        if !self.playing {
            return;
        }
        let mute_gain = if self.muted { 0.0 } else { 1.0 };
        for (i, voice) in self.voices.iter_mut().enumerate() {
            if voice.next_frame() {
                for c in 0..self.channels {
                    // If the voice is mono, use the first channel for all output channels
                    let src_channel = if voice.channels == 1 { 0 } else { c };
                    self.buffer[c] += voice.buffer[src_channel] * self.volume * mute_gain;
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
}

impl Voice {
    /// Returns false when the voice is out of samples (has no next frame)
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
    UpdateListenerPosition {
        position: Vec3,
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

        let mut tracks = Vec::with_capacity(32);
        tracks.push(Track {
            volume: 1.0,
            playing: true,
            voices: Vec::new(),
            buffer: vec![0.0; channels],
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

        let listener_position = None;

        s.stream = Some(s.build_stream(&device, config, tracks, paused, consumer, muted, listener_position));
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
        mut listener_position: Option<Vec3>,
    ) -> Stream {
        let channels = config.channels() as usize;
        let mut mixed = vec![0.0; channels];
        let stream = device
            .build_output_stream(
                &config.into(),
                move |output: &mut [f32], _: &cpal::OutputCallbackInfo| {
                    Self::process_mixer_commands(
                        &mut consumer,
                        &mut tracks,
                        &mut paused,
                        &mut muted,
                        &mut listener_position,
                    );
                    if paused {
                        for frame_out in output.chunks_mut(channels) {
                            for c in frame_out.iter_mut().take(channels) {
                                *c = 0.0;
                            }
                        }
                        return;
                    }

                    for frame_out in output.chunks_mut(channels) {
                        mixed.fill(0.0);

                        for track in tracks.iter_mut() {
                            // This is ok for now but I need to move to block processing instead of 
                            // per frame... Somehow.
                            track.fill_buffer_from_voices();
                            for (c, channel) in mixed.iter_mut().enumerate().take(channels) {
                                // If the track is mono, use the first channel for all output channels
                                *channel += track.buffer[if track.channels == 1 { 0 } else { c }];
                            }
                        }

                        // clamp to -1..1 to avoid clipping
                        // If the mix is muted, output silence regardless of track buffers
                        let mute_gain = if muted { 0.0 } else { 1.0 };
                        for (c, channel) in frame_out.iter_mut().enumerate().take(channels) {
                            *channel = mixed[c].clamp(-1.0, 1.0) * mute_gain;
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
        listener_position: &mut Option<Vec3>,
    ) {
        while let Some(command) = consumer.pop().ok() {
            match command {
                MixerCommand::AddVoice {
                    track,
                    samples,
                    volume,
                    looping,
                    channels,
                    location
                } => {
                    if let Some(track) = tracks.get_mut(track) {
                        track.voices.push(Voice {
                            samples: samples,
                            cursor: 0,
                            volume: volume,
                            looping: looping,
                            channels: channels,
                            buffer: vec![0.0; channels],
                            location
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
                MixerCommand::UpdateListenerPosition { position } => {
                    *listener_position = Some(position);
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
                AudioCommand::PlaySound {
                    track,
                    sound,
                    volume,
                    looping,
                    location
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
                AudioCommand::UpdateListenerPosition { position } => {
                    self.producer
                        .push(MixerCommand::UpdateListenerPosition { position: *position })
                        .expect(MIXER_FULL_ERROR_MESSAGE);
                }
            }
        }
    }
}
