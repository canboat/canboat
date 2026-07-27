// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense NGT-1 codec adapter for [`super::run`].
//!
//! Wraps [`canboat_core::format::ngt1`]: the device reader pushes
//! incoming bytes through `Ngt1Decoder`, the writer encodes
//! [`RawFrame`]s as `N2K_MSG_SEND` (0x94) frames, and a 20-second
//! keepalive resends the NGT-1 startup ping while the channel is
//! quiet. Synthetic PGNs (`>= 0x40000`) are skipped on the write
//! path, matching canboat's behaviour.

use std::io::{Read, Write};
use std::time::Duration;

use canboat_core::RawFrame;
use canboat_core::format::{
    encode_n2k_send_frame, encode_startup_ping,
    ngt1::{Ngt1Decoder, NgtEvent},
};

use super::{DeviceDecoder, DeviceEncoder, DeviceEvent, DeviceHandle};

/// Re-ping the NGT-1 startup sequence every 20 s — matches the C
/// `actisense-serial` keepalive.
pub const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(20);

/// Synthetic-PGN marker. Frames at or above this PGN are emitted by
/// internal canboat tooling and must never hit the bus.
pub const ACTISENSE_SYNTHETIC_PGN: u32 = 0x40000;

/// Start the NGT-1 reader/writer threads. See [`super::run`].
pub fn run(reader: Box<dyn Read + Send>, writer: Box<dyn Write + Send>) -> DeviceHandle {
    super::run(Decoder::new(), Encoder, reader, writer)
}

/// Decoder wrapper that adapts [`Ngt1Decoder`] to [`DeviceDecoder`].
pub struct Decoder {
    inner: Ngt1Decoder,
}

impl Decoder {
    pub fn new() -> Self {
        Self {
            inner: Ngt1Decoder::new(),
        }
    }
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceDecoder for Decoder {
    fn decode(&mut self, bytes: &[u8], events: &mut Vec<DeviceEvent>) {
        for ev in self.inner.push_bytes(bytes) {
            match ev {
                NgtEvent::Message(msg) => {
                    if let Some(frame) = msg.to_raw_frame() {
                        events.push(DeviceEvent::Frame(frame));
                    }
                }
                NgtEvent::Error(e) => events.push(DeviceEvent::Error(e.to_string())),
                // EBL header records (timestamp etc.) only appear when
                // the decoder is in EBL mode; this generic NGT-1 device
                // decoder never enables that, so this arm is dead.
                NgtEvent::Header(_) => {}
            }
        }
    }
}

/// Stateless encoder.
pub struct Encoder;

impl DeviceEncoder for Encoder {
    fn init_bytes(&self) -> Vec<u8> {
        encode_startup_ping()
    }

    fn keepalive(&self) -> Option<(Duration, Vec<u8>)> {
        Some((KEEPALIVE_INTERVAL, encode_startup_ping()))
    }

    fn encode_frame(&self, frame: &RawFrame) -> Option<Vec<u8>> {
        if frame.pgn >= ACTISENSE_SYNTHETIC_PGN {
            log::debug!("ngt1: skipping synthetic PGN {}", frame.pgn);
            return None;
        }
        Some(encode_n2k_send_frame(frame))
    }
}
