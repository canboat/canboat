// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Maretron IPG100/200 codec adapter for [`super::run`].
//!
//! Wraps [`canboat_core::format::maretron_ipg`]. Unlike NGT-1 or
//! iKonvert, Maretron has a handshake driven by *reception*: after
//! the writer sends `CONNECT`, the runner waits for the device's
//! `CONNECTED\t<serial>\0` reply before flipping to binary. The
//! decoder emits the post-`CONNECTED` `SET_MODE BINARY` bytes as a
//! [`DeviceEvent::SendBytes`] so the writer thread picks it up
//! without needing a sideband state-sync.

use std::io::{Read, Write};
use std::time::{SystemTime, UNIX_EPOCH};

use canboat_core::RawFrame;
use canboat_core::format::days_to_ymd;
use canboat_core::format::maretron_ipg::{
    MaretronFrame, ParseOutcome, RX_CONNECTED, RX_DETAILED_LICENSES_USED, RX_INSTANCE_DATA,
    RX_LICENSES_USED, RX_NO, RX_SERVER_VERSION, SessionState, build_connect, build_frame,
    build_set_mode_binary, parse,
};

use super::{DeviceDecoder, DeviceEncoder, DeviceEvent, DeviceHandle};

/// Synthetic-PGN marker.
pub const MARETRON_SYNTHETIC_PGN: u32 = 0x40000;

/// Maretron session configuration.
#[derive(Debug, Clone, Default)]
pub struct Config {
    /// Login password. IPG units without security accept the empty
    /// string.
    pub password: String,
    /// Optional fixed timestamp to stamp on every emitted frame —
    /// matches canboat C's `-fixtime` (deterministic-output tests).
    /// When `None`, the host clock is used.
    pub fixtime: Option<String>,
}

/// Start the Maretron reader/writer threads.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
) -> DeviceHandle {
    let password = config.password.clone();
    let decoder = Decoder::new(config.fixtime);
    super::run(decoder, Encoder { password }, reader, writer)
}

pub struct Decoder {
    acc: Vec<u8>,
    state: SessionState,
    fixtime: Option<String>,
}

impl Decoder {
    pub fn new(fixtime: Option<String>) -> Self {
        Self {
            acc: Vec::with_capacity(4096),
            state: SessionState::AwaitHandshake,
            fixtime,
        }
    }
}

impl DeviceDecoder for Decoder {
    fn decode(&mut self, bytes: &[u8], events: &mut Vec<DeviceEvent>) {
        self.acc.extend_from_slice(bytes);
        loop {
            let outcome = parse(&self.acc, self.state);
            match outcome {
                ParseOutcome::NeedMore => return,
                ParseOutcome::Drop { consumed } => {
                    self.acc.drain(..consumed);
                }
                ParseOutcome::Text { line, consumed } => {
                    self.acc.drain(..consumed);
                    self.handle_text(&line, events);
                }
                ParseOutcome::Frame { frame, consumed } => {
                    self.acc.drain(..consumed);
                    events.push(DeviceEvent::Frame(self.to_raw(&frame)));
                }
            }
        }
    }
}

impl Decoder {
    fn handle_text(&mut self, line: &str, events: &mut Vec<DeviceEvent>) {
        let (head, rest) = line.split_once('\t').unwrap_or((line, ""));
        match head {
            RX_CONNECTED => {
                log::info!("maretron: connected (serial {rest})");
                events.push(DeviceEvent::SendBytes(build_set_mode_binary()));
                self.state = SessionState::Streaming;
            }
            RX_NO => {
                events.push(DeviceEvent::Error(
                    "maretron: authentication rejected".to_string(),
                ));
            }
            RX_SERVER_VERSION => log::info!("maretron: server version {rest}"),
            RX_INSTANCE_DATA => log::info!("maretron: instance data {rest}"),
            RX_LICENSES_USED | RX_DETAILED_LICENSES_USED => log::debug!("{head}\t{rest}"),
            _ => log::debug!("maretron control: {head} {rest}"),
        }
    }

    fn to_raw(&self, frame: &MaretronFrame) -> RawFrame {
        let ts = self.fixtime.clone().unwrap_or_else(now_iso_ms);
        frame.to_raw(Some(ts))
    }
}

pub struct Encoder {
    password: String,
}

impl DeviceEncoder for Encoder {
    fn init_bytes(&self) -> Vec<u8> {
        build_connect(&self.password)
    }

    fn encode_frame(&self, frame: &RawFrame) -> Option<Vec<u8>> {
        if frame.pgn >= MARETRON_SYNTHETIC_PGN {
            log::debug!("maretron: skipping synthetic PGN {}", frame.pgn);
            return None;
        }
        match build_frame(frame.pgn, frame.prio, frame.dst, &frame.data) {
            Some(b) => Some(b),
            None => {
                log::warn!(
                    "maretron: cannot encode PGN {} (payload too large)",
                    frame.pgn
                );
                None
            }
        }
    }
}

/// `YYYY-MM-DDTHH:MM:SS.mmm` from the host clock in UTC. Lifted from
/// the old `maretron-ipg` binary so the device crate doesn't pull in
/// `chrono`.
fn now_iso_ms() -> String {
    let dur = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let secs = dur.as_secs() as i64;
    let ms = dur.subsec_millis();
    let days = secs.div_euclid(86_400);
    let day_secs = secs.rem_euclid(86_400) as u32;
    let h = day_secs / 3600;
    let m = (day_secs / 60) % 60;
    let s = day_secs % 60;
    let (y, mo, d) = days_to_ymd(days);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{m:02}:{s:02}.{ms:03}")
}
