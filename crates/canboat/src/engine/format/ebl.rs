// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense `.ebl` binary logs, as the Actisense SDK specifies them:
//! [EBL File Format](https://github.com/Actisense/SDK/blob/main/docs/FileFormats/ebl/ebl-file-format.md)
//! and [Writing an EBL Reader](https://github.com/Actisense/SDK/blob/main/docs/FileFormats/ebl/writing-an-ebl-reader.md).
//!
//! An `.ebl` file is the byte stream of a gateway link with metatags
//! between its bytes: `ESC SOH <tag> <data> ESC LF`. The two are escaped in
//! two independent layers:
//!
//! * the **EBL** layer doubles every `ESC` (`0x1b`), in the stream and in
//!   the metatags alike;
//! * underneath, the stream is BST messages in BDTP framing, `DLE STX …
//!   DLE ETX`, which doubles every `DLE` (`0x10`) inside a message.
//!
//! [`EblDecoder`] removes the EBL layer and hands the stream to a
//! [`BdtpDecoder`], so a metatag can sit anywhere, even inside a message.
//! A `TimeUTC` metatag dates everything after it, until the next one.
//!
//! The encoder writes, per frame, a `TimeUTC` metatag and an
//! `N2K_MSG_RECEIVED` (BST-93) message; [`encode_preamble`] starts a file
//! with the `TimeUTC` and `Version` metatags the SDK expects first.

use super::bst::BdtpDecoder;
use crate::engine::format::ngt1::{
    DLE, EBL_TIMESTAMP, ESC, ETX, FILETIME_TO_UNIX_MS, LF, N2K_MSG_RECEIVED, SOH, STX,
};
use crate::engine::format::timestamp::to_unix_ms;
use crate::engine::frame::RawFrame;

/// The `Version` metatag.
const EBL_VERSION: u8 = 0x01;
/// The `Description` metatag.
const EBL_DESCRIPTION: u8 = 0x02;
/// The `DirectionMarker` metatag.
const EBL_DIRECTION: u8 = 0x05;
/// The `BSTRawFrame` metatag: one BST message, without BDTP framing.
const EBL_BST_RAW_FRAME: u8 = 0x07;

/// The EBL format version canboat reads and writes: v1.002.
pub const EBL_FORMAT_VERSION: u32 = 1002;

/// The largest metatag the SDK writes, after escaping. Longer ones are
/// read, but warned about.
const EBL_TAG_CAPACITY: usize = 1800;

/// Most bytes kept for one metatag, so a file that never ends one does not
/// grow the buffer without limit.
const MAX_TAG: usize = 64 * 1024;

/// Append one metatag, `ESC SOH <tag> <data> ESC LF`, to `out`.
fn push_tag(out: &mut Vec<u8>, tag: u8, data: &[u8]) {
    out.push(ESC);
    out.push(SOH);
    push_header_byte(out, tag);
    for &b in data {
        push_header_byte(out, b);
    }
    out.push(ESC);
    out.push(LF);
}

/// The FILETIME (100-ns ticks since 1601) of `unix_ms`.
fn filetime(unix_ms: u64) -> u64 {
    (unix_ms + FILETIME_TO_UNIX_MS).saturating_mul(10_000)
}

/// Append what an `.ebl` file starts with, as the SDK has it: a `TimeUTC`
/// metatag for `unix_ms`, then a `Version` metatag (v1.002).
pub fn encode_preamble(unix_ms: u64, out: &mut Vec<u8>) {
    push_tag(out, EBL_TIMESTAMP, &filetime(unix_ms).to_le_bytes());
    push_tag(out, EBL_VERSION, &EBL_FORMAT_VERSION.to_le_bytes());
}

/// The time of `frame` in Unix milliseconds; a frame without a parseable
/// absolute timestamp lands on the Unix epoch.
pub fn frame_unix_ms(frame: &RawFrame) -> u64 {
    frame
        .timestamp
        .as_deref()
        .and_then(to_unix_ms)
        .unwrap_or(0)
        .max(0) as u64
}

/// Encode `frame` as one `.ebl` record pair — a `TimeUTC` metatag plus an
/// `N2K_MSG_RECEIVED` message — appended to `out`. Round-trips through
/// [`EblDecoder`].
pub fn encode_frame(frame: &RawFrame, out: &mut Vec<u8>) {
    push_tag(
        out,
        EBL_TIMESTAMP,
        &filetime(frame_unix_ms(frame)).to_le_bytes(),
    );

    // N2K_MSG_RECEIVED message: DLE STX <cmd len payload csum> DLE ETX,
    // dual-stuffed. Payload header is `prio pgn[3 LE] dst src ts[4 LE]
    // dlen`, matching `NgtMessage::to_raw_frame`. The per-message NGT
    // timestamp (ms since gateway startup) is left 0 — the real instant
    // rides in the EBL header above.
    let mut payload = Vec::with_capacity(11 + frame.data.len());
    payload.push(frame.prio);
    payload.push((frame.pgn & 0xff) as u8);
    payload.push(((frame.pgn >> 8) & 0xff) as u8);
    payload.push(((frame.pgn >> 16) & 0xff) as u8);
    payload.push(frame.dst);
    payload.push(frame.src);
    payload.extend_from_slice(&0u32.to_le_bytes());
    payload.push(frame.data.len() as u8);
    payload.extend_from_slice(&frame.data);
    encode_message(N2K_MSG_RECEIVED, &payload, out);
}

/// Append one metatag byte, escaping a literal `ESC` as `ESC ESC`. `DLE`
/// bytes are literal inside a metatag.
fn push_header_byte(out: &mut Vec<u8>, b: u8) {
    out.push(b);
    if b == ESC {
        out.push(ESC);
    }
}

/// Append one message byte, escaping `DLE`→`DLE DLE` and `ESC`→`ESC ESC`.
fn push_message_byte(out: &mut Vec<u8>, b: u8) {
    out.push(b);
    if b == DLE {
        out.push(DLE);
    } else if b == ESC {
        out.push(ESC);
    }
}

/// Frame `(cmd, payload)` as `DLE STX … DLE ETX` with the 8-bit checksum
/// that makes `(cmd + len + payload + csum) ≡ 0 (mod 256)`, dual-stuffed.
/// Mirrors [`crate::engine::format::ngt1::encode_ngt_message`] but escapes `ESC`
/// too, as EBL framing requires.
fn encode_message(cmd: u8, payload: &[u8], out: &mut Vec<u8>) {
    out.push(DLE);
    out.push(STX);
    push_message_byte(out, cmd);
    push_message_byte(out, payload.len() as u8);
    let mut sum = cmd.wrapping_add(payload.len() as u8);
    for &b in payload {
        push_message_byte(out, b);
        sum = sum.wrapping_add(b);
    }
    push_message_byte(out, 0u8.wrapping_sub(sum));
    out.push(DLE);
    out.push(ETX);
}

/// The direction a `DirectionMarker` metatag gives the data after it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// From the device to the host: what it received.
    Rx,
    /// From the host to the device.
    Tx,
}

/// What [`EblDecoder`] finds in an `.ebl` stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EblEvent {
    /// A BST message, unframed and un-stuffed: from the BDTP stream, or a
    /// `BSTRawFrame` metatag (`raw`), which may lack its checksum.
    Message { bytes: Vec<u8>, raw: bool },
    /// `TimeUTC`: the time of what follows, in Unix milliseconds.
    Time(u64),
    /// `Version`: the EBL format version × 1000.
    Version(u32),
    /// `DirectionMarker`: the direction of what follows.
    Direction(Direction),
    /// `Description`: free text about the file.
    Description(String),
    /// A metatag canboat does not use (`ElementType`, or one not yet
    /// defined): its tag ID and data.
    Unknown { tag: u8, data: Vec<u8> },
    /// Damage the decoder stepped over: a bad escape, a metatag cut short or
    /// too long, a BDTP frame given up on.
    Warning(String),
}

/// The decoder's place in the EBL layer.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
enum EblState {
    /// Passing stream bytes to the BDTP layer.
    #[default]
    Stream,
    /// After an `ESC` in the stream.
    StreamEsc,
    /// Inside a metatag.
    Tag,
    /// After an `ESC` inside a metatag.
    TagEsc,
}

/// Reads an `.ebl` stream: feed it bytes, collect [`EblEvent`]s. Tolerant,
/// as the SDK asks: damage is reported as a [`EblEvent::Warning`] and the
/// records around it are still read.
#[derive(Debug, Default)]
pub struct EblDecoder {
    state: EblState,
    tag: Vec<u8>,
    bdtp: BdtpDecoder,
}

impl EblDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed `bytes`; push what they complete onto `events`.
    pub fn push_bytes(&mut self, bytes: &[u8], events: &mut Vec<EblEvent>) {
        for &b in bytes {
            self.push_byte(b, events);
        }
    }

    /// Note the end of the input: a metatag left open is reported.
    pub fn finish(&mut self, events: &mut Vec<EblEvent>) {
        if matches!(self.state, EblState::Tag | EblState::TagEsc) {
            events.push(EblEvent::Warning(
                "the file ends inside a metatag".to_string(),
            ));
        }
        self.state = EblState::Stream;
        self.tag.clear();
    }

    fn push_byte(&mut self, b: u8, events: &mut Vec<EblEvent>) {
        match (self.state, b) {
            (EblState::Stream, ESC) => self.state = EblState::StreamEsc,
            (EblState::Stream, _) => self.stream(b, events),
            (EblState::StreamEsc, ESC) => {
                self.state = EblState::Stream;
                self.stream(ESC, events);
            }
            (EblState::StreamEsc, SOH) | (EblState::TagEsc, SOH) => {
                if self.state == EblState::TagEsc {
                    events.push(EblEvent::Warning(
                        "a metatag starts inside another; the first is dropped".to_string(),
                    ));
                }
                self.tag.clear();
                self.state = EblState::Tag;
            }
            (EblState::StreamEsc, other) => {
                self.state = EblState::Stream;
                events.push(EblEvent::Warning(format!(
                    "ESC {other:#04x} in the stream; skipped"
                )));
            }
            (EblState::Tag, ESC) => self.state = EblState::TagEsc,
            (EblState::Tag, _) => {
                if self.tag.len() < MAX_TAG {
                    self.tag.push(b);
                }
            }
            (EblState::TagEsc, ESC) => {
                self.state = EblState::Tag;
                if self.tag.len() < MAX_TAG {
                    self.tag.push(ESC);
                }
            }
            (EblState::TagEsc, LF) => {
                self.state = EblState::Stream;
                let tag = std::mem::take(&mut self.tag);
                self.metatag(tag, events);
            }
            (EblState::TagEsc, other) => {
                self.state = EblState::Stream;
                self.tag.clear();
                events.push(EblEvent::Warning(format!(
                    "ESC {other:#04x} inside a metatag; the metatag is dropped"
                )));
            }
        }
    }

    /// A stream byte, for the BDTP layer.
    fn stream(&mut self, b: u8, events: &mut Vec<EblEvent>) {
        let dropped = self.bdtp.dropped;
        if let Some(bytes) = self.bdtp.push_byte(b) {
            events.push(EblEvent::Message { bytes, raw: false });
        }
        if self.bdtp.dropped != dropped {
            events.push(EblEvent::Warning(
                "a damaged BDTP frame in the stream; skipped".to_string(),
            ));
        }
    }

    fn metatag(&mut self, tag: Vec<u8>, events: &mut Vec<EblEvent>) {
        let Some((&id, data)) = tag.split_first() else {
            events.push(EblEvent::Warning("an empty metatag".to_string()));
            return;
        };
        if tag.len() > EBL_TAG_CAPACITY {
            events.push(EblEvent::Warning(format!(
                "metatag {id:#04x} is {} bytes, more than the {EBL_TAG_CAPACITY} the format allows",
                tag.len()
            )));
        }
        let event = match (id, data) {
            (EBL_TIMESTAMP, t) if t.len() >= 8 => {
                let ticks = u64::from_le_bytes([t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]]);
                EblEvent::Time((ticks / 10_000).saturating_sub(FILETIME_TO_UNIX_MS))
            }
            (EBL_VERSION, [a, b, c, d, ..]) => {
                EblEvent::Version(u32::from_le_bytes([*a, *b, *c, *d]))
            }
            (EBL_DIRECTION, [0, ..]) => EblEvent::Direction(Direction::Rx),
            (EBL_DIRECTION, [1, ..]) => EblEvent::Direction(Direction::Tx),
            (EBL_DESCRIPTION, text) => EblEvent::Description(String::from_utf8_lossy(text).into()),
            (EBL_BST_RAW_FRAME, bytes) if bytes.len() >= 2 => EblEvent::Message {
                bytes: bytes.to_vec(),
                raw: true,
            },
            (EBL_TIMESTAMP | EBL_VERSION | EBL_DIRECTION | EBL_BST_RAW_FRAME, _) => {
                EblEvent::Warning(format!(
                    "metatag {id:#04x} with bad data {data:02x?}; skipped"
                ))
            }
            _ => EblEvent::Unknown {
                tag: id,
                data: data.to_vec(),
            },
        };
        events.push(event);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::format::bst::to_raw_frame;

    fn frame(data: &[u8]) -> RawFrame {
        RawFrame {
            timestamp: Some("2025-04-25T17:05:21.993Z".into()),
            prio: 3,
            pgn: 127251,
            src: 0x17,
            dst: 0xff,
            data: data.iter().copied().collect(),
        }
    }

    fn decode(wire: &[u8]) -> Vec<EblEvent> {
        let mut d = EblDecoder::new();
        let mut events = Vec::new();
        d.push_bytes(wire, &mut events);
        d.finish(&mut events);
        events
    }

    fn hex(s: &str) -> Vec<u8> {
        s.split_whitespace()
            .map(|b| u8::from_str_radix(b, 16).unwrap())
            .collect()
    }

    #[test]
    fn encode_round_trips() {
        // Include 0x10 (DLE) and 0x1b (ESC) in the data so both escape
        // paths are exercised.
        let f = frame(&[0x00, 0x10, 0x1b, 0xff, 0x10, 0x1b, 0x25, 0x02]);
        let mut wire = Vec::new();
        encode_preamble(1_745_600_721_000, &mut wire);
        encode_frame(&f, &mut wire);

        let events = decode(&wire);
        let [
            EblEvent::Time(_),
            EblEvent::Version(EBL_FORMAT_VERSION),
            EblEvent::Time(ms),
            EblEvent::Message { bytes, raw: false },
        ] = &events[..]
        else {
            panic!("events: {events:?}");
        };
        assert_eq!(*ms, 1_745_600_721_993);
        let back = to_raw_frame(bytes, false).expect("received frame");
        assert_eq!(
            (back.prio, back.pgn, back.src, back.dst),
            (f.prio, f.pgn, f.src, f.dst)
        );
        assert_eq!(back.data.as_slice(), f.data.as_slice());
    }

    #[test]
    fn encode_without_timestamp_uses_epoch() {
        let mut f = frame(&[1, 2, 3]);
        f.timestamp = None;
        let mut wire = Vec::new();
        encode_frame(&f, &mut wire);
        assert_eq!(decode(&wire)[0], EblEvent::Time(0));
    }

    /// A `TimeUTC` metatag from the first record of `samples/actisense1.ebl`,
    /// captured on 2025-04-25 17:05:21.993Z.
    #[test]
    fn time_utc_from_a_real_file() {
        let wire = hex("1b 01 03 90 e3 c8 3a 04 b6 db 01 1b 0a");
        assert_eq!(decode(&wire), [EblEvent::Time(1_745_600_721_993)]);
    }

    /// The SDK's worked example of a classic capture. Its BST-93 checksum,
    /// `BC`, is wrong (the sum is not zero); `40` is right.
    #[test]
    fn sdk_classic_capture() {
        let wire = hex("1B 01 03 40 1B 1B E1 60 CE 40 DA 01 1B 0A \
             1B 01 01 EA 03 00 00 1B 0A \
             1B 01 03 80 24 E2 60 CE 40 DA 01 1B 0A \
             10 02 93 0C 02 00 EE 01 FF EE 00 00 00 00 01 42 40 10 03");
        let events = decode(&wire);
        let ticks = |t: u64| (t / 10_000).saturating_sub(FILETIME_TO_UNIX_MS);
        assert_eq!(events[0], EblEvent::Time(ticks(0x01DA40CE60E11B40)));
        assert_eq!(events[1], EblEvent::Version(1002));
        assert_eq!(events[2], EblEvent::Time(ticks(0x01DA40CE60E22480)));
        let EblEvent::Message { bytes, raw: false } = &events[3] else {
            panic!("{events:?}");
        };
        let f = to_raw_frame(bytes, false).unwrap();
        assert_eq!(
            (f.pgn, f.src, f.data.as_slice()),
            (126464, 0xee, &[0x42][..])
        );
        assert_eq!(events.len(), 4);
    }

    /// The SDK's worked example of a wire trace: direction markers, and a
    /// message in a BSTRawFrame metatag.
    #[test]
    fn sdk_wire_trace() {
        let wire = hex("1B 01 03 40 1B 1B E1 60 CE 40 DA 01 1B 0A \
             1B 01 01 EA 03 00 00 1B 0A \
             1B 01 03 80 24 E2 60 CE 40 DA 01 1B 0A \
             1B 01 05 01 1B 0A \
             10 02 94 02 03 04 F9 10 03 \
             1B 01 03 00 30 E5 60 CE 40 DA 01 1B 0A \
             1B 01 05 00 1B 0A \
             1B 01 07 93 0C 02 00 EE 01 FF EE 00 00 00 00 01 42 1B 0A");
        let events = decode(&wire);
        assert_eq!(events[3], EblEvent::Direction(Direction::Tx));
        assert!(matches!(&events[4], EblEvent::Message { bytes, raw: false } if bytes[0] == 0x94));
        assert_eq!(events[6], EblEvent::Direction(Direction::Rx));
        let EblEvent::Message { bytes, raw: true } = &events[7] else {
            panic!("{events:?}");
        };
        // The example's BSTRawFrame has no checksum.
        let f = to_raw_frame(bytes, true).unwrap();
        assert_eq!((f.pgn, f.src), (126464, 0xee));
        assert_eq!(events.len(), 8);
    }

    /// A metatag can sit inside a message: the writer forces a timestamp
    /// after 500 bytes without one. The message survives.
    #[test]
    fn a_metatag_inside_a_message_leaves_it_whole() {
        let f = frame(&[1, 2, 3, 4, 5, 6, 7, 8]);
        let mut message = Vec::new();
        encode_frame(&f, &mut message);
        let start = message.iter().position(|&b| b == DLE).unwrap();
        let mut wire = message[..start + 6].to_vec();
        push_tag(&mut wire, EBL_TIMESTAMP, &filetime(1000).to_le_bytes());
        wire.extend_from_slice(&message[start + 6..]);
        let events = decode(&wire);
        assert!(events.contains(&EblEvent::Time(1000)), "{events:?}");
        let EblEvent::Message { bytes, .. } = events.last().unwrap() else {
            panic!("{events:?}");
        };
        assert_eq!(
            to_raw_frame(bytes, false).unwrap().data.as_slice(),
            &f.data[..]
        );
    }

    #[test]
    fn damage_is_stepped_over() {
        let mut f = Vec::new();
        encode_frame(&frame(&[9]), &mut f);
        // A bad escape in the stream, a bad escape inside a metatag, an
        // unknown and an empty tag, a bad direction; each followed by a
        // good record.
        let mut wire = vec![ESC, 0x42];
        wire.extend(&f);
        wire.extend([ESC, SOH, EBL_TIMESTAMP, 1, ESC, 0x42]);
        wire.extend(&f);
        wire.extend([ESC, SOH, 0x08, 1, 2, ESC, LF, ESC, SOH, ESC, LF]);
        wire.extend([ESC, SOH, EBL_DIRECTION, 7, ESC, LF]);
        wire.extend(&f);
        wire.extend([ESC, SOH, EBL_TIMESTAMP, 1, 2]);
        let events = decode(&wire);
        let messages = events
            .iter()
            .filter(|e| matches!(e, EblEvent::Message { .. }))
            .count();
        assert_eq!(messages, 3, "{events:?}");
        assert!(events.contains(&EblEvent::Unknown {
            tag: 0x08,
            data: vec![1, 2]
        }));
        let warnings = events
            .iter()
            .filter(|e| matches!(e, EblEvent::Warning(_)))
            .count();
        assert_eq!(warnings, 5, "{events:?}");
    }
}
