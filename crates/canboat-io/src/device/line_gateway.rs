// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Line-based network gateway codecs: Yacht Devices RAW (YDWG-02 /
//! YDEN, also the NavLink2's RAW port) and Actisense N2K ASCII (W2K-1).
//!
//! Both devices speak newline-delimited text over a TCP (or, receive-
//! only, UDP) socket. The decoder half accumulates a partial line
//! across reads, parses each complete line with the matching
//! [`canboat_core::format`] parser and — for RAW, whose lines are
//! individual CAN frames — reassembles fast-packets before emitting.
//! The encoder half writes the device's transmit shape: complete
//! `A…` lines for N2K ASCII, bare `<CANID> <bytes…>` lines for RAW
//! with fast-packet payloads fragmented per ISO 11783-3.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::io::{Read, Write};
use std::sync::Mutex;

use canboat_core::format::{actisense_ascii, iso11783_compose, ydwg02};
use canboat_core::reassembly::{Reassembled, Reassembler};
use canboat_core::{FramePacketType, RawFrame};

use super::{DeviceDecoder, DeviceEncoder, DeviceEvent, DeviceHandle};
use crate::fastpacket;

/// Which line protocol the socket speaks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Protocol {
    /// Yacht Devices RAW: `HH:MM:SS.mmm R <CANID> <bytes…>` per CAN
    /// frame received; transmit lines are `<CANID> <bytes…>`.
    YdwgRaw,
    /// Actisense N2K ASCII: `A…` lines carrying complete (coalesced)
    /// messages in both directions.
    N2kAscii,
}

pub struct Decoder {
    protocol: Protocol,
    acc: String,
    reassembler: Reassembler,
}

impl Decoder {
    pub fn new(protocol: Protocol) -> Self {
        Self {
            protocol,
            acc: String::with_capacity(256),
            reassembler: Reassembler::new(),
        }
    }
}

impl DeviceDecoder for Decoder {
    fn decode(&mut self, bytes: &[u8], events: &mut Vec<DeviceEvent>) {
        self.acc.push_str(&String::from_utf8_lossy(bytes));
        while let Some(pos) = self.acc.find('\n') {
            let line: String = self.acc[..pos].trim().to_string();
            self.acc.drain(..=pos);
            if line.is_empty() {
                continue;
            }
            match self.protocol {
                Protocol::YdwgRaw => {
                    // The gateway echoes our own transmissions with a
                    // `T` direction marker — skip them, we already
                    // emitted the coalesced frame on send.
                    if line.split_whitespace().nth(1) == Some("T") {
                        continue;
                    }
                    match ydwg02::parse_line(&line) {
                        Ok(frame) => {
                            let pt = fastpacket::packet_type(frame.pgn);
                            match self.reassembler.push(frame, pt) {
                                Reassembled::PassThrough(f) | Reassembled::Complete(f) => {
                                    events.push(DeviceEvent::Frame(f))
                                }
                                Reassembled::Partial => {}
                                Reassembled::Error(e) => {
                                    events.push(DeviceEvent::Error(e.to_string()))
                                }
                            }
                        }
                        Err(e) => events.push(DeviceEvent::Error(format!("{line}: {e}"))),
                    }
                }
                Protocol::N2kAscii => match actisense_ascii::parse_line(&line) {
                    Ok(frame) => events.push(DeviceEvent::Frame(frame)),
                    Err(e) => events.push(DeviceEvent::Error(format!("{line}: {e}"))),
                },
            }
        }
    }
}

pub struct Encoder {
    protocol: Protocol,
    /// Per-(pgn, src) fast-packet TX sequence counters, mod 8.
    seq: Mutex<HashMap<(u32, u8), u8>>,
}

impl Encoder {
    pub fn new(protocol: Protocol) -> Self {
        Self {
            protocol,
            seq: Mutex::new(HashMap::new()),
        }
    }

    fn next_seq(&self, pgn: u32, src: u8) -> u8 {
        let mut m = self.seq.lock().expect("seq mutex");
        let e = m.entry((pgn, src)).or_insert(0);
        let s = *e;
        *e = (*e + 1) & 0x07;
        s
    }
}

impl DeviceEncoder for Encoder {
    fn encode_frame(&self, frame: &RawFrame) -> Option<Vec<u8>> {
        // Synthetic canboat-internal PGNs never go on the wire.
        if frame.pgn >= fastpacket::CANBOAT_PGN_START {
            return None;
        }
        let mut out = String::with_capacity(64);
        match self.protocol {
            Protocol::N2kAscii => {
                actisense_ascii::write_line(&mut out, frame).ok()?;
                out.push_str("\r\n");
            }
            Protocol::YdwgRaw => {
                let canid = iso11783_compose(frame.prio, frame.pgn, frame.src, frame.dst);
                let write_one = |out: &mut String, data: &[u8]| {
                    let _ = write!(out, "{canid:08X}");
                    for b in data {
                        let _ = write!(out, " {b:02X}");
                    }
                    out.push_str("\r\n");
                };
                if fastpacket::packet_type(frame.pgn) == FramePacketType::Fast {
                    let seq = self.next_seq(frame.pgn, frame.src);
                    for chunk in fastpacket::fragment(seq, &frame.data) {
                        write_one(&mut out, &chunk);
                    }
                } else {
                    write_one(&mut out, &frame.data);
                }
            }
        }
        Some(out.into_bytes())
    }
}

/// Start the gateway reader/writer threads over any byte transport.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    protocol: Protocol,
) -> DeviceHandle {
    super::run(
        Decoder::new(protocol),
        Encoder::new(protocol),
        reader,
        writer,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(pgn: u32, data: &[u8]) -> RawFrame {
        RawFrame::new(None, 3, pgn, 7, 255, data.iter().copied())
    }

    fn decode_all(dec: &mut Decoder, text: &str) -> Vec<RawFrame> {
        let mut events = Vec::new();
        dec.decode(text.as_bytes(), &mut events);
        events
            .into_iter()
            .filter_map(|e| match e {
                DeviceEvent::Frame(f) => Some(f),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn raw_single_frame_round_trips() {
        let f = frame(130306, &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]);
        let enc = Encoder::new(Protocol::YdwgRaw);
        let tx = String::from_utf8(enc.encode_frame(&f).unwrap()).unwrap();
        // TX shape: bare `<CANID> <bytes…>`; the gateway's RX lines add
        // a time-of-day and direction, which the decoder expects.
        let rx: String = tx
            .lines()
            .map(|l| format!("00:00:01.000 R {l}\r\n"))
            .collect();
        let mut dec = Decoder::new(Protocol::YdwgRaw);
        let out = decode_all(&mut dec, &rx);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].pgn, 130306);
        assert_eq!(out[0].data.as_slice(), f.data.as_slice());
    }

    #[test]
    fn raw_fast_packet_fragments_and_reassembles() {
        // 129029 GNSS Position Data: 43-byte fast-packet payload.
        let payload: Vec<u8> = (0u8..43).collect();
        let f = frame(129029, &payload);
        let enc = Encoder::new(Protocol::YdwgRaw);
        let tx = String::from_utf8(enc.encode_frame(&f).unwrap()).unwrap();
        assert_eq!(tx.lines().count(), 7, "43 bytes -> 1x6 + 6x7 chunk frames");
        let rx: String = tx
            .lines()
            .map(|l| format!("00:00:01.000 R {l}\r\n"))
            .collect();
        let mut dec = Decoder::new(Protocol::YdwgRaw);
        let out = decode_all(&mut dec, &rx);
        assert_eq!(out.len(), 1, "fragments must reassemble to one frame");
        assert_eq!(out[0].pgn, 129029);
        assert_eq!(out[0].data.as_slice(), payload.as_slice());
    }

    #[test]
    fn raw_skips_transmit_echoes_and_partial_lines() {
        let mut dec = Decoder::new(Protocol::YdwgRaw);
        // A transmit echo is skipped; a split line is buffered across
        // reads and completes on the second push.
        let out = decode_all(
            &mut dec,
            "00:00:01.000 T 0DF80523 01 02 03 04 05 06 07 08\r\n00:00:01.010 R 09F1",
        );
        assert!(out.is_empty());
        let out = decode_all(&mut dec, "1223 01 02 03 04 05 06 07 08\r\n");
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].pgn, 127250);
    }

    #[test]
    fn ascii_lines_pass_complete_frames() {
        // Round-trip a coalesced frame through the N2K ASCII writer and
        // parser: W2K lines carry complete messages, no reassembly.
        let payload: Vec<u8> = (0u8..43).collect();
        let f = frame(129029, &payload);
        let enc = Encoder::new(Protocol::N2kAscii);
        let tx = String::from_utf8(enc.encode_frame(&f).unwrap()).unwrap();
        assert_eq!(tx.lines().count(), 1);
        let mut dec = Decoder::new(Protocol::N2kAscii);
        let out = decode_all(&mut dec, &tx);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].pgn, 129029);
        assert_eq!(out[0].data.as_slice(), payload.as_slice());
    }

    #[test]
    fn synthetic_pgns_are_dropped() {
        let enc = Encoder::new(Protocol::YdwgRaw);
        assert!(enc.encode_frame(&frame(0x40200, &[1, 2, 3])).is_none());
    }
}
