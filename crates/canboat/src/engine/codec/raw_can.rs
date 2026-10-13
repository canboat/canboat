// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! What the codecs of the adapters that pass single CAN frames share
//! ([`super::canalyst`], [`super::usb_can_a`]): a frame off the bus into
//! the application's frames, fast-packets joined, and a frame from the
//! application into CAN frames, fast-packets split. A Quick bus carries
//! 11-bit standard frames, NMEA 2000 and J1939 29-bit extended ones; the
//! other kind is skipped.

use std::collections::HashMap;

use smallvec::SmallVec;

use crate::engine::format::{iso11783_compose, iso11783_decompose};
use crate::engine::reassembly::{Reassembled, Reassembler};
use crate::engine::{BusProtocol, FramePacketType, RawFrame, fastpacket, format_iso_ms};

use super::{Event, Refused, SYNTHETIC_PGN_START};

/// One CAN data frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CanFrame {
    /// 29 bits when `extended`, else 11.
    pub id: u32,
    pub extended: bool,
    pub data: SmallVec<[u8; 8]>,
}

/// Hand the application what `id`, `extended` and `data` carry, received
/// at `now_ms` (`None` for a capture, which has no time): a frame, a whole
/// fast-packet once its last frame is in, or nothing for a frame of the
/// other kind.
pub(crate) fn receive(
    protocol: BusProtocol,
    reassembler: &mut Reassembler,
    id: u32,
    extended: bool,
    data: &[u8],
    now_ms: Option<u64>,
    events: &mut Vec<Event>,
) {
    if let Some(now_ms) = now_ms {
        reassembler.tick(now_ms);
    }
    // NMEA 2000 and J1939 are 29-bit, Quick 11-bit: skip the other kind
    // rather than reinterpret its identifier.
    if extended == protocol.standard_frames() {
        return;
    }
    if !extended {
        // A Quick message type has no priority, source or destination, and
        // is never more than one frame.
        let mut frame = RawFrame::new(None, 0, id, 0, 255, data.iter().copied());
        frame.timestamp = now_ms.map(format_iso_ms);
        events.push(Event::Frame(frame));
        return;
    }
    let (prio, pgn, src, dst) = iso11783_decompose(id);
    let mut frame = RawFrame::new(None, prio, pgn, src, dst, data.iter().copied());
    frame.timestamp = now_ms.map(format_iso_ms);
    let pt = protocol.packet_type(frame.pgn);
    match reassembler.push(frame, pt) {
        Reassembled::PassThrough(f) | Reassembled::Complete(f) => events.push(Event::Frame(f)),
        Reassembled::Partial => {}
        Reassembled::Error(e) => events.push(Event::Error(e.to_string())),
    }
}

/// Per-(pgn, src) fast-packet sequence counters, mod 8.
#[derive(Debug, Default)]
pub(crate) struct FastPacketSeq(HashMap<(u32, u8), u8>);

impl FastPacketSeq {
    fn next(&mut self, pgn: u32, src: u8) -> u8 {
        let e = self.0.entry((pgn, src)).or_insert(0);
        let s = *e;
        *e = (*e + 1) & 0x07;
        s
    }
}

/// The CAN frames that carry `frame` on a bus carrying `protocol`.
pub(crate) fn split(
    protocol: BusProtocol,
    seq: &mut FastPacketSeq,
    frame: &RawFrame,
) -> Result<Vec<CanFrame>, Refused> {
    if frame.pgn >= SYNTHETIC_PGN_START {
        return Err(Refused::Synthetic);
    }
    if protocol.standard_frames() {
        // Quick: the message type is the 11-bit identifier, and a message
        // is one frame.
        if frame.pgn > 0x7ff || frame.data.len() > 8 {
            return Err(Refused::TooLarge);
        }
        return Ok(vec![CanFrame {
            id: frame.pgn,
            extended: false,
            data: frame.data.iter().copied().collect(),
        }]);
    }
    let id = iso11783_compose(frame.prio & 7, frame.pgn, frame.src, frame.dst);
    let extended = |data: &[u8]| CanFrame {
        id,
        extended: true,
        data: SmallVec::from_slice(data),
    };
    if protocol.packet_type(frame.pgn) == FramePacketType::Fast {
        let chunks = fastpacket::fragment(seq.next(frame.pgn, frame.src), &frame.data)
            .ok_or(Refused::TooLarge)?;
        Ok(chunks.iter().map(|c| extended(c)).collect())
    } else if frame.data.len() <= 8 {
        Ok(vec![extended(&frame.data)])
    } else {
        // One message is one CAN frame. A longer message needs ISO TP
        // (J1939), which the codecs receive but do not send; canboat's
        // node sends it.
        Err(Refused::TooLarge)
    }
}
