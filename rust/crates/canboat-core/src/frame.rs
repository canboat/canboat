// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `RawFrame` — a CAN/N2K frame that has been read off a wire or line.
//!
//! A `RawFrame` may represent either a single 8-byte CAN frame, an
//! already-coalesced fast-packet payload (≤ 223 bytes), or an
//! already-coalesced ISO Transport Protocol payload (≤ 1785 bytes,
//! typically PGN 129540 with a full satellite list from a Neon-style
//! GPS). The `data` field's length distinguishes them. The
//! reassembly state machine in [`crate::reassembly`] turns a stream
//! of single-frame `RawFrame`s into coalesced ones for fast-packet
//! and ISO TP PGNs.

use smallvec::SmallVec;

/// Maximum payload size of a coalesced fast-packet PGN.
///
/// Frame 0 carries 6 payload bytes, frames 1..31 carry 7 bytes each:
/// `6 + 31 * 7 = 223`.
pub const FASTPACKET_MAX_SIZE: usize = 223;

/// Maximum payload size a `RawFrame` will accept — the ISO 11783
/// Transport Protocol ceiling (`255 * 7 = 1785`). Fast-packet payloads
/// stay at `FASTPACKET_MAX_SIZE`; TP-carried payloads (and any input
/// format that ships an upstream-coalesced TP record) can go up to
/// this limit before we truncate. Named so the constant announces
/// its scope (any coalesced payload, not just fast-packet).
pub const RAWFRAME_MAX_SIZE: usize = 255 * 7;

/// The NMEA 2000 / J1939 **global (broadcast) address**, 255. Valid as a
/// destination for broadcast PGNs, never as a *source* on the bus — so it
/// also serves as the "unset: fill in my own claimed address" sentinel on
/// outbound sends (the device / pipeline substitutes the real address).
pub const ADDR_GLOBAL: u8 = 255;

/// The **null address**, 254 — a node that has not (yet) claimed an
/// address. The SocketCAN adapter reports this until its ISO claim wins.
pub const ADDR_NULL: u8 = 254;

/// A CAN/N2K frame.
///
/// Single-frame: `data.len() <= 8`. Coalesced fast-packet:
/// `data.len() <= FASTPACKET_MAX_SIZE`. Coalesced ISO TP:
/// `data.len() <= RAWFRAME_MAX_SIZE`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawFrame {
    /// Free-form timestamp string as captured on the input line (or `None`
    /// if the source did not carry one). Kept verbatim in the frame; the
    /// output formatters canonicalise it to
    /// [`normalize_timestamp`](crate::format::normalize_timestamp) on
    /// emit (comma millis → dot, ISO-8601 UTC `T…Z` for date-bearing
    /// values).
    pub timestamp: Option<String>,
    pub prio: u8,
    pub pgn: u32,
    pub src: u8,
    pub dst: u8,
    pub data: SmallVec<[u8; 8]>,
}

impl RawFrame {
    /// Construct a frame with the given header and payload. Does no
    /// validation beyond capping `data` at [`RAWFRAME_MAX_SIZE`] —
    /// which is the ISO TP ceiling, not fast-packet's, so an
    /// upstream-coalesced TP record (e.g. a > 223-byte PGN 129540
    /// line captured from a gateway that did its own reassembly)
    /// passes through untruncated.
    pub fn new(
        timestamp: Option<String>,
        prio: u8,
        pgn: u32,
        src: u8,
        dst: u8,
        data: impl IntoIterator<Item = u8>,
    ) -> Self {
        let mut payload: SmallVec<[u8; 8]> = data.into_iter().collect();
        if payload.len() > RAWFRAME_MAX_SIZE {
            payload.truncate(RAWFRAME_MAX_SIZE);
        }
        Self {
            timestamp,
            prio,
            pgn,
            src,
            dst,
            data: payload,
        }
    }
}
