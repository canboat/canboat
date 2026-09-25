// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Sans-I/O gateway codecs: the byte-level protocol of each serial / TCP
//! NMEA 2000 gateway, with no port, thread or clock of its own.
//!
//! One stateful object per gateway ([`ngt1::Ngt1`], [`ikonvert::Ikonvert`],
//! [`maretron::Maretron`]) owns everything the protocol needs — the partial
//! message between reads, the init handshake, the transmit list — and is
//! driven entirely by its caller through the [`Codec`] trait:
//!
//! * [`Codec::open`] — the bytes to write once the link is up;
//! * [`Codec::receive`] — bytes read from the gateway in, [`Event`]s out:
//!   bus frames, and bytes the handshake needs written back;
//! * [`Codec::tick`] — lets deadlines advance while the line is quiet;
//! * [`Codec::send`] — a frame for the bus in, the gateway's bytes out, or
//!   a [`Refused`] reason;
//! * [`Codec::close`] — the goodbye (an iKonvert goes off the bus).
//!
//! The caller supplies the time (`now_ms`, Unix milliseconds) wherever a
//! codec needs one, so the same code runs under the threaded `bus::open_*`
//! runners, in an async task, or in a browser over WebSerial.

pub mod ikonvert;
pub mod maretron;
pub mod ngt1;
pub mod ngt1_tx_list;

use std::time::Duration;

use crate::engine::RawFrame;

/// What a [`Codec`] hands back as bytes arrive.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// A frame received from the bus — or one the codec synthesises about
    /// the gateway itself, such as its network status (PGN 262400).
    Frame(RawFrame),
    /// Bytes to write to the gateway now: the next step of a handshake
    /// that is driven by what the gateway sends.
    Send(Vec<u8>),
    /// A framing or protocol error. The codec carries on.
    Error(String),
}

/// Why [`Codec::send`] did not encode a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refused {
    /// A canboat-internal PGN (`>= 0x40000`), which never goes on the bus.
    Synthetic,
    /// The application named a transmit list and this PGN is not on it;
    /// the gateway would not transmit it.
    NotOnTxList,
    /// The payload does not fit the gateway's framing.
    TooLarge,
}

impl std::fmt::Display for Refused {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Synthetic => "synthetic PGN, never sent on the bus",
            Self::NotOnTxList => "PGN is not on the named transmit list",
            Self::TooLarge => "payload too large for the gateway",
        })
    }
}

impl std::error::Error for Refused {}

/// A gateway's protocol, driven by its caller. See the [module
/// docs](self).
pub trait Codec {
    /// Bytes to write as soon as the link is up (NGT-1 startup ping,
    /// iKonvert `N2NET_OFFLINE`, Maretron `CONNECT`). Empty when the
    /// gateway needs none.
    fn open(&mut self) -> Vec<u8> {
        Vec::new()
    }

    /// Feed bytes read from the gateway; what they complete is pushed onto
    /// `events`. `now_ms` is the current time in Unix milliseconds.
    fn receive(&mut self, bytes: &[u8], now_ms: u64, events: &mut Vec<Event>);

    /// Advance the codec's deadlines with nothing received. Call it
    /// periodically — the threaded runners do every 250 ms of silence.
    fn tick(&mut self, _now_ms: u64, _events: &mut Vec<Event>) {}

    /// Encode `frame` for the gateway.
    fn send(&mut self, frame: &RawFrame) -> Result<Vec<u8>, Refused>;

    /// Bytes to write when the application closes the link on purpose.
    /// Empty when the gateway needs nothing.
    fn close(&mut self) -> Vec<u8> {
        Vec::new()
    }

    /// Bytes to write every `interval` the link carries nothing else.
    /// `None` when the gateway needs no keepalive.
    fn keepalive(&self) -> Option<(Duration, Vec<u8>)> {
        None
    }
}

/// Synthetic-PGN marker: frames at or above it are canboat-internal and
/// never reach the bus.
pub const SYNTHETIC_PGN_START: u32 = 0x40000;
