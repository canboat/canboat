// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Async tokio adapters for [`canboat_core`].
//!
//! Each adapter wraps an `AsyncRead` byte source (typically a
//! `tokio_serial::SerialStream`, but anything `AsyncRead + Unpin`
//! works), drives the canboat-core sans-I/O state machines, and emits
//! [`canboat_core::DecodedPgn`] events as a `futures::Stream`.
//!
//! Two byte sources are wired up in v0:
//!
//! - [`Ngt1Stream`] — Actisense NGT-1 binary protocol (DLE/STX/ETX
//!   framing, DLE-stuffing, checksum). Mirrors the sync
//!   `actisense-serial` binary.
//! - [`IkonvertStream`] — Digital Yacht iKonvert line-based ASCII
//!   protocol (`!PDGY,...` frames with Base64 payload).
//!
//! Both wrap the same `canboat_core::Reassembler` + `PgnDatabase` so
//! the *decoding* pipeline is identical to the sync binaries.

pub mod ikonvert;
pub mod ngt1;

pub use ikonvert::IkonvertStream;
pub use ngt1::Ngt1Stream;

use canboat_core::DecodedPgn;
use futures::Stream;

/// Marker trait every concrete adapter implements.
///
/// Merrimac (or any other tokio app) can hold an
/// `Pin<Box<dyn AsyncFrameSource>>` to abstract over the wire format.
pub trait AsyncFrameSource: Stream<Item = DecodedPgn> + Send + Unpin {}

impl<T: Stream<Item = DecodedPgn> + Send + Unpin> AsyncFrameSource for T {}
