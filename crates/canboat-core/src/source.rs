// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Frame sources and the decode convenience.
//!
//! [`FrameSource`] is the bring-your-own-transport seam: implement it over
//! any byte source — a CAN driver, a serial gateway, a file, an `mpsc`
//! channel — to feed [`RawFrame`]s into the decoder without depending on
//! `canboat-io`. [`Decoder`] wraps a source with fast-packet reassembly and
//! the schema, yielding a stream of [`DecodedPgn`]s.
//!
//! This is the one place `canboat-core` names [`std::io::Result`]: the trait
//! only *describes* a read and performs no I/O itself. The concrete readers
//! (serial, file, `.pcap`/`.nif` container) live in `canboat-io`.

use std::io;

use crate::{
    DecodedPgn, FramePacketType, PacketType, PgnDatabase, RawFrame, Reassembled, Reassembler,
};

/// Pull-based source of [`RawFrame`]s.
///
/// [`read_frame`](FrameSource::read_frame) returns the next frame, `Ok(None)`
/// at end-of-stream (file EOF, a closed device channel), and `Err` only on a
/// genuine I/O failure that should stop the stream — malformed input is the
/// source's own problem to log and skip.
///
/// Bring your own transport by implementing this over your CAN driver:
///
/// ```no_run
/// use canboat_core::{FrameSource, RawFrame};
/// struct MyCan(/* your driver handle */);
/// impl FrameSource for MyCan {
///     fn read_frame(&mut self) -> std::io::Result<Option<RawFrame>> {
///         // read one CAN frame from your driver, map it to a RawFrame
///         Ok(None)
///     }
/// }
/// ```
pub trait FrameSource {
    /// Read the next frame; `Ok(None)` at end-of-stream.
    fn read_frame(&mut self) -> io::Result<Option<RawFrame>>;
}

/// An `mpsc` channel of frames is a [`FrameSource`]: `recv` waits for the
/// next frame and a disconnected sender (the producer thread ended) is
/// end-of-stream. This makes any live device or worker thread that emits
/// `RawFrame`s usable anywhere a source is expected.
impl FrameSource for std::sync::mpsc::Receiver<RawFrame> {
    fn read_frame(&mut self) -> io::Result<Option<RawFrame>> {
        match self.recv() {
            Ok(frame) => Ok(Some(frame)),
            Err(std::sync::mpsc::RecvError) => Ok(None),
        }
    }
}

/// Turns a [`FrameSource`] into an iterator of [`DecodedPgn`]s: it applies
/// fast-packet reassembly (single frames pass through, fast-packet fragments
/// accumulate until complete) and decodes each assembled message against the
/// schema. Reassembly and decode errors are logged and skipped; only a
/// source I/O error surfaces as `Err`.
///
/// This handles the standard live-bus case — frames of ≤ 8 payload bytes that
/// need reassembly. It does not apply the analyzer's "pre-coalesced" heuristic
/// for replaying capture files that already carry whole fast-packet messages
/// on one line; that is an input-format concern of the `convert`/`analyzer`
/// tooling, not of a transport feeding live frames.
///
/// ```no_run
/// use canboat_core::{Decoder, PgnDatabase, Units};
/// # fn go<S: canboat_core::FrameSource>(source: S) {
/// let db = PgnDatabase::embedded(Units::Metric);
/// for decoded in Decoder::new(source, db) {
///     let decoded = decoded.expect("source I/O");
///     println!("{} from {}", decoded.pgn, decoded.src);
/// }
/// # }
/// ```
pub struct Decoder<S> {
    source: S,
    db: &'static PgnDatabase,
    reasm: Reassembler,
}

impl<S: FrameSource> Decoder<S> {
    /// Wrap `source`, decoding against `db` (from [`PgnDatabase::embedded`]).
    pub fn new(source: S, db: &'static PgnDatabase) -> Self {
        Self {
            source,
            db,
            reasm: Reassembler::new(),
        }
    }

    /// The unit system decoded values are presented in — the `db`'s units.
    pub fn units(&self) -> crate::Units {
        self.db.units()
    }
}

impl<S: FrameSource> Iterator for Decoder<S> {
    type Item = io::Result<DecodedPgn>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let frame = match self.source.read_frame() {
                Ok(Some(f)) => f,
                Ok(None) => return None,
                Err(e) => return Some(Err(e)),
            };
            let packet_type = self
                .db
                .first_pgn(frame.pgn)
                .or_else(|| self.db.fallback_pgn(frame.pgn))
                .map(|p| match p.packet_type {
                    PacketType::Fast => FramePacketType::Fast,
                    PacketType::Single => FramePacketType::Single,
                    _ => FramePacketType::Other,
                })
                .unwrap_or(FramePacketType::Other);
            let assembled = match self.reasm.push(frame, packet_type) {
                Reassembled::PassThrough(f) | Reassembled::Complete(f) => f,
                Reassembled::Partial => continue,
                Reassembled::Error(e) => {
                    log::warn!("reassembly error: {e}");
                    continue;
                }
            };
            match self.db.decode(&assembled) {
                Ok(d) => return Some(Ok(d)),
                Err(e) => {
                    log::warn!("decode error: {e}");
                    continue;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Units;

    /// A slice of frames as a [`FrameSource`] — the BYO-transport shape.
    struct VecSource(std::vec::IntoIter<RawFrame>);
    impl FrameSource for VecSource {
        fn read_frame(&mut self) -> io::Result<Option<RawFrame>> {
            Ok(self.0.next())
        }
    }

    fn iso_address_claim() -> RawFrame {
        // From canboat/analyzer/tests/pgn-test.in — Unique Number = 1088507.
        RawFrame::new(
            None,
            6,
            60928,
            5,
            255,
            [0xfb, 0x9b, 0x70, 0x22, 0x00, 0x9b, 0x50, 0xc0],
        )
    }

    #[test]
    fn decoder_yields_decoded_from_a_custom_source() {
        let db = PgnDatabase::embedded(Units::Metric);
        let src = VecSource(vec![iso_address_claim()].into_iter());
        let decoded: Vec<_> = Decoder::new(src, db).map(|r| r.expect("io")).collect();
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].pgn, 60928);
        assert_eq!(decoded[0].id, "isoAddressClaim");
    }

    #[test]
    fn mpsc_receiver_is_a_frame_source() {
        let db = PgnDatabase::embedded(Units::Metric);
        let (tx, rx) = std::sync::mpsc::channel();
        tx.send(iso_address_claim()).unwrap();
        drop(tx); // close the channel → end-of-stream
        let decoded: Vec<_> = Decoder::new(rx, db).map(|r| r.expect("io")).collect();
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].pgn, 60928);
    }
}
