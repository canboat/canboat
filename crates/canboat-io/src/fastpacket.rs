// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Fast-packet vs single-frame classification for outbound PGN encoding
//! and inbound reassembly.
//!
//! Single-frame PGNs fit in one 8-byte CAN frame and decode against the
//! schema verbatim. Fast-packet PGNs are framed with a one-byte
//! `seq | index` header on every CAN frame, a one-byte length on the
//! first, and 0xff padding on the last to round to 8 bytes. Picking the
//! wrong framing on either path silently corrupts every field — see
//! `device::socketcan::send_pgn` for the encoder side and
//! `socketcan-serial`'s reassembler for the decoder side.
//!
//! Range rules:
//!
//! | PGN range            | Framing                                                  |
//! |----------------------|----------------------------------------------------------|
//! | `0x00000..0x10000`   | Single (J1939 PDU1/PDU2 base)                            |
//! | `0x10000..0x1F000`   | Fast (NMEA 2000 fast-packet-only span)                   |
//! | `0x1F000..0x1FFFF`   | Mixed — consult [`FASTPACKET_MIXED`] (built from JSON)   |
//! | `0x1FFFF..0x20000`   | Single (technically out of fast-packet space)            |
//! | `>= 0x40000` (BEM)   | Fast (synthetic CANboat PGNs, never on the wire)         |
//!
//! The mixed-range table is generated at build time from
//! `crates/canboat-core/data/canboat.json` (see `build.rs`).

use canboat_core::FramePacketType;

include!(concat!(env!("OUT_DIR"), "/fastpacket_table.rs"));

/// Synthetic-PGN marker (CANBOAT_BEM in canboat C). Above this we
/// treat anything as fast-packet, since the synthetic frames we
/// generate internally are by definition coalesced multi-byte
/// payloads.
pub const CANBOAT_PGN_START: u32 = 0x40000;

/// Classify `pgn` as fast-packet or single-frame. Mirrors canboat C's
/// `isFastPacket()` plus the synthetic-PGN extension.
pub fn packet_type(pgn: u32) -> FramePacketType {
    if (FASTPACKET_MIXED_START..FASTPACKET_MIXED_END).contains(&pgn) {
        return if FASTPACKET_MIXED[(pgn - FASTPACKET_MIXED_START) as usize] {
            FramePacketType::Fast
        } else {
            FramePacketType::Single
        };
    }
    if (0x10000..FASTPACKET_MIXED_START).contains(&pgn) {
        return FramePacketType::Fast;
    }
    if pgn >= CANBOAT_PGN_START {
        return FramePacketType::Fast;
    }
    FramePacketType::Single
}

#[cfg(test)]
mod tests {
    use super::*;

    /// PGN 127508 (Battery Status) sits in the mixed range and per
    /// `crates/canboat-core/data/canboat.json` is **single-frame**, not fast — the table
    /// must reflect that, otherwise our encoder wraps an 8-byte
    /// payload in a fast-packet shell and the receiver decodes
    /// garbage. This is the regression test for the original bug.
    #[test]
    fn pgn_127508_battery_status_is_single() {
        assert_eq!(packet_type(127508), FramePacketType::Single);
    }

    /// PGN 127506 (DC Detailed Status) is fast-packet per the spec
    /// and per `crates/canboat-core/data/canboat.json`. Sanity check the table the other
    /// way too.
    #[test]
    fn pgn_127506_dc_detailed_is_fast() {
        assert_eq!(packet_type(127506), FramePacketType::Fast);
    }

    /// PGN 126996 (Product Information) is in the fast-packet-only
    /// range below the mixed band; classifier should resolve it
    /// without touching the table.
    #[test]
    fn pgn_126996_product_info_is_fast() {
        assert_eq!(packet_type(126996), FramePacketType::Fast);
    }

    /// PGN 60928 (ISO Address Claim) is below the fast-packet base,
    /// so it falls through to Single.
    #[test]
    fn pgn_60928_address_claim_is_single() {
        assert_eq!(packet_type(60928), FramePacketType::Single);
    }

    /// PGN 126993 (Heartbeat) is in the mixed range and is single-
    /// frame per the spec — table-backed classification must agree.
    #[test]
    fn pgn_126993_heartbeat_is_single() {
        assert_eq!(packet_type(126993), FramePacketType::Single);
    }
}
