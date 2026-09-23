// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Wire-format helpers shared across the individual converters.
//!
//! These are format-agnostic building blocks that several converters
//! (and the SocketCAN device / `.pcap` container readers) need, so they
//! live here rather than inside one format module where a caller in
//! another module would have to reach across an unrelated namespace.
//!
//! (Calendar math — `days_to_ymd` / `days_since_epoch` — is the other
//! widely-shared primitive; it lives in [`super::timestamp`] alongside
//! the string timestamp normaliser.)

/// Decompose a 29-bit ISO 11783 CAN identifier into (prio, pgn, src, dst).
/// Matches `getISO11783BitsFromCanId` in canboat/common/common.c.
///
///   priority (3 bits) | reserved+dp (2 bits) | PF (8) | PS (8) | SA (8)
///
/// - PDU1 (PF < 240): PGN = (RDP << 16) | (PF << 8); dst = PS.
/// - PDU2 (PF ≥ 240): PGN = (RDP << 16) | (PF << 8) | PS; dst = 255.
///
/// The RDP (Reserved + DataPage) bits are the two just below the
/// priority bits — they push PGNs above 0x10000 into the data-page-1
/// range (e.g. CAN-id 0x09FD020D → PGN 130306 = 0x1FD02, not 0xFD02).
pub fn iso11783_decompose(id: u32) -> (u8, u32, u8, u8) {
    let prio = ((id >> 26) & 0x7) as u8;
    let rdp = (id >> 24) & 0x3;
    let pf = ((id >> 16) & 0xff) as u8;
    let ps = ((id >> 8) & 0xff) as u8;
    let src = (id & 0xff) as u8;
    let (pgn, dst) = if pf < 240 {
        ((rdp << 16) | ((pf as u32) << 8), ps)
    } else {
        ((rdp << 16) | ((pf as u32) << 8) | ps as u32, 0xff)
    };
    (prio, pgn, src, dst)
}

/// Build the 29-bit ISO 11783 CAN identifier from N2K fields — the
/// inverse of [`iso11783_decompose`]. Mirrors `getCanIdFromISO11783Bits`
/// in canboat/common/common.c. For a PDU1 PGN (PF < 240) the `dst`
/// occupies the PS byte; for PDU2 it is part of the PGN and `dst` is
/// ignored (always the 0xFF broadcast on decode).
pub fn iso11783_compose(prio: u8, pgn: u32, src: u8, dst: u8) -> u32 {
    let mut id = src as u32;
    let pf = (pgn >> 8) & 0xff;
    if pf < 240 {
        id |= (dst as u32) << 8;
        id |= pgn << 8;
    } else {
        id |= pgn << 8;
    }
    id |= (prio as u32) << 26;
    id
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compose_is_inverse_of_decompose() {
        // PDU2 (dst is part of the PGN, forced to broadcast on decode).
        assert_eq!(
            iso11783_decompose(iso11783_compose(3, 0x1f50b, 0x23, 0xff)),
            (3, 0x1f50b, 0x23, 0xff)
        );
        // PDU1 (dst rides in the PS byte).
        assert_eq!(
            iso11783_decompose(iso11783_compose(6, 0xee00, 0x05, 0xff)),
            (6, 0xee00, 0x05, 0xff)
        );
        assert_eq!(
            iso11783_decompose(iso11783_compose(2, 0xef00, 0x11, 0x22)),
            (2, 0xef00, 0x11, 0x22)
        );
    }

    #[test]
    fn decompose_pdu2_known_canid() {
        // CAN id 0x0DF50B23: prio=3, RDP=1, PF=0xF5, PS=0x0B, SA=0x23.
        // PF >= 240 → PDU2; PGN = 0x1F50B = 128267; dst=255.
        assert_eq!(iso11783_decompose(0x0DF50B23), (3, 0x1f50b, 0x23, 0xff));
    }
}
