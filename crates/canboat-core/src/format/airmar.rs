// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Airmar single-line capture format.
//!
//! Layout (one whole frame per line, separator is the first run of
//! `" - "` after a leading timestamp):
//!
//! ```text
//!   <timestamp> - <PGN> <CANID:hex> <data>...
//! ```
//!
//!   - `timestamp` — free-form, passed through verbatim to the
//!     output; the parser keeps everything up to the first space.
//!   - `PGN` — decimal PGN number. Discarded — canboat re-derives
//!     the PGN from the ISO-11783 CAN identifier (so a wrong PGN
//!     value in the capture doesn't matter).
//!   - `CANID` — 8-hex-digit 29-bit ISO 11783 identifier (priority
//!     + RDP + PF + PS + SA).
//!   - data bytes follow as 1 or 2 hex digits, separated by spaces
//!     or commas.
//!
//! Mirrors `parseRawFormatAirmar` in canboat/common/parse.c. Multi-
//! packet messages arrive pre-coalesced on a single line, so the
//! caller should treat this format as `MULTIPACKETS_COALESCED`.

use smallvec::SmallVec;

use crate::format::common::iso11783_decompose;
use crate::format::plain::ParseError;
use crate::frame::{FASTPACKET_MAX_SIZE, RawFrame};

pub fn parse_line(line: &str) -> Result<RawFrame, ParseError> {
    let line = line.trim_end_matches(['\r', '\n']);
    if line.is_empty() {
        return Err(ParseError::Empty);
    }
    // Split on the first " - " marker. The bit before is the
    // free-form timestamp; the bit after carries `<pgn> <id> <data>`.
    let (timestamp, rest) = line.split_once(" - ").ok_or(ParseError::BadHeader {
        expected: 3,
        found: 1,
    })?;
    let mut toks = rest.split_whitespace();
    let _pgn_tok = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 2,
    })?;
    let canid_tok = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 3,
    })?;
    let canid = u32::from_str_radix(canid_tok, 16).map_err(|_| ParseError::BadInteger {
        field: "canid",
        value: canid_tok.to_string(),
        offset: None,
    })?;
    let (prio, pgn, src, dst) = iso11783_decompose(canid);

    // Remaining tokens are data bytes — Airmar captures separate them
    // with either spaces or commas. After the whitespace split the
    // commas remain inside the tokens; strip them and parse each
    // 1- or 2-digit hex byte.
    let mut data: SmallVec<[u8; 8]> = SmallVec::new();
    for raw in toks {
        for tok in raw.split(',').filter(|s| !s.is_empty()) {
            if data.len() >= FASTPACKET_MAX_SIZE {
                break;
            }
            let b = u8::from_str_radix(tok, 16).map_err(|_| ParseError::BadHexByte {
                index: data.len(),
                value: tok.to_string(),
                offset: None,
            })?;
            data.push(b);
        }
    }

    Ok(RawFrame {
        timestamp: Some(timestamp.to_string()),
        prio,
        pgn,
        src,
        dst,
        data,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_airmar_line() {
        // CAN id 0x09FD020D — prio=2, RDP=1, PF=0xFD, PS=0x02, SA=0x0D.
        // PF >= 240 → PDU2; PGN = (RDP<<16) | (PF<<8) | PS = 0x1FD02
        // = 130306 ("Wind Data"); dst=255.
        let line = "20:11:00 - 130306 09FD020D FF 8B 72 FF FF FF FF FF";
        let f = parse_line(line).unwrap();
        assert_eq!(f.timestamp.as_deref(), Some("20:11:00"));
        assert_eq!(f.prio, 2);
        assert_eq!(f.pgn, 130306);
        assert_eq!(f.src, 0x0d);
        assert_eq!(f.dst, 0xff);
        assert_eq!(
            f.data.as_slice(),
            &[0xff, 0x8b, 0x72, 0xff, 0xff, 0xff, 0xff, 0xff]
        );
    }

    #[test]
    fn accepts_comma_separated_data() {
        let line = "00:00:00 - 0 18EEFF00 8E,F2,DD,E8,00,96,64,40";
        let f = parse_line(line).unwrap();
        // PF=0xEE (<240) → PDU1; PS=0xFF → dst, src=0x00.
        assert_eq!(f.pgn, 0xee00);
        assert_eq!(f.dst, 0xff);
        assert_eq!(f.src, 0x00);
        assert_eq!(f.data.len(), 8);
    }
}
