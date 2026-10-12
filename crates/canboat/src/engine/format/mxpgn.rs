// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `$MXPGN` line format: ShipModul MiniPlex NMEA 2000 frames inside NMEA
//! 0183 sentences, as a MiniPlex-3 sends them, and as an Actisense
//! gateway does when its NMEA 0183 output is set to it.
//!
//! Layout (ShipModul, "MiniPlex-3 NMEA and Proprietary Sentences",
//! FW V3.40.0, p. 4; the Actisense SDK copies it):
//!
//! ```text
//!   $--PGN,pppppp,aaaa,d--d*hh
//! ```
//!
//!   - `--` — any talker; a MiniPlex sends `MX`.
//!   - `pppppp` — the PGN, hex. For a PDU1 (addressed) PGN its low byte
//!     is the destination.
//!   - `aaaa` — the attribute word, hex: bit 15 the send bit S (0 for a
//!     received frame, 1 for one to send), bits 14–12 the priority, bits
//!     11–8 the data length (9–15: a "Class 2 Transmission ID", with 8
//!     data bytes), bits 7–0 the source address when S is 0, the
//!     destination when it is 1. A sentence to send may leave it empty:
//!     then S is 1, the priority 7, the length that of the data, and the
//!     destination the PGN's low byte.
//!   - `d--d` — the data "as one large number from MSB to LSB" (NMEA 2000
//!     Appendix D.1): the last byte first.
//!   - `*hh` — the NMEA 0183 checksum, checked when present.
//!
//! One sentence is one CAN frame: a fast-packet arrives as one sentence
//! per frame, for the caller to join. Sentences carry no time. A leading
//! NMEA 0183 TAG block (`\s:MX01*hh\`) is skipped, and every other
//! NMEA 0183 sentence — a MiniPlex mixes them in — is no frame.

use std::fmt;

use smallvec::SmallVec;

use crate::engine::format::common::{iso11783_compose, iso11783_decompose};
use crate::engine::format::plain::ParseError;
use crate::engine::frame::RawFrame;

/// Whether `pgn` is PDU1 (addressed): its low byte is the destination.
fn is_pdu1(pgn: u32) -> bool {
    (pgn >> 8) & 0xff < 240
}

/// The XOR of the bytes of an NMEA 0183 sentence between `$` and `*`.
pub(crate) fn nmea_checksum(body: &str) -> u8 {
    body.bytes().fold(0, |sum, b| sum ^ b)
}

/// `line` without a leading NMEA 0183 TAG block (`\…\`).
fn without_tag_block(line: &str) -> &str {
    match line
        .strip_prefix('\\')
        .and_then(|rest| rest.split_once('\\'))
    {
        Some((_, sentence)) => sentence,
        None => line,
    }
}

/// Whether `line` is an `$MXPGN` sentence (any talker), possibly behind a
/// TAG block.
pub fn is_mxpgn(line: &str) -> bool {
    let s = without_tag_block(line.trim_start()).as_bytes();
    s.len() > 7 && s[0] == b'$' && &s[3..7] == b"PGN,"
}

/// Whether `line` is an NMEA 0183 sentence (`$` or `!`), or starts with a
/// TAG block: a line a MiniPlex stream holds that need not be a frame.
pub fn is_nmea0183(line: &str) -> bool {
    matches!(
        line.trim_start().as_bytes().first(),
        Some(b'$' | b'!' | b'\\')
    )
}

fn bad(field: &'static str, value: &str) -> ParseError {
    ParseError::BadInteger {
        field,
        value: value.to_string(),
        offset: None,
    }
}

/// The frame an `$MXPGN` sentence carries, `Ok(None)` for another NMEA
/// 0183 sentence, or an error for a malformed one (bad hex, a checksum
/// that does not match, data that does not fit its length).
pub fn parse_line(line: &str) -> Result<Option<RawFrame>, ParseError> {
    let line = line.trim();
    if line.is_empty() {
        return Err(ParseError::Empty);
    }
    if !is_mxpgn(line) {
        return if is_nmea0183(line) {
            Ok(None)
        } else {
            Err(ParseError::MissingTimestamp)
        };
    }
    let sentence = without_tag_block(line);
    let (body, sum) = match sentence[1..].split_once('*') {
        Some((body, sum)) => (body, Some(sum)),
        None => (&sentence[1..], None),
    };
    if let Some(sum) = sum {
        let want = u8::from_str_radix(sum.trim(), 16).map_err(|_| bad("checksum", sum))?;
        if nmea_checksum(body) != want {
            return Err(bad("checksum", sum));
        }
    }
    let mut fields = body.split(',').skip(1);
    let (Some(pgn_tok), Some(attr_tok), Some(data_tok), None) =
        (fields.next(), fields.next(), fields.next(), fields.next())
    else {
        return Err(ParseError::BadHeader {
            expected: 3,
            found: body.split(',').count() - 1,
        });
    };
    let pgn_field = u32::from_str_radix(pgn_tok, 16).map_err(|_| bad("pgn", pgn_tok))?;
    if pgn_field > 0x3ffff {
        return Err(bad("pgn", pgn_tok));
    }
    if data_tok.len() % 2 != 0 || data_tok.len() > 16 {
        return Err(ParseError::BadPayloadCount {
            expected: 8,
            found: data_tok.len().div_ceil(2),
        });
    }
    let mut data: SmallVec<[u8; 8]> = SmallVec::new();
    // The last byte first: read the pairs from the end.
    for (i, pair) in data_tok.as_bytes().rchunks(2).enumerate() {
        let s = std::str::from_utf8(pair).unwrap_or("");
        data.push(
            u8::from_str_radix(s, 16).map_err(|_| ParseError::BadHexByte {
                index: i,
                value: s.to_string(),
                offset: None,
            })?,
        );
    }
    let attr = if attr_tok.is_empty() {
        // The short send form: S, priority 7, the data's length, and the
        // destination in the PGN.
        0x8000 | 7 << 12 | (data.len() as u16) << 8
    } else {
        u16::from_str_radix(attr_tok, 16).map_err(|_| bad("attribute", attr_tok))?
    };
    let send = attr & 0x8000 != 0;
    let prio = ((attr >> 12) & 7) as u8;
    let dlc = usize::from((attr >> 8) & 0xf);
    let address = (attr & 0xff) as u8;
    let want = if dlc > 8 { 8 } else { dlc };
    if data.len() != want {
        return Err(ParseError::BadPayloadCount {
            expected: want,
            found: data.len(),
        });
    }
    let (pgn, dst) = if is_pdu1(pgn_field) {
        let in_pgn = (pgn_field & 0xff) as u8;
        // A frame to send names its destination in the attribute word, or
        // (address 0) in the PGN.
        let dst = if send && address != 0 {
            address
        } else {
            in_pgn
        };
        (pgn_field & !0xff, dst)
    } else {
        (pgn_field, 255)
    };
    // A received frame names its source; one to send gets the MiniPlex's.
    let src = if send { 0 } else { address };
    Ok(Some(RawFrame {
        timestamp: None,
        prio,
        pgn,
        src,
        dst,
        data,
    }))
}

/// Write `frame`, one CAN frame of at most 8 bytes, as an `$MXPGN`
/// sentence with its checksum (no line end). `send` writes the form a
/// MiniPlex transmits (S set, the destination in the attribute word);
/// otherwise the form it sends for a received frame (the source there).
pub fn write_line<W: fmt::Write>(w: &mut W, frame: &RawFrame, send: bool) -> fmt::Result {
    debug_assert!(frame.data.len() <= 8);
    let id = iso11783_compose(frame.prio & 7, frame.pgn, frame.src, frame.dst);
    let (_, pgn, _, dst) = iso11783_decompose(id);
    let pdu1 = is_pdu1(pgn);
    let (pgn_field, address) = match (send, pdu1) {
        // Sent: the PGN without its destination, which goes in the
        // attribute word; for a global PGN the address is ignored.
        (true, true) => (pgn, dst),
        (true, false) => (pgn, 0),
        // Received: the destination stays in the PGN.
        (false, true) => (pgn | u32::from(dst), frame.src),
        (false, false) => (pgn, frame.src),
    };
    let attr = u16::from(send) << 15
        | u16::from(frame.prio & 7) << 12
        | (frame.data.len() as u16) << 8
        | u16::from(address);
    let mut body = format!("MXPGN,{pgn_field:06X},{attr:04X},");
    for b in frame.data.iter().rev() {
        body.push_str(&format!("{b:02X}"));
    }
    write!(w, "${body}*{:02X}", nmea_checksum(&body))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(line: &str) -> RawFrame {
        parse_line(line).unwrap().unwrap()
    }

    fn head(f: &RawFrame) -> (u8, u32, u8, u8) {
        (f.prio, f.pgn, f.src, f.dst)
    }

    /// The cases of #997, from ShipModul's reference and a capture.
    #[test]
    fn the_golden_sentences() {
        let f = parse("$MXPGN,01F801,2801,C1308AC40C5DE343*19");
        assert_eq!(head(&f), (2, 129025, 1, 255));
        assert_eq!(
            f.data.as_slice(),
            [0x43, 0xe3, 0x5d, 0x0c, 0xc4, 0x8a, 0x30, 0xc1]
        );
        assert_eq!(f.timestamp, None);

        let tagged = parse("\\s:serial,c:1696759212*3E\\$MXPGN,01F801,2801,C1308AC40C5DE343*19");
        assert_eq!(tagged, f);

        let heading = parse("$MXPGN,01F112,2807,FC7FFF7FFF168012*11");
        assert_eq!(head(&heading), (2, 127250, 7, 255));
        assert_eq!(
            heading.data.as_slice(),
            [0x12, 0x80, 0x16, 0xff, 0x7f, 0xff, 0x7f, 0xfc]
        );
        // The send form of the same: S set, the source is the MiniPlex's.
        let send = parse("$MXPGN,01F112,A800,FC7FFF7FFF168012*65");
        assert_eq!(head(&send), (2, 127250, 0, 255));
        assert_eq!(send.data, heading.data);

        // An ISO Request to 9, the destination in the attribute word, or
        // in the PGN with the attribute left out.
        for line in ["$MXPGN,00EA00,F309,01F016*68", "$MXPGN,00EA09,,01F016*1D"] {
            let f = parse(line);
            assert_eq!(head(&f), (7, 59904, 0, 9), "{line}");
            assert_eq!(f.data.as_slice(), [0x16, 0xf0, 0x01]);
        }
    }

    #[test]
    fn a_received_addressed_frame_keeps_its_destination() {
        let f = RawFrame::new(None, 6, 59904, 0x23, 0x09, [0x16, 0xf0, 0x01]);
        let mut line = String::new();
        write_line(&mut line, &f, false).unwrap();
        assert_eq!(line, "$MXPGN,00EA09,6323,01F016*19");
        assert_eq!(head(&parse(&line)), (6, 59904, 0x23, 0x09));
    }

    /// The sentences canboat sends are ShipModul's own examples.
    #[test]
    fn the_send_form_is_shipmoduls() {
        let mut line = String::new();
        let heading = parse("$MXPGN,01F112,2807,FC7FFF7FFF168012*11");
        write_line(&mut line, &heading, true).unwrap();
        assert_eq!(line, "$MXPGN,01F112,A800,FC7FFF7FFF168012*65");

        line.clear();
        let request = RawFrame::new(None, 7, 59904, 0, 0x09, [0x16, 0xf0, 0x01]);
        write_line(&mut line, &request, true).unwrap();
        assert_eq!(line, "$MXPGN,00EA00,F309,01F016*68");
    }

    #[test]
    fn other_sentences_are_no_frames() {
        assert_eq!(
            parse_line("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A")
                .unwrap(),
            None
        );
        assert_eq!(
            parse_line("!AIVDM,1,1,,A,13aEOK?P00PD2wVMdLDRhgvL289?,0*26").unwrap(),
            None
        );
        assert_eq!(
            parse_line("\\s:MX01*4D\\$PSMDNC,R,C032820022220839,7*2A").unwrap(),
            None
        );
        assert!(parse_line("2026-10-13T00:00:00Z,6,59904,1,255,3,00,ee,00").is_err());
    }

    #[test]
    fn bad_sentences_are_errors() {
        // A checksum that does not match.
        assert!(parse_line("$MXPGN,01F801,2801,C1308AC40C5DE343*18").is_err());
        // Data that does not fit its length, odd hex, a bad attribute.
        assert!(parse_line("$MXPGN,01F801,2701,C1308AC40C5DE343").is_err());
        assert!(parse_line("$MXPGN,01F801,2801,C1308AC40C5DE34").is_err());
        assert!(parse_line("$MXPGN,01F801,28G1,C1308AC40C5DE343").is_err());
        // No checksum is accepted; a Class 2 length means 8 bytes.
        assert!(parse_line("$MXPGN,01F801,2801,C1308AC40C5DE343").is_ok());
        assert_eq!(parse("$MXPGN,01F801,2F01,C1308AC40C5DE343").data.len(), 8);
    }

    #[test]
    fn detection() {
        assert!(is_mxpgn("$MXPGN,01F801,2801,C1308AC40C5DE343*19"));
        assert!(is_mxpgn("$YDPGN,01F801,2801,00"));
        assert!(is_mxpgn("\\s:MX01*4D\\$MXPGN,01F801,2801,00"));
        assert!(!is_mxpgn("$PCDIN,01F801,000C7E1B,0F,FFFF*6E"));
        assert!(is_nmea0183("$GPRMC,1"));
        assert!(!is_nmea0183("A173321.107 23FF7 1F513"));
    }
}
