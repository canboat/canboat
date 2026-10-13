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

/// Whether `sum`, what follows the `*`, is the checksum of `body`: exactly
/// two hex digits, as canboat C requires.
pub(crate) fn checksum_matches(body: &str, sum: &str) -> bool {
    let digits = sum.trim_end();
    digits.len() == 2
        && digits.bytes().all(|b| b.is_ascii_hexdigit())
        && u8::from_str_radix(digits, 16) == Ok(nmea_checksum(body))
}

/// `line` without a leading NMEA 0183 TAG block (`\…\`).
pub(crate) fn without_tag_block(line: &str) -> &str {
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

/// Whether `line` is a MiniPlex's `$PSMDNC` or `$PSMDNP` ([`Psmd`]),
/// possibly behind a TAG block: a MiniPlex stream, even without `$MXPGN`.
pub fn is_psmd(line: &str) -> bool {
    let s = without_tag_block(line.trim_start());
    s.starts_with("$PSMDNC,") || s.starts_with("$PSMDNP,")
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
    parse_sentence(line, true)
}

/// [`parse_line`], checking the checksum only if `checksum` is set.
pub fn parse_sentence(line: &str, checksum: bool) -> Result<Option<RawFrame>, ParseError> {
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
    if let Some(sum) = sum.filter(|_| checksum)
        && !checksum_matches(body, sum)
    {
        return Err(bad("checksum", sum));
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

/// The NMEA 2000 network management a MiniPlex sends as its own `$PSMD…`
/// sentences, never as `$MXPGN`, whatever it is set to (ShipModul,
/// "MiniPlex-3 NMEA and Proprietary Sentences", FW V3.40.0, pp. 25–29):
///
/// ```text
///   $PSMDNC,R,<NAME, 16 hex digits>,<source>            PGN 60928
///   $PSMDNP,A,<source>,<version>,<product code>,<certification level>,<load>
///   $PSMDNP,B,<model ID>,<serial code>                  PGN 126996, in
///   $PSMDNP,C,<software version>,<model version>        three parts
/// ```
///
/// An Address Claim comes whenever a device claims an address; Product
/// Information only in answer to `$PSMDNR,126996` ([`request`]). The three
/// `$PSMDNP` parts arrive in order, and only `A` names the source. A
/// software version may hold a comma (an NGT-1's is `1.100, 2.690`): the
/// last field of `B` and `C` is the second one, the rest the first.
#[derive(Debug, Default)]
pub struct Psmd {
    product: Option<Product>,
}

#[derive(Debug)]
struct Product {
    src: u8,
    /// The NMEA 2000 version × 1000, as PGN 126996 carries it.
    version: u64,
    product_code: i64,
    certification_level: i64,
    load_equivalency: i64,
    /// Part `B`: the model ID and the serial code.
    model: Option<(String, String)>,
}

/// The two fields `rest` holds, the second being its last, the first all
/// before it (which may hold a comma).
fn split_last(rest: &str) -> Option<(&str, &str)> {
    rest.rsplit_once(',')
}

impl Psmd {
    /// The message the `$PSMDNC` or `$PSMDNP` sentence `line` completes:
    /// PGN 60928 from a `$PSMDNC` report, PGN 126996 from the `C` part of
    /// a `$PSMDNP` that followed its `A` and `B`; `Ok(None)` for another
    /// part or another sentence; an error for a malformed one. `checksum`
    /// checks the checksum, as [`parse_sentence`] does.
    pub fn push(&mut self, line: &str, checksum: bool) -> Result<Option<RawFrame>, ParseError> {
        let sentence = without_tag_block(line.trim());
        let Some(rest) = sentence.strip_prefix('$') else {
            return Ok(None);
        };
        let (body, sum) = match rest.split_once('*') {
            Some((body, sum)) => (body, Some(sum)),
            None => (rest, None),
        };
        let is_nc = body.starts_with("PSMDNC,");
        let is_np = body.starts_with("PSMDNP,");
        if !is_nc && !is_np {
            return Ok(None);
        }
        if let Some(sum) = sum.filter(|_| checksum)
            && !checksum_matches(body, sum)
        {
            return Err(bad("checksum", sum));
        }
        if is_nc {
            return address_claim(&body["PSMDNC,".len()..]);
        }
        let rest = &body["PSMDNP,".len()..];
        let (part, fields) = rest.split_once(',').unwrap_or((rest, ""));
        match part {
            "A" => {
                self.product = None;
                let f: Vec<&str> = fields.split(',').collect();
                let [src, version, code, level, load] = f[..] else {
                    return Err(ParseError::BadHeader {
                        expected: 6,
                        found: f.len() + 1,
                    });
                };
                let int = |field, v: &str| v.trim().parse::<i64>().map_err(|_| bad(field, v));
                let version_f: f64 = version
                    .trim()
                    .parse()
                    .map_err(|_| bad("nmea2000Version", version))?;
                // Raw, so unchecked by the encoder: 0.000 ..= 65.532.
                let version_raw = (version_f * 1000.0).round();
                if !(0.0..=65532.0).contains(&version_raw) {
                    return Err(bad("nmea2000Version", version));
                }
                self.product = Some(Product {
                    src: src.trim().parse().map_err(|_| bad("src", src))?,
                    version: version_raw as u64,
                    product_code: int("productCode", code)?,
                    certification_level: int("certificationLevel", level)?,
                    load_equivalency: int("loadEquivalency", load)?,
                    model: None,
                });
                Ok(None)
            }
            "B" => {
                let (model, serial) = split_last(fields).ok_or_else(|| bad("modelId", fields))?;
                if let Some(p) = self.product.as_mut() {
                    p.model = Some((model.to_string(), serial.to_string()));
                }
                Ok(None)
            }
            "C" => {
                let (software, version) =
                    split_last(fields).ok_or_else(|| bad("softwareVersionCode", fields))?;
                // Without the parts before it (missed), it completes nothing.
                match self.product.take() {
                    Some(p @ Product { model: Some(_), .. }) => {
                        let (model, serial) = p.model.as_ref().expect("matched above");
                        product_information(&p, model, serial, software, version).map(Some)
                    }
                    _ => Ok(None),
                }
            }
            _ => Ok(None),
        }
    }
}

/// PGN 60928 from the fields of a `$PSMDNC` report: `R`, the NAME, the
/// source. A command (`C`, setting another device's address) is no claim.
fn address_claim(fields: &str) -> Result<Option<RawFrame>, ParseError> {
    let f: Vec<&str> = fields.split(',').collect();
    let [flag, name, src] = f[..] else {
        return Err(ParseError::BadHeader {
            expected: 4,
            found: f.len() + 1,
        });
    };
    if flag != "R" {
        return Ok(None);
    }
    if name.len() != 16 {
        return Err(bad("name", name));
    }
    let name = u64::from_str_radix(name, 16).map_err(|_| bad("name", name))?;
    let src = src.trim().parse::<u8>().map_err(|_| bad("src", src))?;
    // The NAME goes on the bus least significant byte first.
    Ok(Some(RawFrame::new(
        None,
        6,
        60928,
        src,
        255,
        name.to_le_bytes(),
    )))
}

/// PGN 126996 from the three `$PSMDNP` parts.
fn product_information(
    p: &Product,
    model: &str,
    serial: &str,
    software: &str,
    version: &str,
) -> Result<RawFrame, ParseError> {
    use crate::engine::encode::Raw;
    use crate::engine::field::product_information as pi;
    let db = crate::engine::PgnDatabase::embedded(crate::engine::Units::Si);
    let build = || -> Result<RawFrame, crate::engine::encode::EncodeError> {
        let mut b = db.encode("productInformation")?.source(p.src);
        b.push(pi::NMEA2000_VERSION, Raw(p.version))?;
        b.push(pi::PRODUCT_CODE, p.product_code)?;
        b.push(pi::MODEL_ID, model)?;
        b.push(pi::SOFTWARE_VERSION_CODE, software)?;
        b.push(pi::MODEL_VERSION, version)?;
        b.push(pi::MODEL_SERIAL_CODE, serial)?;
        b.push(pi::CERTIFICATION_LEVEL, p.certification_level)?;
        b.push(pi::LOAD_EQUIVALENCY, p.load_equivalency)?;
        b.build()
    };
    build().map_err(|e| bad("productInformation", &e.to_string()))
}

/// The `$PSMDNR` sentence, with its checksum and line end, that asks a
/// MiniPlex to request `pgn` on the bus, from `dst` or (255) everyone, and
/// pass on the answers: Address Claims as `$PSMDNC`, Product Information
/// as `$PSMDNP`, which it sends only so.
pub fn request(pgn: u32, dst: u8) -> String {
    let body = if dst == 255 {
        format!("PSMDNR,{pgn}")
    } else {
        format!("PSMDNR,{pgn},{dst}")
    };
    format!("${body}*{:02X}\r\n", nmea_checksum(&body))
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

    /// A MiniPlex-3Wi-N2K's Address Claims and Product Information, from
    /// ShipModul's capture (samples/miniplex-3wi-n2k-nmea0183.txt): an
    /// NGT-1, whose software version holds a comma.
    #[test]
    fn psmd_address_claim_and_product_information() {
        let mut psmd = Psmd::default();
        let claim = psmd
            .push("$PSMDNC,R,C032820022224170,3*30", true)
            .unwrap()
            .unwrap();
        assert_eq!(head(&claim), (6, 60928, 3, 255));
        assert_eq!(
            u64::from_le_bytes(claim.data[..].try_into().unwrap()),
            0xC032820022224170
        );
        for line in [
            "$PSMDNP,A,3,2.100,28199,2,1*73",
            "$PSMDNP,B,NMEA 2000 PC Interface (NGT-1),147824*65",
        ] {
            assert!(psmd.push(line, true).unwrap().is_none(), "{line}");
        }
        let info = psmd
            .push("$PSMDNP,C,1.100, 2.690,NGT-1-USB  [5]*61", true)
            .unwrap()
            .unwrap();
        assert_eq!(head(&info), (6, 126996, 3, 255));
        assert_eq!(info.data.len(), 134);
        assert_eq!(&info.data[..4], &[0x34, 0x08, 0x27, 0x6e]); // 2100, 28199
        // Fixed strings, padded with 0xff.
        let text = |at: usize| {
            let field = &info.data[at..at + 32];
            let end = field.iter().rposition(|&b| b != 0xff).map_or(0, |i| i + 1);
            String::from_utf8_lossy(&field[..end]).into_owned()
        };
        assert_eq!(text(4), "NMEA 2000 PC Interface (NGT-1)");
        assert_eq!(text(36), "1.100, 2.690");
        assert_eq!(text(68), "NGT-1-USB  [5]");
        assert_eq!(text(100), "147824");
        assert_eq!(&info.data[132..], &[2, 1]);
    }

    /// A part without the ones before it completes nothing; a command, other
    /// MiniPlex sentences and a wrong checksum are no message (the last an
    /// error, unless the checksum is not checked).
    #[test]
    fn psmd_ignores_what_completes_nothing() {
        let mut psmd = Psmd::default();
        for line in [
            "$PSMDNP,C,3.41.0,MPX-3E-N V1.0 - Main*5F",
            "$PSMDNP,B,MiniPlex-3E-N2K - Main,3A000001*4C",
            "$PSMDNP,C,3.41.0,MPX-3E-N V1.0 - Main*5F",
            "$PSMDSTAT,R,1130,15,0,42,0,0*57",
            "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A",
        ] {
            assert!(psmd.push(line, true).unwrap().is_none(), "{line}");
        }
        let command = format!(
            "$PSMDNC,C,C03287004A680009,4*{:02X}",
            nmea_checksum("PSMDNC,C,C03287004A680009,4")
        );
        assert!(psmd.push(&command, true).unwrap().is_none());
        let wrong = "$PSMDNC,R,C03287004A680009,1*48";
        assert!(psmd.push(wrong, true).is_err());
        assert!(psmd.push(wrong, false).unwrap().is_some());
        assert!(is_psmd("$PSMDNP,A,1,2.000,26884,2,0*7A") && !is_psmd("$PSMDSTAT,R,1*00"));
    }

    #[test]
    fn psmd_rejects_out_of_range_product_fields() {
        for a in [
            "$PSMDNP,A,1,65.533,26884,2,0",
            "$PSMDNP,A,1,-1.000,26884,2,0",
            "$PSMDNP,A,1,2.000,65533,2,0",
            "$PSMDNP,A,1,2.000,26884,2,253",
        ] {
            let mut psmd = Psmd::default();
            let product = psmd.push(a, false).and_then(|_| {
                psmd.push("$PSMDNP,B,MiniPlex,3A000001", false)?;
                psmd.push("$PSMDNP,C,3.41.0,MPX", false)
            });
            assert!(product.is_err(), "{a}");
        }
    }

    #[test]
    fn psmd_requests() {
        assert_eq!(request(126996, 255), "$PSMDNR,126996*39\r\n");
        assert_eq!(
            request(60928, 7),
            format!(
                "$PSMDNR,60928,7*{:02X}\r\n",
                nmea_checksum("PSMDNR,60928,7")
            )
        );
    }
}
