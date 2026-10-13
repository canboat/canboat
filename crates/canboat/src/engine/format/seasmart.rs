// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `$PCDIN` line format: Chetco's SeaSmart.Net protocol.
//!
//! Chetco's own gateways write it, and so do others: the Quark-elec
//! QK-A032, QK-A034 and QK-A036 output it over WiFi and USB (the A036
//! also logs it to its SD card), and the Yacht Devices YDWG-02 and
//! YDNG-03 read and write it in NMEA 0183 mode.
//!
//! Layout (Chetco, "SeaSmart.Net Protocol Specification" v1.7.0; one
//! NMEA 0183-style sentence per message):
//!
//! ```text
//!   $PCDIN,<PGN:hex>,<tstamp>,<SRC:hex>,<data_hex>...*<XX>
//! ```
//!
//!   - `PGN` — 24-bit PGN number, hex.
//!   - `tstamp` — 8 characters, which the specification does not define
//!     further. Most gateways write a hex count of milliseconds since
//!     they started (Yacht Devices, Chetco's early firmware), Quark-elec
//!     writes `00000000`, and Chetco's later firmware base-32 seconds
//!     since 2010. Read as hex milliseconds since the Unix epoch, up to
//!     the first non-hex character, as canboat C's `sscanf %x` does; a
//!     live gateway's messages get the host's time instead.
//!   - `SRC` — source address, hex.
//!   - data bytes follow as a continuous hex string (no separators): the
//!     whole message, a fast-packet already joined.
//!   - `*XX` — the NMEA 0183 checksum, checked when present.
//!
//! A leading NMEA 0183 TAG block is skipped, and any other NMEA 0183
//! sentence, which a Quark-elec gateway mixes in, holds no message.
//!
//! Mirrors `parseRawFormatSeaSmart` in canboat/common/parse.c. Output
//! timestamp shape matches canboat: `YYYY-MM-DDTHH:MM:SS,mmm`
//! (UTC seconds + 3-digit ms).
//!
//! The sentence has no priority or destination: priority is emitted as
//! 0 (canboat does the same), and the destination as 255 (broadcast).

use std::fmt;

use smallvec::SmallVec;

use crate::engine::format::mxpgn::{is_nmea0183, nmea_checksum, without_tag_block};
use crate::engine::format::plain::ParseError;
use crate::engine::format::timestamp::days_to_ymd;
use crate::engine::frame::{FASTPACKET_MAX_SIZE, RawFrame};

/// Whether `line` is a `$PCDIN` sentence, possibly behind a TAG block.
pub fn is_pcdin(line: &str) -> bool {
    without_tag_block(line.trim_start()).starts_with("$PCDIN,")
}

/// The message a `$PCDIN` sentence carries, `Ok(None)` for another NMEA
/// 0183 sentence (a Quark-elec gateway mixes them in), or an error for a
/// malformed one, or one whose checksum does not match.
pub fn parse_line(line: &str) -> Result<Option<RawFrame>, ParseError> {
    let line = line.trim();
    if line.is_empty() {
        return Err(ParseError::Empty);
    }
    if !is_pcdin(line) {
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
        let want = u8::from_str_radix(sum.trim(), 16).map_err(|_| ParseError::BadInteger {
            field: "checksum",
            value: sum.to_string(),
            offset: None,
        })?;
        if nmea_checksum(body) != want {
            return Err(ParseError::BadInteger {
                field: "checksum",
                value: sum.to_string(),
                offset: None,
            });
        }
    }
    let rest = &body["PCDIN,".len()..];
    let mut iter = rest.splitn(4, ',');
    let pgn_tok = iter.next().ok_or(ParseError::BadHeader {
        expected: 4,
        found: 1,
    })?;
    let tstamp_tok = iter.next().ok_or(ParseError::BadHeader {
        expected: 4,
        found: 2,
    })?;
    let src_tok = iter.next().ok_or(ParseError::BadHeader {
        expected: 4,
        found: 3,
    })?;
    let data_tok = iter.next().ok_or(ParseError::BadHeader {
        expected: 4,
        found: 4,
    })?;

    let pgn = u32::from_str_radix(pgn_tok, 16).map_err(|_| ParseError::BadInteger {
        field: "pgn",
        value: pgn_tok.to_string(),
        offset: None,
    })?;
    // The tstamp is hex milliseconds; canboat's `sscanf %x`
    // stops at any non-hex byte, so accept a trailing `!` (seen in
    // canboat's own parse comment example: `089C77D!`) by taking
    // only the leading hex run.
    let tstamp_hex: String = tstamp_tok
        .chars()
        .take_while(|c| c.is_ascii_hexdigit())
        .collect();
    let tstamp_ms = u64::from_str_radix(&tstamp_hex, 16).map_err(|_| ParseError::BadInteger {
        field: "tstamp",
        value: tstamp_tok.to_string(),
        offset: None,
    })?;
    let src = u8::from_str_radix(src_tok, 16).map_err(|_| ParseError::BadInteger {
        field: "src",
        value: src_tok.to_string(),
        offset: None,
    })?;

    let data_str = data_tok;
    if data_str.len() % 2 != 0 {
        return Err(ParseError::BadHexByte {
            index: data_str.len() / 2,
            value: data_str.chars().last().unwrap_or(' ').to_string(),
            offset: None,
        });
    }
    let mut data: SmallVec<[u8; 8]> = SmallVec::new();
    for (i, chunk) in data_str.as_bytes().chunks(2).enumerate() {
        if i >= FASTPACKET_MAX_SIZE {
            break;
        }
        let s = std::str::from_utf8(chunk).map_err(|_| ParseError::BadHexByte {
            index: i,
            value: String::from_utf8_lossy(chunk).into_owned(),
            offset: None,
        })?;
        let b = u8::from_str_radix(s, 16).map_err(|_| ParseError::BadHexByte {
            index: i,
            value: s.to_string(),
            offset: None,
        })?;
        data.push(b);
    }

    Ok(Some(RawFrame {
        timestamp: Some(format_seasmart_timestamp(tstamp_ms)),
        prio: 0,
        pgn,
        src,
        dst: 0xff,
        data,
    }))
}

/// Write `frame`, a whole message, as a `$PCDIN` sentence with its
/// checksum (no line end), as a gateway is sent one to put on the bus:
/// the time stamp `00000000`, as Chetco's own switch page sends it, and
/// the source in the fourth field, which the gateway replaces with its
/// own. The sentence has no field for the priority or the destination.
pub fn write_line<W: fmt::Write>(w: &mut W, frame: &RawFrame) -> fmt::Result {
    let mut body = format!("PCDIN,{:06X},00000000,{:02X},", frame.pgn, frame.src);
    for b in &frame.data {
        body.push_str(&format!("{b:02X}"));
    }
    write!(w, "${body}*{:02X}", nmea_checksum(&body))
}

/// `<ms-since-epoch>` → `YYYY-MM-DDTHH:MM:SS,mmm`. Matches the format
/// string canboat assembles in `parseRawFormatSeaSmart`:
///
/// ```c
/// strftime(ts, n, "%Y-%m-%dT%H:%M:%S", localtime_r(&t, &tm));
/// sprintf(ts + strlen(ts), ",%3.3u", tstamp_ms % 1000);
/// ```
fn format_seasmart_timestamp(tstamp_ms: u64) -> String {
    let secs = (tstamp_ms / 1000) as i64;
    let ms = tstamp_ms % 1000;
    let days = secs.div_euclid(86_400);
    let day_seconds = secs.rem_euclid(86_400) as u32;
    let h = day_seconds / 3600;
    let m = (day_seconds / 60) % 60;
    let s = day_seconds % 60;
    let (year, mo, d) = days_to_ymd(days);
    format!("{year:04}-{mo:02}-{d:02}T{h:02}:{m:02}:{s:02},{ms:03}")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `body` (between `$` and `*`) as a sentence with its checksum.
    fn sentence(body: &str) -> String {
        format!("${body}*{:02X}", nmea_checksum(body))
    }

    fn parse(line: &str) -> RawFrame {
        parse_line(line).unwrap().unwrap()
    }

    #[test]
    fn parses_seasmart_sentence() {
        // Synthesised from canboat's own parse comment:
        //   $PCDIN,01F801,FB12345,07,A0B1C2D3E4F506*XX
        // PGN = 0x01F801 = 129025; src = 7; 7 data bytes;
        // tstamp = 0xFB12345 = 263 267 141 ms = 263 267 s + 141 ms
        //                    = 3d + 1h 7m 47s + 141 ms
        //                    → 1970-01-04T01:07:47,141.
        let f = parse(&sentence("PCDIN,01F801,FB12345,07,A0B1C2D3E4F506"));
        assert_eq!(f.pgn, 0x1f801);
        assert_eq!(f.src, 0x07);
        assert_eq!(f.dst, 0xff);
        assert_eq!(f.prio, 0);
        assert_eq!(
            f.data.as_slice(),
            &[0xa0, 0xb1, 0xc2, 0xd3, 0xe4, 0xf5, 0x06]
        );
        assert_eq!(f.timestamp.as_deref(), Some("1970-01-04T01:07:47,141"));
    }

    /// The checksum suffix is not payload — data stops at `*`.
    #[test]
    fn checksum_suffix_is_not_payload() {
        let with = parse(&sentence("PCDIN,01F801,0,07,A0B1"));
        let without = parse("$PCDIN,01F801,0,07,A0B1");
        assert_eq!(with.data.as_slice(), &[0xa0, 0xb1]);
        assert_eq!(with.data, without.data);
    }

    /// canboat's `sscanf %x` stops at the first non-hex byte, so a
    /// trailing marker on the timestamp is tolerated.
    #[test]
    fn trailing_marker_on_the_timestamp_is_tolerated() {
        let plain = parse(&sentence("PCDIN,01F801,089C77D,07,A0"));
        let marked = parse(&sentence("PCDIN,01F801,089C77D!,07,A0"));
        assert_eq!(plain.timestamp, marked.timestamp);
    }

    /// A timestamp with no leading hex run at all has nothing to parse.
    #[test]
    fn timestamp_without_hex_is_rejected() {
        assert!(matches!(
            parse_line(&sentence("PCDIN,01F801,!!!,07,A0")),
            Err(ParseError::BadInteger {
                field: "tstamp",
                ..
            })
        ));
    }

    /// PGN and src are hex, not decimal — a non-hex token is an error.
    #[test]
    fn non_hex_pgn_or_src_is_rejected() {
        assert!(matches!(
            parse_line(&sentence("PCDIN,ZZZZZZ,0,07,A0")),
            Err(ParseError::BadInteger { field: "pgn", .. })
        ));
        assert!(matches!(
            parse_line(&sentence("PCDIN,01F801,0,ZZ,A0")),
            Err(ParseError::BadInteger { field: "src", .. })
        ));
    }

    /// An odd number of hex digits can't be split into bytes.
    #[test]
    fn odd_length_data_is_rejected() {
        assert!(matches!(
            parse_line(&sentence("PCDIN,01F801,0,07,A0B")),
            Err(ParseError::BadHexByte { .. })
        ));
    }

    /// A non-hex digit in the data block is an error rather than a
    /// zero byte.
    #[test]
    fn non_hex_data_digit_is_rejected() {
        assert!(matches!(
            parse_line(&sentence("PCDIN,01F801,0,07,A0ZZ")),
            Err(ParseError::BadHexByte { .. })
        ));
    }

    /// SeaSmart lines are pre-coalesced, but never longer than the
    /// fast-packet ceiling.
    #[test]
    fn data_is_clamped_to_the_fast_packet_ceiling() {
        let hex = "ab".repeat(FASTPACKET_MAX_SIZE + 5);
        let f = parse(&sentence(&format!("PCDIN,01F801,0,07,{hex}")));
        assert_eq!(f.data.len(), FASTPACKET_MAX_SIZE);
    }

    /// An empty data block is a valid zero-length frame.
    #[test]
    fn empty_data_block_parses() {
        let f = parse(&sentence("PCDIN,01F801,0,07,"));
        assert!(f.data.is_empty());
    }

    /// A line with too few comma-separated fields is rejected.
    #[test]
    fn short_sentence_is_rejected() {
        for line in ["$PCDIN,01F801", "$PCDIN,01F801,0", "$PCDIN,01F801,0,07"] {
            assert!(
                matches!(parse_line(line), Err(ParseError::BadHeader { .. })),
                "expected BadHeader for {line}"
            );
        }
    }

    /// Blank lines are `Empty`, which callers skip.
    #[test]
    fn empty_line_is_empty() {
        assert!(matches!(parse_line(""), Err(ParseError::Empty)));
        assert!(matches!(parse_line("\r\n"), Err(ParseError::Empty)));
    }

    /// The ms component survives a day rollover.
    #[test]
    fn timestamp_rolls_over_days() {
        assert_eq!(format_seasmart_timestamp(0), "1970-01-01T00:00:00,000");
        assert_eq!(
            format_seasmart_timestamp(90_061_500),
            "1970-01-02T01:01:01,500"
        );
    }

    #[test]
    fn rejects_missing_prefix() {
        assert!(parse_line("BOGUS,01,02").is_err());
    }

    /// canboatjs's test sentence (127257 Attitude), also behind a TAG
    /// block; the sentence in canboat#1047 had a wrong checksum.
    #[test]
    fn the_canboatjs_sentence() {
        let line = "$PCDIN,01F119,00000000,0F,2AAF00D1067414FF*59";
        for line in [
            line.to_string(),
            format!("\\s:serial,c:1696759212*3E\\{line}"),
        ] {
            let f = parse(&line);
            assert_eq!((f.pgn, f.src, f.dst), (127257, 15, 255));
            assert_eq!(
                f.data.as_slice(),
                &[0x2a, 0xaf, 0x00, 0xd1, 0x06, 0x74, 0x14, 0xff]
            );
        }
        assert!(matches!(
            parse_line("$PCDIN,01F119,00000000,0F,2AAF00D1067414FF*21"),
            Err(ParseError::BadInteger {
                field: "checksum",
                ..
            })
        ));
    }

    /// A Quark-elec gateway mixes other NMEA 0183 in: no message, and
    /// no error.
    #[test]
    fn other_nmea_0183_is_no_message() {
        let rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
        assert!(matches!(parse_line(rmc), Ok(None)));
        assert!(matches!(
            parse_line("!AIVDM,1,1,,A,13aEOK?P00PD2wVMdLDRhgvL289?,0*26"),
            Ok(None)
        ));
    }

    /// What a gateway is sent: the whole message in one sentence, the time
    /// stamp zero, the source in the fourth field. It reads back as sent.
    #[test]
    fn a_message_is_written_whole() {
        let data: Vec<u8> = (0u8..43).collect();
        let frame = RawFrame::new(None, 3, 129029, 0x23, 255, data.iter().copied());
        let mut line = String::new();
        write_line(&mut line, &frame).unwrap();
        assert!(
            line.starts_with("$PCDIN,01F805,00000000,23,000102"),
            "{line}"
        );
        assert_eq!(line, sentence(&line[1..line.len() - 3]));
        let back = parse(&line);
        assert_eq!((back.pgn, back.src), (129029, 0x23));
        assert_eq!(back.data.as_slice(), data.as_slice());

        let mut line = String::new();
        let attitude = RawFrame::new(
            None,
            3,
            127257,
            15,
            255,
            [0x2a, 0xaf, 0x00, 0xd1, 0x06, 0x74, 0x14, 0xff],
        );
        write_line(&mut line, &attitude).unwrap();
        assert_eq!(line, "$PCDIN,01F119,00000000,0F,2AAF00D1067414FF*59");
    }
}
