// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Garmin CSV capture format — two flavours.
//!
//! **CSV1** (relative-timestamp header):
//!
//! ```text
//!   Sequence #,Timestamp,PGN,Name,Manufacturer,Remote Address,
//!     Local Address,Priority,Single Frame,Size,Packet
//!   0,486942,127508,Battery Status,Garmin,6,255,2,1,8,0x017505FF7FFFFFFF
//! ```
//!
//! **CSV2** (absolute-timestamp header with one extra `Processed PGN`
//! column):
//!
//! ```text
//!   Sequence #,Month_Day_Year_Hours_Minutes_Seconds_msTicks,PGN,
//!     Processed PGN,Name,Manufacturer,Remote Address,Local Address,
//!     Priority,Single Frame,Size,Packet
//!   0,6_21_2015_16_11_14_24931648,127508,Processed,Battery Status,…,
//!     6,255,2,1,8,0x017505FF7FFFFFFF
//! ```
//!
//! Mirrors `parseRawFormatGarminCSV` in canboat/common/parse.c. The
//! header line is detected separately and reported as `IsHeader` so
//! the caller can skip it (canboat's analyzer.c does the same).
//! Multi-packet messages arrive pre-coalesced in the trailing `0x…`
//! `Packet` column, so the format should be driven as
//! `MULTIPACKETS_COALESCED`.

use smallvec::SmallVec;

use crate::format::plain::ParseError;
use crate::format::timestamp::days_to_ymd;
use crate::frame::{FASTPACKET_MAX_SIZE, RawFrame};

/// Result of [`parse_line`] — either a parsed frame, a header row to
/// skip, or an error. The caller treats `IsHeader` as a no-op
/// (canboat's `RAWFORMAT_GARMIN_CSV*` detect path skips the first
/// line for the same reason).
#[derive(Debug)]
pub enum GarminCsvLine {
    Frame(RawFrame),
    Header,
}

/// CSV1 header line (relative ms-since-boot timestamps).
pub const CSV1_HEADER: &str = "Sequence #,Timestamp,PGN,Name,Manufacturer,Remote Address,Local Address,Priority,Single Frame,Size,Packet";

/// CSV2 header line (absolute timestamps + Processed PGN column).
pub const CSV2_HEADER: &str = "Sequence #,Month_Day_Year_Hours_Minutes_Seconds_msTicks,PGN,Processed PGN,Name,Manufacturer,Remote Address,Local Address,Priority,Single Frame,Size,Packet";

/// `Variant::Relative` parses CSV1, `Variant::Absolute` parses CSV2.
/// Mirrors canboat's `bool absolute` argument to
/// `parseRawFormatGarminCSV`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    Relative,
    Absolute,
}

pub fn parse_line(line: &str, variant: Variant) -> Result<GarminCsvLine, ParseError> {
    let line = line.trim_end_matches(['\r', '\n']);
    if line.is_empty() {
        return Err(ParseError::Empty);
    }
    if line == CSV1_HEADER || line == CSV2_HEADER {
        return Ok(GarminCsvLine::Header);
    }

    // Column count differs between the two variants:
    //   CSV1: seq, tstamp, pgn, name, mfg, src, dst, prio, single, size, packet
    //   CSV2: seq, tstamp, pgn, processed_pgn, name, mfg, src, dst, prio, single, size, packet
    // The `Name` / `Manufacturer` columns sometimes contain commas
    // themselves (e.g. `"Unknown\nManufacturer"`), which would break
    // a naïve `split(',')`. Canboat sidesteps this by skipping a
    // *fixed number of commas* with `findOccurrence` and then
    // sscanf'ing the trailing field block — replicate that here.
    let mut parts = line.split(',');
    let seq_tok = parts.next().ok_or(ParseError::BadHeader {
        expected: 11,
        found: 0,
    })?;
    let _seq: u32 = seq_tok.parse().map_err(|_| ParseError::BadInteger {
        field: "seq",
        value: seq_tok.to_string(),
        offset: None,
    })?;

    let tstamp_tok = parts.next().ok_or(ParseError::BadHeader {
        expected: 11,
        found: 1,
    })?;
    let pgn_tok = parts.next().ok_or(ParseError::BadHeader {
        expected: 11,
        found: 2,
    })?;
    let pgn: u32 = pgn_tok.parse().map_err(|_| ParseError::BadInteger {
        field: "pgn",
        value: pgn_tok.to_string(),
        offset: None,
    })?;

    // Skip the variant-specific extra columns:
    //   CSV1: name, mfg                (2 more before src/dst block)
    //   CSV2: processed_pgn, name, mfg (3 more)
    // Each may contain embedded commas (e.g. `"Unknown\nManufacturer"`).
    let columns_before_block = if variant == Variant::Absolute { 3 } else { 2 };
    let mut skipped = 0;
    while skipped < columns_before_block {
        parts.next().ok_or(ParseError::BadHeader {
            expected: 11,
            found: 3 + skipped,
        })?;
        skipped += 1;
    }

    // From here on: src, dst, prio, single, size, 0x<hex>
    let src_tok = parts.next().ok_or(ParseError::BadHeader {
        expected: 11,
        found: 5,
    })?;
    let dst_tok = parts.next().ok_or(ParseError::BadHeader {
        expected: 11,
        found: 6,
    })?;
    let prio_tok = parts.next().ok_or(ParseError::BadHeader {
        expected: 11,
        found: 7,
    })?;
    let _single_tok = parts.next().ok_or(ParseError::BadHeader {
        expected: 11,
        found: 8,
    })?;
    let size_tok = parts.next().ok_or(ParseError::BadHeader {
        expected: 11,
        found: 9,
    })?;
    let packet_tok = parts.next().ok_or(ParseError::BadHeader {
        expected: 11,
        found: 10,
    })?;

    let src: u8 = src_tok.parse().map_err(|_| ParseError::BadInteger {
        field: "src",
        value: src_tok.to_string(),
        offset: None,
    })?;
    let dst: u8 = dst_tok.parse().map_err(|_| ParseError::BadInteger {
        field: "dst",
        value: dst_tok.to_string(),
        offset: None,
    })?;
    let prio: u8 = prio_tok.parse().map_err(|_| ParseError::BadInteger {
        field: "prio",
        value: prio_tok.to_string(),
        offset: None,
    })?;
    let size: usize = size_tok.parse().map_err(|_| ParseError::BadInteger {
        field: "size",
        value: size_tok.to_string(),
        offset: None,
    })?;

    let hex_str = packet_tok
        .strip_prefix("0x")
        .or_else(|| packet_tok.strip_prefix("0X"))
        .unwrap_or(packet_tok);
    if hex_str.len() % 2 != 0 {
        return Err(ParseError::BadHexByte {
            index: hex_str.len() / 2,
            value: hex_str.chars().last().unwrap_or(' ').to_string(),
            offset: None,
        });
    }
    let want = size.min(FASTPACKET_MAX_SIZE);
    let mut data: SmallVec<[u8; 8]> = SmallVec::new();
    for (i, chunk) in hex_str.as_bytes().chunks(2).enumerate() {
        if i >= want {
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

    let timestamp = match variant {
        Variant::Relative => format_relative_timestamp(tstamp_tok)?,
        Variant::Absolute => format_absolute_timestamp(tstamp_tok)?,
    };

    Ok(GarminCsvLine::Frame(RawFrame {
        timestamp: Some(timestamp),
        prio,
        pgn,
        src,
        dst,
        data,
    }))
}

/// Format a relative-ms timestamp as canboat does — feed it through
/// `gmtime`-equivalent and assemble `YYYY-MM-DDTHH:MM:SS,mmm`.
fn format_relative_timestamp(ms_str: &str) -> Result<String, ParseError> {
    let ms: u64 = ms_str.parse().map_err(|_| ParseError::BadInteger {
        field: "tstamp",
        value: ms_str.to_string(),
        offset: None,
    })?;
    Ok(format_unix_ms(ms))
}

/// Format an absolute timestamp `Month_Day_Year_Hours_Minutes_Seconds_msTicks`
/// as `YYYY-MM-DDTHH:MM:SS,mmm`.
fn format_absolute_timestamp(tok: &str) -> Result<String, ParseError> {
    let mut p = tok.split('_');
    let bad = || ParseError::BadInteger {
        field: "tstamp",
        value: tok.to_string(),
        offset: None,
    };
    let mo: u32 = p.next().ok_or_else(bad)?.parse().map_err(|_| bad())?;
    let d: u32 = p.next().ok_or_else(bad)?.parse().map_err(|_| bad())?;
    let y: u32 = p.next().ok_or_else(bad)?.parse().map_err(|_| bad())?;
    let h: u32 = p.next().ok_or_else(bad)?.parse().map_err(|_| bad())?;
    let mn: u32 = p.next().ok_or_else(bad)?.parse().map_err(|_| bad())?;
    let s: u32 = p.next().ok_or_else(bad)?.parse().map_err(|_| bad())?;
    let ms: u64 = p.next().ok_or_else(bad)?.parse().map_err(|_| bad())?;
    Ok(format!(
        "{y:04}-{mo:02}-{d:02}T{h:02}:{mn:02}:{s:02},{:03}",
        ms % 1000
    ))
}

fn format_unix_ms(tstamp_ms: u64) -> String {
    let secs = (tstamp_ms / 1000) as i64;
    let ms = tstamp_ms % 1000;
    let days = secs.div_euclid(86_400);
    let day_seconds = secs.rem_euclid(86_400) as u32;
    let h = day_seconds / 3600;
    let m = (day_seconds / 60) % 60;
    let s = day_seconds % 60;
    let (y, mo, d) = days_to_ymd(days);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{m:02}:{s:02},{ms:03}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_csv1_frame() {
        let line = "0,486942,127508,Battery Status,Garmin,6,255,2,1,8,0x017505FF7FFFFFFF";
        let r = parse_line(line, Variant::Relative).unwrap();
        let f = match r {
            GarminCsvLine::Frame(f) => f,
            _ => panic!("expected frame"),
        };
        assert_eq!(f.pgn, 127508);
        assert_eq!(f.src, 6);
        assert_eq!(f.dst, 255);
        assert_eq!(f.prio, 2);
        assert_eq!(
            f.data.as_slice(),
            &[0x01, 0x75, 0x05, 0xff, 0x7f, 0xff, 0xff, 0xff]
        );
        // 486942 ms = 486 s + 942 ms = 1970-01-01T00:08:06,942.
        assert_eq!(f.timestamp.as_deref(), Some("1970-01-01T00:08:06,942"));
    }

    #[test]
    fn parses_csv2_frame() {
        let line = "0,6_21_2015_16_11_14_24931648,127508,Processed,Battery Status,Unknown,6,255,2,1,8,0x017505FF7FFFFFFF";
        let r = parse_line(line, Variant::Absolute).unwrap();
        let f = match r {
            GarminCsvLine::Frame(f) => f,
            _ => panic!("expected frame"),
        };
        assert_eq!(f.pgn, 127508);
        assert_eq!(f.src, 6);
        assert_eq!(f.prio, 2);
        // 24931648 % 1000 = 648
        assert_eq!(f.timestamp.as_deref(), Some("2015-06-21T16:11:14,648"));
    }

    #[test]
    fn header_lines_are_skipped() {
        assert!(matches!(
            parse_line(CSV1_HEADER, Variant::Relative).unwrap(),
            GarminCsvLine::Header
        ));
        assert!(matches!(
            parse_line(CSV2_HEADER, Variant::Absolute).unwrap(),
            GarminCsvLine::Header
        ));
    }
}
