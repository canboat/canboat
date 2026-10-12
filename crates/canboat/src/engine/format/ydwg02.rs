// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Yacht Devices YDWG-02 / YDEN line format.
//!
//! Example line (from YDWG02 docs):
//!
//! ```text
//!   00:17:55.475 R 0DF50B23 FF FF FF FF FF 00 00 FF
//! ```
//!
//! Layout (whitespace-separated):
//!   1. `<HH:MM:SS.mmm>` — time-of-day, no date.
//!   2. Direction marker (`R` = received, `T` = transmitted). We
//!      preserve received frames; transmit lines are skipped by the
//!      caller (return value carries direction).
//!   3. `<CANID>` — 8-hex-digit ISO 11783 29-bit CAN identifier
//!      (prio in bits 26..28, PGN spanning bits 8..26, src in bits 0..8).
//!   4. Subsequent tokens are hex data bytes.
//!
//! Mirrors `parseRawFormatYDWG02` in canboat/common/parse.c.

use std::fmt;

use smallvec::SmallVec;

use crate::engine::format::common::{iso11783_compose, iso11783_decompose};
use crate::engine::format::plain::ParseError;
use crate::engine::format::timestamp::{days_to_ymd, time_of_day};
use crate::engine::frame::RawFrame;

/// Encode `frame` as a YDWG-02 / YDEN *received* line (no trailing
/// newline): `<HH:MM:SS.mmm> R <CANID:08X> <HH HH …>`, upper-case hex,
/// one space-separated byte each. The inverse of [`parse_line`] for the
/// receive direction. Mirrors the shape in `parseRawFormatYDWG02`.
pub fn write_line<W: fmt::Write>(w: &mut W, frame: &RawFrame) -> fmt::Result {
    let canid = iso11783_compose(frame.prio, frame.pgn, frame.src, frame.dst);
    write!(
        w,
        "{} R {canid:08X}",
        time_of_day(frame.timestamp.as_deref())
    )?;
    for b in &frame.data {
        write!(w, " {b:02X}")?;
    }
    Ok(())
}

pub fn parse_line(line: &str) -> Result<RawFrame, ParseError> {
    let line = line.trim_end_matches(['\r', '\n']);
    if line.is_empty() {
        return Err(ParseError::Empty);
    }
    let mut toks = line.split_whitespace();
    let time_tok = toks.next().ok_or(ParseError::MissingTimestamp)?;
    // YDWG02 carries time-of-day only — synthesize an ISO date from
    // the host's local clock so downstream emitters can produce the
    // same `<YYYY-MM-DD>T<HH:MM:SS.mmm>` shape canboat C does
    // (`parse.c:parseRawFormatYDWG02`).
    let timestamp = synth_iso_timestamp(time_tok);
    let _dir = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 1,
    })?;
    let canid_tok = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 2,
    })?;
    let canid = u32::from_str_radix(canid_tok, 16).map_err(|_| ParseError::BadInteger {
        field: "canid",
        value: canid_tok.to_string(),
        offset: None,
    })?;
    let (prio, pgn, src, dst) = iso11783_decompose(canid);

    let mut data: SmallVec<[u8; 8]> = SmallVec::new();
    for (i, t) in toks.enumerate() {
        // One CAN frame per line. More than 8 bytes means two lines were
        // glued together (a lost datagram or a reconnect); the reassembler
        // would decode it as a coalesced message.
        if i == 8 {
            return Err(ParseError::BadPayloadCount {
                expected: 8,
                found: line.split_whitespace().count() - 3,
            });
        }
        let b = u8::from_str_radix(t, 16).map_err(|_| ParseError::BadHexByte {
            index: i,
            value: t.to_string(),
            offset: None,
        })?;
        data.push(b);
    }

    Ok(RawFrame {
        timestamp: Some(timestamp),
        prio,
        pgn,
        src,
        dst,
        data,
    })
}

/// Synthesize `<YYYY-MM-DD>T<time>` from the host clock: the gateway
/// sends only the time of day. See [`date_for_time_of_day`].
#[cfg(not(target_arch = "wasm32"))]
fn synth_iso_timestamp(time: &str) -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let now_ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .ok()
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0);
    let (y, m, d) = days_to_ymd(date_for_time_of_day(time, now_ms));
    format!("{y:04}-{m:02}-{d:02}T{time}")
}

/// The day (since 1970-01-01, UTC) a gateway's `HH:MM:SS[.fff]` time of
/// day falls on, seen from the host clock `now_ms`: yesterday, today or
/// tomorrow, whichever puts it closest to `now_ms`. Near midnight a
/// gateway clock a little behind the host's is still on yesterday, and
/// one a little ahead already on tomorrow (canboat#1039). A time that
/// does not parse is taken as today.
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
fn date_for_time_of_day(time: &str, now_ms: i64) -> i64 {
    const DAY_MS: i64 = 86_400_000;
    let today = now_ms.div_euclid(DAY_MS);
    let Some(tod_ms) = time_of_day_ms(time) else {
        return today;
    };
    (today - 1..=today + 1)
        .min_by_key(|day| (day * DAY_MS + tod_ms - now_ms).abs())
        .unwrap_or(today)
}

/// Milliseconds since midnight of `HH:MM:SS[.fff]`.
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
fn time_of_day_ms(time: &str) -> Option<i64> {
    let (hms, frac) = time.split_once('.').unwrap_or((time, ""));
    let mut parts = hms.split(':').map(|p| p.parse::<u32>().ok().map(i64::from));
    let (h, m, s) = (parts.next()??, parts.next()??, parts.next()??);
    if parts.next().is_some() || h > 23 || m > 59 || s > 60 {
        return None;
    }
    let ms = if frac.is_empty() {
        0
    } else {
        let digits: String = frac.chars().chain("000".chars()).take(3).collect();
        digits.parse::<i64>().ok()?
    };
    Some(((h * 60 + m) * 60 + s) * 1000 + ms)
}

/// wasm32-unknown-unknown has no host clock — `SystemTime::now()`
/// PANICS there rather than erroring. Keep the `<date>T<time>` shape
/// every consumer expects and degrade only the date to the epoch, the
/// same result the native path produces when the clock reads as 0.
#[cfg(target_arch = "wasm32")]
fn synth_iso_timestamp(time: &str) -> String {
    format!("1970-01-01T{time}")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 2026-10-12T00:00:00.000Z, in ms since the epoch.
    const MIDNIGHT_MS: i64 = 1_791_763_200_000;

    fn date_at(time: &str, now_ms: i64) -> (i32, u32, u32) {
        days_to_ymd(date_for_time_of_day(time, now_ms))
    }

    /// canboat#1039: a gateway clock behind the host's stays on
    /// yesterday after the host's midnight, rather than jumping a day
    /// ahead; one ahead of it is on tomorrow before the host's midnight.
    #[test]
    fn the_date_follows_the_gateway_across_midnight() {
        assert_eq!(date_at("23:59:59.161", MIDNIGHT_MS + 100), (2026, 10, 11));
        assert_eq!(date_at("00:00:00.068", MIDNIGHT_MS + 900), (2026, 10, 12));
        assert_eq!(date_at("00:00:00.300", MIDNIGHT_MS - 200), (2026, 10, 12));
        assert_eq!(date_at("23:59:59.800", MIDNIGHT_MS - 200), (2026, 10, 11));
        // Midday: today, however far apart the clocks are.
        let noon = MIDNIGHT_MS + 12 * 3_600_000;
        assert_eq!(date_at("11:00:00", noon), (2026, 10, 12));
        assert_eq!(date_at("13:30:00.5", noon), (2026, 10, 12));
        // Unparseable: today.
        assert_eq!(date_at("garbage", MIDNIGHT_MS + 100), (2026, 10, 12));
        assert_eq!(date_at("-1:00:00", MIDNIGHT_MS - 1_800_000), (2026, 10, 11));
    }

    #[test]
    fn parses_ydwg02_pdu2_frame() {
        // CAN id 0x0DF50B23 — bits laid out as
        //   prio (3) | RDP (2) | PF (8) | PS (8) | SA (8)
        // = 011_01_11110101_00001011_00100011
        //   prio=3, RDP=1, PF=0xF5, PS=0x0B, SA=0x23.
        // PF >= 240 → PDU2; PGN = (RDP<<16) | (PF<<8) | PS = 0x1F50B
        // = 128267 (canboat's "Position, Rapid Update"); dst=255.
        let line = "00:17:55.475 R 0DF50B23 FF FF FF FF FF 00 00 FF";
        let f = parse_line(line).unwrap();
        // YDWG02 synthesises an ISO date from the host clock; the time
        // portion must round-trip verbatim regardless of which day the
        // test runs.
        assert!(
            f.timestamp
                .as_deref()
                .is_some_and(|t| t.ends_with("T00:17:55.475")),
            "timestamp: {:?}",
            f.timestamp
        );
        assert_eq!(f.prio, 3);
        assert_eq!(f.pgn, 0x1f50b);
        assert_eq!(f.src, 0x23);
        assert_eq!(f.dst, 0xff);
        assert_eq!(f.data.len(), 8);
    }

    #[test]
    fn pdu1_keeps_destination() {
        // PF=0xEE (<240) → PDU1; PS becomes dst, PGN = 0xEE00.
        // 0x18EEFF05 → prio=6, PF=0xEE, PS=0xFF, src=0x05.
        let line = "00:00:00.000 R 18EEFF05 01 02 03 04 05 06 07 08";
        let f = parse_line(line).unwrap();
        assert_eq!(f.prio, 6);
        assert_eq!(f.pgn, 0xee00);
        assert_eq!(f.dst, 0xff);
        assert_eq!(f.src, 0x05);
    }

    #[test]
    fn rejects_more_than_eight_data_bytes() {
        // The head of one 129029 line glued to the tail of the next
        // (canboat/canboatjs#497).
        let line = "17:31:22.999 R 0DF80514 60 2F FC 25 C0 8A 5C 84 41 6A";
        assert!(matches!(
            parse_line(line),
            Err(ParseError::BadPayloadCount {
                expected: 8,
                found: 10
            })
        ));
    }

    #[test]
    fn write_line_round_trips_through_parse() {
        let frame = RawFrame {
            timestamp: Some("2026-01-01T00:17:55.475Z".into()),
            prio: 3,
            pgn: 0x1f50b,
            src: 0x23,
            dst: 0xff,
            data: [0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff]
                .into_iter()
                .collect(),
        };
        let mut line = String::new();
        write_line(&mut line, &frame).unwrap();
        assert_eq!(line, "00:17:55.475 R 0DF50B23 FF FF FF FF FF 00 00 FF");
        let back = parse_line(&line).unwrap();
        assert_eq!(
            (back.prio, back.pgn, back.src, back.dst),
            (3, 0x1f50b, 0x23, 0xff)
        );
        assert_eq!(back.data.as_slice(), frame.data.as_slice());
        assert!(back.timestamp.as_deref().unwrap().contains("00:17:55.475"));
    }

    #[test]
    fn write_line_empty_data() {
        let frame = RawFrame {
            timestamp: None,
            prio: 6,
            pgn: 0xee00,
            src: 0x05,
            dst: 0xff,
            data: SmallVec::new(),
        };
        let mut line = String::new();
        write_line(&mut line, &frame).unwrap();
        assert_eq!(line, "00:00:00.000 R 18EEFF05");
    }
}
