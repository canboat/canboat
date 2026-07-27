// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The synthetic "CANboat: Startup" record (PGN 0x40200 / 262656).
//!
//! The canboat device-reader tools (actisense-serial, ikonvert-serial,
//! maretron-ipg, socketcan-serial) emit this as their first PGN so a
//! downstream consumer knows which producer and device a stream came
//! from. Mirrors `emitCanboatStartupRecord` in canboat/common/common.c.

use std::time::{SystemTime, UNIX_EPOCH};

use crate::format::timestamp::{days_since_epoch, days_to_ymd};
use crate::frame::RawFrame;

/// CANboat tool-specific fake-PGN base; also the startup record's PGN.
pub const CANBOAT_BEM: u32 = 0x40200;

/// Build the startup record, stamped with the current UTC time.
///
/// `version` is a `major.minor.patch` string (typically the binary's
/// `CARGO_PKG_VERSION`); `source` is the tool name and `device` the
/// opened device.
pub fn startup_record(version: &str, source: &str, device: &str) -> RawFrame {
    startup_record_at(version, source, device, now_iso())
}

/// As [`startup_record`] but with an explicit ISO-8601 timestamp
/// (used by tests so output is deterministic).
pub fn startup_record_at(version: &str, source: &str, device: &str, timestamp: String) -> RawFrame {
    let mut parts = version.split('.');
    let major: u32 = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    let minor: u32 = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    // Tolerate a trailing pre-release suffix like "1-rc1".
    let patch: u32 = parts
        .next()
        .map(|s| s.split(|c: char| !c.is_ascii_digit()).next().unwrap_or(""))
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let ver = (major * 1000 + minor * 100 + patch) as u16;

    let mut data = [0u8; 66];
    data[0] = (ver & 0xff) as u8;
    data[1] = (ver >> 8) as u8;
    copy_fixed(&mut data[2..34], source);
    copy_fixed(&mut data[34..66], device);

    RawFrame::new(Some(timestamp), 7, CANBOAT_BEM, 0, 255, data)
}

/// Copy a string into a fixed-width NUL-padded field, always leaving at
/// least one trailing NUL (matches canboat's `strncpy(dst, s, len-1)`).
fn copy_fixed(dst: &mut [u8], s: &str) {
    let n = s.len().min(dst.len() - 1);
    dst[..n].copy_from_slice(&s.as_bytes()[..n]);
}

/// Current UTC time as `YYYY-MM-DDTHH:MM:SS.mmmZ`. No chrono — civil
/// date via the Howard-Hinnant algorithm, as elsewhere in this crate.
fn now_iso() -> String {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    format_iso_ms(now.as_millis() as u64)
}

/// Format Unix milliseconds as `YYYY-MM-DDTHH:MM:SS.mmmZ` (ISO-8601
/// Zulu). Used by the EBL reader so timestamp records replay with their
/// captured wall-clock time.
pub fn format_iso_ms(ms: u64) -> String {
    let secs = (ms / 1000) as i64;
    let millis = ms % 1000;
    let days = secs.div_euclid(86_400);
    let tod = secs.rem_euclid(86_400);
    let (y, mo, d) = days_to_ymd(days);
    let (h, mi, s) = (tod / 3600, (tod % 3600) / 60, tod % 60);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}.{millis:03}Z")
}

/// Parse an ISO-8601 timestamp of the shape emitted by
/// [`format_iso_ms`] (`YYYY-MM-DDTHH:MM:SS[.mmm][Z]`) into Unix
/// milliseconds. Accepts a `T`, space, or `-` between date and time
/// (the last is canboat analyzer / n2kd's own `YYYY-MM-DD-HH:MM:SS`
/// form), makes the fractional seconds and trailing `Z` optional, and
/// rejects anything else — this isn't a full RFC-3339 parser, just
/// the specific shapes canboat analyzer / n2kd / EBL reader emit.
///
/// Returns `None` on parse failure so callers can fall back to a
/// local monotonic clock (canboat-tui's `Entry::interval` does this
/// when the analyzer JSON line didn't carry a `timestamp` field).
pub fn parse_iso_ms(s: &str) -> Option<i64> {
    let s = s.trim_end_matches('Z');
    let (date, time) = s
        .split_once('T')
        .or_else(|| s.split_once(' '))
        .or_else(|| {
            // canboat analyzer / n2kd form: `YYYY-MM-DD-HH:MM:SS[.mmm]`.
            // The date's two internal '-' come first, so the 3rd '-'
            // separates date from time.
            s.match_indices('-')
                .nth(2)
                .map(|(i, _)| (&s[..i], &s[i + 1..]))
        })?;
    // YYYY-MM-DD
    let mut date_parts = date.split('-');
    let y: i32 = date_parts.next()?.parse().ok()?;
    let mo: u32 = date_parts.next()?.parse().ok()?;
    let d: u32 = date_parts.next()?.parse().ok()?;
    if date_parts.next().is_some() {
        return None;
    }
    // HH:MM:SS[.mmm]. Left-pad or truncate the fractional part to
    // exactly 3 digits — `.5` → 500 ms, `.123456` → 123 ms.
    let (hms, millis) = match time.split_once('.') {
        Some((hms, frac)) => {
            let mut ms = 0u32;
            for (i, ch) in frac.chars().take(3).enumerate() {
                let d = ch.to_digit(10)?;
                ms += d * 10u32.pow(2 - i as u32);
            }
            (hms, ms as i64)
        }
        None => (time, 0),
    };
    let mut hms_parts = hms.split(':');
    let h: i64 = hms_parts.next()?.parse().ok()?;
    let mi: i64 = hms_parts.next()?.parse().ok()?;
    let s: i64 = hms_parts.next()?.parse().ok()?;
    if hms_parts.next().is_some() {
        return None;
    }
    let days = days_since_epoch(i64::from(y), i64::from(mo), i64::from(d))?;
    Some((days * 86_400 + h * 3600 + mi * 60 + s) * 1000 + millis)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_iso_ms_round_trips_format_iso_ms() {
        for ms in [
            0i64,
            1,
            1_000,
            1_500,
            86_400_000,
            1_735_689_600_000, // 2025-01-01T00:00:00Z
            1_782_115_200_000, // 2026-06-30T12:00:00Z
        ] {
            let s = format_iso_ms(ms as u64);
            assert_eq!(parse_iso_ms(&s), Some(ms), "roundtrip failed for ms={ms}");
        }
    }

    #[test]
    fn parse_iso_ms_accepts_optional_zulu_and_fractional() {
        // With/without Z, with/without fractional seconds.
        assert_eq!(
            parse_iso_ms("2026-01-01T00:00:00.500Z"),
            Some(1_767_225_600_500)
        );
        assert_eq!(
            parse_iso_ms("2026-01-01T00:00:00.500"),
            Some(1_767_225_600_500)
        );
        assert_eq!(parse_iso_ms("2026-01-01T00:00:00"), Some(1_767_225_600_000));
        // Space separator (some analyzer producers).
        assert_eq!(
            parse_iso_ms("2026-01-01 00:00:00.500Z"),
            Some(1_767_225_600_500)
        );
        // Short fractional (millisecond precision recovered).
        assert_eq!(
            parse_iso_ms("2026-01-01T00:00:00.5"),
            Some(1_767_225_600_500)
        );
    }

    #[test]
    fn parse_iso_ms_accepts_canboat_dash_form() {
        // canboat analyzer / n2kd JSON timestamps join date and time
        // with a '-' rather than 'T' — must parse identically.
        assert_eq!(
            parse_iso_ms("2026-01-01-00:00:00.500"),
            parse_iso_ms("2026-01-01T00:00:00.500")
        );
        assert_eq!(
            parse_iso_ms("2026-07-02-10:43:23.516"),
            parse_iso_ms("2026-07-02T10:43:23.516")
        );
        assert!(parse_iso_ms("2026-07-02-10:43:23.516").is_some());
    }

    #[test]
    fn parse_iso_ms_rejects_malformed_input() {
        assert_eq!(parse_iso_ms(""), None);
        assert_eq!(parse_iso_ms("not-a-time"), None);
        assert_eq!(parse_iso_ms("2026-13-01T00:00:00Z"), None); // month 13
        assert_eq!(parse_iso_ms("2026-01-32T00:00:00Z"), None); // day 32
        assert_eq!(parse_iso_ms("2026-01-01T00:00:00:00Z"), None); // extra part
    }

    #[test]
    fn encodes_version_and_strings() {
        let f = startup_record_at(
            "6.2.0",
            "socketcan-serial",
            "nmea2000",
            "2026-01-01T00:00:00.000Z".into(),
        );
        assert_eq!(f.pgn, CANBOAT_BEM);
        assert_eq!(f.prio, 7);
        assert_eq!(f.dst, 255);
        assert_eq!(f.data.len(), 66);
        // 6*1000 + 2*100 + 0 = 6200 = 0x1838, little-endian.
        assert_eq!(f.data[0], 0x38);
        assert_eq!(f.data[1], 0x18);
        assert_eq!(&f.data[2..18], b"socketcan-serial");
        assert_eq!(&f.data[34..42], b"nmea2000");
    }
}
