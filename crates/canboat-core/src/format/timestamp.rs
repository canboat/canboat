// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Canonicalise captured timestamps for output.
//!
//! canboat's input formats carry timestamps in inconsistent shapes:
//! ISO-8601 with a `T` and trailing `Z`, the classic canboat log form
//! `YYYY-MM-DD-HH:MM:SS.mmm` (a dash between date and time), a comma as
//! the millisecond separator (Actisense / Chetco), or bare time-of-day
//! with no date at all (Actisense ASCII, Airmar). We emit a single
//! canonical form so downstream consumers — and comma-delimited PLAIN
//! lines in particular — never trip over a stray comma inside the
//! timestamp field.
//!
//! * Date-bearing → `YYYY-MM-DDTHH:MM:SS.mmmZ` (UTC: `T` separator, dot
//!   fraction, three-digit milliseconds, trailing `Z`).
//! * Time-only    → `HH:MM:SS.mmm` (no date is available to anchor it to
//!   a UTC instant, so none is invented — only the separator is fixed).
//! * Unrecognised → returned verbatim, so an odd timestamp is never
//!   lost or corrupted.

use std::borrow::Cow;

/// Normalise `ts` to canboat-rs's canonical timestamp form (see the
/// module docs). Already-canonical inputs are returned borrowed (no
/// allocation); recognised non-canonical inputs are rewritten; anything
/// unrecognised is returned unchanged.
pub fn normalize_timestamp(ts: &str) -> Cow<'_, str> {
    // Fast path: the common live-gateway form is already canonical, so
    // avoid re-parsing and re-allocating it on the hot output path.
    if is_canonical(ts) {
        return Cow::Borrowed(ts);
    }

    let t = ts.trim();
    // Date-bearing: `YYYY-MM-DD` then one of [`T`, `-`, space] then time.
    if t.len() >= 19 {
        let b = t.as_bytes();
        if b[4] == b'-'
            && b[7] == b'-'
            && matches!(b[10], b'T' | b'-' | b' ')
            && b[0..4].iter().all(u8::is_ascii_digit)
            && b[5..7].iter().all(u8::is_ascii_digit)
            && b[8..10].iter().all(u8::is_ascii_digit)
            && let Some(time) = normalize_time(&t[11..])
        {
            return Cow::Owned(format!("{}T{}Z", &t[0..10], time));
        }
    }
    // Time-only.
    if let Some(time) = normalize_time(t) {
        return Cow::Owned(time);
    }
    Cow::Borrowed(ts)
}

/// Parse a date-bearing timestamp (`YYYY-MM-DD` then `T`/`-`/space then
/// `HH:MM:SS` with an optional `.`/`,` fraction) to Unix milliseconds.
/// Returns `None` for a time-only or unrecognised value. Sans-clock —
/// derived purely from the string. Mirrors `canboat/replay/replay.c`.
pub fn to_unix_ms(ts: &str) -> Option<i64> {
    let b = ts.as_bytes();
    if b.len() < 19 {
        return None;
    }
    let take = |r: std::ops::Range<usize>| std::str::from_utf8(&b[r]).ok();
    let y: i64 = take(0..4)?.parse().ok()?;
    let mo: i64 = take(5..7)?.parse().ok()?;
    let d: i64 = take(8..10)?.parse().ok()?;
    let h: i64 = take(11..13)?.parse().ok()?;
    let mi: i64 = take(14..16)?.parse().ok()?;
    let s: i64 = take(17..19)?.parse().ok()?;
    if b[4] != b'-'
        || b[7] != b'-'
        || !matches!(b[10], b'T' | b'-' | b' ')
        || b[13] != b':'
        || b[16] != b':'
    {
        return None;
    }
    let mut ms: i64 = 0;
    if b.len() >= 23
        && (b[19] == b'.' || b[19] == b',')
        && let Some(v) = take(20..23).and_then(|s| s.parse::<i64>().ok())
    {
        ms = v;
    }
    let days = days_since_epoch(y, mo, d)?;
    Some((days * 86_400 + h * 3600 + mi * 60 + s) * 1000 + ms)
}

/// Days since the Unix epoch for a civil date. Public-domain
/// days-from-civil algorithm (Howard Hinnant); inverse of
/// [`days_to_ymd`]. Returns `None` when the arguments are out of range
/// (month 1..=12, day 1..=31).
///
/// The canonical calendar helper — every timestamp formatter/parser
/// across the workspace (converters, `output`, `startup`, `replay`,
/// n2kd, the SocketCAN/quirk synthesisers) shares this one copy rather
/// than carrying its own.
pub fn days_since_epoch(y: i64, m: i64, d: i64) -> Option<i64> {
    if !(1..=12).contains(&m) || !(1..=31).contains(&d) {
        return None;
    }
    let y = if m <= 2 { y - 1 } else { y };
    let era = (if y >= 0 { y } else { y - 399 }) / 400;
    let yoe = y - era * 400;
    let mp = if m > 2 { m - 3 } else { m + 9 };
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    Some(era * 146_097 + doe - 719_468)
}

/// Civil `(year, month, day)` for a count of days since 1970-01-01.
/// Public-domain civil-from-days algorithm (Howard Hinnant); the
/// inverse of [`days_since_epoch`]. The shared copy for the whole
/// workspace (see that function's note).
pub fn days_to_ymd(days: i64) -> (i32, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = (yoe + era * 400) as i32;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = (if mp < 10 { mp + 3 } else { mp - 9 }) as u32;
    let y = if m <= 2 { y + 1 } else { y };
    (y, m, d)
}

/// Extract the `HH:MM:SS.mmm` time-of-day from a timestamp for the
/// time-only wire formats (YDWG02, Actisense ASCII). The input is first
/// run through [`normalize_timestamp`], so a date-bearing value yields
/// its time part and a bare time is normalised in place. Returns
/// `00:00:00.000` when the source is absent or unrecognised.
pub fn time_of_day(ts: Option<&str>) -> String {
    let Some(raw) = ts else {
        return "00:00:00.000".to_string();
    };
    let norm = normalize_timestamp(raw);
    let s = norm.as_ref();
    let b = s.as_bytes();
    // Date-bearing canonical `YYYY-MM-DDTHH:MM:SS.mmmZ`: time at [11..23].
    if b.len() == 24 && b[10] == b'T' {
        return s[11..23].to_string();
    }
    // Time-only canonical `HH:MM:SS.mmm`.
    if b.len() == 12 && b[2] == b':' {
        return s.to_string();
    }
    "00:00:00.000".to_string()
}

/// True when `ts` is already exactly `YYYY-MM-DDTHH:MM:SS.mmmZ`.
fn is_canonical(ts: &str) -> bool {
    let b = ts.as_bytes();
    b.len() == 24
        && b[4] == b'-'
        && b[7] == b'-'
        && b[10] == b'T'
        && b[13] == b':'
        && b[16] == b':'
        && b[19] == b'.'
        && b[23] == b'Z'
        && b[0..4].iter().all(u8::is_ascii_digit)
        && b[5..7].iter().all(u8::is_ascii_digit)
        && b[8..10].iter().all(u8::is_ascii_digit)
        && b[11..13].iter().all(u8::is_ascii_digit)
        && b[14..16].iter().all(u8::is_ascii_digit)
        && b[17..19].iter().all(u8::is_ascii_digit)
        && b[20..23].iter().all(u8::is_ascii_digit)
}

/// Parse `HH:MM:SS` with an optional `.`/`,` fraction and reformat as
/// `HH:MM:SS.mmm`. Returns `None` when the shape or field ranges don't
/// hold, so the caller can fall back to emitting the value verbatim.
fn normalize_time(s: &str) -> Option<String> {
    let b = s.as_bytes();
    if b.len() < 8 || b[2] != b':' || b[5] != b':' {
        return None;
    }
    if !b[0..2].iter().all(u8::is_ascii_digit)
        || !b[3..5].iter().all(u8::is_ascii_digit)
        || !b[6..8].iter().all(u8::is_ascii_digit)
    {
        return None;
    }
    let h: u32 = s[0..2].parse().ok()?;
    let m: u32 = s[3..5].parse().ok()?;
    let sec: u32 = s[6..8].parse().ok()?;
    if h >= 24 || m >= 60 || sec >= 60 {
        return None;
    }
    // Optional fractional seconds: a `.`/`,` at index 8 then digits.
    // Anything after the digit run (a trailing `Z`, a timezone) is
    // dropped — canboat timestamps are UTC-naive.
    let mut ms = String::new();
    if b.len() > 8 {
        if b[8] != b'.' && b[8] != b',' {
            return None;
        }
        for &c in &b[9..] {
            if c.is_ascii_digit() {
                ms.push(c as char);
            } else {
                break;
            }
        }
    }
    // Pad/truncate the fraction to exactly three digits (milliseconds):
    // `.1` → `100`, `.107` → `107`, `.10734` → `107`.
    while ms.len() < 3 {
        ms.push('0');
    }
    Some(format!(
        "{}:{}:{}.{}",
        &s[0..2],
        &s[3..5],
        &s[6..8],
        &ms[..3]
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn already_canonical_is_borrowed() {
        let ts = "2026-07-06T23:24:29.065Z";
        assert!(matches!(normalize_timestamp(ts), Cow::Borrowed(_)));
        assert_eq!(normalize_timestamp(ts), ts);
    }

    #[test]
    fn classic_canboat_log_form() {
        // Dash between date and time, dot millis, no T, no Z.
        assert_eq!(
            normalize_timestamp("2011-04-25-06:25:03.603"),
            "2011-04-25T06:25:03.603Z"
        );
    }

    #[test]
    fn iso_with_comma_millis_gets_dot_and_z() {
        assert_eq!(
            normalize_timestamp("2018-10-16T22:25:25,166"),
            "2018-10-16T22:25:25.166Z"
        );
    }

    #[test]
    fn iso_missing_z_gets_z() {
        assert_eq!(
            normalize_timestamp("2018-10-16T22:25:25.166"),
            "2018-10-16T22:25:25.166Z"
        );
    }

    #[test]
    fn date_time_without_fraction_pads_millis() {
        assert_eq!(
            normalize_timestamp("2011-04-25-06:25:03"),
            "2011-04-25T06:25:03.000Z"
        );
    }

    #[test]
    fn actisense_time_only_comma() {
        // The case that started this: comma would corrupt a PLAIN line.
        assert_eq!(normalize_timestamp("17:33:21,107"), "17:33:21.107");
    }

    #[test]
    fn airmar_time_only_no_fraction() {
        assert_eq!(normalize_timestamp("20:11:00"), "20:11:00.000");
    }

    #[test]
    fn short_fraction_pads_right() {
        // `.1` is a tenth of a second = 100 ms, not 1 ms.
        assert_eq!(normalize_timestamp("00:00:57,1"), "00:00:57.100");
    }

    #[test]
    fn overlong_fraction_truncates_to_millis() {
        assert_eq!(normalize_timestamp("00:00:57.123456"), "00:00:57.123");
    }

    #[test]
    fn unrecognised_passes_through() {
        assert_eq!(normalize_timestamp("not a timestamp"), "not a timestamp");
        assert_eq!(normalize_timestamp(""), "");
    }

    #[test]
    fn out_of_range_time_is_left_verbatim() {
        // 25:00:00 is not a valid time — don't pretend to normalise it.
        assert_eq!(normalize_timestamp("25:00:00"), "25:00:00");
    }

    #[test]
    fn time_of_day_from_date_bearing() {
        assert_eq!(time_of_day(Some("2011-04-25-06:25:03.603")), "06:25:03.603");
        assert_eq!(
            time_of_day(Some("2026-07-06T23:24:29.065Z")),
            "23:24:29.065"
        );
    }

    #[test]
    fn time_of_day_from_time_only_and_absent() {
        assert_eq!(time_of_day(Some("17:33:21,107")), "17:33:21.107");
        assert_eq!(time_of_day(Some("20:11:00")), "20:11:00.000");
        assert_eq!(time_of_day(None), "00:00:00.000");
        assert_eq!(time_of_day(Some("garbage")), "00:00:00.000");
    }

    #[test]
    fn to_unix_ms_known_values() {
        assert_eq!(to_unix_ms("1970-01-01T00:00:00.000"), Some(0));
        assert_eq!(to_unix_ms("1970-01-01-00:00:01.000"), Some(1000));
        // 2025-04-25 17:05:21.993Z — the EBL sample instant.
        assert_eq!(
            to_unix_ms("2025-04-25T17:05:21.993Z"),
            Some(1_745_600_721_993)
        );
        // Time-only / unrecognised → None.
        assert_eq!(to_unix_ms("17:33:21.107"), None);
        assert_eq!(to_unix_ms("garbage"), None);
    }

    #[test]
    fn days_to_ymd_known_values() {
        assert_eq!(days_to_ymd(0), (1970, 1, 1));
        assert_eq!(days_to_ymd(1), (1970, 1, 2));
        assert_eq!(days_to_ymd(19245), (2022, 9, 10));
        assert_eq!(days_to_ymd(19782), (2024, 2, 29)); // leap day
    }

    #[test]
    fn days_round_trip_through_epoch() {
        for &(y, m, d) in &[(1970, 1, 1), (2024, 2, 29), (2026, 7, 9), (1601, 1, 1)] {
            let days = days_since_epoch(y, m, d).unwrap();
            assert_eq!(days_to_ymd(days), (y as i32, m as u32, d as u32));
        }
    }
}
