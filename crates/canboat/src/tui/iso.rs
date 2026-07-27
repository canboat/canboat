// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Outgoing PGN payload builders.
//!
//! The TUI sends two kinds of frames upstream:
//!
//! * **ISO Request (PGN 59904)** — three-byte LE PGN body, used to
//!   ask a device to (re-)emit a record (PGN 126464 PGN List, PGN
//!   126996 Product Info, …).
//!
//! * **NMEA Request group function (PGN 126208, function code 0)**
//!   — variable-length payload used to ask a device to change the
//!   transmission cadence of one of its outgoing PGNs.
//!   `Transmission interval` is a dedicated 32‑bit DURATION field
//!   inside the envelope (1 ms units; `0` = stop transmitting), so
//!   no parameter pair is needed for the standard form. The
//!   proprietary form appends two parameter pairs so the target
//!   only reacts when the Manufacturer Code + Industry Code on its
//!   own PGN schema match.
//!
//! The Command form (function code 1, parameter index 6 =
//! "Transmission interval") is **not** what real targets implement
//! for rate changes — the captured Furuno SCX‑20 setting tool
//! traffic in `../canboat/samples/scx20-setting-tool-pgn-*.raw`
//! uses Request exclusively. Schema authority for this layout is
//! `../canboat/docs/canboat.json` PGN 126208 variant
//! `nmeaRequestGroupFunction` (see comments below for the bit
//! layout); the SCX‑20 sample lines double as byte‑exact tests at
//! the bottom of this module.
//!
//! Output shape is the canboat PLAIN text line — `<ts>,<prio>,<pgn>,
//! <src>,<dst>,<len>,<hex>,...`. ISO / override frames go to the
//! server's write-only input port for bus injection; the filter control
//! PGN (262657) goes to the bidirectional filter control port. The
//! [`crate::tui::client::Writer`] routes each line to the right port by
//! PGN.

use std::time::{SystemTime, UNIX_EPOCH};

use canboat_core::format_iso_ms;

/// Our local source address — canboat C n2kd uses 0 for synthetic
/// outbound traffic and that suffices for the TUI.
pub const TUI_SRC: u8 = 0;

/// Format a canboat PLAIN line. `data` is the raw PGN payload.
pub fn format_plain(prio: u8, pgn: u32, src: u8, dst: u8, data: &[u8]) -> String {
    let ts = current_timestamp();
    let mut out = String::with_capacity(ts.len() + 16 + 4 + data.len() * 3);
    out.push_str(&ts);
    out.push(',');
    out.push_str(&prio.to_string());
    out.push(',');
    out.push_str(&pgn.to_string());
    out.push(',');
    out.push_str(&src.to_string());
    out.push(',');
    out.push_str(&dst.to_string());
    out.push(',');
    out.push_str(&data.len().to_string());
    for b in data {
        out.push(',');
        out.push_str(&format!("{b:02x}"));
    }
    out
}

/// PLAIN line for `PGN 59904` (ISO Request) addressed to `dst`, asking
/// it to emit `requested_pgn`.
pub fn iso_request(dst: u8, requested_pgn: u32) -> String {
    let payload = [
        (requested_pgn & 0xff) as u8,
        ((requested_pgn >> 8) & 0xff) as u8,
        ((requested_pgn >> 16) & 0xff) as u8,
    ];
    format_plain(6, 59904, TUI_SRC, dst, &payload)
}

/// PLAIN line for the pipeline's NMEA 0183 filter control PGN (262657),
/// **Set** form (Function 1): mute or unmute `sentence` — a 3-letter
/// formatter (e.g. `"VHW"`) or `"ALL"` for the whole source — on the
/// device currently at source address `source`. The pipeline
/// intercepts this before bus injection; it never reaches the wire.
///
/// Payload: `[Function=1, Source, s0, s1, s2, Muted, 0xff, 0xff]`.
pub fn nmea0183_filter_set(source: u8, sentence: &str, muted: bool) -> String {
    let s = sentence.as_bytes();
    let data = [
        1u8, // Function: Set
        source,
        *s.first().unwrap_or(&b' '),
        *s.get(1).unwrap_or(&b' '),
        *s.get(2).unwrap_or(&b' '),
        u8::from(muted),
        0xff,
        0xff,
    ];
    format_plain(7, 262657, TUI_SRC, 255, &data)
}

/// PLAIN line for the filter control PGN (262657), **Request** form
/// (Function 2): ask the server to (re-)send the current filter state on
/// the control connection. Only the function byte is significant; the
/// rest is `0xff` padding. See [`nmea0183_filter_set`] for the Set form.
pub fn nmea0183_filter_request() -> String {
    let data = [
        canboat_bridge::n2kd::nmea_filter::FILTER_FN_REQUEST,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
    ];
    format_plain(7, 262657, TUI_SRC, 255, &data)
}

/// PLAIN line for the server's PGN-rate-override control PGN (262658),
/// **Set** form: ask the server to set the device at source `src` to
/// transmit `pgn` every `interval_ms` ms (`0` = stop). `mfr`/`industry`
/// scope proprietary PGNs (`None` for standard). The server persists the
/// override (keyed by the device's NAME) and injects the actual PGN
/// 126208 Request; this control PGN never reaches the bus.
///
/// Payload: `[Function=1, Source, pgn(3 LE), interval(4 LE), mfr(2 LE),
/// industry]` (12 bytes).
pub fn override_set(
    src: u8,
    pgn: u32,
    interval_ms: u32,
    mfr: Option<u16>,
    industry: Option<u8>,
) -> String {
    format_plain(
        7,
        canboat_bridge::n2kd::overrides::PGN_PGN_OVERRIDE,
        TUI_SRC,
        255,
        &override_payload(
            canboat_bridge::n2kd::overrides::OV_FN_SET,
            src,
            pgn,
            interval_ms,
            mfr,
            industry,
        ),
    )
}

/// PLAIN line for the override control PGN (262658), **Delete** form:
/// remove the override for `pgn` on the device at source `src`.
pub fn override_delete(src: u8, pgn: u32) -> String {
    format_plain(
        7,
        canboat_bridge::n2kd::overrides::PGN_PGN_OVERRIDE,
        TUI_SRC,
        255,
        &override_payload(
            canboat_bridge::n2kd::overrides::OV_FN_DELETE,
            src,
            pgn,
            0,
            None,
            None,
        ),
    )
}

/// PLAIN line for the override control PGN (262658), **Request** form:
/// ask the server to (re-)send the current override state.
pub fn override_request() -> String {
    let mut data = vec![canboat_bridge::n2kd::overrides::OV_FN_REQUEST];
    data.resize(12, 0xff);
    format_plain(
        7,
        canboat_bridge::n2kd::overrides::PGN_PGN_OVERRIDE,
        TUI_SRC,
        255,
        &data,
    )
}

/// Build the 12-byte PGN 262658 payload common to Set / Delete.
fn override_payload(
    function: u8,
    src: u8,
    pgn: u32,
    interval_ms: u32,
    mfr: Option<u16>,
    industry: Option<u8>,
) -> Vec<u8> {
    let mut data = Vec::with_capacity(12);
    data.push(function);
    data.push(src);
    data.extend_from_slice(&pgn.to_le_bytes()[..3]);
    data.extend_from_slice(&interval_ms.to_le_bytes());
    data.extend_from_slice(&mfr.unwrap_or(0xffff).to_le_bytes());
    data.push(industry.unwrap_or(0xff));
    data
}

fn current_timestamp() -> String {
    let ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);
    format_iso_ms(ms)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Strip the leading ISO timestamp so the assertion is stable.
    fn no_ts(line: &str) -> &str {
        line.split_once(',').map(|(_, rest)| rest).unwrap_or(line)
    }

    #[test]
    fn nmea0183_filter_set_whole_source() {
        // Function=1, source=33 (0x21), "ALL" = 41,4c,4c, muted=1.
        let line = nmea0183_filter_set(33, "ALL", true);
        assert_eq!(no_ts(&line), "7,262657,0,255,8,01,21,41,4c,4c,01,ff,ff");
    }

    #[test]
    fn nmea0183_filter_set_one_sentence_unmute() {
        // source=35 (0x23), "VLW" = 56,4c,57, muted=0.
        let line = nmea0183_filter_set(35, "VLW", false);
        assert_eq!(no_ts(&line), "7,262657,0,255,8,01,23,56,4c,57,00,ff,ff");
    }

    #[test]
    fn nmea0183_filter_request_is_a_request_frame() {
        // Function=2, rest padding. The server recognises it via
        // `is_request_frame`, and it must NOT look like a Set.
        let line = nmea0183_filter_request();
        assert_eq!(no_ts(&line), "7,262657,0,255,8,02,ff,ff,ff,ff,ff,ff,ff");
        let frame = canboat_core::format::parse_plain(&line).unwrap();
        assert!(canboat_bridge::n2kd::nmea_filter::is_request_frame(&frame));
        assert!(!canboat_bridge::n2kd::nmea_filter::is_set_frame(&frame));
    }

    #[test]
    fn iso_request_payload_is_le_pgn() {
        // PGN 126464 little-endian = 0x00, 0xee, 0x01.
        let line = iso_request(35, 126464);
        assert!(line.ends_with(",6,59904,0,35,3,00,ee,01"));
    }

    #[test]
    fn override_set_payload_layout() {
        // Function=1, src=52, pgn=130578 (LE 12,fe,01), 1000 ms
        // (e8,03,00,00), mfr n/a (ff,ff), industry n/a (ff).
        let line = override_set(52, 130578, 1000, None, None);
        assert_eq!(
            no_ts(&line),
            "7,262658,0,255,12,01,34,12,fe,01,e8,03,00,00,ff,ff,ff"
        );
    }

    #[test]
    fn override_delete_payload_layout() {
        // Function=3, src=52, pgn=130578, interval unused (0), n/a codes.
        let line = override_delete(52, 130578);
        assert_eq!(
            no_ts(&line),
            "7,262658,0,255,12,03,34,12,fe,01,00,00,00,00,ff,ff,ff"
        );
    }

    #[test]
    fn override_request_is_a_request_frame() {
        let line = override_request();
        let frame = canboat_core::format::parse_plain(&line).unwrap();
        assert!(canboat_bridge::n2kd::overrides::is_request_frame(&frame));
        assert!(!canboat_bridge::n2kd::overrides::is_set_frame(&frame));
    }
}
