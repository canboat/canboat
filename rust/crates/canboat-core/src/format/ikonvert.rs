// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Digital Yacht iKonvert serial protocol — line-based, ASCII control
//! sentences interleaved with binary frames carrying Base64 payload.
//!
//! Two prefixes matter:
//!
//! ```text
//!   !PDGY,<pgn>,<prio>,<src>,<dst>,<sec>.<ms>,<base64 data>
//!   $PDGY,...                          # control sentences
//! ```
//!
//! Only `!PDGY,` lines carry N2K traffic; `$PDGY,` lines are
//! management traffic (init/alive/list) handled by the caller.
//!
//! Reference: `canboat/ikonvert-serial/ikonvert.h`.

use smallvec::SmallVec;

use crate::format::plain::ParseError;
use crate::frame::RawFrame;

/// One parsed line from an iKonvert stream.
#[derive(Debug, Clone)]
pub enum IkonvertLine {
    /// `!PDGY,...` — an N2K frame.
    Frame(RawFrame),
    /// `$PDGY,...` — control / status sentence (carried as-is for the
    /// caller to log or interpret).
    Control(String),
    /// Anything else — typically blank or pre-init noise.
    Other,
}

/// Parse one iKonvert line.
pub fn parse_line(line: &str) -> Result<IkonvertLine, ParseError> {
    let line = line.trim_end_matches(['\r', '\n']);
    if line.is_empty() {
        return Err(ParseError::Empty);
    }
    if let Some(rest) = line.strip_prefix("!PDGY,") {
        return parse_n2k_line(rest).map(IkonvertLine::Frame);
    }
    if let Some(rest) = line.strip_prefix("$PDGY,") {
        return Ok(IkonvertLine::Control(rest.to_string()));
    }
    Ok(IkonvertLine::Other)
}

fn parse_n2k_line(rest: &str) -> Result<RawFrame, ParseError> {
    // Fields: pgn, prio, src, dst, sec.ms, base64-data
    let mut it = rest.splitn(6, ',');
    let pgn = parse_int::<u32>(it.next(), "pgn")?;
    let prio = parse_int::<u8>(it.next(), "prio")?;
    let src = parse_int::<u8>(it.next(), "src")?;
    let dst = parse_int::<u8>(it.next(), "dst")?;
    let ts = it.next().ok_or(ParseError::BadHeader {
        expected: 6,
        found: 4,
    })?;
    let b64 = it.next().ok_or(ParseError::BadHeader {
        expected: 6,
        found: 5,
    })?;
    let data_vec = b64_decode(b64).ok_or(ParseError::BadHexByte {
        index: 0,
        value: b64.to_string(),
        offset: None,
    })?;
    let data: SmallVec<[u8; 8]> = data_vec.into_iter().collect();
    Ok(RawFrame {
        timestamp: Some(ts.to_string()),
        prio,
        pgn,
        src,
        dst,
        data,
    })
}

fn parse_int<T: std::str::FromStr>(
    raw: Option<&str>,
    field: &'static str,
) -> Result<T, ParseError> {
    let s = raw.ok_or(ParseError::BadHeader {
        expected: 6,
        found: 0,
    })?;
    s.trim().parse().map_err(|_| ParseError::BadInteger {
        field,
        value: s.to_string(),
        offset: None,
    })
}

/// Encode `bytes` as RFC 4648 Base64 with `=` padding into `out`.
/// Companion to [`b64_decode`]; kept inline for the same minimal-deps
/// reason.
pub(crate) fn b64_encode(bytes: &[u8], out: &mut String) {
    const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut i = 0;
    while i + 3 <= bytes.len() {
        let n =
            (u32::from(bytes[i]) << 16) | (u32::from(bytes[i + 1]) << 8) | u32::from(bytes[i + 2]);
        out.push(TABLE[((n >> 18) & 63) as usize] as char);
        out.push(TABLE[((n >> 12) & 63) as usize] as char);
        out.push(TABLE[((n >> 6) & 63) as usize] as char);
        out.push(TABLE[(n & 63) as usize] as char);
        i += 3;
    }
    match bytes.len() - i {
        0 => {}
        1 => {
            let n = u32::from(bytes[i]) << 16;
            out.push(TABLE[((n >> 18) & 63) as usize] as char);
            out.push(TABLE[((n >> 12) & 63) as usize] as char);
            out.push('=');
            out.push('=');
        }
        2 => {
            let n = (u32::from(bytes[i]) << 16) | (u32::from(bytes[i + 1]) << 8);
            out.push(TABLE[((n >> 18) & 63) as usize] as char);
            out.push(TABLE[((n >> 12) & 63) as usize] as char);
            out.push(TABLE[((n >> 6) & 63) as usize] as char);
            out.push('=');
        }
        _ => unreachable!(),
    }
}

/// Build the line to send to the device for a transmit request.
///
/// iKonvert's TX format is intentionally shorter than the RX format:
/// `!PDGY,<pgn>,<dst>,<base64-data>\r\n`. The device fills in prio
/// and src from its own state. CRLF terminator matches the C code.
pub fn encode_tx_frame(frame: &crate::frame::RawFrame) -> String {
    let mut out = String::with_capacity(32 + frame.data.len() * 4 / 3 + 4);
    out.push_str("!PDGY,");
    out.push_str(&frame.pgn.to_string());
    out.push(',');
    out.push_str(&frame.dst.to_string());
    out.push(',');
    b64_encode(&frame.data, &mut out);
    out.push_str("\r\n");
    out
}

/// `IKONVERT_BEM` — start of the iKonvert synthetic-PGN range. The
/// network-status heartbeat the device emits unsolicitedly (`$PDGY,
/// 000000,<load>,<errors>,<count>,<uptime>,<addr>,<rejected>`) is
/// surfaced as a synthetic frame at this PGN, matching
/// `IKONVERT_BEM` in `canboat/common/common.h`. The matching field
/// definition lives in `data/synthetic-pgns.json` as
/// `ikonvertNetworkStatus`.
pub const IKONVERT_BEM: u32 = 0x40100;

/// All fields the [`IKONVERT_BEM`] PGN exposes. Each value is
/// `Some` when the gateway can measure it; `None` leaves the
/// canboat-canonical 0xff / 0xffffffff "no data" sentinel in the
/// emitted payload. Used by [`build_network_status`].
#[derive(Debug, Clone, Copy, Default)]
pub struct NetworkStatus {
    /// CAN network load (%).
    pub load_pct: Option<u8>,
    /// Cumulative error count.
    pub errors: Option<u32>,
    /// Distinct N2K source addresses seen on the bus.
    pub device_count: Option<u8>,
    /// Seconds since the gateway started.
    pub uptime_s: Option<u32>,
    /// Gateway's claimed source address. Also used as
    /// [`RawFrame::src`] so downstream tools attribute the record
    /// to this gateway, not to the well-known broadcast slot.
    pub gateway_addr: u8,
    /// Cumulative count of TX requests the gateway refused or
    /// dropped (queue overflow, kernel ENOBUFS, etc).
    pub rejected_tx: Option<u32>,
}

/// Build a `NMEA 2000 gateway: network status` (PGN [`IKONVERT_BEM`])
/// frame from raw gateway-collected stats. Producers fill the fields
/// they can measure and leave the rest at `None`; the encoder writes
/// the canboat 0xff / 0xffffffff "no data" sentinel for each `None`.
///
/// 15-byte payload layout (matches `parseIKonvertAsciiMessage` in
/// canboat C's `ikonvert-serial/ikonvert-serial.c`): u8 load, u32 LE
/// errors, u8 device_count, u32 LE uptime_s, u8 gateway_addr, u32 LE
/// rejected_tx.
pub fn build_network_status(status: NetworkStatus, timestamp: Option<String>) -> RawFrame {
    let mut data = [0xffu8; 15];
    if let Some(v) = status.load_pct {
        data[0] = v;
    }
    if let Some(v) = status.errors {
        data[1..5].copy_from_slice(&v.to_le_bytes());
    }
    if let Some(v) = status.device_count {
        data[5] = v;
    }
    if let Some(v) = status.uptime_s {
        data[6..10].copy_from_slice(&v.to_le_bytes());
    }
    data[10] = status.gateway_addr;
    if let Some(v) = status.rejected_tx {
        data[11..15].copy_from_slice(&v.to_le_bytes());
    }
    RawFrame {
        timestamp,
        prio: 7,
        pgn: IKONVERT_BEM,
        src: status.gateway_addr,
        dst: 255,
        data: data.into_iter().collect(),
    }
}

/// Synthesize a `NMEA 2000 gateway: network status` (PGN
/// [`IKONVERT_BEM`]) frame from an iKonvert `$PDGY,000000,…` body.
/// The PGN itself is gateway-agnostic — `socketcan-serial` and any
/// other gateway producer should call [`build_network_status`]
/// directly with their own stats — this helper is iKonvert-specific
/// because it parses iKonvert's wire format.
///
/// `body` is the text after the `$PDGY,` prefix. Returns `None` for
/// the keep-alive form (all six fields empty) and for any body that
/// does not start with `000000,`. Mirrors the heartbeat branch of
/// `parseIKonvertAsciiMessage`: load and errors are written
/// unconditionally (iKonvert always reports them, even as 0xff /
/// -1), the rest only when iKonvert reports a non-zero value so the
/// N2K "no data" sentinel survives.
pub fn synthesize_network_status(body: &str, timestamp: String) -> Option<RawFrame> {
    let rest = body.strip_prefix("000000,")?;
    if rest == ",,,,," {
        return None;
    }
    let mut fields = rest.split(',');
    let load = parse_status_field(fields.next()).unwrap_or(0xff);
    let errors = parse_status_field(fields.next()).unwrap_or(-1);
    let count = parse_status_field(fields.next()).unwrap_or(0);
    let uptime = parse_status_field(fields.next()).unwrap_or(0);
    let addr = parse_status_field(fields.next()).unwrap_or(0);
    let rejected = parse_status_field(fields.next()).unwrap_or(0);
    Some(build_network_status(
        NetworkStatus {
            load_pct: Some(load as u8),
            errors: Some(errors as u32),
            device_count: (count != 0).then_some(count as u8),
            uptime_s: (uptime != 0).then_some(uptime as u32),
            gateway_addr: addr as u8,
            rejected_tx: (rejected != 0).then_some(rejected as u32),
        },
        Some(timestamp),
    ))
}

fn parse_status_field(s: Option<&str>) -> Option<i32> {
    let s = s?;
    if s.is_empty() {
        return None;
    }
    s.parse().ok()
}

/// `$PDGY,N2NET_INIT,ALL\r\n` — bring the device online with all PGNs.
pub const TX_ONLINE_ALL: &str = "$PDGY,N2NET_INIT,ALL\r\n";

/// `$PDGY,N2NET_INIT,NORMAL\r\n` — bring the device online filtered by RX list.
pub const TX_ONLINE_NORMAL: &str = "$PDGY,N2NET_INIT,NORMAL\r\n";

/// `$PDGY,N2NET_OFFLINE\r\n` — take the device offline.
pub const TX_OFFLINE: &str = "$PDGY,N2NET_OFFLINE\r\n";

/// `$PDGY,N2NET_RESET\r\n` — clear RX/TX lists.
pub const TX_RESET: &str = "$PDGY,N2NET_RESET\r\n";

/// `$PDGY,TX_LIMIT,OFF\r\n` — disable the rate limiter.
pub const TX_LIMIT_OFF: &str = "$PDGY,TX_LIMIT,OFF\r\n";

/// Minimal RFC 4648 Base64 decoder for the iKonvert payload. Returns
/// `None` on any malformed input. We carry the table inline rather
/// than depend on the `base64` crate to keep canboat-core's dep tree
/// small.
fn b64_decode(s: &str) -> Option<Vec<u8>> {
    let s = s.trim();
    if s.is_empty() {
        return Some(Vec::new());
    }
    let bytes = s.as_bytes();
    // Padding lengths must yield a 4-byte-aligned group count.
    if !bytes.len().is_multiple_of(4) {
        return None;
    }
    let mut out = Vec::with_capacity(bytes.len() / 4 * 3);
    let mut group = [0u8; 4];
    let mut pad = 0;
    for chunk in bytes.chunks(4) {
        for (i, b) in chunk.iter().enumerate() {
            group[i] = match *b {
                b'A'..=b'Z' => *b - b'A',
                b'a'..=b'z' => *b - b'a' + 26,
                b'0'..=b'9' => *b - b'0' + 52,
                b'+' => 62,
                b'/' => 63,
                b'=' => {
                    pad += 1;
                    0
                }
                _ => return None,
            };
        }
        let combined = (u32::from(group[0]) << 18)
            | (u32::from(group[1]) << 12)
            | (u32::from(group[2]) << 6)
            | u32::from(group[3]);
        out.push(((combined >> 16) & 0xff) as u8);
        out.push(((combined >> 8) & 0xff) as u8);
        out.push((combined & 0xff) as u8);
    }
    // Pop bytes that correspond to padding chars (1 pad = -1 byte,
    // 2 pad = -2 bytes).
    if pad > 0 {
        out.truncate(out.len() - pad);
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn b64_round_trip_known_vectors() {
        // RFC 4648 examples.
        assert_eq!(b64_decode("").unwrap(), b"");
        assert_eq!(b64_decode("Zg==").unwrap(), b"f");
        assert_eq!(b64_decode("Zm8=").unwrap(), b"fo");
        assert_eq!(b64_decode("Zm9v").unwrap(), b"foo");
        assert_eq!(b64_decode("Zm9vYg==").unwrap(), b"foob");
        assert_eq!(b64_decode("Zm9vYmE=").unwrap(), b"fooba");
        assert_eq!(b64_decode("Zm9vYmFy").unwrap(), b"foobar");
    }

    #[test]
    fn parses_pdgy_frame() {
        // PGN 127257 (Attitude), prio 3, src 35, dst 255, ts 12.345,
        // data = [0x01, 0x02, 0x03] → Base64 "AQID".
        let line = "!PDGY,127257,3,35,255,12.345,AQID";
        let ev = parse_line(line).unwrap();
        match ev {
            IkonvertLine::Frame(f) => {
                assert_eq!(f.pgn, 127257);
                assert_eq!(f.prio, 3);
                assert_eq!(f.src, 35);
                assert_eq!(f.dst, 255);
                assert_eq!(f.timestamp.as_deref(), Some("12.345"));
                assert_eq!(&f.data[..], &[0x01, 0x02, 0x03]);
            }
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    #[test]
    fn classifies_control_sentence() {
        let ev = parse_line("$PDGY,,000000,,,,,").unwrap();
        assert!(matches!(ev, IkonvertLine::Control(_)));
    }

    #[test]
    fn tx_frame_format_matches_canboat() {
        use crate::frame::RawFrame;
        let frame = RawFrame {
            timestamp: None,
            prio: 6,
            pgn: 60928,
            src: 0,
            dst: 255,
            data: smallvec::smallvec![0x01, 0x02, 0x03],
        };
        // canboat C uses TX_PGN_MSG_PREFIX "!PDGY,%u,%u," (pgn, dst)
        // then base64 then CRLF. prio and src are NOT included on TX.
        let line = encode_tx_frame(&frame);
        assert_eq!(line, "!PDGY,60928,255,AQID\r\n");
    }

    #[test]
    fn synthesizes_network_status_from_populated_heartbeat() {
        // `$PDGY,000000,38,1,38,753,2,0` — the form the iKonvert emits
        // once it has bus data. Values from a real capture.
        let f = synthesize_network_status("000000,38,1,38,753,2,0", "ts".into()).unwrap();
        assert_eq!(f.pgn, IKONVERT_BEM);
        assert_eq!(f.prio, 7);
        assert_eq!(f.src, 2, "src tracks the gateway address field");
        assert_eq!(f.dst, 255);
        assert_eq!(f.data.len(), 15);
        assert_eq!(
            f.data.as_slice(),
            &[
                38, // CAN load
                0x01, 0x00, 0x00, 0x00, // errors LE
                38,   // device count
                0xf1, 0x02, 0x00, 0x00, // uptime LE (753)
                2,    // gateway address
                0xff, 0xff, 0xff, 0xff, // rejected = 0 → keep 0xff
            ]
        );
    }

    #[test]
    fn keep_alive_heartbeat_does_not_synthesize() {
        // The iKonvert keep-alive form: `$PDGY,000000,,,,,,`.
        assert!(synthesize_network_status("000000,,,,,,", "ts".into()).is_none());
    }

    #[test]
    fn non_heartbeat_body_does_not_synthesize() {
        assert!(synthesize_network_status("ACK,whatever", "ts".into()).is_none());
        assert!(synthesize_network_status("TEXT,banner", "ts".into()).is_none());
    }

    #[test]
    fn build_network_status_writes_provided_fields_and_keeps_sentinel_for_none() {
        let f = build_network_status(
            NetworkStatus {
                load_pct: Some(42),
                errors: Some(0x0102_0304),
                device_count: Some(7),
                uptime_s: None, // sentinel survives
                gateway_addr: 17,
                rejected_tx: Some(0xdead_beef),
            },
            None,
        );
        assert_eq!(f.pgn, IKONVERT_BEM);
        assert_eq!(f.src, 17, "src mirrors gateway_addr");
        assert_eq!(f.dst, 255);
        assert_eq!(f.prio, 7);
        let d = &f.data[..];
        assert_eq!(d.len(), 15);
        assert_eq!(d[0], 42, "load_pct");
        assert_eq!(&d[1..5], &0x0102_0304u32.to_le_bytes());
        assert_eq!(d[5], 7, "device_count");
        // uptime: untouched 0xff sentinel.
        assert_eq!(&d[6..10], &[0xff, 0xff, 0xff, 0xff]);
        assert_eq!(d[10], 17, "gateway_addr");
        assert_eq!(&d[11..15], &0xdead_beefu32.to_le_bytes());
    }

    #[test]
    fn build_network_status_all_none_is_all_sentinel_except_gateway_addr() {
        let f = build_network_status(
            NetworkStatus {
                gateway_addr: 0,
                ..Default::default()
            },
            None,
        );
        // Every byte should be 0xff (sentinel) except byte 10 which
        // carries gateway_addr = 0.
        for (i, b) in f.data.iter().enumerate() {
            let expected = if i == 10 { 0 } else { 0xff };
            assert_eq!(
                *b, expected,
                "byte {i}: got {b:#04x}, expected {expected:#04x}",
            );
        }
    }

    #[test]
    fn b64_encode_round_trips_known_vectors() {
        let mut out = String::new();
        b64_encode(b"foobar", &mut out);
        assert_eq!(out, "Zm9vYmFy");
        out.clear();
        b64_encode(b"fooba", &mut out);
        assert_eq!(out, "Zm9vYmE=");
        out.clear();
        b64_encode(b"foob", &mut out);
        assert_eq!(out, "Zm9vYg==");
        out.clear();
        b64_encode(b"", &mut out);
        assert_eq!(out, "");
    }
}
