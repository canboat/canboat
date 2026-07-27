// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense `.ebl` binary log **encoder**.
//!
//! An `.ebl` stream interleaves `ESC SOH … ESC LF` header records (a
//! timestamp precedes each message) with `DLE STX … DLE ETX` NGT-1
//! messages. Escaping differs by record:
//!
//! * inside a **header** only `ESC` is an escape byte (`ESC ESC` → a
//!   literal `0x1b`; `DLE`/`0x10` is literal data);
//! * inside a **message** both are (`DLE DLE` → `0x10`, `ESC ESC` →
//!   `0x1b`).
//!
//! We emit, per frame, one type-`0x03` timestamp header followed by one
//! `N2K_MSG_RECEIVED` message. The matching *decoder* lives in
//! [`crate::format::ngt1`] (`Ngt1Decoder::with_ebl`): EBL framing is a
//! superset of NGT-1 framing, so the byte state machine stays there and
//! this module reuses its framing constants.

use crate::format::ngt1::{
    DLE, EBL_TIMESTAMP, ESC, ETX, FILETIME_TO_UNIX_MS, LF, N2K_MSG_RECEIVED, SOH, STX,
};
use crate::format::timestamp::to_unix_ms;
use crate::frame::RawFrame;

/// Encode `frame` as one `.ebl` record pair — a timestamp header plus an
/// `N2K_MSG_RECEIVED` message — appended to `out`. Round-trips through
/// `Ngt1Decoder::with_ebl`.
pub fn encode_frame(frame: &RawFrame, out: &mut Vec<u8>) {
    // Timestamp header: ESC SOH 0x03 <FILETIME LE, 8 bytes> ESC LF.
    // FILETIME is 100-ns ticks since 1601; invert the decoder's
    // `ticks / 10_000 - FILETIME_TO_UNIX_MS`. A frame without a parseable
    // absolute timestamp lands on the Unix epoch (unix_ms = 0).
    let unix_ms = frame
        .timestamp
        .as_deref()
        .and_then(to_unix_ms)
        .unwrap_or(0)
        .max(0) as u64;
    let ticks = (unix_ms + FILETIME_TO_UNIX_MS).saturating_mul(10_000);
    out.push(ESC);
    out.push(SOH);
    push_header_byte(out, EBL_TIMESTAMP);
    for &b in &ticks.to_le_bytes() {
        push_header_byte(out, b);
    }
    out.push(ESC);
    out.push(LF);

    // N2K_MSG_RECEIVED message: DLE STX <cmd len payload csum> DLE ETX,
    // dual-stuffed. Payload header is `prio pgn[3 LE] dst src ts[4 LE]
    // dlen`, matching `NgtMessage::to_raw_frame`. The per-message NGT
    // timestamp (ms since gateway startup) is left 0 — the real instant
    // rides in the EBL header above.
    let mut payload = Vec::with_capacity(11 + frame.data.len());
    payload.push(frame.prio);
    payload.push((frame.pgn & 0xff) as u8);
    payload.push(((frame.pgn >> 8) & 0xff) as u8);
    payload.push(((frame.pgn >> 16) & 0xff) as u8);
    payload.push(frame.dst);
    payload.push(frame.src);
    payload.extend_from_slice(&0u32.to_le_bytes());
    payload.push(frame.data.len() as u8);
    payload.extend_from_slice(&frame.data);
    encode_message(N2K_MSG_RECEIVED, &payload, out);
}

/// Append one header-payload byte, escaping a literal `ESC` as `ESC ESC`.
/// `DLE` bytes are literal inside a header record.
fn push_header_byte(out: &mut Vec<u8>, b: u8) {
    out.push(b);
    if b == ESC {
        out.push(ESC);
    }
}

/// Append one message byte, escaping `DLE`→`DLE DLE` and `ESC`→`ESC ESC`.
fn push_message_byte(out: &mut Vec<u8>, b: u8) {
    out.push(b);
    if b == DLE {
        out.push(DLE);
    } else if b == ESC {
        out.push(ESC);
    }
}

/// Frame `(cmd, payload)` as `DLE STX … DLE ETX` with the 8-bit checksum
/// that makes `(cmd + len + payload + csum) ≡ 0 (mod 256)`, dual-stuffed.
/// Mirrors [`crate::format::ngt1::encode_ngt_message`] but escapes `ESC`
/// too, as EBL framing requires.
fn encode_message(cmd: u8, payload: &[u8], out: &mut Vec<u8>) {
    out.push(DLE);
    out.push(STX);
    push_message_byte(out, cmd);
    push_message_byte(out, payload.len() as u8);
    let mut sum = cmd.wrapping_add(payload.len() as u8);
    for &b in payload {
        push_message_byte(out, b);
        sum = sum.wrapping_add(b);
    }
    push_message_byte(out, 0u8.wrapping_sub(sum));
    out.push(DLE);
    out.push(ETX);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::ngt1::{EblHeader, Ngt1Decoder, NgtEvent};

    fn frame(data: &[u8]) -> RawFrame {
        RawFrame {
            timestamp: Some("2025-04-25T17:05:21.993Z".into()),
            prio: 3,
            pgn: 127251,
            src: 0x17,
            dst: 0xff,
            data: data.iter().copied().collect(),
        }
    }

    #[test]
    fn encode_round_trips_through_with_ebl_decoder() {
        // Include 0x10 (DLE) and 0x1b (ESC) in the data so both escape
        // paths are exercised.
        let f = frame(&[0x00, 0x10, 0x1b, 0xff, 0x10, 0x1b, 0x25, 0x02]);
        let mut wire = Vec::new();
        encode_frame(&f, &mut wire);

        let events = Ngt1Decoder::with_ebl().push_bytes(&wire);
        assert_eq!(events.len(), 2, "events: {events:?}");
        match &events[0] {
            NgtEvent::Header(EblHeader::Timestamp(ms)) => assert_eq!(*ms, 1_745_600_721_993),
            other => panic!("expected timestamp header, got {other:?}"),
        }
        match &events[1] {
            NgtEvent::Message(m) => {
                let back = m.to_raw_frame().expect("received frame");
                assert_eq!(back.prio, f.prio);
                assert_eq!(back.pgn, f.pgn);
                assert_eq!(back.src, f.src);
                assert_eq!(back.dst, f.dst);
                assert_eq!(back.data.as_slice(), f.data.as_slice());
            }
            other => panic!("expected message, got {other:?}"),
        }
    }

    #[test]
    fn encode_without_timestamp_uses_epoch() {
        let mut f = frame(&[1, 2, 3]);
        f.timestamp = None;
        let mut wire = Vec::new();
        encode_frame(&f, &mut wire);
        let events = Ngt1Decoder::with_ebl().push_bytes(&wire);
        match &events[0] {
            NgtEvent::Header(EblHeader::Timestamp(ms)) => assert_eq!(*ms, 0),
            other => panic!("got {other:?}"),
        }
    }
}
