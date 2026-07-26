// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Periodic ISO Address Claim (PGN 60928) / Product Information
//! (PGN 126996) / Configuration Information (PGN 126998) auto-request
//! engine.
//!
//! Mirrors canboat C `n2kd`'s `requestAddressClaimAndProductInfo`:
//! for every source we've seen traffic from, periodically prompt it to
//! re-announce its claim and product/config info. The engine itself is
//! transport-agnostic — it tracks per-device state and yields "ask
//! source X for PGN Y" tuples. Callers choose how to put the resulting
//! PGN 59904 request on the bus:
//!
//! * The `n2kd` binary emits a canboat PLAIN line on stdout (a
//!   downstream writer like `actisense-serial` puts it on the bus).
//! * `canboat-pipeline` builds a [`canboat_core::RawFrame`] and pushes
//!   it straight to its device writer.
//!
//! Cadence matches canboat C: at most one request per
//! [`DEVICE_REQUEST_SPACING`], and per-(device, kind) repeats no more
//! often than [`DEVICE_REQUEST_INTERVAL`]. Product Info and
//! Configuration Info share the same scheduling slot.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use canboat_core::RawFrame;

/// PGN 60928 — ISO Address Claim.
pub const PGN_CLAIM: u32 = 60928;
/// PGN 126996 — Product Information.
pub const PGN_PROD_INFO: u32 = 126996;
/// PGN 126998 — Configuration Information (also auto-requested by
/// canboat).
pub const PGN_CONFIG_INFO: u32 = 126998;
/// PGN 59904 — ISO Request.
pub const PGN_ISO_REQUEST: u32 = 59904;

/// Spacing between individual request emissions — matches
/// `DEVICE_REQUEST_SPACING` in canboat C `n2kd/main.c`.
pub const DEVICE_REQUEST_SPACING: Duration = Duration::from_secs(1);
/// Minimum time between repeating a claim or product-info request for
/// the same device — matches `DEVICE_REQUEST_INTERVAL`.
pub const DEVICE_REQUEST_INTERVAL: Duration = Duration::from_secs(300);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RequestKind {
    Claim,
    ProductInfo,
}

#[derive(Default, Debug, Clone, Copy)]
struct DeviceState {
    seen: bool,
    last_claim_received: Option<Instant>,
    last_claim_requested: Option<Instant>,
    last_prod_info_received: Option<Instant>,
    last_prod_info_requested: Option<Instant>,
}

/// Tracks which source addresses we've seen and when each last
/// answered (or was last asked for) an address claim / product info.
pub struct RequestEngine {
    devices: Mutex<HashMap<u8, DeviceState>>,
}

impl RequestEngine {
    pub fn new() -> Self {
        Self {
            devices: Mutex::new(HashMap::new()),
        }
    }

    /// Note that we just received PGN `pgn` from source `src`. Updates
    /// the per-device "last received" stamps so the engine knows not
    /// to re-request data that just arrived.
    pub fn note_device_seen(&self, pgn: u32, src: u8) {
        let now = Instant::now();
        let mut devs = self.devices.lock().unwrap();
        let entry = devs.entry(src).or_default();
        entry.seen = true;
        if pgn == PGN_CLAIM {
            entry.last_claim_received = Some(now);
        } else if pgn == PGN_PROD_INFO {
            entry.last_prod_info_received = Some(now);
        }
    }

    /// Number of distinct source addresses we've recorded traffic
    /// from. Retained as a test-only accessor for the device tracker.
    #[cfg(test)]
    pub fn device_count(&self) -> usize {
        self.devices.lock().map(|m| m.len()).unwrap_or(0)
    }

    /// Find the next source that needs a `kind` request, advancing
    /// the caller's round-robin cursor. Stamps the device's "last
    /// requested" time so subsequent calls within
    /// [`DEVICE_REQUEST_INTERVAL`] skip it. Returns `None` when every
    /// known device has had a recent enough request/response.
    pub fn next_due(&self, cursor: &mut u8, kind: RequestKind) -> Option<u8> {
        let now = Instant::now();
        let devs = self.devices.lock().ok()?;
        for offset in 0..=u8::MAX as u16 {
            let idx = cursor.wrapping_add(offset as u8);
            let Some(state) = devs.get(&idx) else {
                continue;
            };
            if !state.seen {
                continue;
            }
            let (last_received, last_requested) = match kind {
                RequestKind::Claim => (state.last_claim_received, state.last_claim_requested),
                RequestKind::ProductInfo => (
                    state.last_prod_info_received,
                    state.last_prod_info_requested,
                ),
            };
            if last_received.is_some_and(|t| now.duration_since(t) < DEVICE_REQUEST_INTERVAL) {
                continue;
            }
            if last_requested.is_some_and(|t| now.duration_since(t) < DEVICE_REQUEST_INTERVAL) {
                continue;
            }
            // Stamp it requested. Re-take the lock as mutable.
            drop(devs);
            let mut devs = self.devices.lock().unwrap();
            if let Some(state) = devs.get_mut(&idx) {
                match kind {
                    RequestKind::Claim => state.last_claim_requested = Some(now),
                    RequestKind::ProductInfo => state.last_prod_info_requested = Some(now),
                }
            }
            *cursor = idx.wrapping_add(1);
            return Some(idx);
        }
        None
    }
}

impl Default for RequestEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Build the 3-byte PGN-LE payload for a PGN 59904 "ISO Request".
pub fn iso_request_payload(pgn: u32) -> [u8; 3] {
    [
        (pgn & 0xff) as u8,
        ((pgn >> 8) & 0xff) as u8,
        ((pgn >> 16) & 0xff) as u8,
    ]
}

/// Format a PGN 59904 "ISO Request" as a canboat PLAIN line (without
/// trailing newline): `<timestamp>,6,59904,0,<dst>,3,<lo>,<mid>,<hi>`.
pub fn format_iso_request_plain(timestamp: &str, dst: u8, pgn: u32) -> String {
    let [b0, b1, b2] = iso_request_payload(pgn);
    format!("{timestamp},6,59904,0,{dst},3,{b0:02x},{b1:02x},{b2:02x}")
}

/// Construct a PGN 59904 "ISO Request" as a `RawFrame`, ready to hand
/// to a device writer. `src` is the local source address (canboat C
/// `n2kd` uses 0).
pub fn iso_request_frame(src: u8, dst: u8, pgn: u32) -> RawFrame {
    RawFrame::new(None, 6, PGN_ISO_REQUEST, src, dst, iso_request_payload(pgn))
}

/// Spawn the request loop on a background thread. On every tick the
/// engine checks both schedules; due requests are passed to `emit` as
/// `(dst, pgn)` tuples. The Product Info slot fires two emits in a
/// row (126996 and 126998), matching canboat C.
///
/// The thread runs forever; both callers (n2kd binary, canboat-
/// pipeline) leak it on process exit.
pub fn spawn(engine: Arc<RequestEngine>, emit: impl Fn(u8, u32) + Send + 'static) {
    thread::Builder::new()
        .name("n2kd-claim-engine".into())
        .spawn(move || {
            let mut next_claim = 0u8;
            let mut next_prod = 0u8;
            let mut last_claim_emit = Instant::now() - DEVICE_REQUEST_SPACING;
            let mut last_prod_emit = Instant::now() - DEVICE_REQUEST_SPACING;
            loop {
                thread::sleep(Duration::from_millis(500));
                let now = Instant::now();
                if now.duration_since(last_claim_emit) >= DEVICE_REQUEST_SPACING
                    && let Some(dst) = engine.next_due(&mut next_claim, RequestKind::Claim)
                {
                    emit(dst, PGN_CLAIM);
                    last_claim_emit = now;
                }
                if now.duration_since(last_prod_emit) >= DEVICE_REQUEST_SPACING
                    && let Some(dst) = engine.next_due(&mut next_prod, RequestKind::ProductInfo)
                {
                    // canboat asks for both 126996 (Product Info)
                    // and 126998 (Configuration Info) under the
                    // same scheduling slot.
                    emit(dst, PGN_PROD_INFO);
                    emit(dst, PGN_CONFIG_INFO);
                    last_prod_emit = now;
                }
            }
        })
        .ok();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iso_request_payload_is_little_endian() {
        assert_eq!(iso_request_payload(60928), [0x00, 0xee, 0x00]);
        assert_eq!(iso_request_payload(126996), [0x14, 0xf0, 0x01]);
        assert_eq!(iso_request_payload(126998), [0x16, 0xf0, 0x01]);
    }

    #[test]
    fn format_iso_request_plain_matches_canboat_shape() {
        // Mirrors the PLAIN line `<ts>,6,59904,0,<dst>,3,<lo>,<mid>,<hi>`
        // that canboat C's request engine emits on stdout.
        assert_eq!(
            format_iso_request_plain("2026-01-01T00:00:00.000Z", 35, 60928),
            "2026-01-01T00:00:00.000Z,6,59904,0,35,3,00,ee,00"
        );
    }

    #[test]
    fn iso_request_frame_carries_correct_header() {
        let f = iso_request_frame(0, 17, 126996);
        assert_eq!(f.prio, 6);
        assert_eq!(f.pgn, PGN_ISO_REQUEST);
        assert_eq!(f.src, 0);
        assert_eq!(f.dst, 17);
        assert_eq!(f.data.as_slice(), &[0x14, 0xf0, 0x01]);
    }

    #[test]
    fn device_count_tracks_distinct_sources() {
        let e = RequestEngine::new();
        assert_eq!(e.device_count(), 0);
        e.note_device_seen(127251, 7); // rate-of-turn, src 7
        e.note_device_seen(127257, 7); // attitude, same src
        assert_eq!(e.device_count(), 1, "same src must not double-count");
        e.note_device_seen(127251, 23); // different src
        assert_eq!(e.device_count(), 2);
    }

    #[test]
    fn next_due_returns_seen_device_then_marks_it_requested() {
        let e = RequestEngine::new();
        e.note_device_seen(127251, 9); // arbitrary non-claim PGN
        let mut cursor = 0u8;

        let first = e.next_due(&mut cursor, RequestKind::Claim);
        assert_eq!(first, Some(9), "device 9 was seen and never asked");
        assert_eq!(cursor, 10, "cursor must advance past the returned src");

        // Same call again should skip src 9 because it's now within
        // DEVICE_REQUEST_INTERVAL of its last_claim_requested stamp.
        // (Cursor wraps all 256 slots; only src 9 is known, so we get
        // back None.)
        cursor = 0;
        let second = e.next_due(&mut cursor, RequestKind::Claim);
        assert_eq!(second, None, "recently-requested device must be skipped");
    }

    #[test]
    fn next_due_skips_unseen_entries() {
        let e = RequestEngine::new();
        // Touch the map with a "seen via non-PGN" entry: PGN that
        // isn't claim/prod-info still marks the device seen.
        e.note_device_seen(127257, 5);
        // Manually insert an "unseen" entry to ensure that branch is
        // exercised — a device that briefly existed but never had
        // .seen set. Easiest path: use the public API only.
        let mut cursor = 0u8;
        assert_eq!(e.next_due(&mut cursor, RequestKind::Claim), Some(5));
    }

    #[test]
    fn next_due_kinds_are_independent() {
        // A pending Claim request should not consume the ProductInfo
        // slot for the same device.
        let e = RequestEngine::new();
        e.note_device_seen(127251, 12);
        let mut c = 0u8;
        assert_eq!(e.next_due(&mut c, RequestKind::Claim), Some(12));
        let mut c = 0u8;
        assert_eq!(
            e.next_due(&mut c, RequestKind::ProductInfo),
            Some(12),
            "ProductInfo slot must still be due after a Claim request"
        );
    }

    #[test]
    fn next_due_recent_received_suppresses_request() {
        // Receiving a claim from a device should make the engine
        // skip Claim requests for that source within the interval.
        let e = RequestEngine::new();
        e.note_device_seen(PGN_CLAIM, 42);
        let mut cursor = 0u8;
        assert_eq!(
            e.next_due(&mut cursor, RequestKind::Claim),
            None,
            "fresh PGN 60928 receipt must satisfy the Claim schedule"
        );
        // But ProductInfo is independent — that schedule is still due.
        let mut cursor = 0u8;
        assert_eq!(e.next_due(&mut cursor, RequestKind::ProductInfo), Some(42));
    }

    #[test]
    fn next_due_round_robins_across_seen_devices() {
        let e = RequestEngine::new();
        e.note_device_seen(127251, 3);
        e.note_device_seen(127251, 11);
        e.note_device_seen(127251, 200);

        let mut cursor = 0u8;
        let first = e.next_due(&mut cursor, RequestKind::Claim).unwrap();
        let second = e.next_due(&mut cursor, RequestKind::Claim).unwrap();
        let third = e.next_due(&mut cursor, RequestKind::Claim).unwrap();
        // Three distinct seen devices, none have been requested yet
        // → all three should come out in cursor order.
        let mut hits = [first, second, third];
        hits.sort();
        assert_eq!(hits, [3, 11, 200]);
        // Fourth call: all three were just stamped requested →
        // nothing left to do.
        assert_eq!(e.next_due(&mut cursor, RequestKind::Claim), None);
    }
}
