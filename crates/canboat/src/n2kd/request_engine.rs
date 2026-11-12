// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Periodic ISO Address Claim (PGN 60928) / Product Information
//! (PGN 126996) / Configuration Information (PGN 126998) auto-request
//! engine.
//!
//! For every source we've seen traffic from, prompt it to announce
//! its claim, product and configuration info, and ask again when
//! that info is older than [`DEVICE_REQUEST_INTERVAL`]. The engine
//! itself is transport-agnostic — it tracks per-device state and
//! yields "ask source X for PGN Y" tuples. Callers choose how to put
//! the resulting PGN 59904 request on the bus:
//!
//! * The `n2kd` binary emits a canboat PLAIN line on stdout (a
//!   downstream writer like `actisense-serial` puts it on the bus).
//! * `canboat-pipeline` builds a [`crate::engine::RawFrame`] and pushes
//!   it straight to its device writer.
//!
//! Pacing follows the answers, not a clock: once the bus shows
//! traffic, one broadcast request for 60928 finds every device at
//! once. After that one addressed request is out at a time, and the
//! next goes out when the device answers, refuses (PGN 59392 for that
//! PGN), or after [`REQUEST_TIMEOUT`]. An answer counts whoever asked
//! for it, so a device that recently sent its info isn't asked again.

use std::collections::HashMap;
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crate::engine::RawFrame;

/// PGN 60928 — ISO Address Claim.
pub const PGN_CLAIM: u32 = 60928;
/// PGN 126996 — Product Information.
pub const PGN_PROD_INFO: u32 = 126996;
/// PGN 126998 — Configuration Information (also auto-requested by
/// canboat).
pub const PGN_CONFIG_INFO: u32 = 126998;
/// PGN 59904 — ISO Request.
pub const PGN_ISO_REQUEST: u32 = 59904;
/// PGN 59392 — ISO Acknowledgement, a device's answer when it can't
/// or won't send the requested PGN.
pub const PGN_ISO_ACK: u32 = 59392;

/// How long to wait for an answer before sending the next request.
/// J1939 gives a device 1.25 s to respond; the rest is room for a
/// gateway in between.
pub const REQUEST_TIMEOUT: Duration = Duration::from_secs(2);
/// Minimum time between repeating a claim, product-info or
/// configuration-info request for the same device — matches
/// `DEVICE_REQUEST_INTERVAL` in canboat C `n2kd/main.c`.
pub const DEVICE_REQUEST_INTERVAL: Duration = Duration::from_secs(300);
/// How often the request thread looks for work when nothing is due.
const IDLE_POLL: Duration = Duration::from_millis(500);

const ADDR_NULL: u8 = 254;
const ADDR_GLOBAL: u8 = 255;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RequestKind {
    Claim,
    ProductInfo,
    ConfigInfo,
}

impl RequestKind {
    /// The kinds in the order the engine asks for them.
    pub const ALL: [RequestKind; 3] = [
        RequestKind::Claim,
        RequestKind::ProductInfo,
        RequestKind::ConfigInfo,
    ];

    /// The PGN this kind requests.
    pub fn pgn(self) -> u32 {
        match self {
            RequestKind::Claim => PGN_CLAIM,
            RequestKind::ProductInfo => PGN_PROD_INFO,
            RequestKind::ConfigInfo => PGN_CONFIG_INFO,
        }
    }

    fn from_pgn(pgn: u32) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.pgn() == pgn)
    }

    fn index(self) -> usize {
        self as usize
    }
}

#[derive(Default, Debug, Clone, Copy)]
struct DeviceState {
    last_received: [Option<Instant>; 3],
    last_requested: [Option<Instant>; 3],
}

#[derive(Default)]
struct State {
    devices: HashMap<u8, DeviceState>,
    /// The `(dst, pgn)` request we are waiting on an answer for.
    outstanding: Option<(u8, u32)>,
}

/// Tracks which source addresses we've seen and when each last
/// answered (or was last asked for) an address claim, product info or
/// configuration info, and which request is waiting for an answer.
pub struct RequestEngine {
    state: Mutex<State>,
    answered: Condvar,
}

impl RequestEngine {
    pub fn new() -> Self {
        Self {
            state: Mutex::new(State::default()),
            answered: Condvar::new(),
        }
    }

    /// Note that we just received PGN `pgn` with payload `data` from
    /// source `src`. Updates the per-device "last received" stamps so
    /// the engine knows not to re-request data that just arrived, and
    /// ends the wait when this is the answer, or a PGN 59392 refusal,
    /// for the outstanding request. `data` is only read for PGN 59392;
    /// callers without the payload pass `&[]`. Sources 254 (no address)
    /// and 255 (a gateway's own messages) are not devices to ask.
    pub fn note_device_seen(&self, pgn: u32, src: u8, data: &[u8]) {
        if src >= ADDR_NULL {
            return;
        }
        let now = Instant::now();
        let mut state = self.state.lock().unwrap();
        let entry = state.devices.entry(src).or_default();
        let answers = if let Some(kind) = RequestKind::from_pgn(pgn) {
            entry.last_received[kind.index()] = Some(now);
            Some(pgn)
        } else if pgn == PGN_ISO_ACK && data.len() >= 8 {
            Some(u32::from_le_bytes([data[5], data[6], data[7], 0]))
        } else {
            None
        };
        if answers.is_some() && state.outstanding == answers.map(|p| (src, p)) {
            state.outstanding = None;
            self.answered.notify_all();
        }
    }

    /// Number of distinct source addresses we've recorded traffic
    /// from.
    pub fn device_count(&self) -> usize {
        self.state.lock().map(|s| s.devices.len()).unwrap_or(0)
    }

    /// Find the next source that needs a `kind` request, advancing
    /// the caller's round-robin cursor. Stamps the device's "last
    /// requested" time so subsequent calls within
    /// [`DEVICE_REQUEST_INTERVAL`] skip it. Returns `None` when every
    /// known device has had a recent enough request/response.
    pub fn next_due(&self, cursor: &mut u8, kind: RequestKind) -> Option<u8> {
        let now = Instant::now();
        let recent =
            |t: Option<Instant>| t.is_some_and(|t| now.duration_since(t) < DEVICE_REQUEST_INTERVAL);
        let mut state = self.state.lock().ok()?;
        for offset in 0..=u8::MAX as u16 {
            let idx = cursor.wrapping_add(offset as u8);
            let Some(dev) = state.devices.get_mut(&idx) else {
                continue;
            };
            let k = kind.index();
            if recent(dev.last_received[k]) || recent(dev.last_requested[k]) {
                continue;
            }
            dev.last_requested[k] = Some(now);
            *cursor = idx.wrapping_add(1);
            return Some(idx);
        }
        None
    }

    /// The next `(dst, pgn)` request that is due, of any kind: every
    /// claim first, then product info, then configuration info.
    /// `cursors` holds one round-robin cursor per [`RequestKind`].
    pub fn next_request(&self, cursors: &mut [u8; 3]) -> Option<(u8, u32)> {
        RequestKind::ALL.into_iter().find_map(|kind| {
            self.next_due(&mut cursors[kind.index()], kind)
                .map(|dst| (dst, kind.pgn()))
        })
    }

    /// Mark `(dst, pgn)` as the request waiting for an answer. Call it
    /// before the request goes out, so an answer that comes back
    /// quickly still counts.
    pub fn expect(&self, dst: u8, pgn: u32) {
        self.state.lock().unwrap().outstanding = Some((dst, pgn));
    }

    /// Wait until the outstanding request is answered or refused, or
    /// `timeout` has passed. Returns `true` when it was answered. A
    /// broadcast request is never answered by one device, so it waits
    /// the full `timeout`.
    pub fn wait_for_answer(&self, timeout: Duration) -> bool {
        let state = self.state.lock().unwrap();
        let (mut state, result) = self
            .answered
            .wait_timeout_while(state, timeout, |s| s.outstanding.is_some())
            .unwrap();
        state.outstanding = None;
        !result.timed_out()
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

/// Spawn the request loop on a background thread. Due requests are
/// passed to `emit` as `(dst, pgn)` tuples, one at a time: the next
/// waits for the answer to the previous one, or [`REQUEST_TIMEOUT`].
/// The first is a broadcast request for 60928, sent once the bus has
/// shown traffic.
///
/// The thread runs forever; both callers (n2kd binary, canboat-
/// pipeline) leak it on process exit.
pub fn spawn(engine: Arc<RequestEngine>, emit: impl Fn(u8, u32) + Send + 'static) {
    thread::Builder::new()
        .name("n2kd-claim-engine".into())
        .spawn(move || {
            // The writer may still be opening the device; a frame seen
            // means the bus is up and the broadcast will be heard.
            while engine.device_count() == 0 {
                thread::sleep(IDLE_POLL);
            }
            engine.expect(ADDR_GLOBAL, PGN_CLAIM);
            emit(ADDR_GLOBAL, PGN_CLAIM);
            engine.wait_for_answer(REQUEST_TIMEOUT);

            let mut cursors = [0u8; 3];
            loop {
                match engine.next_request(&mut cursors) {
                    Some((dst, pgn)) => {
                        engine.expect(dst, pgn);
                        emit(dst, pgn);
                        engine.wait_for_answer(REQUEST_TIMEOUT);
                    }
                    None => thread::sleep(IDLE_POLL),
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
        e.note_device_seen(127251, 7, &[]); // rate-of-turn, src 7
        e.note_device_seen(127257, 7, &[]); // attitude, same src
        assert_eq!(e.device_count(), 1, "same src must not double-count");
        e.note_device_seen(127251, 23, &[]); // different src
        assert_eq!(e.device_count(), 2);
    }

    #[test]
    fn next_due_returns_seen_device_then_marks_it_requested() {
        let e = RequestEngine::new();
        e.note_device_seen(127251, 9, &[]); // arbitrary non-claim PGN
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
    fn next_due_any_pgn_marks_device_seen() {
        // A PGN that isn't claim/prod-info still marks the device seen.
        let e = RequestEngine::new();
        e.note_device_seen(127257, 5, &[]);
        let mut cursor = 0u8;
        assert_eq!(e.next_due(&mut cursor, RequestKind::Claim), Some(5));
    }

    #[test]
    fn next_due_kinds_are_independent() {
        // A pending Claim request should not consume the ProductInfo
        // slot for the same device.
        let e = RequestEngine::new();
        e.note_device_seen(127251, 12, &[]);
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
        e.note_device_seen(PGN_CLAIM, 42, &[]);
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
        e.note_device_seen(127251, 3, &[]);
        e.note_device_seen(127251, 11, &[]);
        e.note_device_seen(127251, 200, &[]);

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

    #[test]
    fn null_and_global_sources_are_not_devices() {
        // An NGT-1 sends its own pseudo-PGNs from 255; a device that
        // couldn't claim sends from 254. Asking either would be a
        // broadcast.
        let e = RequestEngine::new();
        e.note_device_seen(262400, 255, &[]);
        e.note_device_seen(PGN_CLAIM, 254, &[]);
        assert_eq!(e.device_count(), 0);
        let mut cursors = [0u8; 3];
        assert_eq!(e.next_request(&mut cursors), None);
    }

    #[test]
    fn config_info_is_tracked_on_its_own() {
        // 126998 from anyone's request satisfies only the ConfigInfo
        // schedule; ProductInfo is still due.
        let e = RequestEngine::new();
        e.note_device_seen(PGN_CONFIG_INFO, 8, &[]);
        let mut c = 0u8;
        assert_eq!(e.next_due(&mut c, RequestKind::ConfigInfo), None);
        let mut c = 0u8;
        assert_eq!(e.next_due(&mut c, RequestKind::ProductInfo), Some(8));
    }

    #[test]
    fn next_request_asks_claims_then_product_then_config() {
        let e = RequestEngine::new();
        e.note_device_seen(127251, 4, &[]);
        e.note_device_seen(127251, 6, &[]);
        let mut cursors = [0u8; 3];
        let got: Vec<_> = std::iter::from_fn(|| e.next_request(&mut cursors)).collect();
        assert_eq!(
            got,
            [
                (4, PGN_CLAIM),
                (6, PGN_CLAIM),
                (4, PGN_PROD_INFO),
                (6, PGN_PROD_INFO),
                (4, PGN_CONFIG_INFO),
                (6, PGN_CONFIG_INFO),
            ]
        );
    }

    /// Note `pgn` from `src` on another thread after a short delay.
    fn answer_later(e: &Arc<RequestEngine>, pgn: u32, src: u8, data: Vec<u8>) {
        let e = Arc::clone(e);
        thread::spawn(move || {
            thread::sleep(Duration::from_millis(50));
            e.note_device_seen(pgn, src, &data);
        });
    }

    #[test]
    fn answer_ends_the_wait() {
        let e = Arc::new(RequestEngine::new());
        e.expect(9, PGN_PROD_INFO);
        answer_later(&e, PGN_PROD_INFO, 9, vec![]);
        let start = Instant::now();
        assert!(e.wait_for_answer(Duration::from_secs(5)));
        assert!(start.elapsed() < Duration::from_secs(1));
    }

    #[test]
    fn refusal_for_the_pgn_ends_the_wait() {
        let e = Arc::new(RequestEngine::new());
        e.expect(9, PGN_CONFIG_INFO);
        // 59392 NAK: control 1, group function 0xff, reserved, PGN 126998.
        let nak = vec![0x01, 0xff, 0xff, 0xff, 0xff, 0x16, 0xf0, 0x01];
        answer_later(&e, PGN_ISO_ACK, 9, nak);
        assert!(e.wait_for_answer(Duration::from_secs(5)));
    }

    #[test]
    fn other_answers_do_not_end_the_wait() {
        let e = Arc::new(RequestEngine::new());
        e.expect(9, PGN_PROD_INFO);
        // Wrong device, wrong PGN, and a refusal for another PGN.
        e.note_device_seen(PGN_PROD_INFO, 10, &[]);
        e.note_device_seen(PGN_CLAIM, 9, &[]);
        e.note_device_seen(
            PGN_ISO_ACK,
            9,
            &[0x01, 0xff, 0xff, 0xff, 0xff, 0x16, 0xf0, 0x01],
        );
        assert!(!e.wait_for_answer(Duration::from_millis(100)));
    }

    #[test]
    fn broadcast_waits_the_full_timeout() {
        let e = Arc::new(RequestEngine::new());
        e.expect(ADDR_GLOBAL, PGN_CLAIM);
        answer_later(&e, PGN_CLAIM, 9, vec![]);
        let start = Instant::now();
        assert!(!e.wait_for_answer(Duration::from_millis(300)));
        assert!(start.elapsed() >= Duration::from_millis(300));
    }

    #[test]
    fn spawn_broadcasts_then_paces_by_answers() {
        let e = Arc::new(RequestEngine::new());
        let (tx, rx) = std::sync::mpsc::channel();
        spawn(Arc::clone(&e), move |dst, pgn| {
            let _ = tx.send((dst, pgn));
        });
        // Nothing goes out before the bus shows traffic.
        assert!(rx.recv_timeout(Duration::from_millis(700)).is_err());

        e.note_device_seen(127251, 7, &[]);
        let first = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(first, (ADDR_GLOBAL, PGN_CLAIM));
        // Device 7 answers the broadcast, so it isn't asked again.
        e.note_device_seen(PGN_CLAIM, 7, &[]);

        let second = rx
            .recv_timeout(REQUEST_TIMEOUT + Duration::from_secs(1))
            .unwrap();
        assert_eq!(second, (7, PGN_PROD_INFO));
        e.note_device_seen(PGN_PROD_INFO, 7, &[]);
        // The answer sends the next request at once, not a second later.
        let third = rx.recv_timeout(Duration::from_millis(300)).unwrap();
        assert_eq!(third, (7, PGN_CONFIG_INFO));
        e.note_device_seen(PGN_CONFIG_INFO, 7, &[]);
        assert!(rx.recv_timeout(Duration::from_millis(700)).is_err());
    }
}
