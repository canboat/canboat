// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Furuno SCX-20 Product-Information impersonation.
//!
//! The SCX-20 satellite compass sometimes "forgets" to answer a PGN 59904
//! ISO Request for its PGN 126996 Product Information, which breaks the
//! Furuno Setting Tool's device discovery. With `--quirk scx20` canboat
//! learns each SCX-20's source address from its Address Claims and, when
//! the bus asks for 126996, fabricates the reply *from that unit's source
//! address* on its behalf (see [[device-scx20]] /
//! `samples/scx20-setting-tool-*.raw`).
//!
//! This is impersonation — the emitted frame keeps the SCX-20's `src`, so
//! it is gated to `--socketcan` at the CLI layer (every other backend
//! rewrites the outbound source address).

use std::collections::HashMap;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use canboat_core::encode::EncodeError;
use canboat_core::{ADDR_GLOBAL, DecodedPgn, PgnDatabase, RawFrame, Units, field};

/// NMEA 2000 PGN numbers the matcher cares about.
const PGN_ISO_REQUEST: u32 = 59904;
const PGN_ISO_ADDRESS_CLAIM: u32 = 60928;
const PGN_PRODUCT_INFO: u32 = 126996;

/// ISO 11783-5 NAME fields that identify the SCX-20 specifically (not
/// just any Furuno device). All three must match.
const FURUNO_MFG: u16 = 1855;
const NAV_CLASS: u8 = 60;
const SCX20_FUNCTION: u8 = 140;

/// Throttle per impersonated source — at most one synthesised response
/// per second per known SCX-20, matching canboat C `socketcan-serial`'s
/// product-info rate limit. The Setting Tool only needs one response
/// per discovery sweep; bursts would just spam the bus.
const MIN_SYNTH_GAP: Duration = Duration::from_secs(1);

/// PGN 126996 "Product Information" field values captured from a real
/// SCX-20 (`canboat/samples/scx20-setting-tool-start.raw` and friends).
/// The Setting Tool keys its device-list entry on Model ID and Cert
/// Level, so this single captured set covers every unit's discovery
/// needs even though Model Serial Code would naturally differ. They are
/// encoded through the schema-driven [`canboat_core::encode::PgnBuilder`]
/// (see [`build_scx20_product_info`]).
///
/// The DB Version is the raw field value (`res 0.001` → 2100 = "2.100").
const SCX20_DB_VERSION: i64 = 2100;
const SCX20_PRODUCT_CODE: i64 = 10571;
const SCX20_MODEL_ID: &str = "SCX-20";
const SCX20_SOFTWARE_VERSION: &str = "2051601-01.04";
const SCX20_MODEL_VERSION: &str = "20P8235-00";
const SCX20_MODEL_SERIAL: &str = "100118101056";
const SCX20_CERTIFICATION_LEVEL: i64 = 2;
const SCX20_LOAD_EQUIVALENCY: i64 = 4;

/// Stateful SCX-20 impersonation. One entry per learned SCX-20 unit,
/// keyed by its currently claimed source address; the value is the last
/// time we synthesised a response on its behalf (for [`MIN_SYNTH_GAP`]).
pub struct Scx20 {
    learned: HashMap<u8, Instant>,
}

impl Default for Scx20 {
    fn default() -> Self {
        Self::new()
    }
}

impl Scx20 {
    pub fn new() -> Self {
        Self {
            learned: HashMap::new(),
        }
    }

    /// Address-Claim learning + Product-Info impersonation. `now` is
    /// injectable so the rate-limit branch is unit-testable without
    /// `thread::sleep`.
    pub fn process(&mut self, d: &DecodedPgn, now: Instant) -> Vec<RawFrame> {
        // 1. Learn / refresh a unit's source address from its Address
        //    Claim broadcasts. Address Claim is NOT a synthesis trigger —
        //    only a discovery channel.
        if d.pgn == PGN_ISO_ADDRESS_CLAIM {
            if let Some(name) = d.iso_name()
                && is_scx20_name(name)
            {
                // Stamp newly-discovered units with a "never-synthesised"
                // sentinel so the first matching request fires immediately
                // rather than waiting out the gap.
                self.learned
                    .entry(d.src)
                    .or_insert_with(|| now - MIN_SYNTH_GAP);
            }
            return Vec::new();
        }

        // 2. The real trigger: an ISO Request whose target PGN is 126996,
        //    addressed to either broadcast or a known SCX-20.
        if d.pgn != PGN_ISO_REQUEST {
            return Vec::new();
        }
        let target = d
            .field(field::iso_request::PGN)
            .and_then(|f| f.value.as_i64());
        if target != Some(i64::from(PGN_PRODUCT_INFO)) {
            return Vec::new();
        }

        // 3. Decide which learned SCX-20 unit(s) the request reaches. A
        //    directed request hits at most one; broadcast hits every known
        //    unit on the bus.
        let mut targets: Vec<u8> = if d.dst == ADDR_GLOBAL {
            self.learned.keys().copied().collect()
        } else if self.learned.contains_key(&d.dst) {
            vec![d.dst]
        } else {
            return Vec::new();
        };
        // Stable order is friendlier for test assertions.
        targets.sort_unstable();

        let mut out = Vec::with_capacity(targets.len());
        for src in targets {
            let last = self
                .learned
                .get_mut(&src)
                .expect("targets came from self.learned");
            if now.duration_since(*last) < MIN_SYNTH_GAP {
                continue;
            }
            *last = now;
            // Loud so it never becomes an invisible default: we are
            // impersonating another node on the bus, which is unusual
            // enough that an operator scanning the log should always see
            // it happening. `d.src`/`d.dst` identify the requester so the
            // user can correlate with their tool of choice.
            log::warn!(
                "quirk(scx20): fabricating PGN 126996 from src={src} \
                 in response to PGN 59904 request {}->{}",
                d.src,
                d.dst,
            );
            if let Some(frame) = build_scx20_product_info(src) {
                out.push(frame);
            }
        }
        out
    }
}

/// `true` iff `name` (the ISO 11783-5 NAME) identifies a Furuno
/// Navigation device with the SCX-20's specific Device Function. All
/// three NAME fields must match — manufacturer alone is too coarse, it
/// would also accept a Furuno GPS or compass.
fn is_scx20_name(name: u64) -> bool {
    let mfg = ((name >> 21) & 0x7FF) as u16;
    let function = ((name >> 40) & 0xFF) as u8;
    let class = ((name >> 49) & 0x7F) as u8;
    mfg == FURUNO_MFG && class == NAV_CLASS && function == SCX20_FUNCTION
}

/// Encode PGN 126996 for source `src` via the schema-driven message
/// encoder. Returns `None` only on an encoder error, which would be a
/// schema/library bug (logged loudly).
fn build_scx20_product_info(src: u8) -> Option<RawFrame> {
    let db = PgnDatabase::embedded(Units::Si);
    let build = || -> Result<RawFrame, EncodeError> {
        let mut b = db
            .encode("productInformation")?
            .priority(6)
            .source(src)
            .destination(ADDR_GLOBAL)
            .timestamp(now_iso());
        use field::product_information as pi;
        b.push(pi::NMEA2000_VERSION, SCX20_DB_VERSION)?;
        b.push(pi::PRODUCT_CODE, SCX20_PRODUCT_CODE)?;
        b.push(pi::MODEL_ID, SCX20_MODEL_ID)?;
        b.push(pi::SOFTWARE_VERSION_CODE, SCX20_SOFTWARE_VERSION)?;
        b.push(pi::MODEL_VERSION, SCX20_MODEL_VERSION)?;
        b.push(pi::MODEL_SERIAL_CODE, SCX20_MODEL_SERIAL)?;
        b.push(pi::CERTIFICATION_LEVEL, SCX20_CERTIFICATION_LEVEL)?;
        b.push(pi::LOAD_EQUIVALENCY, SCX20_LOAD_EQUIVALENCY)?;
        b.build()
    };
    match build() {
        Ok(frame) => Some(frame),
        Err(e) => {
            log::error!("quirk(scx20): failed to encode PGN 126996: {e}");
            None
        }
    }
}

/// `YYYY-MM-DDTHH:MM:SS.mmmZ` from the host clock — same shape canboat
/// C's `fmtTimestamp` emits.
fn now_iso() -> String {
    let ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);
    canboat_core::format_iso_ms(ms)
}

#[cfg(test)]
mod tests {
    use super::*;
    use canboat_core::{PgnDatabase, Units};

    /// The captured SCX-20 NAME from `samples/scx20-setting-tool-start.all`.
    /// LE byte order on the wire, decodes to manufacturer=1855, class=60,
    /// function=140 — the triple we match on.
    const SCX20_NAME_BYTES: [u8; 8] = [0x64, 0x43, 0xe0, 0xe7, 0x00, 0x8c, 0x78, 0xc0];

    fn name_le(bytes: [u8; 8]) -> u64 {
        u64::from_le_bytes(bytes)
    }

    fn db() -> &'static PgnDatabase {
        PgnDatabase::embedded(Units::Si)
    }

    /// Decode a raw frame into the `DecodedPgn` the quirk now consumes.
    fn decode(frame: &RawFrame) -> DecodedPgn {
        db().decode(frame).expect("decodes")
    }

    fn claim(src: u8, name: [u8; 8]) -> DecodedPgn {
        decode(&RawFrame::new(
            None,
            6,
            PGN_ISO_ADDRESS_CLAIM,
            src,
            ADDR_GLOBAL,
            name,
        ))
    }

    /// A decoded PGN 59904 ISO Request for `target_pgn`, from `src` to `dst`.
    fn iso_request(src: u8, dst: u8, target_pgn: u32) -> DecodedPgn {
        let data = [
            target_pgn as u8,
            (target_pgn >> 8) as u8,
            (target_pgn >> 16) as u8,
        ];
        decode(&RawFrame::new(None, 6, PGN_ISO_REQUEST, src, dst, data))
    }

    /// The exact 134-byte PGN 126996 payload captured from a real SCX-20
    /// (`samples/scx20-setting-tool-start.raw`). The PgnBuilder output
    /// must reproduce this byte-for-byte or the impersonation drifts from
    /// what the Setting Tool recorded — string fields NUL-padded and all.
    const CAPTURED_PRODUCT_INFO: [u8; 134] = captured_product_info();

    const fn captured_product_info() -> [u8; 134] {
        let mut d = [0u8; 134];
        d[0] = 0x34; // DB Version LE u16 = 2100
        d[1] = 0x08;
        d[2] = 0x4b; // Product Code LE u16 = 10571
        d[3] = 0x29;
        copy_bytes(&mut d, 4, b"SCX-20");
        copy_bytes(&mut d, 36, b"2051601-01.04");
        copy_bytes(&mut d, 68, b"20P8235-00");
        copy_bytes(&mut d, 100, b"100118101056");
        d[132] = 0x02; // Certification Level
        d[133] = 0x04; // Load Equivalency Number
        d
    }

    const fn copy_bytes(dst: &mut [u8; 134], off: usize, s: &[u8]) {
        let mut i = 0;
        while i < s.len() {
            dst[off + i] = s[i];
            i += 1;
        }
    }

    #[test]
    fn built_product_info_matches_captured_payload() {
        // The whole point of routing through PgnBuilder: it must still
        // reproduce the captured payload exactly. If any byte drifts (a
        // schema change, a padding change), the discovery workaround
        // silently breaks — this catches it.
        let f = build_scx20_product_info(52).expect("encodes");
        assert_eq!(f.data.as_slice(), &CAPTURED_PRODUCT_INFO);
    }

    #[test]
    fn name_match_accepts_captured_scx20() {
        assert!(is_scx20_name(name_le(SCX20_NAME_BYTES)));
    }

    #[test]
    fn name_match_rejects_non_scx20() {
        // Furuno-Navigation but different function (the same class/mfg
        // hosts non-SCX-20 Furuno devices, so function-only narrowing is
        // required).
        let mut other = SCX20_NAME_BYTES;
        other[5] = 0x8b; // function 139 instead of 140
        assert!(!is_scx20_name(name_le(other)));

        // Same function/class but Navico manufacturer (275).
        let navico_navigation_140: u64 = (17252_u64)
            | (275_u64 << 21)
            | (140_u64 << 40)
            | (60_u64 << 49)
            | (4_u64 << 60)
            | (1_u64 << 63);
        assert!(!is_scx20_name(navico_navigation_140));

        // Furuno, but in Instrumentation class instead of Navigation.
        let furuno_instrumentation: u64 = (17252_u64)
            | (1855_u64 << 21)
            | (140_u64 << 40)
            | (80_u64 << 49) // class 80 = Instrumentation
            | (4_u64 << 60)
            | (1_u64 << 63);
        assert!(!is_scx20_name(furuno_instrumentation));
    }

    #[test]
    fn product_info_frame_header_is_correct() {
        let f = build_scx20_product_info(52).expect("encodes");
        assert_eq!(f.prio, 6);
        assert_eq!(f.pgn, PGN_PRODUCT_INFO);
        assert_eq!(f.src, 52);
        assert_eq!(f.dst, ADDR_GLOBAL);
        assert_eq!(f.data.len(), 134);
        assert!(f.timestamp.is_some());
    }

    #[test]
    fn address_claim_alone_does_not_synthesise() {
        let mut q = Scx20::new();
        let out = q.process(&claim(52, SCX20_NAME_BYTES), Instant::now());
        assert!(
            out.is_empty(),
            "Address Claim is a discovery channel, not a trigger"
        );
    }

    #[test]
    fn request_before_any_claim_is_ignored() {
        let mut q = Scx20::new();
        // No SCX-20 learned yet, broadcast request → nothing.
        let out = q.process(
            &iso_request(7, ADDR_GLOBAL, PGN_PRODUCT_INFO),
            Instant::now(),
        );
        assert!(out.is_empty());
    }

    #[test]
    fn broadcast_request_after_claim_synthesises_one_per_unit() {
        let mut q = Scx20::new();
        // Learn two units, e.g. on a multi-SCX-20 bus.
        let t0 = Instant::now();
        q.process(&claim(52, SCX20_NAME_BYTES), t0);
        q.process(&claim(99, SCX20_NAME_BYTES), t0);
        // Broadcast request for 126996 — both should answer.
        let out = q.process(&iso_request(7, ADDR_GLOBAL, PGN_PRODUCT_INFO), t0);
        let srcs: Vec<u8> = out.iter().map(|f| f.src).collect();
        assert_eq!(srcs, vec![52, 99]);
        assert!(out.iter().all(|f| f.pgn == PGN_PRODUCT_INFO));
        assert!(out.iter().all(|f| f.dst == ADDR_GLOBAL));
    }

    #[test]
    fn directed_request_to_unknown_dst_is_ignored() {
        let mut q = Scx20::new();
        let t0 = Instant::now();
        q.process(&claim(52, SCX20_NAME_BYTES), t0);
        // Directed at some other device on the bus.
        let out = q.process(&iso_request(7, 99, PGN_PRODUCT_INFO), t0);
        assert!(out.is_empty());
    }

    #[test]
    fn directed_request_to_known_dst_synthesises() {
        let mut q = Scx20::new();
        let t0 = Instant::now();
        q.process(&claim(52, SCX20_NAME_BYTES), t0);
        let out = q.process(&iso_request(7, 52, PGN_PRODUCT_INFO), t0);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].src, 52);
    }

    #[test]
    fn rate_limited_within_one_second() {
        let mut q = Scx20::new();
        let t0 = Instant::now();
        q.process(&claim(52, SCX20_NAME_BYTES), t0);
        // First request fires.
        let out = q.process(&iso_request(7, 52, PGN_PRODUCT_INFO), t0);
        assert_eq!(out.len(), 1);
        // Second request 500 ms later: throttled out.
        let out = q.process(
            &iso_request(7, 52, PGN_PRODUCT_INFO),
            t0 + Duration::from_millis(500),
        );
        assert!(
            out.is_empty(),
            "two requests within {MIN_SYNTH_GAP:?} must collapse"
        );
        // After the gap fully elapses, the next one fires again.
        let out = q.process(
            &iso_request(7, 52, PGN_PRODUCT_INFO),
            t0 + MIN_SYNTH_GAP + Duration::from_millis(1),
        );
        assert_eq!(out.len(), 1);
    }

    #[test]
    fn request_for_other_pgn_is_ignored() {
        let mut q = Scx20::new();
        let t0 = Instant::now();
        q.process(&claim(52, SCX20_NAME_BYTES), t0);
        // Request for PGN 60928 (Address Claim) targeted at SCX-20.
        let out = q.process(&iso_request(7, 52, PGN_ISO_ADDRESS_CLAIM), t0);
        assert!(out.is_empty());
    }
}
