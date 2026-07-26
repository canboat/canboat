// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! B&G H5000 Motion Sensor impersonation.
//!
//! A Navico/B&G **Hercules H5000** sailing processor will only accept
//! roll/pitch attitude for heel/trim motion correction from a device whose
//! **PGN 126996 Product Code == `0x53F0` (21488)** — the code of the real
//! *H5000 Motion Sensor*. It is a single hard equality test in the CPU
//! firmware (`FUN_003ce89c`); manufacturer / device-class / function are
//! *not* the gate. So a generic third-party attitude sensor on the bus is
//! ignored no matter how correct its PGN 127257.
//!
//! With `--quirk motion` canboat presents itself as that Motion Sensor —
//! a distinct virtual node with its own ISO 11783-5 address claim (via the
//! shared [`canboat_io::address_claim::AddressClaim`]) — answering 126996
//! with product code `0x53F0` and emitting the **B&G proprietary PGN 130824**
//! gyro **rate** feed (roll/pitch/yaw rate) that the CPU reads for its
//! stabilization loop. We synthesize that 130824 by transcoding a **Furuno
//! SCX-20 PGN 130842 "Six Degrees Of Freedom Movement"** off the bus — the
//! only source of real roll/pitch rate (no standard PGN carries them). Only
//! emitted while a fresh 130842 is seen.
//!
//! **Attitude (127257) is deliberately *not* emitted.** A live-Hercules test
//! (2026-07-10, recorded) indicated the rate feed alone keeps Motion "green"
//! and stabilizes apparent wind through hard rolling — the mast-tip correction
//! appears driven by angular *rate*, not absolute heel. Relaying attitude under
//! our identity was also redundant: the SCX-20's own 127257 is already on the
//! bus for any attitude consumer. Dropping it halves this quirk's synthetic
//! load. (This is a coarse, qualitative observation, not an instrumented one.)
//!
//! Both proprietary layouts were reverse-engineered by correlation against a
//! known rate reference; see `n2k_research/navico/motion/
//! rate-pgns-130824-130842.md` for the byte maps and evidence, plus
//! `hercules-acceptance.md` (recognition gate) and `digital-twin-spec.md`. The
//! Furuno 130842 rate fields (E/F/G, 32-bit signed, **0.001 °/s**) follow the
//! corrected canboat schema — issue #756 / commit 8a258b9.
//!
//! Scope: address claim, 126996 Product Info, 126464 TX/RX PGN lists, and the
//! 130842→130824 rate transcode. The Simnet source-selection handshake
//! (65323 / 130840) is not implemented yet.

use std::time::{Instant, SystemTime, UNIX_EPOCH};

use canboat_core::{ADDR_GLOBAL, DecodedPgn, RawFrame, field, format_iso_ms};
use canboat_io::address_claim::AddressClaim;
use canboat_io::nmea_responder::{self, ProductInfo};

const PGN_ISO_REQUEST: u32 = 59904;
const PGN_ISO_ADDRESS_CLAIM: u32 = 60928;
const PGN_PGN_LIST: u32 = 126464;
const PGN_PRODUCT_INFO: u32 = 126996;
/// B&G proprietary key-value data — we emit the roll/pitch/yaw **rate** feed
/// the Hercules CPU reads (keys 60/158/68). Priority 3 as the real unit.
const PGN_BG_RATES: u32 = 130824;
const PGN_BG_RATES_PRIO: u8 = 3;
/// Furuno SCX-20 "Six Degrees Of Freedom Movement" — our rate source.
const PGN_SCX_6DOF: u32 = 130842;
const MFG_FURUNO: u16 = 1855;

/// B&G 130824 key IDs for the three rate axes (see `rate-pgns-130824-130842.md`).
const KEY_ROLL_RATE: u16 = 60;
const KEY_PITCH_RATE: u16 = 158; // canboat mislabels this "Pitch Angle"
const KEY_YAW_RATE: u16 = 68; // canboat leaves this key unnamed

/// Rate LSBs, both in **rad/s**. The transcode multiplies each count by their
/// ratio `SCX_RATE_LSB_RAD_S / BG_RATE_LSB_RAD_S` ≈ **×558**.
///
/// **Input (Furuno 130842): 0.001 °/s = 1.7453e-5 rad/s** per count — solid
/// (canboat issue #756 / commit 8a258b9, `ROTATION_FIX32_MDEG_S`: integrating
/// field G against the SCX-20's own unwrapped 127257 yaw gives r=0.9996 at
/// 1.75e-5 rad/s, cross-checked vs an independent Precision-9).
///
/// **Output (B&G 130824): the NMEA-2000 127251 Rate-of-Turn resolution,
/// 3.125e-8 rad/s** (= 1e-6/32). The genuine Motion Sensor reuses that standard
/// gyro-rate LSB: integrating its own 130824 key-60 count against its 127257
/// roll (BnG_Zeus capture) measures 3.05–3.54e-8 rad/s per count, which brackets
/// 3.125e-8. ±1e6 raw counts then read ±1.79 °/s (real swell); i32 full-scale is
/// ±3843 °/s of headroom. The old "0.0001 °/s" (a ×10 gain) came from correlating
/// against the SCX-20's *damped* 127251 ROT — the flaw #756 fixed on the input
/// side — and is dead by the physical-range check (±1e6 would read ±100 °/s).
///
/// NB on the ~558 gain: it is a ratio of two *same-dimension* LSBs, hence
/// **unit-invariant** (identical computed in rad or in deg), so it is NOT a
/// rad↔deg artifact — despite its resemblance to 10·(180/π)=572.96. That number
/// was the *old* buggy code, which divided a rad-labelled SCX LSB by a
/// deg-labelled BG LSB; it happened to land near the true ratio, which is why it
/// accidentally worked. Value confirmed against a genuine unit; a Hercules
/// CPUApp teardown corroborated the architecture (rate → mast-tip `v = ω·h` wind
/// correction) but holds no single scale literal. Tunable if hardware disagrees.
/// See `n2k_research/navico/motion/rate-pgns-130824-130842.md`.
const SCX_RATE_LSB_RAD_S: f64 = 0.001 * std::f64::consts::PI / 180.0;
/// NMEA-2000 PGN 127251 "Rate of Turn" resolution (1e-6/32 rad/s).
const BG_RATE_LSB_RAD_S: f64 = 3.125e-8;

/// ISO NAME fields identifying the H5000 Motion Sensor personality. The
/// class/function are *not* the Hercules acceptance gate (that is the
/// Product Code below), but a faithful twin still presents them.
const MFG_BANDG: u16 = 381;
const DEVICE_FUNCTION: u8 = 140; // "Ownship Attitude"
const DEVICE_CLASS: u8 = 60; // Navigation
const INDUSTRY_MARINE: u16 = 4; // proprietary-PGN header industry code

/// Preferred source address. The real unit was seen at 24; if it's taken
/// the claim state machine picks the lowest free one. Kept non-zero so the
/// pipeline's own-node stamping (which rewrites src 0/255) never touches
/// our frames.
const PREFERRED_ADDRESS: u8 = 24;

/// PGN 126996 "Product Information" for the Motion Sensor — the value that
/// matters is the Product Code; the rest mirrors the captured genuine unit
/// (`n2k_research`: Model ID "H5000 Motion Sensor", Model Version "Motion",
/// SW "1.2.4"). **`PRODUCT_CODE` is THE Hercules acceptance gate.**
const PRODUCT_CODE: i64 = 21488; // 0x53F0
const DB_VERSION: i64 = 2100; // 2.100 at res 0.001
const MODEL_ID: &str = "H5000 Motion Sensor";
const SOFTWARE_VERSION: &str = "1.2.4";
const MODEL_VERSION: &str = "Motion";
const MODEL_SERIAL: &str = "canboat";
const CERTIFICATION_LEVEL: i64 = 2;
const LOAD_EQUIVALENCY: i64 = 1;

/// PGNs the twin transmits / receives, reported via PGN 126464 on request.
const TX_PGN_LIST: [u32; 4] = [
    PGN_ISO_ADDRESS_CLAIM,
    PGN_PRODUCT_INFO,
    PGN_PGN_LIST,
    PGN_BG_RATES,
];
const RX_PGN_LIST: [u32; 3] = [PGN_ISO_REQUEST, PGN_ISO_ADDRESS_CLAIM, PGN_SCX_6DOF];

/// Stateful Motion Sensor impersonation.
pub struct Motion {
    /// Our own virtual node's address-claim state machine.
    claim: AddressClaim,
    /// Base instant for deriving the monotonic ms the claim SM wants; set
    /// on the first `process` call.
    base: Option<Instant>,
    /// False until the claim handshake has been kicked off.
    started: bool,
    /// The bus source whose Furuno 130842 we transcode into 130824. Latched
    /// to the first Furuno 6DOF source seen once we own an address (never
    /// ourselves), so we don't flip between sources or loop on our own
    /// re-emission.
    rate_source: Option<u8>,
}

impl Default for Motion {
    fn default() -> Self {
        Self::new()
    }
}

impl Motion {
    pub fn new() -> Self {
        Self {
            claim: AddressClaim::new(build_name(), PREFERRED_ADDRESS, true),
            base: None,
            started: false,
            rate_source: None,
        }
    }

    /// Drive the claim state machine and relay attitude. `now` is injected
    /// so the (relative) claim clock is deterministic in tests.
    pub fn process(&mut self, d: &DecodedPgn, now: Instant) -> Vec<RawFrame> {
        let base = *self.base.get_or_insert(now);
        let now_ms = now.saturating_duration_since(base).as_millis() as u64;

        let mut out = Vec::new();
        if !self.started {
            self.started = true;
            log::info!(
                "quirk(motion): starting H5000 Motion Sensor impersonation \
                 (claiming address, preferred {PREFERRED_ADDRESS})"
            );
            out.extend(self.claim.start(now_ms));
        }

        let was_claimed = self.claim.is_claimed();

        match d.pgn {
            // Learn/arbitrate addresses; drive our own claim.
            PGN_ISO_ADDRESS_CLAIM => {
                if let Some(their_name) = d.iso_name() {
                    out.extend(self.claim.on_address_claim(now_ms, d.src, their_name));
                }
            }
            // Answer requests addressed to us (or broadcast).
            PGN_ISO_REQUEST => out.extend(self.handle_request(d)),
            // Transcode a Furuno SCX-20 6DOF frame's gyro rates into the
            // B&G 130824 the Hercules CPU wants.
            PGN_SCX_6DOF => {
                if let Some(f) = self.maybe_emit_rates(d) {
                    out.push(f);
                }
            }
            _ => {}
        }

        // Deadline-driven transitions (scan → claim → owned).
        out.extend(self.claim.tick(now_ms));

        // The moment we take ownership: log it and announce ourselves with
        // an unsolicited Product Information (as the real sensor does), so
        // the twin is discoverable — and visible in the log — without
        // waiting for a poll. PGN lists stay request-driven.
        if !was_claimed && let Some(addr) = self.claim.address() {
            log::info!(
                "quirk(motion): claimed address {addr} as the H5000 Motion Sensor \
                 (PGN 126996 product code 0x53F0); announcing product info, waiting \
                 for a Furuno 130842 source to transcode into 130824 rates"
            );
            out.extend(self.product_info());
        }

        stamp(&mut out);
        out
    }

    /// Respond to an ISO Request for one of the PGNs we serve, when it is
    /// addressed to our claimed address or broadcast.
    fn handle_request(&self, d: &DecodedPgn) -> Vec<RawFrame> {
        let Some(requested) = d
            .field(field::iso_request::PGN)
            .and_then(|f| f.value.as_i64())
        else {
            return Vec::new();
        };
        let requested = requested as u32;
        let addressed = self.claim.address() == Some(d.dst) || d.dst == ADDR_GLOBAL;
        if !addressed {
            return Vec::new();
        }
        match requested {
            PGN_ISO_ADDRESS_CLAIM => self.claim.respond_to_claim_request().into_iter().collect(),
            PGN_PRODUCT_INFO => {
                log::debug!(
                    "quirk(motion): answering PGN 126996 request from src {} (the Hercules \
                     acceptance gate)",
                    d.src
                );
                self.product_info().into_iter().collect()
            }
            PGN_PGN_LIST => {
                log::debug!(
                    "quirk(motion): answering PGN 126464 list request from src {}",
                    d.src
                );
                self.pgn_lists()
            }
            _ => Vec::new(),
        }
    }

    /// Transcode a Furuno SCX-20 PGN 130842 "Six Degrees Of Freedom
    /// Movement" (`d`) into a B&G PGN 130824 rate frame emitted from our
    /// claimed address. `None` until we own an address, for a non-Furuno
    /// 130842, or for any source other than the latched rate source.
    fn maybe_emit_rates(&mut self, d: &DecodedPgn) -> Option<RawFrame> {
        let addr = self.claim.address()?;
        if d.src == addr {
            return None; // never consume our own emissions
        }
        // Only the Furuno 6DOF layout has the roll/pitch/yaw-rate fields at
        // the byte offsets we read; guard on the manufacturer in the header.
        if header_manufacturer(&d.data) != Some(MFG_FURUNO) {
            return None;
        }
        let (roll, pitch, yaw) = scx_6dof_rates(&d.data)?;
        // Latch the first Furuno 6DOF source we see once claimed.
        if self.rate_source.is_none() {
            self.rate_source = Some(d.src);
            log::info!(
                "quirk(motion): transcoding SCX-20 PGN 130842 gyro rates from src {} \
                 into B&G PGN 130824 (roll/pitch/yaw rate) under addr {addr}",
                d.src
            );
        }
        if self.rate_source != Some(d.src) {
            return None;
        }
        Some(RawFrame::new(
            None,
            PGN_BG_RATES_PRIO,
            PGN_BG_RATES,
            addr,
            ADDR_GLOBAL,
            build_130824_payload(roll, pitch, yaw),
        ))
    }

    /// Encode PGN 126996 with the Motion Sensor's product code — **the
    /// Hercules acceptance gate** — via the shared responder builder.
    fn product_info(&self) -> Option<RawFrame> {
        let addr = self.claim.address()?;
        ProductInfo {
            db_version: DB_VERSION,
            product_code: PRODUCT_CODE,
            model_id: MODEL_ID,
            software_version: SOFTWARE_VERSION,
            model_version: MODEL_VERSION,
            model_serial: MODEL_SERIAL,
            certification_level: CERTIFICATION_LEVEL,
            load_equivalency: LOAD_EQUIVALENCY,
        }
        .frame(addr)
    }

    /// PGN 126464 Transmit/Receive PGN lists via the shared responder builder.
    fn pgn_lists(&self) -> Vec<RawFrame> {
        match self.claim.address() {
            Some(addr) => {
                nmea_responder::pgn_list_frames(addr, ADDR_GLOBAL, &TX_PGN_LIST, &RX_PGN_LIST)
            }
            None => Vec::new(),
        }
    }
}

/// The 64-bit ISO NAME for the Motion Sensor personality. The Unique
/// Number is derived from the host machine id so it is stable per host and
/// distinct from a real unit's (avoiding a NAME clash if both are present).
/// Marine industry group and arbitrary-address-capable are the builder's
/// defaults.
fn build_name() -> u64 {
    let unique = canboat_core::os::get_machine_id() as u32;
    canboat_io::name::Name::new(MFG_BANDG, unique)
        .device_function(DEVICE_FUNCTION)
        .device_class(DEVICE_CLASS)
        .to_u64()
}

/// The 11-bit manufacturer code from a proprietary PGN's first two header
/// bytes (`manufacturerCode(11) | reserved(2) | industryCode(3)`, LE).
/// `None` if the payload is too short to hold the header.
fn header_manufacturer(data: &[u8]) -> Option<u16> {
    let hdr = u16::from_le_bytes([*data.first()?, *data.get(1)?]);
    Some(hdr & 0x07ff)
}

/// Read the roll/pitch/yaw **rate** fields (E, F, G) out of a Furuno SCX-20
/// PGN 130842 payload, in the source's raw 0.001-°/s counts. Byte offsets are
/// fixed by the canboat schema (issue #756): E = i32 @15, F = i32 @19,
/// G = i32 @23. G is the full 32-bit signed value — the layout's old 16-bit G
/// plus field H are one field, H having been G's sign extension. `None` if the
/// payload is short.
fn scx_6dof_rates(data: &[u8]) -> Option<(i32, i32, i32)> {
    let roll = i32::from_le_bytes(data.get(15..19)?.try_into().ok()?);
    let pitch = i32::from_le_bytes(data.get(19..23)?.try_into().ok()?);
    let yaw = i32::from_le_bytes(data.get(23..27)?.try_into().ok()?);
    Some((roll, pitch, yaw))
}

/// Convert one SCX-20 130842 rate count to a B&G 130824 rate count by the LSB
/// ratio [`SCX_RATE_LSB_RAD_S`] / [`BG_RATE_LSB_RAD_S`] (≈×558), saturating to
/// the i32 the wire field holds.
fn transcode_rate(scx_count: i32) -> i32 {
    let rad_s = scx_count as f64 * SCX_RATE_LSB_RAD_S;
    (rad_s / BG_RATE_LSB_RAD_S)
        .round()
        .clamp(i32::MIN as f64, i32::MAX as f64) as i32
}

/// Hand-pack a B&G PGN 130824 "bGKeyValueData" payload carrying the three
/// gyro rates. The schema-driven [`canboat_core::encode`] builder cannot
/// emit this — its fields are `DYNAMIC_FIELD_KEY/LENGTH/VALUE` — so it is
/// laid out by hand: a 2-byte mfr/industry header, then three
/// `{ key|len<<12 (u16 LE), value (i32 LE) }` entries (len 4 bytes each).
fn build_130824_payload(roll: i32, pitch: i32, yaw: i32) -> Vec<u8> {
    // Proprietary header: manufacturer(11) | reserved(2 = 0b11) | industry(3).
    let header: u16 = MFG_BANDG | (0b11 << 11) | (INDUSTRY_MARINE << 13);
    let mut v = Vec::with_capacity(2 + 3 * 6);
    v.extend_from_slice(&header.to_le_bytes());
    for (key, count) in [
        (KEY_ROLL_RATE, roll),
        (KEY_PITCH_RATE, pitch),
        (KEY_YAW_RATE, yaw),
    ] {
        let key_len: u16 = (key & 0x0fff) | (4 << 12); // 4-byte value
        v.extend_from_slice(&key_len.to_le_bytes());
        v.extend_from_slice(&transcode_rate(count).to_le_bytes());
    }
    v
}

/// Stamp a host-clock timestamp on any frame that lacks one, so the local
/// analyzer / snapshot streams show these synthetic frames with a time.
fn stamp(frames: &mut [RawFrame]) {
    if frames.iter().all(|f| f.timestamp.is_some()) {
        return;
    }
    let ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);
    let ts = format_iso_ms(ms);
    for f in frames.iter_mut().filter(|f| f.timestamp.is_none()) {
        f.timestamp = Some(ts.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use canboat_core::{PgnDatabase, Units};
    use std::time::Duration;

    fn si() -> &'static PgnDatabase {
        PgnDatabase::embedded(Units::Si)
    }

    fn dec(f: &RawFrame) -> DecodedPgn {
        si().decode(f).expect("decodes")
    }

    /// A decoded PGN 59904 ISO Request for `target`, from `src` to `dst`.
    fn request(src: u8, dst: u8, target: u32) -> DecodedPgn {
        let data = [target as u8, (target >> 8) as u8, (target >> 16) as u8];
        dec(&RawFrame::new(None, 6, PGN_ISO_REQUEST, src, dst, data))
    }

    /// A decoded PGN 127257 Attitude from `src`. The quirk no longer acts on
    /// 127257, so this is just a spare, unmatched frame used to advance the
    /// claim clock in [`drive_to_claimed`].
    fn attitude(src: u8) -> DecodedPgn {
        let data = [0x00, 0xff, 0x7f, 0xcd, 0x00, 0x01, 0xfc, 0xff];
        dec(&RawFrame::new(None, 3, 127257, src, ADDR_GLOBAL, data))
    }

    /// A decoded Furuno SCX-20 PGN 130842 6DOF frame from `src`, using the
    /// exact 29 bytes captured from a real SCX-20 (`scx20-setting-tool-start
    /// .raw`): roll-rate field E = -2184, pitch-rate F = 1467, yaw-rate G =
    /// 774 (raw 0.001-°/s counts; G read as the full 32-bit field, its high
    /// two bytes 0x0000 being the sign extension — see canboat issue #756).
    fn six_dof(src: u8) -> DecodedPgn {
        let data = [
            0x3f, 0x9f, 0x54, 0xff, 0xff, 0xff, 0xd4, 0xff, 0xff, 0xff, 0x2d, 0x03, 0x00, 0x00,
            0x00, 0x78, 0xf7, 0xff, 0xff, 0xbb, 0x05, 0x00, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00,
            0x00,
        ];
        dec(&RawFrame::new(
            None,
            7,
            PGN_SCX_6DOF,
            src,
            ADDR_GLOBAL,
            data,
        ))
    }

    /// Advance `m` to a claimed address by feeding a spare frame across the
    /// scan + claim windows on an idle bus. Returns the claimed address.
    fn drive_to_claimed(m: &mut Motion, t0: Instant) -> u8 {
        // First call kicks off the scan (deadline t0 + 1000 ms).
        m.process(&attitude(200), t0);
        // Past scan window → begins claim (deadline + 250 ms).
        m.process(&attitude(200), t0 + Duration::from_millis(1001));
        // Past claim window → owned.
        m.process(&attitude(200), t0 + Duration::from_millis(1300));
        m.claim.address().expect("claimed")
    }

    #[test]
    fn claims_preferred_address_on_idle_bus() {
        let mut m = Motion::new();
        let addr = drive_to_claimed(&mut m, Instant::now());
        assert_eq!(addr, PREFERRED_ADDRESS);
    }

    #[test]
    fn answers_product_info_with_the_gate_code() {
        let mut m = Motion::new();
        let t0 = Instant::now();
        let addr = drive_to_claimed(&mut m, t0);
        let out = m.process(
            &request(9, addr, PGN_PRODUCT_INFO),
            t0 + Duration::from_millis(1400),
        );
        let pi = out
            .iter()
            .find(|f| f.pgn == PGN_PRODUCT_INFO)
            .expect("product info emitted");
        assert_eq!(pi.src, addr);
        let d = si().decode(pi).unwrap();
        assert_eq!(
            d.field(field::product_information::PRODUCT_CODE)
                .and_then(|f| f.value.as_i64()),
            Some(PRODUCT_CODE),
            "the Hercules acceptance gate"
        );
        assert_eq!(
            d.field(field::product_information::MODEL_ID)
                .and_then(|f| f.value.as_str()),
            Some(MODEL_ID)
        );
    }

    #[test]
    fn transcode_rate_scales_and_saturates() {
        // Gain is the LSB ratio (≈558) — assert against the constants, not a
        // baked-in number, so tuning BG_RATE_LSB_RAD_S never breaks this test.
        // (The earlier "clean ×10", from a 57× too-coarse B&G LSB, undershot the
        // CPU rate feed.)
        let gain = SCX_RATE_LSB_RAD_S / BG_RATE_LSB_RAD_S;
        assert_eq!(transcode_rate(1000), (1000.0 * gain).round() as i32);
        assert_eq!(transcode_rate(-1000), -(1000.0 * gain).round() as i32);
        assert!(
            gain > 100.0,
            "sanity: a mid-hundreds gain, never the ×10 trap"
        );
        assert_eq!(transcode_rate(0), 0);
        // A full-scale i32 input saturates rather than wrapping/panicking.
        assert_eq!(transcode_rate(i32::MAX), i32::MAX);
        assert_eq!(transcode_rate(i32::MIN), i32::MIN);
    }

    #[test]
    fn builds_130824_with_the_three_rate_keys() {
        // e=-2184, f=1467, g=774 (the captured six_dof values).
        let p = build_130824_payload(-2184, 1467, 774);
        assert_eq!(p.len(), 20);
        assert_eq!(&p[0..2], &[0x7d, 0x99], "B&G mfr 381 + marine header");
        // Parse the three key/len + value entries.
        let entry = |off: usize| -> (u16, u8, i32) {
            let kl = u16::from_le_bytes([p[off], p[off + 1]]);
            let val = i32::from_le_bytes(p[off + 2..off + 6].try_into().unwrap());
            (kl & 0x0fff, (kl >> 12) as u8, val)
        };
        assert_eq!(entry(2), (KEY_ROLL_RATE, 4, transcode_rate(-2184)));
        assert_eq!(entry(8), (KEY_PITCH_RATE, 4, transcode_rate(1467)));
        assert_eq!(entry(14), (KEY_YAW_RATE, 4, transcode_rate(774)));
    }

    #[test]
    fn transcodes_furuno_6dof_into_bg_rates() {
        let mut m = Motion::new();
        let t0 = Instant::now();
        let addr = drive_to_claimed(&mut m, t0);
        let out = m.process(&six_dof(52), t0 + Duration::from_millis(1400));
        let f = out
            .iter()
            .find(|f| f.pgn == PGN_BG_RATES)
            .expect("130824 emitted");
        assert_eq!(f.src, addr);
        assert_eq!(f.prio, PGN_BG_RATES_PRIO);
        assert_eq!(f.data.as_slice(), build_130824_payload(-2184, 1467, 774));
        // A second Furuno source is ignored once one is latched.
        let out = m.process(&six_dof(60), t0 + Duration::from_millis(1500));
        assert!(out.iter().all(|f| f.pgn != PGN_BG_RATES));
    }

    #[test]
    fn ignores_non_furuno_6dof() {
        // header_manufacturer guards the fixed-offset field read.
        assert_eq!(header_manufacturer(&[0x3f, 0x9f]), Some(MFG_FURUNO));
        assert_eq!(header_manufacturer(&[0x7d, 0x99]), Some(MFG_BANDG));
        assert_eq!(header_manufacturer(&[0x00]), None);
    }

    #[test]
    fn answers_address_claim_request() {
        let mut m = Motion::new();
        let t0 = Instant::now();
        let addr = drive_to_claimed(&mut m, t0);
        let out = m.process(
            &request(9, ADDR_GLOBAL, PGN_ISO_ADDRESS_CLAIM),
            t0 + Duration::from_millis(1400),
        );
        let c = out
            .iter()
            .find(|f| f.pgn == PGN_ISO_ADDRESS_CLAIM)
            .expect("claim answered");
        assert_eq!(c.src, addr);
    }
}
