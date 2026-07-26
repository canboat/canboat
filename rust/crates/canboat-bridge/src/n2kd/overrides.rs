// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Server-owned PGN transmission-interval overrides.
//!
//! A user can ask a device to change how often it transmits a PGN via
//! the NMEA 2000 **Request Group Function** (PGN 126208, function code
//! 0). Those requests are stateful — a device forgets them on power
//! cycle, and the request has to be re-sent when it comes back — so the
//! always-on `canboat server`, not an occasional TUI, owns them:
//!
//! * Overrides are persisted keyed by the device's stable 64-bit ISO
//!   **NAME** (from PGN 60928), so an override follows a device across
//!   address (source) reassignment.
//! * When the server first sees a NAME this session (or the NAME turns
//!   up at a new source), it **replays** that device's overrides —
//!   [`OverrideEngine::note_address_claim`] returns the PGN 126208
//!   Request frames to inject.
//! * When the bus **NAKs** a Request (PGN 126208 Acknowledge with a
//!   non-zero error), the server **forgets** that override
//!   ([`OverrideEngine::note_nak`]) so it isn't replayed forever.
//!
//! The TUI edits overrides over a dedicated bidirectional control port
//! carrying the synthetic BEM PGN [`PGN_PGN_OVERRIDE`] (262658), exactly
//! like the NMEA 0183 filter's PGN 262657 channel: the client writes
//! `Set` / `Delete` / `Request` frames and the server answers with
//! `Report` frames on the same socket. The control channel addresses
//! devices by **source** (what the TUI sees); this module resolves
//! src↔NAME at the boundary, so a `Set` for a source whose NAME we
//! haven't learned yet is a no-op.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use canboat_core::RawFrame;
use serde::{Deserialize, Serialize};

/// Synthetic BEM PGN carrying the override control channel — never a
/// real bus frame. Rides its own dedicated bidirectional control port
/// (`--overrides-port`), request/response like PGN 262657.
pub const PGN_PGN_OVERRIDE: u32 = 262658;
/// PGN 262658 `Function`: server → client, current override state.
pub const OV_FN_REPORT: u8 = 0;
/// PGN 262658 `Function`: client → server, add / change an override.
pub const OV_FN_SET: u8 = 1;
/// PGN 262658 `Function`: client → server, "send me the current state".
pub const OV_FN_REQUEST: u8 = 2;
/// PGN 262658 `Function`: client → server, remove an override.
pub const OV_FN_DELETE: u8 = 3;

/// PGN 126208 Request Group Function — the real bus frame an override
/// is applied with.
pub const PGN_REQUEST_GROUP_FUNCTION: u32 = 126208;

/// Local source address for server-emitted frames (canboat C `n2kd`
/// uses 0; the device adapter rewrites it to the gateway's claim).
const LOCAL_SRC: u8 = 0;
/// "Don't change" sentinel for the 16-bit Transmission interval offset.
const OFFSET_DONT_CHANGE: u16 = 0xFFFF;
/// Control-payload sentinel: this override is not for a proprietary PGN.
const MFR_NA: u16 = 0xFFFF;
/// Control-payload sentinel: this override is not for a proprietary PGN.
const IND_NA: u8 = 0xFF;

/// `true` when `frame` is an override `Set` control message.
pub fn is_set_frame(frame: &RawFrame) -> bool {
    frame.pgn == PGN_PGN_OVERRIDE && frame.data.first() == Some(&OV_FN_SET)
}

/// `true` when `frame` is an override `Request` control message.
pub fn is_request_frame(frame: &RawFrame) -> bool {
    frame.pgn == PGN_PGN_OVERRIDE && frame.data.first() == Some(&OV_FN_REQUEST)
}

/// `true` when `frame` is an override `Delete` control message.
pub fn is_delete_frame(frame: &RawFrame) -> bool {
    frame.pgn == PGN_PGN_OVERRIDE && frame.data.first() == Some(&OV_FN_DELETE)
}

/// One override's payload, keyed by `(NAME, pgn)` in [`OverrideEngine`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Override {
    /// Requested transmission interval in ms; `0` = stop transmitting.
    pub interval_ms: u32,
    /// Manufacturer Code for proprietary PGNs; `None` for standard PGNs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub manufacturer_code: Option<u16>,
    /// Industry Code (typically 4 = Marine) for proprietary PGNs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub industry_code: Option<u8>,
    /// Human-readable PGN description, filled from the schema at store
    /// time so the JSON file is legible; not used for anything functional.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// On-disk row: a NAME + a PGN + its override.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct FileEntry {
    /// Full 64-bit ISO NAME as lowercase 16-hex.
    name: String,
    pgn: u32,
    #[serde(flatten)]
    ov: Override,
}

/// Persisted wire shape — a wrapper object holding the entry list, so
/// schema-level metadata can be added later without breaking readers.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
struct FileFormat {
    #[serde(default)]
    entries: Vec<FileEntry>,
}

/// Runtime override store: NAME-keyed rules plus the live `src → NAME`
/// map rebuilt from PGN 60928 each session. The control channel and
/// reports address by source; this resolves to NAME at the boundary.
pub struct OverrideEngine {
    /// Where overrides are persisted; rewritten on every edit.
    path: PathBuf,
    /// `src → NAME`, learned from ISO Address Claims this session.
    src_name: [Option<u64>; 256],
    /// `NAME → (pgn → override)`.
    rules: HashMap<u64, HashMap<u32, Override>>,
    /// `NAME → the source we last replayed its overrides to`, so we
    /// replay once on first sight and again if the NAME moves address.
    replayed: HashMap<u64, u8>,
}

impl OverrideEngine {
    /// Load overrides from `path`. A missing file yields an empty store
    /// (nothing is replayed). A malformed file is an error so a typo
    /// doesn't silently drop every override.
    pub fn load(path: &Path) -> Result<Self> {
        let rules = match std::fs::read_to_string(path) {
            Ok(body) => {
                let file: FileFormat = serde_json::from_str(&body)
                    .with_context(|| format!("parsing overrides {}", path.display()))?;
                let mut rules: HashMap<u64, HashMap<u32, Override>> = HashMap::new();
                for e in file.entries {
                    if let Some(name) = parse_name(&e.name) {
                        rules.entry(name).or_default().insert(e.pgn, e.ov);
                    }
                }
                rules
            }
            Err(_) => HashMap::new(),
        };
        Ok(Self {
            path: path.to_path_buf(),
            src_name: [None; 256],
            rules,
            replayed: HashMap::new(),
        })
    }

    /// Persist the current overrides to [`Self::path`], sorted for a
    /// clean diff. Called after every edit.
    fn save(&self) -> Result<()> {
        if let Some(parent) = self.path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("creating {}", parent.display()))?;
        }
        let mut entries: Vec<FileEntry> = self
            .rules
            .iter()
            .flat_map(|(name, pgns)| {
                pgns.iter().map(move |(pgn, ov)| FileEntry {
                    name: format!("{name:016x}"),
                    pgn: *pgn,
                    ov: ov.clone(),
                })
            })
            .collect();
        entries.sort_by(|a, b| a.name.cmp(&b.name).then(a.pgn.cmp(&b.pgn)));
        let body = serde_json::to_string_pretty(&FileFormat { entries })
            .context("serialising overrides")?;
        std::fs::write(&self.path, body).with_context(|| format!("writing {}", self.path.display()))
    }

    /// Record an ISO Address Claim: `src` now belongs to `name`. Returns
    /// the PGN 126208 Request frames to inject — one per override for
    /// this NAME — when the NAME is seen for the first time this session
    /// or has moved to a different source. Empty otherwise.
    pub fn note_address_claim(&mut self, src: u8, name: u64) -> Vec<RawFrame> {
        self.src_name[src as usize] = Some(name);
        if self.replayed.get(&name) == Some(&src) {
            return Vec::new();
        }
        self.replayed.insert(name, src);
        self.rules
            .get(&name)
            .map(|pgns| {
                pgns.iter()
                    .map(|(pgn, ov)| {
                        request_interval_frame(
                            src,
                            *pgn,
                            ov.interval_ms,
                            ov.manufacturer_code,
                            ov.industry_code,
                        )
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Apply a PGN 262658 `Set` payload: upsert the override under the
    /// NAME currently at `source`, persist, and return the PGN 126208
    /// Request to inject now. `None` when the source has no learned NAME
    /// (we key by NAME, so there's nothing to persist yet).
    pub fn apply_set_frame(&mut self, data: &[u8]) -> Option<RawFrame> {
        let c = parse_control(data)?;
        let name = self.src_name[c.src as usize]?;
        let ov = Override {
            interval_ms: c.interval_ms,
            manufacturer_code: c.manufacturer_code,
            industry_code: c.industry_code,
            description: pgn_description(c.pgn),
        };
        self.rules.entry(name).or_default().insert(c.pgn, ov);
        // We're applying to this source now, so treat it as replayed.
        self.replayed.insert(name, c.src);
        if let Err(e) = self.save() {
            log::warn!("failed to persist override change: {e:#}");
        }
        Some(request_interval_frame(
            c.src,
            c.pgn,
            c.interval_ms,
            c.manufacturer_code,
            c.industry_code,
        ))
    }

    /// Apply a PGN 262658 `Delete` payload: drop the `(NAME, pgn)`
    /// override for the NAME currently at `source`, and persist.
    pub fn apply_delete_frame(&mut self, data: &[u8]) {
        let Some(c) = parse_control(data) else {
            return;
        };
        let Some(name) = self.src_name[c.src as usize] else {
            return;
        };
        self.remove(name, c.pgn);
    }

    /// Forget the override the bus NAKed: the acknowledging device at
    /// `src` errored on `acked_pgn`. Returns whether an override was
    /// removed.
    pub fn note_nak(&mut self, src: u8, acked_pgn: u32) -> bool {
        let Some(name) = self.src_name[src as usize] else {
            return false;
        };
        self.remove(name, acked_pgn)
    }

    /// Remove `(name, pgn)`, dropping the NAME entry when it has no more
    /// overrides, and persist. Returns whether anything was removed.
    fn remove(&mut self, name: u64, pgn: u32) -> bool {
        let removed = self
            .rules
            .get_mut(&name)
            .map(|pgns| pgns.remove(&pgn).is_some())
            .unwrap_or(false);
        if removed {
            if self.rules.get(&name).is_some_and(HashMap::is_empty) {
                self.rules.remove(&name);
            }
            if let Err(e) = self.save() {
                log::warn!("failed to persist override removal: {e:#}");
            }
        }
        removed
    }

    /// One PGN 262658 `Report` frame per override whose NAME currently
    /// maps to a source — the server's answer to a client's connect /
    /// `Set` / `Delete` / `Request`. Overrides for a NAME not on the bus
    /// right now are persisted but omitted (there's no source to address
    /// or display them by). Every frame carries `timestamp`.
    pub fn report_frames(&self, timestamp: Option<String>) -> Vec<RawFrame> {
        // Reverse the src→NAME map so we can address each override's NAME
        // by its current source.
        let mut name_src: HashMap<u64, u8> = HashMap::new();
        for (src, name) in self.src_name.iter().enumerate() {
            if let Some(n) = name {
                name_src.entry(*n).or_insert(src as u8);
            }
        }
        let mut rows: Vec<(u8, u32, &Override)> = Vec::new();
        for (name, pgns) in &self.rules {
            let Some(&src) = name_src.get(name) else {
                continue;
            };
            for (pgn, ov) in pgns {
                rows.push((src, *pgn, ov));
            }
        }
        rows.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));
        rows.into_iter()
            .map(|(src, pgn, ov)| report_frame(timestamp.clone(), src, pgn, ov.interval_ms, ov))
            .collect()
    }
}

/// Build a PGN 262658 `Report` frame for one override.
fn report_frame(
    timestamp: Option<String>,
    src: u8,
    pgn: u32,
    interval_ms: u32,
    ov: &Override,
) -> RawFrame {
    let mfr = ov.manufacturer_code.unwrap_or(MFR_NA);
    let ind = ov.industry_code.unwrap_or(IND_NA);
    let mut data = Vec::with_capacity(12);
    data.push(OV_FN_REPORT);
    data.push(src);
    data.extend_from_slice(&pgn.to_le_bytes()[..3]);
    data.extend_from_slice(&interval_ms.to_le_bytes());
    data.extend_from_slice(&mfr.to_le_bytes());
    data.push(ind);
    RawFrame::new(timestamp, 7, PGN_PGN_OVERRIDE, LOCAL_SRC, 255, data)
}

/// Build the PGN 126208 Request Group Function frame that applies one
/// override: set `dst`'s transmission interval for `pgn` to `interval_ms`
/// (0 = stop). For proprietary PGNs, `mfr` + `industry` append the two
/// scoping parameter pairs (indices 1 and 3). Byte-identical to the
/// captured Furuno SCX-20 setting-tool traffic (see tests).
pub fn request_interval_frame(
    dst: u8,
    pgn: u32,
    interval_ms: u32,
    mfr: Option<u16>,
    industry: Option<u8>,
) -> RawFrame {
    let mut data = Vec::with_capacity(16);
    data.push(0x00); // Function Code: Request
    data.extend_from_slice(&pgn.to_le_bytes()[..3]);
    data.extend_from_slice(&interval_ms.to_le_bytes());
    data.extend_from_slice(&OFFSET_DONT_CHANGE.to_le_bytes());
    match (mfr, industry) {
        (Some(m), Some(i)) => {
            data.push(2); // Number of Parameters
            data.push(1); // param index 1: Manufacturer Code (11-bit → 2 LE bytes)
            data.extend_from_slice(&(m & 0x07FF).to_le_bytes());
            data.push(3); // param index 3: Industry Code (3-bit → 1 byte)
            data.push(i & 0x07);
        }
        _ => data.push(0), // Number of Parameters = 0
    }
    RawFrame::new(None, 3, PGN_REQUEST_GROUP_FUNCTION, LOCAL_SRC, dst, data)
}

/// A decoded PGN 262658 control payload. `manufacturer_code` /
/// `industry_code` are `None` when they carry the "n/a" sentinel.
struct Control {
    src: u8,
    pgn: u32,
    interval_ms: u32,
    manufacturer_code: Option<u16>,
    industry_code: Option<u8>,
}

/// Parse the fixed-offset PGN 262658 control payload.
/// Layout: `[fn, src, pgn(3 LE), interval(4 LE), mfr(2 LE), industry]`.
fn parse_control(data: &[u8]) -> Option<Control> {
    if data.len() < 12 {
        return None;
    }
    let pgn = u32::from(data[2]) | (u32::from(data[3]) << 8) | (u32::from(data[4]) << 16);
    let interval_ms = u32::from_le_bytes([data[5], data[6], data[7], data[8]]);
    let mfr = u16::from_le_bytes([data[9], data[10]]);
    let industry = data[11];
    Some(Control {
        src: data[1],
        pgn,
        interval_ms,
        manufacturer_code: (mfr != MFR_NA).then_some(mfr),
        industry_code: (industry != IND_NA).then_some(industry),
    })
}

/// The human-readable description for `pgn` from the embedded schema,
/// used only to label the persisted JSON.
fn pgn_description(pgn: u32) -> Option<String> {
    canboat_core::PgnDatabase::embedded(canboat_core::Units::Metric)
        .first_pgn(pgn)
        .map(|info| info.description.to_string())
}

/// Parse a 16-hex ISO NAME token.
fn parse_name(s: &str) -> Option<u64> {
    u64::from_str_radix(s.trim_start_matches("0x"), 16).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    const NAME_A: u64 = 0x0004_0000_1066_7913;
    const NAME_B: u64 = 0x0004_0000_0761_9208;

    fn engine() -> OverrideEngine {
        OverrideEngine {
            path: std::env::temp_dir().join(format!("cb-ov-test-{}.json", std::process::id())),
            src_name: [None; 256],
            rules: HashMap::new(),
            replayed: HashMap::new(),
        }
    }

    fn set_payload(src: u8, pgn: u32, interval: u32, mfr: u16, ind: u8) -> Vec<u8> {
        let mut d = vec![OV_FN_SET, src];
        d.extend_from_slice(&pgn.to_le_bytes()[..3]);
        d.extend_from_slice(&interval.to_le_bytes());
        d.extend_from_slice(&mfr.to_le_bytes());
        d.push(ind);
        d
    }

    #[test]
    fn request_frame_standard_matches_scx20_sample() {
        // ,3,126208,0,52,11,00,12,fe,01,00,00,00,00,ff,ff,00
        let f = request_interval_frame(52, 130578, 0, None, None);
        assert_eq!(f.pgn, 126208);
        assert_eq!(f.dst, 52);
        assert_eq!(
            f.data.as_slice(),
            &[
                0x00, 0x12, 0xfe, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00
            ]
        );
    }

    #[test]
    fn request_frame_standard_enable_matches_scx20_sample() {
        // ,3,126208,0,52,11,00,12,fe,01,e8,03,00,00,ff,ff,00
        let f = request_interval_frame(52, 130578, 1000, None, None);
        assert_eq!(
            f.data.as_slice(),
            &[
                0x00, 0x12, 0xfe, 0x01, 0xe8, 0x03, 0x00, 0x00, 0xff, 0xff, 0x00
            ]
        );
    }

    #[test]
    fn request_frame_proprietary_matches_scx20_sample() {
        // ,3,126208,0,52,16,00,1a,ff,01,00,00,00,00,ff,ff,02,01,3f,07,03,04
        let f = request_interval_frame(52, 130842, 0, Some(1855), Some(4));
        assert_eq!(
            f.data.as_slice(),
            &[
                0x00, 0x1a, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x02, 0x01, 0x3f, 0x07,
                0x03, 0x04
            ]
        );
    }

    #[test]
    fn first_sight_replays_overrides_to_current_src() {
        let mut e = engine();
        // Seed a rule by learning NAME_A at src 9 then setting via control.
        e.src_name[9] = Some(NAME_A);
        e.apply_set_frame(&set_payload(9, 127258, 0, MFR_NA, IND_NA));
        // A fresh session: NAME_A claims src 42. Replay must target 42.
        let mut e2 = engine();
        e2.rules = e.rules.clone();
        let frames = e2.note_address_claim(42, NAME_A);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].pgn, 126208);
        assert_eq!(frames[0].dst, 42);
        // A second claim from the same src does not re-replay.
        assert!(e2.note_address_claim(42, NAME_A).is_empty());
        // But moving to a new src does.
        assert_eq!(e2.note_address_claim(43, NAME_A).len(), 1);
    }

    #[test]
    fn set_then_report_round_trips_by_src() {
        let mut e = engine();
        e.src_name[52] = Some(NAME_B);
        let applied = e
            .apply_set_frame(&set_payload(52, 130842, 200, 1855, 4))
            .expect("known src applies");
        assert_eq!(applied.dst, 52);
        // Proprietary Request carries the parameter pairs.
        assert_eq!(applied.data.last(), Some(&0x04));
        let reports = e.report_frames(Some("2026-07-10T00:00:00.000Z".into()));
        assert_eq!(reports.len(), 1);
        let c = parse_control(&reports[0].data).unwrap();
        assert_eq!((c.src, c.pgn, c.interval_ms), (52, 130842, 200));
        assert_eq!(
            (c.manufacturer_code, c.industry_code),
            (Some(1855), Some(4))
        );
    }

    #[test]
    fn set_on_unknown_src_is_noop() {
        let mut e = engine();
        assert!(
            e.apply_set_frame(&set_payload(99, 127258, 0, MFR_NA, IND_NA))
                .is_none()
        );
        assert!(e.rules.is_empty());
    }

    #[test]
    fn nak_forgets_the_override() {
        let mut e = engine();
        e.src_name[52] = Some(NAME_B);
        e.apply_set_frame(&set_payload(52, 130578, 0, MFR_NA, IND_NA));
        assert!(e.note_nak(52, 130578));
        assert!(e.report_frames(None).is_empty());
        assert!(!e.note_nak(52, 130578), "already gone");
    }

    #[test]
    fn delete_removes_only_the_named_override() {
        let mut e = engine();
        e.src_name[52] = Some(NAME_B);
        e.apply_set_frame(&set_payload(52, 130578, 0, MFR_NA, IND_NA));
        e.apply_set_frame(&set_payload(52, 130842, 0, 1855, 4));
        let mut del = vec![OV_FN_DELETE, 52];
        del.extend_from_slice(&130578u32.to_le_bytes()[..3]);
        del.extend_from_slice(&0u32.to_le_bytes());
        del.extend_from_slice(&MFR_NA.to_le_bytes());
        del.push(IND_NA);
        e.apply_delete_frame(&del);
        let reports = e.report_frames(None);
        assert_eq!(reports.len(), 1);
        assert_eq!(parse_control(&reports[0].data).unwrap().pgn, 130842);
    }

    #[test]
    fn classifiers_match_only_their_function() {
        let set = RawFrame::new(
            None,
            7,
            PGN_PGN_OVERRIDE,
            0,
            255,
            set_payload(1, 2, 3, 4, 5),
        );
        assert!(is_set_frame(&set));
        assert!(!is_request_frame(&set));
        assert!(!is_delete_frame(&set));
        let other = RawFrame::new(None, 6, 127251, 5, 255, [0u8; 8]);
        assert!(!is_set_frame(&other));
    }
}
