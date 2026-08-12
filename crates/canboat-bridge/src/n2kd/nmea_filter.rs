// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Per-device NMEA 0183 output filter.
//!
//! The pipeline converts every decoded N2K record to NMEA 0183, so a
//! bus where several devices report the same measurement produces
//! several copies of each sentence on the 0183 outputs. The [`RateLimiter`]
//! (`crate::n2kd::nmea0183`) caps each *source* to 1 Hz, but N devices reporting
//! one quantity still yield N sentences/s. This filter removes the
//! redundant *devices*: the user picks which device should own each
//! measurement, and the others are muted.
//!
//! Devices are keyed by their stable **NAME** (the full 64-bit ISO NAME
//! from PGN 60928 ISO Address Claim) rather than source address, which
//! can be reassigned at any address-claim round. A consequence, chosen
//! deliberately: **a source whose NAME we haven't learned yet produces
//! no 0183 at all** — we can't attribute it to a filter rule, so it
//! stays silent until it claims. The pipeline's request engine actively
//! solicits PGN 60928, so this is a brief startup gap, not a permanent
//! blackout. A device that transmits an "unavailable" manufacturer code
//! is still uniquely identified by the rest of its NAME, so it is
//! filtered like any other — not lumped together or ignored.
//!
//! This filter only ever affects the NMEA 0183 outputs. The N2K bus,
//! the analyzer JSON port, and the snapshot cache are untouched, so
//! other consumers (Signal K, plotters) still see every device. AIS
//! (`!AI…`) is converted on a separate path and is never filtered here.
//!
//! Scope of this module today: load mute rules from a JSON file and
//! apply them. The synthetic-PGN control channel that lets canboat-tui
//! edit the rules live plugs in on top of this core.

use std::collections::{BTreeSet, HashMap};
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result};
use canboat_core::RawFrame;
use serde::{Deserialize, Serialize};

/// Synthetic BEM PGN carrying the per-device NMEA 0183 filter control
/// channel — never a real bus frame. It rides its own **dedicated
/// bidirectional control port** (`--nmea0183-filter-port`), never the
/// read-only analyzer stream, so no other consumer ever sees it. The
/// exchange is request/response: a client (the TUI) writes a `Set` to
/// change a rule or a `Request` to (re-)poll, and the server answers on
/// the same socket with `Report` frames describing the current state.
pub const PGN_NMEA0183_FILTER: u32 = 262657;
/// PGN 262657 `Function`: server → client, current state.
pub const FILTER_FN_REPORT: u8 = 0;
/// PGN 262657 `Function`: client → server, apply a change.
pub const FILTER_FN_SET: u8 = 1;
/// PGN 262657 `Function`: client → server, "send me the current state".
/// The server also reports unprompted once on connect, so a client
/// learns the state without asking; this is for re-polling as devices
/// come and go.
pub const FILTER_FN_REQUEST: u8 = 2;
/// How often a client re-`Request`s the full filter state while its
/// control connection is open, so newly-observed sources/sentences
/// appear without the server having to push unsolicited updates.
pub const FILTER_REPORT_INTERVAL: Duration = Duration::from_secs(2);

/// `true` when `frame` is a filter `Set` control message.
pub fn is_set_frame(frame: &RawFrame) -> bool {
    frame.pgn == PGN_NMEA0183_FILTER && frame.data.first() == Some(&FILTER_FN_SET)
}

/// `true` when `frame` is a filter `Request` control message.
pub fn is_request_frame(frame: &RawFrame) -> bool {
    frame.pgn == PGN_NMEA0183_FILTER && frame.data.first() == Some(&FILTER_FN_REQUEST)
}

/// Formatter token used in PGN 262657 / [`FilterReportRow`] to mean
/// "the whole source", as opposed to a specific 3-letter sentence.
pub const ALL_SENTENCES: &str = "ALL";

/// Stable device identity: the full 64-bit ISO NAME from the PGN 60928
/// payload (8 bytes, little-endian — the value the bus arbitrates on).
///
/// We key on the whole NAME rather than just Manufacturer Code + Unique
/// Number because some devices transmit an "unavailable" (`0x7FF`)
/// manufacturer code: their identity then lives entirely in the other
/// NAME fields (device function / class / instance). Those devices are
/// still unique on the bus — arbitration guarantees no two share a NAME
/// — so keying on the full 64 bits gives every claiming device a
/// distinct, stable key instead of lumping the "degenerate" ones
/// together or dropping them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NameKey(pub u64);

impl NameKey {
    fn to_hex(self) -> String {
        format!("{:016x}", self.0)
    }

    fn from_hex(s: &str) -> Option<Self> {
        u64::from_str_radix(s.trim_start_matches("0x"), 16)
            .ok()
            .map(NameKey)
    }
}

/// What the pipeline should do with one source's converted 0183 for a
/// single record.
pub enum Decision<'a> {
    /// Emit nothing — either the source has no learned NAME, or the
    /// whole device is muted.
    DropAll,
    /// Emit, but drop any sentence whose 3-letter formatter appears in
    /// this list (empty = pass everything through).
    Keep(&'a [String]),
}

/// One device's mute rule, keyed by NAME in [`FilterConfig`].
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeviceRule {
    /// Mute the whole device — none of its measurements reach 0183.
    #[serde(default)]
    pub muted: bool,
    /// Mute individual sentence formatters (e.g. `["VHW", "VLW"]`)
    /// while letting the device's other sentences through.
    #[serde(default)]
    pub muted_sentences: Vec<String>,
}

/// On-disk config row: a NAME plus its rule.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct DeviceEntry {
    /// Full 64-bit ISO NAME as lowercase hex (see [`NameKey`]).
    name: String,
    #[serde(flatten)]
    rule: DeviceRule,
}

/// Persisted wire shape — a wrapper object holding the device list, so
/// schema-level metadata can be added later without breaking readers.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
struct FileFormat {
    #[serde(default)]
    devices: Vec<DeviceEntry>,
}

/// One row the pipeline broadcasts as a PGN 262657 Report so the TUI
/// can show, per source, what it emits and whether it's muted.
pub struct FilterReportRow {
    pub src: u8,
    /// 3-letter formatter, or [`ALL_SENTENCES`] for the whole source.
    pub sentence: String,
    pub muted: bool,
}

/// Runtime NMEA 0183 filter: the live `src → NAME` map (rebuilt from
/// PGN 60928 each session) plus the NAME-keyed mute rules. Rules are
/// keyed by NAME for stability across address changes, but the control
/// channel (PGN 262657) and reports address devices by source, which
/// this resolves at the boundary.
pub struct NmeaFilter {
    /// Where mute rules are persisted; rewritten on every edit.
    path: PathBuf,
    /// `src → NAME`, learned from ISO Address Claims this session.
    src_name: [Option<NameKey>; 256],
    /// NAME → mute rule, loaded from the config file.
    rules: HashMap<NameKey, DeviceRule>,
    /// `src → the sentence formatters we've seen it produce` this
    /// session — the menu of "what could be sent" a report exposes,
    /// recorded before any mute is applied so muted sentences still
    /// appear. Keyed by source (what the TUI addresses), not NAME.
    observed: HashMap<u8, BTreeSet<String>>,
}

impl NmeaFilter {
    /// Load mute rules from `path`. A missing file yields an empty
    /// rule set (filter is active but mutes nothing yet — it still
    /// enforces the "no NAME, no 0183" gate). A malformed file is an
    /// error so a typo doesn't silently disable filtering.
    pub fn load(path: &Path) -> Result<Self> {
        let rules = match std::fs::read_to_string(path) {
            Ok(body) => {
                let file: FileFormat = serde_json::from_str(&body).with_context(|| {
                    format!("parsing NMEA 0183 filter config {}", path.display())
                })?;
                file.devices
                    .into_iter()
                    .filter_map(|e| NameKey::from_hex(&e.name).map(|k| (k, e.rule)))
                    .collect()
            }
            Err(_) => HashMap::new(),
        };
        Ok(Self {
            path: path.to_path_buf(),
            src_name: [None; 256],
            rules,
            observed: HashMap::new(),
        })
    }

    /// Persist the current rules to [`Self::path`]. Called after every
    /// edit so a change survives a pipeline restart.
    fn save(&self) -> Result<()> {
        if let Some(parent) = self.path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("creating {}", parent.display()))?;
        }
        let mut devices: Vec<DeviceEntry> = self
            .rules
            .iter()
            .map(|(k, rule)| DeviceEntry {
                name: k.to_hex(),
                rule: rule.clone(),
            })
            .collect();
        // Deterministic order so the file diffs cleanly.
        devices.sort_by(|a, b| a.name.cmp(&b.name));
        let body = serde_json::to_string_pretty(&FileFormat { devices })
            .context("serialising NMEA 0183 filter config")?;
        std::fs::write(&self.path, body).with_context(|| format!("writing {}", self.path.display()))
    }

    /// Record an ISO Address Claim: `src` now belongs to this NAME
    /// (the full 64-bit value from the PGN 60928 payload).
    pub fn note_address_claim(&mut self, src: u8, name: u64) {
        self.src_name[src as usize] = Some(NameKey(name));
    }

    /// Decide what to do with `src`'s converted 0183 for one record.
    pub fn decide(&self, src: u8) -> Decision<'_> {
        // A rule-less filter is a true no-op: pass every source through.
        // The "no learned NAME ⇒ drop all" gate below only makes sense
        // once the user has actually muted something, so an always-on
        // filter with an empty config file behaves like no filter at all.
        if self.rules.is_empty() {
            return Decision::Keep(&[]);
        }
        let Some(name) = self.src_name[src as usize] else {
            // No learned NAME — deliberately emit nothing.
            return Decision::DropAll;
        };
        match self.rules.get(&name) {
            Some(rule) if rule.muted => Decision::DropAll,
            Some(rule) => Decision::Keep(&rule.muted_sentences),
            None => Decision::Keep(&[]),
        }
    }

    /// Apply the filter to `buf`, a freshly-converted block of one or
    /// more `$`-prefixed NMEA 0183 sentences for source `src`. Records
    /// each formatter as observed (so a report lists it even when
    /// muted), then rewrites `buf` in place, dropping muted sentences
    /// (or all of them). Lines that don't start with `$` are left
    /// untouched.
    pub fn apply(&mut self, src: u8, buf: &mut String) {
        // Observe first — the report's "what could be sent" menu must
        // include muted sentences, so record before dropping anything.
        for line in buf.split_inclusive('\n') {
            if let Some(fmt) = sentence_formatter(line)
                && !self.observed.get(&src).is_some_and(|s| s.contains(fmt))
            {
                self.observed
                    .entry(src)
                    .or_default()
                    .insert(fmt.to_string());
            }
        }
        match self.decide(src) {
            Decision::DropAll => buf.clear(),
            Decision::Keep(muted) => {
                if muted.is_empty() {
                    return;
                }
                let mut kept = String::with_capacity(buf.len());
                for line in buf.split_inclusive('\n') {
                    match sentence_formatter(line) {
                        Some(fmt) if muted.iter().any(|m| m == fmt) => {}
                        _ => kept.push_str(line),
                    }
                }
                *buf = kept;
            }
        }
    }

    /// Apply a PGN 262657 Set command addressed by source. `sentence`
    /// is a 3-letter formatter or [`ALL_SENTENCES`] for the whole
    /// device. A source with no learned NAME can't be persisted (we
    /// key rules by NAME) — such devices produce no 0183 anyway, so
    /// this is a no-op returning `false`. Otherwise the rule is updated
    /// and the config saved; returns `true`.
    pub fn set(&mut self, src: u8, sentence: &str, muted: bool) -> bool {
        let Some(name) = self.src_name[src as usize] else {
            log::debug!("PGN 262657 Set for src {src} ignored: no learned NAME");
            return false;
        };
        let rule = self.rules.entry(name).or_default();
        if sentence == ALL_SENTENCES {
            rule.muted = muted;
        } else if muted {
            if !rule.muted_sentences.iter().any(|s| s == sentence) {
                rule.muted_sentences.push(sentence.to_string());
            }
        } else {
            rule.muted_sentences.retain(|s| s != sentence);
        }
        // Drop a rule that no longer constrains anything, so the config
        // file doesn't accrue empty entries.
        if !rule.muted && rule.muted_sentences.is_empty() {
            self.rules.remove(&name);
        }
        if let Err(e) = self.save() {
            log::warn!("failed to persist NMEA 0183 filter change: {e:#}");
        }
        true
    }

    /// Build the per-source Report rows the pipeline broadcasts as PGN
    /// 262657 so the TUI can render the device/sentence matrix: for
    /// every source we've observed, an [`ALL_SENTENCES`] row carrying
    /// the whole-source mute state, then one row per observed formatter
    /// with its effective mute state.
    pub fn report(&self) -> Vec<FilterReportRow> {
        let mut rows = Vec::new();
        let mut srcs: Vec<u8> = self.observed.keys().copied().collect();
        srcs.sort_unstable();
        for src in srcs {
            let source_muted = matches!(self.decide(src), Decision::DropAll);
            rows.push(FilterReportRow {
                src,
                sentence: ALL_SENTENCES.to_string(),
                muted: source_muted,
            });
            if let Some(formatters) = self.observed.get(&src) {
                for fmt in formatters {
                    let muted = source_muted
                        || matches!(self.decide(src), Decision::Keep(m) if m.iter().any(|s| s == fmt));
                    rows.push(FilterReportRow {
                        src,
                        sentence: fmt.clone(),
                        muted,
                    });
                }
            }
        }
        rows
    }

    /// Apply a PGN 262657 `Set` payload (`[function, src, s0, s1, s2,
    /// muted, …]`) to the filter. Malformed / empty-sentence payloads
    /// are ignored.
    pub fn apply_set_frame(&mut self, data: &[u8]) {
        if data.len() < 6 {
            return;
        }
        let src = data[1];
        let sentence = std::str::from_utf8(&data[2..5])
            .unwrap_or("")
            .trim_matches(|c| c == '\0' || c == ' ');
        if sentence.is_empty() {
            return;
        }
        self.set(src, sentence, data[5] != 0);
    }

    /// One PGN 262657 `Report` frame per [`Self::report`] row, for the
    /// server to answer a client's connect / `Set` / `Request` with, so
    /// the client renders the current device/sentence mute matrix. Every
    /// frame carries `timestamp` (the moment the report was generated) —
    /// a real timestamp rather than the `None` the old broadcast used, so
    /// the decoded JSON has a `timestamp` field like every other record.
    pub fn report_frames(&self, timestamp: Option<String>) -> Vec<RawFrame> {
        self.report()
            .into_iter()
            .map(|row| {
                let b = row.sentence.as_bytes();
                let data = [
                    FILTER_FN_REPORT,
                    row.src,
                    *b.first().unwrap_or(&b' '),
                    *b.get(1).unwrap_or(&b' '),
                    *b.get(2).unwrap_or(&b' '),
                    u8::from(row.muted),
                    0xff,
                    0xff,
                ];
                RawFrame::new(timestamp.clone(), 7, PGN_NMEA0183_FILTER, 0, 255, data)
            })
            .collect()
    }
}

/// Extract the 3-letter sentence formatter from a `$ttFFF,…` line
/// (`tt` = 2-letter talker). Returns `None` for lines that aren't a
/// `$`-sentence or are too short.
fn sentence_formatter(line: &str) -> Option<&str> {
    let s = line.strip_prefix('$')?;
    // s = "ttFFF,…". Formatter is the 3 chars after the 2 talker chars.
    s.get(2..5)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Arbitrary distinct 64-bit NAMEs for the tests.
    const NAME_SPEED: u64 = 0x0004_0000_1066_7913;
    const NAME_AIRMAR: u64 = 0x0004_0000_0761_9208;

    fn filter_with(rules: &[(u64, DeviceRule)]) -> NmeaFilter {
        let mut f = NmeaFilter {
            path: PathBuf::from("/dev/null"),
            src_name: [None; 256],
            rules: HashMap::new(),
            observed: HashMap::new(),
        };
        for (name, rule) in rules {
            f.rules.insert(NameKey(*name), rule.clone());
        }
        f
    }

    #[test]
    fn no_name_drops_everything_once_a_rule_exists() {
        // The unknown-NAME gate only engages when the filter has at
        // least one rule; seed an unrelated mute so it's active.
        let mut f = filter_with(&[(
            NAME_SPEED,
            DeviceRule {
                muted: true,
                ..Default::default()
            },
        )]);
        let mut buf = "$CBVHW,,T,,M,0.0,N,0.0,K*54\r\n".to_string();
        f.apply(33, &mut buf); // src 33 never claimed
        assert!(buf.is_empty());
    }

    #[test]
    fn empty_filter_passes_everything() {
        // No rules at all ⇒ a source with no learned NAME still emits.
        let mut f = filter_with(&[]);
        let mut buf = "$CBVHW,,T,,M,0.0,N,0.0,K*54\r\n".to_string();
        f.apply(33, &mut buf);
        assert_eq!(buf, "$CBVHW,,T,,M,0.0,N,0.0,K*54\r\n");
    }

    #[test]
    fn known_name_no_rule_passes() {
        let mut f = filter_with(&[]);
        f.note_address_claim(33, NAME_SPEED);
        let mut buf = "$CBVHW,,T,,M,0.0,N,0.0,K*54\r\n".to_string();
        f.apply(33, &mut buf);
        assert_eq!(buf, "$CBVHW,,T,,M,0.0,N,0.0,K*54\r\n");
    }

    #[test]
    fn muted_source_drops_everything() {
        let mut f = filter_with(&[(
            NAME_SPEED,
            DeviceRule {
                muted: true,
                ..Default::default()
            },
        )]);
        f.note_address_claim(33, NAME_SPEED);
        let mut buf = "$CBVHW,,T,,M,0.0,N,0.0,K*54\r\n".to_string();
        f.apply(33, &mut buf);
        assert!(buf.is_empty());
    }

    #[test]
    fn muted_sentence_drops_only_that_formatter() {
        let mut f = filter_with(&[(
            NAME_AIRMAR,
            DeviceRule {
                muted: false,
                muted_sentences: vec!["VLW".to_string()],
            },
        )]);
        f.note_address_claim(35, NAME_AIRMAR);
        // Airmar DST emits VHW + VLW + MTW; mute only VLW.
        let mut buf =
            "$CDVHW,,T,,M,4.6,N,8.6,K*5E\r\n$CDVLW,14467.8,N,14467.8,N*4A\r\n$CDMTW,17.0,C*01\r\n"
                .to_string();
        f.apply(35, &mut buf);
        assert_eq!(buf, "$CDVHW,,T,,M,4.6,N,8.6,K*5E\r\n$CDMTW,17.0,C*01\r\n");
    }

    #[test]
    fn degenerate_name_is_still_keyed() {
        // A device that claims an "unavailable" manufacturer code still
        // has a unique 64-bit NAME — it must be filterable, not dropped.
        let degenerate: u64 = 0xffff_ff00_ffe0_ffff;
        let mut f = filter_with(&[]);
        f.path = std::env::temp_dir().join("canboat-nmea-filter-degen.json");
        f.note_address_claim(53, degenerate);
        let mut buf = "$DFMWV,308.3,R,20.9,K,A*09\r\n".to_string();
        f.apply(53, &mut buf);
        assert_eq!(buf, "$DFMWV,308.3,R,20.9,K,A*09\r\n"); // passes: known NAME
        assert!(f.set(53, ALL_SENTENCES, true)); // and can be muted
        let mut buf2 = "$DFMWV,308.3,R,20.9,K,A*09\r\n".to_string();
        f.apply(53, &mut buf2);
        assert!(buf2.is_empty());
    }

    #[test]
    fn set_by_source_resolves_to_name_and_reports() {
        let mut f = filter_with(&[]);
        f.path = std::env::temp_dir().join("canboat-nmea-filter-test.json");
        f.note_address_claim(35, NAME_AIRMAR);
        // Observe what the Airmar produces.
        let mut buf = "$CDVHW,x*01\r\n$CDVLW,y*02\r\n$CDMTW,z*03\r\n".to_string();
        f.apply(35, &mut buf);
        // Mute only VLW via a Set addressed by source.
        assert!(f.set(35, "VLW", true));
        let mut buf2 = "$CDVHW,x*01\r\n$CDVLW,y*02\r\n$CDMTW,z*03\r\n".to_string();
        f.apply(35, &mut buf2);
        assert_eq!(buf2, "$CDVHW,x*01\r\n$CDMTW,z*03\r\n");
        // Report exposes ALL + every observed formatter, VLW muted.
        let rows = f.report();
        let vlw = rows
            .iter()
            .find(|r| r.src == 35 && r.sentence == "VLW")
            .unwrap();
        assert!(vlw.muted);
        let vhw = rows
            .iter()
            .find(|r| r.src == 35 && r.sentence == "VHW")
            .unwrap();
        assert!(!vhw.muted);
        assert!(
            rows.iter()
                .any(|r| r.src == 35 && r.sentence == ALL_SENTENCES && !r.muted)
        );
    }

    #[test]
    fn set_on_unknown_source_is_noop() {
        let mut f = filter_with(&[]);
        f.path = std::env::temp_dir().join("canboat-nmea-filter-noop.json");
        assert!(!f.set(99, ALL_SENTENCES, true)); // src 99 never claimed
    }

    #[test]
    fn set_frame_applies_and_report_frames_reflect_it() {
        let mut f = filter_with(&[]);
        f.path = std::env::temp_dir().join("canboat-nmea-filter-setframe.json");
        f.note_address_claim(35, NAME_AIRMAR);
        // Observe a VHW sentence so it shows up in the report matrix.
        let mut buf = "$CDVHW,,T,,M,4.6,N,8.6,K*5E\r\n".to_string();
        f.apply(35, &mut buf);
        assert_eq!(buf, "$CDVHW,,T,,M,4.6,N,8.6,K*5E\r\n"); // no rule yet

        // A TUI Set frame: mute VHW on src 35. The `\0` pad exercises
        // the trim in `apply_set_frame`.
        f.apply_set_frame(&[FILTER_FN_SET, 35, b'V', b'H', b'W', 1]);
        let mut buf = "$CDVHW,,T,,M,4.6,N,8.6,K*5E\r\n".to_string();
        f.apply(35, &mut buf);
        assert!(buf.is_empty(), "VHW should now be muted");

        // report_frames advertises the state: a VHW row muted for src 35.
        let frames = f.report_frames(Some("2026-07-09T00:00:00.000Z".into()));
        assert!(
            frames.iter().all(|fr| fr.timestamp.is_some()),
            "report frames must carry a timestamp"
        );
        assert!(
            frames.iter().any(|fr| fr.pgn == PGN_NMEA0183_FILTER
                && fr.data.len() >= 6
                && fr.data[0] == FILTER_FN_REPORT
                && fr.data[1] == 35
                && &fr.data[2..5] == b"VHW"
                && fr.data[5] == 1),
            "expected a muted VHW report row for src 35, got {frames:?}"
        );
    }

    #[test]
    fn is_set_frame_matches_only_the_set_function() {
        let set = RawFrame::new(
            None,
            7,
            PGN_NMEA0183_FILTER,
            0,
            255,
            [FILTER_FN_SET, 35, b'V', b'H', b'W', 1, 0xff, 0xff],
        );
        assert!(is_set_frame(&set));
        let report = RawFrame::new(
            None,
            7,
            PGN_NMEA0183_FILTER,
            0,
            255,
            [FILTER_FN_REPORT, 35, b'V', b'H', b'W', 1, 0xff, 0xff],
        );
        assert!(!is_set_frame(&report), "a Report is not a Set");
        let other = RawFrame::new(None, 6, 127251, 5, 255, [0u8; 8]);
        assert!(!is_set_frame(&other), "a non-262657 PGN is not a Set");
    }

    #[test]
    fn formatter_extraction() {
        assert_eq!(sentence_formatter("$CBVHW,x*54\r\n"), Some("VHW"));
        assert_eq!(sentence_formatter("$AARSA,2.0*6B\r\n"), Some("RSA"));
        assert_eq!(sentence_formatter("!AIVDM,1*7B\r\n"), None);
        assert_eq!(sentence_formatter("$AB\r\n"), None);
    }
}
