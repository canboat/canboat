// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Persistent NAME → product-info cache.
//!
//! On the wire, every N2K device advertises a globally-unique 64-bit
//! **NAME** via PGN 60928 (ISO Address Claim). That NAME never
//! changes; the src address it lives at can. And a device's product
//! metadata (PGN 126996 Product Information, PGN 126998 Configuration
//! Information) is only broadcast on request, so a log capture will
//! usually not include it — leaving the DeviceDetail screen with
//! `(unknown)` manufacturer / empty model.
//!
//! This cache remembers what we've learned per-NAME across
//! canboat-tui runs. On startup we load it and enrich the current
//! session's device list with anything the log or bus hasn't shown
//! us yet; on exit we save everything back, merging in whatever new
//! info this session gathered. Because NAME is unique the cache
//! never needs invalidation — a stale entry can only be superseded
//! by a fresh observation of the same NAME.
//!
//! Storage format: a JSON list of entries, each keyed by NAME
//! stringified as `"<manufacturer_code>-<unique_number>"` (both
//! decimal). The list shape (rather than an object keyed by NAME)
//! is deliberately serde-friendly and human-readable, and lets us
//! carry per-entry timestamps that a plain map couldn't.
//!
//! Path: `$XDG_CONFIG_HOME/canboat-tui/device-cache.json` when the
//! env var is set, otherwise `~/.config/canboat-tui/device-cache.json`.

use std::collections::HashMap;
use std::path::PathBuf;

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

/// (manufacturer_code, unique_number) pair — the two ISO NAME
/// fields that uniquely identify a device (Unique Number is 21 bits,
/// unique per manufacturer; Manufacturer Code is 11 bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NameKey {
    pub manufacturer_code: u16,
    pub unique_number: u32,
}

impl NameKey {
    /// Textual key used as the JSON map key when persisted.
    pub fn to_token(self) -> String {
        format!("{}-{}", self.manufacturer_code, self.unique_number)
    }

    pub fn from_token(s: &str) -> Option<Self> {
        let (mfr, uid) = s.split_once('-')?;
        Some(Self {
            manufacturer_code: mfr.parse().ok()?,
            unique_number: uid.parse().ok()?,
        })
    }
}

/// Everything we've learned about one device across all past
/// canboat-tui sessions. Every optional field lives here so a
/// partial observation (e.g. we saw the ISO Claim but not the
/// Product Information) doesn't wipe fields learned earlier.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CachedInfo {
    // ISO NAME decomposition (all from PGN 60928).
    pub manufacturer_name: Option<String>,
    pub device_function: Option<u32>,
    pub device_class: Option<u32>,
    pub industry_group: Option<u32>,

    // PGN 126996 Product Information.
    pub product_code: Option<u32>,
    pub model_id: Option<String>,
    pub software_version: Option<String>,
    pub model_version: Option<String>,
    pub model_serial_code: Option<String>,
    pub nmea_db_version: Option<u32>,
    pub certification_level: Option<u32>,
    pub load_equivalency: Option<u32>,

    // PGN 126998 Configuration Information.
    pub installation_description_1: Option<String>,
    pub installation_description_2: Option<String>,
    pub manufacturer_information: Option<String>,

    /// ISO wire timestamp of the first observation we cached for
    /// this NAME. Only rewritten if the on-disk value is `None`
    /// or if we've never seen the NAME before.
    pub first_seen: Option<String>,
    /// ISO wire timestamp of the most recent observation. Refreshed
    /// on every merge.
    pub last_seen: Option<String>,
}

impl CachedInfo {
    /// Merge `other` into `self`, preferring `Some` values from
    /// `other` over existing ones (so a fresher observation
    /// supersedes an older one). Empty strings in `other` are
    /// treated as `None` so a PGN 126998 that emitted padding
    /// spaces for a missing field doesn't overwrite real data.
    pub fn merge_from(&mut self, other: CachedInfo) {
        fn take<T>(dst: &mut Option<T>, src: Option<T>) {
            if src.is_some() {
                *dst = src;
            }
        }
        fn take_str(dst: &mut Option<String>, src: Option<String>) {
            match src {
                Some(s) if !s.trim().is_empty() => *dst = Some(s),
                _ => {}
            }
        }
        take_str(&mut self.manufacturer_name, other.manufacturer_name);
        take(&mut self.device_function, other.device_function);
        take(&mut self.device_class, other.device_class);
        take(&mut self.industry_group, other.industry_group);
        take(&mut self.product_code, other.product_code);
        take_str(&mut self.model_id, other.model_id);
        take_str(&mut self.software_version, other.software_version);
        take_str(&mut self.model_version, other.model_version);
        take_str(&mut self.model_serial_code, other.model_serial_code);
        take(&mut self.nmea_db_version, other.nmea_db_version);
        take(&mut self.certification_level, other.certification_level);
        take(&mut self.load_equivalency, other.load_equivalency);
        take_str(
            &mut self.installation_description_1,
            other.installation_description_1,
        );
        take_str(
            &mut self.installation_description_2,
            other.installation_description_2,
        );
        take_str(
            &mut self.manufacturer_information,
            other.manufacturer_information,
        );
        if self.first_seen.is_none() {
            self.first_seen = other.first_seen.clone().or(other.last_seen.clone());
        }
        if let Some(ls) = other.last_seen {
            self.last_seen = Some(ls);
        }
    }
}

/// One row of the persisted on-disk cache.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct FileEntry {
    /// `"<manufacturer_code>-<unique_number>"`.
    pub name: String,
    #[serde(flatten)]
    pub info: CachedInfo,
}

/// Persisted wire shape — a wrapper object holding an entry list.
/// The outer wrapper leaves room for schema-level metadata later
/// (e.g. cache version, host that wrote it) without breaking older
/// readers.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
struct FileFormat {
    #[serde(default)]
    pub entries: Vec<FileEntry>,
}

/// Load the persisted cache. A missing / malformed file is treated
/// as an empty cache — the on-disk copy has no schema versioning
/// yet, and a corrupt file shouldn't crash the TUI at startup.
pub fn load_or_default(path: &PathBuf) -> HashMap<NameKey, CachedInfo> {
    let Ok(body) = std::fs::read_to_string(path) else {
        return HashMap::new();
    };
    let file: FileFormat = match serde_json::from_str(&body) {
        Ok(f) => f,
        Err(_) => return HashMap::new(),
    };
    file.entries
        .into_iter()
        .filter_map(|e| NameKey::from_token(&e.name).map(|k| (k, e.info)))
        .collect()
}

/// Write `cache` to `path`, creating the parent directory if
/// needed. On failure the caller is responsible for surfacing the
/// error (the TUI's exit path just logs it — an unwritable cache
/// isn't fatal, just annoying).
pub fn save(cache: &HashMap<NameKey, CachedInfo>, path: &PathBuf) -> Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("creating {}", parent.display()))?;
    }
    let mut entries: Vec<FileEntry> = cache
        .iter()
        .map(|(k, v)| FileEntry {
            name: k.to_token(),
            info: v.clone(),
        })
        .collect();
    // Deterministic ordering so `git diff` on the cache file is
    // readable when the user syncs it between machines.
    entries.sort_by(|a, b| a.name.cmp(&b.name));
    let file = FileFormat { entries };
    let body = serde_json::to_string_pretty(&file).context("serialising device cache")?;
    std::fs::write(path, body).with_context(|| format!("writing {}", path.display()))?;
    Ok(())
}

/// Resolve the default cache path, honouring `XDG_CONFIG_HOME`.
pub fn default_path() -> PathBuf {
    if let Ok(xdg) = std::env::var("XDG_CONFIG_HOME") {
        return PathBuf::from(xdg).join("canboat-tui/device-cache.json");
    }
    if let Ok(home) = std::env::var("HOME") {
        return PathBuf::from(home).join(".config/canboat-tui/device-cache.json");
    }
    PathBuf::from("canboat-tui-device-cache.json")
}
