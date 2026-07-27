//! keel - the CANboat PGN database tool. See DESIGN.md.
//!
//! The database under `database/` is the source of truth; keel validates it,
//! generates the analyzer's C tables and canboat.xml, decodes sample frames
//! and serves the editor.
//!
//! This is both a binary and a library. The library exists so the Rust runtime
//! can derive its compiled-in schema straight from `database/` at build time
//! (`crates/canboat-core/build.rs`) instead of parsing a generated
//! `docs/canboat.json` — one source of truth, and `cargo build` picks up a
//! YAML edit with no `make` in between.
//!
//! Keep the dependency closure tiny: keel is cargo-shimmed from
//! `analyzer/Makefile` and is now a build-dependency of the runtime, so
//! anything added here is paid for by every build. See
//! `tools/check-keel-deps.sh` and MERGE-CANBOAT-RS.md §9.

pub mod bits;
pub mod cformat;
pub mod check;
pub mod decode;
pub mod derive;
pub mod edit;
pub mod emit_c;
pub mod emit_text;
pub mod emit_xml;
pub mod generate;
pub mod harvest;
pub mod model;
pub mod rules;
pub mod samples;
pub mod yamlio;

use std::fs;
use std::path::{Path, PathBuf};

pub fn find_repo_root(start: &Path) -> Result<PathBuf, String> {
    let mut path = start
        .canonicalize()
        .map_err(|e| format!("{}: {e}", start.display()))?;
    loop {
        if path.join("analyzer").is_dir() && path.join("docs").is_dir() {
            return Ok(path);
        }
        if !path.pop() {
            return Err(
                "cannot find canboat repository root (looked for analyzer/ and docs/)".into(),
            );
        }
    }
}

pub fn read_versions(root: &Path) -> Result<(String, String), String> {
    let text = fs::read_to_string(root.join("common/version.h"))
        .map_err(|e| format!("common/version.h: {e}"))?;
    let grab = |key: &str| -> Result<String, String> {
        text.lines()
            .find(|l| l.contains(&format!("#define {key} ")))
            .and_then(|l| l.split('"').nth(1))
            .map(String::from)
            .ok_or_else(|| format!("common/version.h: no #define {key}"))
    };
    Ok((grab("VERSION")?, grab("SCHEMA_VERSION")?))
}

/// Load and fully derive the database at `root` (the repo root). This is the
/// one call a consumer needs: it is what `keel check` / `generate` do before
/// touching anything.
pub fn load(root: &Path) -> Result<model::Database, String> {
    let (version, schema) = read_versions(root)?;
    let db_dir = root.join("database");
    if !db_dir.is_dir() {
        return Err(format!("no database/ tree at {}", db_dir.display()));
    }
    let mut db = yamlio::load_database(&db_dir, &version, &schema)?;
    derive::fill(&mut db)?;
    Ok(db)
}
