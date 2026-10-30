// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Capture the git short SHA (and a dirty flag) at build time so the
//! `analyzer` version banner can report the exact commit the binary was
//! built from. Non-fatal: outside a git worktree (a crates.io or source
//! tarball, `cargo package`'s verification build) vergen emits nothing
//! and `build_info.rs` reads the variables with `option_env!`, so the
//! banner reports `unknown`. Note that vergen does not return an error
//! in that case — it only warns — which is why the fallback lives on
//! the reading side rather than here.

//!
//! It also computes SCHEMA_HASH from the generated tables (see
//! `schema_hash` below).

use std::path::Path;
use std::{env, fs};

use vergen_gitcl::{Emitter, Gitcl};

fn main() {
    schema_hash();
    let gitcl = Gitcl::builder()
        .sha(true) // short SHA → VERGEN_GIT_SHA
        .dirty(false) // tracked-file changes only
        .build();
    if Emitter::default()
        .add_instructions(&gitcl)
        .and_then(|e| e.emit())
        .is_err()
    {
        println!("cargo:warning=git commit info unavailable; version banner will report 'unknown'");
    }
}

/// SCHEMA_HASH: FNV-1a/64 over the generated tables in `src/engine/`
/// (`*_generated*.rs`, sorted, name included). Two processes compare it
/// before trusting each other's field indices, so any change to the
/// decoding database must move it. It lives here rather than in
/// `schema_generated.rs`, where every database PR rewrote the same line
/// and so conflicted with every other one.
fn schema_hash() {
    let dir = Path::new("src/engine");
    let mut files: Vec<String> = fs::read_dir(dir)
        .expect("src/engine")
        .flatten()
        .filter_map(|e| e.file_name().into_string().ok())
        .filter(|n| n.ends_with(".rs") && n.contains("_generated"))
        .collect();
    files.sort();
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut feed = |bytes: &[u8]| {
        for &b in bytes {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for name in &files {
        let path = dir.join(name);
        println!("cargo:rerun-if-changed={}", path.display());
        feed(name.as_bytes());
        feed(&fs::read(&path).expect("generated table"));
    }
    println!("cargo:rerun-if-changed={}", dir.display());
    let out = Path::new(&env::var("OUT_DIR").expect("OUT_DIR")).join("schema_hash.rs");
    fs::write(out, format!("{h}\n")).expect("write schema_hash.rs");
}
