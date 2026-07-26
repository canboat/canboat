// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Generate the fast-packet lookup table for the "mixed" PGN range
//! (0x1F000..0x1FFFF) at build time from the canboat PGN database, so
//! the single/fast decision always tracks the spec. PGNs outside that
//! range are decided by range alone (see `fastpacket::packet_type`).
//!
//! The table is consumed by `canboat_io::fastpacket::packet_type`:
//! - the SocketCAN encoder uses it so it never wraps an 8-byte
//!   single-frame PGN (e.g. PGN 127508 Battery Status) in a
//!   fast-packet shell;
//! - the `socketcan-serial` binary's stdout reassembler uses it to
//!   correctly identify single vs fast arriving from the bus.
//!
//! Mirrors `canboat/socketcan-serial/gen-fastpacket-table.py`.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};

const MIXED_START: u32 = 0x1F000;
const MIXED_END: u32 = 0x20000; // exclusive; table covers 0x1F000..0x1FFFF
const PROPRIETARY_START: u32 = 0x1FF00;

fn main() {
    let manifest = env::var("CARGO_MANIFEST_DIR").unwrap();
    // The repository's own generated schema — see canboat-core/build.rs for
    // why the vendored copy under canboat-core/data/ is gone.
    let json_path = Path::new(&manifest).join("../../../docs/canboat.json");
    println!("cargo:rerun-if-changed={}", json_path.display());
    println!("cargo:rerun-if-changed=build.rs");

    let text = fs::read_to_string(&json_path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", json_path.display()));
    let db: serde_json::Value =
        serde_json::from_str(&text).expect("canboat.json is not valid JSON");

    let size = (MIXED_END - MIXED_START) as usize;
    // Default: the manufacturer-proprietary tail is fast-packet, the
    // standardized part defaults to single frame; explicit PGN
    // definitions override below.
    let mut fast = vec![false; size];
    for (i, slot) in fast.iter_mut().enumerate() {
        *slot = MIXED_START + i as u32 >= PROPRIETARY_START;
    }

    for p in db["PGNs"].as_array().expect("PGNs array") {
        let pgn = p["PGN"].as_u64().unwrap_or(0) as u32;
        if !(MIXED_START..MIXED_END).contains(&pgn) {
            continue;
        }
        if p.get("Fallback").and_then(|v| v.as_bool()).unwrap_or(false) {
            continue; // range fallbacks, not a real PGN
        }
        fast[(pgn - MIXED_START) as usize] = p["Type"].as_str() == Some("Fast");
    }

    let mut out = String::new();
    out.push_str(&format!(
        "pub const FASTPACKET_MIXED_START: u32 = 0x{MIXED_START:05X};\n"
    ));
    out.push_str(&format!(
        "pub const FASTPACKET_MIXED_END: u32 = 0x{MIXED_END:05X};\n"
    ));
    out.push_str(&format!(
        "/// 1 = fast-packet, 0 = single-frame, for PGNs 0x{:05X}..0x{:05X}.\n",
        MIXED_START,
        MIXED_END - 1
    ));
    out.push_str(&format!(
        "pub static FASTPACKET_MIXED: [bool; 0x{size:X}] = ["
    ));
    for (i, v) in fast.iter().enumerate() {
        if i % 32 == 0 {
            out.push_str("\n    ");
        }
        out.push_str(if *v { "true," } else { "false," });
    }
    out.push_str("\n];\n");

    let dest = PathBuf::from(env::var("OUT_DIR").unwrap()).join("fastpacket_table.rs");
    fs::write(&dest, out).expect("writing fastpacket_table.rs");
}
