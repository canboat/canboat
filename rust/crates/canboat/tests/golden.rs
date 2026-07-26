// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

// argv[0] routing (the `analyzer` alias) is a Unix symlink concept;
// `CommandExt::arg0` is Unix-only. The golden fixtures live in a
// sibling canboat checkout that isn't present on Windows CI anyway,
// so gate the whole module to Unix.
#![cfg(unix)]

//! Golden tests against canboat's `analyzer/tests/*.in` / `*.out`
//! files. Each case feeds an `.in` through the `canboat` binary
//! invoked as `analyzer` (its argv[0] alias) with the same flags
//! canboat C passes, then byte-diffs against the corresponding `.out`.
//!
//! Test data is read straight from the sibling canboat checkout at
//! `../canboat/analyzer/tests/`. If that directory isn't present the
//! tests skip gracefully — they're not blockers for users building
//! canboat-rs in isolation. Set `CANBOAT_GOLDEN=required` (CI does)
//! to turn a missing fixture directory into a hard failure instead.
//!
//! New cases are added by name. v0 covers the ones our decoder can
//! already render; the rest will be added (with documentation of
//! missing features) as their support lands.

use std::io::Write;
use std::os::unix::process::CommandExt;
use std::path::PathBuf;
use std::process::{Command, Stdio};

/// Where the canboat C repo's `analyzer/tests/` lives, relative to
/// this crate.
///
/// Resolution order, first match wins:
///
/// 1. `$CANBOAT_DIR` (when set) — explicit override for CI / local
///    development against an in-flight branch.
/// 2. `../canboat-*` — every sibling worktree (alphabetical, so a
///    `canboat-fix-pk` worktree carrying staged schema changes is
///    preferred over the main `canboat/` checkout while the PR is
///    open). Skip non-directories and any entry whose
///    `analyzer/tests/` doesn't exist.
/// 3. `../canboat` — the default sibling clone.
///
/// Returns `None` only when none of the above resolves — in that
/// case `run_case_skipping` skips the test gracefully so users
/// building canboat-rs in isolation aren't blocked.
fn canboat_tests_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("CANBOAT_DIR") {
        let p = PathBuf::from(dir).join("analyzer").join("tests");
        if p.is_dir() {
            return Some(p);
        }
    }
    let manifest: PathBuf = env!("CARGO_MANIFEST_DIR").into();
    let github_root = manifest.parent()?.parent()?.parent()?; // …/github
    let mut worktree_candidates: Vec<PathBuf> = std::fs::read_dir(github_root)
        .ok()?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("canboat-"))
                && p.is_dir()
        })
        .collect();
    worktree_candidates.sort();
    for p in worktree_candidates {
        let cand = p.join("analyzer").join("tests");
        if cand.is_dir() {
            return Some(cand);
        }
    }
    let p = github_root.join("canboat").join("analyzer").join("tests");
    if p.is_dir() { Some(p) } else { None }
}

fn canboat_path() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_canboat"))
}

/// Drive the analyzer binary on `<test_dir>/<in_name>` with `args`,
/// then byte-diff stdout against `<test_dir>/<expected_name>`.
fn run_case(in_name: &str, expected_name: &str, args: &[&str]) {
    run_case_skipping(in_name, expected_name, args, &[]);
}

/// Same as [`run_case`] but allow specific line indices (0-based) to
/// differ. Used for canboat-reference outputs that contain
/// canboat-specific quirks (e.g. reading uninitialized memory past
/// the declared payload) we deliberately don't replicate.
fn run_case_skipping(in_name: &str, expected_name: &str, args: &[&str], skip_lines: &[usize]) {
    let Some(dir) = canboat_tests_dir() else {
        if std::env::var_os("CANBOAT_GOLDEN").is_some_and(|v| v == "required") {
            panic!(
                "CANBOAT_GOLDEN=required but no canboat fixture directory found \
                 (set CANBOAT_DIR or provide a ../canboat checkout)"
            );
        }
        eprintln!("skipping: canboat test directory not available");
        return;
    };
    let input = std::fs::read(dir.join(in_name)).expect("read .in");
    let expected = std::fs::read(dir.join(expected_name)).expect("read .out");

    // Invoke the canboat binary as `analyzer` so its argv[0] alias
    // dispatches into the analyzer path — same as the installed shim.
    let mut child = Command::new(canboat_path())
        .arg0("analyzer")
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn analyzer");
    child
        .stdin
        .as_mut()
        .unwrap()
        .write_all(&input)
        .expect("write stdin");
    let out = child.wait_with_output().expect("wait analyzer");
    assert!(
        out.status.success(),
        "analyzer exited with {:?}\nstderr: {}",
        out.status,
        String::from_utf8_lossy(&out.stderr)
    );

    if skip_lines.is_empty() {
        if out.stdout != expected {
            let actual = String::from_utf8_lossy(&out.stdout);
            let expected_str = String::from_utf8_lossy(&expected);
            panic!(
                "Golden mismatch for {in_name} → {expected_name}\n\
                 --- expected ({} bytes) ---\n{}\n\
                 --- actual ({} bytes) ---\n{}\n",
                expected.len(),
                expected_str,
                out.stdout.len(),
                actual,
            );
        }
        return;
    }

    // Line-by-line compare with allowed skips.
    let actual_str = String::from_utf8(out.stdout).expect("utf-8 stdout");
    let expected_str = String::from_utf8(expected).expect("utf-8 expected");
    let actual_lines: Vec<&str> = actual_str.lines().collect();
    let expected_lines: Vec<&str> = expected_str.lines().collect();
    assert_eq!(
        actual_lines.len(),
        expected_lines.len(),
        "line count differs for {in_name} → {expected_name}: actual={} expected={}",
        actual_lines.len(),
        expected_lines.len(),
    );
    for (i, (a, e)) in actual_lines.iter().zip(expected_lines.iter()).enumerate() {
        if skip_lines.contains(&i) {
            continue;
        }
        if a != e {
            panic!(
                "Golden mismatch at {in_name}:{i}\n--- expected ---\n{e}\n--- actual ---\n{a}\n",
            );
        }
    }
}

#[test]
fn pgn_60928_json_nv() {
    // The simplest passing case: two PGN 60928 frames, JSON -nv.
    run_case(
        "pgn-60928.in",
        "pgn-60928-nv.out",
        &["--json", "--nv", "--fixtime", "pgn-test"],
    );
}

#[test]
fn pgn_126983_json_nv() {
    // Exercises ISO_NAME recursive decode, Reserved-as-hex, and
    // PartOfPrimaryKey "key":true annotation.
    run_case(
        "pgn-126983.in",
        "pgn-126983-nv.out",
        &["--json", "--nv", "--fixtime", "pgn-test"],
    );
}

/// Big multi-PGN regression test (24 PGNs covering most field types,
/// repeating sets 1 and 2, ISO_NAME, STRING_LAU, DYNAMIC fields,
/// unit conversions, lat/lon width padding). Skips one line at the
/// end of the GNSS Sats in View payload where canboat C reads from
/// uninitialized memory past the declared payload length — that
/// behavior is a canboat-specific quirk we deliberately don't
/// replicate, see analyzer/print.c::extractNumber and the static
/// `RawMessage::data[FASTPACKET_MAX_SIZE]` buffer reuse.
#[test]
fn pgn_test_json() {
    // Line 6 (0-based) in pgn-test-json.out is the PGN 129540 "GNSS
    // Sats in View" frame. The last sat's `Range residuals` field
    // crosses the payload-end boundary; canboat extracts whatever was
    // in memory (happens to be `0.00000`), we correctly drop it.
    run_case_skipping(
        "pgn-test.in",
        "pgn-test-json.out",
        &["--json", "--fixtime", "pgn-test"],
        &[6],
    );
}

/// Same corpus as `pgn_test_json` but with `--nv`, exercising the
/// `{"value":N,"name":"..."}` lookup-object form, `"key":true`
/// annotation on primary-key Lookup / MMSI fields, the bit-flag
/// object form for BITLOOKUP, and the {value,name} wrappers for
/// Date / Time / Pgn fields.
#[test]
fn pgn_test_json_nv() {
    run_case_skipping(
        "pgn-test.in",
        "pgn-test-json-nv.out",
        &["--json", "--nv", "--fixtime", "pgn-test"],
        &[6],
    );
}

/// Actisense N2K ASCII input format + `-debug` byte/bit annotation +
/// `-nv` lookup form. Whole canboat reference output matches byte-
/// for-byte (no skip needed — the Actisense corpus has nothing that
/// reads past the payload like PGN 129540 does).
#[test]
fn pgn_test_actisense_json_nv_debug() {
    run_case(
        "pgn-test-actisense.in",
        "pgn-test-actisense.out",
        &["--json", "--nv", "--debug", "--fixtime", "pgn-test"],
    );
}

/// Text mode + `-debug` byte/bit annotation against the canboat
/// reference. Exercises everything the JSON tests do plus the
/// canboat text-mode quirks: repeating-field iteration suffixes
/// (`PRN 1 =`, `PRN 2 =`), uppercase Binary, "None" for empty
/// BITLOOKUPs, MMSI with quotes, PGN with description in parens.
///
/// Same line-6 skip as `pgn_test_json` — the PGN 129540 final-sat
/// short-payload extraction differs from canboat's uninitialised-
/// memory read.
#[test]
fn pgn_test_text_debug() {
    run_case_skipping("pgn-test.in", "pgn-test.out", &["--debug"], &[6]);
}

/// Actisense N2K ASCII input, plain text output (no -debug). Tests
/// the basic parse path plus canboat's "Unknown" display for
/// `NotAvailable` numeric fields.
#[test]
fn actisense_format_text() {
    run_case("actisense-format.in", "actisense-format.out", &[]);
}

/// Text + `-debug` of a Seatalk PGN-65379 capture. Exercises the
/// fast-packet PGN-65379 corner cases.
#[test]
fn pgn_65379_text_debug() {
    run_case("pgn-65379-test.in", "pgn-65379-test.out", &["--debug"]);
}

/// Issue #623 regression: a fast-packet first frame with no payload
/// followed by a single continuation frame produces a 2-frame
/// reassembly where the analyzer should fill missing bytes with 0xFF
/// (not read uninitialized memory). Exercises the extract_bits
/// padding path for fields that start in-bounds and run off the end.
#[test]
fn short_frame_text_debug() {
    run_case("short-frame.in", "short-frame.out", &["--debug"]);
}

/// Same pgn-test corpus through `-json -debug` (no -nv). Exercises
/// the debug-mode bytes annotation across every JSON path that's
/// shaped differently from -nv: Lookup as string (not {value,name}),
/// BitField as array of strings (not array of objects), Date/Time
/// as named strings, PGN as bare numeric value (name only with -nv).
/// Same line-6 skip as `pgn_test_json` for the canboat-uninitialized-
/// memory read on the GNSS Sats in View last sat.
#[test]
fn pgn_test_json_debug() {
    // See `pgn_test_json` for skip rationale.
    run_case_skipping(
        "pgn-test.in",
        "pgn-test-json-debug.out",
        &["--json", "--debug", "--fixtime", "pgn-test"],
        &[6],
    );
}

/// pgn-test corpus through `-json -nv -debug`. Exercises the full
/// debug JSON output: every field wrapped, lookup objects keep
/// {value, name} form with name nullable, PGN reference fields
/// keyed with name even when unresolved. Same line-6 skip.
#[test]
fn pgn_test_json_nv_debug() {
    // See `pgn_test_json` for skip rationale.
    run_case_skipping(
        "pgn-test.in",
        "pgn-test-json-nv-debug.out",
        &["--json", "--nv", "--debug", "--fixtime", "pgn-test"],
        &[6],
    );
}

/// Mixed PLAIN_MIX_FAST capture exercising DYNAMIC_FIELD_KEY /
/// DYNAMIC_FIELD_LENGTH / DYNAMIC_FIELD_VALUE on PGN 130824 (B&G
/// key-value data). Two reassembled / coalesced records and three
/// frames that pass through individually (the format intentionally
/// interleaves coalesced FAST records with raw 8-byte continuations).
#[test]
fn mixed_format_text_debug() {
    run_case(
        "mixed-format.in",
        "mixed-format.out",
        &["--format", "plain_mix_fast", "--debug"],
    );
}

/// Switch-multi capture: exercises the synthetic-PGN range (PGN
/// 262386 = `ACTISENSE_BEM + 0xf2`, "Actisense: System status",
/// defined in canboat's `analyzer/pgn.h` but not in canboat.json —
/// loaded from `data/synthetic-pgns.json`), plus STRING_FIX's
/// embedded-NUL truncation on a Mastervolt Product Information
/// field that packs two C-strings into one fixed-width slot.
#[test]
fn switch_multi_text_debug() {
    run_case(
        "switch-multi-to-one-line.in",
        "switch-multi-to-one-line.out",
        &["--debug"],
    );
}

/// `-geo dd|dm|dms` lat/lon formatting. The canboat reference test
/// runs the analyzer three times and concatenates the outputs; do
/// the same here.
#[test]
fn dms_format_text() {
    let Some(dir) = canboat_tests_dir() else {
        if std::env::var_os("CANBOAT_GOLDEN").is_some_and(|v| v == "required") {
            panic!(
                "CANBOAT_GOLDEN=required but no canboat fixture directory found \
                 (set CANBOAT_DIR or provide a ../canboat checkout)"
            );
        }
        eprintln!("skipping: canboat test directory not available");
        return;
    };
    let input = std::fs::read(dir.join("dms-format.in")).expect("read .in");
    let expected = std::fs::read(dir.join("dms-format.out")).expect("read .out");
    let mut combined = Vec::new();
    for geo in ["dd", "dm", "dms"] {
        let mut child = Command::new(canboat_path())
            .arg0("analyzer")
            .args(["--geo", geo])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .expect("spawn analyzer");
        child.stdin.as_mut().unwrap().write_all(&input).unwrap();
        let out = child.wait_with_output().expect("wait analyzer");
        assert!(out.status.success(), "analyzer failed");
        combined.extend_from_slice(&out.stdout);
    }
    assert_eq!(
        combined,
        expected,
        "dms-format mismatch\nactual:\n{}\nexpected:\n{}",
        String::from_utf8_lossy(&combined),
        String::from_utf8_lossy(&expected),
    );
}

/// Big multi-scenario fast-packet recombination test. Covers in-order
/// frames with interspersed short frames, two parallel fast-packets
/// from the same source, out-of-order frames, a sequence where the
/// duplicate-frame-index recovery has to discard a partial and
/// continue, and a 32-frame (max-size) fast-packet from an unknown
/// PGN that has to fall through to the `0x1FF00-0x1FFFF` catch-all.
/// Exercises: reassembly slot reuse, fallback PGN lookup, the count=0
/// repeating-set quirk, BINARY field width clamping.
#[test]
fn recombine_frames_text_debug() {
    run_case("recombine-frames.in", "recombine-frames.out", &["--debug"]);
}
