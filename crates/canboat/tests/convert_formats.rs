// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Integration smoke for the frame-level `convert --to` output formats.
//! Drives the real `canboat convert` binary: PLAIN in → each new format
//! → (for the text formats) parse back to PLAIN and confirm the header
//! and payload survive the round trip.

use std::io::Write;
use std::process::{Command, Stdio};

fn canboat() -> &'static str {
    env!("CARGO_BIN_EXE_canboat")
}

/// Run `canboat <args>` feeding `input` on stdin; return stdout, or
/// panic with stderr on a non-zero exit.
fn run(args: &[&str], input: &[u8]) -> Vec<u8> {
    let mut child = Command::new(canboat())
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn canboat");
    child
        .stdin
        .as_mut()
        .unwrap()
        .write_all(input)
        .expect("write stdin");
    let out = child.wait_with_output().expect("wait canboat");
    assert!(
        out.status.success(),
        "canboat {args:?} exited {:?}\nstderr: {}",
        out.status,
        String::from_utf8_lossy(&out.stderr)
    );
    out.stdout
}

/// A single received fast-packet PGN 127251 record.
const PLAIN: &[u8] = b"2026-01-01T00:00:00.000Z,3,127251,27,255,8,00,ca,8f,f3,ff,25,02,ff\n";
const BODY: &str = ",3,127251,27,255,8,00,ca,8f,f3,ff,25,02,ff";

#[test]
fn ydwg02_round_trips_to_plain() {
    let ydwg = run(&["convert", "--to", "ydwg02"], PLAIN);
    assert!(
        String::from_utf8_lossy(&ydwg).contains(" R 0DF1131B "),
        "ydwg: {ydwg:?}"
    );
    let back = run(&["convert", "--from", "ydwg02", "--to", "plain"], &ydwg);
    let s = String::from_utf8(back).unwrap();
    assert!(s.contains(BODY), "round-trip lost the body: {s}");
}

#[test]
fn actisense_round_trips_to_plain() {
    let acti = run(&["convert", "--to", "actisense"], PLAIN);
    assert!(
        String::from_utf8_lossy(&acti).starts_with('A'),
        "acti: {acti:?}"
    );
    let back = run(&["convert", "--from", "actisense", "--to", "plain"], &acti);
    let s = String::from_utf8(back).unwrap();
    assert!(s.contains(BODY), "round-trip lost the body: {s}");
}

#[test]
fn actisense_ebl_emits_binary_framing() {
    // EBL is output-only; assert it produced a binary record that opens
    // with the `ESC SOH` timestamp-header framing.
    let ebl = run(&["convert", "--to", "actisense-ebl"], PLAIN);
    assert!(ebl.len() > 16, "ebl too short: {ebl:?}");
    assert_eq!(&ebl[..2], &[0x1b, 0x01], "expected ESC SOH opener");
}
