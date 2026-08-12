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

/// `--to json` leads with the same producer banner `analyzer` emits.
/// `n2kd` reads `"units"` off it to pick the schema it rebuilds records
/// against, so a missing banner means a stream decoded in the wrong
/// unit system, silently.
#[test]
fn json_leads_with_the_version_banner() {
    let out = run(&["convert", "--to", "json"], PLAIN);
    let s = String::from_utf8(out).unwrap();
    let banner = s.lines().next().expect("a first line");
    assert!(banner.starts_with(r#"{"version":"#), "banner: {banner}");
    assert!(banner.contains(r#""units":"si""#), "banner: {banner}");
    assert!(
        banner.contains(r#""showLookupValues":false"#),
        "banner: {banner}"
    );
    // …and the records still follow it.
    assert!(
        s.lines()
            .nth(1)
            .is_some_and(|l| l.contains("\"pgn\":127251"))
    );
}

#[test]
fn banner_tracks_the_output_shape() {
    let metric = String::from_utf8(run(
        &["convert", "--to", "json", "--units", "metric", "--nv"],
        PLAIN,
    ))
    .unwrap();
    let banner = metric.lines().next().unwrap();
    assert!(banner.contains(r#""units":"std""#), "banner: {banner}");
    assert!(
        banner.contains(r#""showLookupValues":true"#),
        "banner: {banner}"
    );
}

#[test]
fn no_banner_suppresses_it_and_text_never_has_one() {
    let bare = String::from_utf8(run(&["convert", "--to", "json", "--no-banner"], PLAIN)).unwrap();
    assert!(
        !bare.contains("\"version\""),
        "banner leaked through --no-banner: {bare}"
    );
    // canboat C emits no banner in text mode either.
    let text = String::from_utf8(run(&["convert", "--to", "text"], PLAIN)).unwrap();
    assert!(!text.contains("\"version\""), "text got a banner: {text}");
}

/// The banner must not break `--from json`: it is not a record, and the
/// re-encoder skips `{"version":…}` lines. Uses PGN 127250 rather than
/// the shared `PLAIN` fixture because 127251's trailing Reserved field
/// renders as a hex display string that the encoder can't take back —
/// a pre-existing limitation, unrelated to the banner.
const HEADING: &[u8] = b"2026-01-01T00:00:00.000Z,3,127250,27,255,8,00,ca,8f,ff,7f,ff,7f,fd\n";

#[test]
fn json_round_trips_through_its_own_banner() {
    let json = run(&["convert", "--to", "json", "--nv"], HEADING);
    assert!(String::from_utf8_lossy(&json).starts_with(r#"{"version":"#));
    let back = run(&["convert", "--from", "json", "--to", "plain"], &json);
    assert_eq!(
        String::from_utf8(back).unwrap().as_bytes(),
        HEADING,
        "round trip through the banner is not byte-exact"
    );
}

/// `--from json` reads bare physical values against the unit system the
/// input's *banner* declares, not the one `--units` asks for on output.
/// That makes the pair a unit converter — and, more importantly, stops a
/// canboat C `"units":"std"` stream from having its degrees re-encoded
/// as radians now that SI is the default.
#[test]
fn json_input_follows_the_banners_units() {
    // Metric out: heading in degrees, banner says "std".
    let metric = run(&["convert", "--to", "json", "--units", "metric"], HEADING);
    let s = String::from_utf8(metric.clone()).unwrap();
    assert!(s.lines().next().unwrap().contains(r#""units":"std""#));
    assert!(s.contains(r#""heading":210.9"#), "metric json: {s}");

    // Feed it back with the default (SI) output: the same record must
    // come out in radians, not have 210.9 read as radians.
    let si = String::from_utf8(run(&["convert", "--from", "json"], &metric)).unwrap();
    assert!(si.lines().next().unwrap().contains(r#""units":"si""#));
    assert!(si.contains(r#""heading":3.68"#), "si json: {si}");
}

#[test]
fn the_banner_outranks_the_units_flag_on_input() {
    // SI-bannered input, `--units metric` on the command line: the flag
    // governs the output, the banner governs how the input is read, so
    // the wire bits survive.
    let si_json = run(&["convert", "--to", "json"], HEADING);
    let back = run(
        &[
            "convert", "--from", "json", "--to", "plain", "--units", "metric",
        ],
        &si_json,
    );
    assert_eq!(
        String::from_utf8(back).unwrap().as_bytes(),
        HEADING,
        "banner-declared input units were ignored"
    );
}

#[test]
fn actisense_ebl_emits_binary_framing() {
    // EBL is output-only; assert it produced a binary record that opens
    // with the `ESC SOH` timestamp-header framing.
    let ebl = run(&["convert", "--to", "actisense-ebl"], PLAIN);
    assert!(ebl.len() > 16, "ebl too short: {ebl:?}");
    assert_eq!(&ebl[..2], &[0x1b, 0x01], "expected ESC SOH opener");
}
