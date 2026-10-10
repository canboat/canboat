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

/// Two different `--protocol` values are refused on every path, including
/// the frame-level one that never consults a PGN table.
#[test]
fn two_buses_are_refused_before_any_output() {
    let out = Command::new(canboat())
        .args([
            "convert",
            "--to",
            "plain",
            "--protocol",
            "nmea2000",
            "--protocol",
            "j1939",
        ])
        .stdin(Stdio::null())
        .output()
        .expect("run canboat");
    assert!(!out.status.success());
    assert!(out.stdout.is_empty());
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(stderr.contains("not supported yet"), "{stderr}");
}

/// A J1939 Engine Hours, Revolutions (65253) frame: 500 h, 10^9 revolutions.
const J1939_PLAIN: &[u8] = b"2026-01-01T00:00:00.000Z,6,65253,0,255,8,10,27,00,00,40,42,0f,00\n";
const J1939_BODY: &str = ",6,65253,0,255,8,10,27,00,00,40,42,0f,00\n";

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

/// The banner names the table the records were decoded against: the
/// same PGN number means different things on NMEA 2000 and J1939.
#[test]
fn banner_names_the_protocol() {
    let n2k = String::from_utf8(run(&["convert", "--to", "json"], PLAIN)).unwrap();
    let banner = n2k.lines().next().unwrap();
    assert!(
        banner.contains(r#""protocol":"nmea2000""#),
        "banner: {banner}"
    );

    let j1939 = String::from_utf8(run(
        &["convert", "--to", "json", "--protocol", "j1939"],
        J1939_PLAIN,
    ))
    .unwrap();
    let banner = j1939.lines().next().unwrap();
    assert!(banner.contains(r#""protocol":"j1939""#), "banner: {banner}");
}

/// A J1939 JSON stream re-encodes against the J1939 table when asked
/// to, and is refused, naming the fix, when read as NMEA 2000.
#[test]
fn a_j1939_json_stream_needs_protocol_j1939() {
    let json = run(
        &["convert", "--to", "json", "--protocol", "j1939"],
        J1939_PLAIN,
    );
    let plain = String::from_utf8(run(
        &[
            "convert",
            "--from",
            "json",
            "--to",
            "plain",
            "--protocol",
            "j1939",
        ],
        &json,
    ))
    .unwrap();
    assert!(plain.ends_with(J1939_BODY), "round trip: {plain}");

    let out = Command::new(canboat())
        .args(["convert", "--from", "json", "--to", "plain"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .and_then(|mut child| {
            child.stdin.take().unwrap().write_all(&json)?;
            child.wait_with_output()
        })
        .expect("run canboat");
    assert!(!out.status.success());
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(stderr.contains("--protocol j1939"), "{stderr}");
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

/// Drop every `"timestamp":"…",` from JSON lines: a BST-D0 message
/// carries only the device's own clock, so its frames have none.
fn without_timestamps(json: &[u8]) -> String {
    let text = String::from_utf8_lossy(json);
    let mut out = String::new();
    let mut rest = text.as_ref();
    while let Some(i) = rest.find("\"timestamp\":\"") {
        out.push_str(&rest[..i]);
        let after = &rest[i + "\"timestamp\":\"".len()..];
        let end = after
            .find("\",")
            .expect("timestamp is followed by another field");
        rest = &after[end + 2..];
    }
    out.push_str(rest);
    out
}

/// Actisense BST-D0 messages (#993), as a W2K-1 sends them in its
/// Actisense mode. The fixture is pgn-test.in's 31 messages, 23 of them
/// fast-packets, written from the Actisense SDK's description of BST-D0
/// and BDTP (13 of its DLEs are doubled). They decode exactly as
/// pgn-test.in does.
#[test]
fn bst_d0_decodes_as_pgn_test() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../analyzer/tests");
    let plain = std::fs::read(dir.join("pgn-test.in")).expect("read pgn-test.in");
    let bst = std::fs::read(dir.join("pgn-test-bst-d0.bin")).expect("read fixture");
    let want = without_timestamps(&run(&["convert", "--no-banner"], &plain));
    let got = without_timestamps(&run(&["convert", "--no-banner", "--from", "bst-d0"], &bst));
    assert_eq!(got.lines().count(), 31);
    assert_eq!(got, want);
}

/// Actisense BST-95 raw CAN frames (#1011), as a PRO-NDC-1E2K or W2K-1
/// sends them in its "CAN Actisense" mode. The fixture is pgn-test.in's 31
/// messages as 115 CAN frames: the 26 fast-packets split into their frames,
/// written from the Actisense SDK's description of BST-95 and BDTP (18 of
/// its DLEs are doubled). Joined again, they decode exactly as pgn-test.in
/// does.
#[test]
fn bst_95_decodes_as_pgn_test() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../analyzer/tests");
    let plain = std::fs::read(dir.join("pgn-test.in")).expect("read pgn-test.in");
    let bst = std::fs::read(dir.join("pgn-test-bst-95.bin")).expect("read fixture");
    let want = without_timestamps(&run(&["convert", "--no-banner"], &plain));
    let got = without_timestamps(&run(&["convert", "--no-banner", "--from", "bst-95"], &bst));
    assert_eq!(got.lines().count(), 31);
    assert_eq!(got, want);
}

/// candump in the style of the Angstrom distribution's can-utils (#998),
/// against the same golden file candump2analyzer is tested with. The
/// lines carry no time, so both compare everything after the timestamp
/// column. The capture starts with candump's `interface = …` banner,
/// which must not make format detection pick PLAIN.
#[test]
fn candump_angstrom_matches_candump2analyzer() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../candump2analyzer/tests");
    let input = std::fs::read(dir.join("angstrom.in")).expect("read angstrom.in");
    let want = std::fs::read_to_string(dir.join("angstrom.out")).expect("read angstrom.out");
    let out = run(&["convert", "--no-banner", "--to", "plain"], &input);
    let got: String = String::from_utf8(out)
        .expect("utf-8")
        .lines()
        .map(|l| format!("{}\n", l.split_once(',').map_or(l, |(_, rest)| rest)))
        .collect();
    assert_eq!(got.lines().count(), 51);
    assert_eq!(got, want.replace("\r\n", "\n"));
}

/// One BDTP-framed BST-95 message from the bus: `data` (at most 8 bytes)
/// with the identifier of `prio`, `pgn` (PDU2), `src`.
fn bst95(prio: u8, pgn: u32, src: u8, data: &[u8]) -> Vec<u8> {
    let mut m = vec![
        0x95,
        6 + data.len() as u8,
        0,
        0,
        src,
        pgn as u8,
        (pgn >> 8) as u8,
        (prio << 2) | (pgn >> 16) as u8,
    ];
    m.extend_from_slice(data);
    let sum = m.iter().fold(0u8, |s, &b| s.wrapping_add(b));
    m.push(0u8.wrapping_sub(sum));
    let mut out = vec![0x10, 0x02];
    for b in m {
        out.push(b);
        if b == 0x10 {
            out.push(0x10);
        }
    }
    out.extend_from_slice(&[0x10, 0x03]);
    out
}

/// PGN 130816 is a fast-packet on NMEA 2000 but a single frame on J1939, so
/// a BST-95 capture read with `--protocol j1939` gives the frame as it is,
/// where NMEA 2000 waits for the rest of a fast-packet that never comes.
#[test]
fn bst_95_follows_the_protocol() {
    // Read as a fast-packet, its first two bytes say: sequence 1, frame 0,
    // a message of 20 bytes.
    let frame = bst95(6, 130816, 0x2a, &[0x20, 0x14, 2, 3, 4, 5, 6, 7]);
    let j1939 = run(
        &[
            "convert",
            "--from",
            "bst-95",
            "--protocol",
            "j1939",
            "--to",
            "plain",
        ],
        &frame,
    );
    let j1939 = String::from_utf8(j1939).unwrap();
    assert!(
        j1939.contains(",6,130816,42,255,8,20,14,02,03,04,05,06,07"),
        "{j1939}"
    );
    let n2k = run(&["convert", "--from", "bst-95", "--to", "plain"], &frame);
    assert!(
        !String::from_utf8(n2k).unwrap().contains("130816"),
        "on NMEA 2000 it is the first frame of a fast-packet"
    );
}
