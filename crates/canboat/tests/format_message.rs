// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Integration smoke for `canboat format-message`, driving the real
//! binary. Locks the canboat C parity payload and the SI/Metric units
//! contract end to end.

use std::process::Command;

fn canboat() -> &'static str {
    env!("CARGO_BIN_EXE_canboat")
}

/// Run `canboat format-message <args>`; return stdout, panic on failure.
fn fm(args: &[&str]) -> String {
    let out = Command::new(canboat())
        .arg("format-message")
        .args(args)
        .output()
        .expect("spawn canboat");
    assert!(
        out.status.success(),
        "format-message {args:?} failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8(out.stdout).expect("utf8 stdout")
}

#[test]
fn iso_request_matches_canboat_c_payload() {
    // canboat C `format-message`: PGN=126996 request → payload 14 f0 01
    // (little-endian of 126996 = 0x01_F014).
    let line = fm(&["isoRequest", "--dst", "0", "PGN=126996"]);
    assert!(
        line.trim_end().ends_with("59904,0,0,3,14,f0,01"),
        "got {line:?}"
    );
}

#[test]
fn si_and_metric_encode_identical_bytes() {
    // 93.8734° (Metric) and 1.6384 rad (SI) are the same physical angle,
    // so they must pack to identical wire bytes.
    let metric = fm(&[
        "windData",
        "Wind Speed=0",
        "Wind Angle=93.8734",
        "Reference=Apparent",
    ]);
    let si = fm(&[
        "--si",
        "windData",
        "Wind Speed=0",
        "Wind Angle=1.6384",
        "Reference=Apparent",
    ]);
    assert_eq!(metric, si, "SI and Metric produced different frames");
    // Sanity: the wind-angle raw is 0x4000 = 16384 → bytes ...,40,...
    assert!(metric.contains(",40,"), "got {metric:?}");
}

#[test]
fn per_pgn_help_lists_fields_with_units() {
    let help = fm(&["windData", "--help"]);
    assert!(help.contains("Wind Angle"), "help missing field: {help}");
    assert!(help.contains("[deg]"), "help missing metric unit: {help}");
    assert!(
        help.contains("WIND_REFERENCE"),
        "help missing lookup: {help}"
    );
}

#[test]
fn si_help_reports_si_unit() {
    let help = fm(&["--si", "windData", "--help"]);
    assert!(help.contains("[rad]"), "SI help should show rad: {help}");
}

#[test]
fn list_includes_known_pgn() {
    let list = fm(&["--list"]);
    assert!(list.contains("isoRequest"), "list missing isoRequest");
    assert!(list.contains("windData"), "list missing windData");
}
