// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The shared `--quirk` flag for the binaries that decode a stream:
//! `canboat convert`, `canboat interface` and the `analyzer` shim.
//!
//! These are the *decode-time* quirks from [`canboat_core::quirk`] —
//! corrections applied to a decoded value. `canboat server` has its own
//! `--quirk` covering the same names plus the bus-side quirks that emit
//! frames (`scx20`, `wmm`, `motion`).
//!
//! Every quirk is off by default: each one will happily "correct" data
//! that was never wrong, which is exactly what happens when you replay
//! an old capture.

use canboat_core::quirk;

/// Decode-time quirks a stream-reading binary can switch on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum DecodeQuirk {
    /// Correct dates from a GPS receiver that never learned about the
    /// 1024-week rollover and reports one or two epochs in the past.
    GpsRollover,
}

impl DecodeQuirk {
    /// Switch this quirk on in `canboat-core`.
    pub fn enable(self) {
        match self {
            DecodeQuirk::GpsRollover => quirk::enable_gps_rollover(),
        }
    }
}

/// The `--quirk` flag itself, `#[command(flatten)]`ed into a parser.
#[derive(Debug, Clone, Default, clap::Args)]
pub struct QuirkArgs {
    /// Enable a decode-time device quirk (repeatable). `gps-rollover`
    /// corrects GNSS dates from a receiver that never handled the GPS
    /// week rollover. Off by default — do not use it when replaying a
    /// capture made before April 2019.
    #[arg(long = "quirk", value_enum, value_name = "QUIRK")]
    pub quirk: Vec<DecodeQuirk>,
}

impl QuirkArgs {
    /// Apply every requested quirk. Call once, before decoding starts.
    pub fn apply(&self) {
        for q in &self.quirk {
            q.enable();
        }
    }
}
