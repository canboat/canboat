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

use std::str::FromStr;

use canboat_core::quirk::{self, Target};

/// Decode-time quirks a stream-reading binary can switch on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecodeQuirk {
    /// Correct dates from a GPS receiver that never learned about the
    /// 1024-week rollover and reports one or two epochs in the past —
    /// and, for the listed devices, every date they stamp from it.
    GpsRollover(Target),
}

impl DecodeQuirk {
    /// The name a quirk is switched on by, without any `=argument`.
    pub fn name(&self) -> &'static str {
        match self {
            DecodeQuirk::GpsRollover(_) => "gps-rollover",
        }
    }

    /// Switch this quirk on in `canboat-core`.
    pub fn enable(&self) {
        match self {
            DecodeQuirk::GpsRollover(target) => quirk::enable_gps_rollover(target.clone()),
        }
    }
}

impl FromStr for DecodeQuirk {
    type Err = String;

    /// `gps-rollover`, or `gps-rollover=<devices>` — see
    /// [`Target::parse`] for the device syntax.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (name, args) = match s.split_once('=') {
            Some((n, a)) => (n.trim(), Some(a)),
            None => (s.trim(), None),
        };
        match name {
            "gps-rollover" => Target::parse(args).map(DecodeQuirk::GpsRollover),
            _ => Err(format!(
                "unknown quirk '{name}'; the decode-time quirks are: gps-rollover"
            )),
        }
    }
}

/// The `--quirk` flag itself, `#[command(flatten)]`ed into a parser.
#[derive(Debug, Clone, Default, clap::Args)]
pub struct QuirkArgs {
    /// Enable a decode-time device quirk (repeatable). Off by default.
    ///
    /// `gps-rollover` corrects GNSS dates (PGN 129029, 129033, and
    /// 126992 from a GPS source) from a receiver that never handled the
    /// GPS week rollover. `gps-rollover=<device>[,<device>...]` also
    /// corrects every date the listed devices stamp from that clock,
    /// such as a DSC radio's Date of Receipt; a device is a source
    /// address (`4`), its manufacturer code and unique number
    /// (`1851:491603`), or its ISO NAME in hex (`0x…`). `gps-rollover=all`
    /// corrects every date on the bus. Do not use any of these when
    /// replaying a capture made before April 2019.
    #[arg(long = "quirk", value_name = "QUIRK")]
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

#[cfg(test)]
mod tests {
    use super::*;
    use canboat_core::quirk::Device;

    #[test]
    fn parses_the_flag_forms() {
        assert_eq!(
            "gps-rollover".parse::<DecodeQuirk>(),
            Ok(DecodeQuirk::GpsRollover(Target::Gnss))
        );
        assert_eq!(
            "gps-rollover=all".parse::<DecodeQuirk>(),
            Ok(DecodeQuirk::GpsRollover(Target::All))
        );
        assert_eq!(
            "gps-rollover=4,1851:491603".parse::<DecodeQuirk>(),
            Ok(DecodeQuirk::GpsRollover(Target::Devices(vec![
                Device::Address(4),
                Device::Product {
                    manufacturer: 1851,
                    unique: 491603
                },
            ])))
        );
        assert!("gps-rollover=".parse::<DecodeQuirk>().is_err());
        assert!("gps-rollover=vhf".parse::<DecodeQuirk>().is_err());
        assert!("wmm".parse::<DecodeQuirk>().is_err());
        assert!("scx20=4".parse::<DecodeQuirk>().is_err());
    }
}
