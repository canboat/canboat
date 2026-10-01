// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `--bus`: what the CAN bus carries, shared by every command that reads
//! or drives one, so they can't drift apart.
//!
//! The option repeats because a bus can carry more than one protocol — a
//! Quick bus spliced into an NMEA 2000 backbone, say — and decoding such
//! a bus will one day take `--bus nmea2000 --bus quick`. Today one
//! protocol is all a command can handle, so a second, different one is
//! refused rather than half-honoured.

use anyhow::{Result, bail};
use clap::builder::{PossibleValuesParser, TypedValueParser};

use crate::engine::BusProtocol;

#[derive(Debug, Clone, Default, clap::Args)]
pub struct BusArgs {
    /// What the CAN bus carries: `nmea2000` (default) or `j1939`. Picks
    /// the PGN table and the framing — J1939 has no fast-packet, only
    /// single frames and ISO TP.
    #[arg(
        long = "bus",
        value_name = "PROTOCOL",
        value_parser = PossibleValuesParser::new(["nmea2000", "j1939"])
            .map(|s| s.parse::<BusProtocol>().expect("listed in PossibleValuesParser")),
    )]
    bus: Vec<BusProtocol>,

    /// Same as `--bus j1939` (the name this option had before `--bus`).
    #[arg(long, hide = true)]
    j1939: bool,
}

impl BusArgs {
    /// The one protocol this command runs on.
    pub fn protocol(&self) -> Result<BusProtocol> {
        let mut chosen = self.bus.clone();
        if self.j1939 {
            chosen.push(BusProtocol::J1939);
        }
        chosen.dedup();
        match chosen.as_slice() {
            [] => Ok(BusProtocol::Nmea2000),
            [one] => Ok(*one),
            many => {
                let names: Vec<_> = many.iter().map(|b| b.as_str()).collect();
                bail!(
                    "--bus {}: combining protocols on one bus is not supported yet",
                    names.join(" --bus ")
                )
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[derive(Debug, Parser)]
    struct Cli {
        #[command(flatten)]
        bus: BusArgs,
    }

    fn protocol(args: &[&str]) -> Result<BusProtocol> {
        let mut argv = vec!["test"];
        argv.extend_from_slice(args);
        Cli::try_parse_from(argv)?.bus.protocol()
    }

    #[test]
    fn defaults_to_nmea2000() {
        assert_eq!(protocol(&[]).unwrap(), BusProtocol::Nmea2000);
    }

    #[test]
    fn picks_the_named_bus() {
        assert_eq!(protocol(&["--bus", "j1939"]).unwrap(), BusProtocol::J1939);
        assert_eq!(
            protocol(&["--bus", "nmea2000"]).unwrap(),
            BusProtocol::Nmea2000
        );
    }

    #[test]
    fn j1939_flag_is_an_alias() {
        assert_eq!(protocol(&["--j1939"]).unwrap(), BusProtocol::J1939);
        assert_eq!(
            protocol(&["--j1939", "--bus", "j1939"]).unwrap(),
            BusProtocol::J1939
        );
    }

    #[test]
    fn refuses_two_protocols() {
        let err = protocol(&["--bus", "nmea2000", "--bus", "j1939"]).unwrap_err();
        assert!(err.to_string().contains("not supported yet"), "{err}");
        assert!(protocol(&["--j1939", "--bus", "nmea2000"]).is_err());
    }

    #[test]
    fn refuses_unknown_names() {
        assert!(protocol(&["--bus", "quick"]).is_err());
    }
}
