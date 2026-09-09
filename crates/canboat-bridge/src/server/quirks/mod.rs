// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Device-specific firmware-quirk workarounds.
//!
//! Each variant of [`QuirkKind`] is a known piece of misbehaviour we
//! actively cover for. [`Quirks::process_decoded`] is handed every
//! *decoded* bus PGN from the pipeline's post-decode tap and returns one
//! or more synthetic `RawFrame`s for the pipeline to (1) write to the bus
//! on our (or the impersonated device's) behalf, and (2) feed back
//! through its own processing path so the local analyzer / snapshot /
//! JSON streams reflect the synthetic frame too.
//!
//! Working from the *decoded* PGN (rather than a raw pre-reassembly
//! frame) means quirks read typed, named fields — [`DecodedPgn::iso_name`],
//! [`DecodedPgn::field_ref`] — instead of hand-unpacking bytes, and can
//! act on fast-packet PGNs too.
//!
//! Two quirks are recognised:
//! - `Scx20` — impersonate a Furuno SCX-20's Product Information (see
//!   [[device-scx20]] / `samples/scx20-setting-tool-*.raw`). Gated to
//!   `--socketcan` at the CLI layer because every other device backend
//!   rewrites the outbound source address, defeating the impersonation.
//!   Lives in [`scx20`].
//! - `Wmm` — compute magnetic variation locally (WMM 2025) and emit our
//!   own PGN 127258, plus ask older-WMM sources to stop. It emits
//!   canboat's *own* node (no impersonation), so it works with any
//!   writable backend. Lives in [`wmm`].
//!
//! A third kind is not a frame emitter at all: `GpsRollover` corrects a
//! decoded date in place, and lives in [`canboat_core::quirk`] so that
//! a plain `canboat convert` over a capture gets it too. [`Quirks::new`]
//! just switches it on there; [`Quirks::process_decoded`] never sees
//! it. Its bus-side companion `GpsRelay` re-sends a listed device's
//! broadcasts from canboat's own address with the dates corrected, so
//! other devices can pick canboat as their source. Lives in
//! [`gps_relay`].

pub mod gps_relay;
pub mod motion;
pub mod scx20;
pub mod wmm;

use std::str::FromStr;
use std::time::Instant;

use canboat_core::quirk::Target;
use canboat_core::{DecodedPgn, RawFrame};

/// One `--quirk` value. Parsed from its name — `scx20`, `wmm`, `motion`,
/// `gps-rollover` — plus, for `gps-rollover`, an optional `=<devices>`
/// argument; see [`Target::parse`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum QuirkKind {
    /// Furuno SCX-20: fabricate its PGN 126996 Product Information when
    /// the bus asks (its firmware sometimes "forgets" to answer, breaking
    /// Setting-Tool discovery). Impersonation — needs --socketcan.
    Scx20,
    /// Compute magnetic variation locally with WMM 2025 and emit our own
    /// PGN 127258 (~1 Hz), asking older-WMM sources to stop. Needs any
    /// writable backend.
    Wmm,
    /// Impersonate a B&G H5000 Motion Sensor when a Furuno SCX-20 is
    /// present, so a Navico Hercules accepts it. Needs to claim a new
    /// device, so needs socketcan.
    Motion,
    /// Correct GNSS dates from a receiver that never learned about the
    /// GPS 1024-week rollover and reports one or two epochs in the
    /// past — and every date the listed devices stamp from that clock.
    /// Rewrites the decoded value (see [`canboat_core::quirk`]);
    /// nothing is written to the bus, and no particular backend is
    /// needed.
    GpsRollover(Target),
    /// Re-send every broadcast PGN from the devices listed in
    /// `gps-rollover=…` from canboat's own address, dates corrected, so
    /// other devices on the bus can select canboat as their GPS / time
    /// source instead of the rolled-over one. Needs a writable backend
    /// and a `gps-rollover` device list.
    GpsRelay,
}

impl QuirkKind {
    /// The name this quirk is switched on by, without any `=argument`.
    pub fn name(&self) -> &'static str {
        match self {
            QuirkKind::Scx20 => "scx20",
            QuirkKind::Wmm => "wmm",
            QuirkKind::Motion => "motion",
            QuirkKind::GpsRollover(_) => "gps-rollover",
            QuirkKind::GpsRelay => "gps-relay",
        }
    }

    /// Does this list of quirks give `gps-relay` something to relay —
    /// a `gps-rollover` with named devices? `all` is refused: relaying
    /// every device on the bus as canboat's own is nobody's intent.
    pub fn gps_relay_has_devices(kinds: &[QuirkKind]) -> bool {
        kinds
            .iter()
            .any(|k| matches!(k, QuirkKind::GpsRollover(Target::Devices(d)) if !d.is_empty()))
    }
}

impl FromStr for QuirkKind {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (name, args) = match s.split_once('=') {
            Some((n, a)) => (n.trim(), Some(a)),
            None => (s.trim(), None),
        };
        let no_args = |kind: QuirkKind| match args {
            None => Ok(kind),
            Some(_) => Err(format!("--quirk {name} takes no argument")),
        };
        match name {
            "scx20" => no_args(QuirkKind::Scx20),
            "wmm" => no_args(QuirkKind::Wmm),
            "motion" => no_args(QuirkKind::Motion),
            "gps-rollover" => Target::parse(args).map(QuirkKind::GpsRollover),
            "gps-relay" => no_args(QuirkKind::GpsRelay),
            _ => Err(format!(
                "unknown quirk '{name}'; the quirks are: scx20, wmm, motion, gps-rollover, gps-relay"
            )),
        }
    }
}

/// Stateful side of the quirk machinery. Owned by the pipeline; lives
/// inside [`crate::server::pipeline::Hubs`] for the lifetime of `pipeline::run`.
pub struct Quirks {
    /// Present iff `--quirk scx20`. Owns the SCX-20 impersonation state.
    scx20: Option<scx20::Scx20>,
    /// Present iff `--quirk wmm`. Owns the WMM state machine.
    wmm: Option<wmm::WmmQuirk>,
    /// Present iff `--quirk motion`. Owns the Motion-Sensor impersonation.
    motion: Option<motion::Motion>,
    /// Present iff `--quirk gps-relay`. Re-sends a listed device's data.
    gps_relay: Option<gps_relay::GpsRelay>,
}

impl Quirks {
    pub fn new(kinds: Vec<QuirkKind>) -> Self {
        // Not a frame emitter: it flips a switch in the decoder and
        // has no state of its own here. The switch is process-wide --
        // `decode()` takes no options -- so set it both ways, or a
        // pipeline built without the quirk would inherit it from an
        // earlier one in the same process.
        let gps_rollover = kinds.iter().find_map(|k| match k {
            QuirkKind::GpsRollover(target) => Some(target.clone()),
            _ => None,
        });
        match gps_rollover {
            Some(target) => canboat_core::quirk::enable_gps_rollover(target),
            None => canboat_core::quirk::disable_gps_rollover(),
        }
        Self {
            scx20: kinds.contains(&QuirkKind::Scx20).then(scx20::Scx20::new),
            wmm: kinds.contains(&QuirkKind::Wmm).then(wmm::WmmQuirk::new),
            motion: kinds.contains(&QuirkKind::Motion).then(motion::Motion::new),
            gps_relay: kinds
                .contains(&QuirkKind::GpsRelay)
                .then(gps_relay::GpsRelay::new),
        }
    }

    /// `true` iff at least one quirk is enabled — lets the pipeline skip
    /// the per-frame call entirely on the common path.
    pub fn is_enabled(&self) -> bool {
        self.scx20.is_some()
            || self.wmm.is_some()
            || self.motion.is_some()
            || self.gps_relay.is_some()
    }

    /// Inspect one decoded bus PGN and produce zero or more synthetic
    /// responses. The pipeline pushes each returned `RawFrame` to the
    /// device writer (so it lands on the wire with the intended `src`)
    /// and back through its own processing path (so the local n2kd /
    /// snapshot / JSON / NMEA 0183 view reflects it too). `own_addr` is
    /// canboat's claimed address when it holds one, so a quirk can
    /// recognise its own emissions coming back round.
    pub fn process_decoded(&mut self, decoded: &DecodedPgn, own_addr: Option<u8>) -> Vec<RawFrame> {
        let now = Instant::now();
        let mut out = Vec::new();
        if let Some(scx20) = self.scx20.as_mut() {
            out.extend(scx20.process(decoded, now));
        }
        if let Some(wmm) = self.wmm.as_mut() {
            out.extend(wmm.process(decoded, now));
        }
        if let Some(motion) = self.motion.as_mut() {
            out.extend(motion.process(decoded, now));
        }
        if let Some(relay) = self.gps_relay.as_mut() {
            out.extend(relay.process(decoded, own_addr));
        }
        out
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;

    /// `Quirks::new` writes the process-wide GPS rollover switch, so the
    /// tests that call it — here and in `gps_relay` — must not run
    /// concurrently with each other.
    pub(super) static SWITCH: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn parses_every_name() {
        assert_eq!("gps-relay".parse::<QuirkKind>(), Ok(QuirkKind::GpsRelay));
        assert!("gps-relay=4".parse::<QuirkKind>().is_err());
        assert_eq!(
            "gps-rollover=4".parse::<QuirkKind>(),
            Ok(QuirkKind::GpsRollover(Target::Devices(vec![
                canboat_core::quirk::Device::Address(4)
            ])))
        );
        assert!("gps-rollover=lots".parse::<QuirkKind>().is_err());
        assert!("nope".parse::<QuirkKind>().is_err());
    }

    #[test]
    fn relay_needs_a_device_list() {
        let devices = "gps-rollover=4".parse::<QuirkKind>().unwrap();
        assert!(QuirkKind::gps_relay_has_devices(&[
            QuirkKind::GpsRelay,
            devices
        ]));
        for kinds in [
            vec![QuirkKind::GpsRelay],
            vec![QuirkKind::GpsRelay, QuirkKind::GpsRollover(Target::Gnss)],
            vec![QuirkKind::GpsRelay, QuirkKind::GpsRollover(Target::All)],
        ] {
            assert!(!QuirkKind::gps_relay_has_devices(&kinds), "{kinds:?}");
        }
    }

    #[test]
    fn enabled_reflects_configured_quirks() {
        let _guard = SWITCH.lock().unwrap_or_else(|e| e.into_inner());
        assert!(!Quirks::new(vec![]).is_enabled());
        assert!(Quirks::new(vec![QuirkKind::Scx20]).is_enabled());
        assert!(Quirks::new(vec![QuirkKind::Wmm]).is_enabled());
        let both = Quirks::new(vec![QuirkKind::Scx20, QuirkKind::Wmm]);
        assert!(both.is_enabled());
        assert!(both.scx20.is_some() && both.wmm.is_some());
    }

    /// The GPS rollover switch lives in the decoder, which is
    /// process-wide, so a second pipeline that did not ask for it must
    /// not inherit it from the first.
    #[test]
    fn gps_rollover_does_not_leak_into_the_next_pipeline() {
        let _guard = SWITCH.lock().unwrap_or_else(|e| e.into_inner());
        let _first = Quirks::new(vec![QuirkKind::GpsRollover(Target::Gnss)]);
        assert!(canboat_core::quirk::gps_rollover_reference_day().is_some());
        let _second = Quirks::new(vec![QuirkKind::Wmm]);
        assert!(canboat_core::quirk::gps_rollover_reference_day().is_none());
    }
}
