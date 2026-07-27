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

pub mod motion;
pub mod scx20;
pub mod wmm;

use std::time::Instant;

use canboat_core::{DecodedPgn, RawFrame};

/// Under the `cli` feature clap derives `ValueEnum` so the user types e.g.
/// `--quirk scx20`; the enum itself is always available for `BridgeConfig`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "cli", derive(clap::ValueEnum))]
#[cfg_attr(feature = "cli", clap(rename_all = "kebab-case"))]
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
}

impl Quirks {
    pub fn new(kinds: Vec<QuirkKind>) -> Self {
        Self {
            scx20: kinds.contains(&QuirkKind::Scx20).then(scx20::Scx20::new),
            wmm: kinds.contains(&QuirkKind::Wmm).then(wmm::WmmQuirk::new),
            motion: kinds.contains(&QuirkKind::Motion).then(motion::Motion::new),
        }
    }

    /// `true` iff at least one quirk is enabled — lets the pipeline skip
    /// the per-frame call entirely on the common path.
    pub fn is_enabled(&self) -> bool {
        self.scx20.is_some() || self.wmm.is_some() || self.motion.is_some()
    }

    /// Inspect one decoded bus PGN and produce zero or more synthetic
    /// responses. The pipeline pushes each returned `RawFrame` to the
    /// device writer (so it lands on the wire with the intended `src`)
    /// and back through its own processing path (so the local n2kd /
    /// snapshot / JSON / NMEA 0183 view reflects it too).
    pub fn process_decoded(&mut self, decoded: &DecodedPgn) -> Vec<RawFrame> {
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
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enabled_reflects_configured_quirks() {
        assert!(!Quirks::new(vec![]).is_enabled());
        assert!(Quirks::new(vec![QuirkKind::Scx20]).is_enabled());
        assert!(Quirks::new(vec![QuirkKind::Wmm]).is_enabled());
        let both = Quirks::new(vec![QuirkKind::Scx20, QuirkKind::Wmm]);
        assert!(both.is_enabled());
        assert!(both.scx20.is_some() && both.wmm.is_some());
    }
}
