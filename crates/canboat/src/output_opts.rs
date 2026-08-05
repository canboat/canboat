// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Shared decoded-output shape flags for `canboat convert` and the
//! `analyzer` shim.
//!
//! Both CLIs decide three orthogonal things about a decoded record: how
//! field keys and PGN descriptions are spelled (`--id`), which unit
//! system numeric fields are decoded into (`--units`), and whether the
//! record is nested under its PGN id (`--wrap`). They are declared once
//! here and `#[command(flatten)]`ed into both parsers so the two can't
//! drift.
//!
//! **The defaults are camelCase identifiers, strict SI units and no
//! wrapper** — `--id camel --units si`. That is exactly the shape the
//! JSON consumers already want: canboatjs' `FromPgn` defaults to
//! `useCamel: true`, has no unit conversion at all (so Signal K has
//! always read SI), and emits each record flat.
//!
//! canboat C's spellings — `--camel`, `--upper-camel`, `--si` — are kept
//! as deprecated aliases so existing command lines keep producing their
//! exact output; `--camel` / `--upper-camel` therefore still imply
//! `--wrap`, which in canboat C rides along with camelCase. Pre-change
//! behaviour is `--id spaces --units metric`.

use canboat_core::Units;
use canboat_core::output::CamelCase;

/// `--id` values: how field keys and the PGN description are spelled.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, clap::ValueEnum)]
pub enum IdStyle {
    /// Human-readable names — `"Unique Number"`, `"ISO Address Claim"`.
    /// What canboat C prints without `-camel`.
    Spaces,
    /// lowerCamelCase identifiers — `"uniqueNumber"`. The default.
    #[default]
    Camel,
    /// UpperCamelCase identifiers — `"UniqueNumber"`.
    #[value(name = "uppercamel", alias = "upper-camel")]
    UpperCamel,
}

impl From<IdStyle> for CamelCase {
    fn from(s: IdStyle) -> Self {
        match s {
            IdStyle::Spaces => CamelCase::Off,
            IdStyle::Camel => CamelCase::Lower,
            IdStyle::UpperCamel => CamelCase::Upper,
        }
    }
}

/// `--units` values: which unit system numeric fields decode into.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, clap::ValueEnum)]
pub enum UnitSystem {
    /// Strict SI base units — `rad`, `K`, `Pa`, `C`. The default.
    #[default]
    Si,
    /// canboat's practical humanized units — `deg`, `°C`, `bar`, `Ah`.
    /// What canboat C prints without `-si`.
    Metric,
}

impl From<UnitSystem> for Units {
    fn from(u: UnitSystem) -> Self {
        match u {
            UnitSystem::Si => Units::Si,
            UnitSystem::Metric => Units::Metric,
        }
    }
}

/// The `--id` / `--units` pair plus the deprecated flags they replace.
/// Flattened into both `convert` and the `analyzer` shim.
#[derive(Debug, Clone, Default, clap::Args)]
pub struct ShapeArgs {
    /// Spelling of field keys and PGN descriptions: `spaces`
    /// ("Unique Number"), `camel` ("uniqueNumber", the default) or
    /// `uppercamel` ("UniqueNumber").
    #[arg(long, value_enum, value_name = "STYLE",
          conflicts_with_all = ["camel", "upper_camel"])]
    id: Option<IdStyle>,

    /// Unit system for numeric fields: `si` (rad/K/Pa, the default) or
    /// `metric` (canboat's humanized deg/°C/bar).
    #[arg(long, value_enum, value_name = "SYSTEM", conflicts_with = "si")]
    units: Option<UnitSystem>,

    /// JSON: nest each record in `{"<pgnId>":{…}}` instead of emitting
    /// it flat. Off by default; implied by the deprecated `--camel` /
    /// `--upper-camel`, which is how canboat C spells it.
    #[arg(long)]
    wrap: bool,

    /// Deprecated: camelCase is now the default. Same as
    /// `--id camel --wrap`.
    #[arg(long, conflicts_with = "upper_camel")]
    camel: bool,

    /// Deprecated: use `--id uppercamel --wrap`.
    #[arg(long = "upper-camel")]
    upper_camel: bool,

    /// Deprecated: SI is now the default. Same as `--units si`.
    #[arg(long)]
    si: bool,
}

impl ShapeArgs {
    /// The resolved identifier style. `--id` wins; the deprecated
    /// `--camel` / `--upper-camel` are honoured when it is absent.
    pub fn id_style(&self) -> IdStyle {
        match self.id {
            Some(id) => id,
            None if self.upper_camel => IdStyle::UpperCamel,
            None if self.camel => IdStyle::Camel,
            None => IdStyle::default(),
        }
    }

    /// The resolved identifier style, as the output layer's enum.
    pub fn camel_case(&self) -> CamelCase {
        self.id_style().into()
    }

    /// The resolved unit system. `--units` wins; the deprecated `--si`
    /// is honoured when it is absent.
    pub fn unit_system(&self) -> UnitSystem {
        match self.units {
            Some(u) => u,
            None if self.si => UnitSystem::Si,
            None => UnitSystem::default(),
        }
    }

    /// The resolved unit system, as the database's enum.
    pub fn units(&self) -> Units {
        self.unit_system().into()
    }

    /// Whether each JSON record is nested under its PGN key. Explicit
    /// `--wrap`, or the deprecated flags that imply it — so a command
    /// line written for canboat C's `-camel` keeps its exact output.
    pub fn wrap(&self) -> bool {
        self.wrap || self.camel || self.upper_camel
    }

    /// True when decoding into strict SI — what the analyzer JSON
    /// banner reports as `"units":"si"`.
    pub fn is_si(&self) -> bool {
        self.unit_system() == UnitSystem::Si
    }

    /// Nudge users off the canboat C spellings, on stderr so decoded
    /// stdout stays clean. Call once, before the decode loop.
    pub fn warn_deprecated(&self) {
        if self.camel {
            eprintln!(
                "warning: --camel is deprecated; camelCase is now the default, so use --wrap \
                 if you still want records nested under their PGN id"
            );
        }
        if self.upper_camel {
            eprintln!("warning: --upper-camel is deprecated; use --id uppercamel --wrap");
        }
        if self.si {
            eprintln!(
                "warning: --si is deprecated and is now the default; drop it, or say --units si"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[derive(Debug, Parser)]
    struct Harness {
        #[command(flatten)]
        shape: ShapeArgs,
    }

    fn shape(args: &[&str]) -> ShapeArgs {
        let mut argv = vec!["test"];
        argv.extend_from_slice(args);
        Harness::parse_from(argv).shape
    }

    #[test]
    fn defaults_are_camel_si_and_unwrapped() {
        let s = shape(&[]);
        assert_eq!(s.camel_case(), CamelCase::Lower);
        assert_eq!(s.units(), Units::Si);
        assert!(!s.wrap());
    }

    #[test]
    fn legacy_defaults_are_reachable() {
        let s = shape(&["--id", "spaces", "--units", "metric"]);
        assert_eq!(s.camel_case(), CamelCase::Off);
        assert_eq!(s.units(), Units::Metric);
        assert!(!s.wrap());
    }

    #[test]
    fn deprecated_flags_still_resolve() {
        assert_eq!(shape(&["--camel"]).camel_case(), CamelCase::Lower);
        assert_eq!(shape(&["--upper-camel"]).camel_case(), CamelCase::Upper);
        assert_eq!(shape(&["--si"]).units(), Units::Si);
    }

    /// canboat C ties wrapping to `-camel`, so the deprecated spellings
    /// must keep producing the wrapped record their users expect —
    /// while `--id camel` on its own does not.
    #[test]
    fn only_the_deprecated_camel_flags_imply_wrap() {
        assert!(shape(&["--camel"]).wrap());
        assert!(shape(&["--upper-camel"]).wrap());
        assert!(!shape(&["--id", "camel"]).wrap());
        assert!(!shape(&["--id", "uppercamel"]).wrap());
        assert!(shape(&["--id", "uppercamel", "--wrap"]).wrap());
    }

    #[test]
    fn uppercamel_accepts_both_spellings() {
        assert_eq!(
            shape(&["--id", "uppercamel"]).camel_case(),
            CamelCase::Upper
        );
        assert_eq!(
            shape(&["--id", "upper-camel"]).camel_case(),
            CamelCase::Upper
        );
    }

    #[test]
    fn new_and_deprecated_forms_conflict() {
        for argv in [
            vec!["test", "--id", "camel", "--camel"],
            vec!["test", "--id", "spaces", "--upper-camel"],
            vec!["test", "--units", "metric", "--si"],
        ] {
            assert!(
                Harness::try_parse_from(&argv).is_err(),
                "expected {argv:?} to be rejected"
            );
        }
    }
}
