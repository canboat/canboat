// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `--manufacturer-allow`, `--manufacturer-deny` and `--list-manufacturers`,
//! shared by every command that reads a bus or a capture (canboat#1177).
//! See [`crate::engine::manufacturer_filter`] for what the filter does.

use std::collections::BTreeSet;

use anyhow::{Result, anyhow, bail};

use crate::engine::BusProtocol;
use crate::engine::manufacturer_filter::{ManufacturerList, manufacturer_table};

/// The help heading of these options.
const HEADING: &str = "Filter by manufacturer";

#[derive(Debug, Clone, Default, clap::Args)]
pub struct ManufacturerArgs {
    /// Only keep devices of these manufacturers: a name or a code,
    /// repeated or comma-separated.
    ///
    /// A message goes by the device that sent it, so every PGN from that
    /// source address goes the same way. Nothing from an address passes
    /// until its manufacturer is known, from its ISO Address Claim or,
    /// on NMEA 2000, provisionally from its proprietary PGNs. A name
    /// ignores case, spaces and punctuation, and a unique prefix is enough.
    #[arg(
        long = "manufacturer-allow",
        value_name = "NAME|CODE",
        conflicts_with = "deny",
        help_heading = HEADING
    )]
    allow: Vec<String>,

    /// Drop devices of these manufacturers: a name or a code, repeated or
    /// comma-separated. See `--manufacturer-allow`.
    #[arg(
        long = "manufacturer-deny",
        value_name = "NAME|CODE",
        help_heading = HEADING
    )]
    deny: Vec<String>,

    /// List the manufacturers and their codes, and exit.
    #[arg(long = "list-manufacturers", id = "list", help_heading = HEADING)]
    pub list: bool,
}

impl ManufacturerArgs {
    /// The list asked for, or `None` without one. The names are looked up in
    /// what `protocol` uses, and one that can't be resolved stops the command
    /// before it reads any data.
    pub fn resolve(&self, protocol: BusProtocol) -> Result<Option<ManufacturerList>> {
        if self.allow.is_empty() && self.deny.is_empty() {
            return Ok(None);
        }
        let codes = |values: &[String]| -> Result<BTreeSet<u16>> {
            let names = table(protocol)?;
            let mut codes = BTreeSet::new();
            for value in values {
                // Many names hold a comma ("Tohatsu Co, JP"): a value that is
                // one whole name is not a list.
                let whole = normalize(value);
                if names.iter().any(|(_, name)| normalize(name) == whole) {
                    codes.insert(resolve_one(value, protocol)?);
                    continue;
                }
                for part in value.split(',') {
                    codes.insert(resolve_one(part, protocol)?);
                }
            }
            Ok(codes)
        };
        Ok(Some(if self.allow.is_empty() {
            ManufacturerList::Deny(codes(&self.deny)?)
        } else {
            ManufacturerList::Allow(codes(&self.allow)?)
        }))
    }

    /// Print the list when asked for, and say whether it was.
    pub fn print_list(&self, protocol: BusProtocol) -> Result<bool> {
        if !self.list {
            return Ok(false);
        }
        let mut rows = table(protocol)?;
        rows.sort_by_key(|(_, name)| *name);
        for (code, name) in rows {
            println!("{code:>5}  {name}");
        }
        Ok(true)
    }
}

/// The manufacturers of `protocol`, as `(code, name)`.
fn table(protocol: BusProtocol) -> Result<Vec<(u16, &'static str)>> {
    let table = manufacturer_table(protocol)
        .ok_or_else(|| anyhow!("a {protocol} bus has no manufacturers to filter by"))?;
    Ok(table
        .values
        .iter()
        .map(|v| (v.value as u16, v.name))
        .collect())
}

/// Lower case letters and digits only, so that `b&g` is `B & G`.
fn normalize(name: &str) -> String {
    name.chars()
        .filter(char::is_ascii_alphanumeric)
        .map(|c| c.to_ascii_lowercase())
        .collect()
}

/// Turn one `--manufacturer-*` value into its code.
fn resolve_one(value: &str, protocol: BusProtocol) -> Result<u16> {
    let value = value.trim();
    let names = table(protocol)?;
    if let Ok(code) = value.parse::<u32>() {
        if code > 0x7ff {
            bail!("manufacturer code {code} is more than the 2047 that fits in 11 bits");
        }
        return Ok(code as u16);
    }
    let wanted = normalize(value);
    let matching = |f: &dyn Fn(&str) -> bool| -> Vec<(u16, &str)> {
        names
            .iter()
            .filter(|(_, name)| f(&normalize(name)))
            .copied()
            .collect()
    };
    // An exact match wins; otherwise a unique prefix is enough.
    let mut found = matching(&|name| name == wanted);
    if found.is_empty() && !wanted.is_empty() {
        found = matching(&|name| name.starts_with(&wanted));
    }
    match found.as_slice() {
        [(code, _)] => Ok(*code),
        [] => match closest(&wanted, &names) {
            Some((code, name)) => {
                bail!("unknown manufacturer '{value}'; did you mean {name} ({code})?")
            }
            None => bail!("unknown manufacturer '{value}'"),
        },
        several => {
            let mut several = several.to_vec();
            several.sort_by_key(|(_, name)| *name);
            let list: Vec<_> = several
                .iter()
                .map(|(code, name)| format!("{name} ({code})"))
                .collect();
            bail!("'{value}' matches {}: be more specific", list.join(", "))
        }
    }
}

/// The manufacturer with the name nearest to `wanted`, if any is near.
fn closest(wanted: &str, names: &[(u16, &'static str)]) -> Option<(u16, &'static str)> {
    if wanted.is_empty() {
        return None;
    }
    // A typo in the first word: compare with a prefix of the same length.
    let distance = |name: &str| {
        let name = normalize(name);
        let cut = name.len().min(wanted.len() + 1);
        edit_distance(wanted, &name[..cut])
    };
    names
        .iter()
        .map(|&(code, name)| (distance(name), name, code))
        .min()
        .filter(|(d, ..)| *d <= 1 + wanted.len() / 3)
        .map(|(_, name, code)| (code, name))
}

/// The Levenshtein distance between two ASCII strings.
fn edit_distance(a: &str, b: &str) -> usize {
    let (a, b) = (a.as_bytes(), b.as_bytes());
    let mut row: Vec<usize> = (0..=b.len()).collect();
    for (i, ca) in a.iter().enumerate() {
        let mut diagonal = row[0];
        row[0] = i + 1;
        for (j, cb) in b.iter().enumerate() {
            let cost = diagonal + usize::from(ca != cb);
            diagonal = row[j + 1];
            row[j + 1] = cost.min(row[j] + 1).min(row[j + 1] + 1);
        }
    }
    row[b.len()]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one(value: &str) -> Result<u16> {
        resolve_one(value, BusProtocol::Nmea2000)
    }

    #[test]
    fn a_name_ignores_case_spaces_and_punctuation() {
        assert_eq!(one("358").unwrap(), 358);
        assert_eq!(one("Victron Energy").unwrap(), 358);
        assert_eq!(one("victron").unwrap(), 358);
        assert_eq!(one("VICTRON-ENERGY").unwrap(), 358);
        assert_eq!(one("b&g").unwrap(), 381);
        assert_eq!(one("B & G").unwrap(), 381);
    }

    #[test]
    fn a_name_that_cant_be_resolved_says_why() {
        let typo = one("vicron").unwrap_err().to_string();
        assert_eq!(
            typo,
            "unknown manufacturer 'vicron'; did you mean Victron Energy (358)?"
        );
        let several = one("sim").unwrap_err().to_string();
        assert!(several.starts_with("'sim' matches "), "{several}");
        assert!(several.contains("Simrad (1857)"), "{several}");
        assert!(several.ends_with(": be more specific"), "{several}");
        assert!(
            one("qqqqqqqqqqqq")
                .unwrap_err()
                .to_string()
                .ends_with("'qqqqqqqqqqqq'")
        );
        assert!(one("2048").is_err());
    }

    #[test]
    fn j1939_has_its_own_names_and_quick_has_none() {
        assert_eq!(
            resolve_one("motorenfabrik", BusProtocol::J1939).unwrap(),
            1351
        );
        assert!(one("motorenfabrik").is_err());
        assert_eq!(one("cummins").unwrap(), 440);
        assert!(resolve_one("cummins", BusProtocol::J1939).is_err());
        assert!(resolve_one("victron", BusProtocol::Quick).is_err());
    }

    /// The arguments as clap hands them over.
    fn resolved(allow: &[&str]) -> Result<Option<ManufacturerList>> {
        ManufacturerArgs {
            allow: allow.iter().map(ToString::to_string).collect(),
            ..Default::default()
        }
        .resolve(BusProtocol::Nmea2000)
    }

    #[test]
    fn values_repeat_and_split_on_commas_but_a_whole_name_stays_whole() {
        let want = |codes: &[u16]| Some(ManufacturerList::Allow(codes.iter().copied().collect()));
        assert_eq!(resolved(&["victron,b&g"]).unwrap(), want(&[358, 381]));
        assert_eq!(resolved(&["victron", "381"]).unwrap(), want(&[358, 381]));
        assert_eq!(resolved(&["Tohatsu Co, JP"]).unwrap(), want(&[431]));
        assert!(resolved(&["victron,vicron"]).is_err());
    }
}
