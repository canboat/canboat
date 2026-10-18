// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The gateway help of `canboat interface` and `canboat server`, written
//! from [`Kind`]: which kinds there are, what each can carry, which take
//! which option. Nothing here says what a gateway can do; it only says
//! what `Kind` says.

use crate::engine::BusProtocol;
use crate::io::device::kind::{Kind, Setting};

/// `a`, `a or b`, `a, b or c`.
fn or_list(items: &[String]) -> String {
    match items {
        [] => String::new(),
        [one] => one.clone(),
        [init @ .., last] => format!("{} or {last}", init.join(", ")),
    }
}

/// The kinds that pass `filter`, as the help names them.
fn kinds_where(filter: impl Fn(Kind) -> bool) -> String {
    let names: Vec<String> = Kind::ALL
        .iter()
        .copied()
        .filter(|&k| filter(k))
        .map(|k| k.to_string())
        .collect();
    or_list(&names)
}

/// The kinds `--protocol protocol` works with: `socketcan, canalyst or
/// usb-can-a`.
pub fn kinds_carrying(protocol: BusProtocol) -> String {
    kinds_where(|k| k.capabilities().protocols.contains(&protocol))
}

/// The kinds that have `setting`.
pub fn kinds_taking(setting: Setting) -> String {
    kinds_where(|k| k.takes(setting))
}

/// What `--kind` lists for `kind`: the gateways it stands for, all of an
/// Actisense's data formats' together.
pub fn kind_help(kind: Kind) -> String {
    let family: Vec<Kind> = Kind::ALL
        .iter()
        .copied()
        .filter(|k| k.name() == kind.name())
        .collect();
    if family.len() > 1 {
        // "Actisense NGT-1", "Actisense W2K-1, PRO-NDC-1E2K", …: the
        // vendor once, and each model once.
        let mut vendor = "";
        let mut models: Vec<&str> = Vec::new();
        for k in &family {
            let line = k.describe();
            let (first, rest) = line.split_once(' ').unwrap_or(("", line));
            vendor = first;
            let rest = rest.split(" (").next().unwrap_or(rest);
            for model in rest.split(", ") {
                if !models.contains(&model) {
                    models.push(model);
                }
            }
        }
        return format!("{vendor} {}; see --data-format", models.join(", "));
    }
    let caps = kind.capabilities();
    if caps.node {
        format!("{}; canboat is the node", kind.describe())
    } else {
        kind.describe().to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_kinds_that_carry_a_protocol_are_listed() {
        assert_eq!(
            kinds_carrying(BusProtocol::Quick),
            "socketcan, canalyst or usb-can-a"
        );
        let j1939 = kinds_carrying(BusProtocol::J1939);
        for kind in ["actisense (mxpgn)", "yachtd", "miniplex", "socketcan"] {
            assert!(j1939.contains(kind), "{j1939}");
        }
        assert!(!j1939.contains("ikonvert"), "{j1939}");
    }

    #[test]
    fn an_actisense_lists_every_model() {
        assert_eq!(
            kind_help(Kind::NGT1),
            "Actisense NGT-1, W2K-1, PRO-NDC-1E2K, NGX, W2K-2; see --data-format"
        );
        assert!(kind_help(Kind::Canalyst).ends_with("canboat is the node"));
    }
}
