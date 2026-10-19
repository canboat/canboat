// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The gateway help of `canboat interface` and `canboat server`, and the
//! tables in `doc/gateways.md`, written from [`Kind`]: which kinds there
//! are, how each is reached, what it can carry, which takes which option
//! and what it does with the PGN lists. Nothing here says what a gateway
//! can do; it only says what `Kind` says, so the help cannot drift from
//! the code that opens the gateways.

#[cfg(test)]
use std::fmt::Write as _;
use std::sync::LazyLock;

use crate::engine::BusProtocol;
use crate::io::device::kind::{ActisenseFormat, Bitrates, Kind, ListUse, Setting, Transport};

/// The help's width, as the rest of it is written.
const WIDTH: usize = 79;

/// The options that only some kinds take, by their flags.
const OPTIONS: &[(&str, Setting)] = &[
    ("--baud", Setting::Baud),
    ("--bitrate", Setting::Bitrate),
    ("--channel", Setting::Channel),
    ("--mode", Setting::Mode),
    ("--password", Setting::Password),
    ("--no-tx-rate-limit", Setting::NoTxRateLimit),
    ("--ignore-checksum", Setting::IgnoreChecksum),
    ("--node", Setting::RunNode),
    ("node options", Setting::Node),
    ("--no-claim", Setting::NoClaim),
];

/// `a`, `a or b`, `a, b or c`.
fn or_list(items: &[String]) -> String {
    match items {
        [] => String::new(),
        [one] => one.clone(),
        [init @ .., last] => format!("{} or {last}", init.join(", ")),
    }
}

/// The kinds that pass `filter`, as the help names them.
fn kinds(filter: impl Fn(Kind) -> bool) -> Vec<String> {
    Kind::ALL
        .iter()
        .copied()
        .filter(|&k| filter(k))
        .map(|k| k.to_string())
        .collect()
}

/// The kinds `--protocol protocol` works with: `socketcan, canalyst or
/// usb-can-a`.
pub fn kinds_carrying(protocol: BusProtocol) -> String {
    or_list(&kinds(|k| k.capabilities().protocols.contains(&protocol)))
}

/// The kinds that have `setting`: `socketcan, canalyst or usb-can-a`.
pub fn kinds_taking(setting: Setting) -> String {
    or_list(&kinds(|k| k.takes(setting)))
}

/// The same, as a plain list: `socketcan, canalyst, usb-can-a`.
fn kinds_taking_list(setting: Setting) -> String {
    kinds(|k| k.takes(setting)).join(", ")
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
    if kind.takes(Setting::RunNode) {
        format!("{}; canboat is the node with --node", kind.describe())
    } else if kind.capabilities().node {
        format!("{}; canboat is the node", kind.describe())
    } else {
        kind.describe().to_string()
    }
}

/// `text` wrapped to [`WIDTH`], every line after the first indented by
/// `hang`, the first by `first`.
fn wrap(text: &str, first: usize, hang: usize) -> String {
    let mut out = String::new();
    let mut line = " ".repeat(first);
    let mut empty = true;
    for word in text.split_whitespace() {
        if !empty && line.len() + 1 + word.len() > WIDTH {
            out.push_str(line.trim_end());
            out.push('\n');
            line = " ".repeat(hang);
            empty = true;
        }
        if !empty {
            line.push(' ');
        }
        line.push_str(word);
        empty = false;
    }
    out.push_str(line.trim_end());
    out
}

/// A text table: each row's cells padded to their column, the last
/// wrapped under itself.
fn text_table(rows: &[Vec<String>]) -> String {
    let columns = rows.iter().map(Vec::len).max().unwrap_or(0);
    let widths: Vec<usize> = (0..columns)
        .map(|c| {
            rows.iter()
                .filter_map(|r| r.get(c))
                .map(|cell| cell.chars().count())
                .max()
                .unwrap_or(0)
        })
        .collect();
    let mut out = String::new();
    for row in rows {
        let mut line = String::from("  ");
        for (c, cell) in row.iter().enumerate() {
            if c + 1 == row.len() {
                // The first line starts where the column does, too.
                let indent = line.chars().count();
                let wrapped = wrap(cell, indent, indent);
                line.push_str(wrapped.trim_start_matches(' '));
            } else {
                let pad = widths[c] - cell.chars().count();
                line.push_str(cell);
                line.push_str(&" ".repeat(pad + 2));
            }
        }
        out.push_str(line.trim_end());
        out.push('\n');
    }
    out
}

fn yes(on: bool) -> &'static str {
    if on { "yes" } else { "-" }
}

/// The data format column: an Actisense gateway's, with `*` on the one
/// it is when none is given.
fn format_cell(kind: Kind, star: &str) -> String {
    match kind.data_format() {
        Some(ActisenseFormat::NgtActisense) => {
            format!("{}{star}", ActisenseFormat::NgtActisense.name())
        }
        Some(format) => format.name().to_string(),
        None => String::new(),
    }
}

/// The kind column: the name on the first row of a family only.
fn kind_cell(kind: Kind, previous: Option<Kind>) -> Option<&'static str> {
    (previous.map(Kind::name) != Some(kind.name())).then(|| kind.name())
}

/// Whether the kind both reads and writes, at best.
fn read_write(kind: Kind) -> &'static str {
    if kind.capabilities().sends { "rw" } else { "r" }
}

fn device_text(kind: Kind) -> String {
    kind.transports()
        .iter()
        .map(|t| match *t {
            Transport::Serial { .. } => "/dev/tty…".to_string(),
            Transport::Usb => "usb".to_string(),
            Transport::Tcp { port } => format!("tcp:{port}"),
            Transport::Udp { .. } => "udp".to_string(),
            Transport::Interface => "can0".to_string(),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(test)]
fn device_markdown(kind: Kind) -> String {
    kind.transports()
        .iter()
        .map(|t| match *t {
            Transport::Serial { baud } => format!("serial ({baud})"),
            Transport::Usb => "`usb`".to_string(),
            Transport::Tcp { port } => format!("`tcp://` ({port})"),
            Transport::Udp { port: Some(port) } => format!("`udp://` ({port})"),
            Transport::Udp { port: None } => "`udp://`".to_string(),
            Transport::Interface => "`can0`".to_string(),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// The kinds that do the same with both PGN lists, grouped, in
/// [`Kind::ALL`]'s order; those that ignore both last.
fn pgn_list_groups() -> Vec<(Vec<Kind>, ListUse, ListUse)> {
    let mut groups: Vec<(Vec<Kind>, ListUse, ListUse)> = Vec::new();
    for &kind in Kind::ALL {
        let uses = kind.pgn_lists();
        match groups
            .iter_mut()
            .find(|(_, rx, tx)| (*rx, *tx) == (uses.rx, uses.tx))
        {
            Some((kinds, _, _)) => kinds.push(kind),
            None => groups.push((vec![kind], uses.rx, uses.tx)),
        }
    }
    let ignored = |&(_, rx, tx): &(Vec<Kind>, ListUse, ListUse)| {
        rx == ListUse::Ignored && tx == ListUse::Ignored
    };
    groups.sort_by_key(|g| ignored(g));
    groups
}

/// `name`, and that what it does with the PGN lists is its node's, which
/// runs only with `--node` (`flag`) on a kind that takes it.
fn with_node(kind: &Kind, name: String, flag: &str) -> String {
    if kind.takes(Setting::RunNode) && kind.pgn_lists().rx == ListUse::Advertised {
        format!("{name} (with {flag})")
    } else {
        name
    }
}

/// The end of `--help` for both commands.
pub fn gateway_help() -> &'static str {
    static HELP: LazyLock<String> = LazyLock::new(render_gateway_help);
    &HELP
}

fn render_gateway_help() -> String {
    let mut out = String::from("Gateways (--kind) and their data formats (--data-format):\n\n");
    let mut rows = vec![
        ["KIND", "--data-format", "DEVICE", "R/W", "J1939", "11-bit"]
            .map(String::from)
            .to_vec(),
    ];
    let mut previous = None;
    for &kind in Kind::ALL {
        let caps = kind.capabilities();
        rows.push(vec![
            kind_cell(kind, previous).unwrap_or("").to_string(),
            format_cell(kind, "*"),
            device_text(kind),
            read_write(kind).to_string(),
            yes(caps.protocols.contains(&BusProtocol::J1939)).to_string(),
            yes(caps.standard_frames).to_string(),
        ]);
        previous = Some(kind);
    }
    out.push_str(&text_table(&rows));
    out.push('\n');
    let notes = [
        format!(
            "* the default. rw: canboat reads from and writes to the bus; a \
             udp://[bind:]port only reads. J1939 (--protocol j1939): where it says yes; \
             ISO TP messages are received there, and sent on {}. 11-bit: Quick PCS (--protocol quick). The others carry NMEA 2000 \
             only.",
            or_list(&kinds(|k| k.capabilities().iso_tp_sends))
        ),
        "A W2K-1 sends ngt-actisense but does not take it; canboat does not know that \
         yet. seasmart ($PCDIN) has no priority or destination, and not every gateway \
         takes it to send: Yacht Devices only a few PGNs."
            .to_string(),
        "actisense mxpgn: $MXPGN with the data in bus order, unlike a MiniPlex's (--kind \
         miniplex). A W2K-2 sends it only on its hidden CANDIRECT data server (port \
         20000) set to NMEA 0183; its own data servers' NMEA 0183 has no $MXPGN. A W2K-1 \
         is not known to send it."
            .to_string(),
        "candump runs canboat's node only with --node: then it claims an address, \
         answers ISO Requests and advertises the PGN lists, as on socketcan. Without, frames go \
         out as they are given, source address and all, and the PGN lists are ignored."
            .to_string(),
        "ikonvert: tcp:// is the better mode for a NavLink2. Over udp://[bind:][port] \
         (its UDP output, port 2000) canboat only receives: it expects the gateway to be \
         on the bus already, put there by another client, and never takes it off. Its \
         PGN lists and rate limit stay as they are."
            .to_string(),
        format!(
            "aos is the Across Ocean Systems simulator, which speaks the NGT-1's format but \
             is no node: canboat claims an address for it. actisense (ngt-actisense) and aos \
             each give up on a gateway that turns out to be the other. The names of canboat \
             8.3.0 still work: --kind {}.",
            Kind::OLD_NAMES
                .iter()
                .map(|(name, _)| *name)
                .collect::<Vec<_>>()
                .join(", ")
        ),
    ];
    for note in &notes {
        out.push_str(&wrap(note, 2, 2));
        out.push_str("\n\n");
    }

    out.push_str("Options only some gateways take:\n\n");
    let rows: Vec<Vec<String>> = OPTIONS
        .iter()
        .map(|&(flag, setting)| vec![flag.to_string(), kinds_taking_list(setting)])
        .collect();
    out.push_str(&text_table(&rows));
    out.push('\n');

    out.push_str("What each gateway does with the PGN lists:\n\n");
    for (kinds, rx, tx) in pgn_list_groups() {
        let who = if rx == ListUse::Ignored && tx == ListUse::Ignored {
            "the others".to_string()
        } else {
            kinds
                .iter()
                .map(|k| with_node(k, k.to_string(), "--node"))
                .collect::<Vec<_>>()
                .join(", ")
        };
        out.push_str(&wrap(
            &format!(
                "{who}: --rx-pgn {}; --tx-pgn {}.",
                rx.describe(),
                tx.describe()
            ),
            2,
            4,
        ));
        out.push('\n');
    }
    out.push('\n');
    out.push_str(&wrap(
        &format!(
            "--rx-pgn never filters; --rx-filter does, in the gateway on {} and in canboat \
             elsewhere. Network management PGNs always pass, and are always sent.",
            kinds_taking(Setting::RxFilter)
        ),
        2,
        2,
    ));
    out
}

/// The help heading of the CAN controller options.
pub fn can_heading() -> &'static str {
    static HEADING: LazyLock<String> =
        LazyLock::new(|| format!("CAN controller ({})", kinds_taking_list(Setting::Bitrate)));
    &HEADING
}

/// The help heading of canboat's node options.
pub fn node_heading() -> &'static str {
    static HEADING: LazyLock<String> =
        LazyLock::new(|| format!("canboat's node ({})", kinds_taking_list(Setting::Node)));
    &HEADING
}

/// `--data-format`'s help.
pub fn data_format_help() -> String {
    format!(
        "{}: the data format it is set to [default: {}].",
        Kind::NGT1.name(),
        ActisenseFormat::NgtActisense.name()
    )
}

/// `--baud`'s help: each kind's serial speed when none is given.
pub fn baud_help() -> String {
    let mut by_baud: Vec<(u32, Vec<String>)> = Vec::new();
    for &kind in Kind::ALL {
        let Some(baud) = kind.default_baud() else {
            continue;
        };
        match by_baud.iter_mut().find(|(b, _)| *b == baud) {
            Some((_, kinds)) => kinds.push(kind.to_string()),
            None => by_baud.push((baud, vec![kind.to_string()])),
        }
    }
    let defaults: Vec<String> = by_baud
        .iter()
        .map(|(baud, kinds)| format!("{} {baud}", kinds.join(", ")))
        .collect();
    format!("Serial baud rate [default: {}]", defaults.join("; "))
}

/// `--bitrate`'s help.
pub fn bitrate_help() -> String {
    let fixed: Vec<String> = kinds(|k| matches!(k.capabilities().bitrates, Bitrates::Only(_)));
    let any: Vec<String> = kinds(|k| k.capabilities().bitrates == Bitrates::Any);
    format!(
        "The CAN bus bit rate [default: {} 250000; {} as configured]",
        fixed.join(", "),
        any.join(", ")
    )
}

/// `--bitrate`'s long help.
pub fn bitrate_long_help() -> String {
    format!(
        "{}\n\nOn {} this brings the interface down, sets the rate and brings it up \
         again, which needs root or CAP_NET_ADMIN. 250000 for NMEA 2000, 500000 for a \
         J1939-14 bus. Not the serial speed to a gateway such as the NGT-1: that is \
         --baud.",
        bitrate_help(),
        kinds_taking(Setting::ConfigureLink)
    )
}

/// The help of an option only `setting`'s kinds have: `what`, after them.
pub fn only_help(setting: Setting, what: &str) -> String {
    format!("{}: {what}", kinds_taking(setting))
}

/// `--tx-pgn`'s help.
pub fn tx_pgn_help() -> String {
    format!(
        "PGNs the application transmits; on {} the only ones sent",
        or_list(&kinds(|k| k.pgn_lists().tx.exclusive()))
    )
}

/// `--tx-pgn`'s long help.
pub fn tx_pgn_long_help() -> String {
    let exclusive = or_list(&kinds(|k| k.pgn_lists().tx.exclusive()));
    format!(
        "{}.\n\nRepeatable, or comma-separated. See the table at the end.\n\nWARNING — WITH \
         {} THIS IS THE WHOLE TRANSMIT LIST: ONCE ANY --tx-pgn IS GIVEN, EVERY OTHER PGN \
         IS REFUSED (NOT SENT), apart from network management. List every PGN you will \
         send.",
        tx_pgn_help(),
        exclusive.to_uppercase()
    )
}

/// `--rx-filter`'s long help.
pub fn rx_filter_long_help() -> String {
    format!(
        "Pass on only these PGNs (and network management); drop the rest.\n\nRepeatable, \
         or comma-separated. {} filters itself (in `NORMAL` mode, and the list is \
         advertised as with --rx-pgn); for every other gateway canboat drops the rest as \
         they arrive. The network-management PGNs (address claims, requests, product \
         information, …) and canboat's own gateway status always pass.",
        kinds_taking(Setting::RxFilter)
    )
}

/// The table sections of `doc/gateways.md`, by the name their markers
/// carry.
#[cfg(test)]
pub fn markdown_section(name: &str) -> Option<String> {
    Some(match name {
        "kinds" => markdown_kinds(),
        "options" => markdown_options(),
        "pgn-lists" => markdown_pgn_lists(),
        _ => return None,
    })
}

/// A Markdown table row.
#[cfg(test)]
fn markdown_row(cells: &[&str]) -> String {
    let mut row = String::from("|");
    for cell in cells {
        if cell.is_empty() {
            row.push_str(" |");
        } else {
            let _ = write!(row, " {cell} |");
        }
    }
    row.push('\n');
    row
}

#[cfg(test)]
fn markdown_kinds() -> String {
    let mut out = String::from(
        "| `--kind` | `--data-format` | Gateways | Device | Read/write | J1939 | 11-bit |\n\
         | --- | --- | --- | --- | --- | --- | --- |\n",
    );
    let mut previous = None;
    for &kind in Kind::ALL {
        let caps = kind.capabilities();
        let code = |s: &str| {
            if s.is_empty() {
                String::new()
            } else {
                format!("`{s}`")
            }
        };
        out.push_str(&markdown_row(&[
            &code(kind_cell(kind, previous).unwrap_or("")),
            &code(&format_cell(kind, "")),
            kind.describe(),
            &device_markdown(kind),
            read_write(kind),
            if caps.protocols.contains(&BusProtocol::J1939) {
                "yes"
            } else {
                "–"
            },
            if caps.standard_frames { "yes" } else { "–" },
        ]));
        previous = Some(kind);
    }
    out
}

#[cfg(test)]
fn markdown_options() -> String {
    let mut out = String::from("| Option | Gateways |\n| --- | --- |\n");
    for &(flag, setting) in OPTIONS {
        let flag = if flag.starts_with("--") {
            format!("`{flag}`")
        } else {
            flag.to_string()
        };
        let kinds: Vec<String> = kinds(|k| k.takes(setting))
            .iter()
            .map(|k| format!("`{k}`"))
            .collect();
        let _ = writeln!(out, "| {flag} | {} |", kinds.join(", "));
    }
    out
}

#[cfg(test)]
fn markdown_pgn_lists() -> String {
    let mut out = String::from(
        "| Gateway | Receive list (`--rx-pgn`) | Transmit list (`--tx-pgn`) |\n\
         | --- | --- | --- |\n",
    );
    for (kinds, rx, tx) in pgn_list_groups() {
        let who = if rx == ListUse::Ignored && tx == ListUse::Ignored {
            "The others".to_string()
        } else {
            kinds
                .iter()
                .map(|k| with_node(k, format!("`{k}`"), "`--node`"))
                .collect::<Vec<_>>()
                .join(", ")
        };
        let _ = writeln!(out, "| {who} | {} | {} |", rx.describe(), tx.describe());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_kinds_that_carry_a_protocol_are_listed() {
        assert_eq!(
            kinds_carrying(BusProtocol::Quick),
            "candump, socketcan, canalyst or usb-can-a"
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

    #[test]
    fn the_help_names_every_kind_and_option_and_fits() {
        let help = gateway_help();
        for &kind in Kind::ALL {
            assert!(help.contains(kind.name()), "{kind}");
            if let Some(format) = kind.data_format() {
                assert!(help.contains(format.name()), "{kind}");
            }
        }
        for &(flag, _) in OPTIONS {
            assert!(help.contains(flag), "{flag}");
        }
        for line in help.lines() {
            assert!(line.chars().count() <= WIDTH, "too wide: {line}");
        }
        assert!(help.contains("tcp:60002"), "{help}");
        assert!(baud_help().contains("ikonvert 230400"), "{}", baud_help());
        assert!(tx_pgn_help().contains("ikonvert"), "{}", tx_pgn_help());
    }

    /// The tables in `doc/gateways.md` are this module's: run with
    /// `CANBOAT_BLESS=1` to write them, then `make markdown-docs`.
    #[test]
    fn the_tables_in_doc_gateways_md_are_current() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../doc/gateways.md");
        let doc = std::fs::read_to_string(path).expect("doc/gateways.md");
        let mut out = String::new();
        let mut rest = doc.as_str();
        let begin = "<!-- BEGIN generated by canboat: ";
        let mut sections = 0;
        while let Some(at) = rest.find(begin) {
            let after = &rest[at + begin.len()..];
            let name_end = after.find(" -->").expect("marker ends");
            let name = &after[..name_end];
            let end = format!("<!-- END generated by canboat: {name} -->");
            let body_start = at + begin.len() + name_end + " -->\n".len();
            let body_end = rest.find(&end).expect("END marker");
            let generated = markdown_section(name).unwrap_or_else(|| panic!("no {name}"));
            out.push_str(&rest[..body_start]);
            out.push_str(&generated);
            rest = &rest[body_end..];
            sections += 1;
        }
        out.push_str(rest);
        assert_eq!(
            sections, 3,
            "doc/gateways.md has its three generated tables"
        );
        if std::env::var_os("CANBOAT_BLESS").is_some() {
            std::fs::write(path, &out).expect("write doc/gateways.md");
        } else {
            assert!(
                out == doc,
                "doc/gateways.md's tables are out of date: run \
                 CANBOAT_BLESS=1 cargo test -p canboat the_tables_in_doc_gateways_md_are_current, \
                 then make markdown-docs"
            );
        }
    }
}
