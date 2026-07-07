//! `keel explain` - the human-readable text dump of the PGN database.
//!
//! Port of analyzer-explain.c explain()/explainPGN() (deleted at migration
//! step 5). Informational output for humans; not a golden-tested artifact,
//! but kept close to the original format.

use crate::cformat::c_g;
use crate::model::{ACTISENSE_BEM, Database, Field, Interval, Pgn};

pub fn emit_text(db: &Database, j1939: bool) -> String {
    let mut out = String::with_capacity(1 << 20);
    out.push_str(&format!(
        "CANboat version v{}\n\n\
         This program can understand a number of N2K messages. What follows is an explanation of the messages\n\
         that it understands. First is a list of completely understood messages, as far as I can tell.\n\
         What follows is a list of messages that contain fields that have unknown content or size, or even\n\
         completely unknown fields. If you happen to know more, please tell me!\n\n",
        db.version
    ));
    let list = if j1939 { &db.pgns_j1939 } else { &db.pgns };
    out.push_str("_______ Complete PGNs _________\n\n");
    for p in list.iter().filter(|p| p.is_complete() && p.pgn < ACTISENSE_BEM) {
        explain_pgn(db, p, &mut out);
    }
    out.push_str("_______ Incomplete PGNs _________\n\n");
    for p in list.iter().filter(|p| !p.is_complete() && p.pgn < ACTISENSE_BEM) {
        explain_pgn(db, p, &mut out);
    }
    out
}

fn explain_pgn(db: &Database, p: &Pgn, out: &mut String) {
    out.push_str(&format!(
        "PGN: {} / {:08o} / {:05X} - {}\n\n",
        p.pgn, p.pgn, p.pgn, p.description
    ));
    if let Some(e) = &p.explanation {
        out.push_str(&format!("     {e}\n"));
    }
    if let Some(u) = &p.url {
        out.push_str(&format!("     URL: {u}\n"));
    }
    if p.is_variable {
        out.push_str(&format!("     The length is variable but at least {} bytes\n", p.length));
    } else {
        out.push_str(&format!("     The length is {} bytes\n", p.length));
    }

    for (rep, _n) in [(&p.repeating1, 1), (&p.repeating2, 2)] {
        if let Some(rep) = rep {
            if rep.count > 0 {
                match rep.count_field {
                    Some(cf) => out.push_str(&format!(
                        "     Fields {} thru {} repeat n times, where n is the value contained in field {}.\n\n",
                        rep.start,
                        rep.start + rep.count,
                        cf
                    )),
                    None => out.push_str(&format!(
                        "     Fields {} thru {} repeat until the data in the PGN is exhausted.\n\n",
                        rep.start,
                        rep.start + rep.count
                    )),
                }
            }
        }
    }

    match p.interval {
        Interval::Ms(ms) if ms != 0 => {
            out.push_str(&format!("     The PGN is normally transmitted every {ms} ms\n"))
        }
        Interval::Irregular => {
            out.push_str("     The PGN is transmitted on-demand or when data is available\n")
        }
        _ => {}
    }
    if p.priority != 0 {
        out.push_str(&format!("     The default priority is {}\n", p.priority));
    }

    for f in &p.fields {
        explain_field(db, f, out);
    }
    out.push_str("\n\n");
}

fn explain_field(db: &Database, f: &Field, out: &mut String) {
    let desc = f.description.as_deref().filter(|d| !d.is_empty() && !d.starts_with(','));
    out.push_str(&format!(
        "  Field #{}: {}{}{}\n",
        f.order,
        f.name,
        if desc.is_some() { " - " } else { "" },
        desc.unwrap_or("")
    ));
    if f.res_bits == 0 {
        out.push_str("                  Bits: variable\n");
    } else {
        out.push_str(&format!("                  Bits: {}\n", f.res_bits));
    }

    if let Some(m) = &f.match_ {
        out.push_str(&format!("                  Match: {m}\n"));
    } else if let Some(u) = &f.res_unit {
        out.push_str(&format!("                  Unit: {u}\n"));
    }
    if f.res_resolution != 0.0 {
        out.push_str(&format!("                  Resolution: {}\n", c_g(f.res_resolution)));
    }
    let ft = &db.fieldtypes[f.ft];
    out.push_str(&format!(
        "                  Signed: {}\n",
        if ft.has_sign == Some(true) { "true" } else { "false" }
    ));
    if f.res_offset != 0 {
        if f.res_resolution == 1.0 || f.res_resolution == 0.0 {
            out.push_str(&format!("                  Offset: {}\n", f.res_offset));
        } else {
            out.push_str(&format!(
                "                  Offset: {}\n",
                (f.res_offset as f64 * f.res_resolution) as i64
            ));
        }
    }
    if f.primary_key {
        out.push_str("                  Part Of Primary Key: true\n");
    }

    if let Some((kind, name)) = f.lookup_ref() {
        let lk = &db.lookups[name];
        match kind {
            "pair" | "fieldtype" => {
                out.push_str(&format!("                  Enumeration: {name}\n"));
                if f.match_.is_none() {
                    let max = (1u128 << f.res_bits) - 1;
                    out.push_str(&format!("                  Range: 0..{max}\n"));
                    if kind == "pair" {
                        for (v, s) in &lk.pairs {
                            out.push_str(&format!("                  Lookup: {v}={s}\n"));
                        }
                    } else {
                        for e in &lk.fieldtypes {
                            out.push_str(&format!(
                                "                  Lookup: {}={}, fieldType '{}'\n",
                                e.value, e.name, e.fieldtype
                            ));
                        }
                    }
                }
            }
            "triplet" => {
                let max = (1u128 << f.res_bits) - 1;
                out.push_str(&format!("                  IndirectEnumeration: {name}\n"));
                out.push_str(&format!("                  Range: 0..{max}\n"));
                for (v1, v2, s) in &lk.triplets {
                    out.push_str(&format!("                  Lookup: {v1},{v2}={s}\n"));
                }
            }
            "bit" => {
                out.push_str(&format!("                  BitEnumeration: {name}\n"));
                out.push_str(&format!("                  BitRange: 0..{}\n", f.res_bits));
                for (b, s) in &lk.pairs {
                    out.push_str(&format!("                  Bit: {b}={s}\n"));
                }
            }
            _ => {}
        }
    }
}
