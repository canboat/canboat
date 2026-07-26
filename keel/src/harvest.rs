//! `keel harvest`: batch-decode capture files and populate `samples:` blocks.
//!
//! DESIGN.md §7.2 makes captured decodes the regression tests (rule R40).
//! The editor grows a `samples:` block one paste at a time ("adopt current
//! decodes as expectations"); this does the same thing in bulk across the
//! `samples/` corpus so every PGN we actually see on a bus gets 1-3 diverse
//! samples without hand-typing any `expects`.
//!
//! Contract, so a harvested sample always passes R40 by construction:
//!   * the decoder here is the same one R40 re-runs, so the values agree;
//!   * `expect_value` is the Rust mirror of the editor's `adoptExpectations`,
//!     emitting each `Value` as a YAML scalar that parses back to the
//!     `Expected` the checker compares against.
//! Strings are always double-quoted to sidestep YAML 1.1 footguns (a lookup
//! named `On`/`Off` would otherwise parse as a boolean).

use crate::decode::{self, Value};
use crate::model::Database;
use crate::samples::{parse_line, RawFrame};
use std::collections::{HashMap, HashSet};
use std::path::Path;

type Result<T> = std::result::Result<T, String>;

// Fixed so the committed YAML is reproducible run to run; the timestamp is
// cosmetic (parse_line ignores it) and matches the house sample style.
const FIXED_TS: &str = "2023-01-01-00:00:00.000";

struct Candidate {
    prio: u8,
    src: u8,
    dst: u8,
    data: Vec<u8>,
    source: String, // corpus file basename, for the provenance comment
}

/// Harvest samples from `files` (given in priority order — earliest wins ties,
/// so pass merrimac captures first) into `database/pgns/*.yaml` under `root`.
/// `per_pgn` caps samples per variant. Returns a human summary.
pub fn harvest(db: &Database, files: &[String], per_pgn: usize, root: &Path) -> Result<String> {
    let fast: HashSet<u32> =
        db.pgns.iter().filter(|p| p.type_ == "Fast").map(|p| p.pgn).collect();

    // (pgn, variant-id) -> distinct-payload set + ordered candidates.
    let mut cand: HashMap<(u32, String), (HashSet<Vec<u8>>, Vec<Candidate>)> = HashMap::new();
    const CAP_PER_VARIANT: usize = 512;

    let mut files_read = 0usize;
    let mut msgs_seen = 0usize;
    for file in files {
        let text = match std::fs::read_to_string(root.join(file)) {
            Ok(t) => t,
            Err(_) => match std::fs::read_to_string(file) {
                Ok(t) => t,
                Err(e) => {
                    eprintln!("keel harvest: skip {file}: {e}");
                    continue;
                }
            },
        };
        files_read += 1;
        let base = Path::new(file)
            .file_name()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| file.clone());

        let mut re = Reassembler::default();
        for line in text.lines() {
            let t = line.trim();
            if t.is_empty() || t.starts_with('#') {
                continue;
            }
            let Ok(f) = parse_line(t) else { continue };
            let Some(a) = re.feed(f, |pgn| fast.contains(&pgn)) else { continue };
            msgs_seen += 1;
            let Some(p) = decode::select_variant(db, a.pgn, &a.data, false) else { continue };
            // Only decodable variants are worth a sample.
            if decode::decode(db, p, &a.data).is_err() {
                continue;
            }
            let entry = cand.entry((a.pgn, p.id.clone())).or_default();
            if entry.1.len() >= CAP_PER_VARIANT {
                continue;
            }
            if entry.0.insert(a.data.clone()) {
                entry.1.push(Candidate {
                    prio: a.prio,
                    src: a.src,
                    dst: a.dst,
                    data: a.data.clone(),
                    source: base.clone(),
                });
            }
        }
    }

    // Select + write.
    let mut variants_written = 0usize;
    let mut samples_written = 0usize;
    let mut skipped_existing = 0usize;
    let mut missing_file = 0usize;

    let mut keys: Vec<&(u32, String)> = cand.keys().collect();
    keys.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));

    for key in keys {
        let (_, cands) = &cand[key];
        let (pgn, id) = (key.0, key.1.as_str());
        let path = root.join(format!("database/pgns/{:06}-{}.yaml", pgn, id));
        if !path.exists() {
            missing_file += 1;
            continue;
        }
        let existing = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        if has_samples_block(&existing) {
            skipped_existing += 1;
            continue;
        }
        let Some(p) = db.pgns.iter().find(|q| q.pgn == pgn && q.id == *id) else { continue };

        let is_fast = p.type_ == "Fast";
        let chosen = select_diverse(cands, per_pgn);
        let mut block = String::from("samples:\n");
        for c in &chosen {
            let raws = raw_lines(pgn, c, is_fast);
            if raws.len() == 1 {
                block.push_str(&format!("- raw: {}  # from {}\n", dq(&raws[0]), c.source));
            } else {
                block.push_str("- raw:\n");
                for (i, r) in raws.iter().enumerate() {
                    if i == 0 {
                        block.push_str(&format!("  - {}  # from {}\n", dq(r), c.source));
                    } else {
                        block.push_str(&format!("  - {}\n", dq(r)));
                    }
                }
            }
            block.push_str("  expects:\n");
            let fields = decode::decode(db, p, &c.data).map_err(|e| format!("{}: {e}", path.display()))?;
            for f in &fields {
                if f.id.starts_with("reserved") || f.id.starts_with("spare") || f.id.is_empty() {
                    continue;
                }
                let Some(val) = expect_value(&f.value) else { continue };
                let ekey = if f.instance > 1 { format!("{}.{}", f.id, f.instance) } else { f.id.clone() };
                block.push_str(&format!("    {ekey}: {val}\n"));
            }
        }

        let mut out = existing;
        if !out.ends_with('\n') {
            out.push('\n');
        }
        out.push_str(&block);
        std::fs::write(&path, out).map_err(|e| format!("{}: {e}", path.display()))?;
        variants_written += 1;
        samples_written += chosen.len();
    }

    Ok(format!(
        "keel harvest: read {files_read} file(s), {msgs_seen} message(s); \
         wrote {samples_written} sample(s) to {variants_written} variant(s) \
         (skipped {skipped_existing} already-sampled, {missing_file} with no yaml)"
    ))
}

/// Greedy max-min diversity: seed with the first candidate (highest-priority
/// file), then repeatedly add the one farthest from everything chosen so far.
fn select_diverse<'a>(cands: &'a [Candidate], n: usize) -> Vec<&'a Candidate> {
    let mut chosen: Vec<&Candidate> = Vec::new();
    if cands.is_empty() {
        return chosen;
    }
    chosen.push(&cands[0]);
    while chosen.len() < n && chosen.len() < cands.len() {
        let mut best: Option<(&Candidate, usize)> = None;
        for c in cands {
            if chosen.iter().any(|x| std::ptr::eq(*x, c)) {
                continue;
            }
            let d = chosen.iter().map(|x| dist(&x.data, &c.data)).min().unwrap_or(0);
            if best.map_or(true, |(_, bd)| d > bd) {
                best = Some((c, d));
            }
        }
        match best {
            Some((c, _)) => chosen.push(c),
            None => break,
        }
    }
    chosen
}

/// Byte-level distance: Hamming over the shared prefix plus the length gap.
fn dist(a: &[u8], b: &[u8]) -> usize {
    let common = a.len().min(b.len());
    let mut d = a.len().abs_diff(b.len());
    for i in 0..common {
        if a[i] != b[i] {
            d += 1;
        }
    }
    d
}

/// Synthesize the `raw:` PLAIN line(s) for a candidate.
///
/// Non-fast, or fast with a payload > 8 bytes: one pre-assembled coalesced
/// line (R40 reassembly passes len>8 straight through). A fast packet whose
/// assembled payload is <= 8 bytes (a single-frame fast packet, e.g. total
/// length 6) can't be stored coalesced: a bare <=8-byte line is
/// indistinguishable from one CAN frame and R40 would re-read it as a
/// fast-packet frame. Those are re-fragmented into real fast-packet frames
/// (sequence 0) so R40 reassembles them back to the same payload.
fn raw_lines(pgn: u32, c: &Candidate, is_fast: bool) -> Vec<String> {
    let plain = |data: &[u8]| -> String {
        let hex: Vec<String> = data.iter().map(|b| format!("{b:02x}")).collect();
        format!("{FIXED_TS},{},{},{},{},{},{}", c.prio, pgn, c.src, c.dst, data.len(), hex.join(","))
    };
    if !is_fast || c.data.len() > 8 {
        return vec![plain(&c.data)];
    }
    // Re-fragment: frame 0 = [0x00, total, up to 6 bytes], then [idx, up to 7].
    let total = c.data.len();
    let mut lines = Vec::new();
    let n0 = total.min(6);
    let mut frame0 = vec![0x00u8, total as u8];
    frame0.extend_from_slice(&c.data[..n0]);
    lines.push(plain(&frame0));
    let mut pos = n0;
    let mut idx = 1u8;
    while pos < total {
        let n = (total - pos).min(7);
        let mut fr = vec![idx];
        fr.extend_from_slice(&c.data[pos..pos + n]);
        lines.push(plain(&fr));
        pos += n;
        idx += 1;
    }
    lines
}

/// A top-level `samples:` key already present? (crude but sufficient: our
/// files are keel-generated, so a line-start `samples:` is unambiguous.)
fn has_samples_block(yaml: &str) -> bool {
    yaml.lines().any(|l| l == "samples:" || l.starts_with("samples:"))
}

/// `Value` -> YAML scalar for an `expects` entry. `None` = don't assert this
/// field. Mirror of the editor's `adoptExpectations`.
fn expect_value(v: &Value) -> Option<String> {
    Some(match v {
        Value::Number { value, decimals } => {
            if !value.is_finite() {
                return None;
            }
            format!("{:.*}", *decimals as usize, value)
        }
        Value::Lookup { name: Some(n), .. } => dq(n),
        Value::Lookup { name: None, value } => value.to_string(),
        Value::Bits(names) => {
            format!("[{}]", names.iter().map(|s| dq(s)).collect::<Vec<_>>().join(", "))
        }
        Value::Str(s) => dq(s),
        Value::Binary(h) => dq(&format!("0x{h}")),
        Value::Unavailable => "~".to_string(),
        Value::OutOfRange => dq("OutOfRange"),
        Value::Reserved => dq("Reserved"),
    })
}

/// Double-quote a string as a YAML 1.1 flow scalar (escaping the few chars
/// that matter). Always quoting keeps `On`/`Yes`/`123`-style lookup names
/// from being reinterpreted as booleans or numbers.
fn dq(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for ch in s.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\t' => out.push_str("\\t"),
            '\r' => out.push_str("\\r"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\x{:02x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// Streaming fast-packet reassembler (single-message-at-a-time port of
/// samples::reassemble_lenient, so a 76 MB capture never materializes all
/// messages at once).
#[derive(Default)]
struct Reassembler {
    slots: HashMap<(u32, u8, u8), Slot>,
}

#[derive(Default)]
struct Slot {
    total: usize,
    seen: u32,
    data: Vec<u8>,
}

pub struct HarvestMsg {
    pub prio: u8,
    pub pgn: u32,
    pub src: u8,
    pub dst: u8,
    pub data: Vec<u8>,
}

impl Reassembler {
    /// Feed one frame; returns a completed message if this frame finished one.
    fn feed<F: Fn(u32) -> bool>(&mut self, f: RawFrame, is_fast: F) -> Option<HarvestMsg> {
        // Single-frame (or already-assembled coalesced) -> pass straight through.
        if !is_fast(f.pgn) || f.data.len() > 8 {
            return Some(HarvestMsg { prio: f.prio, pgn: f.pgn, src: f.src, dst: f.dst, data: f.data });
        }
        if f.data.is_empty() {
            return None;
        }
        let seq = f.data[0] >> 5;
        let index = (f.data[0] & 0x1f) as usize;
        let key = (f.pgn, f.src, seq);
        let slot = self.slots.entry(key).or_default();
        if index == 0 {
            if f.data.len() < 2 {
                self.slots.remove(&key);
                return None;
            }
            slot.total = f.data[1] as usize;
            slot.data = vec![0xff; slot.total.max(6)];
            let n = (f.data.len() - 2).min(6).min(slot.total);
            slot.data[..n].copy_from_slice(&f.data[2..2 + n]);
            slot.seen = 1;
        } else {
            if slot.total == 0 {
                self.slots.remove(&key);
                return None;
            }
            let start = 6 + (index - 1) * 7;
            if start < slot.total {
                let n = (f.data.len() - 1).min(7).min(slot.total - start);
                if slot.data.len() < start + n {
                    slot.data.resize(start + n, 0xff);
                }
                slot.data[start..start + n].copy_from_slice(&f.data[1..1 + n]);
            }
            slot.seen |= 1 << index;
        }
        let frames_needed = if slot.total <= 6 { 1 } else { 1 + (slot.total - 6).div_ceil(7) };
        let mask = if frames_needed >= 32 { u32::MAX } else { (1u32 << frames_needed) - 1 };
        if slot.total > 0 && slot.seen & mask == mask {
            let mut data = std::mem::take(&mut slot.data);
            data.truncate(slot.total);
            let (pgn, src, _) = key;
            let prio = f.prio;
            self.slots.remove(&key);
            return Some(HarvestMsg { prio, pgn, src, dst: f.dst, data });
        }
        None
    }
}
