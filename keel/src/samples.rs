//! Sample-record parsing and fast-packet reassembly.
//!
//! The editor wizard and the R40 sample tests accept real captures in a few
//! formats and need assembled payloads. This is a compact, batch-oriented
//! subset of canboat-rs's streaming Reassembler (canboat-io), kept as a copy
//! because a crate dependency would be circular (canboat-rs consumes this
//! repo's canboat.json). ISO-TP reassembly is a possible follow-up; pasted
//! samples for TP-wrapped PGNs can use the pre-assembled `data:` form.

type Result<T> = std::result::Result<T, String>;

#[derive(Debug, Clone)]
pub struct RawFrame {
    pub prio: u8,
    pub pgn: u32,
    pub src: u8,
    pub dst: u8,
    pub data: Vec<u8>,
}

/// Parse one sample line; format auto-detected:
///  - canboat PLAIN: `<ts>,<prio>,<pgn>,<src>,<dst>,<len>,<hh>,<hh>,...`
///  - candump:       `[(ts)] [ifname] <8-hex-ID>#<hexdata>` (29-bit id)
pub fn parse_line(line: &str) -> Result<RawFrame> {
    let line = line.trim();
    if line.is_empty() || line.starts_with('#') {
        return Err("empty line".into());
    }
    if let Some(hash) = line.find('#') {
        // candump: the token immediately before '#' is the CAN id
        let (head, hex) = line.split_at(hash);
        let id_tok = head
            .split_whitespace()
            .last()
            .ok_or("candump: missing CAN id")?;
        let id = u32::from_str_radix(id_tok, 16).map_err(|e| format!("candump id: {e}"))?;
        let data = parse_hex(&hex[1..].replace(' ', ""))?;
        return Ok(from_can_id(id, data));
    }
    // canboat PLAIN
    let parts: Vec<&str> = line.split(',').collect();
    if parts.len() < 7 {
        return Err(format!("not a recognized sample format: {line}"));
    }
    let prio: u8 = parts[1].trim().parse().map_err(|e| format!("prio: {e}"))?;
    let pgn: u32 = parts[2].trim().parse().map_err(|e| format!("pgn: {e}"))?;
    let src: u8 = parts[3].trim().parse().map_err(|e| format!("src: {e}"))?;
    let dst: u8 = parts[4].trim().parse().map_err(|e| format!("dst: {e}"))?;
    let len: usize = parts[5].trim().parse().map_err(|e| format!("len: {e}"))?;
    if parts.len() < 6 + len {
        return Err(format!("PLAIN line declares {len} bytes but carries {}", parts.len() - 6));
    }
    let data = parts[6..6 + len]
        .iter()
        .map(|b| u8::from_str_radix(b.trim(), 16).map_err(|e| format!("hex byte: {e}")))
        .collect::<Result<Vec<u8>>>()?;
    Ok(RawFrame { prio, pgn, src, dst, data })
}

pub fn parse_hex(s: &str) -> Result<Vec<u8>> {
    let s: String = s.chars().filter(|c| !c.is_whitespace() && *c != ',').collect();
    if s.len() % 2 != 0 {
        return Err("odd number of hex digits".into());
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| format!("hex: {e}")))
        .collect()
}

/// Split a 29-bit extended CAN id into (prio, pgn, src, dst) per J1939/N2K.
fn from_can_id(id: u32, data: Vec<u8>) -> RawFrame {
    let prio = ((id >> 26) & 0x7) as u8;
    let src = (id & 0xff) as u8;
    let dp = (id >> 24) & 3;
    let pf = (id >> 16) & 0xff;
    let ps = (id >> 8) & 0xff;
    let (pgn, dst) = if pf < 240 {
        ((dp << 16) | (pf << 8), ps as u8) // PDU1: PS is the destination
    } else {
        ((dp << 16) | (pf << 8) | ps, 255) // PDU2: broadcast
    };
    RawFrame { prio, pgn, src, dst, data }
}

/// One reassembled message ready for decoding.
#[derive(Debug, Clone)]
pub struct Assembled {
    pub prio: u8,
    pub pgn: u32,
    pub src: u8,
    pub dst: u8,
    pub data: Vec<u8>,
}

/// Batch fast-packet reassembly over an ordered set of frames.
///
/// `is_fast(pgn)` comes from the database. Single-frame (and unknown) PGNs
/// pass through unchanged. Mirrors the C reassembly: byte 0 = sequence
/// (high 3 bits) + frame index (low 5), byte 1 of frame 0 = total length.
pub fn reassemble<F: Fn(u32) -> bool>(frames: &[RawFrame], is_fast: F) -> Result<Vec<Assembled>> {
    let (out, warnings) = reassemble_lenient(frames, is_fast)?;
    if let Some(w) = warnings.first() {
        return Err(w.clone());
    }
    Ok(out)
}

/// Like [`reassemble`], but out-of-order / incomplete fast-packet groups
/// become warnings instead of failing the whole batch - for `keel decode`
/// over arbitrary capture files. R40 uses the strict variant.
pub fn reassemble_lenient<F: Fn(u32) -> bool>(
    frames: &[RawFrame],
    is_fast: F,
) -> Result<(Vec<Assembled>, Vec<String>)> {
    #[derive(Default)]
    struct Slot {
        total: usize,
        seen: u32,
        data: Vec<u8>,
    }
    let mut out = Vec::new();
    let mut warnings = Vec::new();
    let mut slots: std::collections::HashMap<(u32, u8, u8), Slot> = Default::default();

    for f in frames {
        // len > 8 = already-assembled (coalesced) payload, pass through
        if !is_fast(f.pgn) || f.data.len() > 8 {
            out.push(Assembled {
                prio: f.prio,
                pgn: f.pgn,
                src: f.src,
                dst: f.dst,
                data: f.data.clone(),
            });
            continue;
        }
        if f.data.is_empty() {
            return Err(format!("PGN {}: empty fast-packet frame", f.pgn));
        }
        let seq = f.data[0] >> 5;
        let index = (f.data[0] & 0x1f) as usize;
        let key = (f.pgn, f.src, seq);
        let slot = slots.entry(key).or_default();
        if index == 0 {
            if f.data.len() < 2 {
                return Err(format!("PGN {}: fast-packet frame 0 too short", f.pgn));
            }
            slot.total = f.data[1] as usize;
            slot.data = vec![0xff; slot.total.max(6)];
            let n = (f.data.len() - 2).min(6).min(slot.total);
            slot.data[..n].copy_from_slice(&f.data[2..2 + n]);
            slot.seen |= 1;
        } else {
            if slot.total == 0 {
                warnings.push(format!(
                    "PGN {}: fast-packet frame {index} before frame 0, group skipped",
                    f.pgn
                ));
                slots.remove(&key);
                continue;
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
        // Complete when every needed frame index is present
        let frames_needed = if slot.total <= 6 { 1 } else { 1 + (slot.total - 6).div_ceil(7) };
        let mask = if frames_needed >= 32 { u32::MAX } else { (1u32 << frames_needed) - 1 };
        if slot.total > 0 && slot.seen & mask == mask {
            let mut data = std::mem::take(&mut slot.data);
            data.truncate(slot.total);
            out.push(Assembled {
                prio: f.prio,
                pgn: f.pgn,
                src: f.src,
                dst: f.dst,
                data,
            });
            slots.remove(&key);
        }
    }
    for ((pgn, src, _seq), slot) in slots {
        if slot.seen != 0 {
            warnings.push(format!(
                "PGN {pgn} src {src}: incomplete fast-packet (total {} bytes, frames seen {:#b})",
                slot.total, slot.seen
            ));
        }
    }
    Ok((out, warnings))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plain_single_frame() {
        let f = parse_line("2023-01-01-00:00:00.000,2,130306,42,255,8,00,b4,01,ac,58,03,fa,ff").unwrap();
        assert_eq!(f.pgn, 130306);
        assert_eq!(f.src, 42);
        assert_eq!(f.data.len(), 8);
        assert_eq!(f.data[1], 0xb4);
    }

    #[test]
    fn candump_pdu2() {
        // 0x19F51423: prio 6, DP 1, PF 0xF5, PS 0x14 -> PGN 0x1F514, src 0x23
        let f = parse_line("can0  19F51423   [8]  01 02 03 04 05 06 07 08".replace("   [8]  ", "#0102030405060708").as_str()).unwrap();
        assert_eq!(f.pgn, 0x1F514);
        assert_eq!(f.src, 0x23);
        assert_eq!(f.prio, 6);
    }

    #[test]
    fn fastpacket_two_frames() {
        let frames = vec![
            parse_line("t,3,129029,1,255,8,a0,2b,e7,95,3d,00,73,d6").unwrap(),
            parse_line("t,3,129029,1,255,8,a1,29,00,da,04,73,db,c9").unwrap(),
        ];
        // 0x2b = 43 bytes total; only 2 of 7 frames -> incomplete must error
        assert!(reassemble(&frames, |_| true).is_err());
    }

    #[test]
    fn fastpacket_complete() {
        let frames = vec![
            parse_line("t,2,127489,16,255,8,c0,1a,00,c8,00,30,09,38").unwrap(),
            parse_line("t,2,127489,16,255,8,c1,0b,64,00,7d,05,64,00").unwrap(),
            parse_line("t,2,127489,16,255,8,c2,00,00,00,00,7f,7f,ff").unwrap(),
            parse_line("t,2,127489,16,255,8,c3,ff,ff,ff,ff,ff,ff,ff").unwrap(),
        ];
        let out = reassemble(&frames, |_| true).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].data.len(), 0x1a);
        assert_eq!(out[0].data[0], 0x00);
        assert_eq!(out[0].data[5], 0x38);
        assert_eq!(out[0].data[6], 0x0b);
    }
}
