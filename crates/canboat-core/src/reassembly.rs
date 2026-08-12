// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Fast-packet reassembly state machine (sans-I/O).
//!
//! NMEA 2000 fast-packet PGNs are split across up to 32 CAN frames.
//! The first byte of each frame's CAN payload is a header:
//!
//! ```text
//!   bits 7..5 : 3-bit sequence counter (rotates 0..7 per PGN/src)
//!   bits 4..0 : 5-bit frame index (0..31 within this sequence)
//! ```
//!
//! Frame 0 then carries one byte of total payload length and 6 bytes of
//! payload. Subsequent frames carry 7 bytes of payload each.
//!
//! The caller feeds raw CAN-sized frames (≤8 data bytes) along with
//! whether the PGN is fast-packet (looked up from the PGN database).
//! Single-frame PGNs and already-coalesced frames (`data.len() > 8`)
//! pass straight through.
//!
//! Slot eviction follows canboat: 64 slots, FIFO oldest-out on overflow.

use crate::frame::{FASTPACKET_MAX_SIZE, RawFrame};

const REASSEMBLY_BUFFER_SIZE: usize = 64;
const BUCKET_0_SIZE: usize = 6;
const BUCKET_N_SIZE: usize = 7;
const BUCKET_0_OFFSET: usize = 2;
const BUCKET_N_OFFSET: usize = 1;
const FASTPACKET_MAX_INDEX: u32 = 0x1f;

/// ISO Transport Protocol carries messages that don't fit in a
/// single CAN frame OR in fast-packet framing (223-byte ceiling).
/// Newer devices (Neon GPS is the trigger for this code) wrap PGN
/// 129540 (Satellites in View) in ISO TP when the sat list is
/// large enough to exceed fast-packet's 223 bytes.
///
/// Two PGNs are involved:
///
/// * **PGN 60416 (`0xEC00`) — TP.CM (Connection Management).** One
///   frame that announces the transfer. The Group Function Code in
///   byte 0 says what kind:
///     * `32` (0x20) BAM — Broadcast Announce, no ACK expected.
///     * `16` (0x10) RTS — Request To Send, addressed, with CTS
///       handshake.
///     * `17` / `19` / `255` — CTS / EOM / Abort, only meaningful
///       to the peers of an RTS session.
///
///   Layout of a BAM / RTS CM frame:
///
///   ```text
///     B0        control (32 or 16)
///     B1..B2    total message size, little-endian
///     B3        packet count
///     B4        reserved / max-packets — we don't send CTS so it's
///               ignored on the receive side
///     B5..B7    target PGN, little-endian (24 bits)
///   ```
///
/// * **PGN 60160 (`0xEB00`) — TP.DT (Data Transfer).** The actual
///   payload, split into 7-byte chunks with a 1-based sequence
///   number:
///
///   ```text
///     B0        sequence (1-based)
///     B1..B7    payload chunk (7 bytes; last frame padded)
///   ```
///
/// Both PGN wire frames are single-frame PGNs on the bus, but for
/// a monitoring tool they're plumbing, not data — so the
/// reassembler swallows them and emits a synthesized [`RawFrame`]
/// for the target PGN once all `TP.DT` sequence numbers are in.
const PGN_ISO_TP_CM: u32 = 60416;
const PGN_ISO_TP_DT: u32 = 60160;
const TP_CM_BAM: u8 = 32;
const TP_CM_RTS: u8 = 16;
const TP_CM_ABORT: u8 = 255;
/// Max slots for in-flight ISO TP sessions — one per source, so 16
/// covers a very active bus.
const ISO_TP_SLOTS: usize = 16;
/// Absolute upper bound on a TP payload — 255 packets × 7 bytes.
/// Real-world PGN 129540 with many sats is well under 500 bytes.
const ISO_TP_MAX_SIZE: usize = 255 * BUCKET_N_SIZE;

/// What the reassembler emitted in response to a `push` call.
#[derive(Debug, PartialEq, Eq)]
pub enum Reassembled {
    /// PGN is not fast-packet, or the frame was already coalesced. The
    /// frame is returned unchanged.
    PassThrough(RawFrame),
    /// Fast-packet reassembly is complete; the coalesced payload is
    /// ready for decoding.
    Complete(RawFrame),
    /// More frames needed. The reassembler holds state internally.
    Partial,
    /// A protocol error was detected (sequence drift, missing frame 0,
    /// buffer overflow). The frame is dropped from the in-flight slot.
    Error(ReassemblyError),
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum ReassemblyError {
    #[error("fast-packet frame received with empty data")]
    EmptyData,
    #[error("no reassembly slot available; dropped pgn {pgn} src {src}")]
    OutOfSlots { pgn: u32, src: u8 },
}

/// Bare minimum slice of PGN-database state the reassembler needs.
/// Kept narrow so the reassembler doesn't link to the full database
/// and the caller can mock it freely in tests.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FramePacketType {
    /// Single-frame PGN — pass through unchanged.
    Single,
    /// Fast-packet PGN — reassemble across frames.
    Fast,
    /// Unknown PGN or anything else — pass through.
    Other,
}

#[derive(Debug, Clone)]
struct Slot {
    used: bool,
    pgn: u32,
    src: u8,
    seq: u8,
    /// Bitmask of frame indices received so far.
    frames: u32,
    /// Bitmask of frame indices required to complete.
    all_frames: u32,
    /// Total payload size from frame 0's length byte.
    size: usize,
    /// Sequence number in which the slot was claimed (monotonic). Used
    /// to evict the oldest slot when all slots are in use.
    age: u64,
    data: [u8; FASTPACKET_MAX_SIZE],
}

impl Default for Slot {
    fn default() -> Self {
        Self {
            used: false,
            pgn: 0,
            src: 0,
            seq: 0,
            frames: 0,
            all_frames: 0,
            size: 0,
            age: 0,
            data: [0u8; FASTPACKET_MAX_SIZE],
        }
    }
}

/// Render `bits` as a comma-separated list of set bit positions
/// (e.g. `0b00101101` → `"0,2,3,5"`). Used to make the
/// incomplete-fast-packet warning legible.
fn describe_frames(bits: u32) -> String {
    let mut out = String::new();
    let mut first = true;
    for i in 0..=FASTPACKET_MAX_INDEX {
        if bits & (1u32 << i) != 0 {
            if !first {
                out.push(',');
            }
            out.push_str(&i.to_string());
            first = false;
        }
    }
    if out.is_empty() {
        out.push_str("none");
    }
    out
}

/// Format the "what was expected but missing" half of the
/// incomplete-fast-packet warning. `all_frames=0` means we never saw
/// frame 0, so the total count is still unknown.
fn describe_expectation(all_frames: u32, size: usize, frames: u32) -> String {
    if all_frames == 0 {
        return ", declared total size unknown (frame 0 not yet received)".to_string();
    }
    let missing = all_frames & !frames;
    let total = all_frames.count_ones();
    if missing == 0 {
        format!(
            ", expected {} frame{} (declared size {} bytes), all of them present",
            total,
            if total == 1 { "" } else { "s" },
            size,
        )
    } else {
        format!(
            ", expected {} frame{} (declared size {} bytes), still missing {{{}}}",
            total,
            if total == 1 { "" } else { "s" },
            size,
            describe_frames(missing),
        )
    }
}

/// One in-flight ISO TP session (BAM or RTS). Keyed on source
/// address — the spec allows one transfer per source at a time.
#[derive(Debug, Clone, Default)]
struct TpSlot {
    used: bool,
    src: u8,
    dst: u8,
    prio: u8,
    target_pgn: u32,
    /// Bytes declared by the CM frame; final payload is truncated to
    /// this when we emit the reassembled `RawFrame`.
    total_size: usize,
    /// Packet count declared by the CM frame.
    packets: u8,
    /// Bitmask of received sequence numbers, bit `n` = seq `n+1`.
    received: [u64; 4],
    /// ISO timestamp of the CM frame — reused on the synthesized
    /// reassembled frame so downstream ordering is stable.
    timestamp: Option<String>,
    /// Payload buffer sized to the spec's ceiling. Vec so a stalled
    /// session doesn't cost 1.7 kB per unused slot.
    data: Vec<u8>,
    /// Monotonic age for oldest-out eviction, same convention as
    /// the fast-packet slots.
    age: u64,
}

/// Fast-packet + ISO Transport Protocol reassembler.
pub struct Reassembler {
    slots: Vec<Slot>,
    tp_slots: Vec<TpSlot>,
    next_age: u64,
    tp_next_age: u64,
}

impl Default for Reassembler {
    fn default() -> Self {
        Self::new()
    }
}

impl Reassembler {
    pub fn new() -> Self {
        Self {
            slots: vec![Slot::default(); REASSEMBLY_BUFFER_SIZE],
            tp_slots: vec![TpSlot::default(); ISO_TP_SLOTS],
            next_age: 0,
            tp_next_age: 0,
        }
    }

    /// Push one CAN frame.
    ///
    /// `packet_type` should come from the PGN database. For unknown
    /// PGNs pass `FramePacketType::Other` — the frame will pass through
    /// untouched.
    pub fn push(&mut self, frame: RawFrame, packet_type: FramePacketType) -> Reassembled {
        // ISO Transport Protocol frames are single-frame PGNs on the
        // wire but they're plumbing for a larger virtual payload.
        // Swallow them: hand back `Partial` while accumulating, and
        // `Complete` with a synthesized target-PGN frame on the
        // last TP.DT of a session.
        if frame.pgn == PGN_ISO_TP_CM {
            return self.handle_tp_cm(frame);
        }
        if frame.pgn == PGN_ISO_TP_DT {
            return self.handle_tp_dt(frame);
        }

        // Coalesced payloads (len > 8) and non-fast-packet PGNs are
        // returned verbatim. This matches the C analyzer's gate.
        if frame.data.len() > 8 || packet_type != FramePacketType::Fast {
            return Reassembled::PassThrough(frame);
        }
        if frame.data.is_empty() {
            return Reassembled::Error(ReassemblyError::EmptyData);
        }

        let header = frame.data[0];
        let frame_index = (header & 0x1f) as u32;
        let seq = header & 0xe0;

        // Find an existing slot for (pgn, src, seq) — or claim a new one.
        let slot_idx = match self.find_slot(frame.pgn, frame.src, seq) {
            Some(i) => i,
            None => match self.claim_slot(frame.pgn, frame.src, seq) {
                Some(i) => i,
                None => {
                    return Reassembled::Error(ReassemblyError::OutOfSlots {
                        pgn: frame.pgn,
                        src: frame.src,
                    });
                }
            },
        };

        // Two situations mean the assembly in the slot belongs to an
        // earlier message and must be restarted with this frame:
        //
        // * **A duplicate frame index.** The wrapped sequence counter
        //   reused this (pgn, src, seq) after the previous burst lost
        //   frames (analyzer.c:811-815 logs and resets the bitmask the
        //   same way). Keeping `size` / `all_frames` is harmless:
        //   completion requires frame 0's bit, and frame 0 always
        //   refreshes them (below) before setting it.
        //
        // * **Frame 0 that would complete instantly off held bits.**
        //   Frame 0 with earlier indices already in the slot is
        //   ambiguous: an out-of-order retransmission (YDWG delivers
        //   frames reordered; canboat's recombine-frames fixture
        //   requires those to survive) looks exactly like the body of
        //   a previous burst whose frame 0 was lost. The tell is
        //   completion: a genuinely reordered burst still has frames
        //   in flight, so it completes on a later index, while stale
        //   leftovers satisfy the mask the moment frame 0 lands —
        //   emitting a payload gluing this frame 0 onto the previous
        //   message's body, and (when the sizes differ) wedging the
        //   slot with `frames ⊋ all_frames` so nothing ever completes
        //   again. So: if the held bits plus frame 0 would satisfy the
        //   mask this frame declares, discard them as stale; otherwise
        //   keep them. (`all_frames == 1` is exempt — a ≤6-byte
        //   payload legitimately completes on frame 0 alone, and uses
        //   no held data.)
        let stale = self.slots[slot_idx].frames;
        let duplicate = stale & (1u32 << frame_index) != 0;

        // Frame 0 carries the total payload size in data[1]; compute
        // the mask of frame indices it declares as required.
        let new_all_frames = (frame_index == 0).then(|| {
            let declared = frame.data.get(1).copied().unwrap_or(0) as usize;
            let size = declared.min(FASTPACKET_MAX_SIZE);
            // canboat: number of frames needed = 1 + size/7 (integer
            // division). Frame 0 holds 6 bytes; each later frame holds
            // 7. The integer-truncated formula gives the right count
            // because frame 0's extra byte effectively borrows from the
            // first chunk of 7.
            let needed = (1 + size / BUCKET_N_SIZE).min((FASTPACKET_MAX_INDEX as usize) + 1);
            let all_frames = match needed {
                0 => 0,
                n if n >= 32 => u32::MAX,
                n => (1u32 << n) - 1,
            };
            (size, all_frames)
        });
        let completes_instantly = new_all_frames.is_some_and(|(_, mask)| {
            !duplicate && stale != 0 && mask != 1 && (stale | 1) & mask == mask
        });

        if duplicate || completes_instantly {
            let slot = &self.slots[slot_idx];
            log::warn!(
                "Incomplete fast packet pgn={} src={} seq=0x{:02x}: \
                 frame index {} {} before the sequence completed. \
                 Already received frame{} {{{}}}{}. Restarting assembly with this frame.",
                frame.pgn,
                frame.src,
                seq,
                frame_index,
                if duplicate {
                    "arrived again"
                } else {
                    "started a new message"
                },
                if slot.frames.count_ones() == 1 {
                    ""
                } else {
                    "s"
                },
                describe_frames(slot.frames),
                describe_expectation(slot.all_frames, slot.size, slot.frames),
            );
            self.slots[slot_idx].frames = 0;
        }

        if let Some((size, all_frames)) = new_all_frames {
            self.slots[slot_idx].size = size;
            self.slots[slot_idx].all_frames = all_frames;
        }

        // Copy this frame's payload into the assembled buffer at the
        // correct offset. Missing trailing bytes are padded with 0xFF
        // to match what canboat does when a frame is truncated.
        let (dst_off, src_off, bucket_len) = if frame_index == 0 {
            (0usize, BUCKET_0_OFFSET, BUCKET_0_SIZE)
        } else {
            let idx = BUCKET_0_SIZE + (frame_index as usize - 1) * BUCKET_N_SIZE;
            (idx, BUCKET_N_OFFSET, BUCKET_N_SIZE)
        };

        let slot = &mut self.slots[slot_idx];
        // The bucket may run off the end of FASTPACKET_MAX_SIZE for
        // pathological inputs (frame index 31 on a small declared size).
        let end = (dst_off + bucket_len).min(FASTPACKET_MAX_SIZE);
        let bucket_capacity = end.saturating_sub(dst_off);

        let available = frame.data.len().saturating_sub(src_off);
        let copy_len = available.min(bucket_capacity);
        if copy_len > 0 {
            slot.data[dst_off..dst_off + copy_len]
                .copy_from_slice(&frame.data[src_off..src_off + copy_len]);
        }
        if copy_len < bucket_capacity {
            for byte in &mut slot.data[dst_off + copy_len..end] {
                *byte = 0xff;
            }
        }
        slot.frames |= 1u32 << frame_index;

        // Complete?
        if slot.all_frames != 0 && slot.frames == slot.all_frames {
            let size = slot.size;
            let mut data: smallvec::SmallVec<[u8; 8]> = smallvec::SmallVec::with_capacity(size);
            data.extend_from_slice(&slot.data[..size]);
            let reassembled = RawFrame {
                timestamp: frame.timestamp,
                prio: frame.prio,
                pgn: frame.pgn,
                src: frame.src,
                dst: frame.dst,
                data,
            };
            // Free the slot.
            slot.used = false;
            slot.frames = 0;
            slot.size = 0;
            return Reassembled::Complete(reassembled);
        }

        Reassembled::Partial
    }

    /// Handle a PGN 60416 TP.CM frame. `BAM` and `RTS` open a fresh
    /// TP session for this source; `Abort` closes one down; the
    /// other control bytes (CTS / EOM) are peer responses to an
    /// RTS session we, as a monitor, don't participate in and are
    /// swallowed too.
    ///
    /// Never emits `PassThrough` — a CM frame is plumbing, never
    /// user-facing.
    fn handle_tp_cm(&mut self, frame: RawFrame) -> Reassembled {
        if frame.data.is_empty() {
            return Reassembled::Error(ReassemblyError::EmptyData);
        }
        let control = frame.data[0];
        match control {
            TP_CM_BAM | TP_CM_RTS => {}
            TP_CM_ABORT => {
                // Drop any in-flight session for this source.
                if let Some(i) = self.find_tp_slot(frame.src) {
                    self.tp_slots[i].used = false;
                }
                return Reassembled::Partial;
            }
            _ => return Reassembled::Partial,
        }
        // Need 8 CM bytes to decode. A short frame is malformed; log
        // and swallow rather than surface an error variant — the
        // reassembler's `Error` is reserved for out-of-slot / empty.
        if frame.data.len() < 8 {
            log::warn!(
                "ISO TP CM frame from src={} has {} bytes (need 8); dropping",
                frame.src,
                frame.data.len()
            );
            return Reassembled::Partial;
        }
        let total_size = u16::from_le_bytes([frame.data[1], frame.data[2]]) as usize;
        let packets = frame.data[3];
        let target_pgn =
            (frame.data[5] as u32) | ((frame.data[6] as u32) << 8) | ((frame.data[7] as u32) << 16);
        if packets == 0 || total_size == 0 || total_size > ISO_TP_MAX_SIZE {
            log::warn!(
                "ISO TP CM from src={} declares implausible size={} packets={}; dropping",
                frame.src,
                total_size,
                packets
            );
            return Reassembled::Partial;
        }
        let slot_idx = match self.claim_tp_slot(frame.src) {
            Some(i) => i,
            None => {
                return Reassembled::Error(ReassemblyError::OutOfSlots {
                    pgn: target_pgn,
                    src: frame.src,
                });
            }
        };
        let slot = &mut self.tp_slots[slot_idx];
        slot.dst = frame.dst;
        slot.prio = frame.prio;
        slot.target_pgn = target_pgn;
        slot.total_size = total_size;
        slot.packets = packets;
        slot.received = [0; 4];
        slot.timestamp = frame.timestamp;
        slot.data.clear();
        slot.data.resize(total_size, 0xff);
        Reassembled::Partial
    }

    /// Handle a PGN 60160 TP.DT frame. Copies the 7-byte chunk into
    /// the matching session's buffer at offset `(seq - 1) * 7`;
    /// emits `Complete` with a synthesized target-PGN frame the
    /// moment every declared sequence has been seen.
    ///
    /// A DT frame with no matching open session is silently swallowed
    /// (we may have missed the CM) — better than surfacing a
    /// misleading pass-through for a frame the user never wants to
    /// see anyway.
    fn handle_tp_dt(&mut self, frame: RawFrame) -> Reassembled {
        if frame.data.is_empty() {
            return Reassembled::Error(ReassemblyError::EmptyData);
        }
        let Some(slot_idx) = self.find_tp_slot(frame.src) else {
            log::debug!(
                "ISO TP DT from src={} with no open session; dropping",
                frame.src
            );
            return Reassembled::Partial;
        };
        let slot = &mut self.tp_slots[slot_idx];
        let sequence = frame.data[0];
        if sequence == 0 || sequence > slot.packets {
            log::warn!(
                "ISO TP DT src={} seq={} out of range 1..={}; dropping",
                frame.src,
                sequence,
                slot.packets
            );
            return Reassembled::Partial;
        }
        // 1-based sequence → 0-based bit position → chunk offset.
        let seq_zero_based = (sequence - 1) as usize;
        let bit_word = seq_zero_based / 64;
        let bit_slot = seq_zero_based % 64;
        slot.received[bit_word] |= 1u64 << bit_slot;
        let offset = seq_zero_based * BUCKET_N_SIZE;
        let end = (offset + BUCKET_N_SIZE).min(slot.total_size);
        let copy_len = end.saturating_sub(offset);
        let avail = frame.data.len().saturating_sub(1);
        let n = copy_len.min(avail);
        if n > 0 {
            slot.data[offset..offset + n].copy_from_slice(&frame.data[1..1 + n]);
        }

        // Complete iff every declared sequence has arrived.
        let packets = slot.packets as usize;
        let full_words = packets / 64;
        let partial_bits = packets % 64;
        let all_full = slot.received[..full_words].iter().all(|w| *w == u64::MAX);
        let last_ok = if partial_bits == 0 {
            true
        } else {
            let mask = (1u64 << partial_bits) - 1;
            slot.received[full_words] & mask == mask
        };
        if !(all_full && last_ok) {
            return Reassembled::Partial;
        }

        let mut data: smallvec::SmallVec<[u8; 8]> =
            smallvec::SmallVec::with_capacity(slot.total_size);
        data.extend_from_slice(&slot.data[..slot.total_size]);
        let reassembled = RawFrame {
            timestamp: slot.timestamp.take(),
            prio: slot.prio,
            pgn: slot.target_pgn,
            src: frame.src,
            dst: slot.dst,
            data,
        };
        slot.used = false;
        Reassembled::Complete(reassembled)
    }

    fn find_tp_slot(&self, src: u8) -> Option<usize> {
        self.tp_slots.iter().position(|s| s.used && s.src == src)
    }

    /// Reuse or reset a slot for `src`. Prefers an existing slot for
    /// this source (matching the "one session per src" invariant);
    /// otherwise picks a free slot, else evicts the oldest.
    fn claim_tp_slot(&mut self, src: u8) -> Option<usize> {
        if let Some(existing) = self.find_tp_slot(src) {
            self.tp_next_age = self.tp_next_age.wrapping_add(1);
            let age = self.tp_next_age;
            self.tp_slots[existing].src = src;
            self.tp_slots[existing].age = age;
            self.tp_slots[existing].used = true;
            return Some(existing);
        }
        let free = self.tp_slots.iter().position(|s| !s.used);
        let idx = match free {
            Some(i) => i,
            None => self
                .tp_slots
                .iter()
                .enumerate()
                .min_by_key(|(_, s)| s.age)
                .map(|(i, _)| i)?,
        };
        self.tp_next_age = self.tp_next_age.wrapping_add(1);
        let age = self.tp_next_age;
        self.tp_slots[idx] = TpSlot {
            used: true,
            src,
            age,
            ..TpSlot::default()
        };
        Some(idx)
    }

    fn find_slot(&self, pgn: u32, src: u8, seq: u8) -> Option<usize> {
        self.slots
            .iter()
            .position(|s| s.used && s.pgn == pgn && s.src == src && s.seq == seq)
    }

    fn claim_slot(&mut self, pgn: u32, src: u8, seq: u8) -> Option<usize> {
        // Prefer an unused slot.
        let free = self.slots.iter().position(|s| !s.used);
        let idx = match free {
            Some(i) => i,
            None => {
                // Evict the oldest in-use slot. (canboat just drops the
                // frame; we make best effort to keep going by reusing
                // the slot least-recently claimed.)
                self.slots
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, s)| s.age)
                    .map(|(i, _)| i)?
            }
        };
        let age = self.next_age;
        self.next_age = self.next_age.wrapping_add(1);
        self.slots[idx] = Slot {
            used: true,
            pgn,
            src,
            seq,
            frames: 0,
            all_frames: 0,
            size: 0,
            age,
            data: [0u8; FASTPACKET_MAX_SIZE],
        };
        Some(idx)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::smallvec;

    #[test]
    fn describe_frames_renders_set_bits_in_order() {
        assert_eq!(describe_frames(0), "none");
        assert_eq!(describe_frames(0b1), "0");
        assert_eq!(describe_frames(0b101101), "0,2,3,5");
        assert_eq!(describe_frames(0b111111), "0,1,2,3,4,5");
    }

    #[test]
    fn describe_expectation_says_unknown_before_frame0() {
        let s = describe_expectation(0, 0, 0);
        assert!(s.contains("unknown"), "got: {s}");
    }

    #[test]
    fn describe_expectation_lists_missing_indices() {
        // all_frames = 0..5, seen = 0,1,2,3,5 → missing {4}, size 35
        let s = describe_expectation(0b111111, 35, 0b101111);
        assert!(s.contains("expected 6 frames"), "got: {s}");
        assert!(s.contains("declared size 35"), "got: {s}");
        assert!(s.contains("missing {4}"), "got: {s}");
    }

    fn frame(pgn: u32, src: u8, data: Vec<u8>) -> RawFrame {
        RawFrame {
            timestamp: None,
            prio: 3,
            pgn,
            src,
            dst: 255,
            data: data.into(),
        }
    }

    #[test]
    fn passes_through_single_frame_pgn() {
        let mut r = Reassembler::new();
        let f = frame(60928, 0, vec![1, 2, 3, 4, 5, 6, 7, 8]);
        match r.push(f.clone(), FramePacketType::Single) {
            Reassembled::PassThrough(g) => assert_eq!(g, f),
            other => panic!("expected pass-through, got {other:?}"),
        }
    }

    #[test]
    fn passes_through_coalesced_payload() {
        let mut r = Reassembler::new();
        // Already-coalesced 14-byte payload: should pass through even
        // if the PGN is fast-packet.
        let f = frame(129029, 0, (0..14u8).collect());
        match r.push(f.clone(), FramePacketType::Fast) {
            Reassembled::PassThrough(g) => assert_eq!(g, f),
            other => panic!("expected pass-through, got {other:?}"),
        }
    }

    #[test]
    fn reassembles_two_frame_fast_packet() {
        let mut r = Reassembler::new();
        // Total payload 9 bytes: frame 0 carries 6, frame 1 carries 3.
        // Frame 0 header: seq=0, index=0 → 0x00; size byte = 9; then 6 payload bytes.
        let f0 = frame(129029, 0, vec![0x00, 9, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5]);
        match r.push(f0, FramePacketType::Fast) {
            Reassembled::Partial => (),
            other => panic!("expected partial, got {other:?}"),
        }
        // Frame 1 header: seq=0, index=1 → 0x01; then 7 payload bytes (last 4 padded FF).
        let f1 = frame(
            129029,
            0,
            vec![0x01, 0xa6, 0xa7, 0xa8, 0xff, 0xff, 0xff, 0xff],
        );
        let expected_payload: smallvec::SmallVec<[u8; 8]> =
            smallvec![0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8];
        match r.push(f1, FramePacketType::Fast) {
            Reassembled::Complete(g) => {
                assert_eq!(g.pgn, 129029);
                assert_eq!(g.data, expected_payload);
            }
            other => panic!("expected complete, got {other:?}"),
        }
    }

    #[test]
    fn separates_sequences_per_pgn_src_seq() {
        let mut r = Reassembler::new();
        // Two interleaved sequences with different seq numbers should
        // not corrupt each other.
        // Sequence A: seq=0x00
        let a0 = frame(129029, 0, vec![0x00, 7, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5]);
        // Sequence B: seq=0x20
        let b0 = frame(129029, 0, vec![0x20, 7, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5]);
        assert_eq!(r.push(a0, FramePacketType::Fast), Reassembled::Partial);
        assert_eq!(r.push(b0, FramePacketType::Fast), Reassembled::Partial);
        let a1 = frame(
            129029,
            0,
            vec![0x01, 0xa6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
        );
        let b1 = frame(
            129029,
            0,
            vec![0x21, 0xb6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
        );
        let a = match r.push(a1, FramePacketType::Fast) {
            Reassembled::Complete(g) => g,
            other => panic!("a: expected complete, got {other:?}"),
        };
        let b = match r.push(b1, FramePacketType::Fast) {
            Reassembled::Complete(g) => g,
            other => panic!("b: expected complete, got {other:?}"),
        };
        assert_eq!(&a.data[..], &[0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6]);
        assert_eq!(&b.data[..], &[0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6]);
    }

    /// Build the CAN frames of one fast-packet burst: `seq` in the
    /// top three bits, declared payload size `size`, every payload
    /// byte set to `fill` so tests can tell bursts apart.
    fn burst(pgn: u32, src: u8, seq: u8, size: usize, fill: u8) -> Vec<RawFrame> {
        let n = 1 + size / BUCKET_N_SIZE;
        (0..n)
            .map(|i| {
                let mut d = vec![seq | i as u8];
                if i == 0 {
                    d.push(size as u8);
                    d.extend(std::iter::repeat_n(fill, 6));
                } else {
                    d.extend(std::iter::repeat_n(fill, 7));
                }
                frame(pgn, src, d)
            })
            .collect()
    }

    /// Push all frames, asserting none of them completes a message.
    fn push_partial(r: &mut Reassembler, frames: &[RawFrame]) {
        for f in frames {
            assert_eq!(
                r.push(f.clone(), FramePacketType::Fast),
                Reassembled::Partial,
                "unexpected result for frame {:02x}",
                f.data[0],
            );
        }
    }

    /// The SCX-20 wedge: a burst whose frame 0 was lost leaves stale
    /// index bits in the slot; when the wrapped sequence counter
    /// reuses the slot, the next message must still come out intact.
    /// (Previously the stale bits made `frames` a strict superset of
    /// `all_frames`, so nothing ever completed and every following
    /// frame logged an "Incomplete fast packet" warning.)
    #[test]
    fn recovers_when_previous_burst_lost_frame_zero() {
        let mut r = Reassembler::new();
        // Burst A: 115 bytes → 17 frames, but frame 0 never arrives.
        let a = burst(130845, 52, 0xe0, 115, 0xaa);
        push_partial(&mut r, &a[1..]);
        // Burst B: 103 bytes → 15 frames, same (pgn, src, seq), complete.
        let b = burst(130845, 52, 0xe0, 103, 0xbb);
        push_partial(&mut r, &b[..14]);
        match r.push(b[14].clone(), FramePacketType::Fast) {
            Reassembled::Complete(g) => {
                assert_eq!(g.data.len(), 103);
                assert!(g.data.iter().all(|&x| x == 0xbb), "payload mixes bursts");
            }
            other => panic!("expected complete, got {other:?}"),
        }
    }

    /// A lost tail frame must cost exactly one message: the next
    /// frame 0 on the same (pgn, src, seq) restarts the slot.
    #[test]
    fn recovers_when_previous_burst_lost_tail_frame() {
        let mut r = Reassembler::new();
        // Burst A: 17 frames, last one lost.
        let a = burst(130845, 52, 0xe0, 115, 0xaa);
        push_partial(&mut r, &a[..16]);
        // Burst B: complete 15-frame message.
        let b = burst(130845, 52, 0xe0, 103, 0xbb);
        push_partial(&mut r, &b[..14]);
        match r.push(b[14].clone(), FramePacketType::Fast) {
            Reassembled::Complete(g) => {
                assert_eq!(g.data.len(), 103);
                assert!(g.data.iter().all(|&x| x == 0xbb), "payload mixes bursts");
            }
            other => panic!("expected complete, got {other:?}"),
        }
    }

    /// Frame 0 always starts a fresh message — it must never combine
    /// with buckets left over from an earlier burst. (Previously,
    /// when burst B lost its frame 0, burst C's frame 0 completed
    /// immediately against B's buckets and emitted a payload mixing
    /// the two.)
    #[test]
    fn never_emits_payload_mixing_two_bursts() {
        let mut r = Reassembler::new();
        // Burst A: 15 frames, tail frame lost.
        let a = burst(130845, 52, 0xe0, 103, 0xaa);
        push_partial(&mut r, &a[..14]);
        // Burst B: same shape, frame 0 lost.
        let b = burst(130845, 52, 0xe0, 103, 0xbb);
        push_partial(&mut r, &b[1..]);
        // Burst C: complete. Its frame 0 lands on a slot holding B's
        // buckets 1..=14 — completing here would emit C's first six
        // bytes glued to B's ninety-seven.
        let c = burst(130845, 52, 0xe0, 103, 0xcc);
        push_partial(&mut r, &c[..14]);
        match r.push(c[14].clone(), FramePacketType::Fast) {
            Reassembled::Complete(g) => {
                assert_eq!(g.data.len(), 103);
                assert!(g.data.iter().all(|&x| x == 0xcc), "payload mixes bursts");
            }
            other => panic!("expected complete, got {other:?}"),
        }
    }

    /// Mirror of canboat's `recombine-frames.in` fourth group: an
    /// out-of-order burst missing one frame, followed by a full
    /// out-of-order retransmission whose frame 0 arrives *after* one
    /// of its body frames. The held pre-frame-0 frame must survive
    /// the frame 0 restart logic so the retransmission completes.
    #[test]
    fn keeps_reordered_frames_that_precede_frame_zero() {
        let mut r = Reassembler::new();
        let b = burst(129029, 0, 0x00, 43, 0xbb); // 7 frames
        // First transmission, out of order, frame 1 never arrives.
        for i in [0usize, 2, 6, 3, 4, 5] {
            assert_eq!(
                r.push(b[i].clone(), FramePacketType::Fast),
                Reassembled::Partial
            );
        }
        // Retransmission, out of order: frame 2 lands before frame 0
        // (frame 2 is a duplicate → restart; frame 0 must then keep it).
        for i in [2usize, 0, 6, 3, 4, 5] {
            assert_eq!(
                r.push(b[i].clone(), FramePacketType::Fast),
                Reassembled::Partial,
                "frame {i} should not complete yet"
            );
        }
        match r.push(b[1].clone(), FramePacketType::Fast) {
            Reassembled::Complete(g) => {
                assert_eq!(g.data.len(), 43);
                assert!(g.data.iter().all(|&x| x == 0xbb));
            }
            other => panic!("expected complete, got {other:?}"),
        }
    }

    /// Parse one PLAIN-format line `<ts>,<prio>,<pgn>,<src>,<dst>,
    /// <len>,<hex>,...` into a `RawFrame`. Convenience for exercising
    /// the reassembler with real captures.
    fn parse_plain(line: &str) -> RawFrame {
        let parts: Vec<&str> = line.split(',').collect();
        let prio: u8 = parts[1].parse().unwrap();
        let pgn: u32 = parts[2].parse().unwrap();
        let src: u8 = parts[3].parse().unwrap();
        let dst: u8 = parts[4].parse().unwrap();
        let len: usize = parts[5].parse().unwrap();
        let data: smallvec::SmallVec<[u8; 8]> = parts[6..6 + len]
            .iter()
            .map(|b| u8::from_str_radix(b, 16).unwrap())
            .collect();
        RawFrame {
            timestamp: Some(parts[0].to_string()),
            prio,
            pgn,
            src,
            dst,
            data,
        }
    }

    /// A real 32-packet BAM sequence captured off a Navico NAC-3
    /// broadcasting PGN 129540 — `~/src/rev/navico/nac3_nav_mode.raw`
    /// (larger payload than fast-packet's 223-byte ceiling, so the
    /// device falls back to ISO Transport Protocol).
    const NAC3_TP_CM: &str = "2026-06-30T23:30:33.151Z,6,60416,23,255,8,20,db,00,20,ff,04,fa,01";
    const NAC3_TP_DT: &[&str] = &[
        "2026-06-30T23:30:33.216Z,6,60160,23,255,8,01,aa,ff,12,08,39,0a,7e",
        "2026-06-30T23:30:33.291Z,6,60160,23,255,8,02,61,1c,0c,00,00,00,00",
        "2026-06-30T23:30:33.369Z,6,60160,23,255,8,03,02,09,dc,17,5a,79,1c",
        "2026-06-30T23:30:33.450Z,6,60160,23,255,8,04,0c,00,00,00,00,02,0e",
        "2026-06-30T23:30:33.514Z,6,60160,23,255,8,05,22,15,08,a7,f0,0a,00",
        "2026-06-30T23:30:33.601Z,6,60160,23,255,8,06,00,00,00,00,42,73,23",
        "2026-06-30T23:30:33.667Z,6,60160,23,255,8,07,72,7d,fc,08,00,00,00",
        "2026-06-30T23:30:33.744Z,6,60160,23,255,8,08,00,10,01,e8,19,7f,25",
        "2026-06-30T23:30:33.815Z,6,60160,23,255,8,09,98,08,00,00,00,00,00",
        "2026-06-30T23:30:33.890Z,6,60160,23,255,8,0a,02,5c,10,e7,37,98,08",
        "2026-06-30T23:30:33.968Z,6,60160,23,255,8,0b,00,00,00,00,00,13,f4",
        "2026-06-30T23:30:34.044Z,6,60160,23,255,8,0c,0c,87,db,98,08,00,00",
        "2026-06-30T23:30:34.114Z,6,60160,23,255,8,0d,00,00,00,16,d1,15,65",
        "2026-06-30T23:30:34.198Z,6,60160,23,255,8,0e,b7,34,08,00,00,00,00",
        "2026-06-30T23:30:34.271Z,6,60160,23,255,8,0f,00,11,22,24,e4,dc,d0",
        "2026-06-30T23:30:34.344Z,6,60160,23,255,8,10,07,00,00,00,00,00,04",
        "2026-06-30T23:30:34.416Z,6,60160,23,255,8,11,73,23,ac,5a,d0,07,00",
        "2026-06-30T23:30:34.495Z,6,60160,23,255,8,12,00,00,00,02,4e,2e,08",
        "2026-06-30T23:30:34.569Z,6,60160,23,255,8,13,65,a8,d0,07,00,00,00",
        "2026-06-30T23:30:34.661Z,6,60160,23,255,8,14,00,10,03,16,22,45,0c",
        "2026-06-30T23:30:34.721Z,6,60160,23,255,8,15,6c,07,00,00,00,00,02",
        "2026-06-30T23:30:34.791Z,6,60160,23,255,8,16,06,7f,07,2a,cb,08,07",
        "2026-06-30T23:30:34.868Z,6,60160,23,255,8,17,00,00,00,00,00,41,ad",
        "2026-06-30T23:30:34.941Z,6,60160,23,255,8,18,2d,73,23,40,06,00,00",
        "2026-06-30T23:30:35.022Z,6,60160,23,255,8,19,00,00,10,58,5c,1f,dc",
        "2026-06-30T23:30:35.090Z,6,60160,23,255,8,1a,08,40,06,00,00,00,00",
        "2026-06-30T23:30:35.170Z,6,60160,23,255,8,1b,10,48,e8,0a,16,13,00",
        "2026-06-30T23:30:35.252Z,6,60160,23,255,8,1c,00,00,00,00,00,10,4f",
        "2026-06-30T23:30:35.316Z,6,60160,23,255,8,1d,74,05,1f,c9,00,00,00",
        "2026-06-30T23:30:35.391Z,6,60160,23,255,8,1e,00,00,00,10,51,dc,08",
        "2026-06-30T23:30:35.464Z,6,60160,23,255,8,1f,87,db,00,00,00,00,00",
        "2026-06-30T23:30:35.539Z,6,60160,23,255,8,20,00,10,ff,ff,ff,ff,ff",
    ];

    #[test]
    fn iso_tp_bam_reassembles_nac3_pgn_129540() {
        let mut r = Reassembler::new();
        // CM frame is single-frame PGN 60416; packet_type is
        // `Single` from the schema perspective, but the reassembler
        // intercepts it before the pass-through gate.
        let cm = parse_plain(NAC3_TP_CM);
        assert_eq!(
            r.push(cm, FramePacketType::Single),
            Reassembled::Partial,
            "CM frame must never emit pass-through — it's plumbing"
        );

        // 31 partial DTs, then Complete on the 32nd.
        for (i, line) in NAC3_TP_DT.iter().enumerate() {
            let dt = parse_plain(line);
            let got = r.push(dt, FramePacketType::Single);
            if i + 1 < NAC3_TP_DT.len() {
                assert_eq!(
                    got,
                    Reassembled::Partial,
                    "DT seq {} expected Partial",
                    i + 1
                );
            } else {
                match got {
                    Reassembled::Complete(frame) => {
                        assert_eq!(frame.pgn, 129540, "target PGN");
                        assert_eq!(frame.src, 23);
                        assert_eq!(frame.dst, 255);
                        assert_eq!(frame.data.len(), 0xdb, "declared 219 bytes");
                        // First byte of the assembled payload is
                        // seq-1's B1 (offset 0). Sanity check.
                        assert_eq!(frame.data[0], 0xaa);
                        // Last valid byte is at offset 218; frame's
                        // final chunk is `00,10,ff,ff,ff,ff,ff` so
                        // offset 217 = 0x00, 218 = 0x10.
                        assert_eq!(frame.data[218], 0x10);
                    }
                    other => panic!("last DT expected Complete, got {other:?}"),
                }
            }
        }
    }

    #[test]
    fn iso_tp_abort_cm_frees_slot() {
        let mut r = Reassembler::new();
        let cm = parse_plain(NAC3_TP_CM);
        assert_eq!(r.push(cm, FramePacketType::Single), Reassembled::Partial);
        // A subsequent CM with control=255 (Abort) drops the slot.
        // Force one by hand.
        let abort = RawFrame {
            timestamp: Some("2026-06-30T23:30:33.500Z".to_string()),
            prio: 6,
            pgn: 60416,
            src: 23,
            dst: 255,
            data: smallvec![255, 0, 0, 0, 0, 0, 0, 0],
        };
        assert_eq!(r.push(abort, FramePacketType::Single), Reassembled::Partial);
        // A DT arriving after the abort must not resurrect the slot
        // (we no longer have an open session for src 23).
        let dt = parse_plain(NAC3_TP_DT[0]);
        assert_eq!(r.push(dt, FramePacketType::Single), Reassembled::Partial);
    }

    #[test]
    fn iso_tp_dt_without_open_session_is_swallowed() {
        let mut r = Reassembler::new();
        // No prior CM — DT with no matching session must not blow
        // up and must not surface as PassThrough.
        let dt = parse_plain(NAC3_TP_DT[0]);
        assert_eq!(r.push(dt, FramePacketType::Single), Reassembled::Partial);
    }

    #[test]
    fn duplicate_frame_resets_slot_and_keeps_processing() {
        // Canboat treats a duplicate frame index as "the previous
        // partial was lost"; it logs an incomplete-fast-packet warning,
        // zeroes the bitmask and treats the new frame as the start of
        // a fresh attempt within the same slot. Verify the new frame
        // ends up placed (i.e., the slot still says it's partial, not
        // an error variant).
        let mut r = Reassembler::new();
        let f0 = frame(
            129029,
            0,
            vec![0x00, 14, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5],
        );
        assert_eq!(
            r.push(f0.clone(), FramePacketType::Fast),
            Reassembled::Partial
        );
        // Duplicate frame index — must not return an Error variant.
        assert_eq!(r.push(f0, FramePacketType::Fast), Reassembled::Partial);
    }
}
