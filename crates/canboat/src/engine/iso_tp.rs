// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Sending a message over the ISO Transport Protocol (ISO 11783-3 /
//! SAE J1939-21), sans I/O.
//!
//! A payload of 9 to 1785 bytes goes out as a TP.CM (PGN 60416)
//! announcement followed by TP.DT (PGN 60160) data packets of 7 bytes
//! each, numbered from 1, the last padded with 0xFF:
//!
//! * **BAM** (to global): the CM is a Broadcast Announce and the data
//!   packets follow at [`BAM_GAP_MS`] intervals; nobody answers.
//! * **RTS/CTS** (to one address): the CM is a Request To Send; the
//!   receiver answers Clear To Send with how many packets it wants and
//!   from which number, we send that window, it asks again, and when it
//!   has them all it acknowledges with End Of Message. A receiver that
//!   goes quiet for [`T3_MS`], or holds with CTS(0) for more than
//!   [`T4_MS`], gets an Abort.
//!
//! The NMEA 2000 tables reassemble ISO TP as well, but NMEA 2000 sends
//! long messages as fast-packets; this is J1939's way.
//!
//! [`TpSender`] is driven by the caller's clock: [`TpSender::send`]
//! starts a transfer, [`TpSender::on_frame`] feeds it the TP.CM frames
//! the bus delivers, and [`TpSender::poll`] hands back whatever is due.
//! Each returns the frames to put on the bus; [`TpSender::next_deadline`]
//! says when to poll next. Only one transfer runs per (source,
//! destination) pair, as the protocol requires; later ones queue.

use std::collections::VecDeque;

use crate::engine::{ADDR_GLOBAL, RawFrame};

/// TP.CM, connection management.
pub const PGN_TP_CM: u32 = 60416;
/// TP.DT, data transfer.
pub const PGN_TP_DT: u32 = 60160;

/// The largest message ISO TP carries: 255 packets of 7 bytes.
pub const TP_MAX_SIZE: usize = 1785;

/// J1939-21 wants 50 to 200 ms between the packets of a BAM.
pub const BAM_GAP_MS: u64 = 50;
/// How long we wait for a CTS or an End Of Message Acknowledgement.
pub const T3_MS: u64 = 1250;
/// How long a receiver may hold the transfer with CTS(0).
pub const T4_MS: u64 = 1050;

/// The priority J1939-21 gives TP.CM and TP.DT.
const TP_PRIORITY: u8 = 7;

const CM_RTS: u8 = 16;
const CM_CTS: u8 = 17;
const CM_EOMA: u8 = 19;
const CM_BAM: u8 = 32;
const CM_ABORT: u8 = 255;

/// Abort reason: the sender needed its resources for another task.
const ABORT_RESOURCES: u8 = 2;
/// Abort reason: a timeout occurred.
const ABORT_TIMEOUT: u8 = 3;
/// Abort reason: bad sequence number (a CTS asked for a packet the
/// message does not have).
const ABORT_BAD_SEQUENCE: u8 = 7;

/// Transfers waiting behind a running one, per (source, destination).
const MAX_QUEUED: usize = 16;

/// Why a message could not be sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TpError {
    /// Up to 8 bytes fit one frame; ISO TP is for 9 to 1785.
    Size(usize),
    /// Too many transfers already wait for this (source, destination).
    QueueFull,
}

impl std::fmt::Display for TpError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TpError::Size(n) => write!(f, "{n} bytes; ISO TP carries 9 to {TP_MAX_SIZE}"),
            TpError::QueueFull => write!(f, "too many ISO TP transfers queued"),
        }
    }
}

impl std::error::Error for TpError {}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// BAM: data packet `seq` goes out at `at`.
    BamNext { at: u64, seq: u8 },
    /// RTS: waiting for a CTS (or an EOMA after the last window).
    AwaitCts { deadline: u64 },
    /// RTS: a CTS asked for packets `from..=to`; they go out on the
    /// next poll.
    SendWindow { from: u8, to: u8 },
    /// RTS: everything is sent; waiting for the EOMA.
    AwaitEoma { deadline: u64 },
}

#[derive(Debug, Clone)]
struct Transfer {
    pgn: u32,
    src: u8,
    dst: u8,
    data: Vec<u8>,
    state: State,
}

impl Transfer {
    fn packets(&self) -> u8 {
        self.data.len().div_ceil(7) as u8
    }

    fn is_bam(&self) -> bool {
        self.dst == ADDR_GLOBAL
    }

    fn pgn_bytes(&self) -> [u8; 3] {
        let p = self.pgn.to_le_bytes();
        [p[0], p[1], p[2]]
    }

    /// The announcing TP.CM: BAM to global, RTS to one address.
    fn announce(&self) -> RawFrame {
        let size = (self.data.len() as u16).to_le_bytes();
        let [p0, p1, p2] = self.pgn_bytes();
        let control = if self.is_bam() { CM_BAM } else { CM_RTS };
        // Byte 4: reserved (0xFF) for BAM; for RTS the most packets one
        // CTS may ask for, 0xFF meaning no limit.
        frame(
            PGN_TP_CM,
            self.src,
            self.dst,
            [control, size[0], size[1], self.packets(), 0xFF, p0, p1, p2],
        )
    }

    /// Data packet `seq` (1-based), padded with 0xFF.
    fn data_packet(&self, seq: u8) -> RawFrame {
        let start = (seq as usize - 1) * 7;
        let chunk = &self.data[start..(start + 7).min(self.data.len())];
        let mut payload = [0xFF; 8];
        payload[0] = seq;
        payload[1..=chunk.len()].copy_from_slice(chunk);
        frame(PGN_TP_DT, self.src, self.dst, payload)
    }

    fn abort(&self, reason: u8) -> RawFrame {
        let [p0, p1, p2] = self.pgn_bytes();
        frame(
            PGN_TP_CM,
            self.src,
            self.dst,
            [CM_ABORT, reason, 0xFF, 0xFF, 0xFF, p0, p1, p2],
        )
    }
}

fn frame(pgn: u32, src: u8, dst: u8, data: [u8; 8]) -> RawFrame {
    RawFrame::new(None, TP_PRIORITY, pgn, src, dst, data)
}

/// The sending half of ISO TP; see the module documentation.
#[derive(Debug, Default)]
pub struct TpSender {
    /// Running transfers, at most one per (source, destination).
    active: Vec<Transfer>,
    /// Transfers waiting for theirs to finish, oldest first.
    queued: VecDeque<Transfer>,
}

impl TpSender {
    pub fn new() -> Self {
        Self::default()
    }

    /// Start sending `data` as `pgn` from `src` to `dst` (global for a
    /// BAM). Returns the frames due now: the announcement, or nothing
    /// when an earlier transfer between the same pair is still running.
    pub fn send(
        &mut self,
        now: u64,
        pgn: u32,
        src: u8,
        dst: u8,
        data: &[u8],
    ) -> Result<Vec<RawFrame>, TpError> {
        if !(9..=TP_MAX_SIZE).contains(&data.len()) {
            return Err(TpError::Size(data.len()));
        }
        let t = Transfer {
            pgn,
            src,
            dst,
            data: data.to_vec(),
            state: State::AwaitCts { deadline: 0 },
        };
        if self.find(src, dst).is_some() {
            let waiting = self
                .queued
                .iter()
                .filter(|q| q.src == src && q.dst == dst)
                .count();
            if waiting >= MAX_QUEUED {
                return Err(TpError::QueueFull);
            }
            self.queued.push_back(t);
            return Ok(Vec::new());
        }
        Ok(self.start(now, t))
    }

    fn start(&mut self, now: u64, mut t: Transfer) -> Vec<RawFrame> {
        t.state = if t.is_bam() {
            State::BamNext {
                at: now + BAM_GAP_MS,
                seq: 1,
            }
        } else {
            State::AwaitCts {
                deadline: now + T3_MS,
            }
        };
        let announce = t.announce();
        self.active.push(t);
        vec![announce]
    }

    fn find(&self, src: u8, dst: u8) -> Option<usize> {
        self.active
            .iter()
            .position(|t| t.src == src && t.dst == dst)
    }

    /// End transfer `i` and start the next one queued for its pair.
    fn finish(&mut self, now: u64, i: usize) -> Vec<RawFrame> {
        let done = self.active.swap_remove(i);
        match self
            .queued
            .iter()
            .position(|q| q.src == done.src && q.dst == done.dst)
        {
            Some(q) => {
                let next = self.queued.remove(q).expect("position is in range");
                self.start(now, next)
            }
            None => Vec::new(),
        }
    }

    /// Feed a TP.CM frame from the bus. CTS, EOMA and Abort sent *to* a
    /// transfer's source *by* its destination drive that transfer;
    /// everything else is ignored. Returns frames due now.
    pub fn on_frame(&mut self, now: u64, f: &RawFrame) -> Vec<RawFrame> {
        if f.pgn != PGN_TP_CM || f.data.len() < 8 {
            return Vec::new();
        }
        let pgn = u32::from_le_bytes([f.data[5], f.data[6], f.data[7], 0]);
        // Our transfer from f.dst to f.src, for that PGN.
        let Some(i) = self
            .active
            .iter()
            .position(|t| t.src == f.dst && t.dst == f.src && t.pgn == pgn && !t.is_bam())
        else {
            return Vec::new();
        };
        let packets = self.active[i].packets();
        match f.data[0] {
            CM_CTS => {
                let (count, next) = (f.data[1], f.data[2]);
                let t = &mut self.active[i];
                if count == 0 {
                    // Hold: the receiver will ask again.
                    t.state = State::AwaitCts {
                        deadline: now + T4_MS,
                    };
                } else if next == 0 || next > packets {
                    let abort = t.abort(ABORT_BAD_SEQUENCE);
                    let mut out = vec![abort];
                    out.extend(self.finish(now, i));
                    return out;
                } else {
                    let to = (next as u16 + count as u16 - 1).min(packets as u16) as u8;
                    t.state = State::SendWindow { from: next, to };
                }
                Vec::new()
            }
            CM_EOMA => {
                // Only once every packet is out, and only for this
                // message: an early or mismatched acknowledgement must
                // not end the transfer (and start the next) before the
                // receiver has it all. T3 still aborts a receiver that
                // never sends the right one.
                let t = &self.active[i];
                let size = u16::from_le_bytes([f.data[1], f.data[2]]) as usize;
                if matches!(t.state, State::AwaitEoma { .. })
                    && size == t.data.len()
                    && f.data[3] == packets
                {
                    self.finish(now, i)
                } else {
                    Vec::new()
                }
            }
            CM_ABORT => self.finish(now, i),
            _ => Vec::new(),
        }
    }

    /// Frames due at `now`: BAM data packets whose time has come, an
    /// RTS window a CTS asked for, and Aborts for receivers that timed
    /// out.
    pub fn poll(&mut self, now: u64) -> Vec<RawFrame> {
        let mut out = Vec::new();
        let mut i = 0;
        while i < self.active.len() {
            let packets = self.active[i].packets();
            let t = &mut self.active[i];
            match t.state {
                State::BamNext { at, seq } if now >= at => {
                    out.push(t.data_packet(seq));
                    if seq == packets {
                        out.extend(self.finish(now, i));
                        continue;
                    }
                    // Spaced from this packet, not from the schedule, so
                    // a late poll never bunches the rest up.
                    t.state = State::BamNext {
                        at: now + BAM_GAP_MS,
                        seq: seq + 1,
                    };
                }
                State::SendWindow { from, to } => {
                    for seq in from..=to {
                        out.push(t.data_packet(seq));
                    }
                    t.state = if to == packets {
                        State::AwaitEoma {
                            deadline: now + T3_MS,
                        }
                    } else {
                        State::AwaitCts {
                            deadline: now + T3_MS,
                        }
                    };
                }
                State::AwaitCts { deadline } | State::AwaitEoma { deadline } if now >= deadline => {
                    out.push(t.abort(ABORT_TIMEOUT));
                    out.extend(self.finish(now, i));
                    continue;
                }
                _ => {}
            }
            i += 1;
        }
        out
    }

    /// When [`Self::poll`] next has something to do; `None` when idle
    /// apart from frames the bus has to deliver first.
    pub fn next_deadline(&self) -> Option<u64> {
        self.active
            .iter()
            .map(|t| match t.state {
                State::BamNext { at, .. } => at,
                State::SendWindow { .. } => 0,
                State::AwaitCts { deadline } | State::AwaitEoma { deadline } => deadline,
            })
            .min()
    }

    /// Whether any transfer is running or queued.
    pub fn is_idle(&self) -> bool {
        self.active.is_empty() && self.queued.is_empty()
    }

    /// Give up on everything: drop the queue, and return an Abort for
    /// each running RTS/CTS transfer so its receiver need not wait out
    /// its own timeout. A BAM has nobody to tell; it just stops.
    pub fn abort_all(&mut self) -> Vec<RawFrame> {
        self.queued.clear();
        self.active
            .drain(..)
            .filter(|t| !t.is_bam())
            .map(|t| t.abort(ABORT_RESOURCES))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SRC: u8 = 0x21;
    const DST: u8 = 0x00;

    fn payload(n: usize) -> Vec<u8> {
        (1..=n as u32).map(|v| v as u8).collect()
    }

    fn cm(f: &RawFrame) -> [u8; 8] {
        assert_eq!(f.pgn, PGN_TP_CM);
        f.data.as_slice().try_into().unwrap()
    }

    #[test]
    fn rejects_what_needs_no_transport() {
        let mut tp = TpSender::new();
        assert_eq!(
            tp.send(0, 65259, SRC, 255, &payload(8)),
            Err(TpError::Size(8))
        );
        assert_eq!(
            tp.send(0, 65259, SRC, 255, &payload(1786)),
            Err(TpError::Size(1786))
        );
        assert!(tp.is_idle());
    }

    /// A 16-byte BAM: the announcement now, then three packets 50 ms
    /// apart, the last padded with 0xFF.
    #[test]
    fn bam_announces_then_paces_the_packets() {
        let mut tp = TpSender::new();
        let out = tp.send(1000, 0x1FF45, SRC, 255, &payload(16)).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].dst, 255);
        assert_eq!(out[0].prio, 7);
        assert_eq!(cm(&out[0]), [32, 16, 0, 3, 0xFF, 0x45, 0xFF, 0x01]);

        assert!(tp.poll(1049).is_empty(), "not before the gap");
        assert_eq!(tp.next_deadline(), Some(1050));
        let p1 = tp.poll(1050);
        assert_eq!(p1.len(), 1);
        assert_eq!(p1[0].pgn, PGN_TP_DT);
        assert_eq!(p1[0].data.as_slice(), &[1, 1, 2, 3, 4, 5, 6, 7]);
        assert!(tp.poll(1060).is_empty());
        assert_eq!(
            tp.poll(1100)[0].data.as_slice(),
            &[2, 8, 9, 10, 11, 12, 13, 14]
        );
        assert_eq!(
            tp.poll(1150)[0].data.as_slice(),
            &[3, 15, 16, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        );
        assert!(tp.is_idle());
    }

    /// The whole RTS/CTS conversation: RTS, CTS for two, the two
    /// packets, CTS for the last one, it, EOMA.
    #[test]
    fn rts_sends_the_windows_the_receiver_asks_for() {
        let mut tp = TpSender::new();
        let pgn = 0x00_DA00;
        let out = tp.send(0, pgn, SRC, DST, &payload(20)).unwrap();
        assert_eq!(out[0].dst, DST);
        assert_eq!(cm(&out[0]), [16, 20, 0, 3, 0xFF, 0x00, 0xDA, 0x00]);
        assert!(tp.poll(10).is_empty(), "nothing until CTS");

        let cts = |count, next| {
            RawFrame::new(
                None,
                7,
                PGN_TP_CM,
                DST,
                SRC,
                [17, count, next, 0xFF, 0xFF, 0x00, 0xDA, 0x00],
            )
        };
        assert!(tp.on_frame(20, &cts(2, 1)).is_empty());
        let w1 = tp.poll(20);
        assert_eq!(w1.iter().map(|f| f.data[0]).collect::<Vec<_>>(), [1, 2]);
        assert!(w1.iter().all(|f| f.dst == DST && f.pgn == PGN_TP_DT));

        tp.on_frame(30, &cts(5, 3)); // asks beyond the end: just the last
        let w2 = tp.poll(30);
        assert_eq!(w2.len(), 1);
        assert_eq!(w2[0].data.as_slice(), &[3, 15, 16, 17, 18, 19, 20, 0xFF]);
        assert!(!tp.is_idle(), "waits for EOMA");

        let eoma = RawFrame::new(
            None,
            7,
            PGN_TP_CM,
            DST,
            SRC,
            [19, 20, 0, 3, 0xFF, 0x00, 0xDA, 0x00],
        );
        tp.on_frame(40, &eoma);
        assert!(tp.is_idle());
    }

    /// A receiver that never answers gets an Abort (reason 3) after T3.
    #[test]
    fn a_silent_receiver_is_aborted() {
        let mut tp = TpSender::new();
        tp.send(0, 0xDA00, SRC, DST, &payload(10)).unwrap();
        assert!(tp.poll(T3_MS - 1).is_empty());
        let out = tp.poll(T3_MS);
        assert_eq!(cm(&out[0])[..2], [255, 3]);
        assert!(tp.is_idle());
    }

    /// CTS(0) holds the transfer for up to T4 rather than T3.
    #[test]
    fn cts_zero_holds() {
        let mut tp = TpSender::new();
        tp.send(0, 0xDA00, SRC, DST, &payload(10)).unwrap();
        let hold = RawFrame::new(
            None,
            7,
            PGN_TP_CM,
            DST,
            SRC,
            [17, 0, 0, 0xFF, 0xFF, 0x00, 0xDA, 0x00],
        );
        tp.on_frame(100, &hold);
        assert_eq!(tp.next_deadline(), Some(100 + T4_MS));
    }

    /// An End Of Message Acknowledgement before the last window, or for
    /// a different size or packet count, does not end the transfer.
    #[test]
    fn only_a_matching_eoma_after_the_last_packet_completes() {
        let mut tp = TpSender::new();
        tp.send(0, 0xDA00, SRC, DST, &payload(20)).unwrap();
        tp.send(0, 0xDA00, SRC, DST, &payload(10)).unwrap(); // queued behind it
        let cm_from_peer = |b: [u8; 5]| {
            RawFrame::new(
                None,
                7,
                PGN_TP_CM,
                DST,
                SRC,
                [b[0], b[1], b[2], b[3], b[4], 0x00, 0xDA, 0x00],
            )
        };
        // Early: nothing sent yet.
        assert!(
            tp.on_frame(1, &cm_from_peer([19, 20, 0, 3, 0xFF]))
                .is_empty()
        );
        tp.on_frame(2, &cm_from_peer([17, 3, 1, 0xFF, 0xFF]));
        assert_eq!(tp.poll(2).len(), 3, "the whole message");
        // Wrong size, then wrong packet count: still waiting.
        assert!(
            tp.on_frame(3, &cm_from_peer([19, 21, 0, 3, 0xFF]))
                .is_empty()
        );
        assert!(
            tp.on_frame(4, &cm_from_peer([19, 20, 0, 2, 0xFF]))
                .is_empty()
        );
        assert_eq!(
            tp.next_deadline(),
            Some(2 + T3_MS),
            "still the first transfer"
        );
        // The right one completes it and starts the queued message.
        let next = tp.on_frame(5, &cm_from_peer([19, 20, 0, 3, 0xFF]));
        assert_eq!(
            cm(&next[0])[..4],
            [16, 10, 0, 2],
            "RTS for the queued 10 bytes"
        );
    }

    /// Giving up aborts running RTS/CTS transfers (reason 2), stops a
    /// BAM silently, and empties the queue.
    #[test]
    fn abort_all_tells_the_receivers() {
        let mut tp = TpSender::new();
        tp.send(0, 0xDA00, SRC, DST, &payload(10)).unwrap();
        tp.send(0, 0xDB00, SRC, DST, &payload(10)).unwrap(); // queued
        tp.send(0, 0x1FF45, SRC, 255, &payload(10)).unwrap();
        let out = tp.abort_all();
        assert_eq!(out.len(), 1, "one RTS to abort, none for the BAM");
        assert_eq!(out[0].dst, DST);
        assert_eq!(cm(&out[0])[..2], [255, 2]);
        assert_eq!(cm(&out[0])[5..], [0x00, 0xDA, 0x00]);
        assert!(tp.is_idle());
    }

    /// The receiver's Abort ends the transfer quietly.
    #[test]
    fn the_receivers_abort_ends_it() {
        let mut tp = TpSender::new();
        tp.send(0, 0xDA00, SRC, DST, &payload(10)).unwrap();
        let abort = RawFrame::new(
            None,
            7,
            PGN_TP_CM,
            DST,
            SRC,
            [255, 1, 0xFF, 0xFF, 0xFF, 0x00, 0xDA, 0x00],
        );
        assert!(tp.on_frame(5, &abort).is_empty());
        assert!(tp.is_idle());
    }

    /// CM frames from someone else, or for another PGN, are not ours.
    #[test]
    fn ignores_other_conversations() {
        let mut tp = TpSender::new();
        tp.send(0, 0xDA00, SRC, DST, &payload(10)).unwrap();
        let other_pgn = RawFrame::new(
            None,
            7,
            PGN_TP_CM,
            DST,
            SRC,
            [17, 2, 1, 0xFF, 0xFF, 0x00, 0xDB, 0x00],
        );
        let other_node = RawFrame::new(
            None,
            7,
            PGN_TP_CM,
            0x33,
            SRC,
            [17, 2, 1, 0xFF, 0xFF, 0x00, 0xDA, 0x00],
        );
        tp.on_frame(1, &other_pgn);
        tp.on_frame(1, &other_node);
        assert!(tp.poll(2).is_empty());
    }

    /// One transfer per (source, destination): the second BAM waits for
    /// the first, then starts by itself.
    #[test]
    fn a_second_bam_from_the_same_source_queues() {
        let mut tp = TpSender::new();
        tp.send(0, 0x1FF45, SRC, 255, &payload(10)).unwrap();
        assert!(
            tp.send(0, 0x1FF46, SRC, 255, &payload(10))
                .unwrap()
                .is_empty()
        );
        assert_eq!(tp.poll(50).len(), 1);
        // The last packet of the first, then the second's announcement.
        let out = tp.poll(100);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].pgn, PGN_TP_DT);
        assert_eq!(cm(&out[1])[..1], [32]);
        assert_eq!(cm(&out[1])[5..], [0x46, 0xFF, 0x01]);
    }

    /// Every packet the sender emits is one the reassembler takes back
    /// into the original message.
    #[test]
    fn the_reassembler_reads_back_what_a_bam_sends() {
        use crate::engine::{FramePacketType, Reassembled, Reassembler};
        let data = payload(1785);
        let mut tp = TpSender::new();
        let mut frames = tp.send(0, 0x1FF45, SRC, 255, &data).unwrap();
        let mut now = 0;
        while !tp.is_idle() {
            now += BAM_GAP_MS;
            frames.extend(tp.poll(now));
        }
        assert_eq!(frames.len(), 1 + 255);
        let mut r = Reassembler::new();
        let mut got = None;
        for f in frames {
            if let Reassembled::Complete(m) = r.push(f, FramePacketType::Single) {
                got = Some(m);
            }
        }
        let m = got.expect("complete");
        assert_eq!(m.pgn, 0x1FF45);
        assert_eq!(m.data.as_slice(), data.as_slice());
    }
}
