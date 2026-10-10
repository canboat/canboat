// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The NGT-1's Transmit PGN Enable list: the PGNs the gateway will put on
//! the bus, and — for an addressable NGT-1 — the Transmit list it answers
//! PGN 126464 with. A PGN missing from it is silently not transmitted.
//!
//! The commands were first reverse engineered by canboatjs
//! (`lib/actisense-serial.ts`); the [Actisense SDK] now documents them.
//! Each goes out as an `NGT_MSG_SEND` (0xA1) BEM command and is answered by
//! an `NGT_MSG_RECEIVED` (0xA0) BEM response: the BEM id, a sequence byte,
//! the model id, the serial number and an error code (12 bytes), then the
//! command's data.
//!
//! | BEM    | SDK name                         | Sent                 | Answer data |
//! |--------|----------------------------------|----------------------|-------------|
//! | `0x40` | Get Supported PGN List           | —                    | parts, each: transfer id, structure variant `0x1100` (`u32`), N2K database version (`u16`), full size, first index, count, then per PGN its index and the PGN (`u24`) |
//! | `0x4f` | Get Tx PGN Enable List F2        | —                    | parts, each: transfer id, structure variant (`u32`). `0x1102`: full size, first index, count, then per PGN its index in the Supported PGN List, priority and rate (`u16`). `0x1103`: the proprietary PGNs, as two bitmaps of 0xff00‥0xffff and 0x1ff00‥0x1ffff, each a size then the bytes |
//! | `0x49` | Get Tx PGN Enable List F1        | —                    | four messages, sequence 1 to 4: the PGNs, their rates, their timeouts, their priorities; each a count, then `u32`s (`u8`s for the priorities) |
//! | `0x47` | Set Tx PGN Enable                | PGN `u32`, enable `1`, rate `0xfffffffe` (the PGN's default), timeout (ignored); no priority, so it is left as it is | PGN `u32`, enable, rate `u32`, timeout `u32`, priority |
//! | `0x01` | Commit To EEPROM                 | —                    | —           |
//! | `0x4b` | Activate PGN Enable Lists        | —                    | —           |
//!
//! [Actisense SDK]: https://github.com/Actisense/SDK/blob/main/docs/DataFormats/Binary/bem-detail/README.md
//!
//! The SDK marks F1 deprecated in favour of Get Tx PGN Enable List F2
//! (`0x4f`), from firmware v2.500. F2 lists indexes into the gateway's
//! Supported PGN List (`0x40`), so canboat reads both, on a gateway whose
//! Product Info names firmware 2.500 or later. It reads F1 when the
//! firmware is older or unknown, and when F2 goes unanswered or is
//! refused.
//!
//! Seen on an NGT-1-USB with firmware 2.690 (2026-10,
//! `samples/actisense-ngt1-fw2690.txt`):
//!
//! - F1 is incomplete. With 12 standard PGNs and 3 proprietary ones
//!   enabled, it lists 12: the first proprietary one takes the place of
//!   the last standard one, and the other two are missing. F2 lists all 15.
//! - The answers differ from the SDK's examples: the Supported PGN List's
//!   structure variant is `0x1100` and its transfer id 0, and its parts
//!   come highest index first, all with sequence 1. F2's proprietary part
//!   (sequence 2) comes before its standard part (sequence 1). So the
//!   parts are told apart by their structure variant, and put in place by
//!   their first index.
//!
//! Seen on an NGT-1-A, firmware not known (2026-09):
//!
//! - **The read is truncated on a long list**: with 17 PGNs enabled it
//!   reported 14, with 23 it reported 13 (the lowest ones). A PGN past that
//!   point looks missing although it is enabled.
//! - `0x47`'s answer has sequence 1 whatever happens; the outcome is the
//!   error code: 0 added, −996 already on the list (see the truncated
//!   read), −997 refused (PGN 0x40000). The rates read back as `65535`
//!   ("non periodic") for all but 126993 Heartbeat's `60000`.
//! - A PDU1 PGN is stored with its low (destination) byte cleared: enabling
//!   PGN 1 enables PGN 0.
//!
//! The SDK says current firmware saves a Set Tx PGN Enable at once, and
//! still accepts Commit To EEPROM and Activate; canboat sends both, as the
//! NGT-1 predates that.
//!
//! Hence: nothing is saved unless an enable actually added a PGN, and a
//! run remembers what the gateway confirmed ([`TxListRecord`]): a PGN the
//! read cannot show but the gateway has, a refused one, and one already
//! saved. A reconnect therefore never repeats an EEPROM write, while a
//! session cut off before its save is simply retried.
//!
//! canboatjs stopped doing this by default in 2020 ("this is possibly
//! causing issues", canboatjs#136). So this runs only when the embedder
//! names Transmit PGNs, reads the list first, and writes — enable, save,
//! activate — only when a PGN is missing, so a device already set up is
//! never written to again.

use std::sync::{Arc, Mutex};

pub use crate::engine::format::ngt1::NGT_MSG_RECEIVED;
use crate::engine::format::ngt1::{
    BEM_OPERATING_MODE, BemResponse, NGT_MSG_SEND, describe_error, encode_ngt_message,
};

/// BEM Commit To EEPROM.
const BEM_COMMIT_TO_EEPROM: u8 = 0x01;
/// BEM Get Supported PGN List.
const BEM_SUPPORTED_PGN_LIST: u8 = 0x40;
/// BEM Get / Set Tx PGN Enable.
const BEM_TX_PGN_ENABLE: u8 = 0x47;
/// BEM Get Tx PGN Enable List F1.
const BEM_TX_PGN_ENABLE_LIST_F1: u8 = 0x49;
/// BEM Activate PGN Enable Lists.
const BEM_ACTIVATE_PGN_ENABLE_LISTS: u8 = 0x4b;
/// BEM Get Tx PGN Enable List F2.
const BEM_TX_PGN_ENABLE_LIST_F2: u8 = 0x4f;

/// The first firmware with F2, × 1000.
const F2_FIRMWARE: u16 = 2_500;
/// The structure variant of a Supported PGN List part, as an NGT-1 sends
/// it (the SDK's example says 1).
const SV_SUPPORTED_PGN_LIST: u32 = 0x1100;
/// The structure variant of F2's standard PGNs.
const SV_TX_ENABLE_LIST: u32 = 0x1102;
/// The structure variant of F2's proprietary PGNs.
const SV_PROP_TX_ENABLE_LIST: u32 = 0x1103;

/// The F1 list's first message, by its sequence byte: the PGNs.
const F1_PGNS: u8 = 1;
/// The F1 list's last message (the priorities).
const F1_LAST: u8 = 4;
/// The sequence byte of a Set Tx PGN Enable answer.
const ENABLE_SEQUENCE: u8 = 1;

/// A Tx Rate of "the PGN's default".
const TX_RATE_DEFAULT: u32 = 0xffff_fffe;
/// The Tx Timeout of Set Tx PGN Enable, which the gateway ignores. canboat
/// has always sent this value.
const TX_TIMEOUT_IGNORED: u32 = 0xffff_fffe;

/// Wait after the gateway confirms startup before reading the list, as
/// canboatjs does.
const READ_DELAY_MS: u64 = 2_000;
/// How long to wait for any answer before trying again or giving up.
const ANSWER_TIMEOUT_MS: u64 = 10_000;
/// Reads of the list before giving up (an NGT-1 firmware that does not
/// know the command never answers).
const READ_ATTEMPTS: u32 = 3;

#[derive(Debug, PartialEq, Eq)]
enum State {
    /// Waiting for the gateway to confirm the startup command.
    AwaitStartup,
    /// Startup confirmed; read the list at `at`.
    ReadAt {
        at: u64,
    },
    /// F1 list requested; collecting its PGNs.
    Reading {
        have: Vec<u32>,
        attempt: u32,
    },
    /// Supported PGN List and F2 list requested; collecting their parts.
    ReadingF2 {
        list: F2List,
        attempt: u32,
    },
    /// Enabling the PGNs still in `todo`, the first one's answer pending;
    /// `added` are those the gateway added.
    Enabling {
        todo: Vec<u32>,
        added: Vec<u32>,
    },
    Committing {
        added: Vec<u32>,
    },
    Activating {
        added: Vec<u32>,
    },
    Done,
}

/// What this run has learned about the gateway's transmit list, shared by
/// its sessions (see [`TxListSync::new`]). Only what the gateway confirmed
/// goes in, so a session cut off halfway leaves nothing behind and the
/// next one simply tries again.
#[derive(Debug, Default)]
pub struct TxListRecord {
    /// Added and saved: the activate that followed was answered.
    saved: Vec<u32>,
    /// Answered "already enabled": there, though the (truncated) list
    /// read may not show it.
    present: Vec<u32>,
    /// Refused by the gateway.
    refused: Vec<u32>,
}

/// Brings the NGT-1's Transmit PGN Enable list up to `wanted`. Feed it the
/// gateway's `NGT_MSG_RECEIVED` payloads and the passing time; send the
/// byte strings it returns to the gateway.
pub struct TxListSync {
    wanted: Vec<u32>,
    state: State,
    /// When the pending answer times out.
    deadline: u64,
    /// What this run has learned, shared across sessions.
    record: Arc<Mutex<TxListRecord>>,
    /// The gateway's firmware version × 1000, once its Product Info is in.
    firmware: Option<u16>,
}

impl TxListSync {
    /// A sync for `wanted`, sharing `record` with the other sessions of
    /// this run; one that has nothing to do when `wanted` is empty.
    ///
    /// The record keeps a reconnect from writing again what is known: a
    /// PGN the read cannot show but the gateway has, a PGN it refuses, and
    /// — the gateway resetting to an older list, say — a PGN saved once
    /// already, which is then not saved a second time.
    pub fn new(wanted: Vec<u32>, record: Arc<Mutex<TxListRecord>>) -> Self {
        let state = if wanted.is_empty() {
            State::Done
        } else {
            State::AwaitStartup
        };
        Self {
            wanted,
            state,
            deadline: u64::MAX,
            record,
            firmware: None,
        }
    }

    /// The gateway's firmware version × 1000, from its Product Info: from
    /// 2.500 on the list is read in Format 2.
    pub fn set_firmware(&mut self, firmware: u16) {
        self.firmware = Some(firmware);
    }

    /// Start over, as after the gateway restarted: wait for it to answer
    /// the operating mode again, then read its list afresh.
    pub fn restart(&mut self) {
        *self = Self::new(std::mem::take(&mut self.wanted), Arc::clone(&self.record));
    }

    /// Whether the sync has finished, successfully or not.
    pub fn is_done(&self) -> bool {
        self.state == State::Done
    }

    /// Advance on an `NGT_MSG_RECEIVED` payload.
    pub fn on_message(&mut self, payload: &[u8], now: u64) -> Vec<Vec<u8>> {
        let (Some(&cmd), status) = (payload.first(), payload.get(1).copied()) else {
            return Vec::new();
        };
        match (&mut self.state, cmd) {
            // Any well-formed answer will do, even one refusing the mode:
            // the gateway is up, and its list does not depend on the mode.
            (State::AwaitStartup, BEM_OPERATING_MODE)
                if BemResponse::parse(payload).is_some_and(|r| r.operating_mode().is_some()) =>
            {
                self.state = State::ReadAt {
                    at: now + READ_DELAY_MS,
                };
                Vec::new()
            }
            (State::Reading { have, .. }, BEM_TX_PGN_ENABLE_LIST_F1) if status == Some(F1_PGNS) => {
                have.extend(list_pgns(payload));
                self.deadline = now + ANSWER_TIMEOUT_MS;
                Vec::new()
            }
            (State::Reading { have, .. }, BEM_TX_PGN_ENABLE_LIST_F1) if status == Some(F1_LAST) => {
                let have = std::mem::take(have);
                self.on_list(&have, now)
            }
            (State::ReadingF2 { list, .. }, BEM_SUPPORTED_PGN_LIST | BEM_TX_PGN_ENABLE_LIST_F2) => {
                let Some(answer) = BemResponse::parse(payload) else {
                    return Vec::new();
                };
                if answer.error != 0 {
                    log::info!(
                        "ngt1: the gateway refused BEM {cmd:#04x} (error {}); reading the transmit PGN list in Format 1",
                        describe_error(answer.error)
                    );
                    return self.read_f1(1, now);
                }
                if cmd == BEM_SUPPORTED_PGN_LIST {
                    list.add_supported(answer.data);
                } else {
                    list.add_enabled(answer.data);
                }
                self.deadline = now + ANSWER_TIMEOUT_MS;
                match list.pgns() {
                    Some(have) => self.on_list(&have, now),
                    None => Vec::new(),
                }
            }
            (State::Enabling { todo, added }, BEM_TX_PGN_ENABLE) => {
                let pgn = todo.remove(0);
                let result = enable_result(payload, status);
                if let Ok(mut record) = self.record.lock() {
                    match result {
                        Ok(Enabled::Now) if record.saved.contains(&pgn) => log::warn!(
                            "ngt1: the gateway lost transmit PGN {pgn}, saved earlier this run; \
                             not saving it again"
                        ),
                        Ok(Enabled::Now) => added.push(pgn),
                        // The list read is truncated on a long list, so a PGN
                        // can look missing while it is there.
                        Ok(Enabled::Already) => {
                            log::debug!("ngt1: transmit PGN {pgn} was already enabled");
                            record.present.push(pgn);
                        }
                        Err(why) => {
                            log::warn!("ngt1: the gateway refused transmit PGN {pgn} ({why})");
                            record.refused.push(pgn);
                        }
                    }
                }
                self.deadline = now + ANSWER_TIMEOUT_MS;
                match todo.first() {
                    Some(&next) => vec![enable_command(next)],
                    // Nothing added: no save, so no EEPROM write.
                    None if added.is_empty() => {
                        self.state = State::Done;
                        Vec::new()
                    }
                    None => {
                        let added = std::mem::take(added);
                        self.state = State::Committing { added };
                        vec![command(&[BEM_COMMIT_TO_EEPROM])]
                    }
                }
            }
            (State::Committing { added }, BEM_COMMIT_TO_EEPROM) => {
                let added = std::mem::take(added);
                self.state = State::Activating { added };
                self.deadline = now + ANSWER_TIMEOUT_MS;
                vec![command(&[BEM_ACTIVATE_PGN_ENABLE_LISTS])]
            }
            (State::Activating { added }, BEM_ACTIVATE_PGN_ENABLE_LISTS) => {
                log::info!("ngt1: transmit PGN list saved and active");
                if let Ok(mut record) = self.record.lock() {
                    record.saved.append(added);
                }
                self.state = State::Done;
                Vec::new()
            }
            _ => Vec::new(),
        }
    }

    /// The gateway's list is in: enable what `wanted` has and `have` lacks.
    fn on_list(&mut self, have: &[u32], now: u64) -> Vec<Vec<u8>> {
        log::debug!("ngt1: the gateway's transmit PGN list: {have:?}");
        let missing: Vec<u32> = self
            .wanted
            .iter()
            .copied()
            .filter(|pgn| !have.contains(pgn))
            .collect();
        if missing.is_empty() {
            log::info!("ngt1: transmit PGN list already has {:?}", self.wanted);
            self.state = State::Done;
            return Vec::new();
        }
        // Leave out what this run already knows about.
        let Ok(record) = self.record.lock() else {
            self.state = State::Done;
            return Vec::new();
        };
        let todo: Vec<u32> = missing
            .into_iter()
            .filter(|pgn| !record.present.contains(pgn) && !record.refused.contains(pgn))
            .collect();
        drop(record);
        if todo.is_empty() {
            self.state = State::Done;
            return Vec::new();
        }
        log::info!("ngt1: enabling transmit PGNs {todo:?}");
        let first = enable_command(todo[0]);
        self.state = State::Enabling {
            todo,
            added: Vec::new(),
        };
        self.deadline = now + ANSWER_TIMEOUT_MS;
        vec![first]
    }

    /// Advance on the passing of time: start the read once its delay is
    /// over, and retry or give up on an answer that does not come.
    pub fn on_tick(&mut self, now: u64) -> Vec<Vec<u8>> {
        match &self.state {
            State::ReadAt { at } if now >= *at => {
                if self.firmware.is_some_and(|f| f >= F2_FIRMWARE) {
                    self.read_f2(1, now)
                } else {
                    self.read_f1(1, now)
                }
            }
            State::ReadingF2 { attempt, .. } if now >= self.deadline => {
                if *attempt < READ_ATTEMPTS {
                    let attempt = attempt + 1;
                    self.read_f2(attempt, now)
                } else {
                    log::info!(
                        "ngt1: no answer to reading the transmit PGN list in Format 2; trying Format 1"
                    );
                    self.read_f1(1, now)
                }
            }
            State::Reading { attempt, .. } if now >= self.deadline => {
                if *attempt < READ_ATTEMPTS {
                    let attempt = attempt + 1;
                    self.read_f1(attempt, now)
                } else {
                    log::warn!(
                        "ngt1: no answer to reading the transmit PGN list; {:?} may not be transmitted",
                        self.wanted
                    );
                    self.state = State::Done;
                    Vec::new()
                }
            }
            State::Enabling { .. } | State::Committing { .. } | State::Activating { .. }
                if now >= self.deadline =>
            {
                log::warn!(
                    "ngt1: the gateway stopped answering while setting the transmit PGN list"
                );
                self.state = State::Done;
                Vec::new()
            }
            _ => Vec::new(),
        }
    }

    fn read_f1(&mut self, attempt: u32, now: u64) -> Vec<Vec<u8>> {
        self.state = State::Reading {
            have: Vec::new(),
            attempt,
        };
        self.deadline = now + ANSWER_TIMEOUT_MS;
        vec![command(&[BEM_TX_PGN_ENABLE_LIST_F1])]
    }

    fn read_f2(&mut self, attempt: u32, now: u64) -> Vec<Vec<u8>> {
        self.state = State::ReadingF2 {
            list: F2List::default(),
            attempt,
        };
        self.deadline = now + ANSWER_TIMEOUT_MS;
        vec![
            command(&[BEM_SUPPORTED_PGN_LIST]),
            command(&[BEM_TX_PGN_ENABLE_LIST_F2]),
        ]
    }
}

/// A list sent in parts, each naming the full size and its first index.
#[derive(Debug, PartialEq, Eq)]
struct Parts<T> {
    /// `None` until the first part gives the size.
    items: Option<Vec<Option<T>>>,
}

impl<T> Default for Parts<T> {
    fn default() -> Self {
        Self { items: None }
    }
}

impl<T: Copy> Parts<T> {
    /// Put `entries` in place from `first`; a part giving another size
    /// than the earlier ones starts the list over.
    fn add(&mut self, size: u8, first: u8, entries: impl IntoIterator<Item = T>) {
        let size = usize::from(size);
        let items = match &mut self.items {
            Some(items) if items.len() == size => items,
            other => other.insert(vec![None; size]),
        };
        for (slot, entry) in items.iter_mut().skip(usize::from(first)).zip(entries) {
            *slot = Some(entry);
        }
    }

    /// The whole list, once every entry is in.
    fn complete(&self) -> Option<Vec<T>> {
        self.items.as_ref()?.iter().copied().collect()
    }
}

/// The parts of a Format 2 read: the Supported PGN List, and the F2
/// list's standard and proprietary PGNs.
#[derive(Debug, Default, PartialEq, Eq)]
struct F2List {
    /// `(index, PGN)`.
    supported: Parts<(u8, u32)>,
    /// Indexes into the Supported PGN List.
    enabled: Parts<u8>,
    proprietary: Option<Vec<u32>>,
}

impl F2List {
    /// Take in a Supported PGN List answer's data: transfer id, structure
    /// variant, N2K database version, full size, first index, count, then
    /// per PGN its index and the PGN as a `u24`.
    fn add_supported(&mut self, data: &[u8]) {
        let [_, s0, s1, s2, s3, _, _, size, first, count, entries @ ..] = data else {
            return;
        };
        if u32::from_le_bytes([*s0, *s1, *s2, *s3]) != SV_SUPPORTED_PGN_LIST {
            return;
        }
        let entries = entries.as_chunks::<4>().0.iter().take(usize::from(*count));
        self.supported.add(
            *size,
            *first,
            entries.map(|[index, p0, p1, p2]| (*index, u32::from_le_bytes([*p0, *p1, *p2, 0]))),
        );
    }

    /// Take in an F2 answer's data: transfer id and structure variant,
    /// then either the standard PGNs (full size, first index, count, then
    /// per PGN its index, priority and rate) or the proprietary ones (two
    /// bitmaps, each a size then its bytes).
    fn add_enabled(&mut self, data: &[u8]) {
        let [_, s0, s1, s2, s3, rest @ ..] = data else {
            return;
        };
        match u32::from_le_bytes([*s0, *s1, *s2, *s3]) {
            SV_TX_ENABLE_LIST => {
                let [size, first, count, entries @ ..] = rest else {
                    return;
                };
                let entries = entries.as_chunks::<4>().0.iter().take(usize::from(*count));
                self.enabled.add(*size, *first, entries.map(|e| e[0]));
            }
            SV_PROP_TX_ENABLE_LIST => {
                let Some((dp0, rest)) = bitmap(rest) else {
                    return;
                };
                let Some((dp1, _)) = bitmap(rest) else {
                    return;
                };
                self.proprietary = Some(
                    bitmap_pgns(0xff00, dp0)
                        .chain(bitmap_pgns(0x1ff00, dp1))
                        .collect(),
                );
            }
            _ => {}
        }
    }

    /// The enabled PGNs, once every part is in. An index the Supported PGN
    /// List does not have is left out.
    fn pgns(&self) -> Option<Vec<u32>> {
        let supported = self.supported.complete()?;
        let enabled = self.enabled.complete()?;
        let proprietary = self.proprietary.as_ref()?;
        let mut pgns = Vec::with_capacity(enabled.len() + proprietary.len());
        for index in enabled {
            match supported.iter().find(|(i, _)| *i == index) {
                Some((_, pgn)) => pgns.push(*pgn),
                None => log::warn!(
                    "ngt1: transmit PGN index {index} is not in the gateway's Supported PGN List"
                ),
            }
        }
        pgns.extend_from_slice(proprietary);
        Some(pgns)
    }
}

/// A size byte and that many bytes; the rest.
fn bitmap(data: &[u8]) -> Option<(&[u8], &[u8])> {
    let (size, rest) = data.split_first()?;
    rest.split_at_checked(usize::from(*size))
}

/// The PGNs a proprietary bitmap enables: bit `n` is `base + n`, for the
/// 256 PGNs from `base`.
fn bitmap_pgns(base: u32, bitmap: &[u8]) -> impl Iterator<Item = u32> + '_ {
    bitmap
        .iter()
        .take(32)
        .enumerate()
        .flat_map(move |(byte, bits)| {
            (0..8)
                .filter(move |bit| bits >> bit & 1 != 0)
                .map(move |bit| base + (byte * 8 + bit) as u32)
        })
}

/// The PGNs in the F1 list's first message: a count at payload byte 12,
/// then the PGNs as `u32`s.
fn list_pgns(payload: &[u8]) -> Vec<u32> {
    let Some(&count) = payload.get(12) else {
        return Vec::new();
    };
    payload
        .get(13..)
        .unwrap_or(&[])
        .as_chunks::<4>()
        .0
        .iter()
        .take(count as usize)
        .map(|b| u32::from_le_bytes(*b))
        .collect()
}

/// What an accepted Set Tx PGN Enable did.
#[derive(Debug, PartialEq, Eq)]
enum Enabled {
    /// The PGN was added: the list changed and needs saving.
    Now,
    /// The PGN was on the list already: nothing changed.
    Already,
}

/// The Set Tx PGN Enable error code for a PGN already on the list.
const ALREADY_ENABLED: i32 = -996;

/// What a Set Tx PGN Enable answer says. Its sequence byte is 1 whatever
/// happened (on an NGT-1-A); the outcome is the error code: 0 added, −996
/// already on the list, other values refused (−997 for 0x40000).
fn enable_result(payload: &[u8], status: Option<u8>) -> Result<Enabled, String> {
    if status != Some(ENABLE_SEQUENCE) {
        return Err(format!("sequence {status:?}"));
    }
    match BemResponse::parse(payload) {
        Some(answer) => match answer.error {
            0 => Ok(Enabled::Now),
            ALREADY_ENABLED => Ok(Enabled::Already),
            code => Err(format!("error {}", describe_error(code))),
        },
        None => Err("short answer".into()),
    }
}

/// Set Tx PGN Enable: enable `pgn` at its default rate, leaving its
/// priority as it is.
fn enable_command(pgn: u32) -> Vec<u8> {
    let mut payload = vec![BEM_TX_PGN_ENABLE];
    payload.extend_from_slice(&pgn.to_le_bytes());
    payload.push(1);
    payload.extend_from_slice(&TX_RATE_DEFAULT.to_le_bytes());
    payload.extend_from_slice(&TX_TIMEOUT_IGNORED.to_le_bytes());
    command(&payload)
}

fn command(payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(payload.len() + 8);
    encode_ngt_message(NGT_MSG_SEND, payload, &mut out);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The F1 list's first message, carrying `pgns`.
    fn list_part(pgns: &[u32]) -> Vec<u8> {
        let mut p = vec![BEM_TX_PGN_ENABLE_LIST_F1, F1_PGNS];
        p.extend_from_slice(&[0; 10]);
        p.push(pgns.len() as u8);
        for pgn in pgns {
            p.extend_from_slice(&pgn.to_le_bytes());
        }
        p
    }

    /// A Set Tx PGN Enable answer as an NGT-1-A sends it: sequence 1 whatever the
    /// outcome, `result` at byte 8 (0 accepted), the PGN at byte 12.
    fn answer(pgn: u32, result: i32) -> Vec<u8> {
        let mut p = vec![BEM_TX_PGN_ENABLE, 1, 0x0e, 0x00, 0xac, 0x9f, 0x01, 0x00];
        p.extend_from_slice(&result.to_le_bytes());
        p.extend_from_slice(&pgn.to_le_bytes());
        p.extend_from_slice(&[0x01, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0x07, 0x9e]);
        p
    }

    /// The answers an NGT-1-A gave on hardware, all under status 1: PGN 0
    /// added; PGN 1 (the same PDU1 PGN as 0) already there, -996; PGN
    /// 0x40000 refused, -997.
    #[test]
    fn the_result_code_not_the_status_tells() {
        let added =
            hex("47 01 0e 00 ac 9f 01 00 00 00 00 00 00 00 00 00 01 ff ff 00 00 00 00 00 00 07 9e");
        let already =
            hex("47 01 0e 00 ac 9f 01 00 1c fc ff ff 01 00 00 00 01 ff ff 00 00 00 00 00 00 07 87");
        let refused =
            hex("47 01 0e 00 ac 9f 01 00 1b fc ff ff 00 00 04 00 01 fe ff ff ff 00 00 00 00 fe 91");
        let result = |p: &[u8]| enable_result(p, p.get(1).copied());
        assert_eq!(result(&added), Ok(Enabled::Now));
        assert_eq!(result(&already), Ok(Enabled::Already));
        assert_eq!(result(&refused), Err("error -997".to_string()));
    }

    /// A PGN the (truncated) list read missed but the gateway already has
    /// changes nothing, so nothing is saved.
    #[test]
    fn an_already_enabled_pgn_is_not_saved() {
        let mut s = started(vec![127508]);
        s.on_message(&[BEM_TX_PGN_ENABLE_LIST_F1, F1_LAST], 2_100);
        assert!(
            s.on_message(&answer(127508, ALREADY_ENABLED), 2_200)
                .is_empty()
        );
        assert!(s.is_done());
    }

    fn hex(s: &str) -> Vec<u8> {
        s.split(' ')
            .map(|b| u8::from_str_radix(b, 16).unwrap())
            .collect()
    }

    /// Walk a sync through startup up to the list request.
    /// The gateway's answer to Set Operating Mode: NGT Transfer Rx All Mode.
    const MODE_ANSWER: [u8; 14] = [
        BEM_OPERATING_MODE,
        1,
        0x0e,
        0x00,
        0xac,
        0x9f,
        0x01,
        0x00,
        0,
        0,
        0,
        0,
        0x02,
        0x00,
    ];

    /// A malformed answer to Set Operating Mode does not start the sync; a
    /// refusal does, as the gateway is up.
    #[test]
    fn only_a_well_formed_mode_answer_starts_the_sync() {
        let mut s = TxListSync::new(vec![127508], Arc::default());
        s.on_message(&[BEM_OPERATING_MODE, 1], 0);
        assert_eq!(s.state, State::AwaitStartup);
        let mut refused = MODE_ANSWER;
        refused[8..12].copy_from_slice(&(-1159i32).to_le_bytes());
        s.on_message(&refused, 0);
        assert!(matches!(s.state, State::ReadAt { .. }));
    }

    fn started(wanted: Vec<u32>) -> TxListSync {
        let mut s = TxListSync::new(wanted, Arc::default());
        assert!(s.on_message(&MODE_ANSWER, 0).is_empty());
        assert!(
            s.on_tick(READ_DELAY_MS - 1).is_empty(),
            "waits before reading"
        );
        assert_eq!(
            s.on_tick(READ_DELAY_MS),
            vec![command(&[BEM_TX_PGN_ENABLE_LIST_F1])]
        );
        s
    }

    #[test]
    fn nothing_wanted_means_nothing_sent() {
        let mut s = TxListSync::new(Vec::new(), Arc::default());
        assert!(s.is_done());
        assert!(s.on_message(&MODE_ANSWER, 0).is_empty());
        assert!(s.on_tick(u64::MAX).is_empty());
    }

    #[test]
    fn a_list_that_has_every_pgn_is_not_written() {
        let mut s = started(vec![127508, 127506]);
        assert!(s.on_message(&list_part(&[59392, 127508]), 2_100).is_empty());
        assert!(s.on_message(&list_part(&[127506]), 2_200).is_empty());
        assert!(
            s.on_message(&[BEM_TX_PGN_ENABLE_LIST_F1, F1_LAST], 2_300)
                .is_empty()
        );
        assert!(s.is_done());
    }

    #[test]
    fn missing_pgns_are_enabled_saved_and_activated() {
        let mut s = started(vec![127508, 127506, 127258]);
        s.on_message(&list_part(&[127506]), 2_100);
        assert_eq!(
            s.on_message(&[BEM_TX_PGN_ENABLE_LIST_F1, F1_LAST], 2_200),
            vec![enable_command(127508)]
        );
        assert_eq!(
            s.on_message(&answer(127508, 0), 2_300),
            vec![enable_command(127258)]
        );
        assert_eq!(
            s.on_message(&answer(127258, 0), 2_400),
            vec![command(&[BEM_COMMIT_TO_EEPROM])]
        );
        assert_eq!(
            s.on_message(&[BEM_COMMIT_TO_EEPROM], 2_500),
            vec![command(&[BEM_ACTIVATE_PGN_ENABLE_LISTS])]
        );
        assert!(
            s.on_message(&[BEM_ACTIVATE_PGN_ENABLE_LISTS], 2_600)
                .is_empty()
        );
        assert!(s.is_done());
    }

    /// The enable command's bytes, as canboatjs's `composeEnablePGN`
    /// builds them, before NGT-1 framing.
    #[test]
    fn the_enable_command_matches_canboatjs() {
        let mut expected = Vec::new();
        encode_ngt_message(
            NGT_MSG_SEND,
            &[
                0x47, 0x14, 0xf2, 0x01, 0x00, 0x01, 0xfe, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff,
            ],
            &mut expected,
        );
        assert_eq!(enable_command(127508), expected);
    }

    #[test]
    fn an_unanswered_read_is_retried_then_given_up() {
        let mut s = started(vec![127508]);
        let t = READ_DELAY_MS + ANSWER_TIMEOUT_MS;
        assert_eq!(s.on_tick(t), vec![command(&[BEM_TX_PGN_ENABLE_LIST_F1])]);
        assert_eq!(
            s.on_tick(t + ANSWER_TIMEOUT_MS),
            vec![command(&[BEM_TX_PGN_ENABLE_LIST_F1])]
        );
        assert!(s.on_tick(t + 2 * ANSWER_TIMEOUT_MS).is_empty());
        assert!(s.is_done());
    }

    /// The periodic Set Operating Mode keeps being answered; only the first
    /// confirmation starts the sync.
    #[test]
    fn later_startup_confirmations_are_ignored() {
        let mut s = started(vec![127508]);
        assert!(s.on_message(&MODE_ANSWER, 2_100).is_empty());
        assert!(matches!(s.state, State::Reading { .. }));
    }

    /// When the gateway refuses every PGN nothing changed, so nothing is
    /// saved: no EEPROM write.
    #[test]
    fn nothing_is_saved_when_every_pgn_is_refused() {
        let mut s = started(vec![127508]);
        s.on_message(&[BEM_TX_PGN_ENABLE_LIST_F1, F1_LAST], 2_100);
        assert!(s.on_message(&answer(127508, -996), 2_200).is_empty());
        assert!(s.is_done());
    }

    /// A session of this run, sharing `record`, run up to its list read
    /// answered with `have`.
    fn session(
        wanted: Vec<u32>,
        record: &Arc<Mutex<TxListRecord>>,
        have: &[u32],
    ) -> (TxListSync, Vec<Vec<u8>>) {
        let mut s = TxListSync::new(wanted, record.clone());
        s.on_message(&MODE_ANSWER, 0);
        s.on_tick(READ_DELAY_MS);
        s.on_message(&list_part(have), READ_DELAY_MS + 1);
        let sent = s.on_message(&[BEM_TX_PGN_ENABLE_LIST_F1, F1_LAST], READ_DELAY_MS + 2);
        (s, sent)
    }

    /// A PGN saved once this run and gone again (the gateway reset to an
    /// older list) is enabled but not saved a second time.
    #[test]
    fn a_pgn_is_saved_at_most_once_per_run() {
        let record = Arc::default();
        let (mut s, sent) = session(vec![127508], &record, &[]);
        assert_eq!(sent, vec![enable_command(127508)]);
        assert_eq!(
            s.on_message(&answer(127508, 0), 3_000),
            vec![command(&[BEM_COMMIT_TO_EEPROM])]
        );
        s.on_message(&[BEM_COMMIT_TO_EEPROM], 3_100);
        s.on_message(&[BEM_ACTIVATE_PGN_ENABLE_LISTS], 3_200);

        let (mut s, sent) = session(vec![127508], &record, &[]);
        assert_eq!(sent, vec![enable_command(127508)]);
        assert!(
            s.on_message(&answer(127508, 0), 3_000).is_empty(),
            "no second save"
        );
        assert!(s.is_done());
    }

    /// A session cut off before its save leaves nothing recorded: the next
    /// one tries again.
    #[test]
    fn an_interrupted_round_is_retried() {
        let record = Arc::default();
        let (mut s, _) = session(vec![127508], &record, &[]);
        s.on_message(&answer(127508, 0), 3_000); // commit sent, never answered
        let (mut s, sent) = session(vec![127508], &record, &[]);
        assert_eq!(sent, vec![enable_command(127508)]);
        assert_eq!(
            s.on_message(&answer(127508, 0), 3_000),
            vec![command(&[BEM_COMMIT_TO_EEPROM])]
        );
    }

    /// A PGN found already enabled (hidden by the truncated read) or
    /// refused is not enabled again by later sessions.
    #[test]
    fn what_is_known_is_not_enabled_again() {
        let record = Arc::default();
        let (mut s, _) = session(vec![127508, 127506], &record, &[]);
        s.on_message(&answer(127508, ALREADY_ENABLED), 3_000);
        s.on_message(&answer(127506, -997), 3_100);
        assert!(s.is_done(), "nothing added, nothing saved");
        let (s, sent) = session(vec![127508, 127506], &record, &[]);
        assert!(sent.is_empty());
        assert!(s.is_done());
    }

    #[test]
    fn a_refused_pgn_does_not_stop_the_rest() {
        let mut s = started(vec![127508, 127506]);
        s.on_message(&[BEM_TX_PGN_ENABLE_LIST_F1, F1_LAST], 2_100);
        assert_eq!(
            s.on_message(&answer(127508, -996), 2_200),
            vec![enable_command(127506)]
        );
    }

    /// An NGT-1-USB with firmware 2.690's answers to Get Supported PGN
    /// List, in the order it sent them (highest index first), from
    /// `samples/actisense-ngt1-fw2690.txt`.
    const SUPPORTED_2690: [&str; 4] = [
        "40 01 0e 00 ab b0 01 00 00 00 00 00 00 00 11 00 00 34 08 a3 90 13 90 14 fd 01 91 00 fe 01 92 07 fe 01 93 09 fe 01 94 0a fe 01 95 0b fe 01 96 0c fe 01 97 0d fe 01 98 0e fe 01 99 10 fe 01 9a 11 fe 01 9b 12 fe 01 9c 14 fe 01 9d 15 fe 01 9e 16 fe 01 9f 17 fe 01 a0 18 fe 01 a1 19 fe 01 a2 00 ff 01",
        "40 01 0e 00 ab b0 01 00 00 00 00 00 00 00 11 00 00 34 08 a3 60 30 60 02 fb 01 61 03 fb 01 62 04 fb 01 63 05 fb 01 64 06 fb 01 65 07 fb 01 66 08 fb 01 67 09 fb 01 68 0a fb 01 69 0b fb 01 6a 0c fb 01 6b 0d fb 01 6c 0e fb 01 6d 0f fb 01 6e 10 fb 01 6f 11 fb 01 70 12 fb 01 71 13 fb 01 72 14 fb 01 73 15 fb 01 74 04 fc 01 75 05 fc 01 76 06 fc 01 77 0c fc 01 78 0d fc 01 79 10 fc 01 7a 11 fc 01 7b 12 fc 01 7c 13 fc 01 7d 14 fc 01 7e 15 fc 01 7f 16 fc 01 80 17 fc 01 81 18 fc 01 82 19 fc 01 83 1a fc 01 84 02 fd 01 85 06 fd 01 86 07 fd 01 87 08 fd 01 88 09 fd 01 89 0a fd 01 8a 0b fd 01 8b 0c fd 01 8c 10 fd 01 8d 11 fd 01 8e 12 fd 01 8f 13 fd 01",
        "40 01 0e 00 ab b0 01 00 00 00 00 00 00 00 11 00 00 34 08 a3 30 30 30 17 f2 01 31 18 f2 01 32 19 f2 01 33 1a f2 01 34 00 f3 01 35 01 f3 01 36 02 f3 01 37 03 f3 01 38 04 f3 01 39 05 f3 01 3a 06 f3 01 3b 07 f3 01 3c 03 f5 01 3d 0b f5 01 3e 13 f5 01 3f 08 f6 01 40 01 f8 01 41 02 f8 01 42 03 f8 01 43 04 f8 01 44 05 f8 01 45 09 f8 01 46 0e f8 01 47 0f f8 01 48 10 f8 01 49 11 f8 01 4a 14 f8 01 4b 15 f8 01 4c 03 f9 01 4d 04 f9 01 4e 05 f9 01 4f 0b f9 01 50 15 f9 01 51 16 f9 01 52 02 fa 01 53 03 fa 01 54 04 fa 01 55 05 fa 01 56 06 fa 01 57 09 fa 01 58 0a fa 01 59 0b fa 01 5a 0d fa 01 5b 0e fa 01 5c 0f fa 01 5d 14 fa 01 5e 00 fb 01 5f 01 fb 01",
        "40 01 0e 00 ab b0 01 00 00 00 00 00 00 00 11 00 00 34 08 a3 00 30 00 00 00 00 01 00 e8 00 02 00 ea 00 03 00 eb 00 04 00 ec 00 05 00 ee 00 06 00 ef 00 07 00 f0 00 08 d8 fe 00 09 00 ff 00 0a 00 ed 01 0b 00 ee 01 0c 00 ef 01 0d 07 f0 01 0e 08 f0 01 0f 09 f0 01 10 0a f0 01 11 0b f0 01 12 0c f0 01 13 10 f0 01 14 11 f0 01 15 14 f0 01 16 16 f0 01 17 01 f1 01 18 05 f1 01 19 0d f1 01 1a 12 f1 01 1b 13 f1 01 1c 14 f1 01 1d 19 f1 01 1e 1a f1 01 1f 00 f2 01 20 01 f2 01 21 05 f2 01 22 08 f2 01 23 09 f2 01 24 0a f2 01 25 0c f2 01 26 0d f2 01 27 0e f2 01 28 0f f2 01 29 10 f2 01 2a 11 f2 01 2b 12 f2 01 2c 13 f2 01 2d 14 f2 01 2e 15 f2 01 2f 16 f2 01",
    ];

    /// The same gateway's answers to Get Tx PGN Enable List F2: the
    /// proprietary part first.
    const F2_2690: [&str; 2] = [
        "4f 02 0e 00 ab b0 01 00 00 00 00 00 01 03 11 00 00 20 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 20 40 00 00 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00",
        "4f 01 0e 00 ab b0 01 00 00 00 00 00 01 02 11 00 00 0c 00 0c 01 06 ff ff 02 06 ff ff 03 06 ff ff 04 06 ff ff 05 06 ff ff 06 07 ff ff 0a 03 ff ff 0b 06 ff ff 0c 07 ff ff 14 07 60 ea 15 06 ff ff 16 06 ff ff",
    ];

    /// And its answer to Get Tx PGN Enable List F1, the PGNs part.
    const F1_2690: &str = "49 01 0e 00 ab b0 01 00 00 00 00 00 0c 00 e8 00 00 00 ea 00 00 00 eb 00 00 00 ec 00 00 00 ee 00 00 00 ef 00 00 06 ff 00 00 00 ed 01 00 00 ee 01 00 00 ef 01 00 11 f0 01 00 14 f0 01 00";

    /// What that gateway has enabled, as F2 shows it: 12 standard PGNs,
    /// then the proprietary 65286, 130822 and 130847.
    const ENABLED_2690: [u32; 15] = [
        59392, 59904, 60160, 60416, 60928, 61184, 126208, 126464, 126720, 126993, 126996, 126998,
        65286, 130822, 130847,
    ];

    /// A sync started on a gateway with `firmware`, up to its list read.
    fn started_on(firmware: u16, wanted: Vec<u32>) -> (TxListSync, Vec<Vec<u8>>) {
        let mut s = TxListSync::new(wanted, Arc::default());
        s.set_firmware(firmware);
        s.on_message(&MODE_ANSWER, 0);
        let sent = s.on_tick(READ_DELAY_MS);
        (s, sent)
    }

    fn f2_read() -> Vec<Vec<u8>> {
        vec![
            command(&[BEM_SUPPORTED_PGN_LIST]),
            command(&[BEM_TX_PGN_ENABLE_LIST_F2]),
        ]
    }

    #[test]
    fn firmware_from_2_500_is_read_in_format_2() {
        assert_eq!(started_on(2_690, vec![127508]).1, f2_read());
        assert_eq!(started_on(2_500, vec![127508]).1, f2_read());
        assert_eq!(
            started_on(2_190, vec![127508]).1,
            vec![command(&[BEM_TX_PGN_ENABLE_LIST_F1])]
        );
    }

    /// F2 as firmware 2.690 sends it reads back all 15 PGNs, the
    /// proprietary ones too.
    #[test]
    fn the_format_2_list_of_firmware_2690() {
        let mut list = F2List::default();
        for part in SUPPORTED_2690 {
            list.add_supported(&hex(part)[12..]);
            assert_eq!(list.pgns(), None);
        }
        assert_eq!(list.supported.complete().map(|s| s.len()), Some(163));
        list.add_enabled(&hex(F2_2690[0])[12..]);
        assert_eq!(list.pgns(), None, "the standard part is still missing");
        list.add_enabled(&hex(F2_2690[1])[12..]);
        assert_eq!(list.pgns(), Some(ENABLED_2690.to_vec()));
    }

    /// F1 on the same gateway lists 12: proprietary 65286 in place of
    /// 126998, and no 130822 or 130847.
    #[test]
    fn the_format_1_list_of_firmware_2690_is_incomplete() {
        let f1 = list_pgns(&hex(F1_2690));
        assert_eq!(f1.len(), 12);
        assert!(f1.contains(&65286));
        let missing: Vec<u32> = ENABLED_2690
            .into_iter()
            .filter(|pgn| !f1.contains(pgn))
            .collect();
        assert_eq!(missing, [126998, 130822, 130847]);
    }

    /// Fed the captured answers, a sync that wants PGNs F1 cannot show
    /// finds them there and writes nothing; one that wants another PGN
    /// enables it.
    #[test]
    fn a_format_2_read_drives_the_sync() {
        let answers = || SUPPORTED_2690.into_iter().chain(F2_2690).map(hex);

        let (mut s, _) = started_on(2_690, vec![126998, 130847]);
        for answer in answers() {
            assert!(s.on_message(&answer, 2_100).is_empty());
        }
        assert!(s.is_done());

        let (mut s, _) = started_on(2_690, vec![130847, 127508]);
        let sent: Vec<Vec<u8>> = answers()
            .flat_map(|answer| s.on_message(&answer, 2_100))
            .collect();
        assert_eq!(sent, vec![enable_command(127508)]);
    }

    /// A gateway that refuses F2 is read in Format 1.
    #[test]
    fn a_refused_format_2_read_falls_back_to_format_1() {
        let (mut s, _) = started_on(2_690, vec![127508]);
        let mut refused = hex(F2_2690[1]);
        refused[8..12].copy_from_slice(&(-1139i32).to_le_bytes());
        assert_eq!(
            s.on_message(&refused, 2_100),
            vec![command(&[BEM_TX_PGN_ENABLE_LIST_F1])]
        );
        assert!(matches!(s.state, State::Reading { .. }));
    }

    /// A gateway that never answers F2 is read in Format 1.
    #[test]
    fn an_unanswered_format_2_read_falls_back_to_format_1() {
        let (mut s, _) = started_on(2_690, vec![127508]);
        let mut t = READ_DELAY_MS;
        for _ in 1..READ_ATTEMPTS {
            t += ANSWER_TIMEOUT_MS;
            assert_eq!(s.on_tick(t), f2_read());
        }
        assert_eq!(
            s.on_tick(t + ANSWER_TIMEOUT_MS),
            vec![command(&[BEM_TX_PGN_ENABLE_LIST_F1])]
        );
    }

    /// Parts that arrive again (a retried read) or out of order still
    /// make one list; a part giving another size starts it over.
    #[test]
    fn parts_are_put_in_place_by_their_first_index() {
        let mut parts = Parts::default();
        parts.add(4, 2, [12, 13]);
        assert_eq!(parts.complete(), None);
        parts.add(4, 0, [10, 11]);
        parts.add(4, 0, [10, 11]);
        assert_eq!(parts.complete(), Some(vec![10, 11, 12, 13]));
        parts.add(3, 0, [20, 21]);
        assert_eq!(parts.complete(), None);
        let mut empty: Parts<u8> = Parts::default();
        empty.add(0, 0, []);
        assert_eq!(empty.complete(), Some(vec![]));
    }
}
