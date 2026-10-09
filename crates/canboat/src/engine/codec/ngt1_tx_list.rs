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
//! | `0x49` | Get Tx PGN Enable List F1        | —                    | four messages, sequence 1 to 4: the PGNs, their rates, their timeouts, their priorities; each a count, then `u32`s (`u8`s for the priorities) |
//! | `0x47` | Set Tx PGN Enable                | PGN `u32`, enable `1`, rate `0xfffffffe` (the PGN's default), timeout (ignored); no priority, so it is left as it is | PGN `u32`, enable, rate `u32`, timeout `u32`, priority |
//! | `0x01` | Commit To EEPROM                 | —                    | —           |
//! | `0x4b` | Activate PGN Enable Lists        | —                    | —           |
//!
//! [Actisense SDK]: https://github.com/Actisense/SDK/blob/main/docs/DataFormats/Binary/bem-detail/README.md
//!
//! The SDK marks F1 deprecated in favour of Get Tx PGN Enable List F2
//! (`0x4f`), from firmware v2.500. canboat still reads F1: F2 lists indexes
//! into the gateway's Supported PGN List (`0x40`), which then has to be read
//! too, and the NGT-1s in canboat's captures run firmware 2.190. Moving to
//! F2 waits for a test on an NGT-1 that has it.
//!
//! Seen on an NGT-1-A (2026-09):
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
/// BEM Get / Set Tx PGN Enable.
const BEM_TX_PGN_ENABLE: u8 = 0x47;
/// BEM Get Tx PGN Enable List F1.
const BEM_TX_PGN_ENABLE_LIST_F1: u8 = 0x49;
/// BEM Activate PGN Enable Lists.
const BEM_ACTIVATE_PGN_ENABLE_LISTS: u8 = 0x4b;

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
    /// List requested; collecting its PGNs.
    Reading {
        have: Vec<u32>,
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
        }
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

    /// Advance on the passing of time: start the read once its delay is
    /// over, and retry or give up on an answer that does not come.
    pub fn on_tick(&mut self, now: u64) -> Vec<Vec<u8>> {
        match &self.state {
            State::ReadAt { at } if now >= *at => self.read(1, now),
            State::Reading { attempt, .. } if now >= self.deadline => {
                if *attempt < READ_ATTEMPTS {
                    let attempt = attempt + 1;
                    self.read(attempt, now)
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

    fn read(&mut self, attempt: u32, now: u64) -> Vec<Vec<u8>> {
        self.state = State::Reading {
            have: Vec::new(),
            attempt,
        };
        self.deadline = now + ANSWER_TIMEOUT_MS;
        vec![command(&[BEM_TX_PGN_ENABLE_LIST_F1])]
    }
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
}
