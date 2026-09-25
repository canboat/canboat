// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The NGT-1's Transmit PGN Enable list: the PGNs the gateway will put on
//! the bus, and — for an addressable NGT-1 — the Transmit list it answers
//! PGN 126464 with. A PGN missing from it is silently not transmitted.
//!
//! Actisense does not document the commands, so they are taken from
//! canboatjs (`lib/actisense-serial.ts`), which reverse-engineered them.
//! Each goes out as an `NGT_MSG_SEND` (0xA1) message and is answered by an
//! `NGT_MSG_RECEIVED` (0xA0) one carrying the same command byte first:
//!
//! | Command | Sent                                    | Answer                      |
//! |---------|-----------------------------------------|-----------------------------|
//! | `0x49`  | read the Transmit PGN Enable list       | `[0x49, 1, …]` with PGNs, then `[0x49, 4, …]` at the end |
//! | `0x47`  | `[0x47, pgn u32, 1, 0xfffffffe u32 ×2]`: enable one PGN | `[0x47, 1, …, result i32 @8, pgn u32 @12, …]` |
//! | `0x01`  | save the lists to EEPROM                | `[0x01, …]`                 |
//! | `0x4b`  | activate the saved lists                | `[0x4b, …]`                 |
//!
//! In a `[0x49, 1, …]` answer the PGN count is at byte 12 and the PGNs
//! follow as little-endian `u32`s from byte 13. The two `0xfffffffe`
//! words of `0x47` are copied from canboatjs.
//!
//! Seen on an NGT-1-A (2026-09):
//!
//! - The list comes back in four parts, all with the same count. Part 1
//!   has the PGNs; part 2 a per-PGN `u32` that reads as a transmit interval
//!   in ms (`65535` for all but 126993 Heartbeat's `60000`); part 3 a
//!   per-PGN `u32`, all zero; part 4 ends it. The two words of `0x47` are
//!   presumably those two attributes, set to "default".
//! - **The read is truncated on a long list**: with 17 PGNs enabled it
//!   reported 14, with 23 it reported 13 (the lowest ones). A PGN past that
//!   point looks missing although it is enabled.
//! - `0x47` answers status 1 whatever happens; the outcome is the `i32` at
//!   byte 8: 0 added, −996 already on the list (see the truncated read),
//!   −997 refused (PGN 0x40000).
//! - A PDU1 PGN is stored with its low (destination) byte cleared: enabling
//!   PGN 1 enables PGN 0.
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

use crate::engine::format::ngt1::{NGT_MSG_SEND, encode_ngt_message};

/// Actisense's "NGT-specific message received" command: the gateway's
/// answers to [`NGT_MSG_SEND`] commands.
pub const NGT_MSG_RECEIVED: u8 = 0xa0;

const CMD_STARTUP: u8 = 0x11;
const CMD_COMMIT: u8 = 0x01;
const CMD_ENABLE_TX_PGN: u8 = 0x47;
const CMD_READ_TX_LIST: u8 = 0x49;
const CMD_ACTIVATE: u8 = 0x4b;

const LIST_PART: u8 = 1;
const LIST_END: u8 = 4;
const ENABLE_OK: u8 = 1;

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
            (State::AwaitStartup, CMD_STARTUP) => {
                self.state = State::ReadAt {
                    at: now + READ_DELAY_MS,
                };
                Vec::new()
            }
            (State::Reading { have, .. }, CMD_READ_TX_LIST) if status == Some(LIST_PART) => {
                have.extend(list_pgns(payload));
                self.deadline = now + ANSWER_TIMEOUT_MS;
                Vec::new()
            }
            (State::Reading { have, .. }, CMD_READ_TX_LIST) if status == Some(LIST_END) => {
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
            (State::Enabling { todo, added }, CMD_ENABLE_TX_PGN) => {
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
                        vec![command(&[CMD_COMMIT])]
                    }
                }
            }
            (State::Committing { added }, CMD_COMMIT) => {
                let added = std::mem::take(added);
                self.state = State::Activating { added };
                self.deadline = now + ANSWER_TIMEOUT_MS;
                vec![command(&[CMD_ACTIVATE])]
            }
            (State::Activating { added }, CMD_ACTIVATE) => {
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
        vec![command(&[CMD_READ_TX_LIST])]
    }
}

/// The PGNs in one `[0x49, 1, …]` answer.
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

/// What an accepted `0x47` did.
#[derive(Debug, PartialEq, Eq)]
enum Enabled {
    /// The PGN was added: the list changed and needs saving.
    Now,
    /// The PGN was on the list already: nothing changed.
    Already,
}

/// The `0x47` result code for a PGN already on the list.
const ALREADY_ENABLED: i32 = -996;

/// What an `0x47` answer says. The status byte only says the command was
/// taken — an NGT-1-A answers 1 whatever happened — and the outcome is an
/// `i32` at payload byte 8: 0 added, −996 already on the list, other
/// values refused (−997 for 0x40000).
fn enable_result(payload: &[u8], status: Option<u8>) -> Result<Enabled, String> {
    if status != Some(ENABLE_OK) {
        return Err(format!("status {status:?}"));
    }
    match payload.get(8..12) {
        Some(b) => match i32::from_le_bytes([b[0], b[1], b[2], b[3]]) {
            0 => Ok(Enabled::Now),
            ALREADY_ENABLED => Ok(Enabled::Already),
            code => Err(format!("error {code}")),
        },
        None => Err("short answer".into()),
    }
}

fn enable_command(pgn: u32) -> Vec<u8> {
    let mut payload = vec![CMD_ENABLE_TX_PGN];
    payload.extend_from_slice(&pgn.to_le_bytes());
    payload.push(1);
    payload.extend_from_slice(&0xffff_fffe_u32.to_le_bytes());
    payload.extend_from_slice(&0xffff_fffe_u32.to_le_bytes());
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

    /// A `[0x49, 1, …]` answer carrying `pgns`.
    fn list_part(pgns: &[u32]) -> Vec<u8> {
        let mut p = vec![CMD_READ_TX_LIST, LIST_PART];
        p.extend_from_slice(&[0; 10]);
        p.push(pgns.len() as u8);
        for pgn in pgns {
            p.extend_from_slice(&pgn.to_le_bytes());
        }
        p
    }

    /// An `0x47` answer as an NGT-1-A sends it: status 1 whatever the
    /// outcome, `result` at byte 8 (0 accepted), the PGN at byte 12.
    fn answer(pgn: u32, result: i32) -> Vec<u8> {
        let mut p = vec![CMD_ENABLE_TX_PGN, 1, 0x0e, 0x00, 0xac, 0x9f, 0x01, 0x00];
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
        s.on_message(&[CMD_READ_TX_LIST, LIST_END], 2_100);
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
    fn started(wanted: Vec<u32>) -> TxListSync {
        let mut s = TxListSync::new(wanted, Arc::default());
        assert!(s.on_message(&[CMD_STARTUP, 1], 0).is_empty());
        assert!(
            s.on_tick(READ_DELAY_MS - 1).is_empty(),
            "waits before reading"
        );
        assert_eq!(s.on_tick(READ_DELAY_MS), vec![command(&[CMD_READ_TX_LIST])]);
        s
    }

    #[test]
    fn nothing_wanted_means_nothing_sent() {
        let mut s = TxListSync::new(Vec::new(), Arc::default());
        assert!(s.is_done());
        assert!(s.on_message(&[CMD_STARTUP, 1], 0).is_empty());
        assert!(s.on_tick(u64::MAX).is_empty());
    }

    #[test]
    fn a_list_that_has_every_pgn_is_not_written() {
        let mut s = started(vec![127508, 127506]);
        assert!(s.on_message(&list_part(&[59392, 127508]), 2_100).is_empty());
        assert!(s.on_message(&list_part(&[127506]), 2_200).is_empty());
        assert!(
            s.on_message(&[CMD_READ_TX_LIST, LIST_END], 2_300)
                .is_empty()
        );
        assert!(s.is_done());
    }

    #[test]
    fn missing_pgns_are_enabled_saved_and_activated() {
        let mut s = started(vec![127508, 127506, 127258]);
        s.on_message(&list_part(&[127506]), 2_100);
        assert_eq!(
            s.on_message(&[CMD_READ_TX_LIST, LIST_END], 2_200),
            vec![enable_command(127508)]
        );
        assert_eq!(
            s.on_message(&answer(127508, 0), 2_300),
            vec![enable_command(127258)]
        );
        assert_eq!(
            s.on_message(&answer(127258, 0), 2_400),
            vec![command(&[CMD_COMMIT])]
        );
        assert_eq!(
            s.on_message(&[CMD_COMMIT], 2_500),
            vec![command(&[CMD_ACTIVATE])]
        );
        assert!(s.on_message(&[CMD_ACTIVATE], 2_600).is_empty());
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
        assert_eq!(s.on_tick(t), vec![command(&[CMD_READ_TX_LIST])]);
        assert_eq!(
            s.on_tick(t + ANSWER_TIMEOUT_MS),
            vec![command(&[CMD_READ_TX_LIST])]
        );
        assert!(s.on_tick(t + 2 * ANSWER_TIMEOUT_MS).is_empty());
        assert!(s.is_done());
    }

    /// The periodic startup ping keeps being confirmed; only the first
    /// confirmation starts the sync.
    #[test]
    fn later_startup_confirmations_are_ignored() {
        let mut s = started(vec![127508]);
        assert!(s.on_message(&[CMD_STARTUP, 1], 2_100).is_empty());
        assert!(matches!(s.state, State::Reading { .. }));
    }

    /// When the gateway refuses every PGN nothing changed, so nothing is
    /// saved: no EEPROM write.
    #[test]
    fn nothing_is_saved_when_every_pgn_is_refused() {
        let mut s = started(vec![127508]);
        s.on_message(&[CMD_READ_TX_LIST, LIST_END], 2_100);
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
        s.on_message(&[CMD_STARTUP, 1], 0);
        s.on_tick(READ_DELAY_MS);
        s.on_message(&list_part(have), READ_DELAY_MS + 1);
        let sent = s.on_message(&[CMD_READ_TX_LIST, LIST_END], READ_DELAY_MS + 2);
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
            vec![command(&[CMD_COMMIT])]
        );
        s.on_message(&[CMD_COMMIT], 3_100);
        s.on_message(&[CMD_ACTIVATE], 3_200);

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
            vec![command(&[CMD_COMMIT])]
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
        s.on_message(&[CMD_READ_TX_LIST, LIST_END], 2_100);
        assert_eq!(
            s.on_message(&answer(127508, -996), 2_200),
            vec![enable_command(127506)]
        );
    }
}
