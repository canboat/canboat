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
//! | `0x47`  | `[0x47, pgn u32, 1, 0xfffffffe u32 ×2]`: enable one PGN | `[0x47, 1, …]` on success |
//! | `0x01`  | save the lists to EEPROM                | `[0x01, …]`                 |
//! | `0x4b`  | activate the saved lists                | `[0x4b, …]`                 |
//!
//! In a `[0x49, 1, …]` answer the PGN count is at byte 12 and the PGNs
//! follow as little-endian `u32`s from byte 13. The two `0xfffffffe`
//! words of `0x47` are copied from canboatjs; their meaning is unknown.
//!
//! canboatjs stopped doing this by default in 2020 ("this is possibly
//! causing issues", canboatjs#136). So this runs only for PGNs the
//! embedder names or the application actually sends ([`TxListSync::learn`]),
//! reads the list first, and writes — enable, save, activate — only when a
//! PGN is missing, so a device already set up is never written to again.
//! Learned PGNs are batched: one round, one save, a few seconds after the
//! last new one.

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
/// Quiet spell after the last newly learned PGN before a round starts, so
/// PGNs first sent together share one EEPROM save.
const LEARN_DELAY_MS: u64 = 5_000;

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
    /// Enabling the PGNs still in `todo`, the first one's answer pending.
    Enabling {
        todo: Vec<u32>,
    },
    Committing,
    Activating,
    Done,
}

/// Brings the NGT-1's Transmit PGN Enable list up to `wanted`. Feed it the
/// gateway's `NGT_MSG_RECEIVED` payloads and the passing time; send the
/// byte strings it returns to the gateway.
pub struct TxListSync {
    wanted: Vec<u32>,
    state: State,
    /// When the pending answer times out.
    deadline: u64,
    /// Whether the gateway has confirmed startup; nothing is sent before.
    started: bool,
    /// Learned PGNs waiting for the next round, and when it may start.
    learned: Vec<u32>,
    learn_at: u64,
}

impl TxListSync {
    /// A sync for `wanted`; one that has nothing to do when it is empty.
    pub fn new(wanted: Vec<u32>) -> Self {
        let state = if wanted.is_empty() {
            State::Done
        } else {
            State::AwaitStartup
        };
        Self {
            wanted,
            state,
            deadline: u64::MAX,
            started: false,
            learned: Vec::new(),
            learn_at: u64::MAX,
        }
    }

    /// Whether there is nothing to do: the sync has finished, successfully
    /// or not, and no learned PGN is waiting.
    pub fn is_idle(&self) -> bool {
        self.state == State::Done && self.learned.is_empty()
    }

    /// The application just sent `pgn`: make sure the gateway will transmit
    /// it. A few seconds after the last new one, the list is read again and
    /// whatever is missing enabled in one round.
    pub fn learn(&mut self, pgn: u32, now: u64) {
        if pgn > crate::io::pgn_list::MAX_PGN
            || self.wanted.contains(&pgn)
            || self.learned.contains(&pgn)
        {
            return;
        }
        if self.wanted.len() + self.learned.len() >= crate::io::pgn_list::MAX_PGN_LIST_LEN {
            log::warn!("ngt1: transmit PGN list is full; not enabling PGN {pgn}");
            return;
        }
        log::info!("ngt1: sending PGN {pgn}; making sure the gateway's transmit list has it");
        self.learned.push(pgn);
        self.learn_at = now + LEARN_DELAY_MS;
    }

    /// Advance on an `NGT_MSG_RECEIVED` payload.
    pub fn on_message(&mut self, payload: &[u8], now: u64) -> Vec<Vec<u8>> {
        let (Some(&cmd), status) = (payload.first(), payload.get(1).copied()) else {
            return Vec::new();
        };
        if cmd == CMD_STARTUP {
            self.started = true;
        }
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
                let todo: Vec<u32> = self
                    .wanted
                    .iter()
                    .copied()
                    .filter(|pgn| !have.contains(pgn))
                    .collect();
                if todo.is_empty() {
                    log::info!("ngt1: transmit PGN list already has {:?}", self.wanted);
                    self.state = State::Done;
                    return Vec::new();
                }
                log::info!("ngt1: enabling transmit PGNs {todo:?}");
                let first = enable_command(todo[0]);
                self.state = State::Enabling { todo };
                self.deadline = now + ANSWER_TIMEOUT_MS;
                vec![first]
            }
            (State::Enabling { todo }, CMD_ENABLE_TX_PGN) => {
                let pgn = todo.remove(0);
                if status != Some(ENABLE_OK) {
                    log::warn!("ngt1: the gateway refused transmit PGN {pgn} (status {status:?})");
                }
                self.deadline = now + ANSWER_TIMEOUT_MS;
                match todo.first() {
                    Some(&next) => vec![enable_command(next)],
                    None => {
                        self.state = State::Committing;
                        vec![command(&[CMD_COMMIT])]
                    }
                }
            }
            (State::Committing, CMD_COMMIT) => {
                self.state = State::Activating;
                self.deadline = now + ANSWER_TIMEOUT_MS;
                vec![command(&[CMD_ACTIVATE])]
            }
            (State::Activating, CMD_ACTIVATE) => {
                log::info!("ngt1: transmit PGN list saved and active");
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
            State::Done if self.started && !self.learned.is_empty() && now >= self.learn_at => {
                self.wanted.append(&mut self.learned);
                self.read(1, now)
            }
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
            State::Enabling { .. } | State::Committing | State::Activating
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

    /// Walk a sync through startup up to the list request.
    fn started(wanted: Vec<u32>) -> TxListSync {
        let mut s = TxListSync::new(wanted);
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
        let mut s = TxListSync::new(Vec::new());
        assert!(s.is_idle());
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
        assert!(s.is_idle());
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
            s.on_message(&[CMD_ENABLE_TX_PGN, ENABLE_OK], 2_300),
            vec![enable_command(127258)]
        );
        assert_eq!(
            s.on_message(&[CMD_ENABLE_TX_PGN, ENABLE_OK], 2_400),
            vec![command(&[CMD_COMMIT])]
        );
        assert_eq!(
            s.on_message(&[CMD_COMMIT], 2_500),
            vec![command(&[CMD_ACTIVATE])]
        );
        assert!(s.on_message(&[CMD_ACTIVATE], 2_600).is_empty());
        assert!(s.is_idle());
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
        assert!(s.is_idle());
    }

    /// The periodic startup ping keeps being confirmed; only the first
    /// confirmation starts the sync.
    #[test]
    fn later_startup_confirmations_are_ignored() {
        let mut s = started(vec![127508]);
        assert!(s.on_message(&[CMD_STARTUP, 1], 2_100).is_empty());
        assert!(matches!(s.state, State::Reading { .. }));
    }

    /// A PGN the application sends starts its own round, batched after
    /// a quiet spell, once the gateway has started — even with no PGNs
    /// named up front.
    #[test]
    fn a_sent_pgn_is_enabled_after_a_quiet_spell() {
        let mut s = TxListSync::new(Vec::new());
        s.learn(127508, 0);
        assert!(s.on_tick(LEARN_DELAY_MS).is_empty(), "not before startup");
        s.on_message(&[CMD_STARTUP, 1], LEARN_DELAY_MS);
        s.learn(127506, LEARN_DELAY_MS + 100); // pushes the round back
        assert!(s.on_tick(2 * LEARN_DELAY_MS).is_empty());
        assert_eq!(
            s.on_tick(2 * LEARN_DELAY_MS + 100),
            vec![command(&[CMD_READ_TX_LIST])]
        );
        s.on_message(&list_part(&[127508]), 2 * LEARN_DELAY_MS + 200);
        assert_eq!(
            s.on_message(&[CMD_READ_TX_LIST, LIST_END], 2 * LEARN_DELAY_MS + 300),
            vec![enable_command(127506)],
            "only the PGN the gateway lacks"
        );
    }

    #[test]
    fn what_is_known_or_invalid_is_not_learned() {
        let mut s = started(vec![127508]);
        s.learn(127508, 2_100);
        s.learn(0x2_0000, 2_100);
        assert!(s.learned.is_empty());
        s.learn(127506, 2_100);
        s.learn(127506, 2_200);
        assert_eq!(s.learned, [127506]);
    }

    #[test]
    fn a_refused_pgn_does_not_stop_the_rest() {
        let mut s = started(vec![127508, 127506]);
        s.on_message(&[CMD_READ_TX_LIST, LIST_END], 2_100);
        assert_eq!(
            s.on_message(&[CMD_ENABLE_TX_PGN, 0], 2_200),
            vec![enable_command(127506)]
        );
    }
}
