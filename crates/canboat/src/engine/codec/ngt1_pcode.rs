// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The NGT-1's P-Code output per port: BEM `0x13` Get / Set Port P-Code
//! ([SDK](https://github.com/Actisense/SDK/blob/main/docs/DataFormats/Binary/bem-detail/port-pcode-config.md)).
//! With P-Codes on, the gateway sends `Actisense: System status` (BEM
//! `0xF2`) about once a second, with the bus load and the error counts that
//! the synthetic network status (PGN 262400) carries.
//!
//! Only on request ([`super::ngt1::Config::pcode`]): once the transmit list
//! is settled the codec reads the setting (`A1 01 13`) and logs it. With
//! ports to set, and one of them different, it then writes the whole array,
//! `0xFF` for each port left as it is, and logs what the gateway reports.
//! The SDK says the gateway saves it in EEPROM at once, so nothing is
//! written when it already holds what is asked.
//!
//! | Sent | Answer data |
//! |------|-------------|
//! | Get: — | the data size `n`, then `n` bytes, one per port: `0` off, `1` on (older firmware may report the stored raw value, e.g. `0xFF`: any non-zero is on) |
//! | Set: one byte per port, `0` off, `1` on, `0xFF` no change | as Get |
//!
//! A gateway that does not answer within [`ANSWER_TIMEOUT_MS`] is asked
//! once more, then warned about; one that answers with an error is warned
//! about. Either way the codec carries on as before.

use crate::engine::format::ngt1::{
    BEM_PORT_PCODE, BemResponse, describe_error, encode_get_port_pcode, encode_set_port_pcode,
};

/// How long the gateway gets to answer.
pub const ANSWER_TIMEOUT_MS: u64 = 3_000;
/// How often a Get or Set is sent before giving up.
const ATTEMPTS: u8 = 2;

/// What to do with the P-Codes: read them, and set the ports named.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PCodeRequest {
    /// (port, on); ports not named are left as they are.
    pub set: Vec<(u8, bool)>,
}

impl PCodeRequest {
    /// `get`, or a comma list of `<port>=on|off`.
    pub fn parse(text: &str) -> Result<Self, String> {
        let text = text.trim();
        if text.eq_ignore_ascii_case("get") {
            return Ok(Self::default());
        }
        let mut set = Vec::new();
        for part in text.split(',') {
            let (port, value) = part
                .split_once('=')
                .ok_or_else(|| format!("`{part}`: give `get` or `<port>=on|off`"))?;
            let port: u8 = port
                .trim()
                .parse()
                .map_err(|_| format!("`{port}`: not a port number"))?;
            let on = match value.trim().to_ascii_lowercase().as_str() {
                "on" | "1" => true,
                "off" | "0" => false,
                other => return Err(format!("`{other}`: give on or off")),
            };
            if set.iter().any(|&(p, _)| p == port) {
                return Err(format!("port {port} is named twice"));
            }
            set.push((port, on));
        }
        Ok(Self { set })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Getting { attempt: u8, deadline: u64 },
    Setting { attempt: u8, deadline: u64 },
    Done,
}

/// Reads, and if asked sets, the gateway's P-Codes; see the module docs.
#[derive(Debug, Clone)]
pub struct PCodeSync {
    wanted: Option<PCodeRequest>,
    state: State,
    /// The array written, to write it again on a retry.
    written: Vec<u8>,
}

impl PCodeSync {
    /// Nothing is done with `None`.
    pub fn new(wanted: Option<PCodeRequest>) -> Self {
        let state = if wanted.is_some() {
            State::Idle
        } else {
            State::Done
        };
        Self {
            wanted,
            state,
            written: Vec::new(),
        }
    }

    /// The gateway restarted: read (and set) again.
    pub fn restart(&mut self) {
        *self = Self::new(self.wanted.take());
    }

    /// Whether there is nothing (more) to do.
    pub fn is_done(&self) -> bool {
        self.state == State::Done
    }

    /// Start, or retry after a timeout. `ready`: the gateway is set up
    /// (mode settled, transmit list done), so a command is not lost.
    pub fn on_tick(&mut self, now: u64, ready: bool) -> Vec<Vec<u8>> {
        match self.state {
            State::Idle if ready => {
                self.state = State::Getting {
                    attempt: 1,
                    deadline: now + ANSWER_TIMEOUT_MS,
                };
                vec![encode_get_port_pcode()]
            }
            State::Getting { attempt, deadline } if now >= deadline => {
                if attempt < ATTEMPTS {
                    self.state = State::Getting {
                        attempt: attempt + 1,
                        deadline: now + ANSWER_TIMEOUT_MS,
                    };
                    vec![encode_get_port_pcode()]
                } else {
                    log::warn!(
                        "ngt1: no answer to Get Port P-Code (BEM 0x13); this firmware may not have it"
                    );
                    self.state = State::Done;
                    Vec::new()
                }
            }
            State::Setting { attempt, deadline } if now >= deadline => {
                if attempt < ATTEMPTS {
                    self.state = State::Setting {
                        attempt: attempt + 1,
                        deadline: now + ANSWER_TIMEOUT_MS,
                    };
                    vec![encode_set_port_pcode(&self.written)]
                } else {
                    log::warn!("ngt1: no answer to Set Port P-Code (BEM 0x13)");
                    self.state = State::Done;
                    Vec::new()
                }
            }
            _ => Vec::new(),
        }
    }

    /// A BEM response; only `0x13` is ours.
    pub fn on_response(&mut self, response: &BemResponse<'_>, now: u64) -> Vec<Vec<u8>> {
        if response.bem != BEM_PORT_PCODE {
            return Vec::new();
        }
        let setting = match self.state {
            State::Getting { .. } => false,
            State::Setting { .. } => true,
            _ => return Vec::new(),
        };
        let what = if setting { "Set" } else { "Get" };
        if response.error != 0 {
            log::warn!(
                "ngt1: the gateway refused {what} Port P-Code: error {}",
                describe_error(response.error)
            );
            self.state = State::Done;
            return Vec::new();
        }
        let Some(codes) = port_pcodes(response.data) else {
            log::warn!(
                "ngt1: the answer to {what} Port P-Code is malformed: {:02x?}",
                response.data
            );
            self.state = State::Done;
            return Vec::new();
        };
        log::info!("ngt1: P-Codes per port: {}", describe(codes));
        let set = self
            .wanted
            .as_ref()
            .map(|w| w.set.as_slice())
            .unwrap_or(&[]);
        if setting {
            let missed: Vec<u8> = set
                .iter()
                .filter(|&&(port, on)| codes.get(port as usize).map(|&c| c != 0) != Some(on))
                .map(|&(port, _)| port)
                .collect();
            if !missed.is_empty() {
                log::warn!("ngt1: the gateway did not take the P-Code of port(s) {missed:?}");
            }
            self.state = State::Done;
            return Vec::new();
        }
        if let Some(&(port, _)) = set.iter().find(|&&(port, _)| port as usize >= codes.len()) {
            log::warn!(
                "ngt1: no P-Code set: port {port} does not exist, the gateway has {} port(s)",
                codes.len()
            );
            self.state = State::Done;
            return Vec::new();
        }
        let array: Vec<u8> = (0..codes.len())
            .map(
                |i| match set.iter().find(|&&(port, _)| port as usize == i) {
                    Some(&(_, on)) if (codes[i] != 0) != on => u8::from(on),
                    _ => 0xff,
                },
            )
            .collect();
        if array.iter().all(|&b| b == 0xff) {
            self.state = State::Done;
            return Vec::new();
        }
        log::info!("ngt1: setting P-Codes per port: {}", describe(&array));
        self.written = array;
        self.state = State::Setting {
            attempt: 1,
            deadline: now + ANSWER_TIMEOUT_MS,
        };
        vec![encode_set_port_pcode(&self.written)]
    }
}

/// The per-port bytes of a Get / Set Port P-Code answer: a data size, then
/// that many bytes.
pub fn port_pcodes(data: &[u8]) -> Option<&[u8]> {
    let (&n, rest) = data.split_first()?;
    rest.get(..n as usize)
}

/// `port 0 off, port 1 on`; `0xFF` in a Set is `no change`.
fn describe(codes: &[u8]) -> String {
    codes
        .iter()
        .enumerate()
        .map(|(i, &c)| {
            let v = match c {
                0 => "off".to_string(),
                1 => "on".to_string(),
                0xff => "no change / 0xff".to_string(),
                c => format!("on ({c:#04x})"),
            };
            format!("port {i} {v}")
        })
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::format::ngt1::{NGT_MSG_SEND, Ngt1Decoder, NgtEvent};

    fn answer(error: i32, data: &[u8]) -> Vec<u8> {
        let mut p = vec![BEM_PORT_PCODE, 0, 0x0e, 0x00];
        p.extend_from_slice(&1234u32.to_le_bytes());
        p.extend_from_slice(&error.to_le_bytes());
        p.extend_from_slice(data);
        p
    }

    fn sent(bytes: &[u8]) -> Vec<u8> {
        let mut d = Ngt1Decoder::new();
        match &d.push_bytes(bytes)[..] {
            [NgtEvent::Message(m)] => {
                assert_eq!(m.command, NGT_MSG_SEND);
                m.payload.clone()
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn parses_the_option() {
        assert_eq!(PCodeRequest::parse("get").unwrap().set, []);
        assert_eq!(
            PCodeRequest::parse("0=on, 1=off").unwrap().set,
            [(0, true), (1, false)]
        );
        assert!(PCodeRequest::parse("0=maybe").is_err());
        assert!(PCodeRequest::parse("x=on").is_err());
        assert!(PCodeRequest::parse("0=on,0=off").is_err());
    }

    #[test]
    fn nothing_without_a_request() {
        let mut s = PCodeSync::new(None);
        assert!(s.is_done());
        assert!(s.on_tick(0, true).is_empty());
    }

    /// The SDK's Get example: `A1 01 13`.
    #[test]
    fn reads_once_ready_and_only_logs_on_get() {
        let mut s = PCodeSync::new(Some(PCodeRequest::default()));
        assert!(s.on_tick(0, false).is_empty());
        let out = s.on_tick(10, true);
        assert_eq!(out.len(), 1);
        assert_eq!(sent(&out[0]), [0x13]);
        let r = answer(0, &[2, 0, 1]);
        let out = s.on_response(&BemResponse::parse(&r).unwrap(), 20);
        assert!(out.is_empty());
        assert!(s.is_done());
    }

    #[test]
    fn sets_only_the_port_that_differs() {
        let mut s = PCodeSync::new(Some(PCodeRequest::parse("0=on,1=on").unwrap()));
        s.on_tick(0, true);
        let r = answer(0, &[2, 0, 1]);
        let out = s.on_response(&BemResponse::parse(&r).unwrap(), 10);
        assert_eq!(out.len(), 1);
        // port 0 off -> on; port 1 already on: no change
        assert_eq!(sent(&out[0]), [0x13, 0x01, 0xff]);
        let r = answer(0, &[2, 1, 1]);
        assert!(
            s.on_response(&BemResponse::parse(&r).unwrap(), 20)
                .is_empty()
        );
        assert!(s.is_done());
    }

    #[test]
    fn writes_nothing_when_it_already_holds_it() {
        let mut s = PCodeSync::new(Some(PCodeRequest::parse("1=on").unwrap()));
        s.on_tick(0, true);
        // an old firmware's raw 0xff counts as on
        let r = answer(0, &[2, 0, 0xff]);
        assert!(
            s.on_response(&BemResponse::parse(&r).unwrap(), 10)
                .is_empty()
        );
        assert!(s.is_done());
    }

    #[test]
    fn a_port_that_does_not_exist_sets_nothing() {
        let mut s = PCodeSync::new(Some(PCodeRequest::parse("3=on").unwrap()));
        s.on_tick(0, true);
        let r = answer(0, &[2, 0, 0]);
        assert!(
            s.on_response(&BemResponse::parse(&r).unwrap(), 10)
                .is_empty()
        );
        assert!(s.is_done());
    }

    #[test]
    fn retries_once_then_gives_up() {
        let mut s = PCodeSync::new(Some(PCodeRequest::default()));
        assert_eq!(s.on_tick(0, true).len(), 1);
        assert!(s.on_tick(ANSWER_TIMEOUT_MS - 1, true).is_empty());
        assert_eq!(s.on_tick(ANSWER_TIMEOUT_MS, true).len(), 1);
        assert!(s.on_tick(2 * ANSWER_TIMEOUT_MS, true).is_empty());
        assert!(s.is_done());
    }

    #[test]
    fn a_refusal_ends_it() {
        let mut s = PCodeSync::new(Some(PCodeRequest::parse("0=on").unwrap()));
        s.on_tick(0, true);
        let r = answer(-1, &[]);
        assert!(
            s.on_response(&BemResponse::parse(&r).unwrap(), 10)
                .is_empty()
        );
        assert!(s.is_done());
    }

    #[test]
    fn other_answers_are_not_ours() {
        let mut s = PCodeSync::new(Some(PCodeRequest::default()));
        s.on_tick(0, true);
        let mut r = answer(0, &[2, 0, 0]);
        r[0] = 0x11;
        assert!(
            s.on_response(&BemResponse::parse(&r).unwrap(), 10)
                .is_empty()
        );
        assert!(!s.is_done());
    }

    #[test]
    fn a_restart_reads_again() {
        let mut s = PCodeSync::new(Some(PCodeRequest::default()));
        s.on_tick(0, true);
        let r = answer(0, &[1, 1]);
        s.on_response(&BemResponse::parse(&r).unwrap(), 10);
        assert!(s.is_done());
        s.restart();
        assert_eq!(s.on_tick(20, true).len(), 1);
    }
}
