// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The PGNs a node advertises in its PGN 126464 Transmit / Receive PGN lists.
//!
//! Other devices ask a node what it sends and reads with an ISO Request for
//! PGN 126464. NMEA 2000 expects the answer to cover every PGN the node
//! transmits, so a gateway should advertise the PGNs its *application*
//! sends, not only the ISO housekeeping PGNs it originates itself. How much
//! a given display relies on the list (for instance when choosing data
//! sources) varies by make and firmware.
//!
//! [`PgnLists`] is what the application declares, before the device is
//! opened. How a backend honours it differs per gateway, which
//! [`PgnListSupport`] reports: the SocketCAN gateway answers 126464 itself and
//! can advertise anything, while a stand-alone gateway (NGT-1, iKonvert,
//! YDWG, …) answers with lists held in the device.

/// Most PGNs one PGN 126464 list can carry: the list is a single fast-packet
/// message (at most 223 bytes), one function-code byte plus three bytes per
/// PGN.
pub const MAX_PGN_LIST_LEN: usize = 74;

/// Highest valid PGN; anything above it (canboat's synthetic `>= 0x40000`
/// PGNs included) never goes on the wire, so it is never advertised.
pub(crate) const MAX_PGN: u32 = 0x1_FFFF;

/// ISO and NMEA 2000 network-management PGNs: acknowledgement, request,
/// transport protocol, address claim, group function, PGN list, heartbeat,
/// product and configuration information. Always allowed out, whatever the
/// transmit list says.
pub const NETWORK_MANAGEMENT_PGNS: [u32; 10] = [
    59392, 59904, 60160, 60416, 60928, 126208, 126464, 126993, 126996, 126998,
];

/// The PGNs the application transmits and receives, to advertise on top of
/// the gateway's own ISO housekeeping PGNs. Set before the device opens.
///
/// # ⚠️ ON AN iKONVERT, `tx` IS THE WHOLE TRANSMIT LIST
///
/// **ONCE `tx` NAMES ANY PGN, EVERY PGN NOT IN IT IS REFUSED — NOT SENT.**
/// The gateway itself only transmits the PGNs in its transmit list, and that
/// list can only be set before it goes on the bus, so the driver refuses
/// the rest up front (logging each refused PGN once) rather than handing the
/// gateway frames it will reject. Decide the complete list *before* opening
/// the device. The network-management PGNs ([`NETWORK_MANAGEMENT_PGNS`])
/// are always allowed. With `tx` empty nothing is refused, and the
/// gateway's own list decides.
///
/// On SocketCAN the lists are advertisements only: nothing is filtered.
/// `rx` never filters anything.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PgnLists {
    /// PGNs the application transmits (PGN 126464 function code 0). **On an
    /// iKonvert, the only PGNs (besides network management) it will send;
    /// see the type's documentation.**
    pub tx: Vec<u32>,
    /// PGNs the application receives (PGN 126464 function code 1).
    pub rx: Vec<u32>,
}

impl PgnLists {
    /// Whether neither list names a PGN.
    pub fn is_empty(&self) -> bool {
        self.tx.is_empty() && self.rx.is_empty()
    }
}

/// How a backend honours one of the [`PgnLists`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum PgnListSupport {
    /// canboat answers PGN 126464 itself and advertises the list (the
    /// SocketCAN gateway).
    Answered,
    /// The list is written into the gateway, which answers PGN 126464 with
    /// it (iKonvert, NGT-1). On these gateways the Transmit list also
    /// decides which PGNs the gateway will put on the bus at all.
    Pushed,
    /// The backend has no known way to advertise the list; it was ignored.
    Unsupported,
}

/// What a backend did with the [`PgnLists`] it was given, per list.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PgnListStatus {
    pub tx: PgnListSupport,
    pub rx: PgnListSupport,
    /// PGNs from lists the backend supports that are nevertheless not
    /// advertised: invalid PGNs, and those past [`MAX_PGN_LIST_LEN`].
    pub dropped: Vec<u32>,
}

/// Refuses PGNs missing from a transmit list the client named, for the
/// gateways that only send what is on their list (see [`PgnLists`]).
pub(crate) struct TxGate {
    label: &'static str,
    allowed: Vec<u32>,
    /// PGNs refused so far, so each is logged once.
    refused: std::sync::Mutex<Vec<u32>>,
}

impl TxGate {
    /// A gate allowing `named`, canboat's own `extra` PGNs and the
    /// network-management PGNs — or `None` when the client named none, so
    /// nothing is refused. `extra` alone never closes the gate.
    pub(crate) fn new(label: &'static str, named: &[u32], extra: &[u32]) -> Option<Self> {
        if named.is_empty() {
            return None;
        }
        let mut allowed = NETWORK_MANAGEMENT_PGNS.to_vec();
        allowed.extend_from_slice(named);
        allowed.extend_from_slice(extra);
        Some(Self {
            label,
            allowed,
            refused: std::sync::Mutex::new(Vec::new()),
        })
    }

    /// Whether `pgn` may be sent, logging the first refusal of each PGN.
    pub(crate) fn allows(&self, pgn: u32) -> bool {
        if self.allowed.contains(&pgn) {
            return true;
        }
        if let Ok(mut refused) = self.refused.lock()
            && !refused.contains(&pgn)
        {
            refused.push(pgn);
            log::warn!(
                "{}: refusing to send PGN {pgn}: it is not in the transmit list; \
                 name it (--tx-pgn / pgn_lists.tx) before starting",
                self.label
            );
        }
        false
    }
}

/// A list as advertised: `builtin` first, then each PGN of `extra` not
/// already in it. Returns the list and the PGNs of `extra` left out — invalid
/// ones, and those that would take the list past [`MAX_PGN_LIST_LEN`].
pub fn merge(builtin: &[u32], extra: &[u32]) -> (Vec<u32>, Vec<u32>) {
    let mut list = builtin.to_vec();
    let mut dropped = Vec::new();
    for &pgn in extra {
        if list.contains(&pgn) {
            continue;
        }
        if pgn > MAX_PGN || list.len() >= MAX_PGN_LIST_LEN {
            if !dropped.contains(&pgn) {
                dropped.push(pgn);
            }
            continue;
        }
        list.push(pgn);
    }
    (list, dropped)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_gate_allows_what_is_named_and_refuses_the_rest() {
        let gate = TxGate::new("test", &[127508], &[127258]).unwrap();
        for pgn in [127508, 127258, 59904, 126996] {
            assert!(gate.allows(pgn), "{pgn}");
        }
        assert!(!gate.allows(127506));
        assert!(TxGate::new("test", &[], &[127258]).is_none(), "extra alone");
    }

    #[test]
    fn extra_pgns_follow_the_builtin_ones_without_duplicates() {
        let (list, dropped) = merge(&[59392, 126464], &[127508, 126464, 127506, 127508]);
        assert_eq!(list, [59392, 126464, 127508, 127506]);
        assert!(dropped.is_empty());
    }

    #[test]
    fn pgns_that_never_go_on_the_wire_are_dropped() {
        let (list, dropped) = merge(&[59392], &[0x40000, 127508, 0x2_0000]);
        assert_eq!(list, [59392, 127508]);
        assert_eq!(dropped, [0x40000, 0x2_0000]);
    }

    #[test]
    fn the_list_stops_at_one_fast_packet() {
        let extra: Vec<u32> = (130_000..130_100).collect();
        let (list, dropped) = merge(&[59392], &extra);
        assert_eq!(list.len(), MAX_PGN_LIST_LEN);
        assert_eq!(dropped.len(), 100 - (MAX_PGN_LIST_LEN - 1));
        assert_eq!(dropped[0], 130_000 + (MAX_PGN_LIST_LEN as u32 - 1));
    }
}
