// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The PGNs a node advertises in its PGN 126464 Transmit / Receive PGN lists.
//!
//! A display that asks a node what it sends (an ISO Request for 126464) uses
//! the answer to decide which data it can pick that node as a source for —
//! a Navico MFD lists a node's data but will not offer it as a field source
//! when the PGN is missing from the node's Transmit list. A gateway therefore
//! has to advertise the PGNs its *application* sends, not only the ISO
//! housekeeping PGNs the gateway itself originates.
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
const MAX_PGN: u32 = 0x1_FFFF;

/// The PGNs the application transmits and receives, to advertise on top of
/// the gateway's own ISO housekeeping PGNs. Set before the device opens.
///
/// These are advertisements only: they never filter what the gateway passes
/// up or lets out.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PgnLists {
    /// PGNs the application transmits (PGN 126464 function code 0).
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
