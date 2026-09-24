// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Transport-agnostic ISO 11783-5 address-claim state machine.
//!
//! The claim handshake — scan the bus, pick a source address, defend it
//! by NAME arbitration, re-claim on conflict — is identical whether the
//! node lives behind a Linux SocketCAN socket (see
//! [`crate::io::device::socketcan`]) or is a synthetic device conjured by a
//! canboat server quirk (the H5000 Motion Sensor impersonation). This
//! module holds *only* that logic, free of any I/O: it consumes the two
//! ISO PGNs that drive claiming (60928 Address Claim, 59904 ISO Request)
//! plus a monotonic `now_ms`, and returns the [`RawFrame`]s to put on the
//! bus. The caller owns the transport and every device-specific PGN
//! (Product Information, heartbeat, PGN lists, …).
//!
//! Returned frames carry no timestamp (`None`); the caller stamps and
//! sends them however its transport requires. The claim / scan frames are
//! all single-frame ≤8-byte PGNs, so no fast-packet splitting is needed.

use crate::engine::{ADDR_GLOBAL, ADDR_NULL, RawFrame};

/// Highest assignable source address; 254 (null) and 255 (global) are
/// reserved (see [`ADDR_NULL`] / [`ADDR_GLOBAL`]).
pub const ADDR_MAX: u8 = 253;

/// How long a claim must go uncontested before we consider it ours.
pub const CLAIM_TIMEOUT_MS: u64 = 250;
/// How long to listen for others' claims before picking an address.
pub const SCAN_TIMEOUT_MS: u64 = 1000;
/// How long an address stays "in use" after its claim was last heard.
/// Devices don't announce leaving; canboat asks every device to re-claim
/// every 5 minutes (`n2kd::request_engine::DEVICE_REQUEST_INTERVAL`), so
/// three missed rounds mean it is gone. A device that is still there but
/// quiet simply wins or loses the arbitration if we pick its address.
pub const USED_TTL_MS: u64 = 15 * 60 * 1000;

const PGN_ISO_REQUEST: u32 = 59904;
const PGN_ISO_ADDRESS_CLAIM: u32 = 60928;

/// Where the node is in the ISO 11783-5 claim handshake.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClaimState {
    /// Claiming is switched off (passive sniff) — no address is ever taken.
    Disabled,
    /// Listening for existing claims before choosing an address.
    Scanning,
    /// A claim has been broadcast and is awaiting the uncontested timeout.
    Pending,
    /// The address is ours.
    Claimed,
    /// No address could be claimed (bus full, or lost while not
    /// arbitrary-address-capable). The node is silent on the wire.
    Failed,
}

/// The ISO 11783-5 address-claim state machine for one NAME.
pub struct AddressClaim {
    name: u64,
    address: u8,
    preferred: u8,
    state: ClaimState,
    deadline: u64,
    /// The NAME's arbitrary-address-capable bit: when set we move to
    /// another free address on losing a conflict, otherwise we go silent.
    arbitrary: bool,
    /// When each address was last claimed by someone else (a 60928 heard,
    /// or a contest lost); `None` if never. See [`USED_TTL_MS`].
    last_seen: [Option<u64>; 256],
}

impl AddressClaim {
    /// A claimer for `name` that prefers source address `preferred`.
    /// `arbitrary` is the NAME's arbitrary-address-capable bit. Call
    /// [`start`](Self::start) to begin the handshake.
    pub fn new(name: u64, preferred: u8, arbitrary: bool) -> Self {
        Self {
            name,
            address: preferred,
            preferred,
            state: ClaimState::Pending,
            deadline: 0,
            arbitrary,
            last_seen: [None; 256],
        }
    }

    /// A claimer for a built [`Name`](crate::io::name::Name), preferring source
    /// address `preferred` and taking the NAME's own arbitrary-address-capable
    /// bit. The ergonomic pairing of [`Name`](crate::io::name::Name) and this
    /// state machine.
    pub fn for_name(name: &crate::io::name::Name, preferred: u8) -> Self {
        Self::new(
            name.to_u64(),
            preferred,
            name.is_arbitrary_address_capable(),
        )
    }

    /// A disabled (passive-sniff) claimer that never takes an address.
    pub fn disabled(name: u64) -> Self {
        Self {
            state: ClaimState::Disabled,
            ..Self::new(name, 0, true)
        }
    }

    /// Current handshake state.
    pub fn state(&self) -> ClaimState {
        self.state
    }

    /// True once the address is owned.
    pub fn is_claimed(&self) -> bool {
        self.state == ClaimState::Claimed
    }

    /// The claimed source address, or `None` until one is owned.
    pub fn address(&self) -> Option<u8> {
        self.is_claimed().then_some(self.address)
    }

    /// The 64-bit ISO NAME this node claims under.
    pub fn name(&self) -> u64 {
        self.name
    }

    /// The next deadline (epoch ms) while [`Scanning`](ClaimState::Scanning)
    /// or [`Pending`](ClaimState::Pending); meaningless otherwise. A caller
    /// with its own event loop can use this to wake exactly on time.
    pub fn deadline(&self) -> u64 {
        self.deadline
    }

    /// True while a scan/claim deadline is running.
    pub fn is_timing(&self) -> bool {
        matches!(self.state, ClaimState::Scanning | ClaimState::Pending)
    }

    /// Begin the handshake: scan the bus for existing claims. Returns the
    /// ISO Request (sent from the null address, since we own none yet)
    /// asking every node to (re)announce its address claim.
    pub fn start(&mut self, now: u64) -> Vec<RawFrame> {
        if self.state == ClaimState::Disabled {
            return Vec::new();
        }
        self.state = ClaimState::Scanning;
        self.deadline = now + SCAN_TIMEOUT_MS;
        vec![request_frame(ADDR_NULL, ADDR_GLOBAL, PGN_ISO_ADDRESS_CLAIM)]
    }

    /// Feed an inbound PGN 60928 Address Claim from `src` carrying the
    /// 64-bit `their_name`. Learns the address as used and, if it collides
    /// with ours, arbitrates by NAME (lowest wins). Returns any (re)claim
    /// frame to broadcast.
    pub fn on_address_claim(&mut self, now: u64, src: u8, their_name: u64) -> Vec<RawFrame> {
        if src > ADDR_MAX || self.state == ClaimState::Disabled {
            return Vec::new();
        }
        // Our own claim, echoed back to us (e.g. a synthetic node whose
        // frames the pipeline feeds back through its own processing path).
        // ISO NAMEs are globally unique, so an identical NAME is never a
        // real peer — treating it as a conflict would make us "lose" to
        // ourselves and walk through every address. Ignore it.
        if their_name == self.name {
            return Vec::new();
        }
        // While scanning we own no address yet — just learn what's in use.
        if self.state == ClaimState::Scanning || src != self.address {
            self.last_seen[src as usize] = Some(now);
            return Vec::new();
        }
        // Contention on our address. Lowest NAME wins (ISO 11783-5).
        if self.name < their_name {
            log::info!(
                "address claim: kept address {src} against a higher NAME ({}); ours is {}",
                describe_name(their_name),
                describe_name(self.name)
            );
            self.state = ClaimState::Pending;
            self.deadline = now + CLAIM_TIMEOUT_MS;
            return vec![self.claim_frame()];
        }
        // We lost: yield the address.
        self.last_seen[src as usize] = Some(now);
        if self.arbitrary
            && let Some(next) = self.pick_free(now)
        {
            log::warn!(
                "address claim: lost address {src} to a lower NAME ({}); ours ({}) moves to {next}",
                describe_name(their_name),
                describe_name(self.name)
            );
            self.address = next;
            self.state = ClaimState::Pending;
            self.deadline = now + CLAIM_TIMEOUT_MS;
            return vec![self.claim_frame()];
        }
        // Nowhere to go (bus full or not arbitrary-address-capable).
        log::warn!(
            "address claim: lost address {src} to a lower NAME ({}); ours ({}) has no other \
             address to go to and goes silent",
            describe_name(their_name),
            describe_name(self.name)
        );
        self.address = ADDR_NULL;
        self.state = ClaimState::Failed;
        vec![self.claim_frame()] // "cannot claim" — broadcast from the null address
    }

    /// The response to an ISO Request for PGN 60928: our current claim if
    /// we have, or are establishing, one. The caller decides whether the
    /// request was addressed to us / broadcast before calling.
    pub fn respond_to_claim_request(&self) -> Option<RawFrame> {
        matches!(self.state, ClaimState::Claimed | ClaimState::Pending).then(|| self.claim_frame())
    }

    /// Advance the deadline-driven transitions (scan→claim, claim→owned).
    /// Returns any claim frame to broadcast. To detect the moment the
    /// address becomes owned (to announce Product Information, start
    /// heartbeat timers, …), compare [`is_claimed`](Self::is_claimed)
    /// across the call.
    pub fn tick(&mut self, now: u64) -> Vec<RawFrame> {
        match self.state {
            ClaimState::Scanning if now >= self.deadline => self.begin_claim(now),
            ClaimState::Pending if now >= self.deadline => {
                self.state = ClaimState::Claimed;
                Vec::new()
            }
            _ => Vec::new(),
        }
    }

    /// Pick an address (preferred if free, else the lowest free one) and
    /// broadcast the claim.
    fn begin_claim(&mut self, now: u64) -> Vec<RawFrame> {
        if self.in_use(self.preferred, now) {
            match self.pick_free(now) {
                Some(next) => self.address = next,
                None => {
                    self.address = ADDR_NULL;
                    self.state = ClaimState::Failed;
                    return vec![self.claim_frame()];
                }
            }
        } else {
            self.address = self.preferred;
        }
        self.state = ClaimState::Pending;
        self.deadline = now + CLAIM_TIMEOUT_MS;
        vec![self.claim_frame()]
    }

    /// Whether someone else claimed `address` within [`USED_TTL_MS`].
    fn in_use(&self, address: u8, now: u64) -> bool {
        self.last_seen[address as usize].is_some_and(|seen| now.saturating_sub(seen) < USED_TTL_MS)
    }

    /// The first free address upward from the preferred one, wrapping
    /// round. Not the lowest free address: that is where real devices
    /// like to sit, so a node parked there keeps being displaced.
    fn pick_free(&self, now: u64) -> Option<u8> {
        (self.preferred..=ADDR_MAX)
            .chain(0..self.preferred)
            .find(|&a| !self.in_use(a, now))
    }

    fn claim_frame(&self) -> RawFrame {
        RawFrame::new(
            None,
            6,
            PGN_ISO_ADDRESS_CLAIM,
            self.address,
            ADDR_GLOBAL,
            self.name.to_le_bytes(),
        )
    }
}

/// A NAME for the log: the raw 64-bit value, which is what arbitration
/// compares, plus its fields decoded.
fn describe_name(name: u64) -> String {
    format!(
        "{name:#018x}: manufacturer {}, class {}, function {}, instance {}, \
         system instance {}, industry {}, unique {:#x}",
        (name >> 21) & 0x7ff,
        (name >> 49) & 0x7f,
        (name >> 40) & 0xff,
        (name >> 32) & 0xff,
        (name >> 56) & 0x0f,
        (name >> 60) & 0x07,
        name & 0x1f_ffff
    )
}

/// A PGN 59904 ISO Request for `pgn`, from `src` to `dst`.
fn request_frame(src: u8, dst: u8, pgn: u32) -> RawFrame {
    RawFrame::new(
        None,
        6,
        PGN_ISO_REQUEST,
        src,
        dst,
        [pgn as u8, (pgn >> 8) as u8, (pgn >> 16) as u8],
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    // Two arbitrary NAMEs; LOW arbitrates as the winner (lower u64).
    const LOW: u64 = 0x00AA_BBCC_DDEE_FF00;
    const HIGH: u64 = 0x80AA_BBCC_DDEE_FF00;

    fn claim_frames(frames: &[RawFrame]) -> Vec<(u8, u8)> {
        frames
            .iter()
            .filter(|f| f.pgn == PGN_ISO_ADDRESS_CLAIM)
            .map(|f| (f.src, f.dst))
            .collect()
    }

    #[test]
    fn scan_then_claim_preferred() {
        let mut c = AddressClaim::new(LOW, 42, true);
        let out = c.start(0);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].pgn, PGN_ISO_REQUEST);
        assert_eq!(c.state(), ClaimState::Scanning);
        // Scan window elapses → claim the preferred address.
        let out = c.tick(SCAN_TIMEOUT_MS);
        assert_eq!(claim_frames(&out), vec![(42, ADDR_GLOBAL)]);
        assert_eq!(c.state(), ClaimState::Pending);
        assert_eq!(c.address(), None, "not owned until the claim settles");
        // Claim window elapses uncontested → owned.
        let out = c.tick(SCAN_TIMEOUT_MS + CLAIM_TIMEOUT_MS);
        assert!(out.is_empty());
        assert!(c.is_claimed());
        assert_eq!(c.address(), Some(42));
    }

    #[test]
    fn scan_learns_used_and_avoids_preferred() {
        let mut c = AddressClaim::new(LOW, 42, true);
        c.start(0);
        // 42 is already claimed by someone else during the scan.
        assert!(c.on_address_claim(10, 42, HIGH).is_empty());
        let out = c.tick(SCAN_TIMEOUT_MS);
        // Preferred 42 is taken → the next free address up from it.
        assert_eq!(claim_frames(&out), vec![(43, ADDR_GLOBAL)]);
    }

    #[test]
    fn wins_conflict_with_lower_name() {
        let mut c = AddressClaim::new(LOW, 42, true);
        c.start(0);
        c.tick(SCAN_TIMEOUT_MS); // Pending on 42
        // A higher NAME claims 42: we win and re-claim 42.
        let out = c.on_address_claim(SCAN_TIMEOUT_MS, 42, HIGH);
        assert_eq!(claim_frames(&out), vec![(42, ADDR_GLOBAL)]);
        assert_eq!(c.state(), ClaimState::Pending);
    }

    #[test]
    fn loses_conflict_and_moves_when_arbitrary() {
        let mut c = AddressClaim::new(HIGH, 42, true);
        c.start(0);
        c.tick(SCAN_TIMEOUT_MS); // Pending on 42
        // A lower NAME claims 42: we lose and move to another free address.
        let out = c.on_address_claim(SCAN_TIMEOUT_MS, 42, LOW);
        assert_eq!(
            claim_frames(&out),
            vec![(43, ADDR_GLOBAL)],
            "moved up from the contested address, not down to 0"
        );
        assert_eq!(c.state(), ClaimState::Pending);
    }

    #[test]
    fn the_search_for_a_free_address_wraps_round() {
        let mut c = AddressClaim::new(LOW, ADDR_MAX, true);
        c.start(0);
        c.on_address_claim(10, ADDR_MAX, HIGH);
        let out = c.tick(SCAN_TIMEOUT_MS);
        assert_eq!(claim_frames(&out), vec![(0, ADDR_GLOBAL)]);
    }

    /// An address whose claim has not been heard for USED_TTL_MS is free
    /// again: devices never say they leave.
    #[test]
    fn an_address_not_heard_from_is_free_again() {
        let mut c = AddressClaim::new(HIGH, 42, true);
        c.start(0);
        c.tick(SCAN_TIMEOUT_MS); // Pending on 42
        c.on_address_claim(SCAN_TIMEOUT_MS, 42, LOW); // lose 42 → 43
        assert_eq!(c.pick_free(SCAN_TIMEOUT_MS + 1), Some(43));
        assert_eq!(
            c.pick_free(SCAN_TIMEOUT_MS + USED_TTL_MS),
            Some(42),
            "the winner has not re-claimed 42 for a TTL"
        );
    }

    #[test]
    fn a_name_is_described_by_the_fields_that_arbitrate() {
        let name = crate::io::name::Name::new(381, 0x1234)
            .device_function(140)
            .device_class(60)
            .to_u64();
        assert_eq!(
            describe_name(name),
            format!(
                "{name:#018x}: manufacturer 381, class 60, function 140, instance 0, \
                 system instance 0, industry 4, unique 0x1234"
            )
        );
    }

    /// NAMEs that differ only in system instance arbitrate differently,
    /// so the log must tell them apart.
    #[test]
    fn a_description_shows_the_system_instance() {
        let base = crate::io::name::Name::new(999, 0x1234).device_class(25);
        let a = describe_name(base.to_u64());
        let b = describe_name(base.system_instance(15).to_u64());
        assert_ne!(a, b);
        assert!(b.contains("system instance 15"), "{b}");
    }

    #[test]
    fn loses_conflict_and_fails_when_not_arbitrary() {
        let mut c = AddressClaim::new(HIGH, 42, false);
        c.start(0);
        c.tick(SCAN_TIMEOUT_MS);
        let out = c.on_address_claim(SCAN_TIMEOUT_MS, 42, LOW);
        assert_eq!(claim_frames(&out), vec![(ADDR_NULL, ADDR_GLOBAL)]);
        assert_eq!(c.state(), ClaimState::Failed);
        assert_eq!(c.address(), None);
    }

    #[test]
    fn responds_to_claim_request_only_once_owned_or_pending() {
        let mut c = AddressClaim::new(LOW, 42, true);
        c.start(0);
        assert!(
            c.respond_to_claim_request().is_none(),
            "scanning: nothing to answer with"
        );
        c.tick(SCAN_TIMEOUT_MS); // Pending
        assert!(c.respond_to_claim_request().is_some());
        c.tick(SCAN_TIMEOUT_MS + CLAIM_TIMEOUT_MS); // Claimed
        let f = c.respond_to_claim_request().unwrap();
        assert_eq!(f.pgn, PGN_ISO_ADDRESS_CLAIM);
        assert_eq!(f.src, 42);
    }

    #[test]
    fn ignores_its_own_claim_echoed_back() {
        // A synthetic node's own 60928 claim is fed back to it (via the
        // pipeline). It must NOT be read as a conflict — otherwise the node
        // "loses" to itself and walks through every address.
        let mut c = AddressClaim::new(LOW, 42, true);
        c.start(0);
        c.tick(SCAN_TIMEOUT_MS); // claim 42
        c.tick(SCAN_TIMEOUT_MS + CLAIM_TIMEOUT_MS); // owned
        assert_eq!(c.address(), Some(42));
        // Our own claim (same NAME, our address) comes back: no-op.
        let out = c.on_address_claim(5000, 42, LOW);
        assert!(out.is_empty());
        assert_eq!(c.address(), Some(42), "still owns 42, did not move");
    }

    #[test]
    fn disabled_never_claims() {
        let mut c = AddressClaim::disabled(LOW);
        assert!(c.start(0).is_empty());
        assert!(c.on_address_claim(0, 42, HIGH).is_empty());
        assert!(c.tick(10_000).is_empty());
        assert_eq!(c.state(), ClaimState::Disabled);
        assert_eq!(c.address(), None);
    }
}
