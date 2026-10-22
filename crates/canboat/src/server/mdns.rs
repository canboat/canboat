// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The server's mDNS / DNS-SD advert: one `_canboat._tcp` instance per
//! running server, so a client on the LAN finds it without being told a
//! host and port.
//!
//! The SRV record points at the analyzer port (or the first port that is
//! open when that one is off). The TXT record carries:
//!
//! * `version`: the canboat version;
//! * `name` and `serial`: canboat's own ISO NAME (16 hex digits) and the
//!   Model Serial Code in its Product Information, when it runs a node on
//!   the bus, so a client can match this server with what other devices
//!   report about the bus;
//! * one key per open port, named after its option without `-port`
//!   (`analyzer=2598`, `nmea0183-filter=2605`, …);
//! * `quirks`: the quirks that are on, by name, comma-separated;
//! * whatever the quirks set themselves (`mcu006.name`, `mcu006.host`,
//!   `motion.name`), through [`Txt::set`]. A change is announced again.
//!
//! [`Txt`] exists whether or not the advert runs, so a quirk sets its keys
//! the same way either way; without the `mdns` feature it only keeps them.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use crate::server::quirks::QuirkKind;

/// The DNS-SD service type.
pub const SERVICE_TYPE: &str = "_canboat._tcp.local.";

/// The TXT keys the quirks set at runtime, shared between the quirks and
/// the advert. Cloning shares.
#[derive(Clone, Default)]
pub struct Txt(Arc<Mutex<TxtState>>);

#[derive(Default)]
struct TxtState {
    keys: BTreeMap<String, String>,
    #[cfg(feature = "mdns")]
    advert: Option<Advert>,
}

impl Txt {
    /// Set `key` to `value`, or remove it with `None`. A running advert
    /// announces the change.
    pub fn set(&self, key: &str, value: Option<String>) {
        let mut state = self.0.lock().unwrap_or_else(|e| e.into_inner());
        let changed = match value {
            Some(v) => state.keys.insert(key.to_string(), v.clone()) != Some(v),
            None => state.keys.remove(key).is_some(),
        };
        #[cfg(feature = "mdns")]
        if changed {
            let state = &mut *state;
            if let Some(advert) = &state.advert {
                advert.announce(&state.keys);
            }
        }
        #[cfg(not(feature = "mdns"))]
        let _ = changed;
    }

    /// Start advertising `base` (see [`txt_record`]) on `port`, plus the
    /// quirks' keys. `bind` limits it to that interface unless it is
    /// `0.0.0.0`. Replaces an advert already running.
    #[cfg(feature = "mdns")]
    pub fn start(
        &self,
        bind: std::net::Ipv4Addr,
        port: u16,
        base: Vec<(String, String)>,
    ) -> anyhow::Result<()> {
        let advert = Advert::new(bind, port, base)?;
        let mut state = self.0.lock().unwrap_or_else(|e| e.into_inner());
        advert.announce(&state.keys);
        log::info!(
            "mDNS: advertising {:?} ({SERVICE_TYPE}) on port {port}",
            advert.instance
        );
        if let Some(old) = state.advert.replace(advert) {
            old.withdraw();
        }
        Ok(())
    }

    /// Withdraw the advert (a goodbye packet) and stop the responder.
    /// Idempotent.
    pub fn stop(&self) {
        #[cfg(feature = "mdns")]
        {
            let advert = self
                .0
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .advert
                .take();
            if let Some(advert) = advert {
                advert.withdraw();
            }
        }
    }
}

/// canboat's own identity on the bus: the NAME its node claims, and the
/// Model Serial Code it sends in PGN 126996 (NMEA 2000 only).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Identity {
    pub name: u64,
    pub nmea2000: bool,
}

/// A NAME as it appears in the TXT record: 16 lowercase hex digits.
pub fn name_hex(name: u64) -> String {
    format!("{name:016x}")
}

/// The TXT record's fixed keys, in order: `version`, the identity, a key
/// per open port, then `quirks`.
pub fn txt_record(
    identity: Option<Identity>,
    ports: &[(&str, u16)],
    quirks: &[QuirkKind],
) -> Vec<(String, String)> {
    let mut txt = vec![("version".to_string(), env!("CARGO_PKG_VERSION").to_string())];
    if let Some(id) = identity {
        txt.push(("name".into(), name_hex(id.name)));
        if id.nmea2000 {
            // As `iso11783_node` sends it: the NAME's 21-bit unique number.
            txt.push(("serial".into(), (id.name & 0x1f_ffff).to_string()));
        }
    }
    txt.extend(
        ports
            .iter()
            .filter(|(_, port)| *port != 0)
            .map(|(key, port)| (key.to_string(), port.to_string())),
    );
    let mut names: Vec<&str> = Vec::new();
    for quirk in quirks {
        if !names.contains(&quirk.name()) {
            names.push(quirk.name());
        }
    }
    if !names.is_empty() {
        txt.push(("quirks".into(), names.join(",")));
    }
    txt
}

/// The port the SRV record names: the analyzer's, or else the first open.
pub fn srv_port(ports: &[(&str, u16)]) -> Option<u16> {
    let open = || ports.iter().filter(|(_, port)| *port != 0);
    open()
        .find(|(key, _)| *key == "analyzer")
        .or_else(|| open().next())
        .map(|(_, port)| *port)
}

/// The registered service and the responder that answers for it.
#[cfg(feature = "mdns")]
struct Advert {
    daemon: mdns_sd::ServiceDaemon,
    instance: String,
    host: String,
    bind: std::net::Ipv4Addr,
    port: u16,
    base: Vec<(String, String)>,
}

#[cfg(feature = "mdns")]
impl Advert {
    fn new(
        bind: std::net::Ipv4Addr,
        port: u16,
        base: Vec<(String, String)>,
    ) -> anyhow::Result<Self> {
        let daemon = mdns_sd::ServiceDaemon::new()?;
        let host = hostname();
        Ok(Self {
            daemon,
            instance: format!("canboat on {host}"),
            host: format!("{host}.local."),
            bind,
            port,
            base,
        })
    }

    /// Register, or register again with new TXT keys: `mdns-sd` announces
    /// the updated records. A clash gets DNS-SD's ` (2)` suffix.
    fn announce(&self, quirk_keys: &BTreeMap<String, String>) {
        let properties: Vec<(&str, &str)> = self
            .base
            .iter()
            .map(|(k, v)| (k, v))
            .chain(quirk_keys)
            .map(|(k, v)| (k.as_str(), v.as_str()))
            .collect();
        let info = if self.bind.is_unspecified() {
            mdns_sd::ServiceInfo::new(
                SERVICE_TYPE,
                &self.instance,
                &self.host,
                (),
                self.port,
                &properties[..],
            )
            .map(mdns_sd::ServiceInfo::enable_addr_auto)
        } else {
            mdns_sd::ServiceInfo::new(
                SERVICE_TYPE,
                &self.instance,
                &self.host,
                std::net::IpAddr::V4(self.bind),
                self.port,
                &properties[..],
            )
            .map(|mut info| {
                info.set_interfaces(vec![mdns_sd::IfKind::Addr(self.bind.into())]);
                info
            })
        };
        let result = info.and_then(|info| self.daemon.register(info));
        if let Err(e) = result {
            log::warn!("mDNS: cannot advertise {:?}: {e}", self.instance);
        }
    }

    /// Unregister (which sends the goodbye) and wait briefly for it to go
    /// out before stopping the responder.
    fn withdraw(self) {
        let fullname = format!("{}.{SERVICE_TYPE}", self.instance);
        if let Ok(done) = self.daemon.unregister(&fullname) {
            let _ = done.recv_timeout(std::time::Duration::from_secs(1));
        }
        if let Ok(done) = self.daemon.shutdown() {
            let _ = done.recv_timeout(std::time::Duration::from_secs(1));
        }
    }
}

/// This host's name as one DNS label: the part before any domain, with
/// anything but letters, digits and `-` turned into `-`.
#[cfg(feature = "mdns")]
fn hostname() -> String {
    let raw = system_hostname().unwrap_or_default();
    let label: String = raw
        .split('.')
        .next()
        .unwrap_or_default()
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '-' })
        .collect();
    let label = label.trim_matches('-');
    if label.is_empty() {
        "canboat".to_string()
    } else {
        label.to_string()
    }
}

#[cfg(all(feature = "mdns", unix))]
fn system_hostname() -> Option<String> {
    let mut buf = [0u8; 256];
    // SAFETY: the buffer is valid for its length; gethostname writes at
    // most that many bytes.
    let rc = unsafe { libc::gethostname(buf.as_mut_ptr().cast(), buf.len()) };
    if rc != 0 {
        return None;
    }
    let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    Some(String::from_utf8_lossy(&buf[..len]).into_owned())
}

#[cfg(all(feature = "mdns", not(unix)))]
fn system_hostname() -> Option<String> {
    std::env::var("COMPUTERNAME").ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::quirk::Target;

    const PORTS: [(&str, u16); 8] = [
        ("snapshot", 2597),
        ("input", 2600),
        ("raw", 2603),
        ("analyzer", 2598),
        ("nmea0183", 0),
        ("ais", 2601),
        ("nmea0183-filter", 2605),
        ("mcu006", 2607),
    ];

    #[test]
    fn txt_record_carries_identity_ports_and_quirks() {
        let identity = Identity {
            name: 0xc0_32_00_ff_e1_23_45_67,
            nmea2000: true,
        };
        let quirks = [
            QuirkKind::Mcu006,
            QuirkKind::GpsRollover(Target::All),
            QuirkKind::Wmm,
        ];
        let txt = txt_record(Some(identity), &PORTS, &quirks);
        let txt: Vec<(&str, &str)> = txt.iter().map(|(k, v)| (k.as_str(), v.as_str())).collect();
        assert_eq!(
            txt,
            [
                ("version", env!("CARGO_PKG_VERSION")),
                ("name", "c03200ffe1234567"),
                ("serial", &(0x0123_4567 & 0x1f_ffff).to_string()),
                ("snapshot", "2597"),
                ("input", "2600"),
                ("raw", "2603"),
                ("analyzer", "2598"),
                ("ais", "2601"),
                ("nmea0183-filter", "2605"),
                ("mcu006", "2607"),
                ("quirks", "mcu006,gps-rollover,wmm"),
            ]
        );
    }

    #[test]
    fn txt_record_without_a_node_or_quirks() {
        let txt = txt_record(None, &[("analyzer", 2598)], &[]);
        let keys: Vec<&str> = txt.iter().map(|(k, _)| k.as_str()).collect();
        assert_eq!(keys, ["version", "analyzer"]);
        // A J1939 node sends no Product Information, so has no serial.
        let j1939 = Identity {
            name: 1,
            nmea2000: false,
        };
        let txt = txt_record(Some(j1939), &[], &[]);
        let keys: Vec<&str> = txt.iter().map(|(k, _)| k.as_str()).collect();
        assert_eq!(keys, ["version", "name"]);
    }

    #[test]
    fn srv_port_prefers_the_analyzer() {
        assert_eq!(srv_port(&PORTS), Some(2598));
        assert_eq!(
            srv_port(&[("snapshot", 0), ("analyzer", 0), ("raw", 2603)]),
            Some(2603)
        );
        assert_eq!(srv_port(&[("analyzer", 0)]), None);
    }

    #[test]
    fn txt_set_reports_only_changes() {
        let txt = Txt::default();
        txt.set("mcu006.host", Some("00".into()));
        txt.set("mcu006.host", Some("00".into()));
        txt.set("motion.name", Some("01".into()));
        txt.set("motion.name", None);
        let keys = txt.0.lock().unwrap().keys.clone();
        assert_eq!(
            keys.into_iter().collect::<Vec<_>>(),
            [("mcu006.host".to_string(), "00".to_string())]
        );
    }

    /// Advertise on the real network and browse for it. Needs multicast,
    /// which CI may not have: `cargo test -- --ignored mdns`.
    #[cfg(feature = "mdns")]
    #[test]
    #[ignore]
    fn mdns_browse_finds_the_advert() {
        use std::time::{Duration, Instant};
        let txt = Txt::default();
        let base = txt_record(None, &[("analyzer", 42598)], &[QuirkKind::Wmm]);
        txt.start(std::net::Ipv4Addr::UNSPECIFIED, 42598, base)
            .expect("advert");
        txt.set("test.key", Some("1".into()));

        let browser = mdns_sd::ServiceDaemon::new().expect("browser");
        let events = browser.browse(SERVICE_TYPE).expect("browse");
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut found = None;
        while let Some(left) = deadline.checked_duration_since(Instant::now()) {
            match events.recv_timeout(left) {
                Ok(mdns_sd::ServiceEvent::ServiceResolved(info))
                    if info.get_port() == 42598
                        && info.get_property_val_str("test.key") == Some("1") =>
                {
                    found = Some(info);
                    break;
                }
                Ok(_) => {}
                Err(_) => break,
            }
        }
        let _ = browser.shutdown();
        txt.stop();
        let info = found.expect("the advert was found");
        assert_eq!(info.get_property_val_str("analyzer"), Some("42598"));
        assert_eq!(info.get_property_val_str("quirks"), Some("wmm"));
    }
}
