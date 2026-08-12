// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#[cfg(any(target_os = "macos", target_os = "windows"))]
use std::process::Command;

const APP_SALT: &str = "canboat";

/// Returns a stable per-machine u64 derived from an OS-persisted identifier.
/// Same value across runs; no disk writes of our own.
pub fn get_machine_id() -> u64 {
    let raw = machine_string().unwrap_or_default();
    fnv1a_64(format!("{APP_SALT}|{raw}").as_bytes())
}

/// 64-bit FNV-1a. Not cryptographic — fine for a fingerprint.
/// Swap in SHA-256 (sha2 crate) if you want parity with other languages.
fn fnv1a_64(bytes: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325; // offset basis
    for &b in bytes {
        hash ^= b as u64;
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3); // prime
    }
    hash
}

#[cfg(target_os = "linux")]
fn machine_string() -> Option<String> {
    for p in ["/etc/machine-id", "/var/lib/dbus/machine-id"] {
        if let Ok(s) = std::fs::read_to_string(p) {
            let s = s.trim().to_string();
            if !s.is_empty() {
                return Some(s);
            }
        }
    }
    None
}

#[cfg(target_os = "macos")]
fn machine_string() -> Option<String> {
    let out = Command::new("ioreg")
        .args(["-rd1", "-c", "IOPlatformExpertDevice"])
        .output()
        .ok()?;
    let text = String::from_utf8_lossy(&out.stdout);
    // line looks like:  "IOPlatformUUID" = "XXXXXXXX-...-XXXXXXXXXXXX"
    let line = text.lines().find(|l| l.contains("IOPlatformUUID"))?;
    let start = line.find("= \"")? + 3;
    let end = line[start..].find('"')? + start;
    Some(line[start..end].to_string())
}

// Targets without an OS machine identifier (wasm32, BSDs, …): no
// fingerprint. get_machine_id() then hashes just the salt — stable,
// but IDENTICAL on every host. That is fine for the wasm bindings
// (pure decode/encode, no bus identity), but a bus-facing consumer on
// such a target must supply an explicit unique number for its ISO
// NAME instead of relying on this value, or two hosts on one bus
// would claim with colliding NAMEs.
#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
fn machine_string() -> Option<String> {
    None
}

#[cfg(target_os = "windows")]
fn machine_string() -> Option<String> {
    // Shell out to `reg query` to stay dependency-free.
    let out = Command::new("reg")
        .args([
            "query",
            r"HKLM\SOFTWARE\Microsoft\Cryptography",
            "/v",
            "MachineGuid",
        ])
        .output()
        .ok()?;
    let text = String::from_utf8_lossy(&out.stdout);
    // line looks like:  MachineGuid    REG_SZ    xxxxxxxx-....
    let line = text.lines().find(|l| l.contains("MachineGuid"))?;
    Some(line.split_whitespace().last()?.to_string())
}
