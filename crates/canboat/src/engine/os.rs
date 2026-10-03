// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

use std::path::Path;
#[cfg(any(target_os = "macos", target_os = "windows"))]
use std::process::Command;
use std::sync::Mutex;

const APP_SALT: &str = "canboat";

/// The ISO NAME's 21-bit unique number for this machine, the same way
/// canboatjs derives it (canboatjs `lib/machineId.ts`):
///
/// 1. Derived from the OS machine id: the low 21 bits of FNV-1a 64 over
///    `canboat|<id>`. Stable across restarts and not stored, so a copied
///    configuration does not copy the NAME onto another machine.
/// 2. Only when the machine can't be identified (no usable OS id, such as
///    an OS image that has not booted yet): a random number, stored as
///    `unique-number` in `state_dir` so it survives restarts. Without a
///    `state_dir`, or if storing fails, it holds for this run only.
///
/// Never all ones (0x1fffff), the unset value some analyzers hide a
/// device for. canboatjs salts with `canboatjs`, so a canboat server and a
/// canboatjs device on one computer get different NAMEs.
pub fn unique_number(state_dir: Option<&Path>) -> u32 {
    match machine_string() {
        Some(id) => unique_number_for(&id),
        None => stored_unique_number(state_dir),
    }
}

const ALL_ONES: u32 = 0x1f_ffff;
const UNIQUE_NUMBER_FILE: &str = "unique-number";

fn unique_number_for(machine: &str) -> u32 {
    let unique = (fnv1a_64(format!("{APP_SALT}|{machine}").as_bytes()) as u32) & ALL_ONES;
    if unique == ALL_ONES {
        ALL_ONES - 1
    } else {
        unique
    }
}

/// The stored random unique number, drawing and storing one on first use.
/// Serialised, so two devices in one process don't each store their own;
/// across processes [`store_unique_number`] lets only the first one store.
/// A number that can't be stored is kept for the rest of the run.
fn stored_unique_number(state_dir: Option<&Path>) -> u32 {
    static UNSTORED: Mutex<Option<u32>> = Mutex::new(None);
    let mut unstored = UNSTORED.lock().unwrap_or_else(|e| e.into_inner());
    let path = state_dir.map(|d| d.join(UNIQUE_NUMBER_FILE));
    if let Some(n) = path.as_deref().and_then(read_unique_number) {
        return n;
    }
    if let Some(n) = *unstored {
        return n;
    }
    let n = random_unique_number();
    match path.as_deref().map(|p| store_unique_number(p, n)) {
        Some(Ok(stored)) => {
            log::info!(
                "cannot identify this machine; stored random unique number {stored} in {}",
                path.as_deref().unwrap().display()
            );
            return stored;
        }
        Some(Err(e)) => log::warn!(
            "cannot identify this machine, nor store random unique number {n} in {}: {e}; \
             pass --unique to keep the NAME across restarts",
            path.as_deref().unwrap().display()
        ),
        None => log::warn!(
            "cannot identify this machine; using random unique number {n} for this run \
             (pass --unique to keep the NAME across restarts)"
        ),
    }
    *unstored = Some(n);
    n
}

fn read_unique_number(path: &Path) -> Option<u32> {
    let n: u32 = std::fs::read_to_string(path).ok()?.trim().parse().ok()?;
    (n < ALL_ONES).then_some(n)
}

/// Store `n` at `path` unless another process stored one first, and return
/// the number that is stored. The number is written to a temporary file
/// and hard-linked into place: the link is atomic and fails if `path`
/// exists, so exactly one of several processes starting at once stores
/// its number, and the others read it back complete. A `path` holding no
/// valid number is replaced, as it is on a filesystem without hard links.
fn store_unique_number(path: &Path, n: u32) -> std::io::Result<u32> {
    let dir = path.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(dir)?;
    let tmp = dir.join(format!(".{UNIQUE_NUMBER_FILE}.{}.tmp", std::process::id()));
    std::fs::write(&tmp, format!("{n}\n"))?;
    let stored = match std::fs::hard_link(&tmp, path) {
        Ok(()) => Ok(n),
        Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => match read_unique_number(path) {
            Some(theirs) => Ok(theirs),
            None => std::fs::rename(&tmp, path).map(|()| n),
        },
        Err(_) => std::fs::rename(&tmp, path).map(|()| n),
    };
    let _ = std::fs::remove_file(&tmp); // gone already after a rename
    stored
}

/// A random unique number in 0..0x1fffff (all ones excluded), without a
/// dependency: std seeds each `RandomState` from the OS random source.
fn random_unique_number() -> u32 {
    use std::hash::{BuildHasher, Hasher};
    let mut h = std::collections::hash_map::RandomState::new().build_hasher();
    h.write(APP_SALT.as_bytes());
    (h.finish() % u64::from(ALL_ONES)) as u32
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
            let s = s.trim();
            if is_machine_id(s) {
                return Some(s.to_string());
            }
        }
    }
    None
}

/// A usable systemd/D-Bus machine id: 32 lowercase hex digits, not all
/// zeros. An image that has not booted yet holds `uninitialized` or all
/// zeros, which every machine made from it shares.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn is_machine_id(s: &str) -> bool {
    s.len() == 32
        && s.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'))
        && s.bytes().any(|b| b != b'0')
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
    Some(line[start..end].to_string()).filter(|s| !s.is_empty())
}

// Targets without an OS machine identifier (wasm32, BSDs, …): no
// fingerprint. unique_number() then falls back to a stored random
// number. The wasm bindings never ask (pure decode/encode, no bus
// identity).
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
    Some(line.split_whitespace().last()?.to_string()).filter(|s| !s.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fnv1a_matches_the_reference_vectors() {
        assert_eq!(fnv1a_64(b""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(fnv1a_64(b"a"), 0xaf63_dc4c_8601_ec8c);
        assert_eq!(fnv1a_64(b"foobar"), 0x8594_4171_f739_67e8);
    }

    #[test]
    fn unique_number_is_the_low_21_bits_of_the_salted_hash() {
        let machine = "4c4c4544003210508051b2c04f4a3132";
        let n = unique_number_for(machine);
        assert_eq!(
            n,
            (fnv1a_64(format!("canboat|{machine}").as_bytes()) & 0x1f_ffff) as u32
        );
        // canboatjs salts with "canboatjs", so the two differ.
        assert_ne!(
            n,
            (fnv1a_64(format!("canboatjs|{machine}").as_bytes()) & 0x1f_ffff) as u32
        );
    }

    #[test]
    fn unique_number_is_never_all_ones() {
        let all_ones = (0u32..)
            .map(|i| i.to_string())
            .find(|m| fnv1a_64(format!("canboat|{m}").as_bytes()) & 0x1f_ffff == 0x1f_ffff)
            .unwrap();
        assert_eq!(unique_number_for(&all_ones), 0x1f_fffe);
    }

    #[test]
    fn machine_id_accepts_systemd_id() {
        assert!(is_machine_id("4c4c4544003210508051b2c04f4a3132"));
    }

    #[test]
    fn machine_id_refuses_unbooted_image() {
        assert!(!is_machine_id(""));
        assert!(!is_machine_id("uninitialized"));
        assert!(!is_machine_id(&"0".repeat(32)));
        assert!(!is_machine_id("4C4C4544003210508051B2C04F4A3132"));
        assert!(!is_machine_id("4c4c4544003210508051b2c04f4a313"));
        assert!(!is_machine_id("4c4c4544003210508051b2c04f4a313g"));
    }

    #[test]
    fn random_unique_number_is_stored_and_reused() {
        let dir = std::env::temp_dir().join(format!("canboat-unique-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let n = stored_unique_number(Some(&dir));
        assert!(n < 0x1f_ffff);
        let stored = std::fs::read_to_string(dir.join(UNIQUE_NUMBER_FILE)).unwrap();
        assert_eq!(stored.trim().parse::<u32>().unwrap(), n);
        assert_eq!(stored_unique_number(Some(&dir)), n);
        // A stored one is kept, even one this run did not draw.
        std::fs::write(dir.join(UNIQUE_NUMBER_FILE), "424242\n").unwrap();
        assert_eq!(stored_unique_number(Some(&dir)), 424242);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// Another process that stored its number first wins: ours is not
    /// written over it, and the temporary file is gone.
    #[test]
    fn the_first_stored_unique_number_wins() {
        let dir = std::env::temp_dir().join(format!("canboat-unique-race-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let path = dir.join(UNIQUE_NUMBER_FILE);
        assert_eq!(store_unique_number(&path, 1234).unwrap(), 1234);
        assert_eq!(store_unique_number(&path, 5678).unwrap(), 1234);
        assert_eq!(read_unique_number(&path), Some(1234));
        // A file holding no valid number is replaced.
        std::fs::write(&path, "x").unwrap();
        assert_eq!(store_unique_number(&path, 5678).unwrap(), 5678);
        assert_eq!(read_unique_number(&path), Some(5678));
        let names: Vec<_> = std::fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .collect();
        assert_eq!(names, [UNIQUE_NUMBER_FILE], "no temporary file left");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_stored_all_ones_or_garbage_is_ignored() {
        let dir = std::env::temp_dir().join(format!("canboat-unique-bad-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(UNIQUE_NUMBER_FILE);
        for bad in ["2097151", "x", ""] {
            std::fs::write(&path, bad).unwrap();
            assert_eq!(read_unique_number(&path), None, "{bad:?}");
        }
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn random_unique_numbers_differ() {
        assert_ne!(random_unique_number(), random_unique_number());
    }
}
