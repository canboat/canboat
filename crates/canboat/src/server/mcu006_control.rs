// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The control port for `--quirk mcu006`: drive the emulated Furuno
//! MCU-006 remote's keys, pointer and knob over TCP.
//!
//! A client writes one JSON object per line and gets one JSON object back:
//!
//! ```text
//! {"key":"F5","down":true}        hold a key (a name or a HID usage ID)
//! {"key":"F5","down":false}       release it
//! {"tap":"Up"}                    press and release
//! {"pointer":{"dx":40,"dy":-10}}  move the cursor
//! {"knob":-2}                     turn the knob two detents
//! {"button":true}                 hold the knob button; false releases it
//! {"click":true}                  press and release the knob button
//! {"keypad":true}                 keypad mode on (CURS/SCROLL); false turns it off
//! {"release":true}                release everything this client holds
//! {"status":true}                 just report the state
//!
//! → {"ok":true,"addr":110,"host":1,"keys":[82],"button":false,"keypad":false}
//! → {"ok":false,"error":"…", …}
//! ```
//!
//! The reports go straight to the CAN writer, not through the pipeline:
//! the quirks only run when a frame arrives, and a quiet bus would make the
//! cursor lag. While the remote is paired, a keepalive thread repeats the
//! held keys, the button and keypad mode every second, as the firmware does
//! (`FUN_0800ccf0`). A client that disconnects has its keys and button
//! released, so a dropped connection cannot leave a key held down. Keypad
//! mode is a setting of the remote, not something a client holds, so it
//! stays as the last client left it.

use std::io::{BufRead, BufReader, Write};
use std::net::{Ipv4Addr, SocketAddrV4, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use anyhow::{Context, Result};
use serde_json::{Value, json};

use super::quirks::mcu006::{Link, LinkState, keyboard_report, pointer_report};
use crate::io::device::FrameSender;
use crate::n2kd::serving::tcp::accept_until;

const KEEPALIVE: Duration = Duration::from_secs(1);
/// How long a `tap` or `click` holds its key or button.
const TAP: Duration = Duration::from_millis(100);
/// The remote reports at most five keys at once.
const MAX_KEYS: usize = 5;
/// The most a `pointer` or `knob` command moves: 16 reports of 127.
const MAX_MOVE: i64 = 16 * 127;

/// What the remote is doing: the keys held (in the order pressed), the
/// button and keypad mode, across every client.
#[derive(Default)]
struct Remote {
    /// Each key held, with how many clients hold it.
    keys: Vec<(u8, usize)>,
    /// How many clients hold the button.
    button: usize,
    /// The CURS/SCROLL button's mode: the joystick as arrow keys.
    keypad: bool,
}

impl Remote {
    /// The keys held, in the order pressed.
    fn codes(&self) -> Vec<u8> {
        self.keys.iter().map(|&(code, _)| code).collect()
    }

    /// One client lets go of `code`; the key is released when the last
    /// one does.
    fn drop_key(&mut self, code: u8) {
        if let Some(i) = self.keys.iter().position(|&(k, _)| k == code) {
            self.keys[i].1 -= 1;
            if self.keys[i].1 == 0 {
                self.keys.remove(i);
            }
        }
    }

    /// The pointer report's flags: the button and keypad mode.
    fn flags(&self) -> (bool, bool) {
        (self.button > 0, self.keypad)
    }
}

struct Control {
    link: Link,
    sender: FrameSender,
    remote: Mutex<Remote>,
}

/// Bind the MCU-006 control port and start its keepalive.
pub fn spawn_mcu006_control_server(
    bind: Ipv4Addr,
    port: u16,
    link: Link,
    sender: FrameSender,
    stop: Option<Arc<AtomicBool>>,
) -> Result<JoinHandle<()>> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding mcu006 control TCP port {bind}:{port}"))?;
    log::info!("mcu006 control server listening on {bind}:{port} (RW)");
    let control = Arc::new(Control {
        link,
        sender,
        remote: Mutex::new(Remote::default()),
    });
    {
        let control = control.clone();
        let stop = stop.clone();
        thread::Builder::new()
            .name("mcu006-keepalive".into())
            .spawn(move || keepalive(&control, stop))
            .context("spawning the mcu006 keepalive")?;
    }
    Ok(thread::Builder::new()
        .name("mcu006-control-accept".into())
        .spawn(move || {
            accept_until("mcu006-control", listener, stop, |stream, peer| {
                log::info!("mcu006-control client connected: {peer}");
                let control = control.clone();
                thread::Builder::new()
                    .name("mcu006-control-client".into())
                    .spawn(move || run_client(stream, &control))
                    .ok();
            });
        })
        .expect("spawn mcu006-control accept"))
}

fn keepalive(control: &Control, stop: Option<Arc<AtomicBool>>) {
    while stop.as_ref().is_none_or(|s| !s.load(Ordering::Relaxed)) {
        thread::sleep(KEEPALIVE);
        control.send_state();
    }
}

impl Control {
    fn link(&self) -> LinkState {
        *self.link.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn remote(&self) -> std::sync::MutexGuard<'_, Remote> {
        self.remote.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Our address and the host's, or why reports cannot go out yet.
    fn route(&self) -> Result<(u8, u8), &'static str> {
        match self.link() {
            LinkState {
                addr: Some(addr),
                host: Some(host),
            } => Ok((addr, host)),
            LinkState { addr: None, .. } => Err("the remote has not claimed an address yet"),
            _ => Err("the remote is not paired with an MFD yet"),
        }
    }

    fn send(&self, frame: crate::engine::RawFrame) {
        if self.sender.send_frame(frame).is_err() {
            log::warn!("mcu006-control: the CAN writer is gone");
        }
    }

    /// The held keys, the button and keypad mode, as the 1 s keepalive
    /// sends them.
    fn send_state(&self) {
        let Ok((addr, host)) = self.route() else {
            return;
        };
        self.send_keys(addr, host);
        self.send_flags(addr, host);
    }

    fn send_keys(&self, addr: u8, host: u8) {
        let keys = self.remote().codes();
        self.send(keyboard_report(addr, host, &keys));
    }

    /// A pointer report without movement: the button and keypad mode.
    fn send_flags(&self, addr: u8, host: u8) {
        let (button, keypad) = self.remote().flags();
        self.send(pointer_report(addr, host, 0, 0, 0, button, keypad));
    }
}

/// What one client holds, released when it goes away.
#[derive(Default)]
struct Held {
    keys: Vec<u8>,
    button: bool,
}

fn run_client(stream: TcpStream, control: &Control) {
    let Ok(mut out) = stream.try_clone() else {
        return;
    };
    let mut held = Held::default();
    for line in BufReader::new(stream).lines() {
        let Ok(line) = line else { break };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let reply = match serde_json::from_str::<Value>(line) {
            Ok(cmd) => handle(&cmd, control, &mut held),
            Err(e) => Err(format!("not JSON: {e}")),
        };
        let mut reply = match reply {
            Ok(()) => json!({"ok": true}),
            Err(e) => json!({"ok": false, "error": e}),
        };
        add_state(&mut reply, control);
        if writeln!(out, "{reply}").is_err() {
            break;
        }
    }
    release(control, &mut held);
    log::info!("mcu006-control client disconnected");
}

fn add_state(reply: &mut Value, control: &Control) {
    let link = control.link();
    let r = control.remote();
    reply["addr"] = json!(link.addr);
    reply["host"] = json!(link.host);
    reply["keys"] = json!(r.codes());
    reply["button"] = json!(r.button > 0);
    reply["keypad"] = json!(r.keypad);
}

fn handle(cmd: &Value, control: &Control, held: &mut Held) -> Result<(), String> {
    if cmd.get("status").is_some() {
        return Ok(());
    }
    if cmd.get("release").is_some() {
        release(control, held);
        return Ok(());
    }
    let (addr, host) = control.route().map_err(str::to_string)?;
    if let Some(key) = cmd.get("key") {
        let code = key_code(key)?;
        let down = cmd.get("down").and_then(Value::as_bool).unwrap_or(true);
        set_key(&mut control.remote(), held, code, down)?;
        control.send_keys(addr, host);
    } else if let Some(key) = cmd.get("tap") {
        let code = key_code(key)?;
        set_key(&mut control.remote(), held, code, true)?;
        control.send_keys(addr, host);
        thread::sleep(TAP);
        set_key(&mut control.remote(), held, code, false)?;
        control.send_keys(addr, host);
    } else if let Some(p) = cmd.get("pointer") {
        let dx = movement(p.get("dx"), "dx")?;
        let dy = movement(p.get("dy"), "dy")?;
        let (button, keypad) = control.remote().flags();
        for (dx, dy) in steps(dx, dy) {
            control.send(pointer_report(addr, host, dx, dy, 0, button, keypad));
        }
    } else if let Some(n) = cmd.get("knob") {
        let n = movement(Some(n), "knob")?;
        let (button, keypad) = control.remote().flags();
        for (wheel, _) in steps(n, 0) {
            control.send(pointer_report(addr, host, 0, 0, wheel, button, keypad));
        }
    } else if let Some(down) = cmd.get("button") {
        let down = down.as_bool().ok_or("button takes true or false")?;
        set_button(control, held, down);
        control.send_flags(addr, host);
    } else if cmd.get("click").is_some() {
        set_button(control, held, true);
        control.send_flags(addr, host);
        thread::sleep(TAP);
        set_button(control, held, false);
        control.send_flags(addr, host);
    } else if let Some(on) = cmd.get("keypad") {
        control.remote().keypad = on.as_bool().ok_or("keypad takes true or false")?;
        control.send_flags(addr, host);
    } else {
        return Err(
            "unknown command; use key, tap, pointer, knob, button, click, keypad, \
                    release or status"
                .into(),
        );
    }
    Ok(())
}

/// A `pointer` axis or `knob` turn: a whole number within ±[`MAX_MOVE`],
/// 0 when absent.
fn movement(v: Option<&Value>, what: &str) -> Result<i64, String> {
    let Some(v) = v else { return Ok(0) };
    v.as_i64()
        .filter(|n| (-MAX_MOVE..=MAX_MOVE).contains(n))
        .ok_or_else(|| format!("{what} takes a whole number from -{MAX_MOVE} to {MAX_MOVE}"))
}

fn set_key(r: &mut Remote, held: &mut Held, code: u8, down: bool) -> Result<(), String> {
    if down {
        if !held.keys.contains(&code) {
            if let Some((_, holders)) = r.keys.iter_mut().find(|(k, _)| *k == code) {
                *holders += 1;
            } else if r.keys.len() >= MAX_KEYS {
                return Err(format!(
                    "the remote reports at most {MAX_KEYS} keys at once"
                ));
            } else {
                r.keys.push((code, 1));
            }
            held.keys.push(code);
        }
    } else if let Some(i) = held.keys.iter().position(|&k| k == code) {
        held.keys.remove(i);
        r.drop_key(code);
    }
    Ok(())
}

fn set_button(control: &Control, held: &mut Held, down: bool) {
    if held.button == down {
        return;
    }
    held.button = down;
    let mut r = control.remote();
    if down {
        r.button += 1;
    } else {
        r.button = r.button.saturating_sub(1);
    }
}

/// Release whatever this client holds, and tell the MFD.
fn release(control: &Control, held: &mut Held) {
    if held.keys.is_empty() && !held.button {
        return;
    }
    {
        let mut r = control.remote();
        for code in std::mem::take(&mut held.keys) {
            r.drop_key(code);
        }
    }
    set_button(control, held, false);
    if let Ok((addr, host)) = control.route() {
        control.send_keys(addr, host);
        control.send_flags(addr, host);
    }
}

/// Split a movement into reports of at most ±127 per axis.
fn steps(dx: i64, dy: i64) -> Vec<(i8, i8)> {
    let clamp = |v: i64| v.clamp(-127, 127);
    let (mut dx, mut dy) = (dx, dy);
    let mut out = Vec::new();
    while dx != 0 || dy != 0 {
        let (sx, sy) = (clamp(dx), clamp(dy));
        out.push((sx as i8, sy as i8));
        dx -= sx;
        dy -= sy;
    }
    out
}

/// A key as a HID keyboard usage ID: a number, or a name (`F5`, `Up`,
/// `a`, `7`, `Enter`, …). Names ignore case.
fn key_code(key: &Value) -> Result<u8, String> {
    if let Some(n) = key.as_u64() {
        return u8::try_from(n)
            .ok()
            .filter(|&c| c != 0xff)
            .ok_or_else(|| format!("{n} is not a HID usage ID"));
    }
    let name = key.as_str().ok_or("a key is a name or a number")?;
    hid_usage(name).ok_or_else(|| format!("unknown key '{name}'"))
}

fn hid_usage(name: &str) -> Option<u8> {
    let lower = name.to_ascii_lowercase();
    let mut chars = lower.chars();
    if let (Some(c), None) = (chars.next(), chars.next()) {
        return match c {
            'a'..='z' => Some(0x04 + (c as u8 - b'a')),
            '1'..='9' => Some(0x1e + (c as u8 - b'1')),
            '0' => Some(0x27),
            _ => None,
        };
    }
    if let Some(n) = lower.strip_prefix('f').and_then(|n| n.parse::<u8>().ok())
        && (1..=12).contains(&n)
    {
        return Some(0x3a + n - 1);
    }
    Some(match lower.as_str() {
        "enter" | "return" => 0x28,
        "escape" | "esc" => 0x29,
        "backspace" => 0x2a,
        "tab" => 0x2b,
        "space" => 0x2c,
        "insert" => 0x49,
        "home" => 0x4a,
        "pageup" => 0x4b,
        "delete" => 0x4c,
        "end" => 0x4d,
        "pagedown" => 0x4e,
        "right" => 0x4f,
        "left" => 0x50,
        "down" => 0x51,
        "up" => 0x52,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_the_keys_the_remote_sends() {
        // Every code in the MCU-006 key table, by name.
        let table = [
            ("1", 0x1e),
            ("2", 0x1f),
            ("3", 0x20),
            ("4", 0x21),
            ("F4", 0x3d),
            ("F5", 0x3e),
            ("F8", 0x41),
            ("F12", 0x45),
            ("y", 0x1c),
            ("Up", 0x52),
            ("right", 0x4f),
            ("DOWN", 0x51),
            ("Left", 0x50),
        ];
        for (name, code) in table {
            assert_eq!(hid_usage(name), Some(code), "{name}");
        }
        assert_eq!(hid_usage("F13"), None);
        assert_eq!(hid_usage("nope"), None);
        assert_eq!(key_code(&json!(62)), Ok(0x3e));
        assert!(key_code(&json!(255)).is_err());
        assert!(key_code(&json!(300)).is_err());
    }

    #[test]
    fn splits_large_movements() {
        assert_eq!(steps(0, 0), vec![]);
        assert_eq!(steps(16, -5), vec![(16, -5)]);
        assert_eq!(steps(300, -10), vec![(127, -10), (127, 0), (46, 0)]);
        assert_eq!(steps(-128, 0), vec![(-127, 0), (-1, 0)]);
    }

    #[test]
    fn bounds_a_movement() {
        assert_eq!(movement(None, "dx"), Ok(0));
        assert_eq!(movement(Some(&json!(-40)), "dx"), Ok(-40));
        assert_eq!(movement(Some(&json!(MAX_MOVE)), "knob"), Ok(MAX_MOVE));
        assert!(movement(Some(&json!(MAX_MOVE + 1)), "dx").is_err());
        assert!(movement(Some(&json!(i64::MIN)), "dx").is_err());
        assert!(movement(Some(&json!(u64::MAX)), "dx").is_err());
        assert!(movement(Some(&json!("40")), "dx").is_err());
        assert!(movement(Some(&json!(1.5)), "knob").is_err());
    }

    #[test]
    fn a_key_two_clients_hold_stays_down_until_both_let_go() {
        let mut r = Remote::default();
        let (mut a, mut b) = (Held::default(), Held::default());
        set_key(&mut r, &mut a, 0x52, true).unwrap();
        set_key(&mut r, &mut b, 0x52, true).unwrap();
        set_key(&mut r, &mut a, 0x52, true).unwrap(); // a repeat is not a second hold
        assert_eq!(r.codes(), [0x52]);
        set_key(&mut r, &mut a, 0x52, false).unwrap();
        assert_eq!(r.codes(), [0x52]);
        set_key(&mut r, &mut b, 0x52, false).unwrap();
        assert!(r.codes().is_empty());
    }

    #[test]
    fn holds_at_most_five_keys() {
        let mut r = Remote::default();
        let mut a = Held::default();
        for code in 0x1e..0x23 {
            set_key(&mut r, &mut a, code, true).unwrap();
        }
        assert!(set_key(&mut r, &mut a, 0x23, true).is_err());
        // Another client may still hold one of the five.
        set_key(&mut r, &mut Held::default(), 0x1e, true).unwrap();
        assert_eq!(r.codes(), [0x1e, 0x1f, 0x20, 0x21, 0x22]);
    }
}
