// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Output from a live input comes out as it is made, as from the C
//! programs, which flush every message: `actisense-serial | analyzer
//! -json | n2kd` must not wait for a buffer to fill on a quiet bus.

use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};
use std::sync::mpsc;
use std::time::Duration;

const LINE: &[u8] = b"2026-10-19T13:00:00.000Z,6,127508,0,255,8,c8,f9,04,ff,7f,ff,ff,00\n";

/// Run `program args`, write one frame to its stdin and keep stdin open:
/// the first `n` lines of its output must come while it still waits for
/// more.
fn lines_while_stdin_open(program: &std::path::Path, args: &[&str], n: usize) -> String {
    let mut child = Command::new(program)
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn canboat");
    let mut stdin = child.stdin.take().unwrap();
    stdin.write_all(LINE).expect("write stdin");
    stdin.flush().unwrap();
    let stdout = child.stdout.take().unwrap();
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let mut reader = BufReader::new(stdout);
        let mut lines = String::new();
        for _ in 0..n {
            if reader.read_line(&mut lines).unwrap_or(0) == 0 {
                break;
            }
        }
        let _ = tx.send(lines);
    });
    let line = rx.recv_timeout(Duration::from_secs(10));
    let _ = child.kill();
    let _ = child.wait();
    drop(stdin);
    line.expect("no output while stdin was still open")
}

#[test]
fn convert_from_a_pipe_writes_each_message_as_it_comes() {
    let line = lines_while_stdin_open(
        env!("CARGO_BIN_EXE_canboat").as_ref(),
        &["convert", "--to", "json", "--no-banner"],
        1,
    );
    assert!(line.contains("\"pgn\":127508"), "{line}");
}

#[test]
fn convert_from_dash_writes_each_message_as_it_comes() {
    let line = lines_while_stdin_open(
        env!("CARGO_BIN_EXE_canboat").as_ref(),
        &["convert", "--to", "json", "--no-banner", "-"],
        1,
    );
    assert!(line.contains("\"pgn\":127508"), "{line}");
}

/// A binary format has no line end: each record is flushed as it is made.
#[test]
fn convert_to_a_binary_format_writes_each_record_as_it_comes() {
    let mut child = Command::new(env!("CARGO_BIN_EXE_canboat"))
        .args(["convert", "--to", "actisense-ebl"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn canboat");
    let mut stdin = child.stdin.take().unwrap();
    stdin.write_all(LINE).expect("write stdin");
    stdin.flush().unwrap();
    let mut stdout = child.stdout.take().unwrap();
    let (tx, rx) = mpsc::channel();
    // Until the message's own record: the EBL timestamp records before it
    // end in 0x0A, which a line-buffered stdout flushes at anyway.
    std::thread::spawn(move || {
        let mut got = Vec::new();
        let mut buf = [0u8; 64];
        let found = loop {
            if got.windows(3).any(|w| w == [0xc8, 0xf9, 0x04]) {
                break true;
            }
            match std::io::Read::read(&mut stdout, &mut buf) {
                Ok(0) | Err(_) => break false,
                Ok(n) => got.extend_from_slice(&buf[..n]),
            }
        };
        let _ = tx.send(found);
    });
    let got = rx.recv_timeout(Duration::from_secs(10));
    let _ = child.kill();
    let _ = child.wait();
    drop(stdin);
    assert!(
        got.expect("the message's record did not come while stdin was still open"),
        "the output ended without the message's record"
    );
}

#[cfg(unix)]
#[test]
fn analyzer_from_a_pipe_writes_each_message_as_it_comes() {
    let dir = std::env::temp_dir().join(format!("canboat-live-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let analyzer = dir.join("analyzer");
    let _ = std::fs::remove_file(&analyzer);
    std::os::unix::fs::symlink(env!("CARGO_BIN_EXE_canboat"), &analyzer).unwrap();
    // The banner, then the message.
    let lines = lines_while_stdin_open(&analyzer, &["-json"], 2);
    let _ = std::fs::remove_dir_all(&dir);
    assert!(lines.contains("\"pgn\":127508"), "{lines}");
}
