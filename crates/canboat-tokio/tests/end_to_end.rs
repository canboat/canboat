// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Integration tests that prove the same sans-I/O core drives cleanly
//! through tokio. Each test fabricates a byte stream in memory, hands
//! it to a `Stream`, and verifies the emitted `DecodedPgn` events.

use canboat_core::PgnDatabase;
use canboat_tokio::{IkonvertStream, Ngt1Stream};
use futures::StreamExt;

fn db() -> &'static PgnDatabase {
    PgnDatabase::embedded(canboat_core::Units::Metric)
}

/// Build the same NGT-1 wire bytes the actisense-serial replay test
/// builds — one PGN 60928 frame inside DLE/STX/ETX framing.
fn build_ngt1_pgn60928_bytes() -> Vec<u8> {
    const DLE: u8 = 0x10;
    const STX: u8 = 0x02;
    const ETX: u8 = 0x03;
    const N2K_MSG_RECEIVED: u8 = 0x93;

    let pgn = 60928u32;
    let payload_bytes: &[u8] = &[0xfb, 0x9b, 0x70, 0x22, 0x00, 0x9b, 0x50, 0xc0];
    let mut payload = vec![
        6u8, // prio
        (pgn & 0xff) as u8,
        ((pgn >> 8) & 0xff) as u8,
        ((pgn >> 16) & 0xff) as u8,
        255, // dst
        5,   // src
    ];
    payload.extend_from_slice(&0u32.to_le_bytes()); // ts
    payload.push(payload_bytes.len() as u8);
    payload.extend_from_slice(payload_bytes);

    let mut inner = vec![N2K_MSG_RECEIVED, payload.len() as u8];
    inner.extend_from_slice(&payload);
    let cksum = inner.iter().copied().fold(0u8, u8::wrapping_add);
    inner.push(0u8.wrapping_sub(cksum));

    let mut wire = vec![DLE, STX];
    for b in &inner {
        wire.push(*b);
        if *b == DLE {
            wire.push(DLE);
        }
    }
    wire.extend_from_slice(&[DLE, ETX]);
    wire
}

#[tokio::test(flavor = "current_thread")]
async fn ngt1_stream_decodes_pgn60928_from_memory() {
    let bytes = build_ngt1_pgn60928_bytes();
    let reader = std::io::Cursor::new(bytes);
    let mut stream = Ngt1Stream::new(reader, db());
    let decoded = stream.next().await.expect("one DecodedPgn");
    assert_eq!(decoded.pgn, 60928);
    assert_eq!(decoded.id, "isoAddressClaim");
    // Manufacturer Code (field 2) resolves to Navico.
    let mfr = decoded
        .fields
        .iter()
        .find(|f| f.id() == "manufacturerCode")
        .expect("manufacturerCode field");
    match &mfr.value {
        canboat_core::FieldValue::Lookup { name, .. } => {
            assert_eq!(name.as_deref(), Some("Navico"));
        }
        other => panic!("expected lookup, got {other:?}"),
    }
    // EOF cleans up.
    assert!(stream.next().await.is_none());
}

#[tokio::test(flavor = "current_thread")]
async fn ikonvert_stream_decodes_one_pgn_from_memory() {
    // !PDGY line carrying PGN 60928 with the same 8-byte payload as
    // above. Base64 encode the payload bytes.
    let payload: &[u8] = &[0xfb, 0x9b, 0x70, 0x22, 0x00, 0x9b, 0x50, 0xc0];
    let b64 = base64_encode(payload);
    let line = format!("!PDGY,60928,6,5,255,12.345,{}\n", b64);
    let reader = std::io::Cursor::new(line.into_bytes());
    let mut stream = IkonvertStream::new(reader, db());
    let decoded = stream.next().await.expect("one DecodedPgn");
    assert_eq!(decoded.pgn, 60928);
    assert!(stream.next().await.is_none());
}

/// Encode `bytes` as RFC 4648 Base64 with `=` padding.
fn base64_encode(bytes: &[u8]) -> String {
    const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    let mut i = 0;
    while i + 3 <= bytes.len() {
        let n =
            (u32::from(bytes[i]) << 16) | (u32::from(bytes[i + 1]) << 8) | u32::from(bytes[i + 2]);
        out.push(TABLE[((n >> 18) & 63) as usize] as char);
        out.push(TABLE[((n >> 12) & 63) as usize] as char);
        out.push(TABLE[((n >> 6) & 63) as usize] as char);
        out.push(TABLE[(n & 63) as usize] as char);
        i += 3;
    }
    match bytes.len() - i {
        0 => {}
        1 => {
            let n = u32::from(bytes[i]) << 16;
            out.push(TABLE[((n >> 18) & 63) as usize] as char);
            out.push(TABLE[((n >> 12) & 63) as usize] as char);
            out.push('=');
            out.push('=');
        }
        2 => {
            let n = (u32::from(bytes[i]) << 16) | (u32::from(bytes[i + 1]) << 8);
            out.push(TABLE[((n >> 18) & 63) as usize] as char);
            out.push(TABLE[((n >> 12) & 63) as usize] as char);
            out.push(TABLE[((n >> 6) & 63) as usize] as char);
            out.push('=');
        }
        _ => unreachable!(),
    }
    out
}
