// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! End-to-end exercise of the public `read` API: open a capture file and
//! decode it through the facade, the way an external consumer would.

#[test]
fn open_capture_decodes_a_plain_file() {
    // A single-frame PGN 127251 (Rate of Turn) PLAIN line — same fixture the
    // convert tests use.
    let path = std::env::temp_dir().join(format!("canboat_read_api_{}.plain", std::process::id()));
    std::fs::write(
        &path,
        b"2026-01-01T00:00:00.000Z,3,127251,27,255,8,00,ca,8f,f3,ff,25,02,ff\n",
    )
    .expect("write fixture");

    let db = canboat::Database::embedded(canboat::Units::Metric);
    let reader = canboat::read::open_capture(&path).expect("open_capture");
    let decoded: Vec<_> = canboat::read::Decoder::new(reader, db)
        .map(|r| r.expect("source I/O"))
        .collect();

    std::fs::remove_file(&path).ok();

    assert_eq!(decoded.len(), 1, "one frame in, one record out");
    assert_eq!(decoded[0].pgn, 127251);
    assert_eq!(decoded[0].src, 27);
}
