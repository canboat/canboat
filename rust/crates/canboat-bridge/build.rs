// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Capture the git short SHA (and a dirty flag) at build time so the
//! `analyzer` version banner can report the exact commit the binary was
//! built from. Non-fatal: a release build from a source tarball has no
//! `.git`, so we fall back to a placeholder and the `env!` still resolves.

use vergen_gitcl::{Emitter, GitclBuilder};

fn main() {
    let emitted = (|| -> Option<()> {
        let gitcl = GitclBuilder::default()
            .sha(true) // short SHA → VERGEN_GIT_SHA
            .dirty(false) // tracked-file changes only
            .build()
            .ok()?;
        Emitter::default()
            .add_instructions(&gitcl)
            .ok()?
            .emit()
            .ok()?;
        Some(())
    })()
    .is_some();

    // Guarantee the env vars exist even when vergen couldn't resolve them
    // (no git, shallow clone, tarball build).
    if !emitted {
        println!("cargo:warning=git commit info unavailable; version banner will report 'unknown'");
        println!("cargo:rustc-env=VERGEN_GIT_SHA=unknown");
        println!("cargo:rustc-env=VERGEN_GIT_DIRTY=false");
    }
}
