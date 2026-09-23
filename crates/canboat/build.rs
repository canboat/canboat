// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Capture the git short SHA (and a dirty flag) at build time so the
//! `analyzer` version banner can report the exact commit the binary was
//! built from. Non-fatal: outside a git worktree (a crates.io or source
//! tarball, `cargo package`'s verification build) vergen emits nothing
//! and `build_info.rs` reads the variables with `option_env!`, so the
//! banner reports `unknown`. Note that vergen does not return an error
//! in that case — it only warns — which is why the fallback lives on
//! the reading side rather than here.

use vergen_gitcl::{Emitter, Gitcl};

fn main() {
    let gitcl = Gitcl::builder()
        .sha(true) // short SHA → VERGEN_GIT_SHA
        .dirty(false) // tracked-file changes only
        .build();
    if Emitter::default()
        .add_instructions(&gitcl)
        .and_then(|e| e.emit())
        .is_err()
    {
        println!("cargo:warning=git commit info unavailable; version banner will report 'unknown'");
    }
}
