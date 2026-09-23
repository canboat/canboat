// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The `canboat` binary. Everything lives in the library's private `cli`
//! module so the subcommands can reach the `engine` / `io` / `server`
//! internals that the public facade deliberately hides; this file only
//! forwards.

fn main() -> std::process::ExitCode {
    canboat::cli_main()
}
