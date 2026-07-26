# canboat-rs — top-level Makefile
#
# Convenience wrappers over `cargo` for the most common workflows. Nothing
# here is required: `cargo build --release`, `cargo test --workspace`, etc.
# do the same job directly. The value is remembering the right invocation
# in one place, and having a `precommit` target that mirrors what CI checks.
#
# Common targets (see `make help` for the full list):
#   make               - Release build of every workspace member
#   make debug         - Debug build of every workspace member
#   make test          - Run the workspace test suite
#   make fmt           - `cargo fmt --all`
#   make clippy        - Workspace clippy at `-D warnings` (what CI enforces)
#   make precommit     - fmt + clippy + test — run this before pushing
#   make canboat       - Release build of just the `canboat` binary
#   make clean         - `cargo clean`
#
# Per-developer targets (SSH deploys to your own boxes, scratch experiments,
# hardware-in-the-loop scripts) belong in `Makefile.local`, which is
# gitignored and `-include`d at the bottom of this file.

CARGO ?= cargo

.PHONY: all build debug check test fmt fmt-check clippy precommit \
        canboat \
        clean help

all: build

# Full-workspace release build. Everything published lands here.
build:
	$(CARGO) build --release --workspace

# Full-workspace debug build. Faster to compile, slower to run — useful
# for iterating on tests that link into a downstream binary.
debug:
	$(CARGO) build --workspace

# Quick type-check without producing binaries.
check:
	$(CARGO) check --workspace --all-targets

# Workspace test suite. `--workspace` covers every crate; the analyzer
# golden tests read fixtures from a sibling `../canboat*` checkout — the
# tests skip gracefully if that isn't present.
test:
	$(CARGO) test --workspace

fmt:
	$(CARGO) fmt --all

# fmt but read-only (fails when files aren't formatted). What CI runs.
fmt-check:
	$(CARGO) fmt --all --check

# Workspace-wide clippy at the same strictness CI uses. If this passes
# locally your PR won't get bounced on lints.
clippy:
	$(CARGO) clippy --workspace --all-targets -- -D warnings

# Everything you'd want green before opening a PR.
precommit: fmt clippy test

# Release shortcut for just the front-end binary. `cargo build --release
# -p canboat` under the hood — handy when you don't need the whole
# workspace (every tool is now a `canboat` subcommand).
canboat:
	$(CARGO) build --release -p canboat

clean:
	$(CARGO) clean

# List every target with a leading `##` comment above it. Nothing fancy
# — just grep this file.
help:
	@echo "canboat-rs Makefile targets:"
	@echo ""
	@echo "  make                Release build of every workspace member"
	@echo "  make debug          Debug build of every workspace member"
	@echo "  make check          Type-check without producing binaries"
	@echo "  make test           Run the workspace test suite"
	@echo "  make fmt            cargo fmt --all"
	@echo "  make fmt-check      cargo fmt --all --check (CI shape)"
	@echo "  make clippy         Workspace clippy at -D warnings (CI shape)"
	@echo "  make precommit      fmt + clippy + test"
	@echo ""
	@echo "  make canboat        Release build of just the canboat binary"
	@echo ""
	@echo "  make clean          cargo clean"
	@echo ""
	@echo "Per-developer targets live in Makefile.local (gitignored)."

# Optional per-developer extensions — cross-compile deploys, hardware
# rigs, scratch experiments. The leading dash makes the include silent
# when the file is absent, so a fresh clone behaves identically to
# having no file. See the top-level comment for the split's rationale.
-include Makefile.local
