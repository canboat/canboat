#
# Makefile for all UNIX style platforms including Cygwin
#
# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands
#
# $Id:$
#

# s. https://www.gnu.org/prep/standards/html_node/Directory-Variables.html#Directory-Variables
DESTDIR ?= ""
PREFIX ?= /usr/local
EXEC_PREFIX ?= $(PREFIX)
BINDIR=$(EXEC_PREFIX)/bin
DATAROOTDIR ?= $(PREFIX)/share
MANDIR= $(DATAROOTDIR)/man

PLATFORM ?= $(shell uname | tr '[A-Z]' '[a-z]')-$(shell uname -m)
SUBDIRS= actisense-serial analyzer nmea0183 ip group-function candump2analyzer socketcan-writer socketcan-serial ikonvert-serial maretron-ipg replay

BUILDDIR ?= ./rel/$(PLATFORM)

MKDIR = mkdir -p
export HELP2MAN=$(shell command -v help2man 2> /dev/null)


ROOT_UID=0
ROOT_GID=0
ROOT_MOD=0644
EXEC_MOD=0755

all:	bin compile
	@echo "The binaries are now built and are in $(BUILDDIR)"
	@echo "Use 'make generated' to recreate generated XML, HTML, JSON and DBC files."

compile: bin
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir || exit 1; done

tests:  compile
	$(MAKE) -C analyzer tests
	$(MAKE) -C actisense-serial/tests tests
	$(MAKE) -C candump2analyzer/tests tests

# The Cargo workspace at the repo root (crates/*, keel/) is deliberately
# OPT-IN and is NOT a dependency of all/compile/tests: a plain `make` of the C
# tools must never invoke cargo, so C-only contributors and packagers need no
# Rust toolchain (MERGE-CANBOAT-RS.md goal #1). Rust contributors just use
# cargo directly; these targets exist so `make` users have the same shortcuts.
CARGO ?= cargo

# The Rust schema tables are generated from database/ by keel and committed
# (MERGE-CANBOAT-RS.md §5), so a `cargo build` alone will happily compile
# against a stale table after a database edit. Regenerate first.
#
# Deliberately NOT a dependency on `generated`: that additionally produces
# canboat.html/json via xsltproc, validates with xmllint, runs the C golden
# tests and builds the DBC exporter, so it needs xsltproc, libxml2-utils,
# python3 + venv and a C compiler. None of that is required to build the
# Rust side, and demanding it would make `make rust` unusable on a machine
# that only has a Rust toolchain.
#
# keel/keel is self-contained: it builds keel with cargo, or downloads a
# prebuilt binary when there is no toolchain. It writes only the artifacts
# whose content actually changed, so this does not disturb the C build.
.PHONY: keel-generate
keel-generate:
	@keel/keel generate

rust: keel-generate
	$(CARGO) build --release --workspace

rust-debug: keel-generate
	$(CARGO) build --workspace

rust-tests: keel-generate
	$(CARGO) test --workspace

rust-clippy:
	$(CARGO) clippy --workspace --all-targets -- -D warnings

rust-fmt:
	$(CARGO) fmt --all

# Everything worth having green before opening a PR that touches Rust.
rust-precommit: rust-fmt rust-clippy rust-tests

rust-clean:
	$(CARGO) clean

# Regenerate FIRST, then test against the fresh output. keel (run inside
# `analyzer generated`) rewrites the C data tables and canboat.xml from the
# database; only afterwards do we rebuild the analyzer from those fresh
# headers and run the golden-file suite. Running tests before regeneration
# would validate the stale committed tables and never exercise the new output.
generated: research-docs
	$(MAKE) -C analyzer generated
	$(MAKE) tests
	$(MAKE) -C dbc-exporter

# Run before opening a PR: regenerates the database, then reports how this
# change affects the public contract (breaking / minor / additive) and what
# conventional-commit level it needs. Advisory only; CI enforces the same
# check via tools/contract-pr.sh --gate.
pr: generated
	@tools/contract-pr.sh

# Explanation documents: research/*.md -> docs/*.html via a portable,
# dependency-free converter (Python 3 standard library only).
RESEARCH_MD := $(wildcard research/*.md)
RESEARCH_HTML := $(patsubst research/%.md,docs/%.html,$(RESEARCH_MD))

docs/%.html: research/%.md tools/md2html.py
	python3 tools/md2html.py $< -o $@

research-docs: $(RESEARCH_HTML)
	@echo "Research docs generated: $(RESEARCH_HTML)"

# Builder image can be removed with `docker image rm canboat-builder`
docker-build: ## runs `make clean generated` in `ubuntu:22.04` Docker image
	@docker build -t canboat-builder .
	@docker run -it --rm -v $(shell pwd):/project canboat-builder clean generated

bin:	$(BUILDDIR)

CYGWIN_DLL=$(BUILDDIR)/cygwin1.dll

$(CYGWIN_DLL): $(BUILDDIR)
	cp /usr/bin/cygwin1.dll $(CYGWIN_DLL)

CYGWIN=$(findstring cygwin,$(PLATFORM))

ifneq (,$(CYGWIN))
bin:	$(CYGWIN_DLL)
	@echo "Building in $(BUILDDIR) for '$(CYGWIN)' with $(CYGWIN_DLL)"
else
bin:
	@echo "Building in $(BUILDDIR)"
endif

$(BUILDDIR): 
	$(MKDIR) $(BUILDDIR)

man1: man/man1

man/man1:
	$(MKDIR) man/man1

clean:
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir clean; done
	$(MAKE) -C dbc-exporter clean
	-rm -R -f man $(BUILDDIR)

install: $(BUILDDIR)/analyzer $(DESTDIR)$(BINDIR) $(DESTDIR)$(MANDIR)/man1
	for i in $(BUILDDIR)/* util/*; do install -m $(EXEC_MOD) -b $$i $(DESTDIR)$(BINDIR); done
ifeq ($(notdir $(HELP2MAN)),help2man)
	for i in man/man1/*; do echo $$i; install -m $(ROOT_MOD) $$i $(DESTDIR)$(MANDIR)/man1; done
endif

format:
	for file in */*.c */*.h; do clang-format -i $$file; done

release:
	$(MAKE) clean generated
	git diff --exit-code
	git tag v`sed -En 's/.*\ VERSION\ \"([0-9]+\.)([0-9]+\.)?([0-9]+)\"/\1\2\3/p' common/version.h`
	git push --tags

copyright:
	$(MAKE) clean
	rm -rf rel/
	./util/update-copyright.sh

aarch64-linux-musl:
	./cross-compile.sh aarch64-linux-musl


.PHONY : $(SUBDIRS) clean install zip bin format man1 tests generated research-docs compile copyright aarch64-linux-musl openwrt pr rust rust-debug rust-tests rust-clippy rust-fmt rust-precommit rust-clean keel-generate

$(DESTDIR)$(BINDIR):
	$(MKDIR) $(DESTDIR)$(BINDIR)


$(DESTDIR)$(MANDIR)/man1:
	$(MKDIR) $(DESTDIR)$(MANDIR)/man1
