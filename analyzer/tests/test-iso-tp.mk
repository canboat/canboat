# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests ISO 11783-3 Transport Protocol reassembly: a real 32-packet BAM
# sequence (PGN 60416 TP.CM + PGN 60160 TP.DT) captured off a Navico NAC-3
# broadcasting PGN 129540 (GNSS Sats in View) with a satellite list too large
# even for fast-packet's 223-byte ceiling. Also covers a second session that
# is aborted mid-transfer (no output expected) and a stray TP.DT frame with
# no open session (silently ignored).
#
.PHONY: test-iso-tp
tests: test-iso-tp
test-iso-tp:
	$(ANALYZER) -json -nv < iso-tp-test.in > $(TEMPDIR)/iso-tp-test.out -fixtime iso-tp-test 2> $(TEMPDIR)/iso-tp-test.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/iso-tp-test.out
	diff $(TEMPDIR)/iso-tp-test.out iso-tp-test.out
	diff $(TEMPDIR)/iso-tp-test.err iso-tp-test.err
