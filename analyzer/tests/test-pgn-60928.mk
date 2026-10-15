# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that PGN 60928 ISO Address Claim preserves full-range device instance values
#
.PHONY: test-pgn-60928
tests: test-pgn-60928
test-pgn-60928:
	$(ANALYZER) -json -nv < pgn-60928.in > $(TEMPDIR)/pgn-60928-nv.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-60928-nv.err
	diff $(TEMPDIR)/pgn-60928-nv.out pgn-60928-nv.out
	diff $(TEMPDIR)/pgn-60928-nv.err pgn-60928-nv.err
