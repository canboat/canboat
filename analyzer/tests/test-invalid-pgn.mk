# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that out-of-range PGNs don't read past the end of pgnList[] in the
# searchForPgn() binary search (issue #644). The first PGN (393216) is the
# FuzzingLabs PoC value; valid PGN 126992 at the end is a decoding control.
#
.PHONY: test-invalid-pgn
tests: test-invalid-pgn
test-invalid-pgn:
	$(ANALYZER) < invalid-pgn-test.in > $(TEMPDIR)/invalid-pgn-test.out -debug -q -fixtime pgn-test 2> $(TEMPDIR)/invalid-pgn-test.err
	diff $(TEMPDIR)/invalid-pgn-test.out invalid-pgn-test.out
	diff $(TEMPDIR)/invalid-pgn-test.err invalid-pgn-test.err
