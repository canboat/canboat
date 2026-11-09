# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A 126208 Command for a proprietary PGN that gives the Industry Code inside
# parameter 1 (its top bits) instead of as parameter 3 still resolves the
# manufacturer's variant, as with both parameters.
#

.PHONY: test-pgn-126208-command-65287
tests: test-pgn-126208-command-65287
test-pgn-126208-command-65287:
	$(ANALYZER) -json -nv < pgn-126208-command-65287.in > $(TEMPDIR)/pgn-126208-command-65287.out -fixtime pgn-126208-command-65287 2> $(TEMPDIR)/pgn-126208-command-65287.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-126208-command-65287.out
	diff $(TEMPDIR)/pgn-126208-command-65287.out pgn-126208-command-65287.out
	diff $(TEMPDIR)/pgn-126208-command-65287.err pgn-126208-command-65287.err
