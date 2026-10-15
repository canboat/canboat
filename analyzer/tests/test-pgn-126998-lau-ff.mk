# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that a STRING_LAU field whose encoding byte is 0xff (an unset field
# padded with 0xff, e.g. the H5000 pilot in PGN 126998) decodes as an empty field
# instead of raising "Unhandled string type 255" and aborting the whole PGN.
#
.PHONY: test-pgn-126998-lau-ff
tests: test-pgn-126998-lau-ff
test-pgn-126998-lau-ff:
	$(ANALYZER) -json -nv < pgn-126998-lau-ff.in > $(TEMPDIR)/pgn-126998-lau-ff.out -fixtime pgn-126998-lau-ff 2> $(TEMPDIR)/pgn-126998-lau-ff.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-126998-lau-ff.out
	diff $(TEMPDIR)/pgn-126998-lau-ff.out pgn-126998-lau-ff.out
	diff $(TEMPDIR)/pgn-126998-lau-ff.err pgn-126998-lau-ff.err
