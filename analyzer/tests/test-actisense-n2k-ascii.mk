# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests capability to read Actisense N2K Ascii format
#
.PHONY: test-actisense-n2k-ascii
tests: test-actisense-n2k-ascii
test-actisense-n2k-ascii:	
	TZ=UTC-1 $(ANALYZER) < pgn-test-actisense.in > $(TEMPDIR)/pgn-test-actisense.out -nv -debug -q -json -fixtime pgn-test 2> $(TEMPDIR)/pgn-test-actisense.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-test-actisense.out
	diff $(TEMPDIR)/pgn-test-actisense.out pgn-test-actisense.out
	diff $(TEMPDIR)/pgn-test-actisense.err pgn-test-actisense.err
