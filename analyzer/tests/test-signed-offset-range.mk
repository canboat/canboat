# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

.PHONY: test-signed-offset-range
tests: test-signed-offset-range
test-signed-offset-range:
	$(ANALYZER) -json -si -q -nv -fixtime pgn-test < pgn-signed-offset-range.in > $(TEMPDIR)/pgn-signed-offset-range.out 2> $(TEMPDIR)/pgn-signed-offset-range.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-signed-offset-range.out
	diff $(TEMPDIR)/pgn-signed-offset-range.out pgn-signed-offset-range.out
	diff $(TEMPDIR)/pgn-signed-offset-range.err pgn-signed-offset-range.err
