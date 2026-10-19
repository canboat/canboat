# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

.PHONY: test-pgn-126208-request-126720
tests: test-pgn-126208-request-126720
test-pgn-126208-request-126720:
	$(ANALYZER) -json -nv < pgn-126208-request-126720.in > $(TEMPDIR)/pgn-126208-request-126720.out -fixtime pgn-126208-request-126720 2> $(TEMPDIR)/pgn-126208-request-126720.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-126208-request-126720.out
	diff $(TEMPDIR)/pgn-126208-request-126720.out pgn-126208-request-126720.out
	diff $(TEMPDIR)/pgn-126208-request-126720.err pgn-126208-request-126720.err
