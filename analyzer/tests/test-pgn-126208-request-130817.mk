# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

.PHONY: test-pgn-126208-request-130817
tests: test-pgn-126208-request-130817
test-pgn-126208-request-130817:
	$(ANALYZER) -json -nv < pgn-126208-request-130817.in > $(TEMPDIR)/pgn-126208-request-130817.out -fixtime pgn-126208-request-130817 2> $(TEMPDIR)/pgn-126208-request-130817.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-126208-request-130817.out
	diff $(TEMPDIR)/pgn-126208-request-130817.out pgn-126208-request-130817.out
	diff $(TEMPDIR)/pgn-126208-request-130817.err pgn-126208-request-130817.err
