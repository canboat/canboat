# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

.PHONY: test-float-units
tests: test-float-units
test-float-units:
	$(ANALYZER) -json < pgn-float-units.in > $(TEMPDIR)/pgn-float-units.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-float-units.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-float-units.out
	diff $(TEMPDIR)/pgn-float-units.out pgn-float-units.out
	diff $(TEMPDIR)/pgn-float-units.err pgn-float-units.err
	$(ANALYZER) -json -si < pgn-float-units.in > $(TEMPDIR)/pgn-float-units-si.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-float-units-si.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-float-units-si.out
	diff $(TEMPDIR)/pgn-float-units-si.out pgn-float-units-si.out
	diff $(TEMPDIR)/pgn-float-units-si.err pgn-float-units.err
