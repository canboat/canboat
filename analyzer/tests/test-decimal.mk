# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# DECIMAL (#962): a PGN 129808 DSC Message Address that is valid, one with a
# byte that is not a digit pair (0xab), and one the packet ends inside. Only
# the first has a value; the others are not available, not a shorter number.
#
.PHONY: test-decimal
tests: test-decimal
test-decimal:
	$(ANALYZER) -json < pgn-decimal.in > $(TEMPDIR)/pgn-decimal.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-decimal.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-decimal.out
	diff $(TEMPDIR)/pgn-decimal.out pgn-decimal.out
	diff $(TEMPDIR)/pgn-decimal.err pgn-decimal.err
