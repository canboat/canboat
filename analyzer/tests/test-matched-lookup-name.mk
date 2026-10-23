# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A matched LOOKUP field is named by its lookup, not by the field's
# description (#1083), as canboat convert names it: Suzuki's 130830 is
# "Suzuki Motor" (the description says "Suzuki Motor Corporation"),
# Xantrex's 130900 "Xantrex Technology", Fusion's 130820 "SiriusXM Signal".
#
.PHONY: test-matched-lookup-name
tests: test-matched-lookup-name
test-matched-lookup-name:
	$(ANALYZER) -json < pgn-matched-lookup-name.in > $(TEMPDIR)/pgn-matched-lookup-name.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-matched-lookup-name.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-matched-lookup-name.out
	diff $(TEMPDIR)/pgn-matched-lookup-name.out pgn-matched-lookup-name.out
	diff $(TEMPDIR)/pgn-matched-lookup-name.err pgn-matched-lookup-name.err
