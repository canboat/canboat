# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# Above RangeMax (#1167): latitude 111 deg, with longitude at exactly 180; both
# at their edge, 90 and 180; both one step above it; heading at its edge (2 pi),
# one step above it, at the top of its own bits below the sentinels, and all
# ones, not available. In Metric and SI.
#
.PHONY: test-above-range-max
tests: test-above-range-max
test-above-range-max:
	$(ANALYZER) -json < pgn-above-range-max.in > $(TEMPDIR)/pgn-above-range-max.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-above-range-max.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-above-range-max.out
	diff $(TEMPDIR)/pgn-above-range-max.out pgn-above-range-max.out
	diff $(TEMPDIR)/pgn-above-range-max.err pgn-above-range-max.err
	$(ANALYZER) -json -si < pgn-above-range-max.in > $(TEMPDIR)/pgn-above-range-max-si.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-above-range-max-si.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-above-range-max-si.out
	diff $(TEMPDIR)/pgn-above-range-max-si.out pgn-above-range-max-si.out
	diff $(TEMPDIR)/pgn-above-range-max-si.err pgn-above-range-max.err
