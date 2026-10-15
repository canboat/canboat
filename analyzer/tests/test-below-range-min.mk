# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# Below RangeMin (#983): an AtoN at raw 0xbd555556 has latitude -111.8 deg,
# below -90, so no latitude (its longitude, -111.8, is valid); the same AtoN
# at a real position; Attitude with yaw raw 0x8000 and pitch one step below
# -pi, both out of range, next to roll at exactly -pi; and Battery Status
# Current raw 0x8000, -3276.8 A, the full negative end of an int16 and so a
# reading. In Metric and SI.
#
.PHONY: test-below-range-min
tests: test-below-range-min
test-below-range-min:
	$(ANALYZER) -json < pgn-below-range-min.in > $(TEMPDIR)/pgn-below-range-min.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-below-range-min.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-below-range-min.out
	diff $(TEMPDIR)/pgn-below-range-min.out pgn-below-range-min.out
	diff $(TEMPDIR)/pgn-below-range-min.err pgn-below-range-min.err
	$(ANALYZER) -json -si < pgn-below-range-min.in > $(TEMPDIR)/pgn-below-range-min-si.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-below-range-min-si.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-below-range-min-si.out
	diff $(TEMPDIR)/pgn-below-range-min-si.out pgn-below-range-min-si.out
	diff $(TEMPDIR)/pgn-below-range-min-si.err pgn-below-range-min.err
