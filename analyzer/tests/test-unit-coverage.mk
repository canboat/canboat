# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A real value for every unit the database uses, in Metric and SI: the AC
# quantities (A, VA, VAR, Cos Phi, W with its offset), Airmar compass
# calibration (T, an angle in degrees, durations), position deltas (degrees
# that are no angle), the GPS almanac (m/m, sqrt(m), s/s, semi-circles),
# 130321 salinity (ppt, a FLOAT), and dynamic values in Ah, %, A, V, K, W
# (Victron 61184), L and % (Mercury 130824) and K and % (B&G 130824).
#
.PHONY: test-unit-coverage
tests: test-unit-coverage
test-unit-coverage:
	$(ANALYZER) -json < pgn-unit-coverage.in > $(TEMPDIR)/pgn-unit-coverage.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-unit-coverage.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-unit-coverage.out
	diff $(TEMPDIR)/pgn-unit-coverage.out pgn-unit-coverage.out
	diff $(TEMPDIR)/pgn-unit-coverage.err pgn-unit-coverage.err
	$(ANALYZER) -json -si < pgn-unit-coverage.in > $(TEMPDIR)/pgn-unit-coverage-si.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-unit-coverage-si.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-unit-coverage-si.out
	diff $(TEMPDIR)/pgn-unit-coverage-si.out pgn-unit-coverage-si.out
	diff $(TEMPDIR)/pgn-unit-coverage-si.err pgn-unit-coverage.err
