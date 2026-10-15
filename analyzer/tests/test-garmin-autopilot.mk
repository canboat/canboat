# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests the Garmin GHP Reactor autopilot decode over PGN 126720 (sub-protocol 5904):
# Mode State lookup (2 Standby, 3 Shadow Drive, 5 Engaged) and System Voltage at 10 mV/bit
# (~13 V, not the 1.3 V a 1 mV resolution would give). Frames from samples/garmin-ap-reactor-126720.raw.
#
.PHONY: test-garmin-autopilot
tests: test-garmin-autopilot
test-garmin-autopilot:
	$(ANALYZER) -json -nv < pgn-garmin-autopilot.in > $(TEMPDIR)/pgn-garmin-autopilot.out -fixtime pgn-garmin-autopilot 2> $(TEMPDIR)/pgn-garmin-autopilot.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-garmin-autopilot.out
	diff $(TEMPDIR)/pgn-garmin-autopilot.out pgn-garmin-autopilot.out
	diff $(TEMPDIR)/pgn-garmin-autopilot.err pgn-garmin-autopilot.err
