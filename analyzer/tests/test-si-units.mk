# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# -si: the database keeps NMEA 2000's kWh and Ah; strict SI output is J and C
# (65005 energy, 127506 / 127513 charge).
#
.PHONY: test-si-units
tests: test-si-units
test-si-units:
	$(ANALYZER) -json -si < pgn-si-units.in > $(TEMPDIR)/pgn-si-units.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-si-units.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-si-units.out
	diff $(TEMPDIR)/pgn-si-units.out pgn-si-units.out
	diff $(TEMPDIR)/pgn-si-units.err pgn-si-units.err
