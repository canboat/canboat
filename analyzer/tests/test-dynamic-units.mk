# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# Dynamic field values take the unit system too: a field-type lookup entry is
# fixed up just like a field. Victron VE.Can registers 784 Discharged Energy
# (kWh), 60908 Battery Temperature (K) and 4096 Battery Capacity (Ah), and
# Simnet 130845 Heading Offset (rad), in metric and in -si.
#
.PHONY: test-dynamic-units
tests: test-dynamic-units
test-dynamic-units:
	$(ANALYZER) -json < pgn-dynamic-units.in > $(TEMPDIR)/pgn-dynamic-units.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-dynamic-units.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-dynamic-units.out
	diff $(TEMPDIR)/pgn-dynamic-units.out pgn-dynamic-units.out
	diff $(TEMPDIR)/pgn-dynamic-units.err pgn-dynamic-units.err
	$(ANALYZER) -json -si < pgn-dynamic-units.in > $(TEMPDIR)/pgn-dynamic-units-si.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-dynamic-units-si.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-dynamic-units-si.out
	diff $(TEMPDIR)/pgn-dynamic-units-si.out pgn-dynamic-units-si.out
	diff $(TEMPDIR)/pgn-dynamic-units-si.err pgn-dynamic-units.err
