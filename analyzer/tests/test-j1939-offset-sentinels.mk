# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# The same for J1939 temperatures in Excess-K notation (#984), where raw 0 is
# 233.15 K (-40 C): PGN 65262 with a coolant and fuel temperature, then the
# top coolant value 485.15 K next to sentinels, then all 0xff. In Metric and SI.
#
.PHONY: test-j1939-offset-sentinels
tests: test-j1939-offset-sentinels
test-j1939-offset-sentinels:
	$(ANALYZER_J1939) -json < j1939-offset-sentinels.in > $(TEMPDIR)/j1939-offset-sentinels.out -fixtime pgn-test 2> $(TEMPDIR)/j1939-offset-sentinels.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/j1939-offset-sentinels.out
	diff $(TEMPDIR)/j1939-offset-sentinels.out j1939-offset-sentinels.out
	diff $(TEMPDIR)/j1939-offset-sentinels.err j1939-offset-sentinels.err
	$(ANALYZER_J1939) -json -si < j1939-offset-sentinels.in > $(TEMPDIR)/j1939-offset-sentinels-si.out -fixtime pgn-test 2> $(TEMPDIR)/j1939-offset-sentinels-si.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/j1939-offset-sentinels-si.out
	diff $(TEMPDIR)/j1939-offset-sentinels-si.out j1939-offset-sentinels-si.out
	diff $(TEMPDIR)/j1939-offset-sentinels-si.err j1939-offset-sentinels.err
