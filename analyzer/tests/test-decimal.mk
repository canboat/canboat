# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# DECIMAL (#962): a PGN 129808 DSC Message Address that is valid, one with a
# byte that is not a digit pair (0xab), and one the packet ends inside. Only
# the first has a value; the others are not available, not a shorter number.
#
# Then the 0 a DSC call adds after a 9-digit MMSI: 2440608070 is MMSI
# 244060807, as AIS gives it; a coast station keeps its leading zeros; and a
# last digit that is not 0 is no padding, so all ten digits stay.
#
# Last, a call to a geographical area (DSC Format 102): its ten digits all
# count, a last 0 too (trailingZeroIsDigit), also in a distress
# acknowledgement (DSC Category 112), whose byte 4 is a 1st telecommand.
# The same address in an individual call is an MMSI and loses it. The last
# acknowledgement has the nature of distress in byte 5 (Man overboard).
#
.PHONY: test-decimal
tests: test-decimal
test-decimal:
	$(ANALYZER) -json < pgn-decimal.in > $(TEMPDIR)/pgn-decimal.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-decimal.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-decimal.out
	diff $(TEMPDIR)/pgn-decimal.out pgn-decimal.out
	diff $(TEMPDIR)/pgn-decimal.err pgn-decimal.err
