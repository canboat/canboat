# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# An unsigned 64-bit field's top three values are its sentinels, and every
# other value is a reading: Maretron 130833's Memory Capacity and Memory
# Used of 5 and 1000, all ones and two below it, and 0x7000000000000001.
# C compared them as int64_t, where all ones is -1, so 5 was "unknown".
#
.PHONY: test-64-bit-sentinels
tests: test-64-bit-sentinels
test-64-bit-sentinels:
	$(ANALYZER) -json < pgn-64-bit-sentinels.in > $(TEMPDIR)/pgn-64-bit-sentinels.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-64-bit-sentinels.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-64-bit-sentinels.out
	diff $(TEMPDIR)/pgn-64-bit-sentinels.out pgn-64-bit-sentinels.out
	diff $(TEMPDIR)/pgn-64-bit-sentinels.err pgn-64-bit-sentinels.err
