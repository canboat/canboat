# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

# SeaSmart $PCDIN (#1047): canboatjs's sentence, also behind a TAG block,
# Chetco's specification examples, a whole fast-packet, and other NMEA 0183
# sentences before and among them, which hold no message. The last four
# lines are bad: junk after the data, a surplus field, a wrong checksum, and
# junk after the checksum.
.PHONY: test-seasmart
tests: test-seasmart
test-seasmart:
	$(ANALYZER) -json -nv < seasmart.in > $(TEMPDIR)/seasmart.out -fixtime seasmart 2> $(TEMPDIR)/seasmart.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/seasmart.out
	diff $(TEMPDIR)/seasmart.out seasmart.out
	diff $(TEMPDIR)/seasmart.err seasmart.err
	# -ignore-checksum takes the last two lines too: four 127257s in all.
	test `$(ANALYZER) -json -ignore-checksum < seasmart.in 2>/dev/null | grep -c '"pgn":127257'` = 4
