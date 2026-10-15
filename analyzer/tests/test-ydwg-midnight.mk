# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A YDWG-02 time of day just after midnight is on the next day, even when the
# host clock is still before it (canboat#1039).
#
.PHONY: test-ydwg-midnight
tests: test-ydwg-midnight
test-ydwg-midnight:
	$(ANALYZER) -json -nv < ydwg-midnight.in > $(TEMPDIR)/ydwg-midnight.out -fixtime ydwg-midnight 2> $(TEMPDIR)/ydwg-midnight.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/ydwg-midnight.out
	diff $(TEMPDIR)/ydwg-midnight.out ydwg-midnight.out
	diff $(TEMPDIR)/ydwg-midnight.err ydwg-midnight.err
