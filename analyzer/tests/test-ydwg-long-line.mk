# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A YDWG-02 line with more than 8 data bytes is two lines glued together and
# must be rejected (#1001). YDWG-02 lines carry no date, so the parser takes
# the UTC day nearest the -fixtime clock.
#
.PHONY: test-ydwg-long-line
tests: test-ydwg-long-line
test-ydwg-long-line:
	TZ=UTC $(ANALYZER) -json -nv < ydwg-long-line.in > $(TEMPDIR)/ydwg-long-line.out -fixtime ydwg-long-line 2> $(TEMPDIR)/ydwg-long-line.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/ydwg-long-line.out
	diff $(TEMPDIR)/ydwg-long-line.out ydwg-long-line.out
	diff $(TEMPDIR)/ydwg-long-line.err ydwg-long-line.err
