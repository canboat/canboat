# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A PLAIN line whose length field announces more data bytes than are on the line
# must be rejected, not padded with whatever happened to be on the stack.
#
.PHONY: test-plain-short-data
tests: test-plain-short-data
test-plain-short-data:
	$(ANALYZER) -json < plain-short-data.in > $(TEMPDIR)/plain-short-data.out -fixtime pgn-test 2> $(TEMPDIR)/plain-short-data.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/plain-short-data.out
	diff $(TEMPDIR)/plain-short-data.out plain-short-data.out
	diff $(TEMPDIR)/plain-short-data.err plain-short-data.err
