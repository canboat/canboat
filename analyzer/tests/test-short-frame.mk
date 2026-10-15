# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that short fast-packet first frames don't read uninitialized memory (issue #623)
#
.PHONY: test-short-frame
tests: test-short-frame
test-short-frame:
	$(ANALYZER) < short-frame.in > $(TEMPDIR)/short-frame.out -debug -q -fixtime short-frame 2> $(TEMPDIR)/short-frame.err
	diff $(TEMPDIR)/short-frame.out short-frame.out
	diff $(TEMPDIR)/short-frame.err short-frame.err
