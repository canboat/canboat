# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

.PHONY: test-recombine-frames
tests: test-recombine-frames
test-recombine-frames:
	$(ANALYZER) < recombine-frames.in > $(TEMPDIR)/recombine-frames.out -debug -q -fixtime recombine 2> $(TEMPDIR)/recombine-frames.err
	diff $(TEMPDIR)/recombine-frames.out recombine-frames.out
	diff $(TEMPDIR)/recombine-frames.err recombine-frames.err
