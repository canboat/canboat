# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

.PHONY: test-switch-multi-to-one-line
tests: test-switch-multi-to-one-line
test-switch-multi-to-one-line:	
	$(ANALYZER) < switch-multi-to-one-line.in > $(TEMPDIR)/switch-multi-to-one-line.out -debug -q -fixtime switch-multi-to-one-line 2> $(TEMPDIR)/switch-multi-to-one-line.err
	diff $(TEMPDIR)/switch-multi-to-one-line.out switch-multi-to-one-line.out
	diff $(TEMPDIR)/switch-multi-to-one-line.err switch-multi-to-one-line.err
