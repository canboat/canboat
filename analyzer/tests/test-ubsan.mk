# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# Inputs that made a UBSan build of the analyzer report undefined behaviour
# (#1037): a key/value latitude too wide for its degrees to fit an integer, a
# repeat count of 32 bits, and a UTF-16 string at an odd offset.
#
.PHONY: test-ubsan
tests: test-ubsan
test-ubsan:
	$(ANALYZER) -q -geo dm -format FAST < ubsan-latlon.in > $(TEMPDIR)/ubsan.out -fixtime ubsan 2> $(TEMPDIR)/ubsan.err
	$(ANALYZER) -q -geo dms -format FAST < ubsan-latlon.in >> $(TEMPDIR)/ubsan.out -fixtime ubsan 2>> $(TEMPDIR)/ubsan.err
	$(ANALYZER) -q -format PLAIN_MIX_FAST < ubsan-repeat.in >> $(TEMPDIR)/ubsan.out -fixtime ubsan 2>> $(TEMPDIR)/ubsan.err
	$(ANALYZER) -q < ubsan-utf16.in >> $(TEMPDIR)/ubsan.out -fixtime ubsan 2>> $(TEMPDIR)/ubsan.err
	diff $(TEMPDIR)/ubsan.out ubsan.out
	diff $(TEMPDIR)/ubsan.err ubsan.err
