# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A J1939 PGN whose generated field names are longer than 60 bytes, such as
# 64946's "Aftertreatment 1 Diesel Particulate Filter Intermediate Temperature
# Preliminary FMI" (83 bytes). printFields() once copied each name into a
# 60-byte stack buffer with strcpy, so a single valid frame overflowed it; the
# buffer is now 256 bytes and the copies are bounded. This decodes the frame
# and checks it prints the long names without crashing.
#
.PHONY: test-j1939-long-field-name
tests: test-j1939-long-field-name
test-j1939-long-field-name:
	$(ANALYZER_J1939) -json < j1939-long-field-name.in > $(TEMPDIR)/j1939-long-field-name.out -fixtime pgn-test 2> $(TEMPDIR)/j1939-long-field-name.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/j1939-long-field-name.out
	diff $(TEMPDIR)/j1939-long-field-name.out j1939-long-field-name.out
	diff $(TEMPDIR)/j1939-long-field-name.err j1939-long-field-name.err
