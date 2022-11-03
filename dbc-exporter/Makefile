#
# Makefile for dbc-exporter
#

PLATFORM=$(shell uname | tr '[A-Z]' '[a-z]')-$(shell uname -m)
BUILDDIR?=rel/$(PLATFORM)
TARGETDIR=../$(BUILDDIR)
VENV=$(TARGETDIR)/venv


all: pgns.dbc

pgns.dbc: dbc-exporter ../analyzer/pgns.json
	"$(VENV)/bin/canboat-dbc-exporter" ../analyzer/pgns.json pgns.dbc
	"$(VENV)/bin/python3" ../analyzer/fixup-version.py -w $(TARGETDIR)/analyzer pgns.dbc

dbc-exporter: $(VENV)
	"$(VENV)/bin/python3" setup.py install

$(VENV):
	python3 -m venv "$(VENV)"

.PHONY: all dbc-exporter

clean:
	-[ -x "$(VENV)/bin/python3" ] && "$(VENV)/bin/python3" setup.py clean || :
	-rm -rf "$(VENV)" build dist
