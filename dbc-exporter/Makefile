#
# Makefile for dbc-exporter
#

all: pgns.dbc

pgns.dbc: dbc-exporter
	venv/bin/canboat-dbc-exporter ../analyzer/pgns.json pgns.dbc

dbc-exporter: venv
	venv/bin/python3 setup.py install

venv:
	python3 -m venv venv

.PHONY: all dbc-exporter

clean:
	-venv/bin/python3 setup.py clean
	-rm -rf venv build dist pgns.dbc
