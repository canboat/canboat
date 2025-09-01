#
# (C) 2009-2025, Kees Verruijt, Harlingen, The Netherlands.
#  
# This file is part of CANboat.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 

PLATFORM?=$(shell uname | tr '[A-Z]' '[a-z]')-$(shell uname -m)
BUILDDIR?=rel/$(PLATFORM)
TARGETDIR=../$(BUILDDIR)
ANALYZER=$(TARGETDIR)/analyzer
ANALYZER_J1939=$(TARGETDIR)/analyzer-j1939
ANALYZER_EXPLAIN=$(TARGETDIR)/analyzer-explain
ANALYZER_EXPLAIN_J1939=$(TARGETDIR)/analyzer-explain-j1939
TARGETS=$(ANALYZER) $(ANALYZER_J1939) $(ANALYZER_EXPLAIN) $(ANALYZER_EXPLAIN_J1939)
NMEA_XMLFILE=../sources/NMEA_database_1_300.xml
XSL2FILE=../docs/canboat.xsl
XML2FILE=../docs/canboat.xml
XSD2FILE=../docs/canboat.xsd
JSON2FILE=../docs/canboat.json
HTML2FILE=../docs/canboat.html
CSV_XSLFILE=./canboat-csv.xsl
HEADERS=analyzer.h pgn.h lookup.h fieldtype.h
HEADERS_J1939=analyzer.h pgn-j1939.h lookup.h fieldtype.h
COMMONDIR=../common
COMMON=$(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(COMMONDIR)/common.h $(COMMONDIR)/license.h $(COMMONDIR)/utf.h $(COMMONDIR)/version.h
ANALYZER_EXPLAIN_SOURCES=analyzer-explain.c pgn.c lookup.c print.c fieldtype.c $(HEADERS) $(COMMON) Makefile
ANALYZER_EXPLAIN_J1939_SOURCES=analyzer-explain.c pgn.c lookup.c print.c fieldtype.c $(HEADERS_J1939) $(COMMON) Makefile
ANALYZER_EXPLAIN_DEP=$(ANALYZER_EXPLAIN) $(ANALYZER_EXPLAIN_SOURCES)
ANALYZER_EXPLAIN_J1939_DEP=$(ANALYZER_EXPLAIN_J1939) $(ANALYZER_EXPLAIN_J1939_SOURCES)

CFLAGS?=-Wall -O2
LDLIBS=-lm

all: $(TARGETS)

analyzer: $(ANALYZER)

$(ANALYZER): analyzer.c pgn.c lookup.c print.c fieldtype.c $(HEADERS) $(COMMON) Makefile
	@mkdir -p $(TARGETDIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER) -I$(COMMONDIR) pgn.c analyzer.c lookup.c print.c fieldtype.c $(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(LDLIBS$(LDLIBS-$(@)))

$(ANALYZER_J1939): analyzer.c pgn.c lookup.c print.c fieldtype.c $(HEADERS_J1939) $(COMMON) Makefile
	@mkdir -p $(TARGETDIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER_J1939) -I$(COMMONDIR) -DJ1939 pgn.c analyzer.c lookup.c print.c fieldtype.c $(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(LDLIBS$(LDLIBS-$(@)))

$(ANALYZER_EXPLAIN): $(ANALYZER_EXPLAIN_SOURCES)
	@mkdir -p $(TARGETDIR)
	$(CC) -DEXPLAIN $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER_EXPLAIN) -I$(COMMONDIR) pgn.c analyzer-explain.c lookup.c print.c fieldtype.c $(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(LDLIBS$(LDLIBS-$(@)))

$(ANALYZER_EXPLAIN_J1939): $(ANALYZER_EXPLAIN_J1939_SOURCES)
	@mkdir -p $(TARGETDIR)
	$(CC) -DEXPLAIN -DJ1939 $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER_EXPLAIN_J1939) -I$(COMMONDIR) pgn.c analyzer-explain.c lookup.c print.c fieldtype.c $(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(LDLIBS$(LDLIBS-$(@)))

$(XML2FILE): $(ANALYZER_EXPLAIN_DEP) $(XSD2FILE) fixup-version.py
	python3 fixup-version.py $(ANALYZER) $(XSD2FILE)
	python3 fixup-version.py -s $(ANALYZER) $(XSD2FILE)
	$(ANALYZER_EXPLAIN) -explain-xml -camel >$(XML2FILE)
	xmllint --schema $(XSD2FILE) $(XML2FILE) --noout

$(HTML2FILE): $(XSL2FILE) $(XML2FILE)
	xsltproc $(XSL2FILE) $(XML2FILE) | ./fixup-html.sh > $(HTML2FILE)

$(JSON2FILE): $(XML2FILE) canboat2json.xslt validate-json.py
	xsltproc canboat2json.xslt $(XML2FILE) >$(@)
	python3 validate-json.py --range $(@)

validation: $(ANALYZER_EXPLAIN) $(XML2FILE) $(NMEA_XMLFILE) $(CSV_XSLFILE)
	xsltproc $(CSV_XSLFILE) $(XML2FILE) > /tmp/canboat.csv
	xsltproc $(CSV_XSLFILE) $(NMEA_XMLFILE) > /tmp/nmea.csv
	python3 validate-csv.py /tmp/canboat.csv /tmp/nmea.csv
	

generated: validation $(JSON2FILE) $(HTML2FILE) 

clean:
	-rm -f $(TARGETS) *.elf *.gdb

tests:	$(ANALYZER)
	(cd tests; make tests)

webserver:
	cd ../docs; python3 -m http.server --cgi 8080

.PHONY:	generated clean tests webserver analyzer validation
