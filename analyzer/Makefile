#
# (C) 2009-2023, Kees Verruijt, Harlingen, The Netherlands.
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

PLATFORM=$(shell uname | tr '[A-Z]' '[a-z]')-$(shell uname -m)
BUILDDIR?=rel/$(PLATFORM)
TARGETDIR=../$(BUILDDIR)
ANALYZER=$(TARGETDIR)/analyzer
ANALYZER_EXPLAIN=$(TARGETDIR)/analyzer-explain
TARGETS=$(ANALYZER) $(ANALYZER_EXPLAIN)
XMLFILE=pgns.xml
JSONFILE=pgns.json
XSL2FILE=../docs/canboat.xsl
XML2FILE=../docs/canboat.xml
XSD2FILE=../docs/canboat.xsd
JSON2FILE=../docs/canboat.json
HTML2FILE=../docs/canboat.html
NGTXMLFILE=pgns-ngt.xml
NGTJSONFILE=pgns-ngt.json
IKXMLFILE=pgns-ik.xml
IKJSONFILE=pgns-ik.json
NPMFILE=package.json
HEADERS=analyzer.h pgn.h lookup.h fieldtype.h
COMMONDIR=../common
COMMON=$(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(COMMONDIR)/common.h $(COMMONDIR)/license.h $(COMMONDIR)/utf.h $(COMMONDIR)/version.h
ANALYZER_EXPLAIN_SOURCES=analyzer-explain.c pgn.c lookup.c print.c fieldtype.c $(HEADERS) $(COMMON) Makefile
ANALYZER_EXPLAIN_DEP=$(ANALYZER_EXPLAIN) $(ANALYZER_EXPLAIN_SOURCES)

CFLAGS?=-Wall -O2
LDLIBS=-lm

all: $(TARGETS)

analyzer: $(ANALYZER)

$(ANALYZER): analyzer.c pgn.c lookup.c print.c fieldtype.c $(HEADERS) $(COMMON) Makefile
	@mkdir -p $(TARGETDIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER) -I$(COMMONDIR) pgn.c analyzer.c lookup.c print.c fieldtype.c $(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(LDLIBS$(LDLIBS-$(@)))

$(ANALYZER_EXPLAIN): $(ANALYZER_EXPLAIN_SOURCES)
	@mkdir -p $(TARGETDIR)
	$(CC) -DEXPLAIN $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER_EXPLAIN) -I$(COMMONDIR) pgn.c analyzer-explain.c lookup.c print.c fieldtype.c $(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(LDLIBS$(LDLIBS-$(@)))

$(XMLFILE): $(ANALYZER_EXPLAIN_DEP)
	$(ANALYZER_EXPLAIN) -explain-xml -camel -v1 >$(XMLFILE)

$(XML2FILE): $(ANALYZER_EXPLAIN_DEP) $(XSD2FILE) fixup-version.py
	python3 fixup-version.py $(ANALYZER) $(XSD2FILE)
	python3 fixup-version.py -s $(ANALYZER) $(XSD2FILE)
	$(ANALYZER_EXPLAIN) -explain-xml -camel >$(XML2FILE)
	xmllint --schema $(XSD2FILE) $(XML2FILE) --noout

$(HTML2FILE): $(XSL2FILE) $(XML2FILE)
	xsltproc $(XSL2FILE) $(XML2FILE) | ./fixup-html.sh > $(HTML2FILE)

$(NGTXMLFILE): $(ANALYZER_EXPLAIN_DEP)
	$(ANALYZER_EXPLAIN) -explain-ngt-xml -camel -v1 >$(NGTXMLFILE)

$(IKXMLFILE): $(ANALYZER_EXPLAIN_DEP)
	$(ANALYZER_EXPLAIN) -explain-ik-xml -camel -v1 >$(IKXMLFILE)

$(JSONFILE): $(XMLFILE) pgns2json.xslt validate-json.py
	xsltproc pgns2json.xslt $(XMLFILE) >$(@)
	python3 validate-json.py $(@)

$(JSON2FILE): $(XML2FILE) canboat2json.xslt validate-json.py
	xsltproc canboat2json.xslt $(XML2FILE) >$(@)
	python3 validate-json.py --range $(@)

$(NGTJSONFILE): $(NGTXMLFILE) pgns2json.xslt validate-json.py
	xsltproc pgns2json.xslt $(NGTXMLFILE) >$(@)
	python3 validate-json.py $(@)

$(IKJSONFILE): $(IKXMLFILE) pgns2json.xslt validate-json.py
	xsltproc pgns2json.xslt $(IKXMLFILE) >$(@)
	python3 validate-json.py $(@)

$(NPMFILE): ../common/version.h $(ANALYZER) fixup-version.py
	python3 fixup-version.py $(ANALYZER) $(NPMFILE)

generated: $(JSONFILE) $(JSON2FILE) $(NGTJSONFILE) $(IKJSONFILE) $(NPMFILE) $(HTML2FILE)

clean:
	-rm -f $(TARGETS) *.elf *.gdb

tests:	$(ANALYZER)
	(cd tests; make tests)

webserver:
	cd ../docs; python3 -m http.server --cgi 8080

.PHONY:	generated clean tests webserver analyzer
