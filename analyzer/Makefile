#
# (C) 2009-2021, Kees Verruijt, Harlingen, The Netherlands.
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
TARGETDIR=../rel/$(PLATFORM)
ANALYZER=$(TARGETDIR)/analyzer
ANALYZER_EXPLAIN=$(TARGETDIR)/analyzer-explain
TARGETS=$(ANALYZER) $(ANALYZER_EXPLAIN)
XMLFILE=pgns.xml
JSONFILE=pgns.json
XML2FILE=../docs/canboat.xml
XSD2FILE=../docs/canboat.xsd
JSON2FILE=../docs/canboat.json
NGTXMLFILE=pgns-ngt.xml
NGTJSONFILE=pgns-ngt.json
IKXMLFILE=pgns-ik.xml
IKJSONFILE=pgns-ik.json
NPMFILE=package.json
HEADERS=analyzer.h pgn.h lookup.h fieldtype.h
COMMONDIR=../common
COMMON=$(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(COMMONDIR)/common.h $(COMMONDIR)/license.h $(COMMONDIR)/utf.h
ANALYZER_EXPLAIN_SOURCES=analyzer-explain.c pgn.c lookup.c print.c fieldtype.c $(HEADERS) $(COMMON) Makefile
ANALYZER_EXPLAIN_DEP=$(ANALYZER_EXPLAIN) $(ANALYZER_EXPLAIN_SOURCES)

CFLAGS?=-Wall -O2
LDLIBS=-lm

all: $(TARGETS)

$(ANALYZER): analyzer.c pgn.c lookup.c print.c fieldtype.c $(HEADERS) $(COMMON) Makefile
	@mkdir -p $(TARGETDIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER) -I$(COMMONDIR) pgn.c analyzer.c lookup.c print.c fieldtype.c $(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(LDLIBS$(LDLIBS-$(@)))

$(ANALYZER_EXPLAIN): $(ANALYZER_EXPLAIN_SOURCES)
	@mkdir -p $(TARGETDIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER_EXPLAIN) -I$(COMMONDIR) pgn.c analyzer-explain.c lookup.c print.c fieldtype.c $(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(LDLIBS$(LDLIBS-$(@)))

$(XMLFILE): $(ANALYZER_EXPLAIN_DEP)
	$(ANALYZER_EXPLAIN) -explain-xml -expand-lookups >$(XMLFILE)

$(XML2FILE): $(ANALYZER_EXPLAIN_DEP) $(XSD2FILE)
	$(ANALYZER_EXPLAIN) -explain-xml >$(XML2FILE) && xmllint --schema $(XSD2FILE) $(XML2FILE) --noout


$(NGTXMLFILE): $(ANALYZER_EXPLAIN_DEP)
	$(ANALYZER_EXPLAIN) -explain-ngt-xml >$(NGTXMLFILE)

$(IKXMLFILE): $(ANALYZER_EXPLAIN_DEP)
	$(ANALYZER_EXPLAIN) -explain-ik-xml >$(IKXMLFILE)

$(JSONFILE): $(XMLFILE) pgns2json.xslt
	xsltproc pgns2json.xslt $(XMLFILE) >$(JSONFILE)

$(JSON2FILE): $(XML2FILE) pgns2json.xslt
	xsltproc pgns2json.xslt $(XML2FILE) >$(JSON2FILE)

$(NGTJSONFILE): $(NGTXMLFILE) pgns2json.xslt
	xsltproc pgns2json.xslt $(NGTXMLFILE) >$(NGTJSONFILE)

$(IKJSONFILE): $(IKXMLFILE) pgns2json.xslt
	xsltproc pgns2json.xslt $(IKXMLFILE) >$(IKJSONFILE)

$(NPMFILE): ../common/version.h $(ANALYZER)
	./fixup-version.sh $(ANALYZER) $(NPMFILE)

json: $(JSONFILE) $(JSON2FILE) $(NGTJSONFILE) $(IKJSONFILE) $(NPMFILE)

clean:
	-rm -f $(TARGETS) *.elf *.gdb
	-rm -f $(XMLFILE)
	-rm -f $(JSONFILE)

webserver:
	cd ../docs; python3 -m http.server --cgi 8080
