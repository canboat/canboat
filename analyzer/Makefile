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
TARGETS=$(ANALYZER)
XMLFILE=pgns.xml
JSONFILE=pgns.json
NGTXMLFILE=pgns-ngt.xml
NGTJSONFILE=pgns-ngt.json
IKXMLFILE=pgns-ik.xml
IKJSONFILE=pgns-ik.json
HEADERS=analyzer.h pgn.h lookup.h
COMMONDIR=../common
COMMON=$(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/common.h $(COMMONDIR)/license.h

CFLAGS?=-Wall -O2

all: $(TARGETS)

$(ANALYZER): analyzer.c pgn.c lookup.c $(HEADERS) $(COMMON) Makefile
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER) -I$(COMMONDIR) pgn.c analyzer.c lookup.c $(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(LDLIBS$(LDLIBS-$(@)))

$(ANALYZER_EXPLAIN): analyzer-explain.c pgn.c lookup.c $(HEADERS) $(COMMON) Makefile
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER_EXPLAIN) -I$(COMMONDIR) pgn.c analyzer-explain.c lookup.c $(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(LDLIBS$(LDLIBS-$(@)))

json: $(ANALYZER_EXPLAIN) pgns2json.xslt
	$(ANALYZER_EXPLAIN) -explain-xml >$(XMLFILE) && xsltproc pgns2json.xslt $(XMLFILE) >$(JSONFILE)
	$(ANALYZER_EXPLAIN) -explain-ngt-xml >$(NGTXMLFILE) && xsltproc pgns2json.xslt $(NGTXMLFILE) >$(NGTJSONFILE)
	$(ANALYZER_EXPLAIN) -explain-ik-xml >$(IKXMLFILE) && xsltproc pgns2json.xslt $(IKXMLFILE) >$(IKJSONFILE)

clean:
	-rm -f $(TARGETS) *.elf *.gdb
	-rm -f $(XMLFILE)
	-rm -f $(JSONFILE)

