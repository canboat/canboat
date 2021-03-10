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
TARGETS=$(ANALYZER)
XMLFILE=pgns.xml
JSONFILE=pgns.json
NGTXMLFILE=pgns-ngt.xml
NGTJSONFILE=pgns-ngt.json
IKXMLFILE=pgns-ik.xml
IKJSONFILE=pgns-ik.json

CFLAGS?=-Wall -O2

all: $(TARGETS)

$(ANALYZER): analyzer.c pgn.c analyzer.h pgn.h ../common/common.c ../common/common.h ../common/license.h Makefile
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(ANALYZER) -I../common pgn.c analyzer.c ../common/common.c $(LDLIBS$(LDLIBS-$(@)))

json: $(ANALYZER) pgns2json.xslt
	$(ANALYZER) -explain-xml >$(XMLFILE) && xsltproc pgns2json.xslt $(XMLFILE) >$(JSONFILE)
	$(ANALYZER) -explain-ngt-xml >$(NGTXMLFILE) && xsltproc pgns2json.xslt $(NGTXMLFILE) >$(NGTJSONFILE)
	$(ANALYZER) -explain-ik-xml >$(IKXMLFILE) && xsltproc pgns2json.xslt $(IKXMLFILE) >$(IKJSONFILE)

clean:
	-rm -f $(TARGETS) *.elf *.gdb
	-rm -f $(XMLFILE)
	-rm -f $(JSONFILE)

