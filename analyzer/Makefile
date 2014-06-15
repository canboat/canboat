#
# (C) 2009-2014, Kees Verruijt, Harlingen, The Netherlands.
#
# This file is part of CANboat.
#
# CANboat is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CANboat is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with CANboat.  If not, see <http://www.gnu.org/licenses/>.
#

PLATFORM=$(shell uname | tr '[A-Z]' '[a-z]')-$(shell uname -m)
TARGETDIR=../rel/$(PLATFORM)
ANALYZER=$(TARGETDIR)/analyzer
TARGETS=$(ANALYZER)
CFLAGS=-O3
XMLFILE=pgns.xml
JSONFILE=pgns.json

all: $(TARGETS)

$(ANALYZER): analyzer.c analyzer.h pgn.h ../common/common.c ../common/common.h Makefile
	$(CC) $(CFLAGS) -o $(ANALYZER) -I../common analyzer.c ../common/common.c $(LDLIBS$(LDLIBS-$(@)))

json: $(ANALYZER)
	$(ANALYZER) -explain-xml >$(XMLFILE) && xsltproc pgns2json.xslt $(XMLFILE) >$(JSONFILE)

clean:
	-rm -f $(TARGETS) *.elf *.gdb
	-rm -f $(XMLFILE)
	-rm -f $(JSONFILE)

