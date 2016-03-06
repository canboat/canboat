#
# (C) 2009-2015, Kees Verruijt, Harlingen, The Netherlands.
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
SOCKETCAN_WRITER=$(TARGETDIR)/socketcan-writer

ifeq ($(shell uname),Linux)
TARGETS=$(SOCKETCAN_WRITER)
else
TARGETS=
endif

all: $(TARGETS)

$(SOCKETCAN_WRITER): socketcan-writer.c ../analyzer/*.h ../analyzer/*.c ../common/*.h ../common/*.c
	$(CC) -o $(SOCKETCAN_WRITER) -I../analyzer -I../common socketcan-writer.c ../analyzer/pgn.c ../common/common.c $(LDLIBS$(LDLIBS-$(@)))

clean:
	-rm -f $(TARGETS) *.elf *.gdb

