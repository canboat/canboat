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
N2KD=$(TARGETDIR)/n2kd
N2KD_MONITOR=$(TARGETDIR)/n2kd_monitor
TARGETS=$(N2KD) $(N2KD_MONITOR)
CFLAGS=-O3

all: $(TARGETS)

$(N2KD): main.c nmea0183.c nmea0183.h ../common/common.c ../common/common.h Makefile
	$(CC) $(CFLAGS) -o $(N2KD) -I../common main.c nmea0183.c ../common/common.c $(LDLIBS$(LDLIBS-$(@)))

$(N2KD_MONITOR): n2kd_monitor
	cp n2kd_monitor $(N2KD_MONITOR)

clean:
	-rm -f $(N2KD) *.elf *.gdb

