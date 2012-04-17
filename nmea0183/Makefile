#
# nmea0183/Makefile
#
# (C) 2009-2012, Kees Verruijt, Harlingen, The Netherlands.
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
TARGET1=../rel/$(PLATFORM)/nmea0183-serial
TARGET2=../rel/$(PLATFORM)/iptee
TARGETLIST=$(TARGET1) $(TARGET2)
CFLAGS=-O3

all:	$(TARGETDIR) $(TARGETLIST)

$(TARGETDIR):	
	mkdir -p $(TARGETDIR)

clean:
	-rm -f $(TARGETLIST) *.elf *.gdb

$(TARGET1):	nmeareader.c ../common/common.c ../common/common.h
	$(CC) -o $(TARGET1) $(CFLAGS) -I../common nmeareader.c ../common/common.c

$(TARGET2):	iptee.c ../common/common.c ../common/common.h
	$(CC) -o $(TARGET2) $(CFLAGS) -I../common iptee.c ../common/common.c
