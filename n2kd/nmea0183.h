/*

Convert JSON encoded NMEA2000 PGNs to NMEA0183.

At this moment it only supports what one of the authors needed: sensor data
other than GPS and AIS: depth, heading, wind.

(C) 2009-2013, Kees Verruijt, Harlingen, The Netherlands.

This file is part of CANboat.

CANboat is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

CANboat is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with CANboat.  If not, see <http://www.gnu.org/licenses/>.

*/


#include "common.h"

extern void convertJSONToNMEA0183( StringBuffer * msg183, const char * msg );
