/*

Convert JSON encoded NMEA2000 PGNs to NMEA0183.

At this moment it only supports what one of the authors needed: sensor data
other than GPS and AIS: depth, heading, wind.

(C) 2009-2023, Kees Verruijt, Harlingen, The Netherlands.

This file is part of CANboat.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#include "common.h"

/*
 * NMEA 2000 is all ISO units, so m/s, deg K, etc. except for 'degrees',
 *
 * NMEA 0183 uses various units, including metric derived and colonial.
 */
#define SPEED_M_S_TO_KNOTS(s) (s * 1.94384)
#define SPEED_M_S_TO_KMH(s) ((s) *3.6)
#define DIST_M_TO_KM(d) ((d) / 1000.0)
#define DIST_M_TO_NM(d) ((d) / 1852.0)

// For temperature, original contributor (Julius Pabrinkis) asserts that his
// DST800 shows value in Celcius, but I (Kees) really doubt this.
// By checking for a 'ridiculous' value in kelvin, we can have our cake and eat it.
// Anything below 173 deg K is assumed to be really in Celcius.
#define TEMP_K_TO_C(t) (((t) < 173.15) ? (t) : ((t) -273.15))

extern void convertJSONToNMEA0183(StringBuffer *msg183, const char *msg);
extern void nmea0183CreateMessage(StringBuffer *msg183, int src, const char *format, ...);
