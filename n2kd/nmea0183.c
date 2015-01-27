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
#include <math.h>

#include "nmea0183.h"
#include "n2kd.h"

extern char * srcFilter;
extern bool rateLimit;

/*
 * Which PGNs do we care and know about for now?
 *
 * NMEA 0183 information from the excellent reference at
 * http://gpsd.berlios.de/NMEA.txt
 *
 * PGN 127250 "Vessel Heading" -> $xxHDG
 * PGN 130306 "Wind Data"      -> $xxMWV
 * PGN 128267 "Water Depth"    -> $xxDBK/DBS/DBT
 *
 * Typical output of these from analyzer:
 * {"timestamp":"2010-09-12-10:57:41.217","prio":"2","src":"36","dst":"255","pgn":"127250","description":"Vessel Heading","fields":{"SID":"116","Heading":"10.1","Deviation":"0.0","Variation":"0.0","Reference":"Magnetic"}}
 * {"timestamp":"2010-09-12-11:00:20.269","prio":"2","src":"13","dst":"255","pgn":"130306","description":"Wind Data","fields":{"Wind Speed":"5.00","Wind Angle":"308.8","Reference":"Apparent"}}
 * {"timestamp":"2012-12-01-12:53:19.929","prio":"3","src":"35","dst":"255","pgn":"128267","description":"Water Depth","fields":{"SID":"70","Depth":"0.63","Offset":"0.500"}}
 *
 */

#define PGN_VESSEL_HEADING (127250)
#define PGN_WIND_DATA      (130306)
#define PGN_WATER_DEPTH    (128267)

#define VESSEL_HEADING (0)
#define WIND_DATA      (1)
#define WATER_DEPTH    (2)
#define SENTENCE_COUNT (3)

static int64_t rateLimitPassed[256][SENTENCE_COUNT];

static void nmea0183CreateMessage( StringBuffer * msg183, int src, const char * format, ...)
{
  char line[80];
  int chk;
  size_t len;
  size_t i;
  va_list ap;

  va_start(ap, format);

  // Convert the 8 bit value 'n' into a valid NMEA0183 style sender.
  // The first implementation sent out a 2 digit hexadecimal number,
  // but that throws some implementations of receivers off as they
  // cannot handle numeric senders. So now we produce a 2 character
  // code with the src value 0-255 translated into
  // A..N A..N with A representing 0, B representing 1, etc.

  snprintf(line, sizeof(line), "$%c%c"
          , ('A' + ((src >> 4) & 0xf))
          , ('A' + ((src     ) & 0xf))
          );
  vsnprintf(line + 3, sizeof(line) - 3, format, ap);
  va_end(ap);

  len = strlen(line);
  chk = 0;
  for (i = 1; i < len; i++)
  {
    chk ^= line[i];
  }
  sbAppendString(msg183, line);
  snprintf(line, sizeof(line), "*%02X\r\n", chk);
  sbAppendString(msg183, line);
  logDebug("nmea0183 = %s", sbGet(msg183));
}

/*

=== HDG - Heading - Deviation & Variation ===

------------------------------------------------------------------------------
        1   2   3 4   5 6
        |   |   | |   | |
 $--HDG,x.x,x.x,a,x.x,a*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Magnetic Sensor heading in degrees
2. Magnetic Deviation, degrees
3. Magnetic Deviation direction, E = Easterly, W = Westerly
4. Magnetic Variation degrees
5. Magnetic Variation direction, E = Easterly, W = Westerly
6. Checksum


=== HDM - Heading - Magnetic ===

Vessel heading in degrees with respect to magnetic north produced by
any device or system producing magnetic heading.

------------------------------------------------------------------------------
        1   2 3
        |   | |
 $--HDM,x.x,M*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Heading Degrees, magnetic
2. M = magnetic
3. Checksum

=== HDT - Heading - True ===

Actual vessel heading in degrees true produced by any device or system
producing true heading.

------------------------------------------------------------------------------
        1   2 3
        |   | |
 $--HDT,x.x,T*hh<CR><LF>
------------------------------------------------------------------------------

Field Number: 

1. Heading Degrees, true
2. T = True
3. Checksum
*/
static void nmea0183VesselHeading( StringBuffer * msg183, int src, const char * msg )
{
  char heading[10];
  char deviation[10];
  char variation[10];
  char reference[10];

  if (!getJSONValue(msg, "Heading", heading, sizeof(heading))
   || !getJSONValue(msg, "Reference", reference, sizeof(reference)))
  {
    return;
  }
  if (getJSONValue(msg, "Deviation", deviation, sizeof(deviation))
   && getJSONValue(msg, "Variation", variation, sizeof(variation))
   && strcmp(reference, "Magnetic") == 0)
  {
    /* Enough info for HDG message */
    double dev = strtod(deviation, 0);
    double var = strtod(variation, 0);

    nmea0183CreateMessage(msg183, src, "HDG,%s,%04.1f,%c,%04.1f,%c"
                         , heading
                         , fabs(dev)
                         , ((dev < 0.0) ? 'W' : 'E')
                         , fabs(var)
                         , ((var < 0.0) ? 'W' : 'E')
                         );
  }
  else if (strcmp(reference, "True") == 0)
  {
    nmea0183CreateMessage(msg183, src, "HDT,%s,T", heading);
  }
  else if (strcmp(reference, "Magnetic") == 0)
  {
    nmea0183CreateMessage(msg183, src, "HDM,%s,M", heading);
  }
}

/*
=== MWV - Wind Speed and Angle ===

------------------------------------------------------------------------------
        1   2 3   4 5 6
        |   | |   | | |
 $--MWV,x.x,a,x.x,a,a*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Wind Angle, 0 to 360 degrees
2. Reference, R = Relative, T = True
3. Wind Speed
4. Wind Speed Units, K/M/N
5. Active (A) or invalid (V)
6. Checksum
*/

static void nmea0183WindData( StringBuffer * msg183, int src, const char * msg )
{
  char speed[10];
  char angle[10];
  char reference[10];
  double speedInMetersPerSecond;
  double speedInKMPerHour;

  if (!getJSONValue(msg, "Wind Speed", speed, sizeof(speed))
   || !getJSONValue(msg, "Wind Angle", angle, sizeof(angle))
   || !getJSONValue(msg, "Reference", reference, sizeof(reference)))
  {
    return;
  }

  speedInMetersPerSecond = strtod(speed, 0);
  speedInKMPerHour = speedInMetersPerSecond * 3.6;

  if (strcmp(reference, "True") == 0)
  {
    nmea0183CreateMessage(msg183, src, "MWV,%s,T,%.1f,K,A", angle, speedInKMPerHour);
  }
  else if (strcmp(reference, "Apparent") == 0)
  {
    nmea0183CreateMessage(msg183, src, "MWV,%s,R,%.1f,K,A", angle, speedInKMPerHour);
  }
}

/*
=== DBK - Depth Below Keel ===

------------------------------------------------------------------------------
        1   2 3   4 5   6 7
        |   | |   | |   | |
 $--DBK,x.x,f,x.x,M,x.x,F*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Depth, feet
2. f = feet
3. Depth, meters
4. M = meters
5. Depth, Fathoms
6. F = Fathoms
7. Checksum

=== DBS - Depth Below Surface ===

------------------------------------------------------------------------------
        1   2 3   4 5   6 7
        |   | |   | |   | |
 $--DBS,x.x,f,x.x,M,x.x,F*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Depth, feet
2. f = feet
3. Depth, meters
4. M = meters
5. Depth, Fathoms
6. F = Fathoms
7. Checksum

=== DBT - Depth below transducer ===

------------------------------------------------------------------------------
        1   2 3   4 5   6 7
        |   | |   | |   | |
 $--DBT,x.x,f,x.x,M,x.x,F*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Depth, feet
2. f = feet
3. Depth, meters
4. M = meters
5. Depth, Fathoms
6. F = Fathoms
7. Checksum

 * {"timestamp":"2012-12-01-12:53:19.929","prio":"3","src":"35","dst":"255","pgn":"128267","description":"Water Depth","fields":{"SID":"70","Depth":"0.63","Offset":"0.500"}}
 */
static void nmea0183WaterDepth( StringBuffer * msg183, int src, const char * msg )
{
  char depth[10];
  char offset[10];
  double dep = 0;
  double off = 0;

  if (!getJSONValue(msg, "Depth", depth, sizeof(depth)))
  {
    return;
  }
  if (getJSONValue(msg, "Offset", offset, sizeof(offset)))
  {
    off = strtod(offset, 0);
  }
  dep = strtod(depth, 0);

#define INCH_IN_METER (0.0254)
#define FEET_IN_METER (12.0 * INCH_IN_METER)
#define METER_TO_FEET(x) (x / FEET_IN_METER)
#define METER_TO_FATHOM(x) (METER_TO_FEET(x) / 6.0)

  if (off > 0.0)
  {
    nmea0183CreateMessage(msg183, src, "DBS,%04.1f,f,%s,M,%04.1f,F", METER_TO_FEET(dep), depth, METER_TO_FATHOM(dep));
  }
  if (off < 0.0)
  {
    nmea0183CreateMessage(msg183, src, "DBK,%04.1f,f,%s,M,%04.1f,F", METER_TO_FEET(dep), depth, METER_TO_FATHOM(dep));
  }
  if (off == 0.0)
  {
    nmea0183CreateMessage(msg183, src, "DBT,%04.1f,f,%s,M,%04.1f,F", METER_TO_FEET(dep), depth, METER_TO_FATHOM(dep));
  }
}

static bool matchFilter( int n, char * filter )
{
  bool negativeMatch = false;
  int  f;

  while (filter[0])
  {
    if (filter[0] == '!')
    {
      filter++;
      negativeMatch = true;
    }
    f = (int) strtol(filter, &filter, 10);
    logDebug("Src [%ld] == [%ld]?\n", n, f);

    if (n == f)
    {
      logDebug("Src [%ld] matches [%ld]\n", n, f);
      if (negativeMatch)
      {
        return false;
      }
      return true;
    }
    while (filter[0] && filter[0] != ',')
    {
      filter++;
    }
    if (filter[0] == ',')
    {
      filter++;
    }
  }
  return negativeMatch;
}

void convertJSONToNMEA0183( StringBuffer * msg183, const char * msg )
{
  char           str[20];
  int            prn;
  int            src;
  struct timeval tv;
  int            j;

  if (!getJSONValue(msg, "pgn", str, sizeof(str)))
  {
    return;
  }
  prn = atoi(str);

  switch (prn)
  {
  case PGN_VESSEL_HEADING:
    j = VESSEL_HEADING;
    break;
  case PGN_WIND_DATA:
    j = WIND_DATA;
    break;
  case PGN_WATER_DEPTH:
    j = WATER_DEPTH;
    break;
  default:
    return;
  }

  if (!getJSONValue(msg, "src", str, sizeof(str)))
  {
    return;
  }
  src = atoi(str);
  if (srcFilter && !matchFilter(src, srcFilter))
  {
    return;
  }

  logDebug("NMEA passed filter for prn %d src %d\n", src, prn);

  if (rateLimit)
  {
    int64_t now = epoch();

    if (rateLimitPassed[src][j] > (now - 1000L))
    {
      logDebug("Ratelimit for prn %d src %d not reached\n", src, prn);
      return;
    }
    rateLimitPassed[src][j] = now;
    logDebug("Ratelimit passed for prn %d src %d\n", src, prn);
  }

  switch (prn)
  {
  case PGN_VESSEL_HEADING:
    nmea0183VesselHeading(msg183, src, msg);
    break;
  case PGN_WIND_DATA:
    nmea0183WindData(msg183, src, msg);
    break;
  case PGN_WATER_DEPTH:
    nmea0183WaterDepth(msg183, src, msg);
    break;
  default:
    return;
  }

}

