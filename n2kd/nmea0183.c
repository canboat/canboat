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

#include "nmea0183.h"

#include <math.h>

#include "common.h"
#include "gps_ais.h"
#include "n2kd.h"

extern char *srcFilter;
extern bool  rateLimit;

/*
 * Which PGNs do we care and know about for now?
 *
 * NMEA 0183 information from the excellent reference at
 * www.catb.org/gpsd/NMEA.txt
 *
 * PGN 127250 "Vessel Heading" -> $xxHDG
 * PGN 130306 "Wind Data"      -> $xxMWV
 * PGN 128267 "Water Depth"    -> $xxDBK/DBS/DBT
 * PGN 128267 "Water Speed"    -> $xxVHW
 * PGN 127245 "Rudder"         -> $xxRSA
 * PGN 130311 "Environmental Parameters - water temperature" -> $xxMTW
 * PGN 128275 "Distance Log"   -> $xxVLW
 * PGN 126992 "System Time"    -> $xxZDA

 * Some others are in gps_ais.c file
 * PGN 129026 "Track made good and Ground speed"           -> $xxVTG
 * PGN 129539 "GPS DOP"                                    -> $xxGSA
 * PGN 129025 or 129029 "GPS Position"                     -> $xxGLL
 * PGN 129038 "Class A Position Report"                    -> !AIVDM
 * PGN 129039 "AIS Class B Position Report"                -> !AIVDM
 * PGN 129040 "AIS Class B Extended Position Report"       -> !AIVDM
 * PGN 129041 "AIS Aids to Navigation (AtoN) Report"       -> !AIVDM
 * PGN 129793 "AIS UTC and Date Report"                    -> !AIVDM
 * PGN 129794 "AIS Class A Static and Voyage Related Data" -> !AIVDM
 * PGN 129798 "AIS SAR Aircraft Position Report"           -> !AIVDM   PGN incomplete
 * PGN 129801 "AIS Addressed Safety Related Message"       -> !AIVDM
 * PGN 129802 "AIS Safety Related Broadcast Message"       -> !AIVDM   PGN incomplete
 * PGN 129809 "AIS Class B "CS" Static Data Report, Part A"-> !AIVDM
 * PGN 129810 "AIS Class B "CS" Static Data Report, Part B"-> !AIVDM
 */

#define PGN_SYSTEM_TIME (126992)
#define PGN_VESSEL_HEADING (127250)
#define PGN_WIND_DATA (130306)
#define PGN_WATER_DEPTH (128267)
#define PGN_WATER_SPEED (128259)
#define PGN_ENVIRONMENTAL (130311)
#define PGN_DISTANCE_LOG (128275)
#define PGN_RUDDER (127245)
#define PGN_SOG_COG (129026)
#define PGN_GPS_DOP (129539)
#define PGN_POSITION (129029)
#define PGN_AIS_A (129038)
#define PGN_AIS_B (129039)
#define PGN_AIS_4 (129793)
#define PGN_AIS_5 (129794)
#define PGN_AIS_9 (129798)
#define PGN_AIS_12 (129801)
#define PGN_AIS_14 (129802)
#define PGN_AIS_19 (129040)
#define PGN_AIS_21 (129041)
#define PGN_AIS_24A (129809)
#define PGN_AIS_24B (129810)

enum
{
  RATE_NO_LIMIT       = -1,
  RATE_VESSEL_HEADING = 0,
  RATE_WIND_DATA,
  RATE_WATER_DEPTH,
  RATE_WATER_SPEED,
  RATE_RUDDER,
  RATE_GPS_SPEED,
  RATE_GPS_DOP,
  RATE_GPS_POSITION,
  RATE_ENVIRONMENTAL,
  RATE_DISTANCE_LOG,
  RATE_SYSTEM_TIME,
  RATE_COUNT
};

static int64_t rateLimitPassed[256][RATE_COUNT];

extern void nmea0183CreateMessage(StringBuffer *msg183, int src, const char *format, ...)
{
  uint8_t chk;
  size_t  i;
  char    first, second;
  va_list ap;

  va_start(ap, format);

  // Convert the 8 bit value 'n' into a valid NMEA0183 style sender.
  // The first implementation sent out a 2 digit hexadecimal number,
  // but that throws some implementations of receivers off as they
  // cannot handle numeric senders. So now we produce a 2 character
  // code with the src value 0-255 translated into
  // A..Q A..P with A representing 0, B representing 1, etc.
  // P is skipped for the initial letter, as that represents 'proprietary'
  // and OpenCPN does not allow this.

  first  = 'A' + ((src >> 4) & 0xf);
  second = 'A' + ((src) &0xf);
  if (first >= 'P')
  {
    first++;
  }

  // Prepare for calculation of checksum
  i = msg183->len;

  if (src > 255)
    sbAppendFormat(msg183, "!%c%c", first, second);
  else
    sbAppendFormat(msg183, "$%c%c", first, second);
  sbAppendFormatV(msg183, format, ap);

  va_end(ap);

  chk = 0;
  for (i++; i < msg183->len; i++)
  {
    chk ^= (uint8_t) msg183->data[i];
  }
  sbAppendFormat(msg183, "*%02X\r\n", chk);
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
static void nmea0183VesselHeading(StringBuffer *msg183, int src, const char *msg)
{
  double heading;
  double deviation;
  double variation;
  int64_t reference;

  if (getJSONNumber(msg, "Heading", &heading, U_ANGLE) && getJSONLookupValue(msg, "Reference", &reference))
  {
    if (getJSONNumber(msg, "Deviation", &deviation, U_ANGLE) && getJSONNumber(msg, "Variation", &variation, U_ANGLE)
        && reference == 1)
    {
      /* Enough info for HDG message */

      nmea0183CreateMessage(msg183,
                            src,
                            "HDG,%.1f,%.1f,%c,%.1f,%c",
                            heading,
                            fabs(deviation),
                            ((deviation < 0.0) ? 'W' : 'E'),
                            fabs(variation),
                            ((variation < 0.0) ? 'W' : 'E'));
    }
    else if (reference == 0)
    {
      nmea0183CreateMessage(msg183, src, "HDT,%.1f,T", heading);
    }
    else if (reference == 1)
    {
      nmea0183CreateMessage(msg183, src, "HDM,%.1f,M", heading);
    }
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

static void nmea0183WindData(StringBuffer *msg183, int src, const char *msg)
{
  double  speed;
  double  angle;
  int64_t reference;

  if (getJSONNumber(msg, "Wind Speed", &speed, U_VELOCITY) && getJSONNumber(msg, "Wind Angle", &angle, U_ANGLE)
      && getJSONLookupValue(msg, "Reference", &reference))
  {
    if (reference == 3)
    {
      nmea0183CreateMessage(msg183, src, "MWV,%.1f,T,%.1f,K,A", angle, SPEED_M_S_TO_KMH(speed));
    }
    else if (reference == 2)
    {
      nmea0183CreateMessage(msg183, src, "MWV,%.1f,R,%.1f,K,A", angle, SPEED_M_S_TO_KMH(speed));
    }
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

 * {"timestamp":"2012-12-01-12:53:19.929","prio":"3","src":"35","dst":"255","pgn":"128267","description":"Water
Depth","fields":{"SID":"70","Depth":"0.63","Offset":"0.500"}}
 */
static void nmea0183WaterDepth(StringBuffer *msg183, int src, const char *msg)
{
  double depth;
  double offset;

  if (getJSONNumber(msg, "Depth", &depth, U_DISTANCE))
  {
    if (getJSONNumber(msg, "Offset", &offset, U_DISTANCE))
    {
      nmea0183CreateMessage(msg183, src, "DPT,%.1f,%.1f", depth, offset);
    }
    else
    {
      nmea0183CreateMessage(msg183, src, "DPT,%.1f,", depth);
    }
  }
}

/*

=== VHW - Water speed and heading ===
------------------------------------------------------------------------------
        1   2 3   4 5   6 7   8 9
        |   | |   | |   | |   | |
 $--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Degress True
2. T = True
3. Degrees Magnetic
4. M = Magnetic
5. Knots (speed of vessel relative to the water)
6. N = Knots
7. Kilometers (speed of vessel relative to the water)
8. K = Kilometers
9. Checksum
*/

static void nmea0183WaterSpeed(StringBuffer *msg183, int src, const char *msg)
{
  double speed;

  if (getJSONNumber(msg, "Speed Water Referenced", &speed, U_VELOCITY))
  {
    nmea0183CreateMessage(msg183, src, "VHW,,T,,M,%1f,N,%.1f,K", SPEED_M_S_TO_KNOTS(speed), SPEED_M_S_TO_KMH(speed));
  }
}

/*

=== MTW - Mean Temperature of Water ===
------------------------------------------------------------------------------
$--MTW,x.x,C*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:
1. Degrees
2. Unit of Measurement, Celcius
3. Checksum
*/

static void nmea0183WaterTemperature(StringBuffer *msg183, int src, const char *msg)
{
  double  temp;
  int64_t source;

  if (getJSONLookupValue(msg, "Temperature Source", &source) && (source == 0)
      && getJSONNumber(msg, "Temperature", &temp, U_TEMPERATURE))
  {
    nmea0183CreateMessage(msg183, src, "MTW,%.1f,C", TEMP_K_TO_C(temp));
  }
}

/*
VLW - Distance Traveled through Water
------------------------------------------------------------------------------
       1   2 3   4 5
       |   | |   | |
$--VLW,x.x,N,x.x,N*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:
1. Total cumulative distance
2. N = Nautical Miles
3. Distance since Reset
4. N = Nautical Miles
5. Checksum
*/

static void nmea0183DistanceTraveled(StringBuffer *msg183, int src, const char *msg)
{
  double log;
  double trip;

  if (getJSONNumber(msg, "Log", &log, U_DISTANCE) && getJSONNumber(msg, "Trip Log", &trip, U_DISTANCE))
  {
    nmea0183CreateMessage(msg183, src, "VLW,%.1f,N,%.1f,N", DIST_M_TO_NM(log), DIST_M_TO_NM(trip));
  }
}

/*
=== RSA - Rudder Sensor Angle ===
------------------------------------------------------------------------------
        1   2 3   4 5
        |   | |   | |
 $--RSA,x.x,A,x.x,A*hh<CR><LF>
------------------------------------------------------------------------------

Field Number:

1. Starboard (or single) rudder sensor, "-" means Turn To Port
2. Status, A means data is valid
3. Port rudder sensor
4. Status, A means data is valid
5. Checksum
*/

static void nmea0183Rudder(StringBuffer *msg183, int src, const char *msg)
{
  double pos;

  if (getJSONNumber(msg, "Position", &pos, U_ANGLE))
  {
    nmea0183CreateMessage(msg183, src, "RSA,%.1f,A,,F", -pos);
  }
}

/*
=== ZDA - Time & Date - UTC, day, month, year and local time zone ===

This is one of the sentences commonly emitted by GPS units.

        1         2  3  4    5  6  7
        |         |  |  |    |  |  |
 $--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx*hh<CR><LF>

Field Number:

    UTC time (hours, minutes, seconds, may have fractional subseconds)

    Day, 01 to 31

    Month, 01 to 12

    Year (4 digits)

    Local zone description, 00 to +- 13 hours

    Local zone minutes description, 00 to 59, apply same sign as local hours

    Checksum

Example: $GPZDA,160012.71,11,03,2004,-1,00*7D
*/
static void nmea0183SystemTime(StringBuffer *msg183, int src, const char *msg)
{
  char         dateString[20];
  char         timeString[20];
  char         second[10];
  unsigned int year, month, day, hour, minute;

  if (getJSONValue(msg, "Date", dateString, sizeof(dateString)) && getJSONValue(msg, "Time", timeString, sizeof(timeString))
      && sscanf(dateString, "%u.%u.%u", &year, &month, &day) == 3 && sscanf(timeString, "%u:%u:%9s", &hour, &minute, second) == 3)
  {
    nmea0183CreateMessage(msg183, src, "ZDA,%02u%02u%s,%u,%u,%04u,,", hour, minute, second, day, month, year);
  }
}

static bool matchFilter(int n, char *filter)
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

extern void convertJSONToNMEA0183(StringBuffer *msg183, const char *msg)
{
  int prn;
  int src;
  int rateType;

  if (!getJSONInteger(msg, "pgn", &prn))
  {
    return;
  }

  switch (prn)
  {
    case PGN_VESSEL_HEADING:
      rateType = RATE_VESSEL_HEADING;
      break;
    case PGN_WIND_DATA:
      rateType = RATE_WIND_DATA;
      break;
    case PGN_WATER_DEPTH:
      rateType = RATE_WATER_DEPTH;
      break;
    case PGN_WATER_SPEED:
      rateType = RATE_WATER_SPEED;
      break;
    case PGN_ENVIRONMENTAL:
      rateType = RATE_ENVIRONMENTAL;
      break;
    case PGN_DISTANCE_LOG:
      rateType = RATE_DISTANCE_LOG;
      break;
    case PGN_RUDDER:
      rateType = RATE_RUDDER;
      break;
    case PGN_SOG_COG:
      rateType = RATE_GPS_SPEED;
      break;
    case PGN_GPS_DOP:
      rateType = RATE_GPS_DOP;
      break;
    case PGN_SYSTEM_TIME:
      rateType = RATE_SYSTEM_TIME;
      break;
    case PGN_POSITION:
      rateType = RATE_GPS_POSITION;
      break;
    case PGN_AIS_A:
    case PGN_AIS_B:
    case PGN_AIS_4:
    case PGN_AIS_5:
    case PGN_AIS_9:
    case PGN_AIS_12:
    case PGN_AIS_14:
    case PGN_AIS_19:
    case PGN_AIS_21:
    case PGN_AIS_24A:
    case PGN_AIS_24B:
      rateType = RATE_NO_LIMIT;
      break;
    default:
      return;
  }

  if (!getJSONInteger(msg, "src", &src))
  {
    return;
  }
  if (srcFilter && !matchFilter(src, srcFilter))
  {
    return;
  }

  logDebug("NMEA passed filter for prn %d src %d\n", prn, src);

  if (rateLimit && rateType != RATE_NO_LIMIT)
  {
    int64_t now = epoch();

    if (rateLimitPassed[src][rateType] > (now - 1000L))
    {
      logDebug("Ratelimit for prn %d src %d not reached\n", prn, src);
      return;
    }
    rateLimitPassed[src][rateType] = now;
    logDebug("Ratelimit passed for prn %d src %d\n", prn, src);
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
    case PGN_WATER_SPEED:
      nmea0183WaterSpeed(msg183, src, msg);
      break;
    case PGN_ENVIRONMENTAL:
      nmea0183WaterTemperature(msg183, src, msg);
      break;
    case PGN_DISTANCE_LOG:
      nmea0183DistanceTraveled(msg183, src, msg);
      break;
    case PGN_RUDDER:
      nmea0183Rudder(msg183, src, msg);
      break;
    case PGN_SYSTEM_TIME:
      nmea0183SystemTime(msg183, src, msg);
      break;
    case PGN_SOG_COG:
      nmea0183VTG(msg183, src, msg);
      break;
    case PGN_GPS_DOP:
      nmea0183GSA(msg183, src, msg);
      break;
    case PGN_POSITION:
      nmea0183GLL(msg183, src, msg);
      break;
    case PGN_AIS_A:
    case PGN_AIS_B:
    case PGN_AIS_4:
    case PGN_AIS_5:
    case PGN_AIS_9:
    case PGN_AIS_12:
    case PGN_AIS_14:
    case PGN_AIS_19:
    case PGN_AIS_21:
    case PGN_AIS_24A:
    case PGN_AIS_24B:
      nmea0183AIVDM(msg183, src, msg);
      break;
    default:
      return;
  }
}

bool getJSONNumber(const char *message, const char *fieldName, double *value, Unit unit)
{
  char valueStr[16];

  // List of conversion values for each unit. When analyser produces SI convert it to 'human standard'.
  static const double UNIT_CONVERSION[U_MAX][2] = {{1, RadianToDegree}, {1, RadianToDegree}, {1, 1}, {1, 1}, {1, 1}, {1, 1}};
  static const double UNIT_OFFSET[U_MAX][2]     = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, -273.15}, {0, 0}};

  if (getJSONValue(message, fieldName, valueStr, sizeof valueStr))
  {
    *value = strtod(valueStr, 0) * UNIT_CONVERSION[unit][unitSI] + UNIT_OFFSET[unit][unitSI];
    return true;
  }
  *value = nan("");
  return false;
}

/**
 * Extract an integer value out of a JSON message,
 */
bool getJSONInteger(const char *message, const char *fieldName, int *value)
{
  char valueStr[16];

  return getJSONValue(message, fieldName, valueStr, sizeof valueStr) && sscanf(valueStr, "%d", value) == 1;
}
