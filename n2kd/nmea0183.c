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
 * PGN 128267 "Water Speed"    -> $xxVHW
 * PGN 127245 "Rudder"         -> $xxRSA
 *
 * Typical output of these from analyzer:
 * {"timestamp":"2010-09-12-10:57:41.217","prio":"2","src":"36","dst":"255","pgn":"127250","description":"Vessel Heading","fields":{"SID":"116","Heading":"10.1","Deviation":"0.0","Variation":"0.0","Reference":"Magnetic"}}
 * {"timestamp":"2010-09-12-11:00:20.269","prio":"2","src":"13","dst":"255","pgn":"130306","description":"Wind Data","fields":{"Wind Speed":"5.00","Wind Angle":"308.8","Reference":"Apparent"}}
 * {"timestamp":"2012-12-01-12:53:19.929","prio":"3","src":"35","dst":"255","pgn":"128267","description":"Water Depth","fields":{"SID":"70","Depth":"0.63","Offset":"0.500"}}
 * {"timestamp":"2015-12-07-21:51:11.381","prio":"2","src":"4","dst":"255","pgn":"128259","description":"Speed","fields":{"Speed Water Referenced":0.30}}
 * {"timestamp":"2015-12-09-21:53:47.497","prio":"2","src":"1","dst":"255","pgn":"127245","description":"Rudder","fields":{"Angle Order":-0.0,"Position":6.8}}
 * {"timestamp":"2015-12-11T17:56:55.755Z","prio":6,"src":2,"dst":255,"pgn":129539,"description":"GNSS DOPs","fields":{"SID":239,"Desired Mode":"3D","Actual Mode":"3D","HDOP":1.21,"VDOP":1.83,"TDOP":327.67}}
 * {"timestamp":"2015-12-11T17:56:55.755Z","prio":6,"src":2,"dst":255,"pgn":129539,"description":"GNSS DOPs","fields":{"SID":239,"Desired Mode":"3D","Actual Mode":"3D","HDOP":1.21,"VDOP":1.83,"TDOP":327.67}}
 */

#define PGN_VESSEL_HEADING (127250)
#define PGN_WIND_DATA      (130306)
#define PGN_WATER_DEPTH    (128267)
#define PGN_WATER_SPEED    (128259)
#define PGN_RUDDER         (127245)
#define PGN_SOG_COG        (129026)
#define PGN_GPS_DOP        (129539)
#define PGN_RAPID_POSITION (129025)
#define PGN_POSITION       (129029)

#define VESSEL_HEADING (0)
#define WIND_DATA      (1)
#define WATER_DEPTH    (2)
#define WATER_SPEED    (3)
#define RUDDER         (4)
#define SOG_COG        (5)
#define GPS_DOP        (6)
#define GPS_POSITION   (7)
#define SENTENCE_COUNT (8)

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

  if (strcmp(reference, "True") >= 0)
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

  nmea0183CreateMessage(msg183, src, "DPT,%04.1f,%04.1f", dep, off);
  // if (off > 0.0)
  // {
  //   nmea0183CreateMessage(msg183, src, "DBS,%04.1f,f,%s,M,%04.1f,F", METER_TO_FEET(dep), depth, METER_TO_FATHOM(dep));
  // }
  // if (off < 0.0)
  // {
  //   nmea0183CreateMessage(msg183, src, "DBK,%04.1f,f,%s,M,%04.1f,F", METER_TO_FEET(dep), depth, METER_TO_FATHOM(dep));
  // }
  // if (off == 0.0)
  // {
  //   nmea0183CreateMessage(msg183, src, "DBT,%04.1f,f,%s,M,%04.1f,F", METER_TO_FEET(dep), depth, METER_TO_FATHOM(dep));
  // }
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

static void nmea0183WaterSpeed( StringBuffer * msg183, int src, const char * msg )
{
  char speed[10];
  double speedInMetersPerSecond;

  if (!getJSONValue(msg, "Speed Water Referenced", speed, sizeof(speed)))
  {
    return;
  }

  speedInMetersPerSecond = strtod(speed, 0);

#define MS_TO_KNOTS(meters_per_second) (meters_per_second * 1.94384)
#define MS_TO_MKH(meters_per_second) (meters_per_second * 3.6)

  nmea0183CreateMessage(msg183, src, "VHW,0.0,T,0.0,M,%04.1f,N,%04.1f,K", MS_TO_KNOTS(speedInMetersPerSecond), MS_TO_MKH(speedInMetersPerSecond));

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

static void nmea0183Rudder( StringBuffer * msg183, int src, const char * msg )
{
  char position[10];
  double pos = 0;
  double opposite_pos = 0;

  if (!getJSONValue(msg, "Position", position, sizeof(position)))
  {
    return;
  }

  pos = strtod(position, 0);
  opposite_pos = pos * -1;

  nmea0183CreateMessage(msg183, src, "RSA,%04.1f,A,0.0,F", opposite_pos);

}

/*
=== VTG - Track made good and Ground speed ===
This is one of the sentences commonly emitted by GPS units.

         1  2  3  4  5  6  7  8 9   10
         |  |  |  |  |  |  |  | |   |
 $--VTG,x.x,T,x.x,M,x.x,N,x.x,K,m,*hh<CR><LF>

Field Number:
1. Track Degrees
2. T = True
3. Track Degrees
4. M = Magnetic
5. Speed Knots
6. N = Knots
7. Speed Kilometers Per Hour
8. K = Kilometers Per Hour
9. FAA mode indicator (NMEA 2.3 and later)
10. Checksum

{"timestamp":"2015-12-10T22:19:45.330Z","prio":2,"src":2,"dst":255,"pgn":129026,"description":"COG & SOG, Rapid Update","fields":{"SID":9,"COG Reference":"True","COG":0.0,"SOG":0.00}}
$GPVTG,,T,,M,0.150,N,0.278,K,D*2F<0x0D><0x0A>
*/

static void nmea0183VTG( StringBuffer * msg183, int src, const char * msg )
{
  char sog[10];
  char cog[10];
  double speed = 0;
  double course = 0;

  if (!getJSONValue(msg, "SOG", sog, sizeof(sog)) || !getJSONValue(msg, "COG", cog, sizeof(cog)))
  {
    return;
  }

  speed = strtod(sog, 0);
  course = strtod(cog, 0);

#define KNOTS_TO_KMH(knots) (knots / 1.852)

  nmea0183CreateMessage(msg183, src, "VTG,%04.1f,T,,M,%04.3f,N,%04.3f,K", course, speed, KNOTS_TO_KMH(speed));

}

/*
=== GSA - GPS DOP and active satellites
This is one of the sentences commonly emitted by GPS units.

        1 2 3                        14 15  16  17  18
        | | |                         |  |   |   |   |
 $--GSA,a,a,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x.x,x.x,x.x*hh<CR><LF>
Field Number:
1. Selection mode: M=Manual, forced to operate in 2D or 3D, A=Automatic, 3D/2D
2. Mode (1 = no fix, 2 = 2D fix, 3 = 3D fix)
3. ID of 1st satellite used for fix
4. ID of 2nd satellite used for fix
5. ID of 3rd satellite used for fix
6. ID of 4th satellite used for fix
7. ID of 5th satellite used for fix
8. ID of 6th satellite used for fix
9. ID of 7th satellite used for fix
10. ID of 8th satellite used for fix
11. ID of 9th satellite used for fix
12. ID of 10th satellite used for fix
13. ID of 11th satellite used for fix
14. ID of 12th satellite used for fix
15. PDOP
16. HDOP
17. VDOP
18. Checksum

{"timestamp":"2015-12-11T17:30:46.573Z","prio":6,"src":2,"dst":255,"pgn":129539,"description":"GNSS DOPs","fields":{"SID":177,"Desired Mode":"3D","Actual Mode":"3D","HDOP":0.97,"VDOP":1.57,"TDOP":327.67}}
*/

static void nmea0183GSA( StringBuffer * msg183, int src, const char * msg )
{
  char mode_string[10];
  char pdop_string[10];
  char hdop_string[10];
  char vdop_string[10];
  double pdop = 0;
  double hdop = 0;
  double vdop = 0;

  getJSONValue(msg, "Actual Mode", mode_string, sizeof(mode_string));

  if(getJSONValue(msg, "PDOP", pdop_string, sizeof(pdop_string))) {
    pdop = strtod(pdop_string, 0);
  }
  if(getJSONValue(msg, "HDOP", hdop_string, sizeof(hdop_string))) {
    hdop = strtod(hdop_string, 0);
  }
  if(getJSONValue(msg, "VDOP", vdop_string, sizeof(vdop_string))) {
    vdop = strtod(vdop_string, 0);
  }

  nmea0183CreateMessage(msg183, src, "GSA,M,%c,,,,,,,,,,,,,%.3f,%.3f,%.3f", mode_string[0], pdop, hdop, vdop);
}

static void removeChar(char *str, char garbage) {
  char *src, *dst;
  for (src = dst = str; *src != '\0'; src++) {
      *dst = *src;
      if (*dst != garbage) dst++;
  }
  *dst = '\0';
}

/*
=== GLL - Geographic Position - Latitude/Longitude ===

This is one of the sentences commonly emitted by GPS units.

        1       2 3        4 5         6 7   8
        |       | |        | |         | |   |
 $--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,a,m,*hh<CR><LF>

Field Number:

1. Latitude
2. N or S (North or South)
3. Longitude
4. E or W (East or West)
5. Universal Time Coordinated (UTC)
6. Status A - Data Valid, V - Data Invalid
7. FAA mode indicator (NMEA 2.3 and later)
8. Checksum


{"timestamp":"2015-12-11T19:59:22.399Z","prio":2,"src":2,"dst":255,"pgn":129025,"description":"Position, Rapid Update","fields":{"Latitude":36.1571104,"Longitude":-5.3561568}}
{"timestamp":"2015-12-11T20:01:19.010Z","prio":3,"src":2,"dst":255,"pgn":129029,"description":"GNSS Position Data","fields":{"SID":10,"Date":"2015.12.11", "Time": "20:01:19","Latitude":36.1571168,"Longitude":-5.3561616,"GNSS type":"GPS+SBAS/WAAS","Method":"GNSS fix","Integrity":"Safe","Number of SVs":12,"HDOP":0.86,"PDOP":1.68,"Geoidal Separation":-0.01,"Reference Station ID":4087}}

$GPGLL,3609.42711,N,00521.36949,W,200015.00,A,D*72
*/

void nmea0183GLL( StringBuffer * msg183, int src, const char * msg )
{
  char lat_string[10];
  char lon_string[10];
  char time_string[10];
  char lat_hemisphere = 'N';
  char lon_hemisphere = 'E';
  double latitude = 0;
  double longitude = 0;
  int x;

  getJSONValue(msg, "Latitude", lat_string, sizeof(lat_string));
  getJSONValue(msg, "Longitude", lon_string, sizeof(lon_string));
  getJSONValue(msg, "Time", time_string, sizeof(time_string));

  latitude = strtod(lat_string, 0);
  longitude = strtod(lon_string, 0);

  if(latitude < 0) {
    lat_hemisphere = 'S';
    latitude = latitude * -1;
  }

  if(longitude < 0) {
    lon_hemisphere = 'W';
    longitude = longitude * -1;
  }

  removeChar(time_string, ':');

  nmea0183CreateMessage(msg183, src, "GLL,%03.4f,%c,%03.4f,%c,%s,A", latitude, lat_hemisphere, longitude, lon_hemisphere, time_string);
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
  case PGN_WATER_SPEED:
    j = WATER_SPEED;
    break;
  case PGN_RUDDER:
    j = RUDDER;
    break;
  case PGN_SOG_COG:
    j = SOG_COG;
    break;
  case PGN_GPS_DOP:
    j = GPS_DOP;
    break;
  case PGN_RAPID_POSITION:
  case PGN_POSITION:
    j = GPS_POSITION;
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
  case PGN_WATER_SPEED:
    nmea0183WaterSpeed(msg183, src, msg);
    break;
  case PGN_RUDDER:
    nmea0183Rudder(msg183, src, msg);
    break;
  case PGN_SOG_COG:
    nmea0183VTG(msg183, src, msg);
    break;
  case PGN_GPS_DOP:
    nmea0183GSA(msg183, src, msg);
    break;
  case PGN_RAPID_POSITION:
  case PGN_POSITION:
    nmea0183GLL(msg183, src, msg);
    break;
  default:
    return;
  }

}

