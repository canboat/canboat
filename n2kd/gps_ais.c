#include <math.h>

#include "gps_ais.h"
#include "nmea0183.h"

static void removeChar(char *str, char garbage) {
  char *src, *dst;
  for (src = dst = str; *src != '\0'; src++) {
      *dst = *src;
      if (*dst != garbage) dst++;
  }
  *dst = '\0';
}

static double convert2kCoordinateToNMEA0183(double coordinate) {
  double digrees;
  double result;
  digrees = floor(coordinate);
  result = digrees * 100 + (coordinate - digrees) * 60;
  return result;
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

void nmea0183VTG( StringBuffer * msg183, int src, const char * msg )
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

void nmea0183GSA( StringBuffer * msg183, int src, const char * msg )
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

// === GLL - Geographic Position - Latitude/Longitude ===

// This is one of the sentences commonly emitted by GPS units.

//         1       2 3        4 5         6 7   8
//         |       | |        | |         | |   |
//  $--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,a,m,*hh<CR><LF>

// Field Number:

// 1. Latitude
// 2. N or S (North or South)
// 3. Longitude
// 4. E or W (East or West)
// 5. Universal Time Coordinated (UTC)
// 6. Status A - Data Valid, V - Data Invalid
// 7. FAA mode indicator (NMEA 2.3 and later)
// 8. Checksum


// {"timestamp":"2015-12-11T19:59:22.399Z","prio":2,"src":2,"dst":255,"pgn":129025,"description":"Position, Rapid Update","fields":{"Latitude":36.1571104,"Longitude":-5.3561568}}
// {"timestamp":"2015-12-11T20:01:19.010Z","prio":3,"src":2,"dst":255,"pgn":129029,"description":"GNSS Position Data","fields":{"SID":10,"Date":"2015.12.11", "Time": "20:01:19","Latitude":36.1571168,"Longitude":-5.3561616,"GNSS type":"GPS+SBAS/WAAS","Method":"GNSS fix","Integrity":"Safe","Number of SVs":12,"HDOP":0.86,"PDOP":1.68,"Geoidal Separation":-0.01,"Reference Station ID":4087}}

// $GPGLL,3609.42711,N,00521.36949,W,200015.00,A,D*72

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

  latitude = strtod(lat_string, 0);
  longitude = strtod(lon_string, 0);

  if(latitude < 0) {
    lat_hemisphere = 'S';
    latitude = latitude * -1;
  }
  latitude = convert2kCoordinateToNMEA0183(latitude);

  if(longitude < 0) {
    lon_hemisphere = 'W';
    longitude = longitude * -1;
  }
  longitude = convert2kCoordinateToNMEA0183(longitude);

  if(getJSONValue(msg, "Time", time_string, sizeof(time_string))){
    removeChar(time_string, ':');
  }

  nmea0183CreateMessage(msg183, src, "GLL,%.4f,%c,%.4f,%c,%s,A,D", latitude, lat_hemisphere, longitude, lon_hemisphere, time_string);
}
