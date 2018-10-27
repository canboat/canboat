#include "common.h"

#include <math.h>
#include <time.h>

#include "gps_ais.h"
#include "nmea0183.h"

#define MMSI_LENGTH sizeof("244060807")
#define LAT_LENGTH sizeof("-123.1234567890")
#define LON_LENGTH sizeof("-123.1234567890")
#define ANGLE_LENGTH sizeof("-123.999")
#define SPEED_LENGTH sizeof("-12345.999")
#define OTHER_LENGTH (20)

static void removeChar(char *str, char garbage)
{
  char *src, *dst;

  for (src = dst = str; *src; src++)
  {
    *dst = *src;
    if (*dst != garbage)
    {
      dst++;
    }
  }
  *dst = 0;
}

static double convert2kCoordinateToNMEA0183(const char *coordinateString, const char *hemispheres, char *hemisphere)
{
  double coordinate = strtod(coordinateString, 0);
  double degrees;
  double result;

  if (coordinate < 0)
  {
    *hemisphere = hemispheres[1];
    coordinate  = coordinate * -1.;
  }
  else
  {
    *hemisphere = hemispheres[0];
  }

  degrees = floor(coordinate);
  result  = degrees * 100 + (coordinate - degrees) * 60;
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

{"timestamp":"2015-12-10T22:19:45.330Z","prio":2,"src":2,"dst":255,"pgn":129026,"description":"COG & SOG, Rapid
Update","fields":{"SID":9,"COG Reference":"True","COG":0.0,"SOG":0.00}} $GPVTG,,T,,M,0.150,N,0.278,K,D*2F<0x0D><0x0A>
*/

void nmea0183VTG(StringBuffer *msg183, int src, const char *msg)
{
  char sogString[SPEED_LENGTH];
  char cogString[ANGLE_LENGTH];

  if (getJSONValue(msg, "SOG", sogString, sizeof(sogString)) && getJSONValue(msg, "COG", cogString, sizeof(cogString)))
  {
    double speed = strtod(sogString, 0);

    nmea0183CreateMessage(
        msg183, src, "VTG,%s,T,,M,%04.3f,N,%04.3f,K", cogString, SPEED_M_S_TO_KNOTS(speed), SPEED_M_S_TO_KMH(speed));
  }
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

{"timestamp":"2015-12-11T17:30:46.573Z","prio":6,"src":2,"dst":255,"pgn":129539,"description":"GNSS
DOPs","fields":{"SID":177,"Desired Mode":"3D","Actual Mode":"3D","HDOP":0.97,"VDOP":1.57,"TDOP":327.67}}
*/

void nmea0183GSA(StringBuffer *msg183, int src, const char *msg)
{
  char modeString[OTHER_LENGTH] = "";
  char pdopString[OTHER_LENGTH] = "";
  char hdopString[OTHER_LENGTH] = "";
  char vdopString[OTHER_LENGTH] = "";

  getJSONValue(msg, "Actual Mode", modeString, sizeof(modeString));
  getJSONValue(msg, "PDOP", pdopString, sizeof(pdopString));
  getJSONValue(msg, "HDOP", hdopString, sizeof(hdopString));
  getJSONValue(msg, "VDOP", vdopString, sizeof(vdopString));

  modeString[1] = 0; // Abbreviate string, or still empty if not in N2K PGN

  nmea0183CreateMessage(msg183, src, "GSA,M,%s,,,,,,,,,,,,,%s,%s,%s", modeString, pdopString, hdopString, vdopString);
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

{"timestamp":"2015-12-11T19:59:22.399Z","prio":2,"src":2,"dst":255,"pgn":129025,"description":"Position, Rapid
Update","fields":{"Latitude":36.1571104,"Longitude":-5.3561568}}
{"timestamp":"2015-12-11T20:01:19.010Z","prio":3,"src":2,"dst":255,"pgn":129029,"description":"GNSS Position
Data","fields":{"SID":10,"Date":"2015.12.11", "Time": "20:01:19","Latitude":36.1571168,"Longitude":-5.3561616,"GNSS
type":"GPS+SBAS/WAAS","Method":"GNSS fix","Integrity":"Safe","Number of SVs":12,"HDOP":0.86,"PDOP":1.68,"Geoidal
Separation":-0.01,"Reference Station ID":4087}} $GPGLL,3609.42711,N,00521.36949,W,200015.00,A,D*72
*/

void nmea0183GLL(StringBuffer *msg183, int src, const char *msg)
{
  char latString[LAT_LENGTH];
  char lonString[LON_LENGTH];

  if (getJSONValue(msg, "Latitude", latString, sizeof(latString)) && getJSONValue(msg, "Longitude", lonString, sizeof(lonString)))
  {
    char   timeString[OTHER_LENGTH] = "";
    char   latHemisphere;
    char   lonHemisphere;
    double latitude  = convert2kCoordinateToNMEA0183(latString, "NS", &latHemisphere);
    double longitude = convert2kCoordinateToNMEA0183(lonString, "EW", &lonHemisphere);

    if (getJSONValue(msg, "Time", timeString, sizeof(timeString)))
    {
      removeChar(timeString, ':');
    }

    nmea0183CreateMessage(msg183, src, "GLL,%.4f,%c,%.4f,%c,%s,A,D", latitude, latHemisphere, longitude, lonHemisphere, timeString);
  }
}

/*
{"timestamp":"2015-12-14T22:06:21.609Z","prio":4,"src":0,"dst":255,"pgn":129038,"description":"AIS Class A Position
Report","fields":{"Message ID":1,"Repeat Indicator":"Initial","User
ID":224593000,"Longitude":-5.4368332,"Latitude":36.1309824,"Position Accuracy":"Low","RAIM":"not in use","Time
Stamp":"21","COG":165.0,"SOG":0.00,"Communication State":"33576","AIS Transceiver information":"Channel A VDL
reception","Heading":51.0,"Rate of Turn":0.11,"Nav Status":"Under way using engine","Regional Application":1}}
{"timestamp":"2015-12-14T22:06:21.662Z","prio":4,"src":0,"dst":255,"pgn":129039,"description":"AIS Class B Position
Report","fields":{"Message ID":18,"Repeat Indicator":"Initial","User
ID":235015519,"Longitude":-5.3561452,"Latitude":36.1571296,"Position Accuracy":"Low","RAIM":"not in use","Time
Stamp":"5","COG":0.0,"SOG":0.00,"Communication State":"393222","AIS Transceiver information":"Own information not
broadcast","Heading":31.0,"Unit type":"CS","Integrated Display":"No","DSC":"Yes","Band":"entire marine band","Can handle Msg
22":"Yes","AIS mode":"Autonomous","AIS communication state":"ITDMA"}} !AIVDM,1,1,,B,177KQJ5000G?tO`K>RA1wUbN0TKH,0*5C - from
http://catb.org/gpsd/AIVDM.html

AIVDM - Automatic Information System (AIS) position reports from other vessels - from http://opencpn.org/ocpn/nmea_sentences
1. Time (UTC)
2. MMSI Number
3. Latitude
4. Longitude
5. Speed Knots
6. Heading
7. Course over Ground
8. Rate of turn
9. Navigation status

These are not rate limited, as each one is for a different vessel.
*/

void nmea0183AIVDM(StringBuffer *msg183, int src, const char *msg)
{
  struct tm *utc_time;
  time_t     current_time;

  char   mmsiString[MMSI_LENGTH];
  char   latString[LAT_LENGTH];
  char   lonString[LON_LENGTH];
  char   sogString[SPEED_LENGTH];
  char   headingString[ANGLE_LENGTH];
  char   cogString[ANGLE_LENGTH];
  char   rateOfTurnString[ANGLE_LENGTH];
  char   navigationStatusString[OTHER_LENGTH];
  double cog = 0;

  current_time = time(NULL);
  utc_time     = gmtime(&current_time);

  getJSONValue(msg, "User ID", mmsiString, sizeof(mmsiString));
  getJSONValue(msg, "Latitude", latString, sizeof(latString));
  getJSONValue(msg, "Longitude", lonString, sizeof(lonString));
  getJSONValue(msg, "SOG", sogString, sizeof(sogString));
  getJSONValue(msg, "COG", cogString, sizeof(cogString));
  getJSONValue(msg, "Heading", headingString, sizeof(headingString));
  getJSONValue(msg, "Rate of Turn", rateOfTurnString, sizeof(rateOfTurnString));
  getJSONValue(msg, "Nav Status", navigationStatusString, sizeof(navigationStatusString));

  cog = strtod(cogString, 0);

  // This does not work yet. It needs to be encoded to format like !AIVDM,1,1,,B,177KQJ5000G?tO`K>RA1wUbN0TKH,0*5C - from
  // http://catb.org/gpsd/AIVDM.html nmea0183CreateMessage(msg183, src, "PVDM,%d%d%d,%s,%s,%s,%s,%s,%.3f,%s,%s", utc_time->tm_hour,
  // utc_time->tm_min, utc_time->tm_sec, mmsiString, latString, lonString, sogString, headingString, cog, rateOfTurnString,
  // navigationStatusString);
}
