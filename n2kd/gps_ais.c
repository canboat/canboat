/*

(C) 2009-2021, Kees Verruijt, Harlingen, The Netherlands.

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

#include "gps_ais.h"

#include <math.h>
#include <time.h>

#include "common.h"
#include "nmea0183.h"
#include "n2kd.h"

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
  double sog;
  double cog;

  if (getJSONNumber(msg, "SOG", &sog, U_VELOCITY) && getJSONNumber(msg, "COG", &cog, U_ANGLE))
  {
    nmea0183CreateMessage(
        msg183, src, "VTG,%.1f,T,,M,%.2f,N,%.2f,K", cog, SPEED_M_S_TO_KNOTS(sog), SPEED_M_S_TO_KMH(sog));
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
=== AIS VDM/VDO ==
*/

typedef struct
{
  char bitVector[226]; // Corresponds to 300 6-bit characters. More than currently needed.
  int  pos;
} aisVector;

/*
Adds integer values to the packed 6 bit byte ais vector.
*/
static int addAisInt(long int value, int len, aisVector *payload)
{
  int           i, k;
  unsigned char nibble;

  // Break if overflow, does not treat signed positive value properly
  if (value >= (long int) (1 << len) || value < -(long int) (1 << (len - 1)))
    return 0;

  // AIS uses big endian. Thus most significant bits shall be stored first.
  while (len > 0)
  {
    i = payload->pos / 8;
    k = 8 - payload->pos % 8;
    payload->bitVector[i] &= ~(unsigned char) ((1 << k) - 1);
    if (len >= k)
    {
      nibble = ((1 << k) - 1) & (value >> (len - k));
      payload->pos += k;
    }
    else
    {
      nibble = ((1 << k) - 1) & (value << (k - len));
      payload->pos += len;
    }
    payload->bitVector[i] |= nibble;
    len -= k;
  }
  return 1;
}

/*
Adds ascii strings to the packed 6 bit byte ais vector.
*/
static int addAisString(char *string, int len, aisVector *payload)
{
  int           i, k;
  unsigned char nextchar;

  if (string == NULL)
    return 0;

  while (len >= 6)
  {
    // Encode current char to 6-bit ascii
    // 32 -> 63 => 32 -> 63, 64 -> 95 => 0 -> 31
    nextchar = *string;
    if (nextchar == '\0')
      nextchar = 0;
    else if (nextchar < 32 || nextchar > 95)
      nextchar = 32;
    else if (nextchar >= 64)
      nextchar -= 64;
    // Add 6-bit ascii char to payload
    i = payload->pos / 8;
    k = 8 - payload->pos % 8;
    payload->bitVector[i] &= ~(unsigned char) ((1 << k) - 1);
    if (k >= 6)
    {
      payload->bitVector[i] |= ((1 << k) - 1) & (nextchar << (k - 6));
    }
    else
    {
      payload->bitVector[i] |= ((1 << k) - 1) & (nextchar >> (6 - k));
      payload->bitVector[i + 1] = nextchar << (2 + k);
    }
    payload->pos += 6;
    len -= 6;
    if (*string != '\0')
      string++;
  }
  // A string can be padded with extra bits. These should reasonably be cleared
  payload->bitVector[(payload->pos / 8) + 1] = 0;
  payload->pos += len;
  return 1;
}

/*
We want strings to be handled as integers, but C can not return strings from functions,
only pointers to strings. There are two choices, arbitrary sized strings dynamically
allocated on the heap, or a statically allocated string buffer of limited size. Static
allocation has been chosen due to the more robust design despite the need to split
strings exceeding the buffer capacity into several substrings. In currently encoded
AIS sentences the maximum size of strings are 20 characters. The negative effect of
increased memory fotprint is seen as insignificant.
*/
char *aisString(const char *msg, const char *fieldName)
{
  static char jsonString[21];

  if (!getJSONValue(msg, fieldName, jsonString, sizeof(jsonString)))
    jsonString[0] = '\0';
  return jsonString;
}

/*
Handles text strings longer than 20 characters.
*/
static int addAisLongString(const char *msg, const char *fieldName, int maxSize, bool padd, aisVector *payload)
{
  char *jsonString;
  int   len;

  jsonString = (char *) calloc(maxSize + 1, sizeof(char));
  if (jsonString == NULL)
    return 0;

  if (!getJSONValue(msg, fieldName, jsonString, maxSize))
  {
    free(jsonString);
    return 0;
  }

  for (len = 0; len <= maxSize; len++)
  {
    if (*(jsonString + len) == '\0')
      break;
  }

  len *= 6;
  if (padd)
    if (len % 8)
      len += 8 - (len % 8); // Add padd bits

  addAisString(jsonString, len, payload);
  free(jsonString);
  return len;
}

/*
Extracts and encodes next 6 bit value from the packed ais vector.
*/
char nextPayloadChar(aisVector *bitVec, int *opos, unsigned char *padding)
{
  unsigned char i;
  int           start = *opos / 8;

  if (*opos >= bitVec->pos)
    return '\0';

  if (bitVec->pos - *opos < 6)
    *padding = 6 - (bitVec->pos - *opos);

  i = (bitVec->bitVector[start] << (*opos % 8));
  i = i >> 2;
  if ((8 - *opos % 8) + *padding < 6)
    i |= (unsigned char) bitVec->bitVector[start + 1] >> (10 - (*opos % 8));
  i &= 0xff << *padding;
  // Make shure *opos does not exceede bitVec->pos
  *opos += 6 - *padding;

  // Ascii encode the 6-bit value before return
  if (i < 40)
    return (char) (i + 48);
  else
    return (char) (i + 56);
}

/*
Splits the ais payload into sentences, ASCII encodes the payload, creates
and sends the relelvant NMEA 0183 sentences.
With a total of 80 maximum chars exkluding end of line per sentence, and
20 chars head + tail in the nmea 0183 carrier protocol, this leaves 60
char payload, corresponding to 360 bits ais data.
*/
static void aisToNmea0183(StringBuffer *msg183, int src, char *aisTalkerId, char channel, aisVector *bitVec)
{
  static char   sequenceId = '0';
  int           opos       = 0;
  unsigned char padding    = 0;
  int           fragments;
  int           fragCntr = 1;

  fragments = bitVec->pos / 360;
  if (bitVec->pos % 360)
    fragments++;
  // We maintain 10 sequential message ID for multi-sentence messages
  if (fragments > 1)
    sequenceId = (((sequenceId - 48) + 1) % 10) + 48;

  while (fragCntr <= fragments)
  {
    char payload[61];
    int  i = 0;

    while (i < 60 && opos <= bitVec->pos)
    {
      payload[i] = nextPayloadChar(bitVec, &opos, &padding);
      if (payload[i] == '\0')
        break;
      i++;
    }

    if (fragments > 1)
    {
      nmea0183CreateMessage(
          msg183, src, "%s,%d,%d,%c,%c,%s,%d", aisTalkerId, fragments, fragCntr, sequenceId, channel, payload, padding);
    }
    else
    {
      nmea0183CreateMessage(msg183, src, "%s,%d,%d,,%c,%s,%d", aisTalkerId, fragments, fragCntr, channel, payload, padding);
    }

    fragCntr++;
  }
}

int aisEnum(const char *, const char *);

/*
Ais nummerical values
*/
long int aisInteger(const char *msg, const char *fieldName)
{
  typedef struct
  {
    int         hash;
    const char *value;
    long int    min;
    long int    max;
    long int    defValue;
  } intParam;

  // Initialisation of type keys, done at most once
  static const int p0 = 'M' + 'e' + 's' + 's' + 'a' + 'g' + 'e' + ' ' + 'I' + 'D';
  static const int p1 = 'U' + 's' + 'e' + 'r' + ' ' + 'I' + 'D';
  static const int p2 = 'T' + 'i' + 'm' + 'e' + ' ' + 'S' + 't' + 'a' + 'm' + 'p';
  static const int p3
      = 'C' + 'o' + 'm' + 'm' + 'u' + 'n' + 'i' + 'c' + 'a' + 't' + 'i' + 'o' + 'n' + ' ' + 'S' + 't' + 'a' + 't' + 'e';
  static const int p4 = 'I' + 'M' + 'O' + ' ' + 'n' + 'u' + 'm' + 'b' + 'e' + 'r';
  static const int p5 = 'M' + 'o' + 't' + 'h' + 'e' + 'r' + 's' + 'h' + 'i' + 'p' + ' ' + 'U' + 's' + 'e' + 'r' + ' ' + 'I' + 'D';
  static const int p6 = 'S' + 'o' + 'u' + 'r' + 'c' + 'e' + ' ' + 'I' + 'D';
  static const int p7 = 'S' + 'e' + 'q' + 'u' + 'e' + 'n' + 'c' + 'e' + ' ' + 'N' + 'u' + 'm' + 'b' + 'e' + 'r';
  static const int p8 = 'D' + 'e' + 's' + 't' + 'i' + 'n' + 'a' + 't' + 'i' + 'o' + 'n' + ' ' + 'I' + 'D';
  static const int p9 = 'R' + 'e' + 't' + 'r' + 'a' + 'n' + 's' + 'm' + 'i' + 't' + ' ' + 'f' + 'l' + 'a' + 'g';

  char     jsonString[40];
  int      i, h = 0;
  long int value;

  intParam paramRange[] = {{p0, "Message ID", 0, 63, -1},
                           {p1, "User ID", 0, 999999999, 0},
                           {p2, "Time Stamp", 0, 59, 60},
                           {p3, "Communication State", 0, 524287, 393222},
                           {p4, "IMO number", 1000000, 9999999, 0},
                           {p5, "Mothership User ID", 0, 999999999, 0},
                           {p6, "Source ID", 0, 999999999, 0},
                           {p7, "Sequence Number", 0, 3, 0},
                           {p8, "Destination ID", 0, 999999999, 0},
                           {p9, "Retransmit flag", 0, 1, 0}};

  // Calculate index
  for (i = 0; fieldName[i] != '\0'; i++)
    h += fieldName[i];
  for (i = 0; i < sizeof(paramRange) / sizeof(intParam); i++)
  {
    if (paramRange[i].hash == h && !strcmp(fieldName, paramRange[i].value))
      break;
  }

  if (!getJSONValue(msg, fieldName, jsonString, sizeof(jsonString)))
    return paramRange[i].defValue;
  if (jsonString[0] == '\0')
    return paramRange[i].defValue;
  value = atol(jsonString);
  if (value >= paramRange[i].min && value <= paramRange[i].max)
    return value;
  if (paramRange[i].hash != p2)
    return paramRange[i].defValue;
  return aisEnum(msg, "Time Stamp"); /* Time Stamp enum values */
}

long int aisFloat(const char *msg, const char *fieldName)
{
#define DEGREES_PER_RADIAN (57.295779513)
#define SECONDS_PER_MINUTE (60.0)
#define KNOTS_IN_MS (1.943844492)
#define ROT_MULTIPLICATOR (4.733)
#define SOG_MULTIPLICATOR (10.0)
#define COG_MULTIPLICATOR (10.0)
#define NO_MULTIPLICATOR (1.0)
#define LON_MULTIPLICATOR (600000.0)
#define LAT_MULTIPLICATOR (600000.0)
#define SIZE_MULTIPLICATOR (10.0)
#define DRAUGHT_MULTIPLICATOR (10.0)
#define ALTITUDE_MULTIPLICATOR (1.0)

  typedef struct
  {
    const char *value;
    double      min;
    double      max;
    double      defValue;
    double      multiplier;
    Unit        unit;
  } floatParam;

  int      i;
  double   value;
  int      sign;
  long int result;

  floatParam paramRange[] = {{/* Min and max values of RoT are -127 and 127, but these values have
                                 special meaning, which is currently not present in canboats
                                 definition of the parameter. We therefore defaults if we
                                 get values with magnitude above 126.
                              */
                              "Rate of Turn",
                              -126,
                              126,
                              -128,
                              SECONDS_PER_MINUTE},
                             {"SOG", 0, 1022, 1023, KNOTS_IN_MS * SOG_MULTIPLICATOR, U_VELOCITY},
                             {"COG", 0, 3599, 3600, COG_MULTIPLICATOR, U_ANGLE},
                             {"Heading", 0, 359, 511, NO_MULTIPLICATOR, U_ANGLE},
                             {"Longitude", -108000000, 108000000, 0x6791AC0, LON_MULTIPLICATOR, U_GEO},
                             {"Latitude", -54000000, 54000000, 0x3412140, LAT_MULTIPLICATOR, U_GEO},
                             {"Length", 0, 10220, 0, SIZE_MULTIPLICATOR, U_DISTANCE},
                             {"Beam", 0, 1260, 0, SIZE_MULTIPLICATOR, U_DISTANCE},
                             {"Position reference from Starboard", 0, 630, 0, SIZE_MULTIPLICATOR, U_DISTANCE},
                             {"Position reference from Bow", 0, 5110, 0, SIZE_MULTIPLICATOR, U_DISTANCE},
                             {"Draft", 0, 255, 0, DRAUGHT_MULTIPLICATOR, U_DISTANCE},
                             {"True Heading", 0, 359, 511, NO_MULTIPLICATOR},
                             {"Length/Diameter", 0, 10220, 0, SIZE_MULTIPLICATOR, U_DISTANCE},
                             {"Beam/Diameter", 0, 1260, 0, SIZE_MULTIPLICATOR, U_DISTANCE},
                             {"Position Reference from Starboard Edge", 0, 630, 0, SIZE_MULTIPLICATOR, U_DISTANCE},
                             {"Position Reference from True North Facing Edge", 0, 5110, 0, SIZE_MULTIPLICATOR, U_DISTANCE},
                             {"Altitude", 0, 4094, 4095, ALTITUDE_MULTIPLICATOR, U_DISTANCE}};

  for (i = 0; i < ARRAY_SIZE(paramRange); i++)
  {
    if (!strcmp(fieldName, paramRange[i].value))
    {
      break;
    }
  }
  if (i == ARRAY_SIZE(paramRange))
  {
    logAbort("Unhandled AIS number field '%s'; please report this bug\n", fieldName);
  }

  if (!getJSONNumber(msg, fieldName, &value, paramRange[i].unit))
  {
    return paramRange[i].defValue;
  }
  value *= paramRange[i].multiplier;
  sign  = (value >= 0) - (value < 0);
  value *= sign;
  if (paramRange[i].defValue == -128)
  {
    result = (long int) (ROT_MULTIPLICATOR * sqrt(value) + 0.5);
  }
  else
  {
    result = (long int) (value + 0.5);
  }
  result *= sign;
  // Make sure values are valid
  if (result != paramRange[i].defValue && (result < paramRange[i].min || result > paramRange[i].max))
  {
    result = paramRange[i].defValue;
  }

  logDebug("aisFloat %s = %f = %d\n", fieldName, value, result);
  return result;
}

/*
Routine to enumerate ais enum fields based on the json string value.
*/
int aisEnum(const char *msg, const char *fieldName)
{
  typedef struct
  {
    int         hash;
    const char *value;
    int         first;
    int         elements;
    int         defValue;
  } aisKey;

  typedef struct
  {
    int         hash;
    int         num;
    const char *value;
  } aisValue;

  char jsonString[40];
  int  base, elements, defValue;
  int  i, h = 0;

  // Initialisation of type keys, done at most once
  static const int t0 = 'R' + 'e' + 'p' + 'e' + 'a' + 't' + ' ' + 'I' + 'n' + 'd' + 'i' + 'c' + 'a' + 't' + 'o' + 'r';
  static const int t1 = 'N' + 'a' + 'v' + ' ' + 'S' + 't' + 'a' + 't' + 'u' + 's';
  static const int t2 = 'P' + 'o' + 's' + 'i' + 't' + 'i' + 'o' + 'n' + ' ' + 'A' + 'c' + 'c' + 'u' + 'r' + 'a' + 'c' + 'y';
  static const int t3 = 'T' + 'i' + 'm' + 'e' + ' ' + 'S' + 't' + 'a' + 'm' + 'p';
  static const int t4 = 'R' + 'A' + 'I' + 'M';
  static const int t5 = 'A' + 'I' + 'S' + ' ' + 'T' + 'r' + 'a' + 'n' + 's' + 'c' + 'e' + 'i' + 'v' + 'e' + 'r' + ' ' + 'i' + 'n'
                        + 'f' + 'o' + 'r' + 'm' + 'a' + 't' + 'i' + 'o' + 'n';
  static const int t6  = 'D' + 'S' + 'C';
  static const int t7  = 'B' + 'a' + 'n' + 'd';
  static const int t8  = 'C' + 'a' + 'n' + ' ' + 'h' + 'a' + 'n' + 'd' + 'l' + 'e' + ' ' + 'M' + 's' + 'g' + ' ' + '2' + '2';
  static const int t9  = 'A' + 'I' + 'S' + ' ' + 'm' + 'o' + 'd' + 'e';
  static const int t10 = 'A' + 'I' + 'S' + ' ' + 'c' + 'o' + 'm' + 'm' + 'u' + 'n' + 'i' + 'c' + 'a' + 't' + 'i' + 'o' + 'n' + ' '
                         + 's' + 't' + 'a' + 't' + 'e';
  static const int t11 = 'U' + 'n' + 'i' + 't' + ' ' + 't' + 'y' + 'p' + 'e';
  static const int t12 = 'I' + 'n' + 't' + 'e' + 'g' + 'r' + 'a' + 't' + 'e' + 'd' + ' ' + 'D' + 'i' + 's' + 'p' + 'l' + 'a' + 'y';
  static const int t13
      = 'A' + 'I' + 'S' + ' ' + 'v' + 'e' + 'r' + 's' + 'i' + 'o' + 'n' + ' ' + 'i' + 'n' + 'd' + 'i' + 'c' + 'a' + 't' + 'o' + 'r';
  static const int t14 = 'T' + 'y' + 'p' + 'e' + ' ' + 'o' + 'f' + ' ' + 's' + 'h' + 'i' + 'p';
  static const int t15 = 'G' + 'N' + 'S' + 'S' + ' ' + 't' + 'y' + 'p' + 'e';
  static const int t16 = 'D' + 'T' + 'E';
  static const int t17 = 'A' + 'I' + 'S' + ' ' + 'R' + 'A' + 'I' + 'M' + ' ' + 'f' + 'l' + 'a' + 'g';
  static const int t18 = 'A' + 't' + 'o' + 'N' + ' ' + 'T' + 'y' + 'p' + 'e';
  static const int t19 = 'O' + 'f' + 'f' + ' ' + 'P' + 'o' + 's' + 'i' + 't' + 'i' + 'o' + 'n' + ' ' + 'I' + 'n' + 'd' + 'i' + 'c'
                         + 'a' + 't' + 'o' + 'r';
  static const int t20 = 'V' + 'i' + 'r' + 't' + 'u' + 'a' + 'l' + ' ' + 'A' + 't' + 'o' + 'N' + ' ' + 'F' + 'l' + 'a' + 'g';
  static const int t21 = 'A' + 's' + 's' + 'i' + 'g' + 'n' + 'e' + 'd' + ' ' + 'M' + 'o' + 'd' + 'e' + ' ' + 'F' + 'l' + 'a' + 'g';
  static const int t22 = 'S' + 'p' + 'e' + 'c' + 'i' + 'a' + 'l' + ' ' + 'M' + 'a' + 'n' + 'e' + 'u' + 'v' + 'e' + 'r' + ' ' + 'I'
                         + 'n' + 'd' + 'i' + 'c' + 'a' + 't' + 'o' + 'r';

  // Initialisation of value keys, done at most once
  static const int h0 = 'I' + 'n' + 'i' + 't' + 'i' + 'a' + 'l';
  static const int h1
      = 'F' + 'i' + 'r' + 's' + 't' + ' ' + 'r' + 'e' + 't' + 'r' + 'a' + 'n' + 's' + 'm' + 'i' + 's' + 's' + 'i' + 'o' + 'n';
  static const int h2
      = 'S' + 'e' + 'c' + 'o' + 'n' + 'd' + ' ' + 'r' + 'e' + 't' + 'r' + 'a' + 'n' + 's' + 'm' + 'i' + 's' + 's' + 'i' + 'o' + 'n';
  static const int h3
      = 'F' + 'i' + 'n' + 'a' + 'l' + ' ' + 'r' + 'e' + 't' + 'r' + 'a' + 'n' + 's' + 'm' + 'i' + 's' + 's' + 'i' + 'o' + 'n';

  static const int h4 = 'U' + 'n' + 'd' + 'e' + 'r' + ' ' + 'w' + 'a' + 'y' + ' ' + 'u' + 's' + 'i' + 'n' + 'g' + ' ' + 'e' + 'n'
                        + 'g' + 'i' + 'n' + 'e';
  static const int h5 = 'A' + 't' + ' ' + 'a' + 'n' + 'c' + 'h' + 'o' + 'r';
  static const int h6 = 'N' + 'o' + 't' + ' ' + 'u' + 'n' + 'd' + 'e' + 'r' + ' ' + 'c' + 'o' + 'm' + 'm' + 'a' + 'n' + 'd';
  static const int h7 = 'R' + 'e' + 's' + 't' + 'r' + 'i' + 'c' + 't' + 'e' + 'd' + ' ' + 'm' + 'a' + 'n' + 'o' + 'e' + 'u' + 'v'
                        + 'e' + 'r' + 'a' + 'b' + 'i' + 'l' + 'i' + 't' + 'y';
  static const int h8 = 'C' + 'o' + 'n' + 's' + 't' + 'r' + 'a' + 'i' + 'n' + 'e' + 'd' + ' ' + 'b' + 'y' + ' ' + 'h' + 'e' + 'r'
                        + ' ' + 'd' + 'r' + 'a' + 'u' + 'g' + 'h' + 't';
  static const int h9  = 'M' + 'o' + 'o' + 'r' + 'e' + 'd';
  static const int h10 = 'A' + 'g' + 'r' + 'o' + 'u' + 'n' + 'd';
  static const int h11 = 'E' + 'n' + 'g' + 'a' + 'g' + 'e' + 'd' + ' ' + 'i' + 'n' + ' ' + 'F' + 'i' + 's' + 'h' + 'i' + 'n' + 'g';
  static const int h12 = 'U' + 'n' + 'd' + 'e' + 'r' + ' ' + 'w' + 'a' + 'y' + ' ' + 's' + 'a' + 'i' + 'l' + 'i' + 'n' + 'g';
  static const int h13 = 'H' + 'a' + 'z' + 'a' + 'r' + 'd' + 'o' + 'u' + 's' + ' ' + 'm' + 'a' + 't' + 'e' + 'r' + 'i' + 'a' + 'l'
                         + ' ' + '-' + ' ' + 'H' + 'i' + 'g' + 'h' + ' ' + 'S' + 'p' + 'e' + 'e' + 'd';
  static const int h14 = 'H' + 'a' + 'z' + 'a' + 'r' + 'd' + 'o' + 'u' + 's' + ' ' + 'm' + 'a' + 't' + 'e' + 'r' + 'i' + 'a' + 'l'
                         + ' ' + '-' + ' ' + 'W' + 'i' + 'n' + 'g' + ' ' + 'i' + 'n' + ' ' + 'G' + 'r' + 'o' + 'u' + 'n' + 'd';
  static const int h18 = 'A' + 'I' + 'S' + '-' + 'S' + 'A' + 'R' + 'T';

  static const int h20 = 'L' + 'o' + 'w';
  static const int h21 = 'H' + 'i' + 'g' + 'h';
  static const int h22 = 'N' + 'o' + 't' + ' ' + 'a' + 'v' + 'a' + 'i' + 'l' + 'a' + 'b' + 'l' + 'e';
  static const int h23 = 'M' + 'a' + 'n' + 'u' + 'a' + 'l' + ' ' + 'i' + 'n' + 'p' + 'u' + 't' + ' ' + 'm' + 'o' + 'd' + 'e';
  static const int h24
      = 'D' + 'e' + 'a' + 'd' + ' ' + 'r' + 'e' + 'c' + 'k' + 'o' + 'n' + 'i' + 'n' + 'g' + ' ' + 'm' + 'o' + 'd' + 'e';
  static const int h25 = 'P' + 'o' + 's' + 'i' + 't' + 'i' + 'o' + 'n' + 'i' + 'n' + 'g' + ' ' + 's' + 'y' + 's' + 't' + 'e' + 'm'
                         + ' ' + 'i' + 's' + ' ' + 'i' + 'n' + 'o' + 'p' + 'e' + 'r' + 'a' + 't' + 'i' + 'v' + 'e';
  static const int h26 = 'n' + 'o' + 't' + ' ' + 'i' + 'n' + ' ' + 'u' + 's' + 'e';
  static const int h27 = 'i' + 'n' + ' ' + 'u' + 's' + 'e';
  static const int h28 = 'C' + 'h' + 'a' + 'n' + 'n' + 'e' + 'l' + ' ' + 'A' + ' ' + 'V' + 'D' + 'L' + ' ' + 'r' + 'e' + 'c' + 'e'
                         + 'p' + 't' + 'i' + 'o' + 'n';
  static const int h29 = 'C' + 'h' + 'a' + 'n' + 'n' + 'e' + 'l' + ' ' + 'B' + ' ' + 'V' + 'D' + 'L' + ' ' + 'r' + 'e' + 'c' + 'e'
                         + 'p' + 't' + 'i' + 'o' + 'n';
  static const int h30 = 'C' + 'h' + 'a' + 'n' + 'n' + 'e' + 'l' + ' ' + 'A' + ' ' + 'V' + 'D' + 'L' + ' ' + 't' + 'r' + 'a' + 'n'
                         + 's' + 'm' + 'i' + 's' + 's' + 'i' + 'o' + 'n';
  static const int h31 = 'C' + 'h' + 'a' + 'n' + 'n' + 'e' + 'l' + ' ' + 'B' + ' ' + 'V' + 'D' + 'L' + ' ' + 't' + 'r' + 'a' + 'n'
                         + 's' + 'm' + 'i' + 's' + 's' + 'i' + 'o' + 'n';
  static const int h32 = 'O' + 'w' + 'n' + ' ' + 'i' + 'n' + 'f' + 'o' + 'r' + 'm' + 'a' + 't' + 'i' + 'o' + 'n' + ' ' + 'n' + 'o'
                         + 't' + ' ' + 'b' + 'r' + 'o' + 'a' + 'd' + 'c' + 'a' + 's' + 't';
  static const int h33 = 'R' + 'e' + 's' + 'e' + 'r' + 'v' + 'e' + 'd';
  static const int h34 = 'N' + 'o';
  static const int h35 = 'Y' + 'e' + 's';
  static const int h36 = 't' + 'o' + ' ' + '5' + '2' + '5' + ' ' + 'k' + 'H' + 'z' + ' ' + 'o' + 'f' + ' ' + 'm' + 'a' + 'r' + 'i'
                         + 'n' + 'e' + ' ' + 'b' + 'a' + 'n' + 'd';
  static const int h37 = 'e' + 'n' + 't' + 'i' + 'r' + 'e' + ' ' + 'm' + 'a' + 'r' + 'i' + 'n' + 'e' + ' ' + 'b' + 'a' + 'n' + 'd';
  static const int h38 = 'N' + 'o';
  static const int h39 = 'Y' + 'e' + 's';
  static const int h40 = 'A' + 'u' + 't' + 'o' + 'n' + 'o' + 'm' + 'o' + 'u' + 's';
  static const int h41 = 'A' + 's' + 's' + 'i' + 'g' + 'n' + 'e' + 'd';
  static const int h42 = 'S' + 'O' + 'T' + 'D' + 'M' + 'A';
  static const int h43 = 'I' + 'T' + 'D' + 'M' + 'A';
  static const int h44 = 'S' + 'O' + 'T' + 'D' + 'M' + 'A';
  static const int h45 = 'C' + 'S';
  static const int h46 = 'I' + 'T' + 'U' + '-' + 'R' + ' ' + 'M' + '.' + '1' + '3' + '7' + '1' + '-' + '1';
  static const int h47 = 'I' + 'T' + 'U' + '-' + 'R' + ' ' + 'M' + '.' + '1' + '3' + '7' + '1' + '-' + '3';
  // Type of ship
  static const int h50 = 'u' + 'n' + 'a' + 'v' + 'a' + 'i' + 'l' + 'a' + 'b' + 'l' + 'e';
  static const int h52 = 'W' + 'i' + 'n' + 'g' + ' ' + 'I' + 'n' + ' ' + 'G' + 'r' + 'o' + 'u' + 'n' + 'd';
  static const int h53 = 'W' + 'i' + 'n' + 'g' + ' ' + 'I' + 'n' + ' ' + 'G' + 'r' + 'o' + 'u' + 'n' + 'd' + ' ' + '(' + 'n' + 'o'
                         + ' ' + 'o' + 't' + 'h' + 'e' + 'r' + ' ' + 'i' + 'n' + 'f' + 'o' + 'r' + 'm' + 'a' + 't' + 'i' + 'o' + 'n'
                         + ')';
  static const int h54 = 'F' + 'i' + 's' + 'h' + 'i' + 'n' + 'g';
  static const int h55 = 'T' + 'o' + 'w' + 'i' + 'n' + 'g';
  static const int h56 = 'T' + 'o' + 'w' + 'i' + 'n' + 'g' + ' ' + 'e' + 'x' + 'c' + 'e' + 'e' + 'd' + 's' + ' ' + '2' + '0' + '0'
                         + 'm' + ' ' + 'o' + 'r' + ' ' + 'w' + 'i' + 'd' + 'e' + 'r' + ' ' + 't' + 'h' + 'a' + 'n' + ' ' + '2' + '5'
                         + 'm';
  static const int h57 = 'E' + 'n' + 'g' + 'a' + 'g' + 'e' + 'd' + ' ' + 'i' + 'n' + ' ' + 'd' + 'r' + 'e' + 'd' + 'g' + 'i' + 'n'
                         + 'g' + ' ' + 'o' + 'r' + ' ' + 'u' + 'n' + 'd' + 'e' + 'r' + 'w' + 'a' + 't' + 'e' + 'r' + ' ' + 'o' + 'p'
                         + 'e' + 'r' + 'a' + 't' + 'i' + 'o' + 'n' + 's';
  static const int h58 = 'E' + 'n' + 'g' + 'a' + 'g' + 'e' + 'd' + ' ' + 'i' + 'n' + ' ' + 'd' + 'i' + 'v' + 'i' + 'n' + 'g' + ' '
                         + 'o' + 'p' + 'e' + 'r' + 'a' + 't' + 'i' + 'o' + 'n' + 's';
  static const int h59 = 'E' + 'n' + 'g' + 'a' + 'g' + 'e' + 'd' + ' ' + 'i' + 'n' + ' ' + 'm' + 'i' + 'l' + 'i' + 't' + 'a' + 'r'
                         + 'y' + ' ' + 'o' + 'p' + 'e' + 'r' + 'a' + 't' + 'i' + 'o' + 'n' + 's';
  static const int h60 = 'S' + 'a' + 'i' + 'l' + 'i' + 'n' + 'g';
  static const int h61 = 'P' + 'l' + 'e' + 'a' + 's' + 'u' + 'r' + 'e';
  static const int h62 = 'H' + 'i' + 'g' + 'h' + ' ' + 's' + 'p' + 'e' + 'e' + 'd' + ' ' + 'c' + 'r' + 'a' + 'f' + 't';
  static const int h63 = 'H' + 'i' + 'g' + 'h' + ' ' + 's' + 'p' + 'e' + 'e' + 'd' + ' ' + 'c' + 'r' + 'a' + 'f' + 't' + ' ' + 'c'
                         + 'a' + 'r' + 'r' + 'y' + 'i' + 'n' + 'g' + ' ' + 'd' + 'a' + 'n' + 'g' + 'e' + 'r' + 'o' + 'u' + 's' + ' '
                         + 'g' + 'o' + 'o' + 'd' + 's';
  static const int h64 = 'H' + 'i' + 'g' + 'h' + ' ' + 's' + 'p' + 'e' + 'e' + 'd' + ' ' + 'c' + 'r' + 'a' + 'f' + 't' + ' ' + 'h'
                         + 'a' + 'z' + 'a' + 'r' + 'd' + ' ' + 'c' + 'a' + 't' + ' ' + 'B';
  static const int h65 = 'H' + 'i' + 'g' + 'h' + ' ' + 's' + 'p' + 'e' + 'e' + 'd' + ' ' + 'c' + 'r' + 'a' + 'f' + 't' + ' ' + 'h'
                         + 'a' + 'z' + 'a' + 'r' + 'd' + ' ' + 'c' + 'a' + 't' + ' ' + 'C';
  static const int h66 = 'H' + 'i' + 'g' + 'h' + ' ' + 's' + 'p' + 'e' + 'e' + 'd' + ' ' + 'c' + 'r' + 'a' + 'f' + 't' + ' ' + 'h'
                         + 'a' + 'z' + 'a' + 'r' + 'd' + ' ' + 'c' + 'a' + 't' + ' ' + 'D';
  static const int h67 = 'H' + 'i' + 'g' + 'h' + ' ' + 's' + 'p' + 'e' + 'e' + 'd' + ' ' + 'c' + 'r' + 'a' + 'f' + 't' + ' ' + '('
                         + 'n' + 'o' + ' ' + 'a' + 'd' + 'd' + 'i' + 't' + 'i' + 'o' + 'n' + 'a' + 'l' + ' ' + 'i' + 'n' + 'f' + 'o'
                         + 'r' + 'm' + 'a' + 't' + 'i' + 'o' + 'n' + ')';
  static const int h68 = 'P' + 'i' + 'l' + 'o' + 't' + ' ' + 'v' + 'e' + 's' + 's' + 'e' + 'l';
  static const int h69 = 'S' + 'A' + 'R';
  static const int h70 = 'T' + 'u' + 'g';
  static const int h71 = 'P' + 'o' + 'r' + 't' + ' ' + 't' + 'e' + 'n' + 'd' + 'e' + 'r';
  static const int h72 = 'A' + 'n' + 't' + 'i' + '-' + 'p' + 'o' + 'l' + 'l' + 'u' + 't' + 'i' + 'o' + 'n';
  static const int h73 = 'L' + 'a' + 'w' + ' ' + 'e' + 'n' + 'f' + 'o' + 'r' + 'c' + 'e' + 'm' + 'e' + 'n' + 't';
  static const int h74 = 'S' + 'p' + 'a' + 'r' + 'e';
  static const int h75 = 'S' + 'p' + 'a' + 'r' + 'e' + ' ' + '#' + '2';
  static const int h76 = 'M' + 'e' + 'd' + 'i' + 'c' + 'a' + 'l';
  static const int h77
      = 'R' + 'R' + ' ' + 'R' + 'e' + 's' + 'o' + 'l' + 'u' + 't' + 'i' + 'o' + 'n' + ' ' + 'N' + 'o' + '.' + '1' + '8';
  static const int h78 = 'P' + 'a' + 's' + 's' + 'e' + 'n' + 'g' + 'e' + 'r' + ' ' + 's' + 'h' + 'i' + 'p';
  static const int h79 = 'P' + 'a' + 's' + 's' + 'e' + 'n' + 'g' + 'e' + 'r' + ' ' + 's' + 'h' + 'i' + 'p' + ' ' + '(' + 'n' + 'o'
                         + ' ' + 'a' + 'd' + 'd' + 'i' + 't' + 'i' + 'o' + 'n' + 'a' + 'l' + ' ' + 'i' + 'n' + 'f' + 'o' + 'r' + 'm'
                         + 'a' + 't' + 'i' + 'o' + 'n' + ')';
  static const int h80 = 'C' + 'a' + 'r' + 'g' + 'o' + ' ' + 's' + 'h' + 'i' + 'p';
  static const int h81 = 'C' + 'a' + 'r' + 'g' + 'o' + ' ' + 's' + 'h' + 'i' + 'p' + ' ' + 'c' + 'a' + 'r' + 'r' + 'y' + 'i' + 'n'
                         + 'g' + ' ' + 'd' + 'a' + 'n' + 'g' + 'e' + 'r' + 'o' + 'u' + 's' + ' ' + 'g' + 'o' + 'o' + 'd' + 's';
  static const int h82 = 'C' + 'a' + 'r' + 'g' + 'o' + ' ' + 's' + 'h' + 'i' + 'p' + ' ' + 'h' + 'a' + 'z' + 'a' + 'r' + 'd' + ' '
                         + 'c' + 'a' + 't' + ' ' + 'B';
  static const int h83 = 'C' + 'a' + 'r' + 'g' + 'o' + ' ' + 's' + 'h' + 'i' + 'p' + ' ' + 'h' + 'a' + 'z' + 'a' + 'r' + 'd' + ' '
                         + 'c' + 'a' + 't' + ' ' + 'C';
  static const int h84 = 'C' + 'a' + 'r' + 'g' + 'o' + ' ' + 's' + 'h' + 'i' + 'p' + ' ' + 'h' + 'a' + 'z' + 'a' + 'r' + 'd' + ' '
                         + 'c' + 'a' + 't' + ' ' + 'D';
  static const int h85 = 'C' + 'a' + 'r' + 'g' + 'o' + ' ' + 's' + 'h' + 'i' + 'p' + ' ' + '(' + 'n' + 'o' + ' ' + 'a' + 'd' + 'd'
                         + 'i' + 't' + 'i' + 'o' + 'n' + 'a' + 'l' + ' ' + 'i' + 'n' + 'f' + 'o' + 'r' + 'm' + 'a' + 't' + 'i' + 'o'
                         + 'n' + ')';
  static const int h86 = 'T' + 'a' + 'n' + 'k' + 'e' + 'r';
  static const int h87 = 'T' + 'a' + 'n' + 'k' + 'e' + 'r' + ' ' + 'c' + 'a' + 'r' + 'r' + 'y' + 'i' + 'n' + 'g' + ' ' + 'd' + 'a'
                         + 'n' + 'g' + 'e' + 'r' + 'o' + 'u' + 's' + ' ' + 'g' + 'o' + 'o' + 'd' + 's';
  static const int h88
      = 'T' + 'a' + 'n' + 'k' + 'e' + 'r' + ' ' + 'h' + 'a' + 'z' + 'a' + 'r' + 'd' + ' ' + 'c' + 'a' + 't' + ' ' + 'B';
  static const int h89
      = 'T' + 'a' + 'n' + 'k' + 'e' + 'r' + ' ' + 'h' + 'a' + 'z' + 'a' + 'r' + 'd' + ' ' + 'c' + 'a' + 't' + ' ' + 'C';
  static const int h90
      = 'T' + 'a' + 'n' + 'k' + 'e' + 'r' + ' ' + 'h' + 'a' + 'z' + 'a' + 'r' + 'd' + ' ' + 'c' + 'a' + 't' + ' ' + 'D';
  static const int h91 = 'T' + 'a' + 'n' + 'k' + 'e' + 'r' + ' ' + '(' + 'n' + 'o' + ' ' + 'a' + 'd' + 'd' + 'i' + 't' + 'i' + 'o'
                         + 'n' + 'a' + 'l' + ' ' + 'i' + 'n' + 'f' + 'o' + 'r' + 'm' + 'a' + 't' + 'i' + 'o' + 'n' + ')';
  static const int h92 = 'O' + 't' + 'h' + 'e' + 'r';
  static const int h93 = 'O' + 't' + 'h' + 'e' + 'r' + ' ' + 'c' + 'a' + 'r' + 'r' + 'y' + 'i' + 'n' + 'g' + ' ' + 'd' + 'a' + 'n'
                         + 'g' + 'e' + 'r' + 'o' + 'u' + 's' + ' ' + 'g' + 'o' + 'o' + 'd' + 's';
  static const int h94 = 'O' + 't' + 'h' + 'e' + 'r' + ' ' + 'h' + 'a' + 'z' + 'a' + 'r' + 'd' + ' ' + 'c' + 'a' + 't' + ' ' + 'B';
  static const int h95 = 'O' + 't' + 'h' + 'e' + 'r' + ' ' + 'h' + 'a' + 'z' + 'a' + 'r' + 'd' + ' ' + 'c' + 'a' + 't' + ' ' + 'C';
  static const int h96 = 'O' + 't' + 'h' + 'e' + 'r' + ' ' + 'h' + 'a' + 'z' + 'a' + 'r' + 'd' + ' ' + 'c' + 'a' + 't' + ' ' + 'D';
  static const int h97 = 'O' + 't' + 'h' + 'e' + 'r' + ' ' + '(' + 'n' + 'o' + ' ' + 'a' + 'd' + 'd' + 'i' + 't' + 'i' + 'o' + 'n'
                         + 'a' + 'l' + ' ' + 'i' + 'n' + 'f' + 'o' + 'r' + 'm' + 'a' + 't' + 'i' + 'o' + 'n' + ')';
  // EPFD fix types
  static const int h98  = 'u' + 'n' + 'd' + 'e' + 'f' + 'i' + 'n' + 'e' + 'd';
  static const int h99  = 'G' + 'P' + 'S';
  static const int h100 = 'G' + 'L' + 'O' + 'N' + 'A' + 'S' + 'S';
  static const int h101 = 'G' + 'P' + 'S' + '+' + 'G' + 'L' + 'O' + 'N' + 'A' + 'S' + 'S';
  static const int h102 = 'L' + 'o' + 'r' + 'a' + 'n' + '-' + 'C';
  static const int h103 = 'C' + 'h' + 'a' + 'y' + 'k' + 'a';
  static const int h104 = 'i' + 'n' + 't' + 'e' + 'g' + 'r' + 'a' + 't' + 'e' + 'd';
  static const int h105 = 's' + 'u' + 'r' + 'v' + 'e' + 'y' + 'e' + 'd';
  static const int h106 = 'G' + 'a' + 'l' + 'i' + 'l' + 'e' + 'o';
  static const int h107 = 'i' + 'n' + 't' + 'e' + 'r' + 'n' + 'a' + 'l' + ' ' + 'G' + 'N' + 'S' + 'S'; // Not supported by canboat
  // DTE
  static const int h108 = 'a' + 'v' + 'a' + 'i' + 'l' + 'a' + 'b' + 'l' + 'e';
  static const int h109 = 'n' + 'o' + 't' + ' ' + 'a' + 'v' + 'a' + 'i' + 'l' + 'a' + 'b' + 'l' + 'e';
  static const int h110 = 'A' + 'v' + 'a' + 'i' + 'l' + 'a' + 'b' + 'l' + 'e';
  static const int h111 = 'N' + 'o' + 't' + ' ' + 'a' + 'v' + 'a' + 'i' + 'l' + 'a' + 'b' + 'l' + 'e';
  // AtoN Type
  static const int h112 = 'D' + 'e' + 'f' + 'a' + 'u' + 'l' + 't' + ':' + ' ' + 'T' + 'y' + 'p' + 'e' + ' ' + 'o' + 'f' + ' ' + 'A'
                          + 't' + 'o' + 'N' + ' ' + 'n' + 'o' + 't' + ' ' + 's' + 'p' + 'e' + 'c' + 'i' + 'f' + 'i' + 'e' + 'd';
  static const int h113 = 'R' + 'e' + 'f' + 'e' + 'r' + 'e' + 'c' + 'e' + ' ' + 'p' + 'o' + 'i' + 'n' + 't';
  static const int h114 = 'R' + 'A' + 'C' + 'O' + 'N';
  static const int h115 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 's' + 't' + 'r' + 'u' + 'c' + 't' + 'u' + 'r' + 'e' + ' ' + 'o' + 'f'
                          + 'f' + '-' + 's' + 'h' + 'o' + 'r' + 'e';
  static const int h116 = 'R' + 'e' + 's' + 'e' + 'r' + 'v' + 'e' + 'd' + ' ' + 'f' + 'o' + 'r' + ' ' + 'f' + 'u' + 't' + 'u' + 'r'
                          + 'e' + ' ' + 'u' + 's' + 'e';
  static const int h117 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'l' + 'i' + 'g' + 'h' + 't' + ':' + ' ' + 'w' + 'i' + 't' + 'h' + 'o'
                          + 'u' + 't' + ' ' + 's' + 'e' + 'c' + 't' + 'o' + 'r' + 's';
  static const int h118 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'l' + 'i' + 'g' + 'h' + 't' + ':' + ' ' + 'w' + 'i' + 't' + 'h' + ' '
                          + 's' + 'e' + 'c' + 't' + 'o' + 'r' + 's';
  static const int h119 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'l' + 'e' + 'a' + 'd' + 'i' + 'n' + 'g' + ' ' + 'l' + 'i' + 'g' + 'h'
                          + 't' + ' ' + 'f' + 'r' + 'o' + 'n' + 't';
  static const int h120 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'l' + 'e' + 'a' + 'd' + 'i' + 'n' + 'g' + ' ' + 'l' + 'i' + 'g' + 'h'
                          + 't' + ' ' + 'r' + 'e' + 'a' + 'r';
  static const int h121 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 'c' + 'a' + 'r' + 'd'
                          + 'i' + 'n' + 'a' + 'l' + ' ' + 'N';
  static const int h122 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 'c' + 'a' + 'r' + 'd'
                          + 'i' + 'n' + 'a' + 'l' + ' ' + 'E';
  static const int h123 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 'c' + 'a' + 'r' + 'd'
                          + 'i' + 'n' + 'a' + 'l' + ' ' + 'S';
  static const int h124 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 'c' + 'a' + 'r' + 'd'
                          + 'i' + 'n' + 'a' + 'l' + ' ' + 'W';
  static const int h125 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 'p' + 'o' + 'r' + 't'
                          + ' ' + 'h' + 'a' + 'n' + 'd';
  static const int h126 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 's' + 't' + 'a' + 'r'
                          + 'b' + 'o' + 'a' + 'r' + 'd' + ' ' + 'h' + 'a' + 'n' + 'd';
  static const int h127 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 'p' + 'r' + 'e' + 'f'
                          + 'e' + 'r' + 'r' + 'e' + 'd' + ' ' + 'c' + 'h' + 'a' + 'n' + 'n' + 'e' + 'l' + ' ' + 'p' + 'o' + 'r'
                          + 't' + ' ' + 'h' + 'a' + 'n' + 'd';
  static const int h128 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 'p' + 'e' + 'f' + 'e'
                          + 'r' + 'r' + 'e' + 'd' + ' ' + 'c' + 'h' + 'a' + 'n' + 'n' + 'e' + 'l' + ' ' + 's' + 't' + 'a' + 'r'
                          + 'b' + 'o' + 'a' + 'r' + 'd' + ' ' + 'h' + 'a' + 'n' + 'd';
  static const int h129 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 'i' + 's' + 'o' + 'l'
                          + 'a' + 't' + 'e' + 'd' + ' ' + 'd' + 'a' + 'n' + 'g' + 'e' + 'r';
  static const int h130 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 's' + 'a' + 'f' + 'e'
                          + ' ' + 'w' + 'a' + 't' + 'e' + 'r';
  static const int h131 = 'F' + 'i' + 'x' + 'e' + 'd' + ' ' + 'b' + 'e' + 'a' + 'c' + 'o' + 'n' + ':' + ' ' + 's' + 'p' + 'e' + 'c'
                          + 'i' + 'a' + 'l' + ' ' + 'm' + 'a' + 'r' + 'k'; // Missing in canboat!
  static const int h132 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 'c' + 'a' + 'r'
                          + 'd' + 'i' + 'n' + 'a' + 'l' + ' ' + 'N';
  static const int h133 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 'c' + 'a' + 'r'
                          + 'd' + 'i' + 'n' + 'a' + 'l' + ' ' + 'E';
  static const int h134 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 'c' + 'a' + 'r'
                          + 'd' + 'i' + 'n' + 'a' + 'l' + ' ' + 'S';
  static const int h135 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 'c' + 'a' + 'r'
                          + 'd' + 'i' + 'n' + 'a' + 'l' + ' ' + 'W';
  static const int h136 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 'p' + 'o' + 'r'
                          + 't' + ' ' + 'h' + 'a' + 'n' + 'd' + ' ' + 'm' + 'a' + 'r' + 'k';
  static const int h137 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 's' + 't' + 'a'
                          + 'r' + 'b' + 'o' + 'a' + 'r' + 'd' + ' ' + 'h' + 'a' + 'n' + 'd' + ' ' + 'm' + 'a' + 'r' + 'k';
  static const int h138 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 'p' + 'r' + 'e'
                          + 'f' + 'e' + 'r' + 'r' + 'e' + 'd' + ' ' + 'c' + 'h' + 'a' + 'n' + 'n' + 'e' + 'l' + ' ' + 'p' + 'o'
                          + 'r' + 't' + ' ' + 'h' + 'a' + 'n' + 'd';
  static const int h139 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 'p' + 'r' + 'e'
                          + 'f' + 'e' + 'r' + 'r' + 'e' + 'd' + ' ' + 'c' + 'h' + 'a' + 'n' + 'n' + 'e' + 'l' + ' ' + 's' + 't'
                          + 'a' + 'r' + 'b' + 'o' + 'a' + 'r' + 'd' + ' ' + 'h' + 'a' + 'n' + 'd';
  static const int h140 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 'i' + 's' + 'o'
                          + 'l' + 'a' + 't' + 'e' + 'd' + ' ' + 'd' + 'a' + 'n' + 'g' + 'e' + 'r';
  static const int h141 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 's' + 'a' + 'f'
                          + 'e' + ' ' + 'w' + 'a' + 't' + 'e' + 'r';
  static const int h142 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 's' + 'p' + 'e'
                          + 'c' + 'i' + 'a' + 'l' + ' ' + 'm' + 'a' + 'r' + 'k';
  static const int h143 = 'F' + 'l' + 'o' + 'a' + 't' + 'i' + 'n' + 'g' + ' ' + 'A' + 't' + 'o' + 'N' + ':' + ' ' + 'l' + 'i' + 'g'
                          + 'h' + 't' + ' ' + 'v' + 'e' + 's' + 's' + 'e' + 'l' + '/' + 'L' + 'A' + 'N' + 'B' + 'Y' + '/' + 'r'
                          + 'i' + 'g' + 's';
  // Assigned Mode Flag
  static const int h144 = 'A' + 'u' + 't' + 'o' + 'n' + 'o' + 'm' + 'o' + 'u' + 's' + ' ' + 'a' + 'n' + 'd' + ' ' + 'c' + 'o' + 'n'
                          + 't' + 'i' + 'n' + 'u' + 'o' + 'u' + 's';
  static const int h145 = 'A' + 's' + 's' + 'i' + 'g' + 'n' + 'e' + 'd' + ' ' + 'm' + 'o' + 'd' + 'e';
  // Special Maneuver Indicator
  static const int h146 = 'N' + 'o' + 't' + ' ' + 'a' + 'v' + 'a' + 'i' + 'l' + 'a' + 'b' + 'l' + 'e';
  static const int h147 = 'N' + 'o' + 't' + ' ' + 'e' + 'n' + 'g' + 'a' + 'g' + 'e' + 'd' + ' ' + 'i' + 'n' + ' ' + 's' + 'p' + 'e'
                          + 'c' + 'i' + 'a' + 'l' + ' ' + 'm' + 'a' + 'n' + 'e' + 'u' + 'v' + 'e' + 'r';
  static const int h148 = 'E' + 'n' + 'g' + 'a' + 'g' + 'e' + 'd' + ' ' + 'i' + 'n' + ' ' + 's' + 'p' + 'e' + 'c' + 'i' + 'a' + 'l'
                          + ' ' + 'm' + 'a' + 'n' + 'e' + 'u' + 'v' + 'e' + 'r';
  static const int h149 = 'R' + 'e' + 's' + 'e' + 'r' + 'v' + 'e' + 'r' + 'd';

  aisKey eType[] = {{t0, "Repeat Indicator", 0, 4, 0},
                    {t1, "Nav Status", 4, 16, 15},
                    {t2, "Position Accuracy", 20, 2, 0},
                    {t3, "Time Stamp", 22, 4, 60},
                    {t4, "RAIM", 26, 2, 0},
                    {t5, "AIS Transceiver information", 28, 6, 0},
                    {t6, "DSC", 34, 2, 0},
                    {t7, "Band", 36, 2, 0},
                    {t8, "Can handle Msg 22", 38, 2, 0},
                    {t9, "AIS mode", 40, 2, 0},
                    {t10, "AIS communication state", 42, 2, 0},
                    {t11, "Unit type", 44, 2, 1},
                    {t12, "Integrated Display", 34, 2, 0},
                    {t13, "AIS version indicator", 46, 4, 0},
                    {t14, "Type of ship", 50, 48, 0},
                    {t15, "GNSS type", 98, 10, 0},
                    {t16, "DTE", 108, 4, 1},
                    {t17, "AIS RAIM flag", 26, 2, 0},
                    {t18, "AtoN Type", 112, 32, 0},
                    {t19, "Off Position Indicator", 34, 2, 1}, // No documented default value
                    {t20, "Virtual AtoN Flag", 34, 2, 0},
                    {t21, "Assigned Mode Flag", 144, 2, 0},
                    {t22, "Special Maneuver Indicator", 146, 4, 0}};

  aisValue eValue[] = {// Repeat Indicator
                       {h0, 0, "Initial"},
                       {h1, 1, "First retransmission"},
                       {h2, 2, "Second retransmission"},
                       {h3, 3, "Final retransmission"},

                       // Nav Status
                       {h4, 0, "Under way using engine"},
                       {h5, 1, "At anchor"},
                       {h6, 2, "Not under command"},
                       {h7, 3, "Restricted manoeuverability"},
                       {h8, 4, "Constrained by her draught"},
                       {h9, 5, "Moored"},
                       {h10, 6, "Aground"},
                       {h11, 7, "Engaged in Fishing"},
                       {h12, 8, "Under way sailing"},
                       {h13, 9, "Hazardous material - High Speed"},
                       {h14, 10, "Hazardous material - Wing in Ground"},
                       {0, 11, ""},
                       {0, 12, ""},
                       {0, 13, ""},
                       {h18, 14, "AIS-SART"},
                       {0, 15, ""},

                       // Position Accuracy
                       {h20, 0, "Low"},
                       {h21, 1, "High"},

                       // Time Stamp
                       {h22, 60, "Not available"},
                       {h23, 61, "Manual input mode"},
                       {h24, 62, "Dead reckoning mode"},
                       {h25, 63, "Positioning system is inoperative"},

                       // RAIM
                       {h26, 0, "not in use"},
                       {h27, 1, "in use"},

                       // AIS Transceiver information
                       {h28, 0, "Channel A VDL reception"},
                       {h29, 1, "Channel B VDL reception"},
                       {h30, 2, "Channel A VDL transmission"},
                       {h31, 3, "Channel B VDL transmission"},
                       {h32, 4, "Own information not broadcast"},
                       {h33, 5, "Reserved"},

                       // DSC, Integrated Display, Off-position indicator, Virtual AtoN flag
                       {h34, 0, "No"},
                       {h35, 1, "Yes"},

                       // Band
                       {h36, 0, "top 525 kHz of marine band"},
                       {h37, 1, "entire marine band"},

                       // Can handle Msg 22
                       {h38, 0, "No"},
                       {h39, 1, "Yes"},

                       // AIS mode
                       {h40, 0, "Autonomous"},
                       {h41, 1, "Assigned"},

                       // AIS communication state
                       {h42, 0, "SOTDMA"},
                       {h43, 1, "ITDMA"},

                       // Unit type
                       {h44, 0, "SOTDMA"},
                       {h45, 1, "CS"},

                       // AIS version indicator
                       {h46, 0, "ITU-R M.1371-1"},
                       {h47, 1, "ITU-R M.1371-3"},
                       {0, 2, ""},
                       {0, 3, ""},

                       // Type of ship
                       {h50, 0, "unavailable"},
                       {0, 0, "\0"},
                       {h52, 20, "Wing In Ground"},
                       {h53, 29, "Wing In Ground (no other information)"},
                       {h54, 30, "Fishing"},
                       {h55, 31, "Towing"},
                       {h56, 32, "Towing exceeds 200m or wider than 25m"},
                       {h57, 33, "Engaged in dredging or underwater operations"},
                       {h58, 34, "Engaged in diving operations"},
                       {h59, 35, "Engaged in military operations"},
                       {h60, 36, "Sailing"},
                       {h61, 37, "Pleasure"},
                       {h62, 40, "High speed craft"},
                       {h63, 41, "High speed craft carrying dangerous goods"},
                       {h64, 42, "High speed craft hazard cat B"},
                       {h65, 43, "High speed craft hazard cat C"},
                       {h66, 44, "High speed craft hazard cat D"},
                       {h67, 49, "High speed craft (no additional information)"},
                       {h68, 50, "Pilot vessel"},
                       {h69, 51, "SAR"},
                       {h70, 52, "Tug"},
                       {h71, 53, "Port tender"},
                       {h72, 54, "Anti-pollution"},
                       {h73, 55, "Law enforcement"},
                       {h74, 56, "Spare"},
                       {h75, 57, "Spare #2"},
                       {h76, 58, "Medical"},
                       {h77, 59, "RR Resolution No.18"},
                       {h78, 60, "Passenger ship"},
                       {h79, 69, "Passenger ship (no additional information)"},
                       {h80, 70, "Cargo ship"},
                       {h81, 71, "Cargo ship carrying dangerous goods"},
                       {h82, 72, "Cargo ship hazard cat B"},
                       {h83, 73, "Cargo ship hazard cat C"},
                       {h84, 74, "Cargo ship hazard cat D"},
                       {h85, 79, "Cargo ship (no additional information)"},
                       {h86, 80, "Tanker"},
                       {h87, 81, "Tanker carrying dangerous goods"},
                       {h88, 82, "Tanker hazard cat B"},
                       {h89, 83, "Tanker hazard cat C"},
                       {h90, 84, "Tanker hazard cat D"},
                       {h91, 89, "Tanker (no additional information)"},
                       {h92, 90, "Other"},
                       {h93, 91, "Other carrying dangerous goods"},
                       {h94, 92, "Other hazard cat B"},
                       {h95, 93, "Other hazard cat C"},
                       {h96, 94, "Other hazard cat D"},
                       {h97, 99, "Other (no additional information)"},

                       // EPFD fix types
                       {h98, 0, "undefined"},
                       {h99, 1, "GPS"},
                       {h100, 2, "GLONASS"},
                       {h101, 3, "GPS+GLONASS"},
                       {h102, 4, "Loran-C"},
                       {h103, 5, "Chayka"},
                       {h104, 6, "integrated"},
                       {h105, 7, "surveyed"},
                       {h106, 8, "Galileo"},
                       {h107, 15, "internal GNSS"}, // Not supported by canboat
                                                    // DTE
                       {h108, 0, "available"},
                       {h109, 1, "not available"},
                       {h110, 0, "Available"},
                       {h111, 1, "Not available"},

                       // AtoN Type
                       {h112, 0, "Default: Type of AtoN not specified"},
                       {h113, 1, "Referece point"},
                       {h114, 2, "RACON"},
                       {h115, 3, "Fixed structure off-shore"},
                       {h116, 4, "Reserved for future use"},
                       {h117, 5, "Fixed light: without sectors"},
                       {h118, 6, "Fixed light: with sectors"},
                       {h119, 7, "Fixed leading light front"},
                       {h120, 8, "Fixed leading light rear"},
                       {h121, 9, "Fixed beacon: cardinal N"},
                       {h122, 10, "Fixed beacon: cardinal E"},
                       {h123, 11, "Fixed beacon: cardinal S"},
                       {h124, 12, "Fixed beacon: cardinal W"},
                       {h125, 13, "Fixed beacon: port hand"},
                       {h126, 14, "Fixed beacon: starboard hand"},
                       {h127, 15, "Fixed beacon: preferred channel port hand"},
                       {h128, 16, "Fixed beacon: preferred channel starboard hand"},
                       {h129, 17, "Fixed beacon: isolated danger"},
                       {h130, 18, "Fixed beacon: safe water"},
                       {h131, 19, "Fixed beacon: special mark"}, // Missing in canboat!
                       {h132, 20, "Floating AtoN: cardinal N"},
                       {h133, 21, "Floating AtoN: cardinal E"},
                       {h134, 22, "Floating AtoN: cardinal S"},
                       {h135, 23, "Floating AtoN: cardinal W"},
                       {h136, 24, "Floating AtoN: port hand mark"},
                       {h137, 25, "Floating AtoN: starboard hand mark"},
                       {h138, 26, "Floating AtoN: preferred channel port hand"},
                       {h139, 27, "Floating AtoN: preferred channel starboard hand"},
                       {h140, 28, "Floating AtoN: isolated danger"},
                       {h141, 29, "Floating AtoN: safe water"},
                       {h142, 30, "Floating AtoN: special mark"},
                       {h143, 31, "Floating AtoN: light vessel/LANBY/rigs"},

                       // Assigned Mode Flag
                       {h144, 0, "Autonomous and continuous"},
                       {h145, 1, "Assigned mode"},

                       // Special Maneuver Indicator
                       {h146, 0, "Not available"},
                       {h147, 1, "Not engaged in special maneuver"},
                       {h148, 2, "Engaged in special maneuver"},
                       {h149, 3, "Reserved"}};

  // Calculate field offset in eValue list. Dealing with compiled values
  for (i = 0; fieldName[i] != '\0'; i++)
    h += fieldName[i];
  for (i = 0; i < sizeof(eType) / sizeof(aisKey); i++)
  {
    if (eType[i].hash == h && !strcmp(fieldName, eType[i].value))
      break;
  }
  base     = eType[i].first;
  elements = eType[i].elements;
  defValue = eType[i].defValue;

  // Find enum value
  {
    int k = 0;
    if (getJSONValue(msg, fieldName, jsonString, sizeof(jsonString)))
    {
      for (i = 0; jsonString[i] != '\0'; i++)
        k += jsonString[i];
      for (i = base; i < base + elements; i++)
      {
        if (eValue[i].hash == k && !strcmp(jsonString, eValue[i].value))
          break;
      }
      if (k)
        return eValue[i].num;
      else
        return defValue;
    }
    else
    {
      return defValue;
    }
  }
}

// Ship dimensions must be translated from pgn
long int aisShipDimensions(const char *msg, bool ship)
{
  long int length, beam, refBow, refStarboard, toStern, toPort;

  // Fetch pgn values
  if (ship)
  {
    length       = aisFloat(msg, "Length");
    beam         = aisFloat(msg, "Beam");
    refBow       = aisFloat(msg, "Position reference from Bow");
    refStarboard = aisFloat(msg, "Position reference from Starboard");
  }
  else
  {
    length       = aisFloat(msg, "Length/Diameter");
    beam         = aisFloat(msg, "Beam/Diameter");
    refBow       = aisFloat(msg, "Position Reference from True North Facing Edge");
    refStarboard = aisFloat(msg, "Position Reference from Starboard Edge");
  }

  // Calculate sentence values
  toStern = (length - refBow < 5110) ? length - refBow : 5110;
  toPort  = (beam - refStarboard < 630) ? beam - refStarboard : 630;

  // Pack values into 30 bits of the long integer
  return (((refBow + 5) / 10) << 21) + (((toStern + 5) / 10) << 12) + (((toPort + 5) / 10) << 6) + ((refStarboard + 5) / 10);
}

// Estimated time of arrival must be translated from pgn
long int aisETA(const char *msg)
{
  char         jstr[11];
  unsigned int month = 0, day = 0, hour = 24, minute = 60;

  // Get ais values from json (sscanf not used as it fails for som values)
  if (getJSONValue(msg, "ETA Date", jstr, sizeof(jstr)))
  {
    month = 10 * ((unsigned char) jstr[5] - '0') + ((unsigned char) jstr[6] - '0');
    day   = 10 * ((unsigned char) jstr[8] - '0') + ((unsigned char) jstr[9] - '0');
    if (month > 12)
      month = 0;
    if (day > 31)
      day = 0;
  }

  if (getJSONValue(msg, "ETA Time", jstr, sizeof(jstr)))
  {
    hour   = 10 * ((unsigned char) jstr[0] - '0') + ((unsigned char) jstr[1] - '0');
    minute = 10 * ((unsigned char) jstr[3] - '0') + ((unsigned char) jstr[4] - '0');
    if (hour > 24)
      hour = 24;
    if (minute > 60)
      minute = 60;
  }

  // Pack into ais integer
  return (month << 16) + (day << 11) + (hour << 6) + minute;
}

// PGN "Position Date" must be translated to sentence UTC date
long int aisDate(const char *msg)
{
  char         jstr[11];
  unsigned int year = 0, month = 0, day = 0;

  // Get ais values from json (sscanf not used as it fails for som values)
  if (getJSONValue(msg, "Position Date", jstr, sizeof(jstr)))
  {
    sscanf(jstr, "%d%*c%d%*c%d", &year, &month, &day);
    if (year > 9999)
      year = 0;
    if (month > 12)
      month = 0;
    if (day > 31)
      day = 0;
  }

  // Pack into ais integer
  return (year << 9) + (month << 5) + day;
}

// PGN "Position Time" must be translated to sentence UTC time
long int aisTime(const char *msg)
{
  char         jstr[9];
  unsigned int hour = 24, minute = 60, second = 60;

  // Get ais values from json (sscanf not used as it fails for som values)
  if (getJSONValue(msg, "Position Time", jstr, sizeof(jstr)))
  {
    hour   = 10 * ((unsigned char) jstr[0] - '0') + ((unsigned char) jstr[1] - '0');
    minute = 10 * ((unsigned char) jstr[3] - '0') + ((unsigned char) jstr[4] - '0');
    second = 10 * ((unsigned char) jstr[6] - '0') + ((unsigned char) jstr[7] - '0');
    if (hour > 24)
      hour = 24;
    if (minute > 60)
      minute = 60;
    if (second > 60)
      second = 60;
  }

  // Pack into ais integer
  return (hour << 12) + (minute << 6) + second;
}

// "AIS Aids to Navigation (AtoN) Report" has a special name format
char *aisAtoNName(const char *msg, bool extended, int *len)
{
  // The name splits in two parts, name and name extension.
  static char jsonString[35];
  int         i;

  // The extended name part is of variable length. Since the len byte has
  // been dropped, we have to rely on NULL termination of the string in
  // the extended part. First 20 chars ordinary name
  for (i = 20; i < 35; i++)
  {
    jsonString[i] = '\0';
  }

  if (!getJSONValue(msg, "AtoN Name", jsonString, sizeof(jsonString)))
    jsonString[0] = '\0';
  jsonString[34] = '\0';

  if (extended)
  {
    char *eName = (char *) jsonString + 20;
    for (i = 0; i <= 14; i++)
    {
      if (*(eName + i) == '\0')
        break;
    }
    // Each ais char is 6 bit. The length shall be padded to a multiple of 8 bits
    i *= 6;
    if (i % 8)
      i += 8 - i % 8;
    *len = i;
    return eName;
  }
  else
  {
    jsonString[20] = '\0';
  }
  return jsonString;
}

void nmea0183AIVDM(StringBuffer *msg183, int source, const char *msg)
{
  long int msgid, pgn;
  char     jstr[10];
  char     channel      = 'A';
  int      src          = 'I' - 'A'; // Should give source AI in sentence, i.e. Mobile AIS station
  char *   aisSource[2] = {"VDM", "VDO"};
  enum
  {
    VDM,
    VDO
  };
  int       aisTalkerId = VDM;
  aisVector aisPayload;
  aisPayload.pos = 0;

  if (!getJSONValue(msg, "pgn", jstr, sizeof(jstr)))
    return;
  pgn = atoi(jstr);

  // Source information
  switch (aisEnum(msg, "AIS Transceiver information"))
  {
    case 0: // Default
      channel     = 'A';
      aisTalkerId = VDM;
      break;
    case 1:
      channel     = 'B';
      aisTalkerId = VDM;
      break;
    case 2:
      channel     = 'A';
      aisTalkerId = VDO;
      break;
    case 3:
      channel     = 'B';
      aisTalkerId = VDO;
      break;
    case 4:              // Own information, how to code?
      aisTalkerId = VDO; // It is at least from own vessel
      break;
    default: // Last case supposedly not used
      return;
  }

  // AIS payload bit vector
  msgid = aisInteger(msg, "Message ID");
  switch (msgid)
  {
    case 1:
    case 2:
    case 3: // PGN 129038 "Class A position report"
      // Common navigation block
      addAisInt(msgid, 6, &aisPayload);
      addAisInt(aisEnum(msg, "Repeat Indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "User ID"), 30, &aisPayload);
      addAisInt(aisEnum(msg, "Nav Status"), 4, &aisPayload);
      addAisInt(aisFloat(msg, "Rate of Turn"), 8, &aisPayload);
      addAisInt(aisFloat(msg, "SOG"), 10, &aisPayload);
      addAisInt(aisEnum(msg, "Position Accuracy"), 1, &aisPayload);
      addAisInt(aisFloat(msg, "Longitude"), 28, &aisPayload);
      addAisInt(aisFloat(msg, "Latitude"), 27, &aisPayload);
      addAisInt(aisFloat(msg, "COG"), 12, &aisPayload);
      addAisInt(aisFloat(msg, "Heading"), 9, &aisPayload);
      addAisInt(aisInteger(msg, "Time Stamp"), 6, &aisPayload);
      addAisInt(aisEnum(msg, "Special Maneuver Indicator"), 2, &aisPayload);
      addAisInt(0, 3, &aisPayload); /* Spare */
      addAisInt(aisEnum(msg, "RAIM"), 1, &aisPayload);
      addAisInt(aisInteger(msg, "Communication State"), 19, &aisPayload);
      break;
    case 4: // PGN 129793 "AIS UTC and Date Report"
      addAisInt(msgid, 6, &aisPayload);
      addAisInt(aisEnum(msg, "Repeat Indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "User ID"), 30, &aisPayload);
      addAisInt(aisDate(msg), 23, &aisPayload);
      addAisInt(aisTime(msg), 17, &aisPayload);
      addAisInt(aisEnum(msg, "Position Accuracy"), 1, &aisPayload);
      addAisInt(aisFloat(msg, "Longitude"), 28, &aisPayload);
      addAisInt(aisFloat(msg, "Latitude"), 27, &aisPayload);
      addAisInt(aisEnum(msg, "GNSS type"), 4, &aisPayload);
      addAisInt(0, 10, &aisPayload); // Spare
      addAisInt(aisEnum(msg, "RAIM"), 1, &aisPayload);
      addAisInt(aisInteger(msg, "Communication State"), 19, &aisPayload);
      break;
    case 5: // PGN 129794 "AIS Class A Static and Voyage Related Data"
      addAisInt(msgid, 6, &aisPayload);
      addAisInt(aisEnum(msg, "Repeat Indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "User ID"), 30, &aisPayload);
      addAisInt(aisEnum(msg, "AIS version indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "IMO number"), 30, &aisPayload);
      addAisString(aisString(msg, "Callsign"), 42, &aisPayload);
      addAisString(aisString(msg, "Name"), 120, &aisPayload);
      addAisInt(aisEnum(msg, "Type of ship"), 8, &aisPayload);
      addAisInt(aisShipDimensions(msg, true), 30, &aisPayload);
      addAisInt(aisEnum(msg, "GNSS type"), 4, &aisPayload);
      addAisInt(aisETA(msg), 20, &aisPayload);
      addAisInt(aisFloat(msg, "Draft"), 8, &aisPayload);
      addAisString(aisString(msg, "Destination"), 120, &aisPayload);
      addAisInt(aisEnum(msg, "DTE"), 1, &aisPayload);
      addAisInt(0, 1, &aisPayload); // Spare
      break;
    case 9: // PGN 129798 "AIS SAR Aircraft Position Report"
      addAisInt(msgid, 6, &aisPayload);
      addAisInt(aisEnum(msg, "Repeat Indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "User ID"), 30, &aisPayload);
      addAisInt(aisFloat(msg, "Altitude"), 12, &aisPayload);
      addAisInt((aisFloat(msg, "SOG") + 5) / 10, 10, &aisPayload);
      addAisInt(aisEnum(msg, "Position Accuracy"), 1, &aisPayload);
      addAisInt(aisFloat(msg, "Longitude"), 28, &aisPayload);
      addAisInt(aisFloat(msg, "Latitude"), 27, &aisPayload);
      addAisInt(aisFloat(msg, "COG"), 12, &aisPayload);
      addAisInt(aisInteger(msg, "Time Stamp"), 6, &aisPayload);
      addAisInt(0, 8, &aisPayload); /* Regional Reserved */
      addAisInt(aisEnum(msg, "DTE"), 1, &aisPayload);
      addAisInt(0, 3, &aisPayload);                        // Spare
      addAisInt(aisEnum(msg, "AIS mode"), 1, &aisPayload); // Missing in PGN?
      addAisInt(aisEnum(msg, "RAIM"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "AIS communication state"), 1, &aisPayload); // Not in PGN?, defaults to 0
      addAisInt(aisInteger(msg, "Communication State"), 19, &aisPayload);
      break;
    case 12: // PGN 129801 "AIS Addressed Safety Related Message"
      addAisInt(msgid, 6, &aisPayload);
      addAisInt(aisEnum(msg, "Repeat Indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "Source ID"), 30, &aisPayload);
      addAisInt(aisInteger(msg, "Sequence Number"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "Destination ID"), 30, &aisPayload);
      addAisInt(aisInteger(msg, "Retransmit flag"), 1, &aisPayload);
      addAisInt(0, 1, &aisPayload); // Spare
      addAisLongString(msg, "Safety Related Text", 156, true, &aisPayload);
      break;
    case 14: // PGN 129802 "AIS Safety Related Broadcast Message"
      /* Note: The AIS sentence transmitts up to 161 characters of text in the message
         The incomplete PGN in pgns.json has a text field of 288 bits, corresponding
         to a maximum of 36 characters.
      */
      addAisInt(msgid, 6, &aisPayload);
      addAisInt(aisEnum(msg, "Repeat Indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "Source ID"), 30, &aisPayload);
      addAisInt(0, 2, &aisPayload); // Spare
      addAisLongString(msg, "Safety Related Text", 161, true, &aisPayload);
      break;
    case 18: // PGN 129039 "AIS Class B Position Report"
      addAisInt(msgid, 6, &aisPayload);
      addAisInt(aisEnum(msg, "Repeat Indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "User ID"), 30, &aisPayload);
      addAisInt(0, 8, &aisPayload); /* Regional Reserved */
      addAisInt(aisFloat(msg, "SOG"), 10, &aisPayload);
      addAisInt(aisEnum(msg, "Position Accuracy"), 1, &aisPayload);
      addAisInt(aisFloat(msg, "Longitude"), 28, &aisPayload);
      addAisInt(aisFloat(msg, "Latitude"), 27, &aisPayload);
      addAisInt(aisFloat(msg, "COG"), 12, &aisPayload);
      addAisInt(aisFloat(msg, "Heading"), 9, &aisPayload);
      addAisInt(aisInteger(msg, "Time Stamp"), 6, &aisPayload);
      addAisInt(0, 2, &aisPayload); /* Regional Reserved */
      addAisInt(aisEnum(msg, "Unit type"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "Integrated Display"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "DSC"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "Band"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "Can handle Msg 22"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "AIS mode"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "RAIM"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "AIS communication state"), 1, &aisPayload);
      /* If "AIS communication state" is CS (1), then the following
         "Communication State" shall have its default value. Provided the
         PGN is properly encoded this should work even with no test on
         "AIS communication state" */
      addAisInt(aisInteger(msg, "Communication State"), 19, &aisPayload);
      break;
    case 19: // PGN 129040 "AIS Class B Extended Position Report"
      addAisInt(msgid, 6, &aisPayload);
      addAisInt(aisEnum(msg, "Repeat Indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "User ID"), 30, &aisPayload);
      addAisInt(0, 8, &aisPayload); /* Regional Reserved */
      addAisInt(aisFloat(msg, "SOG"), 10, &aisPayload);
      addAisInt(aisEnum(msg, "Position Accuracy"), 1, &aisPayload);
      addAisInt(aisFloat(msg, "Longitude"), 28, &aisPayload);
      addAisInt(aisFloat(msg, "Latitude"), 27, &aisPayload);
      addAisInt(aisFloat(msg, "COG"), 12, &aisPayload);
      addAisInt(aisFloat(msg, "True Heading"), 9, &aisPayload);
      addAisInt(aisInteger(msg, "Time Stamp"), 6, &aisPayload);
      addAisInt(0, 4, &aisPayload); /* Regional Reserved */
      addAisString(aisString(msg, "Name"), 120, &aisPayload);
      addAisInt(aisEnum(msg, "Type of ship"), 8, &aisPayload);
      addAisInt(aisShipDimensions(msg, true), 30, &aisPayload);
      addAisInt(aisEnum(msg, "GNSS type"), 4, &aisPayload);
      addAisInt(aisEnum(msg, "AIS RAIM flag"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "DTE"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "AIS mode"), 1, &aisPayload);
      addAisInt(0, 4, &aisPayload); // Spare
      break;
    case 21: // PGN 129041 "AIS Aids to Navigation (AtoN) Report"
    {
      int   len;
      char *ename;

      addAisInt(msgid, 6, &aisPayload);
      addAisInt(aisEnum(msg, "Repeat Indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "User ID"), 30, &aisPayload);
      addAisInt(aisEnum(msg, "AtoN Type"), 5, &aisPayload);
      addAisString(aisAtoNName(msg, false, &len), 120, &aisPayload);
      addAisInt(aisEnum(msg, "Position Accuracy"), 1, &aisPayload);
      addAisInt(aisFloat(msg, "Longitude"), 28, &aisPayload);
      addAisInt(aisFloat(msg, "Latitude"), 27, &aisPayload);
      addAisInt(aisShipDimensions(msg, false), 30, &aisPayload);
      addAisInt(aisEnum(msg, "GNSS type"), 4, &aisPayload);
      addAisInt(aisInteger(msg, "Time Stamp"), 6, &aisPayload);
      addAisInt(aisEnum(msg, "Off Position Indicator"), 1, &aisPayload);
      addAisInt(0, 8, &aisPayload); /* Regional Reserved */
      addAisInt(aisEnum(msg, "AIS RAIM flag"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "Virtual AtoN Flag"), 1, &aisPayload);
      addAisInt(aisEnum(msg, "Assigned Mode Flag"), 1, &aisPayload);
      addAisInt(0, 1, &aisPayload); // Spare
      ename = aisAtoNName(msg, true, &len);
      addAisString(ename, len, &aisPayload);
    }
    break;
    case 24: // PGN 129809 "AIS Class B "CS" Static Data Report, Part A"
             // PGN 129810 "AIS Class B "CS" Static Data Report, Part B"
      addAisInt(msgid, 6, &aisPayload);
      addAisInt(aisEnum(msg, "Repeat Indicator"), 2, &aisPayload);
      addAisInt(aisInteger(msg, "User ID"), 30, &aisPayload);
      // Part A or B?
      switch (pgn)
      {
        case 129809:                    // Part A
          addAisInt(0, 2, &aisPayload); // Part number
          addAisString(aisString(msg, "Name"), 120, &aisPayload);
          addAisInt(0, 8, &aisPayload); // Spare
          break;
        case 129810:                    // Part B
          addAisInt(1, 2, &aisPayload); // Part number
          addAisInt(aisEnum(msg, "Type of ship"), 8, &aisPayload);
          addAisString(aisString(msg, "Vendor ID"), 42, &aisPayload);
          addAisString(aisString(msg, "Callsign"), 42, &aisPayload);
          addAisInt(aisShipDimensions(msg, true), 30, &aisPayload);
          addAisInt(aisInteger(msg, "Mothership User ID"), 30, &aisPayload);
          addAisInt(0, 6, &aisPayload); // Spare
          break;
        default:
          return;
      }
      break;
    default:
      return;
  }

  // Partition, encode and send nmea0183 ais sentences
  aisToNmea0183(msg183, 256 + src, aisSource[aisTalkerId], channel, &aisPayload);
}
