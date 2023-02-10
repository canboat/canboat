/*

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

#include "gps_ais.h"

#include <math.h>
#include <time.h>

#include "common.h"
#include "n2kd.h"
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

extern void nmea0183VTG(StringBuffer *msg183, int src, const char *msg)
{
  double sog;
  double cog;

  if (getJSONNumber(msg, "SOG", &sog, U_VELOCITY) && getJSONNumber(msg, "COG", &cog, U_ANGLE))
  {
    nmea0183CreateMessage(msg183, src, "VTG,%.1f,T,,M,%.2f,N,%.2f,K", cog, SPEED_M_S_TO_KNOTS(sog), SPEED_M_S_TO_KMH(sog));
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

extern void nmea0183GSA(StringBuffer *msg183, int src, const char *msg)
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

extern void nmea0183GLL(StringBuffer *msg183, int src, const char *msg)
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
  {
    return 0;
  }

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
  {
    return 0;
  }

  while (len >= 6)
  {
    // Encode current char to 6-bit ascii
    // 32 -> 63 => 32 -> 63, 64 -> 95 => 0 -> 31
    nextchar = *string;
    if (nextchar == '\0')
    {
      nextchar = 0;
    }
    else if (nextchar < 32 || nextchar > 95)
    {
      nextchar = 32;
    }
    else if (nextchar >= 64)
    {
      nextchar -= 64;
    }
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
    {
      string++;
    }
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
static char *aisString(const char *msg, const char *fieldName)
{
  static char jsonString[21];

  if (!getJSONValue(msg, fieldName, jsonString, sizeof(jsonString)))
  {
    jsonString[0] = '\0';
  }
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
  {
    return 0;
  }

  if (!getJSONValue(msg, fieldName, jsonString, maxSize))
  {
    free(jsonString);
    return 0;
  }

  for (len = 0; len <= maxSize; len++)
  {
    if (*(jsonString + len) == '\0')
    {
      break;
    }
  }

  len *= 6;
  if (padd)
  {
    if (len % 8)
    {
      len += 8 - (len % 8); // Add padd bits
    }
  }

  addAisString(jsonString, len, payload);
  free(jsonString);
  return len;
}

/*
Extracts and encodes next 6 bit value from the packed ais vector.
*/
static char nextPayloadChar(aisVector *bitVec, int *opos, unsigned char *padding)
{
  unsigned char i;
  int           start = *opos / 8;

  if (*opos >= bitVec->pos)
  {
    return '\0';
  }

  if (bitVec->pos - *opos < 6)
  {
    *padding = 6 - (bitVec->pos - *opos);
  }

  i = (bitVec->bitVector[start] << (*opos % 8));
  i = i >> 2;
  if ((8 - *opos % 8) + *padding < 6)
  {
    i |= (unsigned char) bitVec->bitVector[start + 1] >> (10 - (*opos % 8));
  }
  i &= 0xff << *padding;
  // Make shure *opos does not exceede bitVec->pos
  *opos += 6 - *padding;

  // Ascii encode the 6-bit value before return
  if (i < 40)
  {
    return (char) (i + 48);
  }
  else
  {
    return (char) (i + 56);
  }
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
  {
    fragments++;
  }
  // We maintain 10 sequential message ID for multi-sentence messages
  if (fragments > 1)
  {
    sequenceId = (((sequenceId - '0') + 1) % 10) + '0';
  }

  while (fragCntr <= fragments)
  {
    char payload[61];
    int  i = 0;

    while (i < 60 && opos <= bitVec->pos)
    {
      payload[i] = nextPayloadChar(bitVec, &opos, &padding);
      if (payload[i] == '\0')
      {
        break;
      }
      i++;
    }
    payload[i] = '\0';

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

static int aisEnum(const char *, const char *);

/*
Ais nummerical values
*/
static long int aisInteger(const char *msg, const char *fieldName)
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
  static const int p1 = 'U' + 's' + 'e' + 'r' + ' ' + 'I' + 'D';
  static const int p2
      = 'C' + 'o' + 'm' + 'm' + 'u' + 'n' + 'i' + 'c' + 'a' + 't' + 'i' + 'o' + 'n' + ' ' + 'S' + 't' + 'a' + 't' + 'e';
  static const int p3 = 'I' + 'M' + 'O' + ' ' + 'n' + 'u' + 'm' + 'b' + 'e' + 'r';
  static const int p4 = 'M' + 'o' + 't' + 'h' + 'e' + 'r' + 's' + 'h' + 'i' + 'p' + ' ' + 'U' + 's' + 'e' + 'r' + ' ' + 'I' + 'D';
  static const int p5 = 'S' + 'o' + 'u' + 'r' + 'c' + 'e' + ' ' + 'I' + 'D';
  static const int p6 = 'S' + 'e' + 'q' + 'u' + 'e' + 'n' + 'c' + 'e' + ' ' + 'N' + 'u' + 'm' + 'b' + 'e' + 'r';
  static const int p7 = 'D' + 'e' + 's' + 't' + 'i' + 'n' + 'a' + 't' + 'i' + 'o' + 'n' + ' ' + 'I' + 'D';
  static const int p8 = 'R' + 'e' + 't' + 'r' + 'a' + 'n' + 's' + 'm' + 'i' + 't' + ' ' + 'f' + 'l' + 'a' + 'g';

  char     jsonString[40];
  int      i, h = 0;
  long int value;

  intParam paramRange[] = {{p1, "User ID", 0, 999999999, 0},
                           {p2, "Communication State", 0, 524287, 393222},
                           {p3, "IMO number", 1000000, 9999999, 0},
                           {p4, "Mothership User ID", 0, 999999999, 0},
                           {p5, "Source ID", 0, 999999999, 0},
                           {p6, "Sequence Number", 0, 3, 0},
                           {p7, "Destination ID", 0, 999999999, 0},
                           {p8, "Retransmit flag", 0, 1, 0}};

  // Calculate index
  for (i = 0; fieldName[i] != '\0'; i++)
  {
    h += fieldName[i];
  }
  for (i = 0; i < sizeof(paramRange) / sizeof(intParam); i++)
  {
    if (paramRange[i].hash == h && !strcmp(fieldName, paramRange[i].value))
    {
      break;
    }
  }

  if (!getJSONValue(msg, fieldName, jsonString, sizeof(jsonString)))
  {
    return paramRange[i].defValue;
  }
  if (jsonString[0] == '\0')
  {
    return paramRange[i].defValue;
  }
  value = atol(jsonString);
  if (value >= paramRange[i].min && value <= paramRange[i].max)
  {
    return value;
  }
  return paramRange[i].defValue;
}

static long int aisFloat(const char *msg, const char *fieldName)
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

  floatParam paramRange[] = {{/* Clamp Rate of Turn to -127 .. 127 (which boils down to 720 degs/min)
                               */
                              "Rate of Turn",
                              -127,
                              127,
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
  sign = (value >= 0) - (value < 0);
  value *= sign;
  if (i == 0)
  {
    // ROT is calculated with a non-linear value in ITU-R M.1371...
    result = (long int) (ROT_MULTIPLICATOR * sqrt(value) + 0.5);
    result = CB_MAX(result, paramRange[i].min);
    result = CB_MIN(result, paramRange[i].max);
    logDebug("ROT %f deg/min -> %d\n", value, result);
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
static int aisEnum(const char *msg, const char *fieldName)
{
  int64_t n;

  if (getJSONLookupValue(msg, fieldName, &n))
  {
    logDebug("getJSONLookupValue(msg, '%s') = %" PRId64 "\n", fieldName, n);
    return (int) n;
  }
  logError("getJSONLookupValue(msg, '%s') = no result -> 0\n", fieldName);
  return 0;
}

// Ship dimensions must be translated from pgn
static long int aisShipDimensions(const char *msg, bool ship)
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
static long int aisDate(const char *msg)
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
static long int aisTime(const char *msg)
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

extern void nmea0183AIVDM(StringBuffer *msg183, int source, const char *msg)
{
  long int msgid, pgn;
  char     jstr[10];
  char     channel      = 'A';
  int      src          = 'I' - 'A'; // Should give source AI in sentence, i.e. Mobile AIS station
  char    *aisSource[2] = {"VDM", "VDO"};
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
  msgid = aisEnum(msg, "Message ID");
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
      addAisInt(aisEnum(msg, "Time Stamp"), 6, &aisPayload);
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
      addAisInt(aisEnum(msg, "Time Stamp"), 6, &aisPayload);
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
      addAisInt(aisEnum(msg, "Time Stamp"), 6, &aisPayload);
      addAisInt(0, 4, &aisPayload); /* Regional Reserved */
      addAisString(aisString(msg, "Name"), 120, &aisPayload);
      addAisInt(aisEnum(msg, "Type of ship"), 8, &aisPayload);
      addAisInt(aisShipDimensions(msg, true), 30, &aisPayload);
      addAisInt(aisEnum(msg, "GNSS type"), 4, &aisPayload);
      addAisInt(aisEnum(msg, "RAIM"), 1, &aisPayload);
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
      addAisInt(aisEnum(msg, "Time Stamp"), 6, &aisPayload);
      addAisInt(aisEnum(msg, "Off Position Indicator"), 1, &aisPayload);
      addAisInt(0, 8, &aisPayload); /* Regional Reserved */
      addAisInt(aisEnum(msg, "RAIM"), 1, &aisPayload);
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
