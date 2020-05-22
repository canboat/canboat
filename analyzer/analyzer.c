/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2015, Kees Verruijt, Harlingen, The Netherlands.

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

#define GLOBALS
#include "analyzer.h"
#include "common.h"

enum RawFormats
{
  RAWFORMAT_UNKNOWN,
  RAWFORMAT_PLAIN,
  RAWFORMAT_FAST,
  RAWFORMAT_AIRMAR,
  RAWFORMAT_CHETCO,
  RAWFORMAT_GARMIN_CSV1,
  RAWFORMAT_GARMIN_CSV2,
  RAWFORMAT_YDWG02
};

enum RawFormats format = RAWFORMAT_UNKNOWN;

enum MultiPackets
{
  MULTIPACKETS_COALESCED,
  MULTIPACKETS_SEPARATE
};

enum MultiPackets multiPackets = MULTIPACKETS_SEPARATE;

enum GeoFormats
{
  GEO_DD,
  GEO_DM,
  GEO_DMS
};

typedef struct
{
  size_t   lastFastPacket;
  size_t   size;
  size_t   allocSize;
  uint8_t *data;
} Packet;

typedef struct
{
  Packet packetList[ARRAY_SIZE(pgnList)];
} DevicePackets;

DevicePackets *device[256];
char *         manufacturer[1 << 12];

bool            showRaw       = false;
bool            showData      = false;
bool            showBytes     = false;
bool            showJson      = false;
bool            showJsonValue = false;
bool            showSI        = false; // Output everything in strict SI units
char *          sep           = " ";
char            closingBraces[8]; // } and ] chars to close sentence in JSON mode, otherwise empty string
enum GeoFormats showGeo  = GEO_DD;
int             onlyPgn  = 0;
int             onlySrc  = -1;
int             clockSrc = -1;
size_t          heapSize = 0;

int g_variableFieldRepeat[2]; // Actual number of repetitions
int g_variableFieldIndex;

static enum RawFormats detectFormat(const char *msg);
static bool            printCanFormat(RawMessage *msg);
static bool            printNumber(char *fieldName, Field *field, uint8_t *data, size_t startBit, size_t bits);
static void            camelCase(bool upperCamelCase);
static void            explain(void);
static void            explainXML(void);
static void            fillFieldCounts(void);
static void            fillManufacturers(void);
static void            printCanRaw(RawMessage *msg);

void usage(char **argv, char **av)
{
  printf("Unknown or invalid argument %s\n", av[0]);
  printf("Usage: %s [[-raw] [-json [-nv] [-camel | -upper-camel]] [-data] [-debug] [-d] [-q] [-si] [-geo {dd|dm|dms}] [-src <src> "
         "| <pgn>]] ["
#ifndef SKIP_SETSYSTEMCLOCK
         "-clocksrc <src> | "
#endif
         "-explain | -explain-xml [-upper-camel]] | -version\n",
         argv[0]);
  exit(1);
}

int main(int argc, char **argv)
{
  int    r;
  char   msg[2000];
  FILE * file         = stdin;
  int    ac           = argc;
  char **av           = argv;
  bool   doExplainXML = false;
  bool   doExplain    = false;

  setProgName(argv[0]);

  for (; ac > 1; ac--, av++)
  {
    if (strcasecmp(av[1], "-version") == 0)
    {
      printf("%s\n", VERSION);
      exit(0);
    }
    else if (strcasecmp(av[1], "-explain-xml") == 0)
    {
      doExplainXML = true;
    }
    else if (strcasecmp(av[1], "-explain") == 0)
    {
      doExplain = true;
    }
    else if (strcasecmp(av[1], "-raw") == 0)
    {
      showRaw = true;
    }
    else if (strcasecmp(av[1], "-debug") == 0)
    {
      showBytes = true;
    }
    else if (strcasecmp(av[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
    }
    else if (strcasecmp(av[1], "-q") == 0)
    {
      setLogLevel(LOGLEVEL_ERROR);
    }
    else if (ac > 2 && strcasecmp(av[1], "-geo") == 0)
    {
      if (strcasecmp(av[2], "dd") == 0)
      {
        showGeo = GEO_DD;
      }
      else if (strcasecmp(av[2], "dm") == 0)
      {
        showGeo = GEO_DM;
      }
      else if (strcasecmp(av[2], "dms") == 0)
      {
        showGeo = GEO_DMS;
      }
      else
      {
        usage(argv, av + 1);
      }
      ac--;
      av++;
    }
    else if (strcasecmp(av[1], "-si") == 0)
    {
      showSI = true;
    }
    else if (strcasecmp(av[1], "-nosi") == 0)
    {
      showSI = false;
    }
    else if (strcasecmp(av[1], "-camel") == 0)
    {
      camelCase(false);
    }
    else if (strcasecmp(av[1], "-upper-camel") == 0)
    {
      camelCase(true);
    }
    else if (strcasecmp(av[1], "-json") == 0)
    {
      showJson = true;
    }
    else if (strcasecmp(av[1], "-nv") == 0)
    {
      showJsonValue = true;
    }
    else if (strcasecmp(av[1], "-data") == 0)
    {
      showData = true;
    }
    else if (ac > 2 && strcasecmp(av[1], "-src") == 0)
    {
      onlySrc = strtol(av[2], 0, 10);
      ac--;
      av++;
    }
#ifndef SKIP_SETSYSTEMCLOCK
    else if (ac > 2 && strcasecmp(av[1], "-clocksrc") == 0)
    {
      clockSrc = strtol(av[2], 0, 10);
      ac--;
      av++;
    }
#endif
    else if (ac > 2 && strcasecmp(av[1], "-file") == 0)
    {
      file = fopen(av[2], "r");
      if (!file)
      {
        printf("Cannot open file %s\n", av[2]);
        exit(1);
      }
      ac--;
      av++;
    }
    else
    {
      onlyPgn = strtol(av[1], 0, 10);
      if (onlyPgn > 0)
      {
        printf("Only logging PGN %d\n", onlyPgn);
      }
      else
      {
        usage(argv, av + 1);
      }
    }
  }

  if (doExplain)
  {
    explain();
    exit(0);
  }
  if (doExplainXML)
  {
    if (!pgnList[0].camelDescription)
    {
      camelCase(false);
    }
    explainXML();
    exit(0);
  }

  if (!showJson)
  {
    logInfo("N2K packet analyzer\n" COPYRIGHT);
  }

  fillManufacturers();
  fillFieldCounts();
  checkPgnList();

  while (fgets(msg, sizeof(msg) - 1, file))
  {
    RawMessage   m;
    unsigned int prio, pgn, dst, src, junk;
    char *       p;
    unsigned int i;

    if (*msg == 0 || *msg == '\n')
    {
      continue;
    }

    if (format == RAWFORMAT_UNKNOWN)
    {
      format = detectFormat(msg);
      if (format == RAWFORMAT_GARMIN_CSV1 || format == RAWFORMAT_GARMIN_CSV2)
      {
        // Skip first line containing header line
        continue;
      }
    }

    switch (format)
    {
      case RAWFORMAT_PLAIN:
        r = parseRawFormatPlain(msg, &m, showJson);
        if (r >= 0)
        {
          break;
        }
        // Else fall through to fast!

      case RAWFORMAT_FAST:
        r = parseRawFormatFast(msg, &m, showJson);
        break;

      case RAWFORMAT_AIRMAR:
        r = parseRawFormatAirmar(msg, &m, showJson);
        break;

      case RAWFORMAT_CHETCO:
        r = parseRawFormatChetco(msg, &m, showJson);
        break;

      case RAWFORMAT_GARMIN_CSV1:
      case RAWFORMAT_GARMIN_CSV2:
        r = parseRawFormatGarminCSV(msg, &m, showJson, format == RAWFORMAT_GARMIN_CSV2);
        break;

      case RAWFORMAT_YDWG02:
        r = parseRawFormatYDWG02(msg, &m, showJson);
        break;

      default:
        logError("Unknown message format\n");
        exit(1);
    }

    if (r == 0)
    {
      printCanFormat(&m);
      printCanRaw(&m);
    }
    else
    {
      logError("Unknown message error %d: %s\n", r, msg);
    }
  }

  return 0;
}

enum RawFormats detectFormat(const char *msg)
{
  char *       p;
  int          r;
  unsigned int len;

  if (msg[0] == '$' && strncmp(msg, "$PCDIN", 6) == 0)
  {
    logInfo("Detected Chetco protocol with all data on one line\n");
    multiPackets = MULTIPACKETS_COALESCED;
    return RAWFORMAT_CHETCO;
  }

  if (strcmp(msg, "Sequence #,Timestamp,PGN,Name,Manufacturer,Remote Address,Local Address,Priority,Single Frame,Size,Packet\n")
      == 0)
  {
    logInfo("Detected Garmin CSV protocol with relative timestamps\n");
    multiPackets = MULTIPACKETS_COALESCED;
    return RAWFORMAT_GARMIN_CSV1;
  }

  if (strcmp(msg,
             "Sequence #,Month_Day_Year_Hours_Minutes_Seconds_msTicks,PGN,Processed PGN,Name,Manufacturer,Remote Address,Local "
             "Address,Priority,Single Frame,Size,Packet\n")
      == 0)
  {
    logInfo("Detected Garmin CSV protocol with absolute timestamps\n");
    multiPackets = MULTIPACKETS_COALESCED;
    return RAWFORMAT_GARMIN_CSV2;
  }

  p = strchr(msg, ' ');
  if (p && (p[1] == '-' || p[2] == '-'))
  {
    logInfo("Detected Airmar protocol with all data on one line\n");
    multiPackets = MULTIPACKETS_COALESCED;
    return RAWFORMAT_AIRMAR;
  }

  p = strchr(msg, ',');
  if (p)
  {
    r = sscanf(p, ",%*u,%*u,%*u,%*u,%u,%*x,%*x,%*x,%*x,%*x,%*x,%*x,%*x,%*x", &len);
    if (r < 1)
    {
      return RAWFORMAT_UNKNOWN;
    }
    if (len > 8)
    {
      logInfo("Detected normal format with all data on one line\n");
      multiPackets = MULTIPACKETS_COALESCED;
      return RAWFORMAT_FAST;
    }
    logInfo("Assuming normal format with one line per packet\n");
    multiPackets = MULTIPACKETS_SEPARATE;
    return RAWFORMAT_PLAIN;
  }

  {
    int  a, b, c, d, f;
    char e;
    if (sscanf(msg, "%d:%d:%d.%d %c %02X ", &a, &b, &c, &d, &e, &f) == 6 && (e == 'R' || e == 'T'))
    {
      logInfo("Detected YDWG-02 protocol with one line per packet\n");
      multiPackets = MULTIPACKETS_SEPARATE;
      return RAWFORMAT_YDWG02;
    }
  }

  return RAWFORMAT_UNKNOWN;
}

char *getSep()
{
  char *s = sep;

  if (showJson)
  {
    sep = ",";
    if (strchr(s, '{'))
    {
      if (strlen(closingBraces) >= sizeof(closingBraces) - 2)
      {
        logError("Too many braces\n");
        exit(2);
      }
      strcat(closingBraces, "}");
    }
  }
  else
  {
    sep = ";";
  }

  return s;
}

static void fillManufacturers(void)
{
  size_t i;

  for (i = 0; i < ARRAY_SIZE(manufacturer); i++)
  {
    manufacturer[i] = 0;
  }
  for (i = 0; i < ARRAY_SIZE(companyList); i++)
  {
    manufacturer[companyList[i].id] = companyList[i].name;
  }
}

static void fillFieldCounts(void)
{
  size_t i, j;

  for (i = 0; i < ARRAY_SIZE(pgnList); i++)
  {
    for (j = 0; pgnList[i].fieldList[j].name && j < ARRAY_SIZE(pgnList[i].fieldList); j++)
      ;
    if (j == ARRAY_SIZE(pgnList[i].fieldList))
    {
      logError("Internal error: PGN %d '%s' does not have correct fieldlist.\n", pgnList[i].pgn, pgnList[i].description);
      exit(2);
    }
    if (j == 0 && pgnList[i].complete == PACKET_COMPLETE)
    {
      logError("Internal error: PGN %d '%s' does not have fields.\n", pgnList[i].pgn, pgnList[i].description);
      exit(2);
    }
    pgnList[i].fieldCount = j;
  }
}

char  mbuf[8192];
char *mp = mbuf;

void mprintf(const char *format, ...)
{
  va_list ap;
  int     remain;

  va_start(ap, format);
  remain = sizeof(mbuf) - (mp - mbuf) - 1;
  if (remain > 0)
  {
    mp += vsnprintf(mp, remain, format, ap);
  }
  va_end(ap);
}

void mreset(void)
{
  mp = mbuf;
}

void mwrite(FILE *stream)
{
  fwrite(mbuf, sizeof(char), mp - mbuf, stream);
  fflush(stream);
  mreset();
}

static void printCanRaw(RawMessage *msg)
{
  size_t i;
  FILE * f = stdout;

  if (onlySrc >= 0 && onlySrc != msg->src)
  {
    return;
  }

  if (showJson)
  {
    f = stderr;
  }

  if (showRaw && (!onlyPgn || onlyPgn == msg->pgn))
  {
    fprintf(f, "%s %u %03u %03u %6u :", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn);
    for (i = 0; i < msg->len; i++)
    {
      fprintf(f, " %02x", msg->data[i]);
    }
    putc('\n', f);
  }
}

/*
 * There are three ways to print lat/long: DD, DM, DMS.
 * We print in the way set by the config. Default is DMS. DD is useful for Google Maps.
 */

static bool printLatLon(char *name, double resolution, uint8_t *data, size_t bytes)
{
  uint64_t absVal;
  int64_t  value;
  size_t   i;

  value = 0;
  for (i = 0; i < bytes; i++)
  {
    value |= ((uint64_t) data[i]) << (i * 8);
  }
  if (bytes == 4 && ((data[3] & 0x80) > 0))
  {
    value |= UINT64_C(0xffffffff00000000);
  }
  if (value > ((bytes == 8) ? INT64_C(0x7ffffffffffffffd) : INT64_C(0x7ffffffd)))
  {
    return false;
  }

  if (bytes == 8)
  {
    if (showBytes)
    {
      mprintf("(%" PRIx64 " = %" PRId64 ") ", value, value);
    }

    value /= INT64_C(1000000000);
  }
  absVal = (value < 0) ? -value : value;

  if (showBytes)
  {
    mprintf("(%" PRId64 ") ", value);
  }

  if (showGeo == GEO_DD)
  {
    double dd = (double) value / (double) RES_LAT_LONG_PRECISION;

    if (showJson)
    {
      mprintf("%s\"%s\":%10.7f", getSep(), name, dd);
    }
    else
    {
      mprintf("%s %s = %10.7f", getSep(), name, dd);
    }
  }
  else if (showGeo == GEO_DM)
  {
    /* One degree = 10e6 */

    uint64_t degrees   = (absVal / RES_LAT_LONG_PRECISION);
    uint64_t remainder = (absVal % RES_LAT_LONG_PRECISION);
    double   minutes   = (remainder * 60) / (double) RES_LAT_LONG_PRECISION;

    mprintf((showJson ? "%s\"%s\":\"%02u&deg; %6.3f %c\"" : "%s %s = %02ud %6.3f %c"),
            getSep(),
            name,
            (uint32_t) degrees,
            minutes,
            ((resolution == RES_LONGITUDE) ? ((value >= 0) ? 'E' : 'W') : ((value >= 0) ? 'N' : 'S')));
  }
  else
  {
    uint32_t degrees   = (uint32_t)(absVal / RES_LAT_LONG_PRECISION);
    uint32_t remainder = (uint32_t)(absVal % RES_LAT_LONG_PRECISION);
    uint32_t minutes   = (remainder * 60) / RES_LAT_LONG_PRECISION;
    double   seconds   = (((uint64_t) remainder * 3600) / (double) RES_LAT_LONG_PRECISION) - (60 * minutes);

    mprintf((showJson ? "%s\"%s\":\"%02u&deg;%02u&rsquo;%06.3f&rdquo;%c\"" : "%s %s = %02ud %02u' %06.3f\"%c"),
            getSep(),
            name,
            degrees,
            minutes,
            seconds,
            ((resolution == RES_LONGITUDE) ? ((value >= 0) ? 'E' : 'W') : ((value >= 0) ? 'N' : 'S')));
    if (showJson)
    {
      double dd = (double) value / (double) RES_LAT_LONG_PRECISION;
      mprintf("%s\"%s_dd\":%10.7f", getSep(), name, dd);
    }
  }
  return true;
}

static bool printDate(char *name, uint16_t d)
{
  char       buf[sizeof("2008.03.10") + 1];
  time_t     t;
  struct tm *tm;

  if (d >= 0xfffd)
  {
    return false;
  }

  if (showBytes)
  {
    mprintf("(date %hx = %hd) ", d, d);
  }

  t  = d * 86400;
  tm = gmtime(&t);
  if (!tm)
  {
    logAbort("Unable to convert %u to gmtime\n", (unsigned int) t);
  }
  strftime(buf, sizeof(buf), "%Y.%m.%d", tm);
  if (showJson)
  {
    mprintf("%s\"%s\":\"%s\"", getSep(), name, buf);
  }
  else
  {
    mprintf("%s %s = %s", getSep(), name, buf);
  }
  return true;
}

static bool printTime(char *name, uint32_t t)
{
  uint32_t       hours;
  uint32_t       minutes;
  uint32_t       seconds;
  uint32_t       units;
  const uint32_t unitspersecond = 10000;

  if (t >= 0xfffffffd)
  {
    return false;
  }

  if (showBytes)
  {
    mprintf("(time %x = %u) ", t, t);
  }

  seconds = t / unitspersecond;
  units   = t % unitspersecond;
  minutes = seconds / 60;
  seconds = seconds % 60;
  hours   = minutes / 60;
  minutes = minutes % 60;

  if (showJson)
  {
    if (units)
    {
      mprintf("%s \"%s\": \"%02u:%02u:%02u.%05u\"", getSep(), name, hours, minutes, seconds, units);
    }
    else
    {
      mprintf("%s \"%s\": \"%02u:%02u:%02u\"", getSep(), name, hours, minutes, seconds);
    }
  }
  else
  {
    if (units)
    {
      mprintf("%s %s = %02u:%02u:%02u.%05u", getSep(), name, hours, minutes, seconds, units);
    }
    else
    {
      mprintf("%s %s = %02u:%02u:%02u", getSep(), name, hours, minutes, seconds);
    }
  }
  return true;
}

static bool printTemperature(char *name, uint32_t t, uint32_t bits, double resolution)
{
  double k = t * resolution;
  double c = k - 273.15;
  double f = c * 1.8 + 32;

  if ((bits == 16 && t >= 0xfffd) || (bits == 24 && t >= 0xfffffd))
  {
    return false;
  }

  if (showSI)
  {
    if (showJson)
    {
      mprintf("%s\"%s\":%.2f", getSep(), name, k, f);
    }
    else
    {
      mprintf("%s %s = %.2f K", getSep(), name, k);
    }
    return true;
  }

  if (showJson)
  {
    mprintf("%s\"%s\":%.2f", getSep(), name, c, f);
  }
  else
  {
    mprintf("%s %s = %.2f C (%.1f F)", getSep(), name, c, f);
  }

  return true;
}

static bool printPressure(char *name, uint32_t v, Field *field)
{
  int32_t pressure;
  double  bar;
  double  psi;

  if (field->size <= 16)
  {
    if (v >= 0xfffd)
    {
      return false;
    }
  }
  if (v >= 0xfffffffd)
  {
    return false;
  }

  // There are four types of known pressure: unsigned hectopascal, signed kpa, unsigned kpa, unsigned four bytes in pascal.

  if (field->hasSign)
  {
    pressure = (int16_t) v;
  }
  else
  {
    pressure = v;
  }
  // Now scale pascal properly, it is in hPa or kPa.
  if (field->units)
  {
    switch (field->units[0])
    {
      case 'h':
      case 'H':
        pressure *= 100;
        break;
      case 'k':
      case 'K':
        pressure *= 1000;
        break;
      case 'd':
        pressure /= 10;
        break;
    }
  }

  bar = pressure / 100000.0; /* 1000 hectopascal = 1 Bar */
  psi = pressure / 1450.377; /* Silly but still used in some parts of the world */

  if (showJson)
  {
    mprintf("%s\"%s\":%" PRId32 "", getSep(), name, pressure);
  }
  else
  {
    mprintf("%s %s = %.3f bar (%.1f PSI)", getSep(), name, bar, psi);
  }
  return true;
}

static void print6BitASCIIChar(uint8_t b)
{
  int c;
  if (b < 0x28)
  {
    c = b + 0x30;
  }
  else
  {
    c = b + 0x38;
  }
  if (showJson && (c == '\\'))
  {
    putchar(c);
  }
  putchar(c);
}

static bool print6BitASCIIText(char *name, uint8_t *data, size_t startBit, size_t bits)
{
  uint8_t  value        = 0;
  uint8_t  maxValue     = 0;
  uint8_t  bitMask      = 1 << startBit;
  uint64_t bitMagnitude = 1;
  size_t   bit;
  char     buf[128];

  if (showJson)
  {
    mprintf("%s\"%s\":\"", getSep(), name);
  }
  else
  {
    mprintf("%s %s = ", getSep(), name);
  }

  for (bit = 0; bit < bits && bit < sizeof(buf) * 8; bit++)
  {
    /* Act on the current bit */
    bool bitIsSet = (*data & bitMask) > 0;
    maxValue |= bitMagnitude;
    if (bitIsSet)
    {
      value |= bitMagnitude;
    }

    /* Find the next bit */
    if (bitMask == 128)
    {
      bitMask = 1;
      data++;
    }
    else
    {
      bitMask = bitMask << 1;
    }
    bitMagnitude = bitMagnitude << 1;

    if (bit % 6 == 5)
    {
      print6BitASCIIChar(value);
      value        = 0;
      bitMagnitude = 1;
    }
  }
  if (showJson)
  {
    mprintf("\"");
  }
  return true;
}

static bool printHex(char *name, uint8_t *data, size_t startBit, size_t bits)
{
  uint8_t  value        = 0;
  uint8_t  maxValue     = 0;
  uint8_t  bitMask      = 1 << startBit;
  uint64_t bitMagnitude = 1;
  size_t   bit;
  char     buf[128];

  if (showBytes)
  {
    mprintf("(%s,%p,%zu,%zu) ", name, data, startBit, bits);
  }

  if (showJson)
  {
    mprintf("%s\"%s\":\"", getSep(), name);
  }
  else
  {
    mprintf("%s %s = ", getSep(), name);
  }

  for (bit = 0; bit < bits && bit < sizeof(buf) * 8; bit++)
  {
    /* Act on the current bit */
    bool bitIsSet = (*data & bitMask) > 0;
    maxValue |= bitMagnitude;
    if (bitIsSet)
    {
      value |= bitMagnitude;
    }

    /* Find the next bit */
    if (bitMask == 128)
    {
      bitMask = 1;
      data++;
    }
    else
    {
      bitMask = bitMask << 1;
    }
    bitMagnitude = bitMagnitude << 1;

    if (bit % 8 == 7)
    {
      mprintf("%02x ", value);
      value        = 0;
      bitMagnitude = 1;
    }
  }
  if (showJson)
  {
    mprintf("\"");
  }
  return true;
}

static bool printDecimal(char *name, uint8_t *data, size_t startBit, size_t bits)
{
  uint8_t  value        = 0;
  uint8_t  maxValue     = 0;
  uint8_t  bitMask      = 1 << startBit;
  uint64_t bitMagnitude = 1;
  size_t   bit;
  char     buf[128];

  if (showBytes)
  {
    mprintf("(%s,%p,%zu,%zu) ", name, data, startBit, bits);
  }

  if (showJson)
  {
    mprintf("%s\"%s\":\"", getSep(), name);
  }
  else
  {
    mprintf("%s %s = ", getSep(), name);
  }

  for (bit = 0; bit < bits && bit < sizeof(buf) * 8; bit++)
  {
    /* Act on the current bit */
    bool bitIsSet = (*data & bitMask) > 0;
    maxValue |= bitMagnitude;
    if (bitIsSet)
    {
      value |= bitMagnitude;
    }

    /* Find the next bit */
    if (bitMask == 128)
    {
      bitMask = 1;
      data++;
    }
    else
    {
      bitMask = bitMask << 1;
    }
    bitMagnitude = bitMagnitude << 1;

    if (bit % 8 == 7)
    {
      if (value < 100)
      {
        mprintf("%02u", value);
      }
      value        = 0;
      bitMagnitude = 1;
    }
  }
  if (showJson)
  {
    mprintf("\"");
  }
  return true;
}

static bool printVarNumber(char *fieldName, Pgn *pgn, uint32_t refPgn, Field *field, uint8_t *data, size_t startBit, size_t *bits)
{
  Field *refField;
  size_t size, bytes;

  /* PGN 126208 contains variable field length.
   * The field length can be derived from the PGN mentioned earlier in the message,
   * plus the field number.
   */

  /*
   * This is rather hacky. We know that the 'data' pointer points to the n-th variable field
   * length and thus that the field number is exactly one byte earlier.
   */

  refField = getField(refPgn, data[-1] - 1);
  if (refField)
  {
    *bits = (refField->size + 7) & ~7; // Round # of bits in field refField up to complete bytes: 1->8, 7->8, 8->8 etc.
    if (showBytes)
    {
      mprintf("(refField %s size = %u in %zu bytes)", refField->name, refField->size, *bits / 8);
    }
    return printNumber(fieldName, field, data, startBit, refField->size);
  }

  logError("Pgn %d Field %s: cannot derive variable length from PGN %d field # %d\n", pgn->pgn, field->name, refPgn, data[-1]);
  *bits = 8; /* Gotta assume something */
  return false;
}

static bool printNumber(char *fieldName, Field *field, uint8_t *data, size_t startBit, size_t bits)
{
  bool    ret = false;
  int64_t value;
  int64_t maxValue;
  int64_t reserved;
  double  a;

  extractNumber(field, data, startBit, bits, &value, &maxValue);

  /* There are max five reserved values according to ISO 11873-9 (that I gather from indirect sources)
   * but I don't yet know which datafields reserve the reserved values.
   */
#define DATAFIELD_UNKNOWN (0)
#define DATAFIELD_ERROR (-1)
#define DATAFIELD_RESERVED1 (-2)
#define DATAFIELD_RESERVED2 (-3)
#define DATAFIELD_RESERVED3 (-4)

  if (maxValue >= 15)
  {
    reserved = 2; /* DATAFIELD_ERROR and DATAFIELD_UNKNOWN */
  }
  else if (maxValue > 1)
  {
    reserved = 1; /* DATAFIELD_UNKNOWN */
  }
  else
  {
    reserved = 0;
  }

  if (fieldName[0] == '#')
  {
    logDebug("g_variableFieldRepeat[%d]=%d\n", g_variableFieldIndex, value);
    g_variableFieldRepeat[g_variableFieldIndex++] = value;
  }

  if (value <= maxValue - reserved)
  {
    if (field->units && field->units[0] == '=')
    {
      char  lookfor[20];
      char *s;

      sprintf(lookfor, "=%" PRId64, value);
      if (strcmp(lookfor, field->units) != 0)
      {
        if (showBytes)
          logError("Field %s value %" PRId64 " does not match %s\n", fieldName, value, field->units + 1);
        return false;
      }
      s = field->description;
      if (!s)
      {
        s = lookfor + 1;
      }
      if (showJson)
      {
        mprintf("%s\"%s\":\"%s\"", getSep(), fieldName, s);
      }
      else
      {
        mprintf("%s %s = %s", getSep(), fieldName, s);
      }
    }

    else if (field->resolution == RES_LOOKUP && field->units)
    {
      char  lookfor[20];
      char *s, *e;

      sprintf(lookfor, ",%" PRId64 "=", value);
      s = strstr(field->units, lookfor);
      if (s)
      {
        s += strlen(lookfor);
        e = strchr(s, ',');
        e = e ? e : s + strlen(s);
        if (showJsonValue)
        {
          mprintf("%s\"%s\":{\"value\":%" PRId64 ",\"name\":\"%.*s\"}", getSep(), fieldName, value, (int) (e - s), s);
        }
        else if (showJson)
        {
          mprintf("%s\"%s\":\"%.*s\"", getSep(), fieldName, (int) (e - s), s);
        }
        else
        {
          mprintf("%s %s = %.*s", getSep(), fieldName, (int) (e - s), s);
        }
      }
      else
      {
        if (showJson)
        {
          mprintf("%s\"%s\":\"%" PRId64 "\"", getSep(), fieldName, value);
        }
        else
        {
          mprintf("%s %s = %" PRId64 "", getSep(), fieldName, value);
        }
      }
    }

    else if (field->resolution == RES_BITFIELD && field->units)
    {
      char         lookfor[20];
      char *       s, *e;
      unsigned int bit;
      uint64_t     bitValue;
      char         sep;

      logDebug("RES_BITFIELD value %" PRIx64 "\n", value);
      if (showJson)
      {
        mprintf("%s\"%s\": ", getSep(), fieldName);
        sep = '[';
      }
      else
      {
        mprintf("%s %s =", getSep(), fieldName);
        sep = ' ';
      }

      for (bitValue = 1, bit = 0; bitValue <= maxValue; (bitValue *= 2), bit++)
      {
        logDebug("RES_BITFIELD is bit %u value %" PRIx64 " set %d\n", bit, bitValue, (value & value) >= 0);
        if ((value & bitValue) != 0)
        {
          sprintf(lookfor, ",%u=", bit);
          s = strstr(field->units, lookfor);
          if (s)
          {
            s += strlen(lookfor);
            e = strchr(s, ',');
            e = e ? e : s + strlen(s);
            if (showJson)
            {
              mprintf("%c\"%.*s\"", sep, (int) (e - s), s);
              sep = ',';
            }
            else
            {
              mprintf("%c%.*s", sep, (int) (e - s), s);
              sep = ',';
            }
          }
          else
          {
            mprintf("%c\"%" PRIu64 "\"", sep, bitValue);
            sep = ',';
          }
        }
      }
      if (showJson)
      {
        if (sep != '[')
        {
          mprintf("]");
        }
        else
        {
          mprintf("[]");
        }
      }
    }

    else if (field->resolution == RES_BINARY)
    {
      if (showJson)
      {
        mprintf("%s\"%s\":\"%" PRId64 "\"", getSep(), fieldName, value);
      }
      else
      {
        mprintf("%s %s = 0x%" PRIx64, getSep(), fieldName, value);
      }
    }
    else if (field->resolution == RES_MANUFACTURER)
    {
      char *m = 0;
      char  unknownManufacturer[30];

      if (value > 0 && value < ARRAY_SIZE(manufacturer))
      {
        m = manufacturer[value];
      }
      if (!m)
      {
        if (showJson)
        {
          mprintf("%s \"%s\":%" PRId64, getSep(), fieldName, value);
          return true;
        }
        sprintf(unknownManufacturer, "Unknown Manufacturer %" PRId64, value);
        m = unknownManufacturer;
      }

      if (showJsonValue)
      {
        mprintf("%s \"%s\":{\"value\":%" PRId64 ",\"name\":\"%s\"}", getSep(), fieldName, value, m);
      }
      else if (showJson)
      {
        mprintf("%s \"%s\": \"%s\"", getSep(), fieldName, m);
      }
      else
      {
        mprintf("%s %s = %s", getSep(), fieldName, m);
      }
    }
    else
    {
      if (field->resolution == RES_INTEGER)
      {
        if (showJson)
        {
          mprintf("%s\"%s\":%" PRId64 "", getSep(), fieldName, value);
        }
        else
        {
          mprintf("%s %s = %" PRId64, getSep(), fieldName, value);
        }
      }
      else
      {
        int         precision;
        double      r;
        const char *units = field->units;

        a = (double) value * field->resolution;

        precision = 0;
        for (r = field->resolution; (r > 0.0) && (r < 1.0); r *= 10.0)
        {
          precision++;
        }

        if (field->resolution == RES_RADIANS)
        {
          units = "rad";
          if (!showSI)
          {
            a *= RadianToDegree;
            precision -= 3;
            units = "deg";
          }
        }
        else if (field->resolution == RES_ROTATION || field->resolution == RES_HIRES_ROTATION)
        {
          units = "rad/s";
          if (!showSI)
          {
            a *= RadianToDegree;
            precision -= 3;
            units = "deg/s";
          }
        }
        else if (units && showSI)
        {
          if (strcmp(units, "kWh") == 0)
          {
            a *= 3.6e6; // 1 kWh = 3.6 MJ.
          }
          else if (strcmp(units, "Ah") == 0)
          {
            a *= 3600.0; // 1 Ah = 3600 C.
          }

          // Many more to follow, but pgn.h is not yet complete enough...
        }
        else if (units && !showSI)
        {
          if (strcmp(units, "C") == 0)
          {
            a /= 3600.0; // 3600 C = 1 Ah
            units = "Ah";
          }
        }

        if (showJson)
        {
          mprintf("%s\"%s\":%.*f", getSep(), fieldName, precision, a);
        }
        else if (units && strcmp(units, "m") == 0 && a >= 1000.0)
        {
          mprintf("%s %s = %.*f km", getSep(), fieldName, precision + 3, a / 1000);
        }
        else
        {
          mprintf("%s %s = %.*f", getSep(), fieldName, precision, a);
          if (units)
          {
            mprintf(" %s", units);
          }
        }
      }
    }
  }
  else
  {
    /* For json, which is supposed to be effective for machine operations
     * we just ignore the special values.
     */
    if (!showJson)
    {
      switch (value - maxValue)
      {
        case DATAFIELD_UNKNOWN:
          mprintf("%s %s = Unknown", getSep(), fieldName);
          break;
        case DATAFIELD_ERROR:
          mprintf("%s %s = ERROR", getSep(), fieldName);
          break;
        case DATAFIELD_RESERVED1:
          mprintf("%s %s = RESERVED1", getSep(), fieldName);
          break;
        case DATAFIELD_RESERVED2:
          mprintf("%s %s = RESERVED2", getSep(), fieldName);
          break;
        case DATAFIELD_RESERVED3:
          mprintf("%s %s = RESERVED3", getSep(), fieldName);
          break;
        default:
          mprintf("%s %s = Unhandled value %ld max %ld (%ld)", getSep(), fieldName, value, maxValue, value - maxValue);
      }
    }
  }

  return true;
}

void setSystemClock(uint16_t currentDate, uint32_t currentTime)
{
#ifndef SKIP_SETSYSTEMCLOCK
  static uint16_t prevDate        = UINT16_MAX;
  static uint32_t prevTime        = UINT32_MAX;
  const uint32_t  unitspersecond  = 10000;
  const uint32_t  microsperunit   = 100;
  const uint32_t  microspersecond = 1000000;
  const uint32_t  secondsperday   = 86400;
  struct timeval  now;
  struct timeval  gps;
  struct timeval  delta;
  struct timeval  olddelta;

#ifdef HAS_ADJTIME
  const int maxDelta = 30;
#else
  const int maxDelta = 1;
#endif

  logDebug("setSystemClock = %u/%u\n", currentDate, currentTime);

  if (prevDate == UINT16_MAX)
  {
    logDebug("setSystemClock: first time\n");
    prevDate = currentDate;
    prevTime = currentTime;
    return;
  }
  if (prevTime == currentTime && prevDate == currentDate)
  {
    logDebug("System clock not changed\n");
    return;
  }

  if (gettimeofday(&now, 0))
  {
    logError("Can't get system clock\n");
    return;
  }

  gps.tv_sec  = currentDate * secondsperday + currentTime / unitspersecond;
  gps.tv_usec = (currentTime % unitspersecond) * microsperunit;

  if (gps.tv_sec < now.tv_sec - maxDelta || gps.tv_sec > now.tv_sec + maxDelta)
  {
    if (settimeofday(&gps, 0))
    {
      logError("Failed to adjust system clock to %" PRIu64 "/%06u\n", (uint64_t) gps.tv_sec, gps.tv_usec);
      return;
    }
    if (showBytes)
    {
      logInfo("Set system clock to %" PRIu64 "/%06u\n", (uint64_t) gps.tv_sec, gps.tv_usec);
    }
    return;
  }

#ifdef HAS_ADJTIME

  delta.tv_sec  = 0;
  delta.tv_usec = gps.tv_usec - now.tv_usec + microspersecond * (gps.tv_sec - now.tv_sec);

  if (delta.tv_usec < 2000 && delta.tv_usec > -2000)
  {
    if (showBytes)
    {
      logDebug("Forget about small system clock skew %d\n", delta.tv_usec);
    }
    return;
  }

  if (adjtime(&delta, &olddelta))
  {
    logError("Failed to adjust system clock by %d usec\n", delta.tv_usec);
    return;
  }

  if (showBytes)
  {
    logDebug("Now = %" PRIu64 "/%06u ", (uint64_t) now.tv_sec, now.tv_usec);
    logDebug("GPS = %" PRIu64 "/%06u ", (uint64_t) gps.tv_sec, gps.tv_usec);
    logDebug("Adjusting system clock by %d usec\n", delta.tv_usec);
    if (olddelta.tv_sec || olddelta.tv_usec)
    {
      logDebug("(Old delta not yet completed %" PRIu64 "/%d\n", (uint64_t) olddelta.tv_sec, olddelta.tv_usec);
    }
  }

#endif
#endif
}

static void print_ascii_json_escaped(uint8_t *data, int len)
{
  int c;
  int k;

  for (k = 0; k < len; k++)
  {
    c = data[k];
    switch (c)
    {
      case '\b':
        mprintf("%s", "\\b");
        break;

      case '\n':
        mprintf("%s", "\\n");
        break;

      case '\r':
        mprintf("%s", "\\r");
        break;

      case '\t':
        mprintf("%s", "\\t");
        break;

      case '\f':
        mprintf("%s", "\\f");
        break;

      case '"':
        mprintf("%s", "\\\"");
        break;

      case '\\':
        mprintf("%s", "\\\\");
        break;

      case '/':
        mprintf("%s", "\\/");
        break;

      case '\377':
        // 0xff has been seen on recent Simrad VHF systems, and it seems to indicate
        // end-of-field, with noise following. Assume this does not break other systems.
        return;

      default:
        if (c >= ' ' && c <= '~')
          mprintf("%c", c);
    }
  }
}

void printPacket(size_t index, size_t unknownIndex, RawMessage *msg)
{
  size_t  fastPacketIndex;
  size_t  bucket;
  Packet *packet;
  Pgn *   pgn = &pgnList[index];
  size_t  subIndex;

  if (!device[msg->src])
  {
    heapSize += sizeof(DevicePackets);
    logDebug("New device at address %u (heap %zu bytes)\n", msg->src, heapSize);
    device[msg->src] = calloc(1, sizeof(DevicePackets));
    if (!device[msg->src])
    {
      die("Out of memory\n");
    }
  }
  packet = &(device[msg->src]->packetList[index]);

  if (!packet->data)
  {
    packet->allocSize = max(max(pgn->size, 8) + FASTPACKET_BUCKET_N_SIZE, msg->len);
    heapSize += packet->allocSize;
    logInfo("New PGN %u for device %u (heap %zu bytes)\n", pgn->pgn, msg->src, heapSize);
    packet->data = malloc(packet->allocSize);
    if (!packet->data)
    {
      die("Out of memory\n");
    }
  }

  if (msg->len > 0x8 || multiPackets == MULTIPACKETS_COALESCED)
  {
    if (packet->allocSize < msg->len)
    {
      heapSize += msg->len - packet->allocSize;
      logDebug(
          "Resizing buffer for PGN %u device %u to accommodate %u bytes (heap %zu bytes)\n", pgn->pgn, msg->src, msg->len, heapSize);
      packet->data = realloc(packet->data, msg->len);
      if (!packet->data)
      {
        die("Out of memory\n");
      }
      packet->allocSize = msg->len;
    }
    memcpy(packet->data, msg->data, msg->len);
    packet->size = msg->len;
  }
  else if (pgn->type == PACKET_FAST)
  {
    fastPacketIndex = msg->data[FASTPACKET_INDEX];
    bucket          = fastPacketIndex & FASTPACKET_MAX_INDEX;

    if (bucket == 0)
    {
      size_t newSize = ((size_t) msg->data[FASTPACKET_SIZE]) + FASTPACKET_BUCKET_N_SIZE;

      if (packet->allocSize < newSize)
      {
        heapSize += newSize - packet->allocSize;
        logDebug("Resizing buffer for PGN %u device %u to accommodate %zu bytes (heap %zu bytes)\n",
                 pgn->pgn,
                 msg->src,
                 newSize,
                 heapSize);
        packet->data = realloc(packet->data, newSize);
        if (!packet->data)
        {
          die("Out of memory\n");
        }
        packet->allocSize = newSize;
      }
      packet->size = msg->data[FASTPACKET_SIZE];
      memcpy(packet->data, msg->data + FASTPACKET_BUCKET_0_OFFSET, FASTPACKET_BUCKET_0_SIZE);
    }
    else
    {
      if (packet->lastFastPacket + 1 != fastPacketIndex)
      {
        logError("PGN %u malformed packet from %u received; expected %zu but got %zu\n",
                 pgn->pgn,
                 msg->src,
                 packet->lastFastPacket + 1,
                 fastPacketIndex);
        return;
      }
      memcpy(packet->data + FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * (bucket - 1),
             msg->data + FASTPACKET_BUCKET_N_OFFSET,
             FASTPACKET_BUCKET_N_SIZE);
    }
    packet->lastFastPacket = fastPacketIndex;

    if (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * bucket < packet->size)
    {
      /* Packet is not complete yet */
      return;
    }
  }
  else
  {
    packet->size = msg->len;
    memcpy(packet->data, msg->data, msg->len);
  }

  printPgn(msg, packet->data, packet->size, showData, showJson);
}

static bool printCanFormat(RawMessage *msg)
{
  size_t i;
  size_t unknownIndex = 0;

  if (onlySrc >= 0 && onlySrc != msg->src)
  {
    return false;
  }

  for (i = 0; i < ARRAY_SIZE(pgnList); i++)
  {
    if (msg->pgn == pgnList[i].pgn)
    {
      if (onlyPgn)
      {
        if (msg->pgn == onlyPgn)
        {
          printPacket(i, unknownIndex, msg);
          return true;
        }
        continue;
      }
      if (!pgnList[i].size)
      {
        return true; /* We have field names, but no field sizes. */
      }
      /*
       * Found the pgn that matches this particular packet
       */
      printPacket(i, unknownIndex, msg);
      return true;
    }
    else if (msg->pgn < pgnList[i].pgn)
    {
      break;
    }
    if (pgnList[i].unknownPgn)
    {
      unknownIndex = i;
    }
  }
  if (!onlyPgn)
  {
    printPacket(unknownIndex, unknownIndex, msg);
  }
  return onlyPgn > 0;
}

static void explainPGN(Pgn pgn)
{
  int i;

  printf("PGN: %d / %08o / %05X - %u - %s\n\n", pgn.pgn, pgn.pgn, pgn.pgn, pgn.size, pgn.description);

  if (pgn.repeatingFields >= 100)
  {
    printf("     The last %u and %u fields repeat until the data is exhausted.\n\n",
           pgn.repeatingFields % 100,
           pgn.repeatingFields / 100);
  }
  else if (pgn.repeatingFields)
  {
    printf("     The last %u fields repeat until the data is exhausted.\n\n", pgn.repeatingFields);
  }
  for (i = 0; i < ARRAY_SIZE(pgn.fieldList) && pgn.fieldList[i].name; i++)
  {
    Field f = pgn.fieldList[i];
    printf("  Field #%d: %s%s%s\n",
           i + 1,
           f.name,
           f.name[0] && (f.description && f.description[0] && f.description[0] != ',') ? " - " : "",
           (!f.description || f.description[0] == ',') ? "" : f.description);
    if (!f.size)
    {
      printf("                  Bits: variable\n");
    }
    else
    {
      printf("                  Bits: %u\n", f.size);
    }

    if (f.units && f.units[0] == '=')
    {
      printf("                  Match: %s\n", &f.units[1]);
    }
    else if (f.units && f.units[0] != ',')
    {
      printf("                  Units: %s\n", f.units);
    }

    if (f.resolution < 0.0)
    {
      Resolution t = types[-1 * (int) f.resolution - 1];
      if (t.name)
      {
        printf("                  Type: %s\n", t.name);
      }
      if (t.resolution)
      {
        printf("                  Resolution: %s\n", t.resolution);
      }
      else if (f.resolution == RES_LATITUDE || f.resolution == RES_LONGITUDE)
      {
        if (f.size == BYTES(8))
        {
          printf("                  Resolution: %.16f\n", 1e-16);
        }
        else
        {
          printf("                  Resolution: %.7f\n", 1e-7);
        }
      }
    }
    else if (f.resolution != 1.0)
    {
      printf("                  Resolution: %g\n", f.resolution);
    }
    printf("                  Signed: %s\n", (f.hasSign) ? "true" : "false");
    if (f.offset != 0)
    {
      printf("                  Offset: %d\n", f.offset);
    }

    if ((f.resolution == RES_LOOKUP || f.resolution == RES_BITFIELD) && f.units && f.units[0] == ',')
    {
      char *s, *e;

      for (s = f.units + 1;; s = e + 1)
      {
        e = strchr(s, ',');
        e = e ? e : s + strlen(s);
        printf("                  Lookup: %.*s\n", (int) (e - s), s);
        if (!*e)
        {
          break;
        }
      }
    }
  }

  printf("\n\n");
}

/*
 * Print string but replace special characters by their XML entity.
 */
static void printXML(int indent, const char *element, const char *p)
{
  int i;

  if (p)
  {
    for (i = 0; i < indent; i++)
    {
      fputs(" ", stdout);
    }
    printf("<%s>", element);
    for (; *p; p++)
    {
      switch (*p)
      {
        case '&':
          fputs("&amp;", stdout);
          break;

        case '<':
          fputs("&lt;", stdout);
          break;

        case '>':
          fputs("&gt;", stdout);
          break;

        case '"':
          fputs("&quot;", stdout);
          break;

        default:
          putchar(*p);
      }
    }
    printf("</%s>\n", element);
  }
}

static void explainPGNXML(Pgn pgn)
{
  int      i;
  unsigned bitOffset = 0;
  char *   p;
  bool     showBitOffset = true;

  printf("    <PGNInfo>\n"
         "      <PGN>%u</PGN>\n",
         pgn.pgn);
  printXML(6, "Id", pgn.camelDescription);
  printXML(6, "Description", pgn.description);
  printXML(6, "Type", (pgn.type == PACKET_ISO11783 ? "ISO" : (pgn.type == PACKET_FAST ? "Fast" : "Single")));
  printf("      <Complete>%s</Complete>\n", (pgn.complete == PACKET_COMPLETE ? "true" : "false"));

  if (pgn.complete != PACKET_COMPLETE)
  {
    printf("      <Missing>\n");

    if ((pgn.complete & PACKET_FIELDS_UNKNOWN) != 0)
    {
      printXML(8, "MissingAttribute", "Fields");
    }
    if ((pgn.complete & PACKET_FIELD_LENGTHS_UNKNOWN) != 0)
    {
      printXML(8, "MissingAttribute", "FieldLengths");
    }
    if ((pgn.complete & PACKET_PRECISION_UNKNOWN) != 0)
    {
      printXML(8, "MissingAttribute", "Precision");
    }
    if ((pgn.complete & PACKET_LOOKUPS_UNKNOWN) != 0)
    {
      printXML(8, "MissingAttribute", "Lookups");
    }
    if ((pgn.complete & PACKET_NOT_SEEN) != 0)
    {
      printXML(8, "MissingAttribute", "SampleData");
    }

    printf("      </Missing>\n");
  }

  printf("      <Length>%u</Length>\n", pgn.size);

  if (pgn.repeatingFields >= 100)
  {
    printf("      <RepeatingFieldSet1>%u</RepeatingFieldSet1>\n", pgn.repeatingFields % 100);
    printf("      <RepeatingFieldSet2>%u</RepeatingFieldSet2>\n", pgn.repeatingFields / 100);
  }
  else
  {
    printf("      <RepeatingFields>%u</RepeatingFields>\n", pgn.repeatingFields);
  }

  if (pgn.fieldList[0].name)
  {
    printf("      <Fields>\n");

    for (i = 0; i < ARRAY_SIZE(pgn.fieldList) && pgn.fieldList[i].name; i++)
    {
      Field f = pgn.fieldList[i];

      printf("        <Field>\n"
             "          <Order>%d</Order>\n",
             i + 1);
      printXML(10, "Id", f.camelName);
      printXML(10, "Name", f.name);

      if (f.description && f.description[0] && f.description[0] != ',')
      {
        printXML(10, "Description", f.description);
      }
      printf("          <BitLength>%u</BitLength>\n", f.size);
      if (showBitOffset)
      {
        printf("          <BitOffset>%u</BitOffset>\n", bitOffset);
      }
      printf("          <BitStart>%u</BitStart>\n", bitOffset % 8);
      bitOffset = bitOffset + f.size;

      if (f.units && f.units[0] == '=')
      {
        printf("          <Match>%s</Match>\n", &f.units[1]);
      }
      else if (f.units && f.units[0] != ',')
      {
        printf("          <Units>%s</Units>\n", f.units);
      }

      if (f.resolution < 0.0)
      {
        Resolution t = types[-1 * (int) f.resolution - 1];
        if (t.name)
        {
          printf("                 <Type>%s</Type>\n", t.name);
        }
        if (t.resolution)
        {
          printf("                 <Resolution>%s</Resolution>\n", t.resolution);
        }
        else if (f.resolution == RES_LATITUDE || f.resolution == RES_LONGITUDE)
        {
          if (f.size == BYTES(8))
          {
            printf("                 <Resolution>%.16f</Resolution>\n", 1e-16);
          }
          else
          {
            printf("                 <Resolution>%.7f</Resolution>\n", 1e-7);
          }
        }
      }
      else if (f.resolution != 1.0)
      {
        printf("          <Resolution>%g</Resolution>\n", f.resolution);
      }
      printf("          <Signed>%s</Signed>\n", f.hasSign ? "true" : "false");
      if (f.offset != 0)
      {
        printf("          <Offset>%d</Offset>\n", f.offset);
      }

      if (f.resolution == RES_LOOKUP && f.units && f.units[0] == ',')
      {
        char *s, *e, *p;

        printf("          <EnumValues>\n");

        for (s = f.units + 1;; s = e + 1)
        {
          e = strchr(s, ',');
          e = e ? e : s + strlen(s);
          p = strchr(s, '=');
          if (p)
          {
            printf("            <EnumPair Value='%.*s' Name='%.*s' />\n", (int) (p - s), s, (int) (e - (p + 1)), p + 1);
          }
          if (!*e)
          {
            break;
          }
        }

        printf("          </EnumValues>\n");
      }

      if (f.resolution == RES_BITFIELD && f.units && f.units[0] == ',')
      {
        char *s, *e, *p;

        printf("          <EnumBitValues>\n");

        for (s = f.units + 1;; s = e + 1)
        {
          e = strchr(s, ',');
          e = e ? e : s + strlen(s);
          p = strchr(s, '=');
          if (p)
          {
            printf("            <EnumPair Bit='%.*s' Name='%.*s' />\n", (int) (p - s), s, (int) (e - (p + 1)), p + 1);
          }
          if (!*e)
          {
            break;
          }
        }

        printf("          </EnumBitValues>\n");
      }

      if (f.resolution == RES_STRINGLZ || f.resolution == RES_STRINGLAU)
      {
        showBitOffset = false; // From here on there is no good bitoffset to be printed
      }
      printf("        </Field>\n");
    }
    printf("      </Fields>\n");
  }
  printf("    </PGNInfo>\n");
}

static void explain(void)
{
  int i;

  printf(COPYRIGHT "\n\nThis program can understand a number of N2K messages. What follows is an explanation of the messages\n"
                   "that it understands. First is a list of completely understood messages, as far as I can tell.\n"
                   "What follows is a list of messages that contain fields that have unknown content or size, or even\n"
                   "completely unknown fields. If you happen to know more, please tell me!\n\n");
  printf("_______ Complete PGNs _________\n\n");
  for (i = 1; i < ARRAY_SIZE(pgnList); i++)
  {
    if (pgnList[i].complete == PACKET_COMPLETE && pgnList[i].pgn < ACTISENSE_BEM)
    {
      explainPGN(pgnList[i]);
    }
  }
  printf("_______ Incomplete PGNs _________\n\n");
  for (i = 1; i < ARRAY_SIZE(pgnList); i++)
  {
    if (pgnList[i].complete != PACKET_COMPLETE && pgnList[i].pgn < ACTISENSE_BEM)
    {
      explainPGN(pgnList[i]);
    }
  }
}

char *camelize(const char *str, bool upperCamelCase)
{
  size_t len         = strlen(str);
  char * ptr         = malloc(len + 1);
  char * s           = ptr;
  bool   lastIsAlpha = !upperCamelCase;

  if (!s)
  {
    return 0;
  }

  for (s = ptr; *str; str++)
  {
    if (isalpha(*str) || isdigit(*str))
    {
      if (lastIsAlpha)
      {
        *s = tolower(*str);
      }
      else
      {
        *s          = toupper(*str);
        lastIsAlpha = true;
      }
      s++;
    }
    else
    {
      lastIsAlpha = false;
    }
  }

  *s = 0;
  return ptr;
}

static void camelCase(bool upperCamelCase)
{
  int i, j;

  for (i = 0; i < ARRAY_SIZE(pgnList); i++)
  {
    pgnList[i].camelDescription = camelize(pgnList[i].description, upperCamelCase);
    for (j = 0; j < ARRAY_SIZE(pgnList[i].fieldList) && pgnList[i].fieldList[j].name; j++)
    {
      pgnList[i].fieldList[j].camelName = camelize(pgnList[i].fieldList[j].name, upperCamelCase);
    }
  }
}

static void explainXML(void)
{
  int i;

  printf("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
         "<!--\n" COPYRIGHT "\n-->\n"
         "<PGNDefinitions xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\" "
         "Version=\"0.1\">\n"
         "  <Comment>See https://github.com/canboat/canboat for the full source code</Comment>\n"
         "  <CreatorCode>Canboat NMEA2000 Analyzer</CreatorCode>\n"
         "  <License>GPL v3</License>\n"
         "  <Version>" VERSION "</Version>\n"
         "  <PGNs>\n");

  for (i = 1; i < ARRAY_SIZE(pgnList); i++)
  {
    if (pgnList[i].pgn < ACTISENSE_BEM)
    {
      explainPGNXML(pgnList[i]);
    }
  }

  printf("  </PGNs>\n"
         "</PGNDefinitions>\n");
}

bool printPgn(RawMessage *msg, uint8_t *dataStart, int length, bool showData, bool showJson)
{
  Pgn *pgn;

  uint8_t *data;

  uint8_t *dataEnd = dataStart + length;
  size_t   i;
  Field    field;
  size_t   bits;
  size_t   bytes;
  size_t   startBit;
  int      repetition = 0;
  uint16_t valueu16;
  uint32_t valueu32;
  uint16_t currentDate = UINT16_MAX;
  uint32_t currentTime = UINT32_MAX;
  char     fieldName[60];
  bool     r;
  uint32_t refPgn = 0;
  uint32_t variableFieldCount[2]; // How many variable fields over all repetitions, indexed by group
  uint32_t variableFields[2];     // How many variable fields per repetition, indexed by group
  size_t   variableFieldStart;

  if (!msg)
  {
    return false;
  }
  pgn = getMatchingPgn(msg->pgn, dataStart, length);
  if (!pgn)
  {
    pgn = pgnList;
  }

  if (showData)
  {
    FILE *f = stdout;
    char  c = ' ';

    if (showJson)
    {
      f = stderr;
    }

    fprintf(f, "%s %u %3u %3u %6u %s: ", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    for (i = 0; i < length; i++)
    {
      fprintf(f, " %2.02X", dataStart[i]);
    }
    putc('\n', f);

    fprintf(f, "%s %u %3u %3u %6u %s: ", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    for (i = 0; i < length; i++)
    {
      fprintf(f, "  %c", isalnum(dataStart[i]) ? dataStart[i] : '.');
    }
    putc('\n', f);
  }
  if (showJson)
  {
    if (pgn->camelDescription)
    {
      mprintf("\"%s\":", pgn->camelDescription);
    }
    mprintf("{\"timestamp\":\"%s\",\"prio\":%u,\"src\":%u,\"dst\":%u,\"pgn\":%u,\"description\":\"%s\"",
            msg->timestamp,
            msg->prio,
            msg->src,
            msg->dst,
            msg->pgn,
            pgn->description);
    strcpy(closingBraces, "}");
    sep = ",\"fields\":{";
  }
  else
  {
    mprintf("%s %u %3u %3u %6u %s:", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    sep = " ";
  }

  g_variableFieldRepeat[0] = 255; // Can be overridden by '# of parameters'
  g_variableFieldRepeat[1] = 0;   // Can be overridden by '# of parameters'
  g_variableFieldIndex     = 0;

  if (pgn->repeatingFields >= 100)
  {
    variableFieldCount[0] = pgn->repeatingFields % 100;
    variableFieldCount[1] = pgn->repeatingFields / 100;
  }
  else
  {
    variableFieldCount[0] = pgn->repeatingFields % 100;
    variableFieldCount[1] = 0;
  }

  variableFieldStart = pgn->fieldCount - variableFieldCount[0] - variableFieldCount[1];
  logDebug("fieldCount=%d variableFieldStart=%d\n", pgn->fieldCount, variableFieldStart);

  for (i = 0, startBit = 0, data = dataStart; data < dataEnd; i++)
  {
    Field *field;
    r = true;

    if (variableFieldCount[0] && i == variableFieldStart && repetition == 0)
    {
      repetition = 1;
      if (showJson)
      {
        mprintf("%s\"list\":[{", getSep());
        strcat(closingBraces, "]}");
        sep = "";
      }
      // Only now is g_variableFieldRepeat set via values for parameters starting with '# of ...'
      variableFields[0] = variableFieldCount[0] * g_variableFieldRepeat[0];
      variableFields[1] = variableFieldCount[1] * g_variableFieldRepeat[1];
    }
    if (repetition > 0)
    {
      if (variableFields[0])
      {
        if (showBytes)
        {
          mprintf("\ni=%d fs=%d vf=%d", i, variableFieldStart, variableFields[0]);
        }
        if (i == variableFieldStart + variableFieldCount[0])
        {
          i = variableFieldStart;
          repetition++;
          if (showJson)
          {
            mprintf("},{");
            sep = "";
          }
        }
        variableFields[0]--;
        if (variableFields[0] == 0)
        {
          variableFieldStart += variableFieldCount[0];
        }
      }
      else if (variableFields[1])
      {
        if (variableFields[1] == variableFieldCount[1] * g_variableFieldRepeat[1])
        {
          repetition = 0;
        }
        if (i == variableFieldStart + variableFieldCount[1])
        {
          i = variableFieldStart;
          repetition++;
          if (showJson)
          {
            mprintf("},{");
            sep = "";
          }
        }
        variableFields[1]--;
      }
      else
      {
        break;
      }
    }

    field = &pgn->fieldList[i];

    if (!field->camelName && !field->name)
    {
      logDebug("PGN %u has unknown bytes at end: %u\n", msg->pgn, dataEnd - data);
      break;
    }

    strcpy(fieldName, field->camelName ? field->camelName : field->name);
    if (repetition >= 1 && !showJson)
    {
      strcat(fieldName, field->camelName ? "_" : " ");
      sprintf(fieldName + strlen(fieldName), "%u", repetition);
    }

    bits  = field->size;
    bytes = (bits + 7) / 8;
    bytes = min(bytes, (size_t)(dataEnd - data));
    bits  = min(bytes * 8, bits);

    if (showBytes)
    {
      mprintf("\ndecode %s offset=%u startBit=%u bits=%u bytes=%u:", field->name, data - dataStart, startBit, bits, bytes);
    }

    if (strcmp(fieldName, "PGN") == 0)
    {
      refPgn = data[0] + (data[1] << 8) + (data[2] << 16);
      if (showBytes)
      {
        mprintf("refPgn=%u ", refPgn);
      }
    }

    if (strcmp(fieldName, "Reserved") == 0)
    {
      // Skipping reserved fields. Unfortunately we have some cases now
      // where they are zero. Some AIS devices (SRT) produce an 8 bit long
      // nav status, others just a four bit one.
    }
    else if (field->resolution < 0.0)
    {
      int len;
      int k;

      /* These fields have only been found to start on byte boundaries,
       * making their location easier
       */
      if (field->resolution == RES_STRINGLZ)
      {
        len = *data++;
        bytes--;
        goto ascii_string;
      }

      if (field->resolution == RES_STRINGLAU)
      {
        int control;

        len = *data++;
        bytes--;
        control = *data++;
        bytes--;
        if (control == 0)
        {
          logError("Unhandled UNICODE string in PGN\n");
        }
        goto ascii_string;
      }

      if (field->resolution == RES_ASCII)
      {
        len                    = (int) bytes;
        unsigned char lastbyte = data[len - 1];

        if (lastbyte == 0xff || lastbyte == ' ' || lastbyte == 0 || lastbyte == '@')
        {
          while (len > 0 && (data[len - 1] == lastbyte))
          {
            len--;
          }
        }

      ascii_string:
        if (showBytes)
        {
          for (k = 0; k < len; k++)
          {
            mprintf("%02x ", data[k]);
          }
        }

        if (showJson)
        {
          mprintf("%s\"%s\":\"", getSep(), fieldName);
        }
        else
        {
          mprintf("%s %s = ", getSep(), fieldName);
        }

        if (showJson)
        {
          print_ascii_json_escaped(data, len);
        }
        else
        {
          for (k = 0; k < len; k++)
          {
            if (data[k] == 0xff)
            {
              break;
            }
            if (data[k] >= ' ' && data[k] <= '~')
            {
              mprintf("%c", data[k]);
            }
          }
        }

        if (showJson)
        {
          mprintf("\"");
        }
      }
      else if (field->resolution == RES_STRING)
      {
        int len;
        if (*data == 0x02)
        {
          data++;
          for (len = 0; data + len < dataEnd && data[len] != 0x01; len++)
            ;
          bytes = len + 1;
        }
        else if (*data > 0x02)
        {
          bytes = *data++;
          bytes--; /* Compensate for that we've already increased data by 1 */
          if (*data == 0x01)
          {
            data++;
            bytes--;
          }
          len = bytes - 1;
        }
        else
        {
          bytes = 1;
          len   = 0;
        }
        if (len)
        {
          if (showJson)
          {
            mprintf("%s\"%s\":\"%.*s\"", getSep(), fieldName, (int) len, data);
          }
          else
          {
            mprintf("%s %s = %.*s", getSep(), fieldName, (int) len, data);
          }
        }
        bits = BYTES(bytes);
      }
      else if (field->resolution == RES_LONGITUDE || field->resolution == RES_LATITUDE)
      {
        printLatLon(fieldName, field->resolution, data, bytes);
      }
      else if (field->resolution == RES_DATE)
      {
        valueu16 = data[0] + (data[1] << 8);
        printDate(fieldName, valueu16);
        currentDate = valueu16;
      }
      else if (field->resolution == RES_TIME)
      {
        valueu32 = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
        printTime(fieldName, valueu32);
        currentTime = valueu32;
      }
      else if (field->resolution == RES_PRESSURE)
      {
        valueu32 = data[0] + (data[1] << 8);
        printPressure(fieldName, valueu32, field);
      }
      else if (field->resolution == RES_PRESSURE_HIRES)
      {
        valueu32 = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
        printPressure(fieldName, valueu32, field);
      }
      else if (field->resolution == RES_TEMPERATURE)
      {
        valueu32 = data[0] + (data[1] << 8);
        printTemperature(fieldName, valueu32, 16, 0.01);
      }
      else if (field->resolution == RES_TEMPERATURE_HIGH)
      {
        valueu32 = data[0] + (data[1] << 8);
        printTemperature(fieldName, valueu32, 16, 0.1);
      }
      else if (field->resolution == RES_TEMPERATURE_HIRES)
      {
        valueu32 = data[0] + (data[1] << 8) + (data[2] << 16);
        printTemperature(fieldName, valueu32, 24, 0.001);
      }
      else if (field->resolution == RES_6BITASCII)
      {
        print6BitASCIIText(fieldName, data, startBit, bits);
      }
      else if (field->resolution == RES_DECIMAL)
      {
        printDecimal(fieldName, data, startBit, bits);
      }
      else if (bits == LEN_VARIABLE)
      {
        printVarNumber(fieldName, pgn, refPgn, field, data, startBit, &bits);
      }
      else if (bits > BYTES(8))
      {
        printHex(fieldName, data, startBit, bits);
      }
      else if (field->resolution == RES_INTEGER || field->resolution == RES_LOOKUP || field->resolution == RES_BITFIELD
               || field->resolution == RES_BINARY || field->resolution == RES_MANUFACTURER)
      {
        printNumber(fieldName, field, data, startBit, bits);
      }
      else
      {
        logError("Unknown resolution %f for %s\n", field->resolution, fieldName);
      }
    }
    else if (field->resolution > 0.0)
    {
      printNumber(fieldName, field, data, startBit, bits);
    }
    if (!r)
    {
      return false;
    }

    startBit += bits;
    data += startBit / 8;
    startBit %= 8;
  }

  if (showJson)
  {
    for (i = strlen(closingBraces); i;)
    {
      mprintf("%c", closingBraces[--i]);
    }
  }
  mprintf("\n");

  if (r)
  {
    mwrite(stdout);
  }
  else
  {
    mreset();
  }

  if (msg->pgn == 126992 && currentDate < UINT16_MAX && currentTime < UINT32_MAX && clockSrc == msg->src)
  {
    setSystemClock(currentDate, currentTime);
  }
  return r;
}
