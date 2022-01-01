/*

Analyzes NMEA 2000 PGNs.

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

#define GLOBALS
#include "analyzer.h"

#include "parse.h"

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

/* There are max five reserved values according to ISO 11873-9 (that I gather from indirect sources)
 * but I don't yet know which datafields reserve the reserved values.
 */
#define DATAFIELD_UNKNOWN (0)
#define DATAFIELD_ERROR (-1)
#define DATAFIELD_RESERVED1 (-2)
#define DATAFIELD_RESERVED2 (-3)
#define DATAFIELD_RESERVED3 (-4)

DevicePackets *device[256];

bool            showRaw       = false;
bool            showData      = false;
bool            showBytes     = false;
bool            showJson      = false;
bool            showJsonEmpty = false;
bool            showJsonValue = false;
bool            showSI        = false; // Output everything in strict SI units
char           *sep           = " ";
char            closingBraces[8]; // } and ] chars to close sentence in JSON mode, otherwise empty string
enum GeoFormats showGeo  = GEO_DD;
int             onlyPgn  = 0;
int             onlySrc  = -1;
int             clockSrc = -1;
size_t          heapSize = 0;

int g_variableFieldRepeat[2]; // Actual number of repetitions
int g_variableFieldIndex;

static uint16_t currentDate = UINT16_MAX;
static uint32_t currentTime = UINT32_MAX;

static enum RawFormats detectFormat(const char *msg);
static bool            printCanFormat(RawMessage *msg);
static bool            printNumber(char *fieldName, Field *field, uint8_t *data, size_t startBit, size_t bits);
static bool            printField(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
static void            fillFieldCounts(void);
static void            printCanRaw(RawMessage *msg);

static void usage(char **argv, char **av)
{
  printf("Unknown or invalid argument %s\n", av[0]);
  printf("Usage: %s [[-raw] [-json [-empty] [-nv] [-camel | -upper-camel]] [-data] [-debug] [-d] [-q] [-si] [-geo {dd|dm|dms}] "
         "[-src <src> | <pgn>]] ["
#ifndef SKIP_SETSYSTEMCLOCK
         "-clocksrc <src> | "
#endif
         "-version\n",
         argv[0]);
  printf("     -json             Output in json format, for program consumption. Empty values are skipped\n");
  printf("     -empty            Modified json format where empty values are shown as NULL\n");
  printf("     -nv               Modified json format where lookup values are shown as name, value pair\n");
  printf("     -camel            Show fieldnames in normalCamelCase\n");
  printf("     -upper-camel      Show fieldnames in UpperCamelCase\n");
  printf("     -d                Print logging from level ERROR, INFO and DEBUG\n");
  printf("     -q                Print logging from level ERROR\n");
  printf("     -si               Show values in strict SI units: degrees Kelvin, rotation in radians/sec, etc.\n");
  printf("     -geo dd           Print geographic format in dd.dddddd format\n");
  printf("     -geo dm           Print geographic format in dd.mm.mmm format\n");
  printf("     -geo dms          Print geographic format in dd.mm.sss format\n");
#ifndef SKIP_SETSYSTEMCLOCK
  printf("     -clocksrc         Set the systemclock from time info from this NMEA source address\n");
#endif
  printf("     -version          Print the version of the program and quit\n");
  printf("\nThe following options are used to debug the analyzer:\n");
  printf("     -raw              Print raw bytes (obsolete, use -data)\n");
  printf("     -data             Print the PGN three times: in hex, ascii and analyzed\n");
  printf("     -debug            Print raw value per field\n");
  printf("\n");
  exit(1);
}

int main(int argc, char **argv)
{
  int    r;
  char   msg[2000];
  FILE  *file = stdin;
  int    ac   = argc;
  char **av   = argv;

  setProgName(argv[0]);

  for (; ac > 1; ac--, av++)
  {
    if (strcasecmp(av[1], "-version") == 0)
    {
      printf("%s\n", VERSION);
      exit(0);
    }
    else if (strcasecmp(av[1], "-camel") == 0)
    {
      camelCase(false);
    }
    else if (strcasecmp(av[1], "-upper-camel") == 0)
    {
      camelCase(true);
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
    else if (strcasecmp(av[1], "-json") == 0)
    {
      showJson = true;
    }
    else if (strcasecmp(av[1], "-empty") == 0)
    {
      showJsonEmpty = true;
      showJson      = true;
    }
    else if (strcasecmp(av[1], "-nv") == 0)
    {
      showJsonValue = true;
      showJson      = true;
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
        logInfo("Only logging PGN %d\n", onlyPgn);
      }
      else
      {
        usage(argv, av + 1);
      }
    }
  }

  if (!showJson)
  {
    logInfo("N2K packet analyzer\n" COPYRIGHT);
  }
  else
  {
    printf("{\"version\":\"%s\",\"units\":\"%s\"}\n", VERSION, showSI ? "si" : "std");
  }

  fillFieldCounts();
  fillLookups();
  checkPgnList();

  while (fgets(msg, sizeof(msg) - 1, file))
  {
    RawMessage m;

    if (*msg == 0 || *msg == '\n' || *msg == '#')
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

static enum RawFormats detectFormat(const char *msg)
{
  char        *p;
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

static char *getSep()
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

static void printEmpty(const char *name, int64_t exceptionValue)
{
  if (showJsonEmpty)
  {
    mprintf("%s\"%s\":null", getSep(), name);
  }
  else if (!showJson)
  {
    switch (exceptionValue)
    {
      case DATAFIELD_UNKNOWN:
        mprintf("%s %s = Unknown", getSep(), name);
        break;
      case DATAFIELD_ERROR:
        mprintf("%s %s = ERROR", getSep(), name);
        break;
      case DATAFIELD_RESERVED1:
        mprintf("%s %s = RESERVED1", getSep(), name);
        break;
      case DATAFIELD_RESERVED2:
        mprintf("%s %s = RESERVED2", getSep(), name);
        break;
      case DATAFIELD_RESERVED3:
        mprintf("%s %s = RESERVED3", getSep(), name);
        break;
      default:
        mprintf("%s %s = Unhandled value %ld", getSep(), name, exceptionValue);
    }
  }
}

static void printCanRaw(RawMessage *msg)
{
  size_t i;
  FILE  *f = stdout;

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
  int64_t  maxValue;
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
  maxValue = (bytes == 8) ? INT64_C(0x7fffffffffffffff) : INT64_C(0x7fffffff);
  if (value > maxValue - 2)
  {
    printEmpty(name, value - maxValue);
    return true;
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
    printEmpty(name, d - INT64_C(0xffff));
    return true;
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
    printEmpty(name, t - INT64_C(0xffffffff));
    return true;
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
  double  k        = t * resolution;
  double  c        = k - 273.15;
  double  f        = c * 1.8 + 32;
  int64_t maxValue = (bits == 16) ? 0xffff : 0xffffff;

  if (t >= maxValue - 2)
  {
    printEmpty(name, t - maxValue);
    return true;
  }

  if (showSI)
  {
    if (showJson)
    {
      mprintf("%s\"%s\":%.2f", getSep(), name, k);
    }
    else
    {
      mprintf("%s %s = %.2f K", getSep(), name, k);
    }
    return true;
  }

  if (showJson)
  {
    mprintf("%s\"%s\":%.2f", getSep(), name, c);
  }
  else
  {
    mprintf("%s %s = %.2f C (%.1f F)", getSep(), name, c, f);
  }

  return true;
}

static bool printPressure(char *name, uint32_t v, Field *field)
{
  double pressure;
  double bar;
  double psi;
  int    precision = 5;

  pressure = v;
  if (field->size <= 16)
  {
    if (v >= 0xfffd)
    {
      printEmpty(name, v - INT64_C(0xffff));
      return true;
    }
    if (field->hasSign)
    {
      pressure = (int16_t) v;
    }
  }
  else
  {
    if (v >= 0xfffffffd)
    {
      printEmpty(name, v - INT64_C(0xffffffff));
      return true;
    }
    if (field->hasSign)
    {
      pressure = (int32_t) v;
    }
  }

  // There are four types of known pressure: unsigned hectopascal, signed kpa, unsigned kpa, unsigned four bytes in pascal.

  // Now scale pascal properly, it is in hPa or kPa.
  if (field->units)
  {
    switch (field->units[0])
    {
      case 'h':
      case 'H':
        pressure *= 100.0;
        precision -= 2;
        break;
      case 'k':
      case 'K':
        pressure *= 1000.0;
        precision -= 3;
        break;
      case 'd':
        pressure /= 10.0;
        precision += 1;
        break;
    }
  }

  bar = pressure * 1e-5;        /* 1000 hectopascal = 1 Bar */
  psi = pressure * 1.450377e-4; /* Silly but still used in some parts of the world */

  if (showJson)
  {
    if (precision <= 3)
    {
      mprintf("%s\"%s\":%.0f", getSep(), name, pressure);
    }
    else
    {
      mprintf("%s\"%s\":%.*f", getSep(), name, precision - 3, pressure);
    }
  }
  else
  {
    mprintf("%s %s = %.*f bar (%.*f PSI)", getSep(), name, precision, bar, precision - 1, psi);
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

  logDebug("printHex(\"%s\", %p, %zu, %zu)\n", name, data, startBit, bits);

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

static bool
printVarField(Field *field, char *fieldName, uint32_t refPgn, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  Field *refField;

  logDebug("printVarField(<%s>,\"%s\",%u, ..., %zu, %zu, %zu)\n", field->name, fieldName, refPgn, dataLen, startBit, *bits);

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
    return printField(refField, fieldName, data, dataLen, startBit, bits);
  }

  logError("Field %s: cannot derive variable length for PGN %d field # %d\n", fieldName, refPgn, data[-1]);
  *bits = 8; /* Gotta assume something */
  return false;
}

static bool printNumber(char *fieldName, Field *field, uint8_t *data, size_t startBit, size_t bits)
{
  int64_t value;
  int64_t maxValue;
  int64_t reserved;
  double  a;

  extractNumber(field, data, startBit, bits, &value, &maxValue);

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
    if (field->units && field->units[0] == '=' && isdigit(field->units[2]))
    {
      char        lookfor[20];
      const char *s;

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

    else if ((field->resolution == RES_LOOKUP || field->resolution == RES_MANUFACTURER) && field->lookupValue)
    {
      const char *s = field->lookupValue[value];

      if (s)
      {
        if (showJsonValue)
        {
          mprintf("%s\"%s\":{\"value\":%" PRId64 ",\"name\":\"%s\"}", getSep(), fieldName, value, s);
        }
        else if (showJson)
        {
          mprintf("%s\"%s\":\"%s\"", getSep(), fieldName, s);
        }
        else
        {
          mprintf("%s %s = %s", getSep(), fieldName, s);
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

    else if (field->resolution == RES_BITFIELD && field->lookupValue)
    {
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
          const char *s = field->lookupValue[value];

          if (s)
          {
            if (showJson)
            {
              mprintf("%c\"%s\"", sep, s);
            }
            else
            {
              mprintf("%c%s", sep, s);
            }
          }
          else
          {
            mprintf("%c\"%" PRIu64 "\"", sep, bitValue);
          }
          sep = ',';
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
        else if (units && units[0] != '=')
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
    printEmpty(fieldName, value - maxValue);
  }

  return true;
}

void setSystemClock(void)
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
  Pgn    *pgn = &pgnList[index];

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
      logDebug("Resizing buffer for PGN %u device %u to accommodate %u bytes (heap %zu bytes)\n",
               pgn->pgn,
               msg->src,
               msg->len,
               heapSize);
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

  logDebug("printPacket size=%zu len=%zu\n", packet->size, msg->len);
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

static bool printString(Field *field, const char *fieldName, uint8_t *data, size_t bytes, size_t *bits)
{
  uint8_t len;
  int     k;
  uint8_t lastbyte;

  logDebug("printString(<%s>,\"%s\",[%x,%x],%zu)\n", field->name, fieldName, data[0], data[1], bytes);

  if (field->resolution == RES_STRINGLZ)
  {
    // STRINGLZ format is <len> [ <data> ... ]
    len   = *data++;
    bytes = len + 1;
  }
  else if (field->resolution == RES_STRINGLAU)
  {
    // STRINGLAU format is <len> <control> [ <data> ... ]
    // where <control> == 0 = UNICODE, but we don't know whether it is UTF16, UTF8, etc. Not seen in the wild yet!
    //       <control> == 1 = ASCII(?) or maybe UTF8?
    int control;

    bytes   = *data++;
    control = *data++;
    len     = bytes - 2;
    if (control == 0)
    {
      logError("Unhandled UNICODE string in PGN\n");
      return false;
    }
    if (control > 1)
    {
      logError("Unhandled string type %d in PGN\n");
      return false;
    }
  }
  else if (field->resolution == RES_STRING)
  {
    // STRING format is <start> [ <data> ... ] <stop>
    //                  <len> [ <data> ... ] (with len > 2)
    //                  <stop>                                 zero length data
    //                  <#00>  ???
    if (*data == 0x02)
    {
      data++;
      for (len = 0; len < bytes && data[len] != 0x01; len++)
        ;
      bytes = len + 2;
    }
    else if (*data > 0x02)
    {
      bytes = *data++;
      len   = bytes - 1;

      // This is actually more like a STRINGLAU control byte, not sure
      // whether these fields are actually just STRINGLAU?
      if (*data == 0x01)
      {
        data++;
        len--;
      }
    }
    else
    {
      bytes = 1;
      len   = 0;
    }
  }
  else
  {
    // ASCII format is a fixed length string
    len   = BITS_TO_BYTES(field->size);
    bytes = len;
  }

  *bits = BYTES(bytes);

  logDebug("printString res=%f len=%zu bytes=%zu\n", field->resolution, len, bytes);

  // Sanity check
  if (len > bytes)
  {
    len = bytes;
  }

  if (showBytes)
  {
    for (k = 0; k < len; k++)
    {
      mprintf("%02x ", data[k]);
    }
  }

  if (len > 0)
  {
    // rtrim funny stuff from end, we see all sorts
    lastbyte = data[len - 1];
    if (lastbyte == 0xff || isspace(lastbyte) || lastbyte == 0 || lastbyte == '@')
    {
      while (len > 0 && (data[len - 1] == lastbyte))
      {
        len--;
      }
    }

    if (showJson)
    {
      mprintf("%s\"%s\":\"", getSep(), fieldName);
      print_ascii_json_escaped(data, len);
      mprintf("\"");
    }
    else
    {
      mprintf("%s %s = ", getSep(), fieldName);
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
  }
  else
  {
    printEmpty(fieldName, DATAFIELD_UNKNOWN);
  }

  return true;
}

static bool printField(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  static uint32_t refPgn = 0; // Remember this over the entire set of fields

  size_t   bytes;
  uint16_t valueu16;
  uint32_t valueu32;

  if (fieldName == NULL)
  {
    fieldName = field->camelName ? field->camelName : field->name;
  }

  logDebug("printField(<%s>, \"%s\", ..., %zu, %zu) res=%f\n", field->name, fieldName, dataLen, startBit, field->resolution);

  *bits = field->size;
  bytes = (*bits + 7) / 8;
  bytes = min(bytes, dataLen);
  *bits = min(bytes * 8, *bits);

  if (strcmp(fieldName, "PGN") == 0)
  {
    refPgn = data[0] + (data[1] << 8) + (data[2] << 16);
  }

  if (strcmp(fieldName, "Reserved") == 0)
  {
    // Skipping reserved fields. Unfortunately we have some cases now
    // where they are zero. Some AIS devices (SRT) produce an 8 bit long
    // nav status, others just a four bit one.
    return true;
  }

  if (field->units && strcmp(field->units, PROPRIETARY_PGN_ONLY) == 0)
  {
    if ((refPgn >= 65280 && refPgn <= 65535) || (refPgn >= 126720 && refPgn <= 126975) || (refPgn >= 130816 && refPgn <= 131071))
    {
      // proprietary, allow field
    }
    else
    {
      // standard PGN, skip field
      *bits = 0;
      return true;
    }
  }
  if (field->resolution < 0.0)
  {
    /* These fields have only been found to start on byte boundaries,
     * making their location easier
     */
    if (field->resolution == RES_STRINGLZ || field->resolution == RES_STRINGLAU || field->resolution == RES_ASCII
        || field->resolution == RES_STRING)
    {
      return printString(field, fieldName, data, dataLen, bits);
    }

    if (field->resolution == RES_LONGITUDE || field->resolution == RES_LATITUDE)
    {
      return printLatLon(fieldName, field->resolution, data, bytes);
    }
    if (field->resolution == RES_DATE)
    {
      valueu16    = data[0] + (data[1] << 8);
      currentDate = valueu16;
      return printDate(fieldName, valueu16);
    }
    if (field->resolution == RES_TIME)
    {
      valueu32    = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
      currentTime = valueu32;
      return printTime(fieldName, valueu32);
    }
    if (field->resolution == RES_PRESSURE)
    {
      valueu32 = data[0] + (data[1] << 8);
      return printPressure(fieldName, valueu32, field);
    }
    if (field->resolution == RES_PRESSURE_HIRES)
    {
      valueu32 = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
      return printPressure(fieldName, valueu32, field);
    }
    if (field->resolution == RES_TEMPERATURE)
    {
      valueu32 = data[0] + (data[1] << 8);
      return printTemperature(fieldName, valueu32, 16, 0.01);
    }
    if (field->resolution == RES_TEMPERATURE_HIGH)
    {
      valueu32 = data[0] + (data[1] << 8);
      return printTemperature(fieldName, valueu32, 16, 0.1);
    }
    if (field->resolution == RES_TEMPERATURE_HIRES)
    {
      valueu32 = data[0] + (data[1] << 8) + (data[2] << 16);
      return printTemperature(fieldName, valueu32, 24, 0.001);
    }
    if (field->resolution == RES_6BITASCII)
    {
      return print6BitASCIIText(fieldName, data, startBit, *bits);
    }
    if (field->resolution == RES_DECIMAL)
    {
      return printDecimal(fieldName, data, startBit, *bits);
    }
    if (field->resolution == RES_VARIABLE)
    {
      return printVarField(field, fieldName, refPgn, data, dataLen, startBit, bits);
    }
    if (*bits > BYTES(8))
    {
      return printHex(fieldName, data, startBit, *bits);
    }
    if (field->resolution == RES_INTEGER || field->resolution == RES_LOOKUP || field->resolution == RES_BITFIELD
        || field->resolution == RES_BINARY || field->resolution == RES_MANUFACTURER)
    {
      return printNumber(fieldName, field, data, startBit, *bits);
    }
    logError("Unknown resolution %f for %s\n", field->resolution, fieldName);
    return false;
  }
  return printNumber(fieldName, field, data, startBit, *bits);
}

bool printPgn(RawMessage *msg, uint8_t *dataStart, int length, bool showData, bool showJson)
{
  Pgn *pgn;

  uint8_t *data;

  uint8_t *dataEnd = dataStart + length;
  size_t   i;
  size_t   bits;
  size_t   startBit;
  int      repetition = 0;
  char     fieldName[60];
  bool     r;
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

  r = true;
  for (i = 0, startBit = 0, data = dataStart; data < dataEnd; i++)
  {
    Field *field;

    if (variableFieldCount[0] > 0 && i == variableFieldStart && repetition == 0)
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
        logDebug("variableFields: field=%d variableFieldStart=%d variableFields[0]=%d\n", i, variableFieldStart, variableFields[0]);
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

    if (!printField(field, fieldName, data, dataEnd - data, startBit, &bits))
    {
      logError("PGN %u field %s error\n", msg->pgn, fieldName);
      r = false;
      break;
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
    logError("PGN %u analysis error\n", msg->pgn);
  }

  if (msg->pgn == 126992 && currentDate < UINT16_MAX && currentTime < UINT32_MAX && clockSrc == msg->src)
  {
    setSystemClock();
  }
  return r;
}
