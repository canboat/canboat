/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2014, Kees Verruijt, Harlingen, The Netherlands.

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
#include "analyzer.h"

DevicePackets * device[256];
char * manufacturer[1 << 12];

enum RawFormats
{
  RAWFORMAT_PLAIN,
  RAWFORMAT_FAST,
  RAWFORMAT_AIRMAR,
  RAWFORMAT_CHETCO
};

enum RawFormats format = RAWFORMAT_PLAIN;

enum GeoFormats
{
  GEO_DD,
  GEO_DM,
  GEO_DMS
};


bool showRaw = false;
bool showData = false;
bool showBytes = false;
bool showJson = false;
char * sep = " ";
int  braceCount = 0; // Open json output { characters that need to be closed
enum GeoFormats showGeo = GEO_DD;
int onlyPgn = 0;
int onlySrc = -1;
int clockSrc = -1;
size_t heapSize = 0;

static void fillManufacturers(void);
static void fillFieldCounts(void);
static uint8_t scanNibble(char c);
static int scanHex(char ** p, uint8_t * m);
static bool printNumber(char * fieldName, Field * field, uint8_t * data, size_t startBit, size_t bits);

void initialize(void);
void printCanRaw(RawMessage * msg);
bool printCanFormat(RawMessage * msg);
void explain(void);
void explainXML(void);
void camelCase(bool upperCamelCase);

void usage(char ** argv, char ** av)
{
  printf("Unknown or invalid argument %s\n", av[0]);
  printf("Usage: %s [[-raw] [-json [-camel | -upper-camel]] [-data] [-debug] [-d] [-q] [-geo {dd|dm|dms}] [-src <src> | <pgn>]] ["
#ifndef SKIP_SETSYSTEMCLOCK
         "-clocksrc <src> | "
#endif
         "-explain | -explain-xml [-upper-camel]]\n", argv[0]);
  exit(1);
}

int main(int argc, char ** argv)
{
  int r;
  char msg[2000];
  FILE * file = stdin;
  int ac = argc;
  char ** av = argv;
  bool doExplainXML = false;
  bool doExplain = false;

  setProgName(argv[0]);

  for ( ; ac > 1; ac--, av++)
  {
    if (strcasecmp(av[1], "-explain-xml") == 0)
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
        usage(argv, av);
      }
      ac--;
      av++;
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
# ifndef SKIP_SETSYSTEMCLOCK
    else if (ac > 2 && strcasecmp(av[1], "-clocksrc") == 0)
    {
      clockSrc = strtol(av[2], 0, 10);
      ac--;
      av++;
    }
# endif
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
        usage(argv, av);
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
    logInfo("N2K packet analyzer "PROGRAM_REV" from "PROGRAM_DATE"\n" COPYRIGHT);
  }

  fillManufacturers();
  fillFieldCounts();

  while (fgets(msg, sizeof(msg) - 1, file))
  {
    RawMessage m;
    unsigned int prio, pgn, dst, src, len, junk;
    char * p;
    unsigned int i;

    if (*msg == 0 || *msg == '\n')
    {
      continue;
    }

    if (format != RAWFORMAT_CHETCO && msg[0] == '$' && strncmp(msg, "$PCDIN", 6) == 0)
    {
      if (showBytes)
      {
        logInfo("Detected Chetco protocol with all data on one line\n");
      }
      format = RAWFORMAT_CHETCO;
    }

    if (format != RAWFORMAT_CHETCO)
    {
      if (format != RAWFORMAT_AIRMAR)
      {
        p = strchr(msg, ',');
      }
      else
      {
        p = strchr(msg, ' ');
      }

      if (!p)
      {
        p = strchr(msg, ' ');
        if (p && (p[1] == '-' || p[2] == '-'))
        {
          if (format != RAWFORMAT_AIRMAR && showBytes)
          {
            logInfo("Detected Airmar protocol with all data on one line\n");
          }
          format = RAWFORMAT_AIRMAR;
        }
      }
      if (!p || p >= msg + sizeof(m.timestamp) - 1)
      {
        logError("Error reading message, scanning timestamp from %s", msg);
        if (!showJson) fprintf(stdout, "%s", msg);
        continue;
      }
    }

    if (format == RAWFORMAT_PLAIN)
    {
      unsigned int data[8];

      memcpy(m.timestamp, msg, p - msg);
      m.timestamp[p - msg] = 0;

      /* Moronic Windows does not support %hh<type> so we use intermediate variables */
      r = sscanf( p
        , ",%u,%u,%u,%u,%u"
        ",%x,%x,%x,%x,%x,%x,%x,%x,%x"
        , &prio
        , &pgn
        , &src
        , &dst
        , &len
        , &data[0]
        , &data[1]
        , &data[2]
        , &data[3]
        , &data[4]
        , &data[5]
        , &data[6]
        , &data[7]
        , &junk
      );
      if (r < 5)
      {
        logError("Error reading message, scanned %u from %s", r, msg);
        if (!showJson) fprintf(stdout, "%s", msg);
        continue;
      }
      if (r <= 5 + 8)
      {
        for (i = 0; i < len; i++)
        {
          m.data[i] = data[i];
        }
      }
      else
      {
        if (showBytes)
        {
          logInfo("Detected Fast protocol with all data on one line\n");
        }
        format = RAWFORMAT_FAST;
      }
    }
    if (format == RAWFORMAT_FAST)
    {
      memcpy(m.timestamp, msg, p - msg);
      m.timestamp[p - msg] = 0;

      /* Moronic Windows does not support %hh<type> so we use intermediate variables */
      r = sscanf( p
        , ",%u,%u,%u,%u,%u "
        , &prio
        , &pgn
        , &src
        , &dst
        , &len
      );
      if (r < 5)
      {
        logError("Error reading message, scanned %u from %s", r, msg);
        if (!showJson) fprintf(stdout, "%s", msg);
        continue;
      }
      for (i = 0; *p && i < 5;)
      {
        if (*++p == ',')
        {
          i++;
        }
      }
      if (!p)
      {
        logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
        if (!showJson) fprintf(stdout, "%s", msg);
        continue;
      }
      p++;
      for (i = 0; i < len; i++)
      {
        if (scanHex(&p, &m.data[i]))
        {
          logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
          if (!showJson) fprintf(stdout, "%s", msg);
          continue;
        }
        if (i < len)
        {
          if (*p != ',' && !isspace(*p))
          {
            logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
            if (!showJson) fprintf(stdout, "%s", msg);
            continue;
          }
          p++;
        }
      }
    }
    else if (format == RAWFORMAT_AIRMAR)
    {
      unsigned int id;

      memcpy(m.timestamp, msg, p - msg - 1);
      m.timestamp[p - msg - 1] = 0;
      p += 3;

      /* Moronic Windows does not support %hh<type> so we use intermediate variables */
      pgn = strtoul(p, &p, 10);
      if (*p == ' ')
      {
        id = strtoul(++p, &p, 16);
      }
      if (*p != ' ')
      {
        logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
        if (!showJson) fprintf(stdout, "%s", msg);
        continue;
      }

      getISO11783BitsFromCanId(id, &prio, &pgn, &src, &dst);

      p++;
      len = strlen(p) / 2;
      for (i = 0; i < len; i++)
      {
        if (scanHex(&p, &m.data[i]))
        {
          logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
          if (!showJson) fprintf(stdout, "%s", msg);
          continue;
        }
        if (i < len)
        {
          if (*p != ',' && *p != ' ')
          {
            logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
            if (!showJson) fprintf(stdout, "%s", msg);
            continue;
          }
          p++;
        }
      }
    }
    else if (format == RAWFORMAT_CHETCO)
    {
      unsigned int tstamp;
      time_t t;
      struct tm tm;

      if (sscanf(msg, "$PCDIN,%x,%x,%x,", &pgn, &tstamp, &src) < 3)
      {
        logError("Error reading Chetco message: %s", msg);
        if (!showJson) fprintf(stdout, "%s", msg);
        continue;
      }

      t = (time_t) tstamp / 1000;
      localtime_r(&t, &tm);
      strftime(m.timestamp, sizeof(m.timestamp), "%Y-%m-%d-%H:%M:%S", &tm);
      sprintf(m.timestamp + strlen(m.timestamp), ",%u", tstamp % 1000);

      p = msg + STRSIZE("$PCDIN,01FD07,089C77D!,03,"); // Fixed length where data bytes start;

      for (i = 0; *p != '*'; i++)
      {
        if (scanHex(&p, &m.data[i]))
        {
          logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
          if (!showJson) fprintf(stdout, "%s", msg);
          continue;
        }
      }

      prio = 0;
      dst = 255;
      len = i + 1;
    }

    m.prio = prio;
    m.pgn  = pgn;
    m.dst  = dst;
    m.src  = src;
    m.len  = len;

    printCanFormat(&m);
    printCanRaw(&m);
  }

  return 0;
}

static uint8_t scanNibble(char c)
{
  if (isdigit(c))
  {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F')
  {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f')
  {
    return c - 'a' + 10;
  }
  return 16;
}

static int scanHex(char ** p, uint8_t * m)
{
  uint8_t hi, lo;

  if (!(*p)[0] || !(*p)[1])
  {
    return 1;
  }

  hi = scanNibble((*p)[0]);
  if (hi > 15)
  {
    return 1;
  }
  lo = scanNibble((*p)[1]);
  if (lo > 15)
  {
    return 1;
  }
  (*p) += 2;
  *m = hi << 4 | lo;
  /* printf("(b=%02X,p=%p) ", *m, *p); */
  return 0;
}

char * getSep()
{
  char * s = sep;

  if (showJson)
  {
    sep = ",";
    if (strchr(s, '{'))
    {
      braceCount++;
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
    for (j = 0; pgnList[i].fieldList[j].name && j < 80; j++);
    if (j == 80)
    {
      logError("Internal PGN %d does not have correct fieldlist.\n", pgnList[i].pgn);
      exit(2);
    }
    pgnList[i].fieldCount = j;
  }
}

char mbuf[8192];
char * mp = mbuf;

void mprintf(const char * format, ...)
{
  va_list ap;
  int remain;

  va_start(ap, format);
  remain = sizeof(mbuf) - (mp - mbuf) - 1;
  mp += vsnprintf(mp, remain, format, ap);
  va_end(ap);
}


void mreset(void)
{
  mp = mbuf;
}

void mwrite(FILE * stream)
{
  fwrite(mbuf, sizeof(char), mp - mbuf, stream);
  fflush(stream);
  mreset();
}

void printCanRaw(RawMessage * msg)
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

static bool printLatLon(char * name, double resolution, uint8_t * data, size_t bytes)
{
  uint64_t absVal;
  int64_t value;

  value = 0;
  memcpy(&value, data, bytes);
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
      mprintf("(%"PRIx64" = %"PRId64") ", value, value);
    }

    value /= INT64_C(1000000000);
  }
  absVal = (value < 0) ? -value : value;

  if (showBytes)
  {
    mprintf("(%"PRId64") ", value);
  }

  if (showGeo == GEO_DD)
  {
    double dd = (double) value / (double) RES_LAT_LONG_PRECISION;

    if (showJson)
    {
      mprintf("%s\"%s\":%010.7f", getSep(), name, dd);
    }
    else
    {
      mprintf("%s %s = %010.7f", getSep(), name, dd);
    }
  }
  else if (showGeo == GEO_DM)
  {
    /* One degree = 10e6 */

    uint64_t degrees = (absVal / RES_LAT_LONG_PRECISION);
    uint64_t remainder = (absVal % RES_LAT_LONG_PRECISION);
    double minutes = (remainder * 60) / (double) RES_LAT_LONG_PRECISION;

    mprintf((showJson ? "%s\"%s\":\"%02u&deg; %06.3f %c\"" : "%s %s = %02ud %06.3f %c")
           , getSep(), name, (uint32_t) degrees, minutes
           , ((resolution == RES_LONGITUDE)
              ? ((value >= 0) ? 'E' : 'W')
              : ((value >= 0) ? 'N' : 'S')
             )
           );
  }
  else
  {
    uint32_t degrees = (uint32_t) (absVal / RES_LAT_LONG_PRECISION);
    uint32_t remainder = (uint32_t) (absVal % RES_LAT_LONG_PRECISION);
    uint32_t minutes = (remainder * 60) / RES_LAT_LONG_PRECISION;
    double seconds = (((uint64_t) remainder * 3600) / (double) RES_LAT_LONG_PRECISION) - (60 * minutes);

    mprintf( (showJson ? "%s\"%s\":\"%02u&deg;%02u&rsquo;%06.3f&rdquo;%c\"": "%s %s = %02ud %02u' %06.3f\"%c")
           , getSep(), name, degrees, minutes, seconds
           , ((resolution == RES_LONGITUDE)
              ? ((value >= 0) ? 'E' : 'W')
              : ((value >= 0) ? 'N' : 'S')
             )
           );
    if (showJson)
    {
      double dd = (double) value / (double) RES_LAT_LONG_PRECISION;
      mprintf("%s\"%s_dd\":%010.7f", getSep(), name, dd);
    }
  }
  return true;
}

static bool printDate(char * name, uint16_t d)
{
  char buf[sizeof("2008.03.10") + 1];
  time_t t;
  struct tm * tm;

  if (d >= 0xfffd)
  {
    return false;
  }

  if (showBytes)
  {
    mprintf("(date %hx = %hd) ", d, d);
  }

  t = d * 86400;
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

static bool printTime(char * name, uint32_t t)
{
  uint32_t hours;
  uint32_t minutes;
  uint32_t seconds;
  uint32_t units;
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
  units = t % unitspersecond;
  minutes = seconds / 60;
  seconds = seconds % 60;
  hours = minutes / 60;
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

static bool printTemperature(char * name, uint16_t t)
{
  double c = t / 100.0 - 273.15;
  double f = c * 1.8 + 32;

  if (t >= 0xfffd)
  {
    return false;
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

static bool printPressure(char * name, uint16_t v, Field * field)
{
  int32_t pressure;
  double bar;
  double psi;

  if (v >= 0xfffd)
  {
    return false;
  }

  // There are three types of known pressure: unsigned hectopascal, signed kpa, unsigned kpa.

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
    }
  }

  bar = pressure / 100000.0; /* 1000 hectopascal = 1 Bar */
  psi = pressure / 1450.377; /* Silly but still used in some parts of the world */

  if (showJson)
  {
    mprintf("%s\"%s\":%"PRId32"", getSep(), name, pressure);
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

static bool print6BitASCIIText(char * name, uint8_t * data, size_t startBit, size_t bits)
{
  uint8_t value = 0;
  uint8_t maxValue = 0;
  uint8_t bitMask = 1 << startBit;
  uint64_t bitMagnitude = 1;
  size_t bit;
  char buf[128];

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
      value = 0;
      bitMagnitude = 1;
    }
  }
  if (showJson)
  {
    mprintf("\"");
  }
  return true;
}

static bool printHex(char * name, uint8_t * data, size_t startBit, size_t bits)
{
  uint8_t value = 0;
  uint8_t maxValue = 0;
  uint8_t bitMask = 1 << startBit;
  uint64_t bitMagnitude = 1;
  size_t bit;
  char buf[128];

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
      value = 0;
      bitMagnitude = 1;
    }
  }
  if (showJson)
  {
    mprintf("\"");
  }
  return true;
}


/*
 *
 * This is perhaps as good a place as any to explain how CAN messages are layed out by the
 * NMEA. Basically, it's a mess once the bytes are recomposed into bytes (the on-the-wire
 * format is fine).
 *
 * For fields that are aligned on bytes there isn't much of an issue, they appear in our
 * buffers in standard Intel 'least endian' format.
 * For instance the MMSI # 244050447 is, in hex: 0x0E8BEA0F. This will be found in the CAN data as:
 * byte x+0: 0x0F
 * byte x+1: 0xEA
 * byte x+2: 0x8B
 * byte x+3: 0x0e
 *
 * To gather together we loop over the bytes, and keep increasing the magnitude of what we are
 * adding:
 *    for (i = 0, magnitude = 0; i < 4; i++)
 *    {
 *      value += data[i] << magnitude;
 *      magnitude += 8;
 *    }
 *
 * However, when there are two bit fields after each other, lets say A of 2 and then B of 6 bits:
 * then that is layed out MSB first, so the bit mask is 0b11000000 for the first
 * field and 0b00111111 for the second field.
 *
 * This means that if we have a bit field that crosses a byte boundary and does not start on
 * a byte boundary, the bit masks are like this (for a 16 bit field starting at the 3rd bit):
 *
 * 0b00111111 0b11111111 0b11000000
 *     ------   --------   --
 *     000000   11110000   11
 *     543210   32109876   54
 *
 * So we are forced to mask bits 0 and 1 of the first byte. Since we need to process the previous
 * field first, we cannot repeatedly shift bits out of the byte: if we shift left we get the first
 * field first, but in MSB order. We need bit values in LSB order, as the next byte will be more
 * significant. But we can't shift right as that will give us bits in LSB order but then we get the
 * two fields in the wrong order...
 *
 * So for that reason we explicitly test, per byte, how many bits we need and how many we have already
 * used.
 *
 */

static void extractNumber(Field * field, uint8_t * data, size_t startBit, size_t bits, int64_t * value, int64_t * maxValue)
{
  bool hasSign = field->hasSign;

  size_t firstBit = startBit;
  size_t bitsRemaining = bits;
  size_t magnitude = 0;
  size_t bitsInThisByte;
  uint64_t bitMask;
  uint64_t allOnes;
  uint64_t valueInThisByte;

  *value = 0;
  *maxValue = 0;

  if (showBytes)
  {
    mprintf("(en f=%s,sb=%u,b=%u) ", field->name, (unsigned int) startBit, (unsigned int) firstBit);
  }

  while (bitsRemaining)
  {
    bitsInThisByte = min(8 - firstBit, bitsRemaining);
    allOnes = (uint64_t) ((((uint64_t) 1) << bitsInThisByte) - 1);

    //How are bits ordered in bytes for bit fields? There are two ways, first field at LSB or first
    //field as MSB.
    //Experimentation, using the 129026 PGN, has shown that the most likely candidate is LSB.
    bitMask = allOnes << firstBit;
    valueInThisByte = (*data & bitMask) >> firstBit;

    *value |= valueInThisByte << magnitude;
    *maxValue |= (int64_t) allOnes << magnitude;

    if (showBytes)
    {
      mprintf("(d=%x,bib=%u,fb=%u,msk=%x,v=%x,mag=%x) ", *data, bitsInThisByte, firstBit, (unsigned int)bitMask, (unsigned int)valueInThisByte, (unsigned int)magnitude);
    }

    magnitude += bitsInThisByte;
    bitsRemaining -= bitsInThisByte;
    firstBit += bitsInThisByte;
    if (firstBit >= 8)
    {
      firstBit -= 8;
      data++;
    }
  }

  if (hasSign)
  {
    *maxValue >>= 1;

    if (field->offset) /* J1939 Excess-K notation */
    {
      *value += field->offset;
    }
    else
    {
      bool negative = (*value & (((uint64_t) 1) << (bits - 1))) > 0;

      if (negative)
      {
        /* Sign extend value for cases where bits < 64 */
        /* Assume we have bits = 16 and value = -2 then we do: */
        /* 0000.0000.0000.0000.0111.1111.1111.1101 value    */
        /* 0000.0000.0000.0000.0111.1111.1111.1111 maxvalue */
        /* 1111.1111.1111.1111.1000.0000.0000.0000 ~maxvalue */
        *value |= ~*maxValue;
      }
    }
  }

  if (showBytes)
  {
    mprintf("(v=%llx,m=%llx) ", *value, *maxValue);
  }
}

static Field * getField(uint32_t pgn, uint32_t field)
{
  int index;

  for (index = 0; index < ARRAY_SIZE(pgnList); index++)
  {
    if (pgn == pgnList[index].pgn)
    {
      if (field > pgnList[index].fieldCount)
      {
        return 0;
      }
      return &(pgnList[index].fieldList[field]);
    }
  }
  return 0;
}

static bool printVarNumber(char * fieldName, Pgn * pgn, uint32_t refPgn, Field * field, uint8_t * data, size_t startBit, size_t * bits)
{
  Field * refField;
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

  logError("Pgn %d Field %s: cannot derive variable length from PGN %d field # %d\n"
          , pgn->pgn, field->name, refPgn, data[-1]);
  *bits = 8; /* Gotta assume something */
  return false;
}

static bool printNumber(char * fieldName, Field * field, uint8_t * data, size_t startBit, size_t bits)
{
  bool ret = false;
  int64_t value;
  int64_t maxValue;
  int64_t reserved;
  double a;

  extractNumber(field, data, startBit, bits, &value, &maxValue);

  /* There are max five reserved values according to ISO 11873-9 (that I gather from indirect sources)
   * but I don't yet know which datafields reserve the reserved values.
   */
#define DATAFIELD_UNKNOWN   (0)
#define DATAFIELD_ERROR     (-1)
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

  if (value <= maxValue - reserved)
  {
    if (field->units && field->units[0] == '=')
    {
      char lookfor[20];
      char * s;

      sprintf(lookfor, "=%"PRId64, value);
      if (strcmp(lookfor, field->units) != 0)
      {
        if (showBytes) logError("Field %s value %"PRId64" does not match %s\n", fieldName, value, field->units + 1);
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
    else
    if (field->resolution == RES_LOOKUP && field->units)
    {
      char lookfor[20];
      char * s, * e;

      sprintf(lookfor, ",%"PRId64"=", value);
      s = strstr(field->units, lookfor);
      if (s)
      {
        s += strlen(lookfor);
        e = strchr(s, ',');
        e = e ? e : s + strlen(s);
        if (showJson)
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
          mprintf("%s\"%s\":\"%"PRId64"\"", getSep(), fieldName, value);
        }
        else
        {
          mprintf("%s %s = %"PRId64"", getSep(), fieldName, value);
        }
      }
    }

    else if (field->resolution == RES_BINARY)
    {
      if (showJson)
      {
        mprintf("%s\"%s\":\"%"PRId64"\"", getSep(), fieldName, value);
      }
      else
      {
        mprintf("%s %s = 0x%"PRIx64, getSep(), fieldName, value);
      }
    }
    else if (field->resolution == RES_MANUFACTURER)
    {
      char * m = 0;
      char unknownManufacturer[30];

      if (value > 0 && value < ARRAY_SIZE(manufacturer))
      {
        m = manufacturer[value];
      }
      if (!m)
      {
        sprintf(unknownManufacturer, "Unknown Manufacturer %"PRId64, value);
        m = unknownManufacturer;
      }

      if (showJson)
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
          mprintf("%s\"%s\":%"PRId64"", getSep(), fieldName, value);
        }
        else
        {
          mprintf("%s %s = %"PRId64, getSep(), fieldName, value);
        }
      }
      else
      {
        int precision = 0;
        double r;

        a = value * field->resolution;

        if (field->resolution == RES_DEGREES)
        {
          precision = 1;
        }
        else if (field->resolution == RES_DEGREES * 0.0001)
        {
          precision = 4;
        }
        else
        {
          for (r = field->resolution; (r > 0.0) && (r < 1.0); r *= 10.0)
          {
            precision++;
          }
        }

        if (showJson)
        {
          mprintf("%s\"%s\":%.*f", getSep(), fieldName, precision, a);
        }
        else if (field->units && strcmp(field->units, "m") == 0 && a >= 1000.0)
        {
          mprintf("%s %s = %.*f km", getSep(), fieldName, precision + 3, a / 1000);
        }
        else
        {
          mprintf("%s %s = %.*f", getSep(), fieldName, precision, a);
          if (field->units)
          {
            mprintf(" %s", field->units);
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
        mprintf("%s %s = Unhandled value %ld (%ld)", getSep(), fieldName, value, value - maxValue);
      }
    }
  }

  return true;
}

void setSystemClock(uint16_t currentDate, uint32_t currentTime)
{

#ifndef SKIP_SETSYSTEMCLOCK
  static uint16_t prevDate = UINT16_MAX;
  static uint32_t prevTime = UINT32_MAX;
  const uint32_t unitspersecond = 10000;
  const uint32_t microsperunit = 100;
  const uint32_t microspersecond = 1000000;
  const uint32_t secondsperday = 86400;
  struct timeval now;
  struct timeval gps;
  struct timeval delta;
  struct timeval olddelta;

#ifdef HAS_ADJTIME
  const int maxDelta = 30;
#else
  const int maxDelta = 1;
#endif

  logDebug("setSystemClock = %u/%u\n", currentDate, currentTime);

  if (prevDate == UINT16_MAX)
  {
    logDebug("setSytemClock: first time\n");
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

  gps.tv_sec = currentDate * secondsperday + currentTime / unitspersecond;
  gps.tv_usec = (currentTime % unitspersecond) * microsperunit;

  if (gps.tv_sec < now.tv_sec - maxDelta || gps.tv_sec > now.tv_sec + maxDelta)
  {
    if (settimeofday(&gps, 0))
    {
      logError("Failed to adjust system clock to %"PRIu64"/%06u\n", (uint64_t) gps.tv_sec, gps.tv_usec);
      return;
    }
    if (showBytes)
    {
      logInfo("Set system clock to %"PRIu64"/%06u\n", (uint64_t) gps.tv_sec, gps.tv_usec);
    }
    return;
  }

#ifdef HAS_ADJTIME

  delta.tv_sec = 0;
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
    logDebug("Now = %"PRIu64"/%06u ", (uint64_t) now.tv_sec, now.tv_usec);
    logDebug("GPS = %"PRIu64"/%06u ", (uint64_t) gps.tv_sec, gps.tv_usec);
    logDebug("Adjusting system clock by %d usec\n", delta.tv_usec);
    if (olddelta.tv_sec || olddelta.tv_usec)
    {
      logDebug("(Old delta not yet completed %"PRIu64"/%d\n", (uint64_t) olddelta.tv_sec, olddelta.tv_usec);
    }
  }

#endif
#endif
}

void print_json_escaped(uint8_t *data, int len) 
{

  int c;
  int k;
  for (k = 0; k < len; k++)
    {
      c = data[k];
      switch(c)
      {
        case '\b':
        case '\n':
        case '\r':
        case '\t':
        case '\f':
        case '"':
        case '\\':
        case '/':

          if(c == '\b') mprintf("%s", "\\b");
          else if(c == '\n') mprintf("%s", "\\n");
          else if(c == '\r') mprintf("%s", "\\r");
          else if(c == '\t') mprintf("%s", "\\t");
          else if(c == '\f') mprintf("%s", "\\f");
          else if(c == '"') mprintf("%s", "\\\"");
          else if(c == '\\') mprintf("%s", "\\\\");
          else if(c == '/') mprintf("%s", "\\/");
          break;
        default:
          if (c >= ' ' && c <= '~')
            mprintf("%c", c);
      }
    }
}


bool printPgn(int index, int subIndex, RawMessage * msg)
{
  uint8_t * dataStart;
  uint8_t * data;
  size_t size;
  uint8_t * dataEnd;
  size_t i;
  Field field;
  size_t bits;
  size_t bytes;
  size_t startBit;
  Pgn * pgn;
  int      repetition = 1;
  uint16_t valueu16;
  uint32_t valueu32;
  uint16_t currentDate = UINT16_MAX;
  uint32_t currentTime = UINT32_MAX;
  char fieldName[60];
  bool r;
  bool matchedFixedField;
  bool hasFixedField;
  uint32_t refPgn = 0;

  if (!device[msg->src])
  {
    return false;
  }
  dataStart = device[msg->src]->packetList[index].data;
  if (!dataStart)
  {
    return false;
  }
  size = device[msg->src]->packetList[index].size;
  dataEnd = dataStart + size;

  for (;(index < ARRAY_SIZE(pgnList)) && (msg->pgn == pgnList[index].pgn); index++)
  {
    matchedFixedField = true;
    hasFixedField = false;

    /* There is a next index that we can use as well. We do so if the 'fixed' fields don't match */

    pgn = &pgnList[index];

    for (i = 0, startBit = 0, data = dataStart; i < pgn->fieldCount; i++)
    {
      field = pgn->fieldList[i];
      if (!field.name || !field.size)
      {
        break;
      }

      bits = field.size;

      if (field.units && field.units[0] == '=')
      {
        int64_t value, desiredValue;
        int64_t maxValue;

        hasFixedField = true;
        extractNumber(&field, data, startBit, field.size, &value, &maxValue);
        desiredValue = strtol(field.units + 1, 0, 10);
        if (value != desiredValue)
        {
          matchedFixedField = false;
          break;
        }
      }
      startBit += bits;
      data += startBit / 8;
      startBit %= 8;
    }
    if (! hasFixedField || (hasFixedField && matchedFixedField))
    {
      break;
    }
  }

  if ((index >= ARRAY_SIZE(pgnList)) || (msg->pgn != pgnList[index].pgn))
  {
    index = 0;
  }

  pgn = &pgnList[index];

  if (showData)
  {
    FILE * f = stdout;
    char c = ' ';

    if (showJson)
    {
      f = stderr;
    }

    fprintf(f, "%s %u %3u %3u %6u %s: ", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    for (i = 0; i < size; i++)
    {
      fprintf(f, " %2.02X", dataStart[i]);
    }
    putc('\n', f);

    fprintf(f, "%s %u %3u %3u %6u %s: ", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    for (i = 0; i < size; i++)
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
    mprintf("{\"timestamp\":\"%s\",\"prio\":%u,\"src\":%u,\"dst\":%u,\"pgn\":%u,\"description\":\"%s\"", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    braceCount = 1;
    sep = ",\"fields\":{";
  }
  else
  {
    mprintf("%s %u %3u %3u %6u %s:", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    sep = " ";
  }

  for (i = 0, startBit = 0, data = dataStart; data < dataEnd; i++)
  {
    r = true;
    field = pgn->fieldList[i];
    if (!field.name)
    {
      if (pgn->repeatingFields)
      {
        i = i - pgn->repeatingFields;
        field = pgn->fieldList[i];
        repetition++;
      }
      else
      {
        break;
      }
    }

    strcpy(fieldName, field.camelName ? field.camelName : field.name);
    if (repetition > 1)
    {
      strcat(fieldName, field.camelName ? "_" : " ");
      sprintf(fieldName + strlen(fieldName), "%u", repetition);
    }

    bits  = field.size;
    bytes = (bits + 7) / 8;
    bytes = min(bytes, (size_t) (dataEnd - data));
    bits  = min(bytes * 8, bits);

    if (showBytes)
    {
      mprintf("\ndecode %s offset=%u startBit=%u bits=%u bytes=%u:", field.name, data - dataStart, startBit, bits, bytes);
    }

    if (strcmp(fieldName, "PGN") == 0)
    {
      refPgn = data[0] + (data[1] << 8) + (data[2] << 16);
      if (showBytes)
      {
        mprintf("refPgn=%u ", refPgn);
      }
    }

    if (field.resolution < 0.0)
    {
      int len;
      int k;

      /* These fields have only been found to start on byte boundaries,
       * making their location easier
       */
      if (field.resolution == RES_STRINGLZ)
      {
        len = *data++;
        bytes--;
        goto ascii_string;
      }

      if (field.resolution == RES_ASCII)
      {
        len = (int) bytes;
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
          print_json_escaped(data, len);
        } else
        {
          for (k = 0; k < len; k++)
          {
            if (data[k] >= ' ' && data[k] <= '~')
            {
              int c = data[k];

              if (showJson && (c == '\\'))
              {
                mprintf("%c", c);
              }
              mprintf("%c", c);
            }
          }
        }

        if (showJson)
        {
          mprintf("\"");
        }

      }
      else if (field.resolution == RES_STRING)
      {
        int len;
        if (*data == 0x02)
        {
          data++;
          for (len = 0; data + len < dataEnd && data[len] != 0x01; len++);
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
          len = 0;
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
      else if (field.resolution == RES_LONGITUDE || field.resolution == RES_LATITUDE)
      {
        printLatLon(fieldName, field.resolution, data, bytes);
      }
      else if (field.resolution == RES_DATE)
      {
        memcpy((void *) &valueu16, data, 2);
        printDate(fieldName, valueu16);
        currentDate = valueu16;
      }
      else if (field.resolution == RES_TIME)
      {
        memcpy((void *) &valueu32, data, 4);
        printTime(fieldName, valueu32);
        currentTime = valueu32;
      }
      else if (field.resolution == RES_TEMPERATURE)
      {
        memcpy((void *) &valueu16, data, 2);
        printTemperature(fieldName, valueu16);
      }
      else if (field.resolution == RES_PRESSURE)
      {
        memcpy((void *) &valueu16, data, 2);
        printPressure(fieldName, valueu16, &field);
      }
      else if (field.resolution == RES_6BITASCII)
      {
        print6BitASCIIText(fieldName, data, startBit, bits);
      }
      else if (bits == LEN_VARIABLE)
      {
        printVarNumber(fieldName, pgn, refPgn, &field, data, startBit, &bits);
      }
      else if (bits > BYTES(8))
      {
        printHex(fieldName, data, startBit, bits);
      }
      else if (field.resolution == RES_INTEGER
            || field.resolution == RES_LOOKUP
            || field.resolution == RES_BINARY
            || field.resolution == RES_MANUFACTURER
              )
      {
        printNumber(fieldName, &field, data, startBit, bits);
      }
      else
      {
        logError("Unknown resolution %f for %s\n", field.resolution, fieldName);
      }
    }
    else if (field.resolution > 0.0)
    {
      printNumber(fieldName, &field, data, startBit, bits);
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
    for (;braceCount > 0; braceCount--)
    {
      mprintf("}");
    }
  }
  mprintf("\n");

  if (msg->pgn == 126992 && currentDate < UINT16_MAX && currentTime < UINT32_MAX && clockSrc == msg->src)
  {
    setSystemClock(currentDate, currentTime);
  }
  return r;
}

void printPacket(size_t index, RawMessage * msg)
{
  size_t fastPacketIndex;
  size_t bucket;
  Packet * packet;
  Pgn * pgn = &pgnList[index];
  size_t subIndex;

  if (!device[msg->src])
  {
    heapSize += sizeof(DevicePackets);
    if (showBytes)
    {
      logInfo("New device at address %u (heap %zu bytes)\n", msg->src, heapSize);
    }
    device[msg->src] = calloc(1, sizeof(DevicePackets));
    if (!device[msg->src])
    {
      die("Out of memory\n");
    }
  }
  packet = &(device[msg->src]->packetList[index]);

  if (!packet->data)
  {
    packet->allocSize = max(min(pgn->size, 8) + FASTPACKET_BUCKET_N_SIZE, msg->len);
    heapSize += packet->allocSize;
    logInfo("New PGN %u for device %u (heap %zu bytes)\n", pgn->pgn, msg->src, heapSize);
    packet->data = malloc(packet->allocSize);
    if (!packet->data)
    {
      die("Out of memory\n");
    }
  }

  if (msg->len > 0x8 || format != RAWFORMAT_PLAIN)
  {
    if (packet->allocSize < msg->len)
    {
      heapSize += msg->len - packet->allocSize;
      packet->data = realloc(packet->data, msg->len);
      logDebug("Resizing buffer for PGN %u device %u to accomodate %u bytes (heap %zu bytes)\n", pgn->pgn, msg->src, msg->len, heapSize);
      packet->data = realloc(packet->data, msg->len);
      if (!packet->data)
      {
        die("Out of memory\n");
      }
      packet->allocSize = msg->len;
    }
    memcpy( packet->data
          , msg->data
          , msg->len
          );
    packet->size = msg->len;
  }
  else if (pgn->size > 0x8)
  {
    fastPacketIndex = msg->data[FASTPACKET_INDEX];
    bucket = fastPacketIndex & FASTPACKET_MAX_INDEX;

    if (bucket == 0)
    {
      size_t newSize = msg->data[FASTPACKET_SIZE] + FASTPACKET_BUCKET_N_SIZE;

      if (packet->allocSize < newSize)
      {
        heapSize += newSize - packet->allocSize;
        logDebug("Resizing buffer for PGN %u device %u to accomodate %zu bytes (heap %zu bytes)\n", pgn->pgn, msg->src, newSize, heapSize);
        packet->data = realloc(packet->data, newSize);
        if (!packet->data)
        {
          die("Out of memory\n");
        }
        packet->allocSize = newSize;
      }
      packet->size = msg->data[FASTPACKET_SIZE];
      memcpy( packet->data
            , msg->data + FASTPACKET_BUCKET_0_OFFSET
            , FASTPACKET_BUCKET_0_SIZE
            );
    }
    else
    {
      if (packet->lastFastPacket + 1 != fastPacketIndex)
      {
        logError("PGN %u malformed packet for %u received; expected %zu but got %zu\n"
                , pgn->pgn, msg->src, packet->lastFastPacket + 1, fastPacketIndex
                );
        return;
      }
      memcpy( packet->data + FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * (bucket - 1)
            , msg->data + FASTPACKET_BUCKET_N_OFFSET
            , FASTPACKET_BUCKET_N_SIZE
            );
    }
    packet->lastFastPacket = fastPacketIndex;

    if (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * bucket < packet->size)
    {
      /* Packet is not complete yet */
      return;
    }
  }
  else /* msg->len <= 8 && pgn->size <= 0x8 */
  {
    packet->size = msg->len;
    memcpy( packet->data
          , msg->data
          , msg->len
          );
  }

  /*
   * If there are multiple PGN descriptions we have to find the matching
   * one.
   */
  subIndex = index;
  for (subIndex = index; subIndex < ARRAY_SIZE(pgnList) && (msg->pgn == pgnList[subIndex].pgn || !index); subIndex++)
  {
    if (printPgn(index, subIndex, msg)) /* Only the really matching ones will actually return true */
    {
      if (index != subIndex)
      {
        logDebug("PGN %d matches version %zu\n", msg->pgn, subIndex - index);
      }
      mwrite(stdout);
      break;
    }
    else
    {
      mreset();
    }
  }
}

bool printCanFormat(RawMessage * msg)
{
  size_t i;

  if (onlySrc >=0 && onlySrc != msg->src)
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
          printPacket(i, msg);
          return true;
        }
        continue;
      }
      if (!pgnList[i].size)
      {
        return true; /* Determine size by raw packet first */
      }
      /*
       * Found the pgn that matches this particular packet
       */
      printPacket(i, msg);
      return true;
    }
  }
  if (!onlyPgn && (i == ARRAY_SIZE(pgnList)))
  {
    printPacket(0, msg);
  }
  return onlyPgn > 0;
}


static void explainPGN(Pgn pgn)
{
  int i;

  printf("PGN: %d / %08o / %05X - %u - %s\n\n", pgn.pgn, pgn.pgn, pgn.pgn, pgn.size, pgn.description);

  if (pgn.repeatingFields)
  {
    printf("     The last %u fields repeat until the data is exhausted.\n\n", pgn.repeatingFields);
  }
  for (i = 0; i < ARRAY_SIZE(pgn.fieldList) && pgn.fieldList[i].name; i++)
  {
    Field f = pgn.fieldList[i];
    printf("  Field #%d: %s%s%s\n", i + 1, f.name
      , f.name[0] && (f.description && f.description[0] && f.description[0] != ',') ? " - " : ""
      , (!f.description || f.description[0] == ',') ? "" :  f.description);
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
    else if (f.units && strcmp(f.units, "deg/s") == 0)
    {
      printf("                  Units: rad/s\n");
    }
    else if (f.resolution == RES_DEGREES
     || f.resolution == RES_DEGREES * 0.0001)
    {
      printf("                  Units: rad\n");
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
    else if (f.resolution == RES_DEGREES
          || f.resolution == RES_DEGREES * 0.0001)
    {
      printf("                  Type: Angle\n");
      printf("                  Resolution: %g\n", f.resolution / RadianToDegree);
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

    if (f.resolution == RES_LOOKUP && f.units && f.units[0] == ',')
    {
      char * s, * e;

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

static void explainPGNXML(Pgn pgn)
{
  int i;
  unsigned bitOffset = 0;
  char * p;

  printf("    <PGNInfo>\n"
         "       <PGN>%u</PGN>\n"
         "       <Id>%s</Id>\n"
         "       <Description>"
         , pgn.pgn
         , pgn.camelDescription
         );

  for (p = pgn.description; p && *p; p++)
  {
    if (*p != '&')
    {
      putchar(*p);
    }
    else
    {
      fputs("&amp;", stdout);
    }
  }

  printf("</Description>\n"
         "       <Complete>%s</Complete>\n"
         "       <Length>%u</Length>\n"
         "       <RepeatingFields>%u</RepeatingFields>\n"
         , (pgn.known ? "true" : "false")
         , pgn.size
         , pgn.repeatingFields
         );



  if (pgn.fieldList[0].name)
  {
    printf("       <Fields>\n");

    for (i = 0; i < ARRAY_SIZE(pgn.fieldList) && pgn.fieldList[i].name; i++)
    {
      Field f = pgn.fieldList[i];


      printf("         <Field>\n"
             "           <Order>%d</Order>\n"
             "           <Id>%s</Id>\n"
             "           <Name>%s</Name>\n", i + 1, f.camelName, f.name);

      if (f.description && f.description[0] && f.description[0] != ',')
      {
        printf("           <Description>%s</Description>\n", f.description);
      }
      printf("           <BitLength>%u</BitLength>\n", f.size);
      printf("           <BitOffset>%u</BitOffset>\n", bitOffset);
      printf("           <BitStart>%u</BitStart>\n", bitOffset % 8);
      bitOffset = bitOffset + f.size;

      if (f.units && f.units[0] == '=')
      {
        printf("           <Match>%s</Match>\n", &f.units[1]);
      }
      else if (f.units && strcmp(f.units, "deg/s") == 0)
      {
        printf("           <Units>rad/s</Units>\n");
      }
      else if (f.resolution == RES_DEGREES
       || f.resolution == RES_DEGREES * 0.0001)
      {
        printf("           <Units>rad</Units>\n");
      }
      else if (f.units && f.units[0] != ',')
      {
        printf("           <Units>%s</Units>\n", f.units);
      }

      if (f.resolution < 0.0)
      {
        Resolution t = types[-1 * (int) f.resolution - 1];
        if (t.name)
        {
          printf("                  <Type>%s</Type>\n", t.name);
        }
        if (t.resolution)
        {
          printf("                  <Resolution>%s</Resolution>\n", t.resolution);
        }
        else if (f.resolution == RES_LATITUDE || f.resolution == RES_LONGITUDE)
        {
          if (f.size == BYTES(8))
          {
            printf("                  <Resolution>%.16f</Resolution>\n", 1e-16);
          }
          else
          {
            printf("                  <Resolution>%.7f</Resolution>\n", 1e-7);
          }
        }
      }
      else if (f.resolution == RES_DEGREES)
      {
        printf("           <Type>Angle</Type>\n");
        printf("           <Resolution>%g</Resolution>\n", f.resolution / RadianToDegree);
      }
      else if (f.resolution != 1.0)
      {
        printf("           <Resolution>%g</Resolution>\n", f.resolution);
      }
      printf("           <Signed>%s</Signed>\n", f.hasSign ? "true" : "false");
      if (f.offset != 0)
      {
        printf("           <Offset>%d</Offset>\n", f.offset);
      }

      if (f.resolution == RES_LOOKUP && f.units && f.units[0] == ',')
      {
        char * s, * e, * p;

        printf("           <EnumValues>\n");

        for (s = f.units + 1;; s = e + 1)
        {
          e = strchr(s, ',');
          e = e ? e : s + strlen(s);
          p = strchr(s, '=');
          if (p)
          {
            printf("             <EnumPair Value='%.*s' Name='%.*s' />\n", (int) (p - s), s, (int) (e - (p + 1)), p + 1);
          }
          if (!*e)
          {
            break;
          }
        }

        printf("           </EnumValues>\n");
      }
      printf("         </Field>\n");
    }
    printf("       </Fields>\n");
  }
  printf("    </PGNInfo>\n");
}

void explain(void)
{
  int i;

  printf(COPYRIGHT"\n\nThis program can understand a number of N2K messages. What follows is an explanation of the messages\n"
         "that it understands. First is a list of completely understood messages, as far as I can tell.\n"
         "What follows is a list of messages that contain fields that have unknown content or size, or even\n"
         "completely unknown fields. If you happen to know more, please tell me!\n\n");
  printf("_______ Complete PGNs _________\n\n");
  for (i = 1; i < ARRAY_SIZE(pgnList); i++)
  {
    if (pgnList[i].known == true && pgnList[i].pgn < ACTISENSE_BEM)
    {
      explainPGN(pgnList[i]);
    }
  }
  printf("_______ Incomplete PGNs _________\n\n");
  for (i = 1; i < ARRAY_SIZE(pgnList); i++)
  {
    if (pgnList[i].known == false && pgnList[i].pgn < ACTISENSE_BEM)
    {
      explainPGN(pgnList[i]);
    }
  }

}

char * camelize(const char *str, bool upperCamelCase)
{
  size_t len = strlen(str);
  char *ptr = malloc(len + 1);
  char *s = ptr;
  bool lastIsAlpha = !upperCamelCase;

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
        *s = toupper(*str);
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

void camelCase(bool upperCamelCase)
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

void explainXML(void)
{
  int i;

  printf("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
    "<!--\n"COPYRIGHT"\n-->\n"
    "<PGNDefinitions xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\" Version=\"0.1\">\n"
    "  <Date>"PROGRAM_DATE"</Date>\n"
    "  <Comment>See https://github.com/canboat/canboat for the full source code</Comment>\n"
    "  <CreatorCode>Canboat NMEA2000 Analyzer</CreatorCode>\n"
    "  <License>GPL v3</License>\n"
    "  <PGNs>\n"
    );

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
