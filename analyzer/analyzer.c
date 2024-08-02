/*

Analyzes NMEA 2000 PGNs.

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

#define GLOBALS
#include "analyzer.h"

#include "parse.h"

enum RawFormats
{
  RAWFORMAT_UNKNOWN,
  RAWFORMAT_PLAIN,
  RAWFORMAT_FAST,
  RAWFORMAT_PLAIN_OR_FAST,
  RAWFORMAT_PLAIN_MIX_FAST,
  RAWFORMAT_AIRMAR,
  RAWFORMAT_CHETCO,
  RAWFORMAT_GARMIN_CSV1,
  RAWFORMAT_GARMIN_CSV2,
  RAWFORMAT_YDWG02,
  RAWFORMAT_ACTISENSE_N2K_ASCII
};

enum RawFormats format = RAWFORMAT_UNKNOWN;

const char *RAW_FORMAT_STR[] = {"UNKNOWN",
                                "PLAIN",
                                "FAST",
                                "PLAIN_OR_FAST",
                                "PLAIN_MIX_FAST",
                                "AIRMAR",
                                "CHETCO",
                                "GARMIN_CSV1",
                                "GARMIN_CSV2",
                                "YDWG02",
                                "ACTISENSE_N2K_ASCII"};

enum MultiPackets
{
  MULTIPACKETS_COALESCED,
  MULTIPACKETS_SEPARATE
};

enum MultiPackets multiPackets = MULTIPACKETS_SEPARATE;

typedef struct
{
  size_t   size;
  uint8_t  data[FASTPACKET_MAX_SIZE];
  uint32_t frames;    // Bit is one when frame is received
  uint32_t allFrames; // Bit is one when frame needs to be present
  int      pgn;
  int      src;
  bool     used;
} Packet;

#define REASSEMBLY_BUFFER_SIZE (64)

Packet reassemblyBuffer[REASSEMBLY_BUFFER_SIZE];

bool       showRaw       = false;
bool       showData      = false;
bool       showBytes     = false;
bool       showAllBytes  = false;
bool       showJson      = false;
bool       showJsonEmpty = false;
bool       showJsonValue = false;
bool       showVersion   = true;
bool       showSI        = false; // Output everything in strict SI units
GeoFormats showGeo       = GEO_DD;

char *sep = " ";
char  closingBraces[16]; // } and ] chars to close sentence in JSON mode, otherwise empty string

int    onlyPgn  = 0;
int    onlySrc  = -1;
int    onlyDst  = -1;
int    clockSrc = -1;
size_t heapSize = 0;

int g_variableFieldRepeat[2]; // Actual number of repetitions
int g_variableFieldIndex;

static uint16_t currentDate = UINT16_MAX;
static uint32_t currentTime = UINT32_MAX;

static enum RawFormats detectFormat(const char *msg);
static void            printCanFormat(RawMessage *msg);
static bool            printField(const Field   *field,
                                  const char    *fieldName,
                                  const uint8_t *data,
                                  size_t         dataLen,
                                  size_t         startBit,
                                  size_t        *bits);
static void            printCanRaw(const RawMessage *msg);
static void            showBuffers(void);
static unsigned int    getMessageByteCount(const char *const msg);

static void usage(char **argv, char **av)
{
  printf("Unknown or invalid argument %s\n", av[0]);
  printf("Usage: %s [[-raw] [-json [-empty] [-nv] [-camel | -upper-camel]] [-data] [-debug] [-d] [-q] [-si] [-geo {dd|dm|dms}] "
         "-format <fmt> "
         "[-src <src> | -dst <dst> | <pgn>]] ["
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
  printf("     -format <fmt>     Select a particular format, either: ");
  for (size_t i = 1; i < ARRAY_SIZE(RAW_FORMAT_STR); i++)
  {
    printf("%s, ", RAW_FORMAT_STR[i]);
  }
  printf("\n");
  printf("     -version          Print the version of the program and quit\n");
  printf("\nThe following options are used to debug the analyzer:\n");
  printf("     -raw              Print the PGN in a format suitable to be fed to analyzer again (in standard raw format)\n");
  printf("     -data             Print the PGN three times: in hex, ascii and analyzed\n");
  printf("     -debug            Print raw value per field\n");
  printf("     -debugdata        Print raw value per pgn\n");
  printf("     -fixtime str      Print str as timestamp in logging\n");
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
    else if (strcasecmp(av[1], "-schema-version") == 0)
    {
      printf("%s\n", SCHEMA_VERSION);
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
      showJsonEmpty = true;
      showBytes     = true;
    }
    else if (strcasecmp(av[1], "-debugdata") == 0)
    {
      showJsonEmpty = true;
      showAllBytes  = true;
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
    else if (ac > 2 && strcasecmp(av[1], "-fixtime") == 0)
    {
      setFixedTimestamp(av[2]);
      if (strstr(av[2], "n2kd") == NULL)
      {
        showVersion = false;
      }
      ac--;
      av++;
    }
    else if (ac > 2 && strcasecmp(av[1], "-src") == 0)
    {
      onlySrc = strtol(av[2], 0, 10);
      ac--;
      av++;
    }
    else if (ac > 2 && strcasecmp(av[1], "-dst") == 0)
    {
      onlyDst = strtol(av[2], 0, 10);
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
        logAbort("Cannot open file %s\n", av[2]);
      }
      ac--;
      av++;
    }
    else if (ac > 2 && strcasecmp(av[1], "-format") == 0)
    {
      for (size_t i = 1; i < ARRAY_SIZE(RAW_FORMAT_STR); i++)
      {
        if (strcasecmp(av[2], RAW_FORMAT_STR[i]) == 0)
        {
          format = (enum RawFormats) i;
          if (format != RAWFORMAT_PLAIN && format != RAWFORMAT_PLAIN_OR_FAST && format != RAWFORMAT_PLAIN_MIX_FAST
              && format != RAWFORMAT_YDWG02)
          {
            multiPackets = MULTIPACKETS_COALESCED;
          }
          break;
        }
      }
      if (format == RAWFORMAT_UNKNOWN)
      {
        logAbort("Unknown message format '%s'\n", av[2]);
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
  else if (showVersion)
  {
    printf("{\"version\":\"%s\",\"units\":\"%s\",\"showLookupValues\":%s}\n",
           VERSION,
           showSI ? "si" : "std",
           showJsonValue ? "true" : "false");
  }

  fillLookups();
  fillFieldType(true);
  checkPgnList();

  while (fgets(msg, sizeof(msg) - 1, file))
  {
    RawMessage m;

    if (*msg == 0 || *msg == '\r' || *msg == '\n' || *msg == '#')
    {
      if (*msg == '#')
      {
        if (strncmp(msg + 1, "SHOWBUFFERS", STRSIZE("SHOWBUFFERS")) == 0)
        {
          showBuffers();
        }
      }

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
      case RAWFORMAT_PLAIN_OR_FAST:
        if (getMessageByteCount(msg) <= 8)
        {
          r = parseRawFormatPlain(msg, &m, showJson);
          logDebug("plain_or_fast: plain r=%d\n", r);
        }
        else
        {
          r = parseRawFormatFast(msg, &m, showJson);
          if (r >= 0)
          {
            format       = RAWFORMAT_FAST;
            multiPackets = MULTIPACKETS_COALESCED;
            logDebug("plain_or_fast: fast r=%d\n", r);
          }
        }
        break;

      case RAWFORMAT_PLAIN_MIX_FAST:
        if (getMessageByteCount(msg) <= 8)
        {
          r = parseRawFormatPlain(msg, &m, showJson);
          logDebug("plain_or_fast: plain r=%d\n", r);
        }
        else
        {
          r = parseRawFormatFast(msg, &m, showJson);
          logDebug("plain_or_fast: fast r=%d\n", r);
        }
        break;

      case RAWFORMAT_PLAIN:
        r = parseRawFormatPlain(msg, &m, showJson);
        break;

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

      case RAWFORMAT_ACTISENSE_N2K_ASCII:
        r = parseRawFormatActisenseN2KAscii(msg, &m, showJson);
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
      logError("Unknown message error %d: '%s'\n", r, msg);
    }
  }

  return 0;
}

static unsigned int getMessageByteCount(const char *const msg)
{
  const char  *p;
  int          r;
  unsigned int len;

  p = strchr(msg, ',');
  if (p)
  {
    r = sscanf(p, ",%*u,%*u,%*u,%*u,%u,%*x,%*x,%*x,%*x,%*x,%*x,%*x,%*x,%*x", &len);
    if (r >= 1)
    {
      return len;
    }
  }
  return 0;
}

static enum RawFormats detectFormat(const char *const msg)
{
  const char  *p;
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

  len = getMessageByteCount(msg);
  if (len > 0)
  {
    if (len > 8)
    {
      logInfo("Detected FAST format with all frames on one line\n");
      multiPackets = MULTIPACKETS_COALESCED;
      return RAWFORMAT_FAST;
    }
    logInfo("Assuming PLAIN_OR_FAST format with one line per frame or one line per message\n");
    return RAWFORMAT_PLAIN_OR_FAST;
  }

  {
    int  a, b, c, d, f;
    char e;
    if (sscanf(msg, "%d:%d:%d.%d %c %02X ", &a, &b, &c, &d, &e, &f) == 6 && (e == 'R' || e == 'T'))
    {
      logInfo("Detected YDWG-02 protocol with one line per frame\n");
      multiPackets = MULTIPACKETS_SEPARATE;
      return RAWFORMAT_YDWG02;
    }
  }

  {
    int a, b, c, d;
    if (sscanf(msg, "A%d.%d %x %x ", &a, &b, &c, &d) == 4 || sscanf(msg, "A%d %x %x ", &a, &b, &c) == 3)
    {
      logInfo("Detected Actisense N2K Ascii protocol with all frames on one line\n");
      multiPackets = MULTIPACKETS_COALESCED;
      return RAWFORMAT_ACTISENSE_N2K_ASCII;
    }
  }

  return RAWFORMAT_UNKNOWN;
}

static void printCanRaw(const RawMessage *msg)
{
  size_t i;
  FILE  *f = stdout;

  if (onlySrc >= 0 && onlySrc != msg->src)
  {
    return;
  }
  if (onlyDst >= 0 && onlyDst != msg->dst)
  {
    return;
  }
  if (onlyPgn > 0 && onlyPgn != msg->pgn)
  {
    return;
  }

  if (showJson)
  {
    f = stderr;
  }

  if (showRaw && (!onlyPgn || onlyPgn == msg->pgn))
  {
    fprintf(f, "%s,%u,%u,%u,%u,%u", msg->timestamp, msg->prio, msg->pgn, msg->src, msg->dst, msg->len);
    for (i = 0; i < msg->len; i++)
    {
      fprintf(f, ",%02x", msg->data[i]);
    }
    putc('\n', f);
  }
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
    logDebug("Set system clock to %" PRIu64 "/%06u\n", (uint64_t) gps.tv_sec, gps.tv_usec);
    return;
  }

#ifdef HAS_ADJTIME

  delta.tv_sec  = 0;
  delta.tv_usec = gps.tv_usec - now.tv_usec + microspersecond * (gps.tv_sec - now.tv_sec);

  if (delta.tv_usec < 2000 && delta.tv_usec > -2000)
  {
    logDebug("Forget about small system clock skew %d\n", delta.tv_usec);
    return;
  }

  if (adjtime(&delta, &olddelta))
  {
    logError("Failed to adjust system clock by %d usec\n", delta.tv_usec);
    return;
  }

  if (isLogLevelEnabled(LOG_DEBUG))
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

static void showBuffers(void)
{
  size_t  buffer;
  Packet *p;

  for (buffer = 0; buffer < REASSEMBLY_BUFFER_SIZE; buffer++)
  {
    p = &reassemblyBuffer[buffer];

    if (p->used)
    {
      logError("ReassemblyBuffer[%zu] PGN %u: size %zu frames=%x mask=%x\n", buffer, p->pgn, p->size, p->frames, p->allFrames);
    }
    else
    {
      logDebug("ReassemblyBuffer[%zu]: inUse=false\n", buffer);
    }
  }
}

static void printCanFormat(RawMessage *msg)
{
  const Pgn *pgn;
  size_t     buffer;
  Packet    *p;

  if (onlySrc >= 0 && onlySrc != msg->src)
  {
    return;
  }
  if (onlyDst >= 0 && onlyDst != msg->dst)
  {
    return;
  }
  if (onlyPgn > 0 && onlyPgn != msg->pgn)
  {
    return;
  }

  pgn = searchForPgn(msg->pgn);
  if (multiPackets == MULTIPACKETS_SEPARATE && pgn == NULL)
  {
    pgn = searchForUnknownPgn(msg->pgn);
  }
  if (multiPackets == MULTIPACKETS_COALESCED || !pgn || pgn->type != PACKET_FAST || msg->len > 8)
  {
    // No reassembly needed
    printPgn(msg, msg->data, msg->len, showData, showJson);
    return;
  }

  // Fast packet requires re-asssembly
  // We only get here if we know for sure that the PGN is fast-packet
  // Possibly it is of unknown length when the PGN is unknown.

  for (buffer = 0; buffer < REASSEMBLY_BUFFER_SIZE; buffer++)
  {
    p = &reassemblyBuffer[buffer];

    if (p->used && p->pgn == msg->pgn && p->src == msg->src)
    {
      // Found existing slot
      break;
    }
  }
  if (buffer == REASSEMBLY_BUFFER_SIZE)
  {
    // Find a free slot
    for (buffer = 0; buffer < REASSEMBLY_BUFFER_SIZE; buffer++)
    {
      p = &reassemblyBuffer[buffer];
      if (!p->used)
      {
        break;
      }
    }
    if (buffer == REASSEMBLY_BUFFER_SIZE)
    {
      logError("Out of reassembly buffers; ignoring PGN %u\n", msg->pgn);
      return;
    }
    p->used   = true;
    p->src    = msg->src;
    p->pgn    = msg->pgn;
    p->frames = 0;
  }

  {
    // YDWG can receive frames out of order, so handle this.
    uint32_t frame    = msg->data[0] & 0x1f;
    uint32_t seq      = msg->data[0] & 0xe0;
    size_t   idx      = (frame == 0) ? 0 : FASTPACKET_BUCKET_0_SIZE + (frame - 1) * FASTPACKET_BUCKET_N_SIZE;
    size_t   frameLen = (frame == 0) ? FASTPACKET_BUCKET_0_SIZE : FASTPACKET_BUCKET_N_SIZE;
    size_t   msgIdx   = (frame == 0) ? FASTPACKET_BUCKET_0_OFFSET : FASTPACKET_BUCKET_N_OFFSET;

    if ((p->frames & (1 << frame)) != 0)
    {
      logError("Received incomplete fast packet PGN %u from source %u\n", msg->pgn, msg->src);
      p->frames = 0;
    }

    if (frame == 0 && p->frames == 0)
    {
      p->size      = msg->data[1];
      p->allFrames = (1 << (1 + (p->size / 7))) - 1;
    }

    memcpy(&p->data[idx], &msg->data[msgIdx], frameLen);
    p->frames |= 1 << frame;

    logDebug("Using buffer %u for reassembly of PGN %u: size %zu frame %u sequence %u idx=%zu frames=%x mask=%x\n",
             buffer,
             msg->pgn,
             p->size,
             frame,
             seq,
             idx,
             p->frames,
             p->allFrames);
    if (p->frames == p->allFrames)
    {
      // Received all data
      printPgn(msg, p->data, p->size, showData, showJson);
      p->used   = false;
      p->frames = 0;
    }
  }
}

static void showBytesOrBits(const uint8_t *data, size_t startBit, size_t bits)
{
  int64_t     value;
  int64_t     maxValue;
  size_t      i;
  size_t      remaining_bits;
  const char *s;
  uint8_t     byte;

  if (showJson)
  {
    size_t location = mlocation();

    if (location == 0 || mchr(location - 1) != '{')
    {
      mprintf(",");
    }
    mprintf("\"bytes\":\"");
  }
  else
  {
    mprintf(" (bytes = \"");
  }
  remaining_bits = bits;
  s              = "";
  for (i = 0; i < (bits + 7) >> 3; i++)
  {
    uint8_t byte = data[i];

    if (i == 0 && startBit != 0)
    {
      byte = byte >> startBit; // Shift off older bits
      if (remaining_bits + startBit < 8)
      {
        byte = byte & ((1 << remaining_bits) - 1);
      }
      byte = byte << startBit; // Shift zeros back in
      remaining_bits -= (8 - startBit);
    }
    else
    {
      if (remaining_bits < 8)
      {
        // only the lower remaining_bits should be used
        byte = byte & ((1 << remaining_bits) - 1);
      }
      remaining_bits -= 8;
    }
    mprintf("%s%2.02X", s, byte);
    s = " ";
  }
  mprintf("\"");

  if (startBit != 0 || ((bits & 7) != 0))
  {
    extractNumber(NULL, data, (bits + 7) >> 3, startBit, bits, &value, &maxValue);
    if (showJson)
    {
      mprintf(",\"bits\":\"");
    }
    else
    {
      mprintf(", bits = \"");
    }

    for (i = bits; i > 0;)
    {
      i--;
      byte = (value >> (i >> 3)) & 0xff;
      mprintf("%c", (byte & (1 << (i & 7))) ? '1' : '0');
    }
    mprintf("\"");
  }

  if (!showJson)
  {
    mprintf(")");
  }
}

static uint32_t g_refPgn = 0; // Remember this over the entire set of fields

static void fillGlobalsBasedOnFieldName(const char *fieldName, const uint8_t *data, size_t dataLen, size_t startBit, size_t bits)
{
  int64_t value;
  int64_t maxValue;

  if (strcmp(fieldName, "PGN") == 0)
  {
    extractNumber(NULL, data, dataLen, startBit, bits, &value, &maxValue);
    logDebug("Reference PGN = %" PRId64 "\n", value);
    g_refPgn = value;
    return;
  }

  if (strcmp(fieldName, "Length") == 0)
  {
    extractNumber(NULL, data, dataLen, startBit, bits, &value, &maxValue);
    logDebug("for next field: length = %" PRId64 "\n", value);
    g_length = value;
    return;
  }
}

static bool printField(const Field   *field,
                       const char    *fieldName,
                       const uint8_t *data,
                       size_t         dataLen,
                       size_t         startBit,
                       size_t        *bits)
{
  size_t bytes;
  double resolution;
  bool   r;

  if (fieldName == NULL)
  {
    fieldName = field->camelName ? field->camelName : (char *) field->name;
  }

  resolution = field->resolution;
  if (resolution == 0.0)
  {
    resolution = field->ft->resolution;
  }

  logDebug("PGN %u: printField(<%s>, \"%s\", ..., dataLen=%zu, startBit=%zu) resolution=%g\n",
           field->pgn->pgn,
           field->name,
           fieldName,
           dataLen,
           startBit,
           field->resolution);

  if (field->size != 0 || field->ft != NULL)
  {
    *bits = (field->size != 0) ? field->size : field->ft->size;
    bytes = (*bits + 7) / 8;
    bytes = min(bytes, dataLen - startBit / 8);
    *bits = min(bytes * 8, *bits);
  }
  else
  {
    bytes = 0;
    *bits = 0;
  }

  fillGlobalsBasedOnFieldName(field->name, data, dataLen, startBit, *bits);

  logDebug("PGN %u: printField <%s>, \"%s\": bits=%zu proprietary=%u refPgn=%u\n",
           field->pgn->pgn,
           field->name,
           fieldName,
           *bits,
           field->proprietary,
           g_refPgn);

  if (field->proprietary)
  {
    if ((g_refPgn >= 65280 && g_refPgn <= 65535) || (g_refPgn >= 126720 && g_refPgn <= 126975)
        || (g_refPgn >= 130816 && g_refPgn <= 131071))
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

  if (field->ft != NULL && field->ft->pf != NULL)
  {
    size_t location            = mlocation();
    char  *oldSep              = sep;
    size_t oldClosingBracesLen = strlen(closingBraces);
    size_t location2           = 0;
    size_t location3;

    if (field->ft->pf != fieldPrintVariable)
    {
      if (showJson)
      {
        mprintf("%s\"%s\":", getSep(), fieldName);
        sep = ",";
        if (showBytes || showJsonValue)
        {
          location2 = mlocation();
        }
      }
      else
      {
        mprintf("%s %s = ", getSep(), fieldName);
        sep = ";";
      }
    }
    location3 = mlocation();
    logDebug(
        "PGN %u: printField <%s>, \"%s\": calling function for %s\n", field->pgn->pgn, field->name, fieldName, field->fieldType);
    g_skip = false;
    r      = (field->ft->pf)(field, fieldName, data, dataLen, startBit, bits);
    // if match fails, r == false. If field is not printed, g_skip == true
    logDebug("PGN %u: printField <%s>, \"%s\": result %d bits=%zu\n", field->pgn->pgn, field->name, fieldName, r, *bits);
    if (r && !g_skip)
    {
      if (location3 == mlocation() && !showBytes)
      {
        logError("PGN %u: field \"%s\" print routine did not print anything\n", field->pgn->pgn, field->name);
        r = false;
      }
      else if (showBytes && field->ft->pf != fieldPrintVariable)
      {
        location3 = mlocation();
        if (mchr(location3 - 1) == '}')
        {
          mset(location3 - 1);
        }
        showBytesOrBits(data + (startBit >> 3), startBit & 7, *bits);
        if (showJson)
        {
          mprintf("}");
        }
      }
      if (location2 != 0)
      {
        location3 = mlocation();
        if (mchr(location3 - 1) == '}')
        {
          // Prepend {"value":
          minsert(location2, "{\"value\":");
        }
      }
    }
    if (!r || g_skip)
    {
      mset(location);
      sep                                = oldSep;
      closingBraces[oldClosingBracesLen] = '\0';
    }
    return r;
  }
  logError("PGN %u: no function found to print field '%s'\n", field->pgn->pgn, fieldName);
  return false;
}

bool printPgn(const RawMessage *msg, const uint8_t *data, int length, bool showData, bool showJson)
{
  const Pgn *pgn;

  size_t  i;
  size_t  bits;
  size_t  startBit;
  int     repetition;
  char    fieldName[60];
  bool    r;
  size_t  variableFields; // How many variable fields remain (product of repetition count * # of fields)
  uint8_t variableFieldStart;
  uint8_t variableFieldCount;

  if (msg == NULL)
  {
    return false;
  }
  pgn = getMatchingPgn(msg->pgn, data, length);
  if (!pgn)
  {
    logAbort("No PGN definition found for PGN %u\n", msg->pgn);
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
      fprintf(f, " %2.02X", data[i]);
    }
    putc('\n', f);

    fprintf(f, "%s %u %3u %3u %6u %s: ", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    for (i = 0; i < length; i++)
    {
      fprintf(f, "  %c", isalnum(data[i]) ? data[i] : '.');
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
    if (showAllBytes)
    {
      mprintf(",\"data\":\"");
      for (i = 0; i < length; i++)
      {
        mprintf("%2.02X", data[i]);
      }
      mprintf("\"");
    }
    strcpy(closingBraces, "}");
    sep = ",\"fields\":{";
  }
  else
  {
    mprintf("%s %u %3u %3u %6u %s:", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    sep = " ";
  }

  logDebug("fieldCount=%d repeatingStart1=%" PRIu8 "\n", pgn->fieldCount, pgn->repeatingStart1);

  g_variableFieldRepeat[0] = 255; // Can be overridden by '# of parameters'
  g_variableFieldRepeat[1] = 0;   // Can be overridden by '# of parameters'
  repetition               = 0;
  variableFields           = 0;
  r                        = true;
  for (i = 0, startBit = 0; (startBit >> 3) < length; i++)
  {
    const Field *field = &pgn->fieldList[i];

    if (variableFields == 0)
    {
      repetition = 0;
    }

    if (pgn->repeatingCount1 > 0 && field->order == pgn->repeatingStart1 && repetition == 0)
    {
      if (showJson)
      {
        mprintf("%s\"list\":[{", getSep());
        strcat(closingBraces, "]}");
        sep = "";
      }
      // Only now is g_variableFieldRepeat set
      variableFields     = pgn->repeatingCount1 * g_variableFieldRepeat[0];
      variableFieldCount = pgn->repeatingCount1;
      variableFieldStart = pgn->repeatingStart1;
      repetition         = 1;
    }
    if (pgn->repeatingCount2 > 0 && field->order == pgn->repeatingStart2 && repetition == 0)
    {
      if (showJson)
      {
        mprintf("}],\"list2\":[{");
        sep = "";
      }
      // Only now is g_variableFieldRepeat set
      variableFields     = pgn->repeatingCount2 * g_variableFieldRepeat[1];
      variableFieldCount = pgn->repeatingCount2;
      variableFieldStart = pgn->repeatingStart2;
      repetition         = 1;
    }

    if (variableFields > 0)
    {
      if (i + 1 == variableFieldStart + variableFieldCount)
      {
        i     = variableFieldStart - 1;
        field = &pgn->fieldList[i];
        repetition++;
        if (showJson)
        {
          mprintf("},{");
          sep = "";
        }
      }
      logDebug("variableFields: repetition=%d field=%" PRIu8 " variableFieldStart=%" PRIu8 " variableFieldCount=%" PRIu8
               " remaining=%zu\n",
               repetition,
               i + 1,
               variableFieldStart,
               variableFieldCount,
               variableFields);
      variableFields--;
    }

    if (!field->camelName && !field->name)
    {
      logDebug("PGN %u has unknown bytes at end: %u\n", msg->pgn, length - (startBit >> 3));
      break;
    }

    strcpy(fieldName, field->camelName ? field->camelName : field->name);
    if (repetition >= 1 && !showJson)
    {
      strcat(fieldName, field->camelName ? "_" : " ");
      sprintf(fieldName + strlen(fieldName), "%u", repetition);
    }

    if (!printField(field, fieldName, data, length, startBit, &bits))
    {
      r = false;
      break;
    }

    startBit += bits;
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
    if (variableFields > 0 && g_variableFieldRepeat[0] < UINT8_MAX)
    {
      logError("PGN %u has %zu missing fields in repeating set\n", msg->pgn, variableFields);
    }
  }
  else
  {
    if (!showJson)
    {
      mwrite(stdout);
    }
    mreset();
    logError("PGN %u analysis error\n", msg->pgn);
  }

  if (msg->pgn == 126992 && currentDate < UINT16_MAX && currentTime < UINT32_MAX && clockSrc == msg->src)
  {
    setSystemClock();
  }
  return r;
}

extern bool fieldPrintVariable(const Field   *field,
                               const char    *fieldName,
                               const uint8_t *data,
                               size_t         dataLen,
                               size_t         startBit,
                               size_t        *bits)
{
  const Field *refField;
  bool         r;

  refField = getField(g_refPgn, data[startBit / 8 - 1] - 1);
  if (refField)
  {
    logDebug("Field %s: found variable field %u '%s'\n", fieldName, g_refPgn, refField->name);
    r     = printField(refField, fieldName, data, dataLen, startBit, bits);
    *bits = (*bits + 7) & ~0x07; // round to bytes
    return r;
  }

  logError("Field %s: cannot derive variable length for PGN %d field # %d\n", fieldName, g_refPgn, data[-1]);
  *bits = 8; /* Gotta assume something */
  return false;
}
