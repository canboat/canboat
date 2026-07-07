/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

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
  uint8_t  seq;
  bool     used;
} Packet;

#define REASSEMBLY_BUFFER_SIZE (64)

Packet reassemblyBuffer[REASSEMBLY_BUFFER_SIZE];

// ISO 11783-3 Transport Protocol reassembly. Newer devices (a NEON GPS is the
// trigger for this code) wrap PGNs too large even for fast-packet's 223-byte
// ceiling (e.g. PGN 129540 with a large satellite list) in ISO TP instead:
// PGN 60416 (TP.CM) announces the transfer (BAM = broadcast, no ACK; RTS =
// addressed, with a CTS handshake we don't participate in as a passive
// monitor), PGN 60160 (TP.DT) carries the payload in 7-byte chunks with a
// 1-based sequence number. Both PGNs are already decoded individually
// elsewhere in pgn.h; here they are swallowed and replaced with a single
// synthesized frame for the target PGN, the same way fast-packet frames are
// reassembled above.
#define PGN_ISO_TP_CM (60416)
#define PGN_ISO_TP_DT (60160)
#define ISO_TP_CM_BAM (32)
#define ISO_TP_CM_RTS (16)
#define ISO_TP_CM_ABORT (255)
#define ISO_TP_SLOTS (16)

typedef struct
{
  bool     used;
  uint8_t  src;
  uint8_t  dst;
  uint8_t  prio;
  uint32_t targetPgn;
  size_t   totalSize;
  uint8_t  packets;
  uint32_t received[(ISOTP_MAX_PACKETS + 31) / 32]; // Bit n is one when sequence n+1 has been received
  char     timestamp[DATE_LENGTH];
  uint8_t  data[ISOTP_MAX_SIZE];
} TpSlot;

TpSlot tpSlotBuffer[ISO_TP_SLOTS];

bool       showRaw       = false;
bool       showData      = false;
bool       showBytes     = false;
bool       showAllBytes  = false;
bool       showJson      = false;
bool       showJsonEmpty = false;
bool       showJsonValue = false;
bool       showVersion   = true;
bool       fixedTime     = false; // -fixtime in effect (test mode)
bool       showSI        = false; // Output everything in strict SI units
bool       showCamel     = false;
GeoFormats showGeo       = GEO_DD;

char *sep = " ";
char  closingBraces[16]; // } and ] chars to close sentence in JSON mode, otherwise empty string

int    onlyPgnList[16];
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
static bool            isMsgAllowed(const RawMessage *msg);
static bool            isTargetPgnAllowed(uint32_t pgn);
static void            printCanFormat(RawMessage *msg);
static void            handleIsoTpCm(const RawMessage *msg);
static void            handleIsoTpDt(const RawMessage *msg);
static bool            printField(const Field   *field,
                                  const char    *fieldName,
                                  const uint8_t *data,
                                  size_t         dataLen,
                                  size_t         startBit,
                                  size_t        *bits,
                                  bool           allowKey);
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
  char   msg[MAX_MSG_LINE_LENGTH];
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
      showCamel = true;
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
      fixedTime = true;
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
      int prn = strtol(av[1], 0, 10);
      if (prn > 0 && onlyPgn < ARRAY_SIZE(onlyPgnList))
      {
        onlyPgnList[onlyPgn++] = prn;
        if (onlyPgn == 1)
        {
          logInfo("Only logging PGN %d\n", prn);
        }
        else
        {
          logInfo("and PGN %d\n", prn);
        }
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
    printf("{\"version\":\"%s\",\"units\":\"%s\",\"showLookupValues\":%s",
           VERSION,
           showSI ? "si" : "std",
           showJsonValue ? "true" : "false");
    if (showCamel)
    {
      printf(",\"showUniqueId\":true");
    }
    printf("}\n");
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
        else if (format == RAWFORMAT_UNKNOWN && strncmp(msg, CANBOAT_FORMAT_HEADER_PREFIX, STRSIZE(CANBOAT_FORMAT_HEADER_PREFIX)) == 0)
        {
          const char *fmt = msg + STRSIZE(CANBOAT_FORMAT_HEADER_PREFIX);
          for (size_t i = 1; i < ARRAY_SIZE(RAW_FORMAT_STR); i++)
          {
            if (strncasecmp(fmt, RAW_FORMAT_STR[i], strlen(RAW_FORMAT_STR[i])) == 0)
            {
              format = (enum RawFormats) i;
              if (format != RAWFORMAT_PLAIN && format != RAWFORMAT_PLAIN_OR_FAST && format != RAWFORMAT_PLAIN_MIX_FAST
                  && format != RAWFORMAT_YDWG02)
              {
                multiPackets = MULTIPACKETS_COALESCED;
              }
              logInfo("Format set to %s by header\n", RAW_FORMAT_STR[i]);
              break;
            }
          }
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

    logDebug("IN: %s\n", msg);

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
      if (isMsgAllowed(&m))
      {
        printCanFormat(&m);
        printCanRaw(&m);
      }
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

static bool isMsgAllowed(const RawMessage *msg)
{
  // The CANboat startup record embeds the build version, which changes on every
  // release. Drop it in test mode (-fixtime) so golden outputs stay stable.
  if (fixedTime && msg->pgn == CANBOAT_BEM)
  {
    return false;
  }
  if (onlySrc >= 0 && onlySrc != msg->src)
  {
    return false;
  }
  if (onlyDst >= 0 && onlyDst != msg->dst)
  {
    return false;
  }
  if (onlyPgn > 0)
  {
    // ISO Transport Protocol frames must always reach printCanFormat so the
    // reassembler can see them, even when the user filtered on a different
    // PGN - the target PGN is checked separately, via isTargetPgnAllowed(),
    // once a transfer completes.
    if (msg->pgn == PGN_ISO_TP_CM || msg->pgn == PGN_ISO_TP_DT)
    {
      return true;
    }
    for (int i = 0; i < onlyPgn; i++)
    {
      if (onlyPgnList[i] == msg->pgn)
      {
        return true;
      }
    }
    return false;
  }
  return true;
}

static bool isTargetPgnAllowed(uint32_t pgn)
{
  if (onlyPgn == 0)
  {
    return true;
  }
  for (int i = 0; i < onlyPgn; i++)
  {
    if (onlyPgnList[i] == (int) pgn)
    {
      return true;
    }
  }
  return false;
}

static void printCanRaw(const RawMessage *msg)
{
  size_t i;
  FILE  *f = stdout;
  char   ts[DATE_LENGTH];

  if (showJson)
  {
    f = stderr;
  }

  if (showRaw && (!onlyPgn || onlyPgn == msg->pgn))
  {
    normalizeTimestamp(msg->timestamp, ts, sizeof(ts));
    fprintf(f, "%s,%u,%u,%u,%u,%u", ts, msg->prio, msg->pgn, msg->src, msg->dst, msg->len);
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

  if (msg->pgn == PGN_ISO_TP_CM)
  {
    handleIsoTpCm(msg);
    return;
  }
  if (msg->pgn == PGN_ISO_TP_DT)
  {
    handleIsoTpDt(msg);
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

  uint32_t frame = msg->data[0] & 0x1f;
  uint32_t seq   = msg->data[0] & 0xe0;

  for (buffer = 0; buffer < REASSEMBLY_BUFFER_SIZE; buffer++)
  {
    p = &reassemblyBuffer[buffer];

    if (p->used && p->pgn == msg->pgn && p->src == msg->src && p->seq == seq)
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
    p->seq    = seq;
  }

  {
    // YDWG can receive frames out of order, so handle this.
    size_t idx      = (frame == 0) ? 0 : FASTPACKET_BUCKET_0_SIZE + (frame - 1) * FASTPACKET_BUCKET_N_SIZE;
    size_t frameLen = (frame == 0) ? FASTPACKET_BUCKET_0_SIZE : FASTPACKET_BUCKET_N_SIZE;
    size_t msgIdx   = (frame == 0) ? FASTPACKET_BUCKET_0_OFFSET : FASTPACKET_BUCKET_N_OFFSET;

    if ((p->frames & (UINT32_C(1) << frame)) != 0)
    {
      logError("Received incomplete fast packet PGN %u from source %u\n", msg->pgn, msg->src);
      p->frames = 0;
    }

    if (frame == 0)
    {
      // Frame 0 declares the payload size and thus the mask of required
      // frame indices. Frames already held in the slot are ambiguous: an
      // out-of-order retransmission must survive (see recombine-frames.in),
      // but the body of a previous burst whose frame 0 was lost must not —
      // completing against it would emit a payload gluing this frame 0
      // onto the previous message's body, and a size mismatch would leave
      // p->frames a strict superset of p->allFrames so the slot never
      // completes again. The tell is completion: a genuinely reordered
      // burst completes on a later index, stale leftovers would complete
      // the moment frame 0 lands. Discard held bits exactly when they
      // would complete the mask this frame declares. (allFrames == 1 is
      // exempt: a <= 6 byte payload legitimately completes on frame 0
      // alone, using no held data.)
      size_t   size = msg->data[1];
      uint32_t allFrames;

      if (size > FASTPACKET_MAX_SIZE)
      {
        size = FASTPACKET_MAX_SIZE;
      }
      allFrames = (uint32_t) ((UINT64_C(1) << (1 + (size / 7))) - 1);

      if (p->frames != 0 && allFrames != 1 && ((p->frames | UINT32_C(1)) & allFrames) == allFrames)
      {
        logError("Received incomplete fast packet PGN %u from source %u\n", msg->pgn, msg->src);
        p->frames = 0;
      }
      p->size      = size;
      p->allFrames = allFrames;
    }

    if (msg->len > msgIdx)
    {
      size_t available = msg->len - msgIdx;
      if (available < frameLen)
      {
        memcpy(&p->data[idx], &msg->data[msgIdx], available);
        memset(&p->data[idx + available], 0xff, frameLen - available);
      }
      else
      {
        memcpy(&p->data[idx], &msg->data[msgIdx], frameLen);
      }
    }
    else
    {
      memset(&p->data[idx], 0xff, frameLen);
    }
    p->frames |= UINT32_C(1) << frame;

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

// Handle a PGN 60416 TP.CM frame. BAM and RTS open a fresh session for this
// source; Abort closes one down; the other control bytes (CTS / EOM) are
// peer responses to an RTS session we, as a passive monitor, don't
// participate in and are ignored.
static void handleIsoTpCm(const RawMessage *msg)
{
  size_t   buffer;
  TpSlot  *p;
  uint8_t  control;
  uint32_t totalSize;
  uint8_t  packets;
  uint32_t targetPgn;

  if (msg->len == 0)
  {
    logError("ISO TP CM frame from source %u is empty; ignoring\n", msg->src);
    return;
  }

  control = msg->data[0];

  if (control == ISO_TP_CM_ABORT)
  {
    // Drop any in-flight session for this source.
    for (buffer = 0; buffer < ISO_TP_SLOTS; buffer++)
    {
      if (tpSlotBuffer[buffer].used && tpSlotBuffer[buffer].src == msg->src)
      {
        tpSlotBuffer[buffer].used = false;
      }
    }
    return;
  }

  if (control != ISO_TP_CM_BAM && control != ISO_TP_CM_RTS)
  {
    return;
  }

  if (msg->len < 8)
  {
    logError("ISO TP CM frame from source %u has %u bytes (need 8); ignoring\n", msg->src, msg->len);
    return;
  }

  totalSize = (uint32_t) msg->data[1] + ((uint32_t) msg->data[2] << 8);
  packets   = msg->data[3];
  targetPgn = (uint32_t) msg->data[5] + ((uint32_t) msg->data[6] << 8) + ((uint32_t) msg->data[7] << 16);

  if (packets == 0 || totalSize == 0 || totalSize > ISOTP_MAX_SIZE)
  {
    logError("ISO TP CM frame from source %u declares implausible size=%u packets=%u; ignoring\n", msg->src, totalSize, packets);
    return;
  }

  // Find an existing slot for this source, otherwise claim a free one. The
  // spec allows only one in-flight transfer per source at a time, so a new
  // CM for a source that's already got a session simply restarts it.
  for (buffer = 0; buffer < ISO_TP_SLOTS; buffer++)
  {
    if (tpSlotBuffer[buffer].used && tpSlotBuffer[buffer].src == msg->src)
    {
      break;
    }
  }
  if (buffer == ISO_TP_SLOTS)
  {
    for (buffer = 0; buffer < ISO_TP_SLOTS; buffer++)
    {
      if (!tpSlotBuffer[buffer].used)
      {
        break;
      }
    }
    if (buffer == ISO_TP_SLOTS)
    {
      logError("Out of ISO TP reassembly slots; ignoring transfer from source %u\n", msg->src);
      return;
    }
  }

  p            = &tpSlotBuffer[buffer];
  p->used      = true;
  p->src       = msg->src;
  p->dst       = msg->dst;
  p->prio      = msg->prio;
  p->targetPgn = targetPgn;
  p->totalSize = totalSize;
  p->packets   = packets;
  memset(p->received, 0, sizeof(p->received));
  strncpy(p->timestamp, msg->timestamp, sizeof(p->timestamp) - 1);
  p->timestamp[sizeof(p->timestamp) - 1] = '\0';
  memset(p->data, 0xff, totalSize);

  logDebug("ISO TP: opened session for target PGN %u from source %u, %u bytes in %u packets\n",
           targetPgn,
           msg->src,
           totalSize,
           packets);
}

// Handle a PGN 60160 TP.DT frame. Copies the 7-byte chunk into the matching
// session's buffer at offset (seq - 1) * 7; synthesizes and decodes a frame
// for the target PGN the moment every declared sequence number has been
// seen. A DT frame with no matching open session (we may have missed the
// CM) is silently ignored.
static void handleIsoTpDt(const RawMessage *msg)
{
  size_t   buffer;
  TpSlot  *p;
  uint8_t  sequence;
  size_t   seqZeroBased;
  size_t   offset;
  size_t   end;
  size_t   copyLen;
  size_t   available;
  size_t   fullWords;
  size_t   partialBits;
  bool     allReceived;

  if (msg->len == 0)
  {
    logError("ISO TP DT frame from source %u is empty; ignoring\n", msg->src);
    return;
  }

  for (buffer = 0; buffer < ISO_TP_SLOTS; buffer++)
  {
    if (tpSlotBuffer[buffer].used && tpSlotBuffer[buffer].src == msg->src)
    {
      break;
    }
  }
  if (buffer == ISO_TP_SLOTS)
  {
    logDebug("ISO TP DT frame from source %u with no open session; ignoring\n", msg->src);
    return;
  }

  p        = &tpSlotBuffer[buffer];
  sequence = msg->data[0];

  if (sequence == 0 || sequence > p->packets)
  {
    logError("ISO TP DT frame from source %u has sequence %u out of range 1..%u; ignoring\n", msg->src, sequence, p->packets);
    return;
  }

  seqZeroBased = (size_t) sequence - 1;
  p->received[seqZeroBased / 32] |= UINT32_C(1) << (seqZeroBased % 32);

  offset = seqZeroBased * FASTPACKET_BUCKET_N_SIZE;
  end    = offset + FASTPACKET_BUCKET_N_SIZE;
  if (end > p->totalSize)
  {
    end = p->totalSize;
  }
  copyLen   = (end > offset) ? end - offset : 0;
  available = (msg->len > 1) ? (size_t) msg->len - 1 : 0;
  if (copyLen > available)
  {
    copyLen = available;
  }
  if (copyLen > 0)
  {
    memcpy(&p->data[offset], &msg->data[1], copyLen);
  }

  // Complete iff every declared sequence number has arrived.
  fullWords   = p->packets / 32;
  partialBits = p->packets % 32;
  allReceived = true;
  for (size_t i = 0; i < fullWords; i++)
  {
    if (p->received[i] != UINT32_MAX)
    {
      allReceived = false;
      break;
    }
  }
  if (allReceived && partialBits > 0)
  {
    uint32_t mask = (UINT32_C(1) << partialBits) - 1;
    if ((p->received[fullWords] & mask) != mask)
    {
      allReceived = false;
    }
  }
  if (!allReceived)
  {
    return;
  }

  p->used = false;

  if (isTargetPgnAllowed(p->targetPgn))
  {
    RawMessage synthesized;

    memcpy(synthesized.timestamp, p->timestamp, sizeof(synthesized.timestamp));
    synthesized.prio = p->prio;
    synthesized.pgn  = p->targetPgn;
    synthesized.dst  = p->dst;
    synthesized.src  = p->src;
    synthesized.len  = 0; // unused here; printPgn takes data/length as separate arguments

    printPgn(&synthesized, p->data, (int) p->totalSize, showData, showJson);
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

  logDebug("showBytesOrBits(%p, %zu, %zu)\n", data, startBit, bits);

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

static uint32_t   g_refPrn = 0; // Remember this over the entire set of fields
static const Pgn *g_refPgn = 0; // Remember this over the entire set of fields

static void fillGlobalsBasedOnField(const Field *field, const uint8_t *data, size_t dataLen, size_t startBit, size_t bits)
{
  int64_t value;
  int64_t maxValue;

  if (strcmp(field->name, "PGN") == 0)
  {
    extractNumber(NULL, data, dataLen, startBit, bits, &value, &maxValue);
    logDebug("Reference PGN = %" PRId64 "\n", value);
    g_refPrn = value;
    g_refPgn = NULL;
    return;
  }

  if (field->dynamicFieldLength)
  {
    extractNumber(NULL, data, dataLen, startBit, bits, &value, &maxValue);
    g_length      = value - field->dynamicFieldLengthOverhead;
    g_lengthValid = true;
    logDebug("for next field: length = %" PRId64 " (raw %" PRId64 ", overhead %u)\n",
             g_length,
             value,
             field->dynamicFieldLengthOverhead);
    return;
  }
}

static bool printField(const Field   *field,
                       const char    *fieldName,
                       const uint8_t *data,
                       size_t         dataLen,
                       size_t         startBit,
                       size_t        *bits,
                       bool           allowKey)
{
  size_t bytes;
  double resolution;
  bool   r;

  if (fieldName == NULL)
  {
    // Defensive only: both callers pass a name. Key on the mode, not on
    // camelName presence - the generated tables set camelName everywhere.
    fieldName = (showCamel && field->camelName) ? field->camelName : (char *) field->name;
  }

  resolution = field->resolution;
  if (resolution == 0.0)
  {
    resolution = field->ft->resolution;
  }

  logDebug("PGN %u: printField(<%s>, \"%s\", ..., dataLen=%zu, data=%p, startBit=%zu) resolution=%g\n",
           field->pgn->pgn,
           field->name,
           fieldName,
           dataLen,
           data,
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

  fillGlobalsBasedOnField(field, data, dataLen, startBit, *bits);

  logDebug("PGN %u: printField <%s>, \"%s\": bits=%zu proprietary=%u refPgn=%u\n",
           field->pgn->pgn,
           field->name,
           fieldName,
           *bits,
           field->proprietary,
           g_refPrn);

  if (field->proprietary)
  {
    if (IS_PGN_PROPRIETARY(g_refPrn))
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
      else if (field->ft->pf != fieldPrintVariable)
      {
        location3     = mlocation();
        bool endQuote = mchr(location3 - 1) == '}';
        if (endQuote)
        {
          mset(location3 - 1);
          location3--;
        }
        if (showBytes)
        {
          showBytesOrBits(data + (startBit >> 3), startBit & 7, *bits);
          if (showJson)
          {
            endQuote = true;
          }
        }
        if (showJsonValue && field->partOfPrimaryKey && allowKey)
        {
          mprintf(",\"key\":true");
          endQuote = true;
        }
        if (endQuote)
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

  size_t i;
  bool   r;
  size_t variableFields = 0; // How many variable fields remain (product of repetition count * # of fields)
  char   ts[DATE_LENGTH];

  if (msg == NULL)
  {
    return false;
  }
  normalizeTimestamp(msg->timestamp, ts, sizeof(ts));
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

    fprintf(f, "%s %u %3u %3u %6u %s: ", ts, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    for (i = 0; i < length; i++)
    {
      fprintf(f, " %2.02X", data[i]);
    }
    putc('\n', f);

    fprintf(f, "%s %u %3u %3u %6u %s: ", ts, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    for (i = 0; i < length; i++)
    {
      fprintf(f, "  %c", isalnum(data[i]) ? data[i] : '.');
    }
    putc('\n', f);
  }
  if (showJson)
  {
    // The camel-id wrapper follows the -camel mode. This used to key on
    // camelDescription presence as a proxy, which wrapped pinned-id PGNs
    // even in plain JSON and broke down once the generated tables set
    // camelDescription on every PGN.
    if (showCamel)
    {
      mprintf("{\"%s\":", pgn->camelDescription);
    }
    mprintf("{\"timestamp\":\"%s\",\"prio\":%u,\"src\":%u,\"dst\":%u,\"pgn\":%u,\"description\":\"%s\"",
            ts,
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
    if (showCamel)
    {
      strcpy(closingBraces, "}}");
    }
    else
    {
      strcpy(closingBraces, "}");
    }
    sep = ",\"fields\":{";
  }
  else
  {
    mprintf("%s %u %3u %3u %6u %s:", ts, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    sep = " ";
  }
  r = printFields(pgn, data, length, showData, showJson, &variableFields);

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

extern bool printFields(const Pgn *pgn, const uint8_t *data, int length, bool showData, bool showJson, size_t *variableFields)
{
  size_t  i;
  size_t  bits;
  size_t  startBit;
  int     repetition;
  char    fieldName[60];
  uint8_t variableFieldStart;
  uint8_t variableFieldCount;

  bool r = true;

  logDebug("fieldCount=%d repeatingStart1=%" PRIu8 "\n", pgn->fieldCount, pgn->repeatingStart1);

  g_variableFieldRepeat[0] = 255; // Can be overridden by '# of parameters'
  g_variableFieldRepeat[1] = 0;   // Can be overridden by '# of parameters'
  repetition               = 0;
  *variableFields          = 0;

  // Start each PGN with clean dynamic-field state. Two ways stale state can leak into a fresh message:
  //  - g_ftf: probing candidate PGNs in getMatchingPgn() can leave it pointing at a UINT16/etc. field, which
  //    would make the first DYNAMIC_FIELD_VALUE of a PGN that has no preceding DYNAMIC_FIELD_KEY (e.g. the
  //    PGN 130823 / 130822 directory records) decode its raw bytes as that stale type.
  //  - g_length/g_lengthValid: a DYNAMIC_FIELD_LENGTH sets these for the following DYNAMIC_FIELD_VALUE, but if a
  //    message is truncated right after the length field (the value field is never reached) the flag survives and
  //    the next message's first value inherits a bogus length -> spurious "insufficient bytes".
  // Both are always re-established within a well-formed message (Key/length field before each Value), so clearing
  // them here only discards cross-message leakage.
  g_ftf         = NULL;
  g_length      = 0;
  g_lengthValid = false;

  for (i = 0, startBit = 0; (startBit >> 3) < length; i++)
  {
    const Field *field = &pgn->fieldList[i];

    if (*variableFields == 0)
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
      *variableFields    = pgn->repeatingCount1 * g_variableFieldRepeat[0];
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
      *variableFields    = pgn->repeatingCount2 * g_variableFieldRepeat[1];
      variableFieldCount = pgn->repeatingCount2;
      variableFieldStart = pgn->repeatingStart2;
      repetition         = 1;
    }

    if (*variableFields > 0)
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
               *variableFields);
      (*variableFields)--;
    }

    if (!field->camelName && !field->name)
    {
      logDebug("PGN %u has unknown bytes at end: %u\n", pgn->pgn, length - (startBit >> 3));
      break;
    }

    if (showCamel)
    {
      strcpy(fieldName, field->camelName ? field->camelName : field->name);
    }
    else
    {
      strcpy(fieldName, field->name);
    }

    if (repetition >= 1 && !showJson)
    {
      // The separator follows the naming style in use ("windSpeed_2" vs
      // "Wind Speed 2"). This used to key on camelName presence as a cheap
      // proxy for the -camel mode, which broke down once every field
      // carries an explicit camelName (id) from the generated tables.
      strcat(fieldName, showCamel ? "_" : " ");
      sprintf(fieldName + strlen(fieldName), "%u", repetition);
    }

    if (!printField(field, fieldName, data, length, startBit, &bits, true))
    {
      r = false;
      break;
    }

    startBit += bits;
  }

  return r;
}

/*
 * Variable fields only occur in PGN 126208, where they refer
 * to a field in a different PGN definition.
 *
 * The PGN that they refer to is already in g_refPrn,
 * but this may have to be refined for proprietary PGNs or PGNs with
 * other match fields.
 */
extern bool fieldPrintVariable(const Field   *field,
                               const char    *fieldName,
                               const uint8_t *data,
                               size_t         dataLen,
                               size_t         startBit,
                               size_t        *bits)
{
  bool r;

  if (g_refPrn != 0)
  {
    if (g_refPgn == NULL)
    {
      const uint8_t *variableFields = data + startBit / 8 - 2;
      size_t         variableLen    = data + dataLen - variableFields;
      g_refPgn                      = getMatchingPgnByParameters(g_refPrn, variableFields, variableLen);
    }
    if (g_refPgn != NULL)
    {
      int          field    = data[startBit / 8 - 1] - 1;
      const Field *refField = &g_refPgn->fieldList[field];

      if (refField)
      {
        logDebug("Field %s: found variable field %u '%s'\n", fieldName, g_refPrn, refField->name);
        r     = printField(refField, fieldName, data, dataLen, startBit, bits, false);
        *bits = (*bits + 7) & ~0x07; // round to bytes
        return r;
      }
    }
  }

  logError("Field %s: cannot derive variable length for PGN %d field # %d\n", fieldName, g_refPrn, data[-1]);
  *bits = 8; /* Gotta assume something */
  return false;
}
