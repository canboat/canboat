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

#include <parse.h>

bool parseIgnoreChecksum = false;

static char *findOccurrence(char *msg, char c, int count)
{
  int   i;
  char *p;

  if (*msg == 0 || *msg == '\n')
  {
    return 0;
  }
  for (i = 0, p = msg; p && i < count; i++, p++)
  {
    p = strchr(p, c);
    if (!p)
    {
      return 0;
    }
  }
  return p;
}

/*
 * Whether the NMEA 0183 sentence starting at `dollar` (its '$') has the
 * checksum after `star`: exactly two hex digits, then only the line end,
 * as the Rust parsers require.
 */
static bool nmeaChecksumMatches(const char *dollar, const char *star)
{
  unsigned char xor = 0;
  const char   *q;

  for (q = dollar + 1; q < star; q++)
  {
    xor ^= (unsigned char) *q;
  }
  if (!isxdigit((unsigned char) star[1]) || !isxdigit((unsigned char) star[2]))
  {
    return false;
  }
  for (q = star + 3; *q; q++)
  {
    if (!isspace((unsigned char) *q))
    {
      return false;
    }
  }
  return strtoul((char[3]){star[1], star[2], 0}, NULL, 16) == xor;
}

static int setParsedValues(RawMessage *m, unsigned int prio, unsigned int pgn, unsigned int dst, unsigned int src, unsigned int len)
{
  m->prio = prio;
  m->pgn  = pgn;
  m->dst  = dst;
  m->src  = src;
  m->len  = len;

  return 0;
}

int parseRawFormatPlain(char *msg, RawMessage *m, bool showJson)
{
  unsigned int prio, pgn, dst, src, len, junk, i;
  int          r;
  char        *p;
  unsigned int data[8];

  p = findOccurrence(msg, ',', 1);
  if (!p)
  {
    return 1;
  }
  p--; // Back to comma

  {
    size_t tsLen = CB_MIN((size_t)(p - msg), sizeof(m->timestamp) - 1);
    memcpy(m->timestamp, msg, tsLen);
    m->timestamp[tsLen] = 0;
  }

  /* Moronic Windows does not support %hh<type> so we use intermediate variables */
  r = sscanf(p,
             ",%u,%u,%u,%u,%u"
             ",%x,%x,%x,%x,%x,%x,%x,%x,%x",
             &prio,
             &pgn,
             &src,
             &dst,
             &len,
             &data[0],
             &data[1],
             &data[2],
             &data[3],
             &data[4],
             &data[5],
             &data[6],
             &data[7],
             &junk);
  if (r < 5)
  {
    logError("Error reading message, scanned %d from %s", r, msg);
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return 2;
  }

  if (len > 8)
  {
    // This is not PLAIN format but FAST format */
    return -1;
  }

  if (r > 5 + 8)
  {
    return -1;
  }

  if (r < 5 + len)
  {
    // Fewer hex bytes on the line than the length field announces. The
    // missing entries of data[] were never written by sscanf, so copying
    // them would decode stack garbage as PGN field values.
    logError("Error reading message, %u data bytes declared but only %u present in %s", len, r - 5, msg);
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return 2;
  }

  for (i = 0; i < len; i++)
  {
    m->data[i] = data[i];
  }

  return setParsedValues(m, prio, pgn, dst, src, len);
}

int parseRawFormatFast(char *msg, RawMessage *m, bool showJson)
{
  unsigned int prio, pgn, dst, src, len, i;
  int          r;
  char        *p;

  p = findOccurrence(msg, ',', 1);
  if (!p)
  {
    return 1;
  }
  p--; // Back to comma

  {
    size_t tsLen = CB_MIN((size_t)(p - msg), sizeof(m->timestamp) - 1);
    memcpy(m->timestamp, msg, tsLen);
    m->timestamp[tsLen] = 0;
  }

  r = sscanf(p, ",%u,%u,%u,%u,%u ", &prio, &pgn, &src, &dst, &len);
  if (r < 5)
  {
    logError("Error reading message, scanned %d from %s", r, msg);
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return 2;
  }

  if (len > MAX_PGN_SIZE)
  {
    logError("Message size %u exceeds maximum %u: %s", len, MAX_PGN_SIZE, msg);
    return 2;
  }

  p = findOccurrence(p, ',', 6);
  if (!p)
  {
    logError("Error reading message, cannot find sixth comma in %s", msg);
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return 2;
  }
  for (i = 0; i < len; i++)
  {
    if (scanHex(&p, &m->data[i]))
    {
      logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
      if (!showJson)
        fprintf(stdout, "%s", msg);
      return 2;
    }
    if (i < len)
    {
      if (*p != ',' && !isspace((unsigned char) *p))
      {
        logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
        if (!showJson)
          fprintf(stdout, "%s", msg);
        return 2;
      }
      p++;
    }
  }

  return setParsedValues(m, prio, pgn, dst, src, len);
}

int parseRawFormatAirmar(char *msg, RawMessage *m, bool showJson)
{
  unsigned int prio, pgn, dst, src, len, i;
  char        *p;
  unsigned int id;

  p = findOccurrence(msg, ' ', 1);
  if (p < msg + 4 || p >= msg + sizeof(m->timestamp))
  {
    return 1;
  }

  memcpy(m->timestamp, msg, p - msg - 1);
  m->timestamp[p - msg - 1] = 0;
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
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return 2;
  }

  getISO11783BitsFromCanId(id, &prio, &pgn, &src, &dst);

  p++;
  len = CB_MIN(strlen(p) / 2, FASTPACKET_MAX_SIZE);
  for (i = 0; i < len; i++)
  {
    if (scanHex(&p, &m->data[i]))
    {
      logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
      if (!showJson)
        fprintf(stdout, "%s", msg);
      return 2;
    }
    if (i < len)
    {
      if (*p != ',' && *p != ' ')
      {
        logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
        if (!showJson)
          fprintf(stdout, "%s", msg);
        return 2;
      }
      p++;
    }
  }

  return setParsedValues(m, prio, pgn, dst, src, len);
}

int parseRawFormatSeaSmart(char *msg, RawMessage *m, bool showJson)
{
  unsigned int pgn, src, i;
  unsigned int tstamp;
  time_t       t;
  struct tm    tm;
  char        *p = msg;
  char        *star;

  if (*p == '\\')
  {
    p = strchr(p + 1, '\\'); // a TAG block
    if (!p)
    {
      return 1;
    }
    p++;
  }
  if (strncmp(p, "$PCDIN,", STRSIZE("$PCDIN,")) != 0)
  {
    return 1; // empty, or another NMEA 0183 sentence: no message
  }

  star = strchr(p, '*');
  if (star && !parseIgnoreChecksum && !nmeaChecksumMatches(p, star))
  {
    logError("$PCDIN checksum does not match: %s", msg);
    return 2;
  }

  if (sscanf(p, "$PCDIN,%x,%x,%x,", &pgn, &tstamp, &src) < 3)
  {
    logError("Error reading SeaSmart ($PCDIN) message: %s", msg);
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return 2;
  }

  t = (time_t) tstamp / 1000;
  gmtime_r(&t, &tm); // the Z the timestamp is printed with
  strftime(m->timestamp, sizeof(m->timestamp), "%Y-%m-%dT%H:%M:%S", &tm);
  sprintf(m->timestamp + strlen(m->timestamp), ",%3.3u", tstamp % 1000);

  // The data starts after the fourth comma.
  for (i = 0; i < 4 && p; i++)
  {
    p = strchr(p, ',');
    if (p)
    {
      p++;
    }
  }
  if (!p)
  {
    logError("Error reading SeaSmart ($PCDIN) message: %s", msg);
    return 2;
  }

  for (i = 0; *p != '*' && *p != '\r' && *p != '\n' && *p != 0 && i < FASTPACKET_MAX_SIZE; i++)
  {
    if (scanHex(&p, &m->data[i]))
    {
      logError("Error reading $PCDIN data: %s", msg);
      if (!showJson)
        fprintf(stdout, "%s", msg);
      return 2;
    }
  }

  return setParsedValues(m, 0, pgn, 255, src, i);
}

/*
Sequence #,Timestamp,PGN,Name,Manufacturer,Remote Address,Local Address,Priority,Single Frame,Size,Packet
0,486942,127508,Battery Status,Garmin,6,255,2,1,8,0x017505FF7FFFFFFF
129,491183,129029,GNSS Position Data,Unknown
Manufacturer,3,255,3,0,43,0xFFDF40A6E9BB22C04B3666C18FBF0600A6C33CA5F84B01A0293B140000000010FC01AC26AC264A12000000
*/
int parseRawFormatGarminCSV(char *msg, RawMessage *m, bool showJson, bool absolute)
{
  unsigned int seq, tstamp, pgn, src, dst, prio, single, count;
  time_t       t;
  struct tm    tm;
  char        *p;
  int          consumed;
  unsigned int i;

  if (*msg == 0 || *msg == '\n')
  {
    return 1;
  }

  if (absolute)
  {
    unsigned int month, day, year, hours, minutes, seconds, ms;

    if (sscanf(msg, "%u,%u_%u_%u_%u_%u_%u_%u,%u,", &seq, &month, &day, &year, &hours, &minutes, &seconds, &ms, &pgn) < 9)
    {
      logError("Error reading Garmin CSV message: %s", msg);
      if (!showJson)
        fprintf(stdout, "%s", msg);
      return 2;
    }
    snprintf(m->timestamp,
             sizeof(m->timestamp),
             "%04u-%02u-%02uT%02u:%02u:%02u,%03u",
             year,
             month,
             day,
             hours,
             minutes,
             seconds,
             ms % 1000);

    p = findOccurrence(msg, ',', 6);
  }
  else
  {
    if (sscanf(msg, "%u,%u,%u,", &seq, &tstamp, &pgn) < 3)
    {
      logError("Error reading Garmin CSV message: %s", msg);
      if (!showJson)
        fprintf(stdout, "%s", msg);
      return 2;
    }

    t = (time_t) tstamp / 1000;
    localtime_r(&t, &tm);
    strftime(m->timestamp, sizeof(m->timestamp), "%Y-%m-%dT%H:%M:%S", &tm);
    sprintf(m->timestamp + strlen(m->timestamp), ",%3.3u", tstamp % 1000);

    p = findOccurrence(msg, ',', 5);
  }

  if (!p || sscanf(p, "%u,%u,%u,%u,%u,0x%n", &src, &dst, &prio, &single, &count, &consumed) < 5)
  {
    logError("Error reading Garmin CSV message: %s", msg);
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return 3;
  }
  p += consumed;

  if (count > FASTPACKET_MAX_SIZE)
  {
    logError("Garmin CSV message Size %u exceeds maximum %u: %s", count, FASTPACKET_MAX_SIZE, msg);
    return 2;
  }

  for (i = 0; *p && i < count; i++)
  {
    if (scanHex(&p, &m->data[i]))
    {
      logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
      if (!showJson)
        fprintf(stdout, "%s", msg);
      return 2;
    }
  }

  return setParsedValues(m, prio, pgn, dst, src, i);
}

static int hexDigit(char c)
{
  if (c >= '0' && c <= '9')
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
  return -1;
}

/*
 * ShipModul MiniPlex $MXPGN sentences: an NMEA 2000 frame in NMEA 0183
 * (ShipModul, "MiniPlex-3 NMEA and Proprietary Sentences", FW V3.40.0, p. 4).
 *
 *   $--PGN,pppppp,aaaa,d--d*hh
 *
 * pppppp is the PGN, with the destination in its low byte for a PDU1 PGN.
 * aaaa holds the send bit (15), the priority (14-12), the data length (11-8; 9..15
 * mean 8 bytes) and the source address (7-0; the destination when the send bit is
 * set). An omitted aaaa means send, priority 7, the data's length, destination in
 * the PGN. d--d is the data as one number, the last byte first. One sentence is
 * one CAN frame; they carry no time. A leading TAG block (\s:MX01*hh\) is skipped.
 *
 * Returns 0 for a frame, 1 for another NMEA 0183 sentence (a MiniPlex mixes them
 * in), 2 for a malformed $MXPGN sentence.
 */
/*
 * The NMEA 2000 network management a ShipModul MiniPlex sends as its own
 * sentences, never as $MXPGN ("MiniPlex-3 NMEA and Proprietary Sentences",
 * FW V3.40.0, pp. 25-29):
 *
 *   $PSMDNC,R,<NAME, 16 hex digits>,<source>              PGN 60928
 *   $PSMDNP,A,<source>,<version>,<product code>,<certification level>,<load>
 *   $PSMDNP,B,<model ID>,<serial code>                    PGN 126996, in
 *   $PSMDNP,C,<software version>,<model version>          three parts
 *
 * The parts of $PSMDNP arrive in order, and only A names the source. A
 * software version may hold a comma (an NGT-1's is "1.100, 2.690"): the last
 * field of B and C is the second, the rest the first. Mirrors mxpgn::Psmd in
 * the Rust crate.
 */
static struct
{
  bool         haveA;
  bool         haveB;
  unsigned int src;
  unsigned int version;
  unsigned int productCode;
  unsigned int certificationLevel;
  unsigned int loadEquivalency;
  char         model[33];
  char         serial[33];
} psmdnp;

/* A fixed 32-byte string of PGN 126996, padded with 0xff. */
static void putStringFix(uint8_t *data, const char *text)
{
  size_t len = strlen(text);

  if (len > 32)
  {
    len = 32;
  }
  memcpy(data, text, len);
  memset(data + len, 0xff, 32 - len);
}

/* Copy `src` to `dst` (size `size`), cut short to fit. */
static void copyField(char *dst, size_t size, const char *src)
{
  snprintf(dst, size, "%s", src);
}

/*
 * The $PSMDNC or $PSMDNP sentence `p` (its '$', behind any TAG block): 0 with
 * the message in `m` once one is complete, 1 for none (yet), 2 for an error.
 */
static int parsePsmd(char *msg, char *p, RawMessage *m)
{
  char         body[256];
  char        *star = strchr(p, '*');
  char        *last;
  char         now[DATE_LENGTH];
  size_t       len;
  bool         isNc = strncmp(p, "$PSMDNC,", 8) == 0;
  uint64_t     name;
  unsigned int src;
  char         flag;
  char         nameHex[17];

  if (star && !parseIgnoreChecksum && !nmeaChecksumMatches(p, star))
  {
    logError("$PSMD checksum does not match: %s", msg);
    return 2;
  }
  // The fields after "$PSMDNx,", up to the checksum or the line end.
  len = star ? (size_t) (star - p) : strcspn(p, "\r\n");
  if (len < 8 || len - 8 >= sizeof(body))
  {
    return 1;
  }
  memcpy(body, p + 8, len - 8);
  body[len - 8] = 0;

  snprintf(m->timestamp, sizeof(m->timestamp), "%s", fmtNow(now));

  if (isNc)
  {
    int i;

    if (body[0] == 'C' && body[1] == ',')
    {
      return 1; // a command, setting another device's address: no claim
    }
    if (sscanf(body, "%c,%16[0-9A-Fa-f],%u", &flag, nameHex, &src) != 3 || flag != 'R' || strlen(nameHex) != 16 || src > 255)
    {
      logError("Error reading $PSMDNC: %s", msg);
      return 2;
    }
    name = strtoull(nameHex, NULL, 16);
    for (i = 0; i < 8; i++) // the NAME goes on the bus least significant byte first
    {
      m->data[i] = (uint8_t) (name >> (8 * i));
    }
    return setParsedValues(m, 6, 60928, 255, src, 8);
  }

  switch (body[0])
  {
    case 'A':
    {
      double version;

      psmdnp.haveA = psmdnp.haveB = false;
      if (sscanf(body,
                 "A,%u,%lf,%u,%u,%u",
                 &psmdnp.src,
                 &version,
                 &psmdnp.productCode,
                 &psmdnp.certificationLevel,
                 &psmdnp.loadEquivalency)
              != 5
          || psmdnp.src > 255)
      {
        logError("Error reading $PSMDNP: %s", msg);
        return 2;
      }
      psmdnp.version = (unsigned int) (version * 1000.0 + 0.5);
      psmdnp.haveA   = true;
      return 1;
    }

    case 'B':
    case 'C':
      last = strrchr(body, ',');
      if (body[1] != ',' || !last || last == body + 1)
      {
        logError("Error reading $PSMDNP: %s", msg);
        return 2;
      }
      *last = 0;
      if (body[0] == 'B')
      {
        copyField(psmdnp.model, sizeof(psmdnp.model), body + 2);
        copyField(psmdnp.serial, sizeof(psmdnp.serial), last + 1);
        psmdnp.haveB = psmdnp.haveA;
        return 1;
      }
      if (!psmdnp.haveB)
      {
        psmdnp.haveA = false; // the parts before it were missed
        return 1;
      }
      m->data[0] = (uint8_t) psmdnp.version;
      m->data[1] = (uint8_t) (psmdnp.version >> 8);
      m->data[2] = (uint8_t) psmdnp.productCode;
      m->data[3] = (uint8_t) (psmdnp.productCode >> 8);
      putStringFix(m->data + 4, psmdnp.model);
      putStringFix(m->data + 36, body + 2);
      putStringFix(m->data + 68, last + 1);
      putStringFix(m->data + 100, psmdnp.serial);
      m->data[132]  = (uint8_t) psmdnp.certificationLevel;
      m->data[133]  = (uint8_t) psmdnp.loadEquivalency;
      psmdnp.haveA = psmdnp.haveB = false;
      return setParsedValues(m, 6, 126996, 255, psmdnp.src, 134);

    default:
      return 1;
  }
}

int parseRawFormatMxpgn(char *msg, RawMessage *m, bool showJson)
{
  char        *p = msg;
  char        *data;
  char        *star;
  char        *end;
  char         now[DATE_LENGTH];
  unsigned int pgn, attr, dlc, want, address, len, i;
  bool         send;
  bool         omitted;

  (void) showJson;

  if (*p == '\\')
  {
    p = strchr(p + 1, '\\');
    if (!p)
    {
      return 1;
    }
    p++;
  }
  if (strncmp(p, "$PSMDNC,", 8) == 0 || strncmp(p, "$PSMDNP,", 8) == 0)
  {
    return parsePsmd(msg, p, m);
  }
  if (p[0] != '$' || strlen(p) < 7 || strncmp(p + 3, "PGN,", 4) != 0)
  {
    return 1;
  }

  star = strchr(p, '*');
  if (star && !parseIgnoreChecksum && !nmeaChecksumMatches(p, star))
  {
    logError("$MXPGN checksum does not match: %s", msg);
    return 2;
  }

  pgn = (unsigned int) strtoul(p + 7, &end, 16);
  if (end == p + 7 || *end != ',' || pgn > 0x3ffff)
  {
    logError("Error reading $MXPGN PGN: %s", msg);
    return 2;
  }
  p       = end + 1;
  omitted = *p == ',';
  if (omitted)
  {
    attr = 0; // set below, once the data's length is known
    data = p + 1;
  }
  else
  {
    attr = (unsigned int) strtoul(p, &end, 16);
    if (end == p || *end != ',' || attr > 0xffff)
    {
      logError("Error reading $MXPGN attribute: %s", msg);
      return 2;
    }
    data = end + 1;
  }

  for (len = 0; hexDigit(data[len]) >= 0; len++)
    ;
  // The data ends the sentence: then comes its checksum or the line's end.
  for (end = data + len; *end != '*' && isspace((unsigned char) *end); end++)
    ;
  if (len % 2 != 0 || len > 16 || (data[len] != '*' && *end != '\0'))
  {
    logError("Error reading $MXPGN data: %s", msg);
    return 2;
  }
  len /= 2;
  if (omitted)
  {
    attr = 0x8000 | (7 << 12) | (len << 8);
  }

  send    = (attr & 0x8000) != 0;
  dlc     = (attr >> 8) & 0xf;
  address = attr & 0xff;
  want    = dlc > 8 ? 8 : dlc;
  if (len != want)
  {
    logError("$MXPGN data is %u bytes, its length says %u: %s", len, want, msg);
    return 2;
  }

  // The last byte first.
  for (i = 0; i < len; i++)
  {
    const char *pair = data + 2 * (len - 1 - i);

    m->data[i] = (unsigned char) (hexDigit(pair[0]) << 4 | hexDigit(pair[1]));
  }

  snprintf(m->timestamp, sizeof(m->timestamp), "%s", fmtNow(now));

  if (((pgn >> 8) & 0xff) < 240)
  {
    unsigned int dst = (send && address != 0) ? address : (pgn & 0xff);

    return setParsedValues(m, (attr >> 12) & 7, pgn & ~0xffu, dst, send ? 0 : address, len);
  }
  return setParsedValues(m, (attr >> 12) & 7, pgn, 255, send ? 0 : address, len);
}

/* Yacht Digital, YDWG-02

   Example output: 00:17:55.475 R 0DF50B23 FF FF FF FF FF 00 00 FF

   Example usage:

pi@yacht:~/canboat/analyzer $ netcat 192.168.3.2 1457 | analyzer -json
INFO 2018-10-16T09:57:39.665Z [analyzer] Detected YDWG-02 protocol with all data on one line
INFO 2018-10-16T09:57:39.665Z [analyzer] New PGN 128267 for device 35 (heap 5055 bytes)
{"timestamp":"2018-10-16T22:25:25.166","prio":3,"src":35,"dst":255,"pgn":128267,"description":"Water
Depth","fields":{"Offset":0.000}} INFO 2018-10-16T09:57:39.665Z [analyzer] New PGN 128259 for device 35 (heap 5070 bytes)
{"timestamp":"2018-10-16T22:25:25.177","prio":2,"src":35,"dst":255,"pgn":128259,"description":"Speed","fields":{"Speed Water
Referenced":0.00,"Speed Water Referenced Type":"Paddle wheel"}} INFO 2018-10-16T09:57:39.666Z [analyzer] New PGN 128275 for device
35 (heap 5091 bytes)
{"timestamp":"2018-10-16T22:25:25.179","prio":6,"src":35,"dst":255,"pgn":128275,"description":"Distance
Log","fields":{"Date":"1980.05.04"}} INFO 2018-10-16T09:57:39.666Z [analyzer] New PGN 130311 for device 35 (heap 5106 bytes)
{"timestamp":"2018-10-16T22:25:25.181","prio":5,"src":35,"dst":255,"pgn":130311,"description":"Environmental
Parameters","fields":{"Temperature Source":"Sea Temperature","Temperature":13.39}}
{"timestamp":"2018-10-16T22:25:25.181","prio":6,"src":35,"dst":255,"pgn":128275,"description":"Distance
Log","fields":{"Date":"2006.11.06", "Time": "114:38:39.07076","Log":1940}}
{"timestamp":"2018-10-16T22:25:25.185","prio":6,"src":35,"dst":255,"pgn":128275,"description":"Distance
Log","fields":{"Date":"1970.07.14"}} INFO 2018-10-16T09:57:39.666Z [analyzer] New PGN 130316 for device 35 (heap 5121 bytes)
{"timestamp":"2018-10-16T22:25:25.482","prio":5,"src":35,"dst":255,"pgn":130316,"description":"Temperature Extended
Range","fields":{"Instance":0,"Source":"Sea Temperature","Temperature":13.40}}
{"timestamp":"2018-10-16T22:25:25.683","prio":5,"src":35,"dst":255,"pgn":130311,"description":"Environmental
Parameters","fields":{"Temperature Source":"Sea Temperature","Temperature":13.39}}
*/
int parseRawFormatYDWG02(char *msg, RawMessage *m, bool showJson)
{
  char        *token;
  char        *nexttoken;
  time_t       tiden;
  struct tm    tm;
  unsigned int msgid;
  unsigned int prio, pgn, src, dst;
  int          i;

  // parse timestamp. YDWG doesn't give us date so let's figure it out ourself
  token = strtok_r(msg, " ", &nexttoken);
  if (!token)
  {
    return -1;
  }
  tiden = (time_t) (getNow() / UINT64_C(1000));
  {
    // The day is the host's UTC one, moved to yesterday or tomorrow when that
    // puts the gateway's time of day closer to the host clock: near midnight
    // the two clocks are often on different days (canboat#1039).
    unsigned int hh, mm, ss;

    if (sscanf(token, "%u:%u:%u", &hh, &mm, &ss) == 3 && hh < 24 && mm < 60 && ss <= 60)
    {
      int64_t now = (int64_t) tiden;
      int64_t day = now - now % 86400;
      int64_t tod = (int64_t) (hh * 3600 + mm * 60 + ss);
      int64_t off = day + tod - now;

      if (off > 43200)
      {
        tiden -= 86400;
      }
      else if (off < -43200)
      {
        tiden += 86400;
      }
    }
  }
  gmtime_r(&tiden, &tm);
  strftime(m->timestamp, sizeof(m->timestamp), "%Y-%m-%dT", &tm);
  snprintf(m->timestamp + strlen(m->timestamp), sizeof(m->timestamp) - strlen(m->timestamp), "%s", token);

  // parse direction, not really used in analyzer
  token = strtok_r(NULL, " ", &nexttoken);
  if (!token)
  {
    return -1;
  }

  // parse msgid
  token = strtok_r(NULL, " ", &nexttoken);
  if (!token)
  {
    return -1;
  }
  msgid = strtoul(token, NULL, 16);
  getISO11783BitsFromCanId(msgid, &prio, &pgn, &src, &dst);

  // parse data
  i = 0;
  while ((token = strtok_r(NULL, " ", &nexttoken)) != 0)
  {
    if (i >= 8)
    {
      // A YDWG-02 RAW line carries one CAN frame. More than 8 bytes means
      // two lines were glued together (a lost datagram or a reconnect);
      // decoding it as a coalesced message would give a garbage message.
      logError("Error reading message, more than 8 data bytes in YDWG-02 line for PGN %u\n", pgn);
      return 2;
    }
    m->data[i] = strtoul(token, NULL, 16);
    i++;
  }

  return setParsedValues(m, prio, pgn, dst, src, i);
}

bool parseFastFormat(StringBuffer *in, RawMessage *msg)
{
  unsigned int prio;
  unsigned int pgn;
  unsigned int src;
  unsigned int dst;
  unsigned int bytes;

  char        *p;
  int          i;
  unsigned int b;
  unsigned int byt;
  int          r;

  p = strchr(sbGet(in), '\n');
  if (!p)
  {
    return false;
  }

  // Skip the timestamp
  p = strchr(sbGet(in), ',');
  if (!p)
  {
    return false;
  }

  r = sscanf(p, ",%u,%u,%u,%u,%u,%n", &prio, &pgn, &src, &dst, &bytes, &i);
  if (r == 5)
  {
    if (bytes > MAX_PGN_SIZE)
    {
      logError("Message size %u exceeds maximum %u: %s\n", bytes, MAX_PGN_SIZE, sbGet(in));
      return false;
    }

    // now store the timestamp, unchanged
    memset(msg->timestamp, 0, sizeof msg->timestamp);
    memcpy(msg->timestamp, sbGet(in), CB_MIN(p - sbGet(in), sizeof msg->timestamp - 1));

    msg->prio = prio;
    msg->pgn  = pgn;
    msg->src  = src;
    msg->dst  = dst;
    msg->len  = bytes;

    p += i - 1;

    for (b = 0; b < bytes; b++)
    {
      if ((sscanf(p, ",%x%n", &byt, &i) == 1) && (byt < 256))
      {
        msg->data[b] = byt;
      }
      else
      {
        logError("Unable to parse incoming message '%s' data byte %u\n", sbGet(in), b);
        return false;
      }
      p += i;
    }
    return true;
  }
  logError("Unable to parse incoming message '%s', r = %d\n", sbGet(in), r);
  return false;
}

int parseRawFormatActisenseN2KAscii(char *msg, RawMessage *m, bool showJson)
{
  char         *nexttoken;
  char         *p;
  char         *token;
  int           i;
  int           r;
  unsigned int  millis = 0, hh = 0, mm = 0, ss = 0;
  unsigned long n;

  // parse timestamp. Actisense doesn't give us date so let's figure it out ourself
  token = strtok_r(msg, " ", &nexttoken);
  if (!token || token[0] != 'A')
  {
    logError("No message or does not start with 'A'\n");
    return -1;
  }
  token++;

  r = sscanf(token, "%2u%2u%2u.%u", &hh, &mm, &ss, &millis);
  if (r < 3)
  {
    return -1;
  }

  snprintf(m->timestamp, sizeof(m->timestamp), "%02u:%02u:%02u,%3.3u", hh, mm, ss, millis);

  // parse <SRC><DST><P>
  token = strtok_r(NULL, " ", &nexttoken);
  if (!token)
  {
    return -1;
  }
  n       = strtoul(token, NULL, 16);
  m->prio = n & 0xf;
  m->dst  = (n >> 4) & 0xff;
  m->src  = (n >> 12) & 0xff;

  // parse <PGN>
  token = strtok_r(NULL, " ", &nexttoken);
  if (!token)
  {
    logError("Incomplete message\n");
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return -1;
  }
  m->pgn = strtoul(token, NULL, 16);

  // parse DATA
  p = nexttoken;
  for (i = 0; i < MAX_PGN_SIZE; i++)
  {
    if (*p == '\0' || isspace((unsigned char) *p))
    {
      break;
    }
    if (scanHex(&p, &m->data[i]))
    {
      logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
      if (!showJson)
        fprintf(stdout, "%s", msg);
      return 2;
    }
  }
  m->len = i;

  return 0;
}

bool parseTimestamp(const char *msg, uint64_t *when)
{
  struct tm t;
  time_t    epoch;
  int       year  = 0;
  int       month = 0;
  int       day   = 0;
  int       hour  = 0;
  int       min   = 0;
  int       sec   = 0;
  int       milli = 0;
  char     *p;

  p = strstr(msg, "Z,");
  if (p)
  {
    if (sscanf(msg, "%d-%d-%dT%d:%d:%d.%dZ", &year, &month, &day, &hour, &min, &sec, &milli) < 6)
    {
      logDebug("Unable to parse timestamp '%s'\n", msg);
      return false;
    }
    t.tm_year  = year - 1900;
    t.tm_mon   = month - 1;
    t.tm_mday  = day;
    t.tm_hour  = hour;
    t.tm_min   = min;
    t.tm_sec   = sec;
    t.tm_isdst = -1;
    epoch      = timegm(&t);
  }
  else
  {
    if (sscanf(msg, "%d-%d-%d %d:%d:%d.%d", &year, &month, &day, &hour, &min, &sec, &milli) < 6)
    {
      logDebug("Unable to parse timestamp '%s'\n", msg);
      return false;
    }
    t.tm_year  = year - 1900;
    t.tm_mon   = month - 1;
    t.tm_mday  = day;
    t.tm_hour  = hour;
    t.tm_min   = min;
    t.tm_sec   = sec;
    t.tm_isdst = -1;
    epoch      = mktime(&t);
  }

  logDebug("parseTimestamp '%s' => %d-%d-%d %d:%d:%d.%03d\n", msg, year, month, day, hour, min, sec, milli);

  *when = epoch * UINT64_C(1000) + milli;

  return true;
}

/*
 * canboat's input formats carry timestamps in inconsistent shapes: ISO-8601 with a `T` and
 * trailing `Z`, the classic canboat log form `YYYY-MM-DD-HH:MM:SS.mmm` (a dash between date and
 * time), a comma as the millisecond separator (Actisense / SeaSmart), or bare time-of-day with no
 * date at all (Actisense ASCII, Airmar). normalizeTimestamp() rewrites these into a single
 * canonical form so downstream consumers -- and comma-delimited PLAIN lines in particular -- never
 * trip over a stray comma inside the timestamp field.
 *
 *   Date-bearing -> YYYY-MM-DDTHH:MM:SS.mmmZ   (UTC: T separator, dot fraction, 3-digit millis, Z)
 *   Time-only    -> HH:MM:SS.mmm               (no date is available, so none is invented)
 *   Unrecognised -> returned verbatim          (an odd timestamp is never lost or corrupted)
 *
 * This mirrors canboat-rs's normalize_timestamp so the two implementations stay byte-for-byte
 * comparable.
 */

static bool allDigits(const char *p, size_t n)
{
  for (size_t i = 0; i < n; i++)
  {
    if (!isdigit((unsigned char) p[i]))
    {
      return false;
    }
  }
  return true;
}

/*
 * Parse `HH:MM:SS` with an optional `.`/`,` fraction from `s` and write `HH:MM:SS.mmm` into `out`.
 * Returns true on success; false (leaving `out` untouched) when the shape or field ranges don't
 * hold, so the caller can fall back to emitting the value verbatim.
 */
static bool normalizeTime(const char *s, char *out)
{
  size_t len = strlen(s);

  if (len < 8 || s[2] != ':' || s[5] != ':')
  {
    return false;
  }
  if (!allDigits(s, 2) || !allDigits(s + 3, 2) || !allDigits(s + 6, 2))
  {
    return false;
  }

  {
    unsigned int h   = (s[0] - '0') * 10 + (s[1] - '0');
    unsigned int m   = (s[3] - '0') * 10 + (s[4] - '0');
    unsigned int sec = (s[6] - '0') * 10 + (s[7] - '0');

    if (h >= 24 || m >= 60 || sec >= 60)
    {
      return false;
    }
  }

  /* Optional fractional seconds: a `.`/`,` at index 8 then digits. Anything after the digit run (a
   * trailing `Z`, a timezone) is dropped -- canboat timestamps are UTC-naive. Pad/truncate the
   * fraction to exactly three digits (milliseconds): `.1` -> `100`, `.107` -> `107`,
   * `.10734` -> `107`. */
  char ms[4] = "000";
  if (len > 8)
  {
    if (s[8] != '.' && s[8] != ',')
    {
      return false;
    }
    for (size_t i = 9, j = 0; i < len && j < 3 && isdigit((unsigned char) s[i]); i++, j++)
    {
      ms[j] = s[i];
    }
  }

  memcpy(out, s, 8);
  out[8] = '.';
  memcpy(out + 9, ms, 3);
  out[12] = 0;
  return true;
}

/* True when `ts` is already exactly `YYYY-MM-DDTHH:MM:SS.mmmZ`. */
static bool isCanonicalTimestamp(const char *ts)
{
  return strlen(ts) == 24 && ts[4] == '-' && ts[7] == '-' && ts[10] == 'T' && ts[13] == ':' && ts[16] == ':'
         && ts[19] == '.' && ts[23] == 'Z' && allDigits(ts, 4) && allDigits(ts + 5, 2) && allDigits(ts + 8, 2)
         && allDigits(ts + 11, 2) && allDigits(ts + 14, 2) && allDigits(ts + 17, 2) && allDigits(ts + 20, 3);
}

void normalizeTimestamp(const char *in, char *out, size_t outLen)
{
  if (outLen < DATE_LENGTH)
  {
    /* Callers pass a DATE_LENGTH buffer; anything smaller can't hold the canonical form. */
    if (outLen > 0)
    {
      out[0] = 0;
    }
    return;
  }

  /* Fast path: the common live-gateway form is already canonical, so avoid re-parsing it. */
  if (isCanonicalTimestamp(in))
  {
    memcpy(out, in, strlen(in) + 1);
    return;
  }

  /* Skip leading whitespace. */
  const char *t = in;
  while (isspace((unsigned char) *t))
  {
    t++;
  }

  /* Date-bearing: `YYYY-MM-DD` then one of [`T`, `-`, space] then time. */
  {
    size_t tlen = strlen(t);
    if (tlen >= 19 && t[4] == '-' && t[7] == '-' && (t[10] == 'T' || t[10] == '-' || t[10] == ' ') && allDigits(t, 4)
        && allDigits(t + 5, 2) && allDigits(t + 8, 2))
    {
      char time[13];
      if (normalizeTime(t + 11, time))
      {
        memcpy(out, t, 10);
        out[10] = 'T';
        memcpy(out + 11, time, strlen(time));
        out[11 + strlen(time)] = 'Z';
        out[12 + strlen(time)] = 0;
        return;
      }
    }
  }

  /* Time-only. */
  {
    char time[13];
    if (normalizeTime(t, time))
    {
      memcpy(out, time, strlen(time) + 1);
      return;
    }
  }

  /* Unrecognised: emit verbatim. */
  {
    size_t n = strlen(in);
    if (n >= outLen)
    {
      n = outLen - 1;
    }
    memcpy(out, in, n);
    out[n] = 0;
  }
}
