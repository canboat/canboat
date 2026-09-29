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
  unsigned int prio, pgn, dst, src, len, junk, r, i;
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
    logError("Error reading message, scanned %zu from %s", r, msg);
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return 2;
  }

  if (len > 8)
  {
    // This is not PLAIN format but FAST format */
    return -1;
  }

  if (r <= 5 + 8)
  {
    for (i = 0; i < len; i++)
    {
      m->data[i] = data[i];
    }
  }
  else
  {
    return -1;
  }

  return setParsedValues(m, prio, pgn, dst, src, len);
}

/*
 * Parse can-utils "candump" text output.
 *
 * Two shapes are accepted, both straight from can-utils:
 *
 *   live (no -t):    "  can0  18EEFF05   [8]  04 A9 FF FC"
 *   logged (-l):    "(1502979132.106111) slcan0 09F50374#000A00FFFF00FFFF"
 *
 * The two are told apart by the CAN ID, and this is the whole point: candump
 * prints a standard (11-bit) identifier as %03x and an extended (29-bit) one
 * as %08X. So the frame's very shape is recoverable from the text, with no
 * heuristics and nothing invented.
 *
 *   11-bit  -> Quick. The ID is only a message type; there is no J1939
 *              header at all, so prio/src/dst are left zero and the
 *              16-bit talker ID is decoded from the payload as a field.
 *   29-bit  -> NMEA 2000 / J1939. prio/pgn/src/dst come out of the ID.
 *
 * Each build only accepts the shape it has tables for: analyzer-quick has no
 * NMEA 2000 PGN list, and analyzer has no Quick one. A frame of the wrong
 * shape is reported as unparsable rather than decoded into nonsense.
 */
static bool isHexDigit(char c)
{
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

int parseRawFormatCandump(char *msg, RawMessage *m, bool showJson)
{
  char        *p    = msg;
  char        *idtok;
  char        *hash;
  char        *end;
  char         idbuf[16];
  const char  *dp   = NULL;
  double       secs = 0.0;
  bool         hasTs = false;
  unsigned int canId = 0;
  unsigned int prio = 0;
  unsigned int pgn  = 0;
  unsigned int src  = 0;
  unsigned int dst  = 0;
  unsigned int len  = 0;
  unsigned int b;

  while (*p == ' ' || *p == '\t')
  {
    p++;
  }

  /* Optional absolute timestamp, as written by "candump -l". */
  if (*p == '(')
  {
    secs = strtod(p + 1, &end);
    if (end == p + 1 || *end != ')')
    {
      return 1;
    }
    hasTs = true;
    p     = end + 1;
    while (*p == ' ' || *p == '\t')
    {
      p++;
    }
  }

  /* Interface name. */
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n')
  {
    p++;
  }
  if (*p != ' ' && *p != '\t')
  {
    return 1;
  }
  while (*p == ' ' || *p == '\t')
  {
    p++;
  }

  /* CAN ID, which in the logged form runs straight into the '#' and data. */
  idtok = p;
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n')
  {
    p++;
  }
  if (p == idtok)
  {
    return 1;
  }

  /* Copy out just the identifier (up to any '#') rather than terminating the
   * line in place, so a later error report still shows the whole line.
   */
  {
    size_t      idlen = 0;
    const char *q      = idtok;

    while (q < p && *q != '#' && idlen + 1 < sizeof(idbuf))
    {
      idbuf[idlen++] = *q++;
    }
    idbuf[idlen] = 0;
  }

  canId = (unsigned int) strtoul(idbuf, &end, 16);
  if (end == idbuf)
  {
    return 1; // no hex digits: not a candump line
  }

  hash = memchr(idtok, '#', (size_t) (p - idtok));
  if (hash != NULL)
  {
    // "#" = data follows contiguously, "##" = error frame, "#R" = RTR frame.
    // Neither error nor remote frames carry payload we can decode.
    if (hash[1] == '#' || hash[1] == 'R' || hash[1] == 'r' || hash[1] == '\0')
    {
      return 1;
    }
    dp = hash + 1;
    {
      /* Measure the payload in validated hex digits rather than in characters.
       * strlen() would count a CRLF terminator as payload, inflating the length
       * by a byte and pushing the loop below off the end of the data. Only
       * whitespace may follow the payload, and there must be whole bytes.
       */
      size_t hexChars = 0;
      size_t tail;

      while (isHexDigit(dp[hexChars]))
      {
        hexChars++;
      }
      if (hexChars == 0 || (hexChars % 2) != 0)
      {
        return 1; // no payload at all, or a dangling half byte
      }
      for (tail = hexChars; dp[tail] != 0; tail++)
      {
        if (dp[tail] != '\r' && dp[tail] != '\n' && dp[tail] != ' ' && dp[tail] != '\t')
        {
          return 1; // trailing junk: this is not a candump payload
        }
      }
      len = (unsigned int) (hexChars / 2);
    }
  }
  else
  {
    // Live form: "[8]" gives the DLC, then whitespace separated data bytes.
    while (*p == ' ' || *p == '\t')
    {
      p++;
    }
    if (*p != '[')
    {
      return 1;
    }
    len = (unsigned int) strtoul(p + 1, &end, 10);
    if (end == p + 1 || *end != ']')
    {
      return 1;
    }
    p  = end + 1;
    dp = p;
  }

  if (len > MAX_PGN_SIZE)
  {
    if (showJson)
    {
      logError("Message size %u exceeds maximum %u: %s\n", len, (unsigned int) MAX_PGN_SIZE, msg);
    }
    return 1;
  }

#ifdef QUICK
  // analyzer-quick: 11-bit IDs only. The CAN ID is the Quick message type and
  // becomes the PGN; there is no priority, destination or 8-bit source to
  // derive, so none is invented.
  //
  // Classify on the printed width, not on the value. can-utils writes a
  // standard identifier as %03X and an extended one as %08X, so an 8-digit
  // token is a 29-bit frame even when its numeric value happens to fall in the
  // Quick space: 000006C1 is extended, not Quick PGN 1729. Value alone cannot
  // tell the two apart, and accepting it here would decode an extended frame
  // that analyzer rejects, into a Quick packet with invented field values.
  {
    size_t idChars = 0;

    while (idbuf[idChars] != 0)
    {
      idChars++;
    }
    if (idChars > 3)
    {
      return 1;
    }
  }
  if (canId > 0x7ff)
  {
    return 1;
  }
  pgn  = canId;
  prio = 0;
  src  = 0;
  dst  = 0;
#else
  // analyzer: 29-bit identifiers only. An 11-bit standard frame is Quick, and
  // running it through the J1939 bit layout would yield a bogus PGN.
  if (canId <= 0x7ff)
  {
    return 1;
  }
  getISO11783BitsFromCanId(canId, &prio, &pgn, &src, &dst);
#endif

  for (b = 0; b < len; b++)
  {
    unsigned int byt;
    int          used = 0;

    // Reads at most two hex digits, which copes with both the contiguous
    // logged form ("0A00FF") and the space separated live form ("0A 00 FF").
    if (sscanf(dp, " %2x%n", &byt, &used) != 1 || used == 0 || byt > 0xff)
    {
      if (showJson)
      {
        logError("Unable to parse candump data byte %u in '%s'\n", b, msg);
      }
      return 1;
    }
    m->data[b] = (uint8_t) byt;
    dp += used;
  }

  if (hasTs)
  {
    fmtTimestamp(m->timestamp, (uint64_t) (secs * 1000.0 + 0.5));
  }
  else
  {
    // candump without -t prints no timestamp; stamp it as now.
    fmtTimestamp(m->timestamp, UINT64_C(0));
  }

  return setParsedValues(m, prio, pgn, dst, src, len);
}

int parseRawFormatFast(char *msg, RawMessage *m, bool showJson)
{
  unsigned int prio, pgn, dst, src, len, r, i;
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
    logError("Error reading message, scanned %zu from %s", r, msg);
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

int parseRawFormatChetco(char *msg, RawMessage *m, bool showJson)
{
  unsigned int pgn, src, i;
  unsigned int tstamp;
  time_t       t;
  struct tm    tm;
  char        *p;

  if (*msg == 0 || *msg == '\n')
  {
    return 1;
  }

  if (sscanf(msg, "$PCDIN,%x,%x,%x,", &pgn, &tstamp, &src) < 3)
  {
    logError("Error reading Chetco message: %s", msg);
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return 2;
  }

  t = (time_t) tstamp / 1000;
  localtime_r(&t, &tm);
  strftime(m->timestamp, sizeof(m->timestamp), "%Y-%m-%dT%H:%M:%S", &tm);
  sprintf(m->timestamp + strlen(m->timestamp), ",%3.3u", tstamp % 1000);

  p = msg + STRSIZE("$PCDIN,01FD07,089C77D!,03,"); // Fixed length where data bytes start;

  for (i = 0; *p != '*' && i < FASTPACKET_MAX_SIZE; i++)
  {
    if (scanHex(&p, &m->data[i]))
    {
      logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
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
  localtime_r(&tiden, &tm);
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
    if (i >= FASTPACKET_MAX_SIZE)
    {
      return -1;
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
 * time), a comma as the millisecond separator (Actisense / Chetco), or bare time-of-day with no
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
