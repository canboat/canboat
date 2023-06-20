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

  memcpy(m->timestamp, msg, p - msg);
  m->timestamp[p - msg] = 0;

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
    logError("Error reading message, scanned %u from %s", r, msg);
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

  memcpy(m->timestamp, msg, p - msg);
  m->timestamp[p - msg] = 0;

  /* Moronic Windows does not support %hh<type> so we use intermediate variables */
  r = sscanf(p, ",%u,%u,%u,%u,%u ", &prio, &pgn, &src, &dst, &len);
  if (r < 5)
  {
    logError("Error reading message, scanned %u from %s", r, msg);
    if (!showJson)
      fprintf(stdout, "%s", msg);
    return 2;
  }

  p = findOccurrence(p, ',', 6);
  if (!p)
  {
    logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
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
  len = strlen(p) / 2;
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

  for (i = 0; *p != '*'; i++)
  {
    if (scanHex(&p, &m->data[i]))
    {
      logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
      if (!showJson)
        fprintf(stdout, "%s", msg);
      return 2;
    }
  }

  return setParsedValues(m, 0, pgn, 255, src, i + 1);
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

  return setParsedValues(m, prio, pgn, dst, src, i + 1);
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
  sprintf(m->timestamp + strlen(m->timestamp), "%s", token);

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
    m->data[i] = strtoul(token, NULL, 16);
    i++;
    if (i > FASTPACKET_MAX_SIZE)
    {
      return -1;
    }
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
  int          b;
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
    // now store the timestamp, unchanged
    memset(msg->timestamp, 0, sizeof msg->timestamp);
    memcpy(msg->timestamp, sbGet(in), CB_MAX(p - sbGet(in), sizeof msg->timestamp - 1));

    msg->prio = prio;
    msg->pgn  = pgn;
    msg->src  = src;
    msg->dst  = dst;
    msg->len  = bytes;

    p += i - 1;

    for (b = 0; b < CB_MIN(bytes, FASTPACKET_MAX_SIZE); b++)
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
  static time_t tiden = 0;
  struct tm     tm;
  time_t        now;
  unsigned int  millis = 0;
  unsigned int  secs;
  unsigned long n;

  // parse timestamp. Actisense doesn't give us date so let's figure it out ourself
  token = strtok_r(msg, " ", &nexttoken);
  if (!token || token[0] != 'A')
  {
    logError("No message or does not start with 'A'\n");
    return -1;
  }
  token++;

  r = sscanf(token, "%u.%u", &secs, &millis);
  if (r < 1)
  {
    return -1;
  }

  if (tiden == 0)
  {
    tiden = (time_t) (getNow() / UINT64_C(1000)) - secs;
  }
  now = tiden + secs;

  localtime_r(&now, &tm);
  strftime(m->timestamp, sizeof(m->timestamp), "%Y-%m-%dT%H:%M:%S", &tm);
  sprintf(m->timestamp + strlen(m->timestamp), ",%3.3u", millis);

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
  for (i = 0; i < FASTPACKET_MAX_SIZE; i++)
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
