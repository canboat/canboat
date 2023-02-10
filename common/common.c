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

#include "common.h"

StringBuffer sbNew;

static const char *logLevels[] = {"FATAL", "ERROR", "INFO", "DEBUG"};

static LogLevel logLevel = LOGLEVEL_INFO;

static char *progName;
static char  fixedTimestamp[DATE_LENGTH];

#ifndef WIN32

uint64_t getNow(void)
{
  struct timeval tv;

  if (*fixedTimestamp != '\0')
  {
    return UINT64_C(1672527600000); // 2023-01-01 00:00
  }

  if (gettimeofday(&tv, (void *) 0) == 0)
  {
    uint64_t t    = tv.tv_sec;
    uint64_t msec = tv.tv_usec / 1000L;

    return t * 1000L + msec;
  }
  return 0L;
}

void storeTimestamp(char str[DATE_LENGTH], uint64_t when)
{
  time_t    t;
  struct tm tm;
  int       msec;
  size_t    len;

  t    = when / 1000L;
  msec = when % 1000L;
  gmtime_r(&t, &tm);
  strftime(str, DATE_LENGTH - 5, "%Y-%m-%dT%H:%M:%S", &tm);
  len = strlen(str);
  snprintf(str + len, DATE_LENGTH - len, ".%3.3dZ", msec);
}

const char *now(char str[DATE_LENGTH])
{
  uint64_t now = getNow();

  if (fixedTimestamp[0] != '\0')
  {
    return (const char *) fixedTimestamp;
  }

  storeTimestamp(str, now);
  return (const char *) str;
}

#else

const char *now(char str[DATE_LENGTH])
{
  struct _timeb timebuffer;
  struct tm     tm;
  size_t        len;

  if (fixedTimestamp[0] != '\0')
  {
    return (const char *) fixedTimestamp;
  }

  _ftime_s(&timebuffer);
  gmtime_s(&tm, &timebuffer.time);
  strftime(str, DATE_LENGTH - 5, "%Y-%m-%dT%H:%M:%S", &tm);
  len = strlen(str);
  snprintf(str + len, DATE_LENGTH - len, ".%3.3dZ", timebuffer.millitm);

  return (const char *) str;
}

#endif

static int logBase(LogLevel level, const char *format, va_list ap)
{
  char strTmp[DATE_LENGTH];

  if (level > logLevel)
  {
    return 0;
  }

  fprintf(stderr, "%s %s [%s] ", logLevels[level], now(strTmp), progName);

  return vfprintf(stderr, format, ap);
}

int logInfo(const char *format, ...)
{
  int     ret;
  va_list ap;
  va_start(ap, format);

  ret = logBase(LOGLEVEL_INFO, format, ap);
  va_end(ap);

  return ret;
}

int logDebug(const char *format, ...)
{
  int     ret;
  va_list ap;
  va_start(ap, format);

  ret = logBase(LOGLEVEL_DEBUG, format, ap);
  va_end(ap);

  return ret;
}

int logError(const char *format, ...)
{
  int     ret;
  va_list ap;
  va_start(ap, format);

  ret = logBase(LOGLEVEL_ERROR, format, ap);
  va_end(ap);

  return ret;
}

void logAbort(const char *format, ...)
{
  va_list ap;
  va_start(ap, format);

  logBase(LOGLEVEL_FATAL, format, ap);
  va_end(ap);
  fflush(stderr);
  fflush(stdout);
  exit(2);
}

void die(const char *t)
{
  int   e = errno;
  char *s = 0;

  if (e)
  {
    s = strerror(e);
  }

  if (s)
  {
    logAbort("%s: %s\n", t, s);
  }
  else
  {
    logAbort("%s\n", t);
  }
}

void setLogLevel(LogLevel level)
{
  logLevel = CB_MIN(CB_MAX(level, LOGLEVEL_FATAL), LOGLEVEL_DEBUG);
  logDebug("Loglevel now %d\n", logLevel);
}

bool isLogLevelEnabled(LogLevel level)
{
  return logLevel >= level;
}

void setProgName(char *name)
{
  progName = strrchr(name, '/');
  if (!progName)
  {
    progName = strrchr(name, '\\');
  }
  if (!progName)
  {
    progName = name;
  }
  else
  {
    progName++;
  }
}

void setFixedTimestamp(char *fixedStr)
{
  strncpy(fixedTimestamp, fixedStr, sizeof(fixedTimestamp) - 1);
  logInfo("Timestamp fixed\n");
}

static void sbReserve(StringBuffer *const sb, size_t len)
{
  size_t nextSize;

  if (len < sb->len)
  {
    len = sb->len;
  }

  /* Double the allocation until it is large enough */
  /* Note that we reserve len + 1 bytes (== len + zero terminator) */
  for (nextSize = 32; nextSize < len + 1; nextSize = nextSize * 2)
    ;

  if (sb->data)
  {
    sb->data = realloc(sb->data, nextSize);
  }
  else
  {
    sb->data = malloc(nextSize);
  }

  if (!sb->data)
  {
    logAbort("Out of memory in allocating string buffer\n");
  }
  sbTerminate(sb);
  sb->alloc = nextSize;
}

void sbEnsureCapacity(StringBuffer *const sb, size_t len)
{
  len++; // Make room for termination zero byte
  if (!sb->data || (len >= sb->alloc))
  {
    sbReserve(sb, len);
  }
}

void sbDelete(StringBuffer *sb, size_t start, size_t end)
{
  if (end >= sb->len)
  {
    sbTruncate(sb, start);
  }
  else if (start < sb->len && sb->data)
  {
    end = CB_MIN(sb->len, end);
    if (end < sb->len)
    {
      memmove(sb->data + start, sb->data + end, sb->len - end);
    }
    sb->len -= end - start;
    sbTerminate(sb);
  }
}

void sbAppendData(StringBuffer *sb, const void *data, size_t len)
{
  sbEnsureCapacity(sb, sb->len + len);
  memcpy(sb->data + sb->len, data, len);
  sb->len += len;
  sb->data[sb->len] = 0;
}

char hexDigit(uint8_t b)
{
  return (b > 9) ? (char) b + 'a' - 10 : (char) b + '0';
}

void sbAppendEncodeHex(StringBuffer *sb, const void *data, size_t len, char separator)
{
  sbEnsureCapacity(sb, sb->len + len * 3);

  for (; len > 0; len--, data++)
  {
    sbAppendFormat(sb, "%c%c", hexDigit(((uint8_t *) data)[0] >> 4), hexDigit(((uint8_t *) data)[0] & 0x0f));
    if (len > 1 && separator != '\0')
    {
      sbAppendData(sb, &separator, 1);
    }
  }
}

void sbAppendDecodeHex(StringBuffer *sb, const char *data, size_t len)
{
  uint8_t  nibble1;
  uint8_t  nibble2;
  uint8_t *d;

  sbEnsureCapacity(sb, len / 2 + 1 + sbGetLength(sb));
  d = (uint8_t *) sbGet(sb) + sbGetLength(sb);

  while (len >= 2)
  {
    nibble1 = (*data >= 'a') ? (*data - 'a' + 10) : (*data >= 'A' ? (*data - 'A' + 10) : (*data - '0'));
    data++;
    nibble2 = (*data >= 'a') ? (*data - 'a' + 10) : (*data >= 'A' ? (*data - 'A' + 10) : (*data - '0'));
    data++;
    *d++ = nibble1 << 4 | nibble2;
    len -= 2;
  }

  sb->len = d - (uint8_t *) sbGet(sb);
}

void sbAppendString(StringBuffer *sb, const char *string)
{
  size_t len = strlen(string);
  sbAppendData(sb, string, len);
}

void sbAppendFormatV(StringBuffer *const sb, const char *const format, va_list ap)
{
  int     n;
  size_t  len;
  va_list ap2;

  len = 128;

  while (len < 4 * 1024 * 1024)
  {
    sbEnsureCapacity(sb, sb->len + len);

    va_copy(ap2, ap);
    n = vsnprintf(sb->data + sb->len, len, format, ap2);
    va_end(ap2);
    /*
     * Platform returned not an error, and the number of bytes was less
     * than the size of the buffer -> we're done
     * Note that some implementations (tru64) nicely guarantee zero terminated strings,
     * so we test for len - 1, not len.
     */
    if (n > -1 && (size_t) n < len - 1)
    {
      sb->len += n;
      sbTerminate(sb);
      break;
    }

    /*
     * Otherwise, increase the buffer smartly
     */

    if (n == -1)
    {
      len *= 2;
    }
    else
    {
      len = n + 2;
    }
  }
}

void sbAppendFormat(StringBuffer *const sb, const char *const format, ...)
{
  va_list ap;

  va_start(ap, format);
  sbAppendFormatV(sb, format, ap);
  va_end(ap);
}

/*
 * Retrieve a value out of a JSON styled message.
 */
bool getJSONValue(const char *message, const char *fieldName, char *value, size_t len)
{
  const char *loc      = message + 1;
  size_t      fieldLen = strlen(fieldName);

  for (;;)
  {
    loc = strstr(loc, fieldName);
    if (!loc)
    {
      return false;
    }
    if (loc[-1] == '"' && loc[fieldLen] == '"' && loc[fieldLen + 1] == ':')
    {
      break;
    }
    loc += fieldLen;
  }

  /* field has been found */
  loc += fieldLen + 2;

  while (isspace(*loc))
  {
    loc++;
  }

  if (strncmp(loc, "null", 4) == 0)
  {
    return false;
  }

  if (*loc != '"')
  {
    while ((isdigit(*loc) || *loc == '.' || *loc == '-' || *loc == 'E' || *loc == 'e' || *loc == '+') && len > 1)
    {
      *value++ = *loc++;
      len--;
    }
    *value = 0;
    return true;
  }

  /* field is string */

  loc++;

  while (len > 1)
  {
    if (*loc == '\\')
    {
      loc++;
      switch (*loc)
      {
        case 'b':
          *value++ = '\b';
          loc++;
          break;
        case 'f':
          *value++ = '\f';
          loc++;
          break;
        case 'n':
          *value++ = '\n';
          loc++;
          break;
        case 'r':
          *value++ = '\r';
          loc++;
          break;
        case 't':
          *value++ = '\t';
          loc++;
          break;
        case 'u': {
          unsigned int n;
          sscanf(loc, "%4x", &n);
          loc += 4;
          *value++ = n & 255; // We're single byte, forget about the high byte...
        }
        break;

        default:
          *value++ = *loc++;
      }
    }
    else if (*loc == '"')
    {
      break;
    }
    else
    {
      *value++ = *loc++;
    }
    len--;
  }
  *value = 0;
  return true;
}

/*
 * Retrieve a lookup list in a JSON styled message.
 * Example: {"value":0,"name":"Under way using engine"}
 */
static bool getJSONLookupList(const char *message, const char *fieldName, char *value, size_t len)
{
  const char *loc      = message + 1;
  size_t      fieldLen = strlen(fieldName);
  const char *end;

  for (;;)
  {
    loc = strstr(loc, fieldName);
    if (!loc)
    {
      return false;
    }
    if (loc[-1] == '"' && loc[fieldLen] == '"' && loc[fieldLen + 1] == ':')
    {
      break;
    }
    loc += fieldLen;
  }

  /* field has been found */
  loc += fieldLen + 2;

  while (isspace(*loc))
  {
    loc++;
  }

  if (strncmp(loc, "null", 4) == 0)
  {
    return false;
  }

  end = strchr(loc, '}');

  if (*loc != '{' || end == NULL)
  {
    logError("Cannot extract lookup for field '%s': it is not a -nv style lookup\n%s\n", fieldName, loc);
    return false;
  }
  end++;
  len--;
  if (end - loc < len)
  {
    len = end - loc;
  }
  memcpy(value, loc, len);
  value[len] = '\0';

  return true;
}

/*
 * Retrieve a name out of a lookup list in a JSON styled message.
 * Example: {"value":0,"name":"Under way using engine"} -> Under way using engine
 */
bool getJSONLookupName(const char *message, const char *fieldName, char *value, size_t len)
{
  char buffer[128];

  if (getJSONLookupList(message, fieldName, buffer, sizeof(buffer)))
  {
    return getJSONValue(buffer, "name", value, len);
  }
  return false;
}

/*
 * Retrieve a value out of a JSON styled message.
 */
bool getJSONLookupValue(const char *message, const char *fieldName, int64_t *value)
{
  char buffer[128];

  if (getJSONLookupList(message, fieldName, buffer, sizeof(buffer)) && getJSONValue(buffer, "value", buffer, sizeof(buffer))
      && sscanf(buffer, "%" PRId64, value) == 1)
  {
    return true;
  }
  return false;
}

char *sbSearchChar(const StringBuffer *const in, char c)
{
  char  *p = sbGet(in);
  size_t i;

  for (i = 0; i < in->len; i++)
  {
    if (p[i] == c)
    {
      return p + i;
    }
  }
  return 0;
}

/*
 * Table 1 - Mapping of ISO 11783 into CAN's Arbitration and Control Fields
29 Bit Identifiers
CAN   ISO 11783 Bit Number
SOF     SOF*      1
ID 28   P 3       2
ID 27   P 2       3
ID 26   P 1       4
ID 23   R 1       5
ID 24   DP        6
ID 23   PF 8      7
ID 22   PF 7      8
ID 21   PF 6      9
ID 20   PF 5     10
ID 19   PF 4     11
ID 18   PF 3     12
SRR (r) SRR*     13
IDE (r) IDE*     14
ID 17   PF 2     15
ID 16   PF 1     16
ID 13   PS 8     17
ID 14   PS 7     18
ID 13   PS 6     19
ID 12   PS 5     20
ID 11   PS 4     21
ID 10   PS 3     22
ID 9    PS 2     23
ID 8    PS 1     24
ID 7    SA 8     25
ID 6    SA 7     26
ID 3    SA 6     27
ID 4    SA 5     28
ID 3    SA 4     29
ID 2    SA 3     30
ID 1    SA 2     31
ID 0    SA 1     32
RTR (x) RTR*     33
r 1     r 1*     34
r 0     r 0*     35
DLC 4   DLC 4    36
DLC 3   DLC 3    37
DLC 2   DLC 2    38
DLC 1   DLC 1    39
Notes:
SOF - Start of Frame Bit P# - ISO 11783 Priority Bit #n
ID## - Identifier Bit #n R# - ISO 11783 Reserved Bit #n
SRR - Substitute Remote Request SA# - ISO 11783 Source Address Bit #n
RTR - Remote Transmission Request Bit DP - ISO 11783 Data Page
IDE - Identifier Extension Bit PF# - ISO 11783 PDU Format Bit #n
r# - CAN Reserved Bit #n PS# - ISO 11783 PDU Specific Bit #n
DLC# - Data Length Code Bit #n *CAN Defined Bit, Unchanged in ISO 11783
(d) - dominant bit 1 Required format of proprietary 11 bit identifiers
(r) - recessive bit

For NMEA2000 the R bit is always 0, but SAE J1939 it is not. J1939 calls
this the "Extended Data Page" (EDP).
*/

void getISO11783BitsFromCanId(unsigned int id, unsigned int *prio, unsigned int *pgn, unsigned int *src, unsigned int *dst)
{
  unsigned char PF  = (unsigned char) (id >> 16);
  unsigned char PS  = (unsigned char) (id >> 8);
  unsigned char RDP = (unsigned char) (id >> 24) & 3; // Use R + DP bits

  if (src)
  {
    *src = (unsigned char) id >> 0;
  }
  if (prio)
  {
    *prio = (unsigned char) ((id >> 26) & 0x7);
  }

  if (PF < 240)
  {
    /* PDU1 format, the PS contains the destination address */
    if (dst)
    {
      *dst = PS;
    }
    if (pgn)
    {
      *pgn = (RDP << 16) + (PF << 8);
    }
  }
  else
  {
    /* PDU2 format, the destination is implied global and the PGN is extended */
    if (dst)
    {
      *dst = 0xff;
    }
    if (pgn)
    {
      *pgn = (RDP << 16) + (PF << 8) + PS;
    }
  }
}

/*
  This does the opposite from getISO11783BitsFromCanId: given n2k fields produces the extended frame CAN id
*/
unsigned int getCanIdFromISO11783Bits(unsigned int prio, unsigned int pgn, unsigned int src, unsigned int dst)
{
  unsigned int canId
      = (src & 0xff) | 0x80000000U; // src bits are the lowest ones of the CAN ID. Also set the highest bit to 1 as n2k uses
  // only extended frames (EFF bit).

  unsigned int PF = (pgn >> 8) & 0xff;

  if (PF < 240)
  { // PDU 1
    canId |= (dst & 0xff) << 8;
    canId |= (pgn << 8);
  }
  else
  { // PDU 2
    canId |= pgn << 8;
  }
  canId |= prio << 26;

  return canId;
}

static void resolve_address(const char *url, char **host, const char **service)
{
  const char *s;
  size_t      hostlen;

  if (strncmp(url, "tcp:", STRSIZE("tcp:")) == 0)
  {
    url += STRSIZE("tcp:");
  }
  while (*url == '/')
  {
    url++;
  }

  s = strchr(url, ':');
  if (s)
  {
    hostlen = s - url;
    s++;
  }
  else
  {
    hostlen = strlen(url);
  }

  *host = malloc(hostlen + 1);
  if (!*host)
  {
    die("Out of memory");
  }
  memcpy(*host, url, hostlen);
  (*host)[hostlen] = 0;

  if (s)
  {
    *service = s;
  }
  else
  {
    *service = "80";
  }
}

SOCKET open_socket_stream(const char *url)
{
  int             sockfd = INVALID_SOCKET;
  int             n;
  struct addrinfo hints, *res, *addr;
  char           *host;
  const char     *service;

  resolve_address(url, &host, &service);

  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  n = getaddrinfo(host, service, &hints, &res);
  if (n != 0)
  {
    logError("Unable to open connection to %s:%s: %s\n", host, service, gai_strerror(n));
  }
  else
  {
    for (addr = res; addr; addr = addr->ai_next)
    {
      sockfd = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
      if (sockfd == INVALID_SOCKET)
      {
        continue;
      }

      if (!connect(sockfd, res->ai_addr, res->ai_addrlen))
      {
        break;
      }
      close(sockfd);
      sockfd = INVALID_SOCKET;
    }
    if (!addr)
    {
      logError("Unable to open connection to %s:%s: %s\n", host, service, strerror(errno));
    }
  }

  freeaddrinfo(res);
  free(host);

  return sockfd;
}

uint8_t scanNibble(char c)
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

int scanHex(char **p, uint8_t *m)
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

int isReady(int fd1, int fd2, int fd3, int timeout)
{
  fd_set         fds;
  fd_set         fdw;
  struct timeval waitfor;
  int            setsize;
  int            r;
  int            ret = 0;

  FD_ZERO(&fds);
  FD_ZERO(&fdw);
  if (fd1 > INVALID_SOCKET)
  {
    FD_SET(fd1, &fds);
  }
  if (fd2 > INVALID_SOCKET)
  {
    FD_SET(fd2, &fds);
  }
  if (fd3 > INVALID_SOCKET)
  {
    FD_SET(fd3, &fdw);
  }
  waitfor.tv_sec  = timeout ? timeout : 10;
  waitfor.tv_usec = 0;
  setsize         = CB_MAX(CB_MAX(fd1, fd2), fd3) + 1;
  r               = select(setsize, &fds, &fdw, 0, &waitfor);
  if (r < 0)
  {
    logAbort("I/O error; restart by quit\n");
  }
  if (r > 0)
  {
    if (fd1 > INVALID_SOCKET && FD_ISSET(fd1, &fds))
    {
      ret |= FD1_ReadReady;
    }
    if (fd2 > INVALID_SOCKET && FD_ISSET(fd2, &fds))
    {
      ret |= FD2_ReadReady;
    }
    if (fd3 > INVALID_SOCKET && FD_ISSET(fd3, &fdw))
    {
      ret |= FD3_WriteReady;
    }
  }
  if (!ret && timeout)
  {
    logAbort("Timeout %ld seconds; restart by quit\n", timeout);
  }
  return ret;
}

/*
 * Send a message to a serial device.
 *
 * The buffer to the device may be slow or limited in size,
 * cater for this.
 *
 */
int writeSerial(int handle, const uint8_t *data, size_t len)
{
  int     retryCount = 5;
  ssize_t written;

  do
  {
    written = write(handle, data, len);
    if (written != -1)
    {
      data += written;
      len -= written;
    }
    else if (errno == EAGAIN)
    {
      retryCount--;
      usleep(25000);
    }
    else
    {
      break;
    }
  } while (len > 0 && retryCount >= 0);

  if (written == -1)
  {
    logError("Write failed: %s\n", strerror(errno));
    return -1;
  }
  else if (len > 0)
  {
    logError("Write timeout: %s\n", strerror(errno));
    return -1;
  }
  return 0;
}

bool parseInt(const char **msg, int *value, int defValue)
{
  char *end;

  *value = strtol(*msg, &end, 10);
  if (end == *msg)
  {
    *value = defValue;
  }
  *msg = end;
  if (*end == ',')
  {
    *msg = end + 1;
  }
  else if (*end != '\0')
  {
    return false;
  }
  return true;
}

bool parseConst(const char **msg, const char *str)
{
  if (strncmp(*msg, str, strlen(str)) == 0)
  {
    *msg += strlen(str);
    return true;
  }
  return false;
}
