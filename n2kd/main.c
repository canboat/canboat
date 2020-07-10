//#define DEBUG
/*

Runs a TCP server, single threaded. It reads JSON styled NMEA 2000 records (lines)
from stdin, collects this data and sends this out on three types of TCP clients:

- Non stream JSON type get all accumulated data.
- Stream JSON type just receive exactly the same messages as this program
  receives.
- NMEA0183 stream type get those messages which this program knows how to translate
  into NMEA0183. The two letter talkers is the hexadecimal code for the NMEA2000
  sender.

(C) 2009-2013, Kees Verruijt, Harlingen, The Netherlands.

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

/* The code ignores the result for write, but closes the socket anyway, so there is nothing to be gained */
#pragma GCC diagnostic ignored "-Wunused-result"

#include <signal.h>
#include <sys/select.h>
#include "common.h"
#include "n2kd.h"
#include "nmea0183.h"

#define PORT 2597

#define UPDATE_INTERVAL (500) /* Every x milliseconds send the normal 'once' clients all state */

uint16_t port      = PORT;
char *   srcFilter = 0;
bool     rateLimit;

uint32_t protocol = 1;
int      debug    = 0;

#define SENSOR_TIMEOUT (120)       /* Timeout when PGN messages expire (no longer retransmitted) */
#define AIS_TIMEOUT (3600)         /* AIS messages expiration is much longer */
#define SONICHUB_TIMEOUT (8640000) /* SonicHub messages expiration is basically indefinite */
#define CLAIM_TIMEOUT (8640000)    /* .. as are address claims and device names */

static void closeStream(int i);

typedef void (*ReadHandler)(int i);
/* ... is the prototype for the following types of read-read file descriptors: */
static void handleClientRequest(int i);
static void checkEof(int i);
static void acceptAISClient(int i);
static void acceptRawInputClient(int i);
static void acceptJSONClient(int i);
static void acceptJSONStreamClient(int i);
static void acceptNMEA0183StreamClient(int i);

/*
 * TCP clients or servers. We keep an array of TCP sockets, indexed by a small integer.
 * We use the FD_SET bits to keep track of which sockets are active (open).
 */

typedef enum StreamType
{
  SOCKET_TYPE_ANY,
  CLIENT_AIS,
  CLIENT_INPUT_STREAM,
  CLIENT_JSON,
  CLIENT_JSON_STREAM,
  CLIENT_NMEA0183_STREAM,
  SERVER_AIS,
  SERVER_INPUT_STREAM,
  SERVER_JSON,
  SERVER_JSON_STREAM,
  SERVER_NMEA0183_STREAM,
  DATA_INPUT_STREAM,
  DATA_OUTPUT_SINK,
  DATA_OUTPUT_COPY,
  DATA_OUTPUT_STREAM,
  SOCKET_TYPE_MAX
} StreamType;

const char *streamTypeName[] = {"Any",
                                "AIS client",
                                "Raw input client",
                                "JSON client",
                                "JSON stream",
                                "NMEA0183 stream",
                                "AIS server",
                                "Raw input server",
                                "JSON server",
                                "JSON stream server",
                                "NMEA0183 stream server",
                                "Data input stream",
                                "Data output sink",
                                "Data output copy",
                                "Data output stream",
                                "<Max>"};

ReadHandler readHandlers[SOCKET_TYPE_MAX] = {0,
                                             handleClientRequest,
                                             handleClientRequest,
                                             handleClientRequest,
                                             handleClientRequest,
                                             checkEof,
                                             acceptAISClient,
                                             acceptRawInputClient,
                                             acceptJSONClient,
                                             acceptJSONStreamClient,
                                             acceptNMEA0183StreamClient,
                                             handleClientRequest,
                                             0,
                                             0,
                                             0};

int    socketIdxMin = 0;
int    socketIdxMax = 0;
SOCKET socketFdMax  = 0;
fd_set activeSet;
fd_set readSet;
fd_set writeSet;

typedef struct StreamInfo
{
  SOCKET      fd;
  StreamType  type;
  int64_t     timeout;
  ReadHandler readHandler;
  char        buffer[32768]; /* Lines longer than this might get into trouble */
  size_t      len;
} StreamInfo;

StreamInfo stream[FD_SETSIZE];

const int stdinfd  = 0; /* The fd for the stdin port, this receives the analyzed stream of N2K data. */
const int stdoutfd = 1; /* Possible fd for the stdout port */

int outputIdx = -1;

FILE *debugf;

StreamType outputType = DATA_OUTPUT_STREAM;

StringBuffer tcpMessage;  /* Buffer for sending to TCP clients */
StringBuffer outMessage;  /* Buffer for sending to stdout */
StringBuffer nmeaMessage; /* Buffer for sending to NMEA0183 TCP clients */

#define MIN_PGN (59391)
#define MAX_PGN (131000)
#define CANBOAT_RNG (CANBOAT_PGN_END - CANBOAT_PGN_START + 1)
#define NMEA_RNG (MAX_PGN - MIN_PGN + 1)

#define PGN_SPACE (CANBOAT_RNG + NMEA_RNG)
#define PrnToIdx(prn)    \
  ((prn <= MAX_PGN)      \
       ? (prn - MIN_PGN) \
       : ((prn <= CANBOAT_PGN_START + CANBOAT_RNG && prn >= CANBOAT_PGN_START) ? (prn + NMEA_RNG - CANBOAT_PGN_START) : -1))

/*
 * We store messages and where they come from.
 *
 * the 'primary key' is the combination of the following 2 fields:
 * - src
 * - key2 (value of some field in the message, or null)
 *
 */
typedef struct
{
  uint8_t m_src;
  char *  m_key2;
  time_t  m_time;
  char *  m_text;
} Message;

/*
 * Per PGN we keep a list of messages.
 * We use an 'indefinite' array of messages that is extended at runtime.
 */
typedef struct
{
  unsigned int p_prn;
  unsigned int p_maxSrc;
  char *       p_description;
  Message      p_message[];
} Pgn;

/*
 * An index from PRN to index in the data[] array. By keeping
 * the PGNs that we have seen coalesced in data[] we can loop over all
 * of them very efficiently.
 */
Pgn *pgnIdx[PGN_SPACE];

/*
 * Support for 512 different PGNs. Since this is more than there are defined
 * by the NMEA this does not need to be variable.
 * Each entry points to the location of pgnIdx[...].
 */
Pgn ** pgnList[512];
size_t maxPgnList;

/*
 * If one of the fiels is named like one of these then we index
 * the array by its value as well.
 *
 * The easiest insight is that an AIS transmission from a particular User ID
 * is completely separate from that of any other.
 */
static char *secondaryKeyList[] = {
    "Instance\"",        // A different tank or sensor. Note no leading " so any instance will do.
    "\"Reference\"",     // A different type of data value, for instance "True" and "Apparent"
    "\"User ID\"",       // Different AIS transmission source (station)
    "\"Message ID\"",    // Different AIS transmission source (station)
    "\"Proprietary ID\"" // Different SonicHub item
};

static int secondaryKeyTimeout[] = {SENSOR_TIMEOUT, SENSOR_TIMEOUT, AIS_TIMEOUT, AIS_TIMEOUT, SONICHUB_TIMEOUT, SENSOR_TIMEOUT};

/* Characters that occur between key name and value */
#define SKIP_CHARACTERS "\": "

#ifndef ARRAYSIZE
#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))
#endif

/*****************************************************************************************/

static void breakHere(void)
{
  abort();
}

int64_t epoch(void)
{
  struct timeval t;

  if (gettimeofday(&t, 0))
  {
    logAbort("Error on obtaining wall clock\n");
  }
  return (int64_t) t.tv_sec * 1000 + t.tv_usec / 1000;
}

int setFdUsed(SOCKET fd, StreamType ct)
{
  int i;

  /* Find a free entry in socketFd(i) */
  for (i = 0; i <= socketIdxMax; i++)
  {
    if (stream[i].fd == INVALID_SOCKET || stream[i].fd == fd)
    {
      break;
    }
  }

  if (i == FD_SETSIZE)
  {
    logError("Already %d active streams, ignoring new one\n", FD_SETSIZE);
    close(fd);
    return -1;
  }

  stream[i].fd          = fd;
  stream[i].timeout     = epoch() + UPDATE_INTERVAL;
  stream[i].type        = ct;
  stream[i].readHandler = readHandlers[ct];

  FD_SET(fd, &activeSet);
  if (stream[i].readHandler)
  {
    FD_SET(fd, &readSet);
  }
  else
  {
    FD_CLR(fd, &readSet);
  }

  switch (stream[i].type)
  {
    case CLIENT_AIS:
    case CLIENT_JSON:
    case CLIENT_JSON_STREAM:
    case CLIENT_NMEA0183_STREAM:
    case DATA_OUTPUT_STREAM:
    case DATA_OUTPUT_COPY:
      FD_SET(fd, &writeSet);
      break;
    default:
      FD_CLR(fd, &writeSet);
  }

  socketIdxMax = CB_MAX(socketIdxMax, i);
  socketFdMax  = CB_MAX(socketFdMax, fd);
  logDebug("New %s %u (%u..%u fd=%d fdMax=%d)\n", streamTypeName[stream[i].type], i, socketIdxMin, socketIdxMax, fd, socketFdMax);

  return i;
}

static void closeStream(int i)
{
  int j;

  close(stream[i].fd);
  FD_CLR(stream[i].fd, &activeSet);
  FD_CLR(stream[i].fd, &readSet);
  FD_CLR(stream[i].fd, &writeSet);

  stream[i].fd = INVALID_SOCKET; /* Free for re-use */
  if (i == socketIdxMax)
  {
    socketIdxMax = 0;
    socketFdMax  = 0;
    for (j = i - 1; j >= 0; j--)
    {
      if (stream[j].fd != INVALID_SOCKET)
      {
        socketIdxMax = CB_MAX(socketIdxMax, j);
        socketFdMax  = CB_MAX(socketFdMax, stream[j].fd);
      }
    }
  }
  logDebug("closeStream(%d) (%u..%u fdMax=%d)\n", i, socketIdxMin, socketIdxMax, socketFdMax);
}

static char *getFullStateJSON(StreamType stream)
{
  StringBuffer state     = sbNew;
  char         separator = '{';
  time_t       now       = time(0);

  int    i, s;
  Pgn *  pgn;
  size_t l;

  for (l = 0, i = 0; i < maxPgnList; i++)
  {
    pgn = *pgnList[i];

    // AIS data only goes to AIS clients, non-AIS data to non-AIS clients, but
    // PRNs 129026 and 129029 go to both.
    if ((stream == CLIENT_AIS) == (strncmp(pgn->p_description, "AIS", 3) == 0) || pgn->p_prn == 129026 || pgn->p_prn == 129029)
    {
      sbAppendFormat(&state, "%c\"%u\":\n  {\"description\":\"%s\"\n", separator, pgn->p_prn, pgn->p_description);

      for (s = 0; s < pgn->p_maxSrc; s++)
      {
        Message *m = &pgn->p_message[s];

        if (m->m_time >= now)
        {
          sbAppendFormat(&state, "  ,\"%u%s%s\":%s", m->m_src, m->m_key2 ? "_" : "", m->m_key2 ? m->m_key2 : "", m->m_text);
        }
      }
      sbAppendString(&state, "  }\n");

      separator = ',';
    }
  }
  if (separator == ',')
  {
    sbAppendString(&state, "}\n");
  }
  else
  {
    sbAppendString(&state, "\n");
  }

  logDebug("state %d bytes\n", sbGetLength(&state));
  return state.data;
}

static void tcpServer(uint16_t port, StreamType st)
{
  struct sockaddr_in serverAddr;
  int                sockAddrSize = sizeof(serverAddr);
  int                r;
  int                on = 1;
  SOCKET             s;

  s = socket(PF_INET, SOCK_STREAM, 0);
  if (s == INVALID_SOCKET)
  {
    die("Unable to open server socket");
  }
  memset((char *) &serverAddr, 0, sockAddrSize);
  serverAddr.sin_family      = AF_INET;
  serverAddr.sin_port        = htons(port);
  serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char *) &on, (socklen_t) sizeof(on));

  r = bind(s, (struct sockaddr *) &serverAddr, sockAddrSize);
  if (r == INVALID_SOCKET)
  {
    die("Unable to bind server socket");
  }

  r = listen(s, 10);
  if (r == INVALID_SOCKET)
  {
    die("Unable to listen to server socket");
  }

#ifdef O_NONBLOCK
  {
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
  }
#else
  {
    int ioctlOptionValue = 1;

    ioctl(s, FIONBIO, &ioctlOptionValue);
  }
#endif

  logDebug("TCP server fd=%d\n", s);
  setFdUsed(s, st);
}

static void startTcpServers(void)
{
  tcpServer(port, SERVER_JSON);
  logInfo("TCP JSON server listening on port %d\n", port);
  tcpServer(port + 1, SERVER_JSON_STREAM);
  logInfo("TCP JSON stream server listening on port %d\n", port + 1);
  tcpServer(port + 2, SERVER_NMEA0183_STREAM);
  logInfo("TCP NMEA0183 server listening on port %d\n", port + 2);
  tcpServer(port + 3, SERVER_INPUT_STREAM);
  logInfo("TCP input stream server listening on port %d\n", port + 3);
  tcpServer(port + 4, SERVER_AIS);
  logInfo("TCP AIS server listening on port %d\n", port + 4);
}

void acceptClient(SOCKET s, StreamType ct)
{
  SOCKET             r;
  struct sockaddr_in clientAddr;
  socklen_t          clientAddrLen;

  for (;;)
  {
    clientAddrLen = sizeof(clientAddr);
    r             = accept(s, (struct sockaddr *) &clientAddr, &clientAddrLen);
    if (r == INVALID_SOCKET)
    {
      /* No socket ready, just ignore */
      return;
    }

    /* New client found, mark it as such */
    if (setFdUsed(r, ct) < 0)
    {
      /* Too many open clients, ignore */
      return;
    }
  }
}

static void acceptAISClient(int i)
{
  acceptClient(stream[i].fd, CLIENT_AIS);
}

static void acceptRawInputClient(int i)
{
  acceptClient(stream[i].fd, CLIENT_INPUT_STREAM);
}

static void acceptJSONClient(int i)
{
  acceptClient(stream[i].fd, CLIENT_JSON);
}

static void acceptJSONStreamClient(int i)
{
  acceptClient(stream[i].fd, CLIENT_JSON_STREAM);
}

static void acceptNMEA0183StreamClient(int i)
{
  acceptClient(stream[i].fd, CLIENT_NMEA0183_STREAM);
}

void writeAllClients(void)
{
  fd_set         ws;
  struct timeval timeout = {0, 0};
  int            r;
  int            i;
  SOCKET         fd;
  int64_t        now      = 0;
  char *         aisState = 0;
  char *         state    = 0;

  logDebug("writeAllClients tcp.len=%d\n", tcpMessage.len);
  FD_ZERO(&ws);

  if (socketIdxMax >= 0)
  {
    ws = writeSet;
    r  = select(socketFdMax + 1, 0, &ws, 0, &timeout);
    logDebug("write to %d streams (%u..%u fdMax=%d)\n", r, socketIdxMin, socketIdxMax, socketFdMax);

    for (i = socketIdxMin; r > 0 && i <= socketIdxMax; i++)
    {
      fd = stream[i].fd;
      if (fd < 0)
      {
        continue;
      }
      if (fd > 8192)
      {
        logAbort("Stream %d contains invalid fd %d\n", i, fd);
      }
      if (fd > socketFdMax)
      {
        logAbort("Inconsistent: fd[%u]=%d, fdMax=%d\n", i, fd, socketFdMax);
      }
      if (FD_ISSET(fd, &ws))
      {
        logDebug("%s i=%u fd=%d writable=%d\n", streamTypeName[stream[i].type], i, fd, FD_ISSET(fd, &ws));
        r--;
        if (!now)
          now = epoch();

        switch (stream[i].type)
        {
          case CLIENT_AIS:
            if (stream[i].timeout && stream[i].timeout < now)
            {
              if (!aisState)
              {
                aisState = getFullStateJSON(CLIENT_AIS);
              }
              write(fd, aisState, strlen(aisState));
              logDebug("JSON: wrote %u to %d, closing\n", strlen(aisState), fd);
              closeStream(i);
            }
            break;
          case CLIENT_JSON:
            if (stream[i].timeout && stream[i].timeout < now)
            {
              if (!state)
              {
                state = getFullStateJSON(CLIENT_JSON);
                logDebug("json=%s", state);
              }
              write(fd, state, strlen(state));
              logDebug("JSON: wrote %u to %d, closing\n", strlen(state), fd);
              closeStream(i);
            }
            break;
          case CLIENT_NMEA0183_STREAM:
            logDebug("NMEA-> %d\n", nmeaMessage.len);
            if (nmeaMessage.len)
            {
              write(fd, nmeaMessage.data, nmeaMessage.len);
            }
            break;
          case CLIENT_JSON_STREAM:
            if (tcpMessage.len)
            {
              write(fd, tcpMessage.data, tcpMessage.len);
            }
            break;
          case DATA_OUTPUT_STREAM:
          case DATA_OUTPUT_COPY:
            if (outMessage.len)
            {
              write(fd, outMessage.data, outMessage.len);
            }
            break;
          default:
            break;
        }
      }
    }
  }

  if (aisState)
  {
    free(aisState);
  }

  if (state)
  {
    free(state);
  }

  outMessage.len  = 0;
  tcpMessage.len  = 0;
  nmeaMessage.len = 0;

#ifdef NEVER
  {
    static int count = 0;
    if (count++ > 100)
    {
      exit(1);
    }
  }
#endif
}

static bool storeMessage(char *line, size_t len)
{
  char *   s, *e = 0, *e2;
  Message *m;
  int      i, idx, k;
  int      src, dst, prn = 0;
  Pgn *    pgn;
  time_t   now;
  char *   key2 = 0;
  int      valid;
  char     value[16];

  now = time(0);

  logDebug("storeMessage(\"%s\",%u)\n", line, len);

  if (!strstr(line, "\"fields\":"))
  {
    logDebug("Ignore: pgn %u without fields\n", prn);
    return false;
  }
  if (memcmp(line, "{\"timestamp", 11) != 0)
  {
    logDebug("Ignore: no timestamp: '%s'\n", line);
    return false;
  }
  if (memcmp(line + len - 2, "}}", 2) != 0)
  {
    logDebug("Ignore: no line end: '%s'\n", line);
    return false;
  }

  if (getJSONValue(line, "src", value, sizeof(value)))
  {
    sscanf(value, "%d", &src);
  }

  if (getJSONValue(line, "dst", value, sizeof(value)))
  {
    sscanf(value, "%d", &dst);
  }

  if (getJSONValue(line, "pgn", value, sizeof(value)))
  {
    sscanf(value, "%d", &prn);
  }

  idx = PrnToIdx(prn);
  logDebug("src=%d dst=%d prn=%d idx=%d\n", src, dst, prn, idx);
  if (idx < 0)
  {
    logError("Ignore: prn %d: '%s'\n", prn, line);
    return false;
  }

  /* Look for a secondary key */
  for (k = 0; k < ARRAYSIZE(secondaryKeyList); k++)
  {
    s = strstr(line, secondaryKeyList[k]);
    if (s)
    {
      logDebug("Found 2nd key %d = %s\n", k, secondaryKeyList[k]);
      s += strlen(secondaryKeyList[k]);
      while (strchr(SKIP_CHARACTERS, *s))
      {
        s++;
      }

      e  = strchr(s, ' ');
      e2 = strchr(s, '"');
      if (!e || e2 < e)
      {
        e = e2;
      }
      if (!e)
      {
        e = s + strlen(s);
      }
      if (e > s && e[-1] == ',')
      {
        e--;
      }
      key2 = malloc(e - s + 1);
      if (!key2)
      {
        logAbort("Out of memory allocating %u bytes", e - s);
      }
      memcpy(key2, s, e - s);
      key2[e - s] = 0;
      break;
    }
  }

  pgn = pgnIdx[idx];
  if (!pgn)
  {
    if (maxPgnList == ARRAYSIZE(pgnList))
    {
      logAbort("Too many PGNs\n");
    }

    pgn = calloc(1, sizeof(Pgn) + sizeof(Message));
    if (!pgn)
    {
      logAbort("Out of memory allocating %u bytes", sizeof(Pgn) + sizeof(Message));
    }
    pgnIdx[idx]           = pgn;
    pgnList[maxPgnList++] = &pgnIdx[idx];
    logDebug("Storing new PGN %d in index %u\n", prn, idx);
  }

  if (!pgn->p_description)
  {
    pgn->p_prn = prn;
    s          = strstr(line, "\"description\":");
    if (s)
    {
      s  = s + sizeof("\"description\":");
      e  = strchr(s, ':');
      e2 = strchr(s, '"');
      if (!e || e2 < e)
      {
        e = e2;
      }
      if (!e)
      {
        logDebug("Cannot find end of description in %s\n", s);
        return false;
      }
      logDebug("New PGN '%.*s'\n", e - s, s);
      pgn->p_description = malloc(e - s + 1);
      if (!pgn->p_description)
      {
        logAbort("Out of memory allocating %u bytes", e - s);
      }
      memcpy(pgn->p_description, s, e - s);
      pgn->p_description[e - s] = 0;
    }
  }

  /* Find existing key */
  for (i = 0; i < pgn->p_maxSrc; i++)
  {
    if (pgn->p_message[i].m_src == src)
    {
      if (pgn->p_message[i].m_key2)
      {
        if (key2 && strcmp(pgn->p_message[i].m_key2, key2) == 0)
        {
          break;
        }
      }
      else
      {
        break;
      }
    }
  }

  /* Reuse expired key ? */
  if (i == pgn->p_maxSrc)
  {
    for (i = 0; i < pgn->p_maxSrc; i++)
    {
      if (pgn->p_message[i].m_time < now)
      {
        pgn->p_message[i].m_src = (uint8_t) src;
        if (pgn->p_message[i].m_key2)
        {
          free(pgn->p_message[i].m_key2);
        }
        pgn->p_message[i].m_key2 = key2;
        key2                     = 0;
        break;
      }
    }
  }

  /* Create new key */
  if (i == pgn->p_maxSrc)
  {
    size_t newSize;

    pgn->p_maxSrc++;
    newSize = sizeof(Pgn) + pgn->p_maxSrc * sizeof(Message);
    pgn     = realloc(pgnIdx[idx], newSize);
    if (!pgn)
    {
      logAbort("Out of memory allocating %u bytes", newSize);
    }
    pgnIdx[idx]              = pgn;
    pgn->p_message[i].m_src  = (uint8_t) src;
    pgn->p_message[i].m_key2 = key2;
    key2                     = 0;
    pgn->p_message[i].m_text = 0;
  }

  m = &pgn->p_message[i];
  if (m->m_text)
  {
    m->m_text = realloc(m->m_text, len + 2);
  }
  else
  {
    m->m_text = malloc(len + 2);
  }
  if (!m->m_text)
  {
    logAbort("Out of memory allocating %u bytes", len + 1);
  }
  memcpy(m->m_text, line, len);
  m->m_text[len]     = '\n';
  m->m_text[len + 1] = 0;

  if (prn == 60928 || prn == 126996)
  {
    valid = CLAIM_TIMEOUT;
  }
  else if (prn == 130816)
  {
    valid = SONICHUB_TIMEOUT;
  }
  else
  {
    valid = secondaryKeyTimeout[k];
  }
  logDebug("stored prn %d timeout=%d 2ndKey=%d\n", prn, valid, k);
  if (key2)
  {
    free(key2);
  }
  m->m_time = now + valid;
  return true;
}
void checkEof(int i)
{
  ssize_t r;
  char buf[100];
  
  logDebug("checkEof: read i=%d\n", i);
  logDebug("checkEof %s i=%d fd=%d \n", streamTypeName[stream[i].type], i, stream[i].fd);
  r = read(stream[i].fd, buf, 99);

  if (r <= 0)
  {
    logDebug("checkEof %s i=%d fd=%d r=%d\n", streamTypeName[stream[i].type], i, stream[i].fd, r);
    if (stream[i].type == DATA_INPUT_STREAM)
    {
      logAbort("EOF on reading stdin\n");
    }
    closeStream(i);
    return;
  }
}


void handleClientRequest(int i)
{
  ssize_t r;
  char *  p;
  size_t  remain;

  if (stream[i].len >= sizeof(stream[i].buffer) - 2)
  {
    logAbort("Input line on stream %d too long: %s\n", i, stream[i].buffer);
  }
  remain = sizeof(stream[i].buffer) - stream[i].len - 2;

  logDebug("handleClientRequest: read i=%d\n", i);
  logDebug("read %s i=%d fd=%d len=%u remain=%u\n", streamTypeName[stream[i].type], i, stream[i].fd, stream[i].len, remain);
  r = read(stream[i].fd, stream[i].buffer + stream[i].len, remain);

  if (r <= 0)
  {
    logDebug("read %s i=%d fd=%d r=%d\n", streamTypeName[stream[i].type], i, stream[i].fd, r);
    if (stream[i].type == DATA_INPUT_STREAM)
    {
      logAbort("EOF on reading stdin\n");
    }
    closeStream(i);
    return;
  }

  stream[i].len += r;
  stream[i].buffer[stream[i].len] = 0;
  while (stream[i].len > 0)
  {
    size_t len;

    p = strchr(stream[i].buffer, '\n');
    if (!p)
    {
      break;
    }
    len = p - stream[i].buffer;
    if (stream[i].type == CLIENT_INPUT_STREAM)
    {
      write(stdoutfd, stream[i].buffer, len + 1);
    }
    else
    {
      sbAppendData(&tcpMessage, stream[i].buffer, len + 1);
      if (stream[i].type != DATA_INPUT_STREAM || stream[outputIdx].type == DATA_OUTPUT_COPY)
      {
        /* Send all TCP client input and the main stdin stream if the mode is -o */
        /* directly to stdout */
        sbAppendData(&outMessage, stream[i].buffer, len + 1);
      }
    }
    *p = 0;
    if (storeMessage(stream[i].buffer, len))
    {
      convertJSONToNMEA0183(&nmeaMessage, stream[i].buffer);
    }
    p++, len++;
    stream[i].len -= len;

    /* Now remove [buffer..p> == the entire line */
    memmove(stream[i].buffer, p, stream[i].len + 1);
  }
}

void checkReadEvents(void)
{
  fd_set         rs;
  struct timeval timeout = {1, 0};
  int            r;
  size_t         i;
  SOCKET         fd;

  logDebug("checkReadEvents fdMax=%d\n", socketFdMax);

  rs = readSet;

  r = select(socketFdMax + 1, &rs, 0, 0, &timeout);

  for (i = socketIdxMin; r > 0 && i <= socketIdxMax; i++)
  {
    fd = stream[i].fd;

    if (fd >= 0 && FD_ISSET(fd, &rs))
    {
      (stream[i].readHandler)(i);
      r--;
    }
  }
}

void doServerWork(void)
{
  for (;;)
  {
    /* Do a range of non-blocking operations */
    checkReadEvents(); /* Process incoming requests on all clients */
    writeAllClients(); /* Check any timeouts on clients */
  }
}

int main(int argc, char **argv)
{
  struct sigaction sa;

  setProgName(argv[0]);

  FD_ZERO(&activeSet);
  FD_ZERO(&readSet);
  FD_ZERO(&writeSet);

  setFdUsed(stdinfd, DATA_INPUT_STREAM);
  outputIdx = setFdUsed(stdoutfd, DATA_OUTPUT_STREAM);

  while (argc > 1)
  {
    if (strcasecmp(argv[1], "-version") == 0)
    {
      printf("%s\n", VERSION);
      exit(0);
    }
    else if (strcasecmp(argv[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
      debug = 1;
    }
    else if (strcasecmp(argv[1], "-q") == 0)
    {
      setLogLevel(LOGLEVEL_ERROR);
    }
    else if (strcasecmp(argv[1], "-o") == 0)
    {
      outputIdx = setFdUsed(stdoutfd, DATA_OUTPUT_COPY);
    }
    else if (strcasecmp(argv[1], "-r") == 0)
    {
      outputIdx = setFdUsed(stdoutfd, DATA_OUTPUT_SINK);
    }
    else if (strcasecmp(argv[1], "--src-filter") == 0 && argc > 2)
    {
      srcFilter = argv[2];
      argc--, argv++;
    }
    else if (strcasecmp(argv[1], "--rate-limit") == 0)
    {
      rateLimit = true;
    }
    else if (strcasecmp(argv[1], "-p") == 0 && argc > 2)
    {
      unsigned int uPort;

      if (sscanf(argv[2], "%u", &uPort))
      {
        port = (uint16_t) uPort;
      }
      argc--, argv++;
    }
    else
    {
      fprintf(stderr,
              "usage: n2kd [-d] [-q] [-o] [-r] [--src-filter <srclist>] [--rate-limit] [-p <port>] | -version\n\n"
              "  -d                      debug mode\n"
              "  -q                      quiet mode\n"
              "  -o                      output mode, send all TCP client data to stdout (as well as stdin)\n"
              "  -r                      restrict mode, send no data to stdout\n"
              "  --src-filter <srclist>  restrict NMEA0183 stream to particular N2K sources\n"
              "  --rate-limit            restrict NMEA0183 stream to one message per source per second\n"
              "  -p <port>               Start servers at <port> instead of 2597\n"
              "  -version                Show version number on stdout\n\n" COPYRIGHT);
      exit(1);
    }
    argc--, argv++;
  }

  startTcpServers();

  /*  Ignore SIGPIPE, this will let a write to a socket that's closed   */
  /*  at the other end just fail instead of raising SIGPIPE             */
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = SIG_IGN;
  sigaction(SIGPIPE, &sa, 0);

  doServerWork();

  exit(0);
}
