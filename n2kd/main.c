// #define DEBUG
/*

Runs a TCP server, single threaded. It reads JSON styled NMEA 2000 records (lines)
from stdin, collects this data and sends this out on three types of TCP clients:

- Non stream JSON type get all accumulated data except for AIS.
- Stream JSON type just receive exactly the same messages as this program
  receives.
- NMEA0183 stream type get those messages which this program knows how to translate
  into NMEA0183. The two letter talkers is the hexadecimal code for the NMEA2000
  sender.
- Non stream JSON type gets all AIS data.
- Write-only port to write to serial device (NGT-1, iKonvert, YDWG, etc.)


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
char    *srcFilter = 0;
bool     rateLimit;
bool     udp183;
bool     stop;

uint32_t protocol = 1;
bool     unitSI   = false;

struct sockaddr_in udpWildcardAddress;

#define SENSOR_TIMEOUT (120)       /* Timeout when PGN messages expire (no longer retransmitted) */
#define AIS_TIMEOUT (3600)         /* AIS messages expiration is much longer */
#define SONICHUB_TIMEOUT (8640000) /* SonicHub messages expiration is basically indefinite */
#define CLAIM_TIMEOUT (8640000)    /* .. as are address claims and device names */

static void closeStream(int i);

typedef void (*ReadHandler)(int i);
/* ... is the prototype for the following types of read-read file descriptors: */
static void handleClientRequest(int i);
static void closeClientRequest(int i);
static void acceptAISClient(int i);
static void acceptRawInputClient(int i);
static void acceptJSONClient(int i);
static void acceptJSONStreamClient(int i);
static void acceptNMEA0183StreamClient(int i);
static void acceptStatusClient(int i);

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
  CLIENT_STATUS_STREAM,
  SERVER_AIS,
  SERVER_INPUT_STREAM,
  SERVER_JSON,
  SERVER_JSON_STREAM,
  SERVER_NMEA0183_STREAM,
  SERVER_NMEA0183_DATAGRAM,
  SERVER_STATUS,
  DATA_INPUT_STREAM,
  DATA_OUTPUT_SINK,
  DATA_OUTPUT_COPY,
  DATA_OUTPUT_STREAM,
  DATA_OUTPUT_NMEA0183_STREAM,
  SOCKET_TYPE_MAX
} StreamType;

const char *streamTypeName[] = {"Any",
                                "AIS client",
                                "Raw input client",
                                "JSON client",
                                "JSON stream",
                                "NMEA0183 stream",
                                "Status stream",
                                "AIS server",
                                "Raw input server",
                                "JSON server",
                                "JSON stream server",
                                "NMEA0183 stream server",
                                "NMEA0183 datagram server",
                                "Status server",
                                "Data input stream",
                                "Data output sink",
                                "Data output copy",
                                "Data output stream",
                                "Data output NMEA0183 stream",
                                "<Max>"};

ReadHandler readHandlers[SOCKET_TYPE_MAX] = {0,
                                             handleClientRequest,
                                             handleClientRequest,
                                             handleClientRequest,
                                             handleClientRequest,
                                             closeClientRequest,
                                             closeClientRequest,
                                             acceptAISClient,
                                             acceptRawInputClient,
                                             acceptJSONClient,
                                             acceptJSONStreamClient,
                                             acceptNMEA0183StreamClient,
                                             0,
                                             acceptStatusClient,
                                             handleClientRequest,
                                             closeClientRequest,
                                             closeClientRequest,
                                             closeClientRequest,
                                             0};

int    socketIdxMin = 0;
int    socketIdxMax = 0;
SOCKET socketFdMax  = 0;
fd_set activeSet;
fd_set readSet;
fd_set writeSet;

typedef struct StreamInfo
{
  SOCKET       fd;
  StreamType   type;
  uint64_t     timeout;
  ReadHandler  readHandler;
  char         buffer[32768]; /* Lines longer than this might get into trouble */
  StringBuffer writeBuffer;
  size_t       len;
} StreamInfo;

StreamInfo stream[FD_SETSIZE];

const int stdinfd  = 0; /* The fd for the stdin port, this receives the analyzed stream of N2K data. */
const int stdoutfd = 1; /* Possible fd for the stdout port */

bool haveNMEA0183Client = false;

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
  uint8_t  m_src;
  uint32_t m_interval; // Interval to previous m_last
  uint64_t m_time;     // Message valid until this time
  uint64_t m_last;     // When received
  uint32_t m_count;    // How many times received
  char    *m_key2;
  char    *m_text;
} Message;

/*
 * Per PGN we keep a list of messages.
 * We use an 'indefinite' array of messages that is extended at runtime.
 */
typedef struct
{
  unsigned int p_prn;
  unsigned int p_maxSrc;
  char        *p_description;
  Message      p_message[];
} Pgn;

/*
 * An array of pointers to arrays of Pgn, indexed by PrnToIdx(prn);
 * Each pointer points to an array of Pgn structures, dynamically allocated.
 */
Pgn *pgnIdx[PGN_SPACE];

/*
 * Keep track of which pgnIdx[] entries are non zero.
 * Each entry points to the location of pgnIdx[...].
 * This is just a speed up to avoid looping over lots of empty
 * entries in pgnIdx.
 */
Pgn  **pgnList[512];
size_t maxPgnList;

/*
 * If one of the fields is named like one of these then we index
 * the array by its value as well.
 *
 * The easiest insight is that an AIS transmission from a particular User ID
 * is completely separate from that of any other.
 */
static char *secondaryKeyList[] = {
    "Instance\":",        // A different tank or sensor. Note no leading " so any instance will do.
    "\"Reference\":",     // A different type of data value, for instance "True" and "Apparent"
    "\"User ID\":",       // Different AIS transmission source (station)
    "\"Message ID\":",    // Different AIS transmission source (station)
    "\"Proprietary ID\":" // Different SonicHub item
};

static int secondaryKeyTimeout[] = {SENSOR_TIMEOUT, SENSOR_TIMEOUT, AIS_TIMEOUT, AIS_TIMEOUT, SONICHUB_TIMEOUT, SENSOR_TIMEOUT};

/* Characters that occur between key name and value */
#define SKIP_CHARACTERS "\": "

#ifndef ARRAYSIZE
#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))
#endif

/*****************************************************************************************/

/*
 * Return time in milliseconds since UNIX epoch
 */
uint64_t epoch(void)
{
  struct timeval t;

  if (gettimeofday(&t, 0))
  {
    logAbort("Error on obtaining wall clock\n");
  }
  return (uint64_t) t.tv_sec * 1000 + t.tv_usec / 1000;
}

static void setHaveNMEA0183Client(void)
{
  int i;

  haveNMEA0183Client = udp183;
  if (!haveNMEA0183Client)
  {
    for (i = 0; i <= socketIdxMax; i++)
    {
      if (stream[i].fd != INVALID_SOCKET
          && (stream[i].type == CLIENT_NMEA0183_STREAM || stream[i].type == SERVER_NMEA0183_DATAGRAM
              || stream[i].type == DATA_OUTPUT_NMEA0183_STREAM))
      {
        haveNMEA0183Client = true;
        return;
      }
    }
  }
}

static int setFdUsed(SOCKET fd, StreamType ct)
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
  stream[i].writeBuffer = sbNew;

  FD_SET(fd, &activeSet);
  if (stream[i].readHandler)
  {
    FD_SET(fd, &readSet);
  }
  else
  {
    FD_CLR(fd, &readSet);
    /* ignore */ shutdown(fd, SHUT_RD);
  }

  switch (stream[i].type)
  {
    case CLIENT_AIS:
    case CLIENT_JSON:
    case CLIENT_JSON_STREAM:
    case CLIENT_NMEA0183_STREAM:
    case CLIENT_STATUS_STREAM:
    case DATA_OUTPUT_STREAM:
    case DATA_OUTPUT_COPY:
    case DATA_OUTPUT_NMEA0183_STREAM:
    case SERVER_NMEA0183_DATAGRAM:
      FD_SET(fd, &writeSet);
      break;
    default:
      FD_CLR(fd, &writeSet);
      /* ignore */ shutdown(fd, SHUT_WR);
  }

#ifdef O_NONBLOCK
  {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }
#else
  {
    int ioctlOptionValue = 1;

    ioctl(fd, FIONBIO, &ioctlOptionValue);
  }
#endif

  socketIdxMax = CB_MAX(socketIdxMax, i);
  socketFdMax  = CB_MAX(socketFdMax, fd);
  logDebug("New %s %u (%u..%u fd=%d fdMax=%d)\n", streamTypeName[stream[i].type], i, socketIdxMin, socketIdxMax, fd, socketFdMax);

  setHaveNMEA0183Client();
  return i;
}

static void closeStream(int i)
{
  int j;

  close(stream[i].fd);
  FD_CLR(stream[i].fd, &activeSet);
  FD_CLR(stream[i].fd, &readSet);
  FD_CLR(stream[i].fd, &writeSet);
  sbClean(&stream[i].writeBuffer);

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
  setHaveNMEA0183Client();
  logDebug("closeStream(%d) (%u..%u fdMax=%d)\n", i, socketIdxMin, socketIdxMax, socketFdMax);
}

static char *getFullStateJSON(StreamType stream, int64_t now)
{
  StringBuffer state     = sbNew;
  char         separator = '{';

  int  i, s;
  Pgn *pgn;

  if (stream == CLIENT_STATUS_STREAM)
  {
    for (i = 0; i < maxPgnList; i++)
    {
      pgn = *pgnList[i];

      sbAppendFormat(&state, "%c\"%u\":\n  {\"description\":\"%s\"\n", separator, pgn->p_prn, pgn->p_description);

      for (s = 0; s < pgn->p_maxSrc; s++)
      {
        Message *m = &pgn->p_message[s];
        char     last_ts[DATE_LENGTH];

        storeTimestamp(last_ts, m->m_last);
        sbAppendFormat(&state,
                       "  ,\"%u%s%s\":{\"last\":\"%s\",\"interval\":%u,\"count\":%u}\n",
                       m->m_src,
                       m->m_key2 ? "_" : "",
                       m->m_key2 ? m->m_key2 : "",
                       last_ts,
                       m->m_interval,
                       m->m_count);
      }
      sbAppendFormat(&state, "  }\n");
      separator = ',';
    }
  }
  else
  {
    for (i = 0; i < maxPgnList; i++)
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
  bool               udp = (st == SERVER_NMEA0183_DATAGRAM);

  s = socket(PF_INET, udp ? SOCK_DGRAM : SOCK_STREAM, 0);
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

  if (!udp)
  {
    r = listen(s, 10);
    if (r == INVALID_SOCKET)
    {
      die("Unable to listen to server socket");
    }
  }
  else
  {
    setsockopt(s, SOL_SOCKET, SO_BROADCAST, (char *) &on, (socklen_t) sizeof(on));
  }

  setFdUsed(s, st);
}

static void startTcpServers(void)
{
  tcpServer(port, SERVER_JSON);
  logInfo("TCP JSON server listening on port %d\n", port);
  tcpServer(port + 1, SERVER_JSON_STREAM);
  logInfo("TCP JSON stream server listening on port %d\n", port + 1);
  if (udp183)
  {
    tcpServer(port + 2, SERVER_NMEA0183_DATAGRAM);
    logInfo("UDP NMEA0183 datagram server sending on port %d\n", port + 2);
  }
  else
  {
    tcpServer(port + 2, SERVER_NMEA0183_STREAM);
    logInfo("TCP NMEA0183 server listening on port %d\n", port + 2);
  }

  tcpServer(port + 3, SERVER_INPUT_STREAM);
  logInfo("TCP input stream server listening on port %d\n", port + 3);

  tcpServer(port + 4, SERVER_AIS);
  logInfo("TCP AIS server listening on port %d\n", port + 4);

  tcpServer(port + 5, SERVER_STATUS);
  logInfo("TCP status server listening on port %d\n", port + 5);
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

static void acceptStatusClient(int i)
{
  acceptClient(stream[i].fd, CLIENT_STATUS_STREAM);
}

static void writeAndClose(int idx, char *data, size_t len)
{
  int r;

#ifdef O_NONBLOCK
  int flags = fcntl(stream[idx].fd, F_GETFL, 0);
  fcntl(stream[idx].fd, F_SETFL, flags & ~O_NONBLOCK);
#else
  int ioctlOptionValue = 0;
  ioctl(stream[idx].fd, FIONBIO, &ioctlOptionValue);
#endif

  while (len > 0)
  {
    r = write(stream[idx].fd, data, len);
    if (r <= 0)
    {
      break;
    }
    len -= r;
    data += r;
  }
  closeStream(idx);
}

static void safeWriteBuffer(int idx, StringBuffer *sb)
{
  int r;

  if (stream[idx].writeBuffer.len > 0 && sb != &stream[idx].writeBuffer)
  {
    // Awww shit, last time we did not write everything, append the
    // new bits to the old unwritten data and try to write the buffered
    // data
    sbAppendData(&stream[idx].writeBuffer, sb->data, sb->len);
    sb = &stream[idx].writeBuffer;
  }

  // We are told the stream is ready to write, but it is in non-blocked
  // mode so it can write less than the buffer.

  r = write(stream[idx].fd, sb->data, sb->len);
  if (r < sb->len)
  {
    if (r <= 0)
    {
      if (errno != EAGAIN)
      {
        if (stream[idx].type == DATA_OUTPUT_COPY || stream[idx].type == DATA_OUTPUT_STREAM)
        {
          logAbort("Cannot write to stdout: %s\n", strerror(errno));
        }
        logError("Closing %s stream %d: %s\n", streamTypeName[stream[idx].type], stream[idx].fd, strerror(errno));
        closeStream(idx);
      }
    }
    else
    {
      if (sb == &stream[idx].writeBuffer)
      {
        // Remove the part of the buffer we did write
        sbDelete(sb, 0, (size_t) r);
      }
      else
      {
        // Store the remaining part in the per-fd writebuffer
        // and write it on the next attempt
        sbAppendData(&stream[idx].writeBuffer, sb->data + r, sb->len - r);
      }
    }
  }
  else
  {
    if (sb == &stream[idx].writeBuffer)
    {
      sbEmpty(sb);
    }
  }
}

static void writeAllClients(void)
{
  fd_set         ws;
  fd_set         es;
  struct timeval timeout = {0, 0};
  int            r;
  int            i;
  SOCKET         fd;
  int64_t        now      = 0;
  char          *aisState = 0;
  char          *state    = 0;

  logDebug("writeAllClients tcp=%d out=%d nmea=%d\n", tcpMessage.len, outMessage.len, nmeaMessage.len);
  FD_ZERO(&ws);
  FD_ZERO(&es);

  if (socketIdxMax >= 0)
  {
    ws  = writeSet;
    es  = writeSet;
    r   = select(socketFdMax + 1, 0, &ws, &es, &timeout);
    now = epoch();
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
      if (FD_ISSET(fd, &es) && fd != stdoutfd)
      {
        logDebug("%s i=%u fd=%d write error, closing\n", streamTypeName[stream[i].type], i, fd);
        closeStream(i);
      }
      else if (FD_ISSET(fd, &ws))
      {
        logDebug("%s i=%u fd=%d writable=%d timeout=%" PRId64 "\n",
                 streamTypeName[stream[i].type],
                 i,
                 fd,
                 FD_ISSET(fd, &ws),
                 stream[i].timeout);
        r--;

        if (stream[i].writeBuffer.len > 0)
        {
          safeWriteBuffer(i, &stream[i].writeBuffer);
          continue;
        }

        switch (stream[i].type)
        {
          case CLIENT_AIS:
            if (stream[i].timeout && stream[i].timeout < now)
            {
              if (!aisState)
              {
                aisState = getFullStateJSON(CLIENT_AIS, now);
              }
              writeAndClose(i, aisState, strlen(aisState));
            }
            break;
          case CLIENT_JSON:
            if (stream[i].timeout && stream[i].timeout < now)
            {
              if (!state)
              {
                state = getFullStateJSON(CLIENT_JSON, now);
                logDebug("json=%s", state);
              }
              writeAndClose(i, state, strlen(state));
            }
            break;
          case CLIENT_STATUS_STREAM: {
            char *statusState = getFullStateJSON(CLIENT_STATUS_STREAM, now);
            writeAndClose(i, statusState, strlen(statusState));
            free(statusState);
          }
          break;
          case CLIENT_NMEA0183_STREAM:
          case DATA_OUTPUT_NMEA0183_STREAM:
            logDebug("NMEA-> %d\n", nmeaMessage.len);
            if (nmeaMessage.len)
            {
              safeWriteBuffer(i, &nmeaMessage);
            }
            break;
          case SERVER_NMEA0183_DATAGRAM:
            logDebug("udp NMEA-> %d\n", nmeaMessage.len);
            if (nmeaMessage.len)
            {
              sendto(fd, nmeaMessage.data, nmeaMessage.len, 0, (struct sockaddr *) &udpWildcardAddress, sizeof(udpWildcardAddress));
            }
            break;
          case CLIENT_JSON_STREAM:
            if (tcpMessage.len)
            {
              safeWriteBuffer(i, &tcpMessage);
            }
            break;
          case DATA_OUTPUT_STREAM:
          case DATA_OUTPUT_COPY:
            if (outMessage.len)
            {
              safeWriteBuffer(i, &outMessage);
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

  sbEmpty(&tcpMessage);
  sbEmpty(&nmeaMessage);
  sbEmpty(&outMessage);

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

static void checkSrcIsKnown(int src, int64_t n)
{
  static const int PRODUCT_INFO_IDX = PrnToIdx(126996);
  int              i;
  Pgn             *pgn = pgnIdx[PRODUCT_INFO_IDX];

  if (src == 0)
  {
    return;
  }

  if (pgn != NULL)
  {
    for (i = 0; i < pgn->p_maxSrc; i++)
    {
      if (pgn->p_message[i].m_src == src && pgn->p_message[i].m_time >= n)
      {
        // Yes, we have product information for this source
        return;
      }
    }
  }

  // Oops, no product info for this source
  logInfo("New device src=%d seen\n", src);

  if (stream[stdoutfd].type == DATA_OUTPUT_COPY || stream[stdoutfd].type == DATA_OUTPUT_STREAM)
  {
    char         strTmp[DATE_LENGTH];
    StringBuffer msg = sbNew;

    sbAppendFormat(&msg, "%s,6,59904,0,%d,3,14,f0,01\n", now(strTmp), src);
    safeWriteBuffer(stdoutfd, &msg);
    sbClean(&msg);
  }
}

static bool storeMessage(char *line, size_t len)
{
  char    *s, *e = 0, *e2;
  Message *m;
  int      i, idx, k;
  int      src = 0, dst = 255, prn = 0;
  Pgn     *pgn;
  int64_t  now  = epoch();
  char    *key2 = 0;
  int      valid;
  char     value[16];

  if (isLogLevelEnabled(LOG_DEBUG))
  {
    if (len > 80)
    {
      logDebug("storeMessage(\"%1.20s...%1.20s\",%u)\n", line, line + len - 20, len);
    }
    else
    {
      logDebug("storeMessage(\"%s\",%u)\n", line, len);
    }
  }

  if (!strstr(line, "\"fields\":") || memcmp(line, "{\"timestamp", 11) != 0)
  {
    if (getJSONValue(line, "version", value, sizeof(value)))
    {
      logInfo("Found datastream from analyzer version %s\n", value);
      if (getJSONValue(line, "units", value, sizeof(value)))
      {
        if (strcmp(value, "si") == 0)
        {
          logInfo("Datastream uses SI units\n");
          unitSI = true;
        }
      }
      return true;
    }
    logDebug("Ignore: no fields and timestamp\n");
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
      if (*s == '{')
      {
        s = strstr(s, "name\":");
        if (s == NULL)
        {
          continue;
        }
        s += STRSIZE("name\":");
      }
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
        logAbort("Out of memory allocating %u bytes\n", e - s);
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
      logAbort("Out of memory allocating %u bytes\n", sizeof(Pgn) + sizeof(Message));
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
        logAbort("Out of memory allocating %u bytes\n", e - s);
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
      logAbort("Out of memory allocating %u bytes\n", newSize);
    }
    pgnIdx[idx]                  = pgn;
    pgn->p_message[i].m_src      = (uint8_t) src;
    pgn->p_message[i].m_key2     = key2;
    key2                         = 0;
    pgn->p_message[i].m_text     = 0;
    pgn->p_message[i].m_interval = 0;
    pgn->p_message[i].m_last     = 0;
    pgn->p_message[i].m_count    = 0;
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
    logAbort("Out of memory allocating %u bytes\n", len + 1);
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
  m->m_time = now + valid * INT64_C(1000);
  if (m->m_last > 0)
  {
    m->m_interval = (uint32_t) (now - m->m_last);
  }
  m->m_last = now;
  m->m_count++;

  if (prn != 126996)
  {
    checkSrcIsKnown(src, now);
  }
  return true;
}

static void handleClientRequest(int i)
{
  ssize_t r;
  char   *p;
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
      stop = true;
    }
    else
    {
      closeStream(i);
      return;
    }
  }
  logDebug("processing stream %d\n", i);

  stream[i].len += r;
  stream[i].buffer[stream[i].len] = 0;

  while (stream[i].len > 0)
  {
    size_t len;

    logDebug("processing stream %d buffer '%-1.20s...' len=%zu\n", i, stream[i].buffer, stream[i].len);
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
    if (storeMessage(stream[i].buffer, len) && haveNMEA0183Client)
    {
      convertJSONToNMEA0183(&nmeaMessage, stream[i].buffer);
    }
    p++, len++;
    stream[i].len -= len;

    /* Now remove [buffer..p> == the entire line */
    memmove(stream[i].buffer, p, stream[i].len + 1);
  }
}

static void closeClientRequest(int i)
{
  ssize_t r;
  char    buf[4];

  logDebug("closeClientRequest: read i=%d\n", i);
  r = read(stream[i].fd, buf, sizeof(buf));

  logDebug("close-on-eof %s r=%d i=%d fd=%d %s\n", streamTypeName[stream[i].type], (int) r, i, stream[i].fd, strerror(errno));
  closeStream(i);
}

static void checkReadEvents(void)
{
  fd_set         rs;
  fd_set         es;
  struct timeval timeout = {1, 0};
  int            r;
  size_t         i;
  SOCKET         fd;

  logDebug("checkReadEvents fdMax=%d\n", socketFdMax);

  rs = readSet;
  es = readSet;

  r = select(socketFdMax + 1, &rs, 0, &es, &timeout);

  for (i = socketIdxMin; r > 0 && i <= socketIdxMax; i++)
  {
    fd = stream[i].fd;

    if (fd >= 0 && FD_ISSET(fd, &es))
    {
      logDebug("%s i=%u fd=%d read error, closing\n", streamTypeName[stream[i].type], i, fd);
      closeStream(i);
    }
    if (fd >= 0 && FD_ISSET(fd, &rs))
    {
      (stream[i].readHandler)(i);
      r--;
    }
  }
}

static void doServerWork(void)
{
  do
  {
    /* Do a range of non-blocking operations */
    checkReadEvents(); /* Process incoming requests on all clients */
    writeAllClients(); /* Check any timeouts on clients */
  } while (!stop);
}

bool parseUDPAddress(char *target, char *port)
{
  int              r;
  struct addrinfo  hints;
  struct addrinfo *res, *res0;

  hints.ai_flags    = 0;
  hints.ai_family   = PF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  hints.ai_flags    = AI_NUMERICHOST;

  r = getaddrinfo(target, port, &hints, &res0);
  if (r)
  {
    return false;
  }
  for (res = res0; res; res = res->ai_next)
  {
    if (res->ai_addrlen == sizeof udpWildcardAddress)
    {
      memcpy(&udpWildcardAddress, res->ai_addr, sizeof udpWildcardAddress);
      return true;
    }
  }
  return false;
}

static void verifyStdin(void)
{
  int     i      = 0;
  size_t  remain = sizeof(stream[0].buffer) - 1;
  ssize_t r;
  char   *line_end;

  for (;;)
  {
    r = read(stream[i].fd, stream[i].buffer + stream[i].len, remain);
    if (r > 0)
    {
      stream[i].len += r;
      stream[i].buffer[stream[i].len] = '\0';
      line_end                        = strchr(stream[i].buffer, '\n');
      if (line_end != NULL)
      {
        if (strstr(stream[i].buffer, "\"version\":") == NULL || strstr(stream[i].buffer, "\"showLookupValues\":true") == NULL)
        {
          logAbort("Standard input must be piped from `analyzer` in `-json -nv` mode\n");
        }
        remain = stream[i].buffer + stream[i].len - (line_end + 1);
        memcpy(stream[i].buffer, line_end + 1, remain);
        stream[i].len = remain;
        break;
      }
    }
    else
    {
      logAbort("Cannot read from piped input from `analyzer`\n");
    }
  }
}

int main(int argc, char **argv)
{
  bool noServers = false;

  struct sigaction sa;

  setProgName(argv[0]);

  FD_ZERO(&activeSet);
  FD_ZERO(&readSet);
  FD_ZERO(&writeSet);

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
    else if (strcasecmp(argv[1], "-u") == 0 && argc > 3)
    {
      if (!parseUDPAddress(argv[2], argv[3]))
      {
        logError("Invalid UDP address + port\n");
        exit(1);
      }
      udp183 = true;
      argc -= 2;
      argv += 2;
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
    else if (strcasecmp(argv[1], "--nmea0183") == 0)
    {
      outputIdx = setFdUsed(stdoutfd, DATA_OUTPUT_NMEA0183_STREAM);
      noServers = true;
    }
    else if (strcasecmp(argv[1], "-fixtime") == 0 && argc > 2)
    {
      setFixedTimestamp(argv[2]);
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
              "  -u <target-addr> <port> Send UDP datagrams to UDP address indicated, can be wildcard address\n"
              "  --nmea0183              Start no servers and send NMEA0183 data on stdout (this is mainly for debugging)\n"
              "  -fixtime str            Print str as timestamp in logging\n"
              "  -version                Show version number on stdout\n\n" COPYRIGHT);
      exit(1);
    }
    argc--, argv++;
  }

  // Read the first line from stdin, this must contain JSON from analyzer
  verifyStdin();
  setFdUsed(stdinfd, DATA_INPUT_STREAM);

  if (!noServers)
  {
    startTcpServers();
  }

  /*  Ignore SIGPIPE, this will let a write to a socket that's closed   */
  /*  at the other end just fail instead of raising SIGPIPE             */
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = SIG_IGN;
  sigaction(SIGPIPE, &sa, 0);

  doServerWork();

  logInfo("N2KD stopping\n");
  fflush(stdout);
  fflush(stderr);
  exit(0);
}
