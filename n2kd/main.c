/*

Runs a TCP server, single threaded. It opens a COM port representing an UMTS
modems monitor port. Monitoring data is translated and supplied to HTTP sockets
in JSON format suited to putting up on a HTTP page.
It is also possible to send AT requests requested by HTTP sockets.

 The main loop is in serverWork(). It works by checking whether:
 - a TCP client arrives
 - a timeout has expired

(C) 2009-2012, Kees Verruijt, Harlingen, The Netherlands.

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

#define PORT 2597

#define UPDATE_INTERVAL 2       /* Every x seconds send the normal 'once' clients all state */

uint16_t port = PORT;

uint32_t protocol = 1;
unsigned int timeout = 120; /* Timeout when PGN messages expire (no longer retransmitted) */
#define AIS_TIMEOUT (3 * timeout)  /* Note that AIS messsages expiration is 3 * timeout */


/*
 * TCP clients. We keep a simple array of TCP clients, indexed by a small integer.
 * We use the FD_SET bits to keep track of which clients are interesting (open).
 */

int clientIdxMin = 0;
int clientIdxMax = 0;
SOCKET clientFdMax = 0;
fd_set clientSet;
SOCKET clientFd[FD_SETSIZE];
time_t clientTimeout[FD_SETSIZE];
int clientType[FD_SETSIZE];
#define CLIENT_STREAM 1

/*
 * The socket for the TCP server.
 */
SOCKET sockfd = -1;

#ifndef PEEK
const int stdinfd = 0; /* The fd for the stdin port, this receives the analyzed stream of N2K data. */
#else
const int stdinfd = -1; /* Indicate that we can't select() on stdinfd, in which case we use peek() */
#endif

FILE * debugf;
FILE * outf;

#define MIN_PGN (59391)
#define MAX_PGN (131000)
#define ACTISENSE_BEM (0x400000)
#define ACTISENSE_RNG (0x100)

#define PGN_SPACE (ACTISENSE_RNG + MAX_PGN - MIN_PGN)
#define PrnToIdx(prn) ((prn <= MAX_PGN) ? (prn - MIN_PGN) : ((prn <= ACTISENSE_BEM + ACTISENSE_RNG) ? (prn - ACTISENSE_BEM) : -1))

/*
 * We store messages and where they come from.
 *
 * the 'primary key' is the combination of the following 2 fields:
 * - src
 * - key2 (either 0, instance, mmsi or strlen('Reference' field))
 *
 */
typedef struct
{
  uint8_t m_src;
  uint32_t m_key2;
# define m_instance m_key2
# define m_mmsi m_key2
  time_t m_time;
  char * m_text;
} Message;

/*
 * Per PGN we keep a list of messages.
 * We use an 'indefinite' array of messages that is extended at runtime.
 */
typedef struct
{
  unsigned int p_prn;
  unsigned int p_maxSrc;
  char * p_description;
  Message p_message[];
} Pgn;

/*
 * An index from PRN to index in the data[] array. By keeping
 * the PGNs that we have seen coalesced in data[] we can loop over all
 * of them very efficiently.
 */
Pgn * pgnIdx[PGN_SPACE];

/*
 * Support for 512 different PGNs. Since this is more than there are defined
 * by the NMEA this does not need to be variable.
 * Each entry points to the location of pgnIdx[...].
 */
Pgn ** pgnList[512];
size_t maxPgnList;

static char * secondaryKeyList[] =
  { "Instance\":\""
  , "\"Reference\":\""
  , "\"Message ID\":\"" /* Key index 2 */
# define SECONDARY_KEY_MESSAGE_ID (2)
  , "\"User ID\":\""
  };

#ifndef ARRAYSIZE
# define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

/*****************************************************************************************/

int setFdUsed(SOCKET fd)
{
  int i;

  for (i = 0; i <= clientIdxMax; i++)
  {
    if (!clientFd[i])
    {
      break;
    }
  }

  if (i == FD_SETSIZE)
  {
    close(fd);
    return -1;
  }

  clientTimeout[i] = time(0) + UPDATE_INTERVAL;
  clientType[i] = 0;
  clientFd[i] = fd;
  clientIdxMax = CB_MAX(clientIdxMax, i);
  clientFdMax = CB_MAX(clientFdMax, fd);
  logDebug("New client %u %u..%u fd=%d fdMax=%d\n", i, clientIdxMin, clientIdxMax, fd, clientFdMax);
  FD_SET(fd, &clientSet);
  return 0;
}

void closeClient(SOCKET fd)
{
  int i;

  logDebug("closeClient(%d)\n", fd);

  close(fd);
  FD_CLR(fd, &clientSet);

  clientFdMax = 0;
  for (i = clientIdxMin; i <= clientIdxMax; i++)
  {
    if (clientFd[i] == fd)
    {
      clientFd[i] = 0;
      if (i == clientIdxMax)
      {
        clientIdxMax--;
      }
    }
    clientFdMax = CB_MAX(clientFdMax, clientFd[i]);
  }
  logDebug("closeClient(%d) IdMax=%u FdMax=%d\n", fd, clientIdxMax, clientFdMax);
}

# define INITIAL_ALLOC    8192
# define NEXT_ALLOC       4096
# define MAKE_SPACE(x) \
    { if (remain < (size_t)(x)) \
    { \
      alloc += NEXT_ALLOC; \
      state = realloc(state, alloc); \
      remain += NEXT_ALLOC; \
    } }

# define INC_L_REMAIN \
    {l = l + strlen(state + l); remain = alloc - l; }

static char * copyClientState(void)
{
  char separator = '{';
  size_t alloc = INITIAL_ALLOC;
  char * state;
  int i, s;
  Pgn * pgn;
  size_t remain = alloc;
  size_t l;
  time_t now = time(0);

  state = malloc(alloc);
  for (l = 0, i = 0; i < maxPgnList; i++)
  {
    pgn = *pgnList[i];
    MAKE_SPACE(100);
    snprintf(state + l, remain, "%c\"%u\":\n  {\"description\":\"%s\"\n"
            , separator
            , pgn->p_prn
            , pgn->p_description
            );
    INC_L_REMAIN;

    for (s = 0; s < pgn->p_maxSrc; s++)
    {
      Message * m = &pgn->p_message[s];

      if (m->m_time >= now)
      {
        MAKE_SPACE(m->m_text + 32);
        snprintf(state + l, remain, "  ,\"%u_%u\":%s\n"
                , m->m_src
                , m->m_key2
                , m->m_text
                );
        INC_L_REMAIN;
      }
    }
    strcpy(state + l, "  }\n");
    INC_L_REMAIN;

    separator = ',';
  }
  if (separator == ',')
  {
    strcpy(state + l, "}\n");
  }
  else
  {
    strcpy(state + l, "\n");
  }

  return state;
}

static void sendClientState(SOCKET fd, char * state)
{
  send(fd, state, strlen(state), 0);
}

static void tcpServer()
{
  struct sockaddr_in serverAddr;
  int sockAddrSize = sizeof(serverAddr);
  int r;
  int on = 1;

  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if (sockfd == INVALID_SOCKET)
  {
    die("Unable to open server socket");
  }
  memset((char *) &serverAddr, 0, sockAddrSize);
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(port);
  serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char *) &on, (socklen_t) sizeof(on));

  r = bind(sockfd, (struct sockaddr *) &serverAddr, sockAddrSize);
  if (r == INVALID_SOCKET)
  {
    die("Unable to bind server socket");
  }

  r = listen(sockfd, 10);
  if (r == INVALID_SOCKET)
  {
    die("Unable to listen to server socket");
  }

# ifdef O_NONBLOCK
  {
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
  }
# else
  {
    int ioctlOptionValue = 1;

    ioctl(sockfd, FIONBIO, &ioctlOptionValue);
  }
# endif

  logDebug("TCP server fd=%d\n", sockfd);
  logInfo("TCP server listening on port %d\n", port);
  setFdUsed(sockfd);
}

void addTCPClient(void)
{
  SOCKET r;
  struct sockaddr_in clientAddr;
  socklen_t clientAddrLen;


  for (;;)
  {
    clientAddrLen = sizeof(clientAddr);
    r = accept(sockfd, (struct sockaddr *) &clientAddr, &clientAddrLen);
    if (r == INVALID_SOCKET)
    {
      /* No socket ready, just ignore */
      return;
    }

    /* New client found, mark it as such */
    if (setFdUsed(r) < 0)
    {
      /* Too many open clients, ignore */
      return;
    }
  }
}

void processMessageClients(char * message)
{
  size_t i;
  SOCKET fd;

  for (i = clientIdxMin; i <= clientIdxMax; i++)
  {
    fd = clientFd[i];
    if (fd && clientType[i] == CLIENT_STREAM)
    {
      if (send(fd, message, strlen(message), 0) < (int) strlen(message))
      {
        closeClient(fd);
      }
    }
  }
}

void checkClients(void)
{
  fd_set writeSet;
  struct timeval timeout = {0, 0};
  int r;
  int i;
  SOCKET fd;
  time_t now = 0;
  char * state = 0;

  if (clientIdxMax >= 0)
  {
    writeSet = clientSet;
    FD_CLR(sockfd, &writeSet);
    r = select(clientFdMax + 1, 0, &writeSet, 0, &timeout);

    if (r > 0)
    {
      for (i = clientIdxMin; i <= clientIdxMax; i++)
      {
        fd = clientFd[i];
        if (fd == sockfd)
        {
          continue;
        }
        logDebug("checkClients i=%u fd=%d\n", i, fd);
        if (fd && fd > clientFdMax)
        {
          logAbort("Inconsistent: fd[%u]=%d, max=%d\n", i, fd, clientFdMax);
        }
        if (fd && FD_ISSET(fd, &clientSet))
        {
          if (!FD_ISSET(fd, &writeSet))
          {
            closeClient(fd);
          }
          else
          {
            if (!now) now = time(0);
            if (clientTimeout[i] && clientTimeout[i] < now)
            {
              if (!state)
              {
                state = copyClientState();
              }
              sendClientState(fd, state);
              if (clientType[i] == CLIENT_STREAM)
              {
                clientTimeout[i] = time(0) + UPDATE_INTERVAL;
              }
              else
              {
                closeClient(fd);
              }
            }
          }
        }
      }
    }
  }

  if (state)
  {
    free(state);
  }
}

void handleMessageByte(char c)
{
  static char readLine[4096], *readBegin = readLine, *s, *e = 0;
  size_t r;
  Message * m;
  int i, idx, k;
  int src, key2, dst, prn;
  Pgn * pgn;
  time_t now;

  if ((c != '\n') && (readBegin < readLine + sizeof(readLine)))
  {
    *readBegin++ = c;
    return;
  }
  *readBegin = 0;
  key2 = 0;
  now = time(0);

  r = readBegin - readLine;
  readBegin = readLine;
  if (!strstr(readLine, "\"fields\":"))
  {
#ifdef DEBUG
    logDebug("Ignore pgn %u without fields\n", prn);
#endif
    return;
  }
  if (memcmp(readLine, "{\"timestamp", 11) != 0)
  {
    logDebug("Ignore '%s'\n", readLine);
    return;
  }
  if (memcmp(readLine + r - 2, "}}", 2) != 0)
  {
    logDebug("Ignore '%s' (end)\n", readLine);
    return;
  }
#ifdef DEBUG
  logDebug("Message :%s:\n", readLine);
#endif
  s = strstr(readLine, "\"src\":");
  if (s)
  {
    if (sscanf(s + sizeof("\"src\":"), "%u\",\"dst\":\"%u\",\"pgn\":\"%u\"", &src, &dst, &prn))
    {
#ifdef DEBUG
      logDebug("prn=%u src=%u\n", prn, src);
#endif
    }
  }
  if (!prn || !src)
  {
    return;
  }
  if (prn > MAX_PGN)
  {
    return;
  }

  for (k = 0; k < ARRAYSIZE(secondaryKeyList); k++)
  {
    s = strstr(readLine, secondaryKeyList[k]);
    if (s)
    {
      s += strlen(secondaryKeyList[k]);
      if (sscanf(s, "%u", &key2))
      {
#ifdef DEBUG
        logDebug("%s=%u\n", secondaryKeyList[k], key2);
#endif
        break;
      }
      else
      {
        if (k == SECONDARY_KEY_MESSAGE_ID)
        {
          s = strchr(s, '"') - 1;
          src = *s - 'A';
        }
        else
        {
          key2 = strchr(s, '"') - s;
          break;
        }
      }
    }
  }

  processMessageClients(readLine);

  idx = PrnToIdx(prn);
  if (idx < 0)
  {
    logAbort("PRN %d is out of range\n", prn);
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
    pgnIdx[idx] = pgn;
    pgnList[maxPgnList++] = &pgnIdx[idx];
  }

  if (!pgn->p_description)
  {
    pgn->p_prn = prn;
    s = strstr(readLine, "\"description\":");
    if (s)
    {
      s = s + sizeof("\"description\":");
      e = strchr(s, '"');
      if (!e)
      {
        logDebug("Cannot find end of description in %s\n", s);
        return;
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
    if (pgn->p_message[i].m_src == src && pgn->p_message[i].m_key2 == key2)
    {
      break;
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
        pgn->p_message[i].m_key2 = key2;
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
    pgn = realloc(pgnIdx[idx], newSize);
    if (!pgn)
    {
      logAbort("Out of memory allocating %u bytes", newSize);
    }
    pgnIdx[idx] = pgn;
    pgn->p_message[i].m_src = (uint8_t) src;
    pgn->p_message[i].m_key2 = key2;
    pgn->p_message[i].m_text = 0;
  }

  m = &pgn->p_message[i];
  if (m->m_text)
  {
    if (strlen(m->m_text) < r)
    {
      m->m_text = realloc(m->m_text, r + 1);
    }
  }
  else
  {
    m->m_text = malloc(r + 1);
  }
  if (!m->m_text)
  {
    logAbort("Out of memory allocating %u bytes", r + 1);
  }
  strcpy(m->m_text, readLine);
  m->m_time = now + (key2 > 256 ? AIS_TIMEOUT : timeout);
}

void handleMessage()
{
  char readBuffer[16384];
  char * r;
  size_t i, len;
 
  r = fgets(readBuffer, sizeof(readBuffer), stdin);

  if (r)
  {
    len = strlen(r);
    if (outf)
    {
      fwrite(readBuffer, sizeof(char), len, outf);
    }
    for (i = 0; i < len; i++)
    {
      handleMessageByte(readBuffer[i]);
    }
  }
  else
  {
    logAbort("Error on reading stdin\n");
  }
}

void handleClientRequest(int i)
{
  unsigned char readBuffer[4096];
  ssize_t r;

  r = recv(clientFd[i], readBuffer, sizeof(readBuffer) - 1, 0);
  if (r > 0)
  {
    readBuffer[r] = 0;
    if (strchr((char *) readBuffer, '-'))
    {
      clientType[i] = CLIENT_STREAM;
    }
    else
    {
      logDebug("Ignore incoming message '%s'\n", readBuffer);
    }
  }
}

void handleEvents(void)
{
  fd_set readSet;
  struct timeval timeout = {1, 0};
  int r;
  size_t i;
  SOCKET fd;

  logDebug("handleEvents maxfd = %d\n", clientFdMax);

  readSet = clientSet;

  r = select(clientFdMax + 1, &readSet, 0, 0, &timeout);

  if (r > 0)
  {
    if (sockfd >= 0 && FD_ISSET(sockfd, &readSet))
    {
      addTCPClient();
    }

#ifndef PEEK
    if (stdinfd >= 0 && FD_ISSET(stdinfd, &readSet))
    {
      handleMessage();
    }
#endif

    for (i = clientIdxMin; i <= clientIdxMax; i++)
    {
      fd = clientFd[i];

      if (fd && FD_ISSET(fd, &readSet))
      {
        handleClientRequest(i);
      }
    }
  }
}

void doServerWork(void)
{
  for (;;)
  {

#ifdef PEEK
    if (PEEK)
    {
      handleMessage();
    }
#endif

    /* Do a range of non-blocking operations */
    handleEvents();          /* Process incoming requests on all clients */
    checkClients();
  }
}

int main (int argc, char **argv)
{
  setProgName(argv[0]);

  FD_ZERO(&clientSet);

# ifdef WIN32
  initWin32();
# endif
# ifndef PEEK
  setFdUsed(stdinfd);
# endif

  while (argc > 1)
  {
    if (strcasecmp(argv[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
    }
    else if (strcasecmp(argv[1], "-q") == 0)
    {
      setLogLevel(LOGLEVEL_ERROR);
    }
    else if (strcasecmp(argv[1], "-o") == 0)
    {
      outf = stdout;
    }
    else if (strcasecmp(argv[1], "-t") == 0)
    {
      if (argc > 2)
      {
        sscanf(argv[2], "%u", &timeout);
        argc--, argv++;
      }
    }
    else if (strcasecmp(argv[1], "-p") == 0)
    {
      if (argc > 2)
      {
        unsigned int uPort;

        if (sscanf(argv[2], "%u", &uPort))
        {
          port = (uint16_t) uPort;
        }
        argc--, argv++;
      }
    }
    else
    {
      fprintf(stderr, "usage: n2kd [-d] [-o] [-t <timeout>] [-p <port>]\n\n"COPYRIGHT);
      exit(1);
    }
    argc--, argv++;
  }

  tcpServer();

  clientIdxMin = 0;
  clientIdxMax = 0;

  doServerWork();

  exit(0);
}

