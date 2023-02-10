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

#include <signal.h>

#include "common.h"

#define LINESIZE 1024

bool writeonly = false;

typedef enum ConnectionType
{
  ClientUDP,
  ClientTCP,
  ServerTCP
} ConnectionType;

typedef union sockaddr_union
{
  struct sockaddr     any;
  struct sockaddr_in  in;
  struct sockaddr_in6 in6;
} sockaddr_union;

typedef struct Client
{
  int            fd;
  ConnectionType ct;
  bool           reconnect;
  char *         host;
  char *         port;
  sockaddr_union sockaddr;
  socklen_t      socklen;
} Client;

Client client[256]; /* Surely this is enough connections? */
int    clients;

int ipConnect(const char *host, const char *service, ConnectionType ct, struct sockaddr *sockaddr, socklen_t *socklen)
{
  int             sockfd = -1;
  int             n;
  struct addrinfo hints, *res, *addr;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_INET;
  hints.ai_socktype = (ct == ClientUDP) ? SOCK_DGRAM : SOCK_STREAM;

  n = getaddrinfo(host, service, &hints, &res);
  if (n != 0)
  {
    logError("Unable to open connection to %s:%s: %s\n", host, service, gai_strerror(n));
  }

  for (addr = res; addr; addr = addr->ai_next)
  {
    sockfd = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
    if (sockfd < 0)
    {
      continue;
    }

    if (ct == ServerTCP)
    {
      if (!bind(sockfd, addr->ai_addr, addr->ai_addrlen) && !listen(sockfd, 10))
      {
        break;
      }
    }
    else if (ct == ClientTCP)
    {
      if (!connect(sockfd, addr->ai_addr, addr->ai_addrlen))
      {
        break;
      }
    }
    else /* ClientUDP */
    {
      break;
    }
    close(sockfd);
    sockfd = -1;
  }
  if (!addr)
  {
    logError("Unable to open connection to %s:%s: %s\n", host, service, strerror(errno));
  }
  else
  {
    *socklen = addr->ai_addrlen;
    memcpy(sockaddr, addr->ai_addr, addr->ai_addrlen);
  }

  freeaddrinfo(res);

  if (sockfd >= 0 && ct != ClientUDP)
  {
    unsigned long nonblock = 1;
    ioctl(sockfd, FIONBIO, &nonblock); /* Set to non-blocking */
    logInfo("Opened %s %s:%s\n", ((ct == ServerTCP) ? "server for" : "connection to"), host, service);
  }

  return sockfd;
}

int storeNewClient(int i, int sockfd)
{
  char portstr[10];

  client[i].socklen = sizeof(client[i].sockaddr);
  if (getpeername(sockfd, &client[i].sockaddr.any, &client[i].socklen))
  {
    logError("Unknown incoming client\n");
    close(sockfd);
    return 0;
  }

  if (client[i].host)
  {
    free(client[i].host);
    client[i].host = 0;
  }
  if (client[i].port)
  {
    free(client[i].port);
    client[i].port = 0;
  }

  client[i].host = calloc(INET6_ADDRSTRLEN + 2, 1);
  if (client[i].sockaddr.any.sa_family == AF_INET)
  {
    inet_ntop(client[i].sockaddr.any.sa_family, &client[i].sockaddr.in.sin_addr, client[i].host, INET6_ADDRSTRLEN + 1);
    snprintf(portstr, sizeof(portstr), "%u", ntohs(client[i].sockaddr.in.sin_port));
    client[i].port = strdup(portstr);
  }
  else if (client[i].sockaddr.any.sa_family == AF_INET6)
  {
    inet_ntop(client[i].sockaddr.any.sa_family, &client[i].sockaddr.in6.sin6_addr, client[i].host, INET6_ADDRSTRLEN + 1);
    snprintf(portstr, sizeof(portstr), "%u", ntohs(client[i].sockaddr.in6.sin6_port));
    client[i].port = strdup(portstr);
  }
  else
  {
    logAbort("Unknown family %d\n", client[i].sockaddr.any.sa_family);
  }

  client[i].fd        = sockfd;
  client[i].ct        = ClientTCP;
  client[i].reconnect = false;

  logDebug("New TCP client socket %d addr %s port %s\n", sockfd, client[i].host, portstr);

  if (i >= clients)
  {
    clients = i + 1;
  }
  return 1;
}

int main(int argc, char **argv)
{
  char *         host = 0;
  char *         port = 0;
  ConnectionType ct   = ClientUDP; // Default client type is UDP
  char           msg[LINESIZE];
  int            r;
  int            i;
  size_t         len;

  setProgName(argv[0]);

  signal(SIGPIPE, SIG_IGN);

  for (i = 0; i < sizeof(client) / sizeof(client[0]); i++)
  {
    client[i].fd = -1;
  }

  while (argc > 1)
  {
    if (strcasecmp(argv[1], "-version") == 0)
    {
      printf("%s\n", VERSION);
      exit(0);
    }
    else if (strcasecmp(argv[1], "-w") == 0)
    {
      writeonly = true;
    }
    else if (strcasecmp(argv[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
    }
    else if (strcasecmp(argv[1], "-q") == 0)
    {
      setLogLevel(LOGLEVEL_ERROR);
    }
    else if (strcasecmp(argv[1], "-u") == 0)
    {
      ct = ClientUDP;
    }
    else if (strcasecmp(argv[1], "-t") == 0)
    {
      ct = ClientTCP;
    }
    else if (strcasecmp(argv[1], "-s") == 0)
    {
      ct = ServerTCP;
    }
    else if (!host)
    {
      host = argv[1];
    }
    else
    {
      port = argv[1];

      client[clients].fd        = -1;
      client[clients].host      = host;
      client[clients].port      = port;
      client[clients].ct        = ct;
      client[clients].reconnect = true;
      clients++;
      if (clients > sizeof(client) / sizeof(client[0]))
      {
        logAbort("Too many connections requested\n");
      }
      host = 0;
      port = 0;
    }
    argc--;
    argv++;
  }

  if (!clients)
  {
    fprintf(stderr,
            "Usage: iptee [-w] [-d] [-q] [-s|-t|-u] host port [host port ...] | -version\n\n"
            "This program forwards stdin to the given TCP and UDP ports.\n"
            "Stdin is also forwarded to stdout unless -w is used.\n"
            "\n"
            "Options:\n"
            "-w - writeonly - only write to network clients/servers, not stdout\n"
            "-d - debug     - log debug information\n"
            "-q - quiet     - do not log status information\n"
            "-s - server    - host and port are a TCP server\n"
            "-u - udp       - host and port are a UDP address that data is sent to\n"
            "-t - tcp       - host and port are a TCP server that data is sent to\n" COPYRIGHT);
    exit(1);
  }
  logInfo("Sending lines to %d servers\n", clients);

  while (fgets(msg, sizeof(msg), stdin))
  {
    len = strlen(msg);

    for (i = 0; i < clients; i++)
    {
      if ((client[i].fd < 0) && (client[i].reconnect))
      {
        client[i].fd = ipConnect(client[i].host, client[i].port, client[i].ct, &client[i].sockaddr.any, &client[i].socklen);
      }
      if (client[i].fd >= 0)
      {
        if (client[i].ct == ServerTCP)
        {
          int sockfd = accept(client[i].fd, 0, 0);

          if (sockfd >= 0)
          {
            int j;

            for (j = 0; j < sizeof(client) / sizeof(client[0]); j++)
            {
              if (client[j].fd == -1 && client[j].reconnect == false)
              {
                storeNewClient(j, sockfd);
                break;
              }
            }
            if (j == sizeof(client) / sizeof(client[0]))
            {
              logError("no room for new client\n");
              close(sockfd);
            }
          }
        }
        else if (client[i].ct == ClientTCP)
        {
          r = send(client[i].fd, msg, len, 0);
          if (r == 0)
          {
            logError("EOF on %s:%s\n", client[i].host, client[i].port);
            close(client[i].fd);
            client[i].fd = -1;
          }
          if (r < 0)
          {
            logError("error on %s:%s: %s\n", client[i].host, client[i].port, strerror(errno));
            close(client[i].fd);
            client[i].fd = -1;
          }
        }
        else
        {
          r = sendto(client[i].fd, msg, len, 0, &client[i].sockaddr.any, client[i].socklen);
          if (r < 0)
          {
            logError("error on %s:%s: %s\n", client[i].host, client[i].port, strerror(errno));
            close(client[i].fd);
            client[i].fd = -1;
          }
        }
      }
    }
    if (!writeonly)
    {
      logDebug("Writing %s\n", msg);
      if (write(1, msg, len) == -1)
      {
        logError("Cannot write to stdout\n");
        return 1;
      };
    }
  }
  return 0;
}
