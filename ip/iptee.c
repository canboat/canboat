/*

(C) 2009-2014, Kees Verruijt, Harlingen, The Netherlands.

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

#define LINESIZE 1024

bool writeonly = false;

typedef enum ConnectionType
{
  ClientUDP,
  ClientTCP,
  ServerTCP
} ConnectionType;

typedef struct Client
{
  int   fd;
  ConnectionType ct;
  bool  reconnect;
  char *host;
  char *port;
} Client;

Client client[256]; /* Surely this is enough connections? */
int clients;

int ipConnect(const char * host, const char * service, ConnectionType ct)
{
  int sockfd;
  int n;
  struct addrinfo hints, *res, *addr;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = (ct == ServerTCP) ? AF_INET6 : AF_UNSPEC;
  hints.ai_socktype = (ct == ClientTCP) ? SOCK_STREAM : SOCK_DGRAM;

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
      if (!bind(sockfd, res->ai_addr, res->ai_addrlen) && !listen(sockfd, 10))
      {
        break;
      }
    }
    else
    {
      if (!connect(sockfd, res->ai_addr, res->ai_addrlen))
      {
        break;
      }
    }
    close(sockfd);
    sockfd = -1;
  }
  if (!addr)
  {
    logError("Unable to open connection to %s:%s: %s\n", host, service, strerror(errno));
  }

  freeaddrinfo(res);

  if (sockfd >= 0)
  {
    unsigned long nonblock = 1;
    ioctl(sockfd, FIONBIO, &nonblock);  /* Set to non-blocking */
    logInfo("Opened %s %s:%s\n"
            , ((ct == ServerTCP) ? "server for" : "connection to")
            , host, service);
  }

  return sockfd;
}

int storeNewClient(int i, int sockfd)
{
  union
  {
    struct sockaddr any;
    struct sockaddr_in in;
    struct sockaddr_in6 in6;
  } a;

  char portstr[10];
  socklen_t alen = sizeof(a);

  if (getpeername(sockfd, &a.any, &alen))
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
  if (a.any.sa_family == AF_INET)
  {
    inet_ntop(a.any.sa_family, &a.in.sin_addr, client[i].host, INET6_ADDRSTRLEN + 1);
    snprintf(portstr, sizeof(portstr), "%u", ntohs(a.in.sin_port));
    client[i].port = strdup(portstr);
  }
  else if (a.any.sa_family == AF_INET6)
  {
    inet_ntop(a.any.sa_family, &a.in6.sin6_addr, client[i].host, INET6_ADDRSTRLEN + 1);
    snprintf(portstr, sizeof(portstr), "%u", ntohs(a.in6.sin6_port));
    client[i].port = strdup(portstr);
  }
  else
  {
    logAbort("Unknown family %d\n", a.any.sa_family);
  }


  client[i].fd = sockfd;
  client[i].ct = ClientTCP;
  client[i].reconnect = false;

  return 1;
}


int main(int argc, char ** argv)
{
  char * host = 0;
  char * port = 0;
  ConnectionType ct = ClientUDP; // Default client type is UDP
  char msg[LINESIZE];
  int r;
  int i;
  char * line;

  setProgName(argv[0]);

  for (i = 0; i < sizeof(client)/sizeof(client[0]); i++)
  {
    client[i].fd = -1;
  }

  while (argc > 1)
  {
    if (strcasecmp(argv[1], "-w") == 0)
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

      client[clients].fd = -1;
      client[clients].host = host;
      client[clients].port = port;
      client[clients].ct  = ct;
      client[clients].reconnect = true;
      clients++;
      if (clients > sizeof(client)/sizeof(client[0]))
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
    fprintf(stderr, "Usage: iptee [-w] [-t|-u] host port [host port ...]\n\n"
                    "This program forwards stdin to the given TCP and UDP ports.\n"
                    "Stdin is also forwarded to stdout unless -w is used.\n"
                    COPYRIGHT);
    exit(1);
  }
  logInfo("Sending lines to %d servers\n", clients);

  while (fgets(msg, sizeof(msg), stdin))
  {
    for (i = 0; i < clients; i++)
    {
      if ((client[i].fd < 0) && (client[i].reconnect))
      {
        client[i].fd = ipConnect(client[i].host, client[i].port, client[i].ct);
      }
      if (client[i].fd >= 0)
      {
        if (client[i].ct == ServerTCP)
        {
          int sockfd = accept(client[i].fd, 0, 0);

          if (sockfd >= 0)
          {
            int j;

            for (j = 0; j < sizeof(client)/sizeof(client[0]); j++)
            {
              if (client[j].fd == -1 && client[j].reconnect == false)
              {
                storeNewClient(j, sockfd);
                break;
              }
            }
            if (j == sizeof(client)/sizeof(client[0]))
            {
              logError("no room for new client\n");
              close(sockfd);
            }
          }
        }
        else
        {
          r = write(client[i].fd, msg, strlen(msg));
          if (r == 0)
          {
            logError("EOF on %s:%s\n", client[i].host, client[i].port);
            close(client[i].fd);
            client[i].fd = -1;
          }
          if (r < 0)
          {
            logError("error on %s:%s: %s\n", client[i].host, client[i].port, strerror(client[i].fd));
            close(client[i].fd);
            client[i].fd = -1;
          }
        }
      }
    }
    if (!writeonly)
    {
      fputs(msg, stdout);
    }
  }
return 0;
}
