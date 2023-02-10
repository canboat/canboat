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

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "common.h"
#include "license.h"

#define MAX_FIELDS 20

#define PACKED __attribute__((__packed__))

typedef struct
{
  uint8_t functionCode;
  uint8_t pgn[3];
  int     interval : 32;
  int     offset : 16;
  uint8_t count;
  uint8_t parameters[MAX_FIELDS * 5];
} PACKED command_group_function_t;

void usage(char **argv, char **av)
{
  if (av)
  {
    fprintf(stderr, "Unknown or invalid argument %s\n", av[0]);
  }
  fprintf(stderr, "Usage: %s <dest> <prio> <pgn> <interval> <field>=<value> ...\n\n", argv[0]);
  fprintf(stderr, "       <field> is a decimal value\n");
  fprintf(stderr, "       <value> is a hexadecimal value; the length of the value defines how many bytes are encoded\n");
  fprintf(stderr, "       Maximum # of fields: %d\n\n", MAX_FIELDS);
  fprintf(stderr, "This program uses PGN 126208 to request a device to start reporting a PGN at the given interval.\n");
  fprintf(stderr, "The use of this is thus completely dependent on what the device allows.\n\n" COPYRIGHT);
  exit(1);
}

int main(int argc, char **argv)
{
  int                      ac = argc;
  char                   **av = argv;
  long                     dest;
  long                     pgn;
  long                     interval;
  size_t                   cnt = 0;
  command_group_function_t command;
  size_t                   i, bytes;
  char                    *p, *e;
  uint8_t                 *b;
  uint32_t                 v;
  char                     dateStr[DATE_LENGTH];

  if (ac < 6 || ac > 5 + MAX_FIELDS)
  {
    usage(argv, 0);
  }

  ac--, av++; /* lose the command name */
  dest = strtol(av[0], 0, 10);
  ac--, av++; /* lose the dest */
  /* ignore */ strtol(av[0], 0, 10);
  ac--, av++; /* lose the prio */
  pgn = strtol(av[0], 0, 10);
  ac--, av++; /* lose the pgn */
  interval = strtol(av[0], 0, 10);
  ac--, av++; /* lose the interval */
  b = &command.parameters[0];

  for (; ac; ac--, av++)
  {
    p = strchr(av[0], '=');
    if (p)
    {
      p++;
      *b++ = strtol(av[0], 0, 10);

      v = strtoul(p, &e, 16);
      for (i = (e - p) / 2; i; i--)
      {
        *b++ = (v & 255);
        v    = v >> 8;
      }

      cnt++;
    }
    else
    {
      usage(argv, av);
    }
  }

  command.functionCode = 0;
  command.pgn[0]       = (pgn) &0xff;
  command.pgn[1]       = (pgn >> 8) & 0xff;
  command.pgn[2]       = (pgn >> 16);
  command.interval     = interval;
  command.offset       = 0;
  command.count        = cnt;

  bytes = b - (uint8_t *) &command;
  printf("%s,2,126208,0,%lu,%zu", now(dateStr), dest, bytes);
  for (i = 0; i < bytes; i++)
  {
    printf(",%02x", ((unsigned char *) &command)[i]);
  }
  printf("\n");
  exit(0);
}
