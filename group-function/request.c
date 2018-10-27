/*
 * Part of 'packetlogger', a CAN bus analyzer that decodes N2K messages.
 *
 * (C) 2009-2011, Keversoft B.V., Harlingen, the Netherlands
 *
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

typedef unsigned char uint8_t;
typedef unsigned int  uint32_t;

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
  fprintf(stderr, "Usage: %s <dest> <prio> <pgn> <field>=<value> ...\n\n", argv[0]);
  fprintf(stderr, "       <field> is a decimal value\n");
  fprintf(stderr, "       <value> is a hexadecimal value; the length of the value defines how many bytes are encoded\n");
  fprintf(stderr, "       Maximum # of fields: %d\n\n", MAX_FIELDS);
  fprintf(stderr, "This program uses PGN 126208 to request a device to report a PGN for certain values.\n");
  fprintf(stderr, "The use of this is thus completely dependent on what the device allows.\n\n" COPYRIGHT);
  exit(1);
}

int main(int argc, char **argv)
{
  int                      ac = argc;
  char **                  av = argv;
  long                     dest;
  long                     pgn;
  long                     prio;
  long                     fields[MAX_FIELDS];
  long                     values[MAX_FIELDS];
  size_t                   cnt = 0;
  command_group_function_t command;
  size_t                   i, bytes;
  char *                   p, *e;
  uint8_t *                b;
  uint32_t                 v;
  char                     dateStr[DATE_LENGTH];

  if (ac < 5 || ac > 4 + MAX_FIELDS)
  {
    usage(argv, 0);
  }

  ac--, av++; /* lose the command name */
  dest = strtol(av[0], 0, 10);
  ac--, av++; /* lose the dest */
  prio = strtol(av[0], 0, 10);
  ac--, av++; /* lose the prio */
  pgn = strtol(av[0], 0, 10);
  ac--, av++; /* lose the pgn */
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
