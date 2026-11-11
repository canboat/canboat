/*

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

/*
 * Encode one <field>=<value> argument at b: the decimal field number as a byte, then
 * the hexadecimal value little-endian, one byte per two digits. Returns the byte after
 * it; calls usage() when the argument is not of that form, as a sign, a space, an odd
 * number of digits or more than eight of them would be silently mis-encoded.
 */
static uint8_t *encodeField(char **argv, char **av, uint8_t *b)
{
  const char *p = strchr(av[0], '=');
  char        field[8];
  size_t      n;
  size_t      digits;
  uint32_t    v;

  n = p ? (size_t) (p - av[0]) : 0;
  if (n == 0 || n >= sizeof(field))
  {
    usage(argv, av);
    return b;
  }
  memcpy(field, av[0], n);
  field[n] = '\0';
  p++;
  digits = strspn(p, "0123456789abcdefABCDEF");
  if (digits == 0 || digits % 2 != 0 || digits > 8 || p[digits] != '\0')
  {
    usage(argv, av);
    return b;
  }
  *b++ = (uint8_t) parseNumberOption("<field>", field, 1, 255);
  v    = (uint32_t) strtoul(p, NULL, 16);
  for (n = digits / 2; n; n--)
  {
    *b++ = (v & 255);
    v    = v >> 8;
  }
  return b;
}

/*
 * Print, in canboat PLAIN format, a PGN 126208 Request that asks device <dest> to send
 * <pgn> every <interval> ms, from the fields and values on the command line.
 */
int main(int argc, char **argv)
{
  int                      ac = argc;
  char                   **av = argv;
  long                     dest;
  long                     pgn;
  uint32_t                 interval;
  size_t                   cnt = 0;
  command_group_function_t command;
  size_t                   i, bytes;
  uint8_t                 *b;
  char                     dateStr[DATE_LENGTH];

  setProgName(argv[0]);

  if (ac < 6 || ac > 5 + MAX_FIELDS)
  {
    usage(argv, 0);
  }

  ac--, av++; /* lose the command name */
  dest = (long) parseNumberOption("<dest>", av[0], 0, 255);
  ac--, av++; /* lose the dest */
  /* ignore */ parseNumberOption("<prio>", av[0], 0, 8);
  ac--, av++; /* lose the prio */
  pgn = (long) parseNumberOption("<pgn>", av[0], 0, 0x1ffff);
  ac--, av++; /* lose the pgn */
  interval = (uint32_t) parseNumberOption("<interval>", av[0], 0, UINT32_MAX);
  ac--, av++; /* lose the interval */
  b = &command.parameters[0];

  for (; ac; ac--, av++)
  {
    b = encodeField(argv, av, b);
    cnt++;
  }

  command.functionCode = 0;
  command.pgn[0]       = (pgn) & 0xff;
  command.pgn[1]       = (pgn >> 8) & 0xff;
  command.pgn[2]       = (pgn >> 16);
  command.interval     = interval;
  command.offset       = 0;
  command.count        = cnt;

  bytes = b - (uint8_t *) &command;
  printf("%s,2,126208,0,%lu,%zu", fmtNow(dateStr), dest, bytes);
  for (i = 0; i < bytes; i++)
  {
    printf(",%02x", ((unsigned char *) &command)[i]);
  }
  printf("\n");
  exit(0);
}
