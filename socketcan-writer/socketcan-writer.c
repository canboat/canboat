/*

Reads raw n2k ASCII data from stdin and writes it to a Linux SocketCAN device (e.g. can0).

(C) 2009-2015, Kees Verruijt, Harlingen, The Netherlands.

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

#include <stdio.h>
#include <stdlib.h>

#define  GLOBALS
#include "pgn.h"

int main(int argc, char ** argv)
{
  FILE * file = stdin;
  char msg[2000];

  if(argc != 2)
  {
    puts("Usage: socketcan-writer <can-device>");
    exit(1);
  }

  while (fgets(msg, sizeof(msg) - 1, file))
  {
    RawMessage m;
    if(parseRawFormatFast(msg, &m, false))
    {
      continue;  // Parsing failed -> skip the line
    }
    printf("Read: %s, %d, %d, %d, %d, %d\n", m.timestamp, m.prio, m.pgn, m.src, m.dst, m.len);
  }

  exit(0);
}
