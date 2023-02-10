/*

Convert can-utils/candump output format to the analyzer's RAWFORMAT_PLAIN input format.

Many Linux distributions now implement SocketCAN support and further include
the can-utils for monitoring and exercising CAN bus interfaces.

Further info re: SocketCAN and the can-utils can be viewed here...

http://en.wikipedia.org/wiki/SocketCAN

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
#include <stdio.h>
#include <time.h>

#include "common.h"

#define MSG_BUF_SIZE 2000
#define CANDUMP_DATA_INC_3 3
#define CANDUMP_DATA_INC_2 2
#define MAX_DATA_BYTES 223

// There are at least three variations in candump output
// format which are currently handled...
//
#define FMT_TBD 0
#define FMT_1 1 // Angstrom ex:	"<0x18eeff01> [8] 05 a0 be 1c 00 a0 a0 c0"
#define FMT_2 2 // Debian ex:	"   can0  09F8027F   [8]  00 FC FF FF 00 00 FF FF"
#define FMT_3 3 // candump log ex:	"(1502979132.106111) slcan0 09F50374#000A00FFFF00FFFF"
#define FMT_4 4 // tshark of pcap:10131  29.555750              ?              CAN 16 XTD: 0x09fd0223   00 49 02 1c a7 fa ff ff

void gettimeval(struct timeval *tv, double sec)
{
  tv->tv_sec  = sec;
  tv->tv_usec = (sec - tv->tv_sec) * 1000000;
}

int main(int argc, char **argv)
{
  char  msg[MSG_BUF_SIZE];
  FILE *infile  = stdin;
  FILE *outfile = stdout;

  if (argc > 1)
  {
    if (strcasecmp(argv[1], "-version") == 0)
    {
      printf("%s\n", VERSION);
      exit(0);
    }
    infile = fopen(argv[1], "r");
    if (!infile)
    {
      fprintf(stderr, "Could not open input file '%s' (%s)\n", argv[1], strerror(errno));
      return 1;
    }
  }

  // For every line in the candump file...
  //
  int          format           = FMT_TBD;
  unsigned int candump_data_inc = CANDUMP_DATA_INC_3;
  while (fgets(msg, sizeof(msg) - 1, infile))
  {
    // Ignore empty and comment lines within the candump input.
    //
    if (*msg == 0 || *msg == '\n' || *msg == '#')
    {
      continue;
    }

    // Process the CAN ID
    //
    unsigned int canid       = 0;
    int          size        = 0;
    double       currentTime = 0.;

    // Determine which candump format is being used.
    //
    if (format == FMT_TBD)
    {
      // Format not yet detected.
      // See if we can match one.
      //
      if (sscanf(msg, "<%x> [%d] ", &canid, &size) == 2)
        format = FMT_1;
      else if (sscanf(msg, " %*s %x [%d] ", &canid, &size) == 2)
        format = FMT_2;
      else if (sscanf(msg, "(%lf) %*s %8x#", &currentTime, &canid) == 2)
      {
        format           = FMT_3;
        candump_data_inc = CANDUMP_DATA_INC_2;
        size             = (strlen(strchr(msg, '#')) - 1) / 2;
      }
      else if (strstr(msg, "CAN 16 XTD:") != NULL)
        format = FMT_4;
      else
        continue;
    }
    else if (format == FMT_1)
    {
      if (sscanf(msg, "<%x> [%d] ", &canid, &size) != 2)
        continue;
    }
    else if (format == FMT_2)
    {
      if (sscanf(msg, " %*s %x [%d] ", &canid, &size) != 2)
        continue;
    }
    else if (format == FMT_3)
    {
      if (sscanf(msg, "(%lf) %*s %8x#", &currentTime, &canid) != 2)
        continue;
      size = (strlen(strchr(msg, '#')) - 1) / 2;
    }
    else if (format == FMT_4)
    {
      if (sscanf(msg, "%*d %lf %*s CAN %d XTD: 0x%8x   ", &currentTime, &size, &canid) != 3)
        continue;
      size = size - 8;
    }

    unsigned int pri = 0;
    unsigned int src = 0;
    unsigned int dst = 255;
    unsigned int pgn = 0;

    getISO11783BitsFromCanId(canid, &pri, &pgn, &src, &dst);

    int            msec;
    char           timestamp[20];
    struct timeval tv;
    struct tm *    utc;

    // If the candump format includes a usec timestamp, convert
    // that to a timeval, otherwise use gettimeofday.
    //
    if (format >= FMT_3)
    {
      gettimeval(&tv, currentTime);
    }
    else
    {
      gettimeofday(&tv, NULL);
    }

    // strftime doesn't support fractional seconds, so use another
    // variable.
    //
    msec = lrint(tv.tv_usec / 1000.0);
    if (msec >= 1000)
    {
      msec -= 1000;
      tv.tv_sec++;
    }

    utc = gmtime(&tv.tv_sec);

    // %F = YYYY-MM-DD
    // %T = HH:MM:SS
    //
    strftime(timestamp, 20, "%F-%T", utc);

    // Output all but the data bytes.
    //
    fprintf(outfile, "%s.%03d,%d,%d,%d,%d,%d", timestamp, msec, pri, pgn, src, dst, size);

    // Now process the data bytes.
    //
    int          i;
    char *       p;
    char         separator;
    unsigned int data[MAX_DATA_BYTES];

    p = msg;
    if (format == FMT_4)
    {
      p         = strstr(p, "XTD: ") + sizeof("XTD: ");
      separator = ' ';
      for (; p < msg + sizeof(msg) && *p != 0 && *p != separator; ++p)
        ;
    }
    else
    {
      separator = (format == FMT_3) ? '#' : ']';
      for (p = msg; p < msg + sizeof(msg) && *p != 0 && *p != separator; ++p)
        ;
    }
    if (*p == separator)
    {
      if (format == FMT_3)
      {
        p++;
      }
      else
      {
        while (*(++p) == ' ')
          ;
      }
      for (i = 0; i < size; i++, p += candump_data_inc)
      {
        sscanf(p, "%2x", &data[i]);
        fprintf(outfile, ",%02x", data[i]);
      }
    }
    fprintf(outfile, "\n");
    fflush(outfile);
  }
}
