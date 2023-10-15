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

#include "common.h"

#define BUFFER_SIZE 900

static int  readonly = 0;
static bool isFile;

int main(int argc, char **argv)
{
  int            handle;
  struct termios attr;
  char          *device = 0;
  struct stat    statbuf;

  setProgName(argv[0]);

  while (argc > 1)
  {
    if (strcasecmp(argv[1], "-version") == 0)
    {
      printf("%s\n", VERSION);
      exit(0);
    }
    if (strcasecmp(argv[1], "-r") == 0)
    {
      readonly = 1;
    }
    else if (strcasecmp(argv[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
    }
    else if (strcasecmp(argv[1], "-?") == 0)
    {
      break;
    }
    else if (!device)
    {
      device = argv[1];
    }
    else
    {
      device = 0;
      break;
    }
    argc--;
    argv++;
  }

  if (!device)
  {
    fprintf(stderr,
            "Usage: nmea0183-serial [-r] [-d] <device> | -version\n\n"
            "-r : read-only, do not pass stdin to stdout\n"
            "-d : debug mode\n\n"
            "Example: nmea0183-serial /dev/ttyUSB0\n\n" COPYRIGHT);
    exit(1);
  }

  logDebug("Opening %s\n", device);
  handle = open(device, O_RDWR | O_NOCTTY);
  logDebug("fd = %d\n", handle);
  if (handle < 0)
  {
    logAbort("NMEA-00001: Cannot open NMEA-0183 device %s\n", device);
    exit(1);
  }
  if (fstat(handle, &statbuf) < 0)
  {
    logAbort("NMEA-00002: Cannot determine device %s\n", device);
    exit(1);
  }
  isFile = S_ISREG(statbuf.st_mode);

  if (!isFile)
  {
    logDebug("Device is a serial port, set the attributes.\n");

    memset(&attr, 0, sizeof(attr));
    attr.c_cflag     = B38400 | CS8 | CLOCAL | CREAD;
    attr.c_iflag     = IGNPAR;
    attr.c_oflag     = 0;
    attr.c_lflag     = 0;
    attr.c_cc[VMIN]  = 0;
    attr.c_cc[VTIME] = 1;
    tcflush(handle, TCIFLUSH);
    tcsetattr(handle, TCSANOW, &attr);
  }

  for (;;)
  {
    char                 msg[BUFFER_SIZE];
    enum ReadyDescriptor r;
    int                  b;

    r = isReady(handle, readonly ? INVALID_SOCKET : STDIN, INVALID_SOCKET, 0);

    if ((r & FD1_ReadReady) > 0)
    {
      b = read(handle, msg, sizeof(msg));
      if (b < 0)
      {
        break;
      }
      else if (b > 0)
      {
        if (write(STDOUT, msg, b) < b)
        {
          break;
        }
      }
    }
    if ((r & FD2_ReadReady) > 0)
    {
      b = read(0, msg, sizeof(msg));
      if (b < 0)
      {
        break;
      }
      else if (b > 0)
      {
        if (write(STDOUT, msg, b) < b)
        {
          break;
        }
        if (write(handle, msg, b) < b)
        {
          break;
        }
      }
    }
  }

  close(handle);
  return 0;
}
