/*
Read and write to a Digital Yacht iKonvert over its serial device.
This can be a serial version connected to an actual serial port
or an USB version connected to the virtual serial port.

(C) 2009-2018, Kees Verruijt, Harlingen, The Netherlands.

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

#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "ikonvert.h"

#include "license.h"

#define BUFFER_SIZE 900

static int  verbose        = 0;
static int  readonly       = 0;
static int  writeonly      = 0;
static int  passthru       = 0;
static long timeout        = 0;
static int  outputCommands = 0;
static bool isFile;

int baudRate = B230400;

// Yeah globals. We're trying to avoid malloc()/free() to run quickly on limited memory hardware
StringBuffer writeBuffer; // What we still have to write to device
StringBuffer readBuffer;  // What we have already read from device
StringBuffer inBuffer;    // What we have already read from stdin but is not complete yet
StringBuffer dataBuffer;  // Temporary buffer during parse or generate

uint64_t lastNow; // Epoch time of last timestamp
uint64_t lastTS;  // Last timestamp received from iKonvert. Beware roll-around, max value is 999999

static void processInBuffer(StringBuffer *in, StringBuffer *out);
static void processReadBuffer(StringBuffer *in, int out);

int main(int argc, char **argv)
{
  int            r;
  int            handle;
  struct termios attr;
  char *         name   = argv[0];
  char *         device = 0;
  struct stat    statbuf;
  int            pid = 0;
  int            speed;

  setProgName(argv[0]);
  while (argc > 1)
  {
    if (strcasecmp(argv[1], "-version") == 0)
    {
      printf("%s\n", VERSION);
      exit(0);
    }
    else if (strcasecmp(argv[1], "-w") == 0)
    {
      writeonly = 1;
    }
    else if (strcasecmp(argv[1], "-p") == 0)
    {
      passthru = 1;
    }
    else if (strcasecmp(argv[1], "-r") == 0)
    {
      readonly = 1;
    }
    else if (strcasecmp(argv[1], "-v") == 0)
    {
      verbose = 1;
    }
    else if (strcasecmp(argv[1], "-t") == 0 && argc > 2)
    {
      argc--;
      argv++;
      timeout = strtol(argv[1], 0, 10);
      logDebug("timeout set to %ld seconds\n", timeout);
    }
    else if (strcasecmp(argv[1], "-s") == 0 && argc > 2)
    {
      argc--;
      argv++;
      speed = strtol(argv[1], 0, 10);
      switch (speed)
      {
        case 38400:
          baudRate = B38400;
          break;
        case 57600:
          baudRate = B57600;
          break;
        case 115200:
          baudRate = B115200;
          break;
        case 230400:
          baudRate = B230400;
          break;
        default:
          device = 0;
          break;
      }
      logDebug("speed set to %d (%d) baud\n", speed, baudRate);
    }
    else if (strcasecmp(argv[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
    }
    else if (strcasecmp(argv[1], "-o") == 0)
    {
      outputCommands = 1;
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
            "Usage: %s [-w] -[-p] [-r] [-v] [-d] [-s <n>] [-t <n>] device\n"
            "\n"
            "Options:\n"
            "  -w      writeonly mode, no data is read from device\n"
            "  -r      readonly mode, no data is sent to device\n"
            "  -p      passthru mode, data on stdin is sent to stdout but not to device\n"
            "  -v      verbose\n"
            "  -d      debug\n"
            "  -s <n>  set baudrate to 38400, 57600, 115200 or 230400\n"
            "  -t <n>  timeout, if no message is received after <n> seconds the program quits\n"
            "  -o      output commands sent to stdin to the stdout \n"
            "  <device> can be a serial device, a normal file containing a raw log,\n"
            "  or the address of a TCP server in the format tcp://<host>[:<port>]\n"
            "\n"
            "  Examples: %s /dev/ttyUSB0\n"
            "            %s tcp://192.168.1.1:10001\n"
            "\n" COPYRIGHT,
            name,
            name,
            name);
    exit(1);
  }

retry:
  logDebug("Opening %s\n", device);
  if (strncmp(device, "tcp:", STRSIZE("tcp:")) == 0)
  {
    handle = open_socket_stream(device);
    logDebug("socket = %d\n", handle);
    isFile = true;
    if (handle < 0)
    {
      logAbort("Cannot open TCP stream %s\n", device);
    }
  }
  else
  {
    handle = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    logDebug("fd = %d\n", handle);
    if (handle < 0)
    {
      logAbort("Cannot open device %s\n", device);
    }
    if (fstat(handle, &statbuf) < 0)
    {
      logAbort("Cannot determine device %s\n", device);
    }
    isFile = S_ISREG(statbuf.st_mode);
  }

  if (isFile)
  {
    logInfo("Device is a normal file, do not set the attributes.\n");
  }
  else
  {
    logDebug("Device is a serial port, set the attributes.\n");

    memset(&attr, 0, sizeof(attr));
    cfsetspeed(&attr, baudRate);
    attr.c_cflag |= CS8 | CLOCAL | CREAD;

    attr.c_iflag |= IGNPAR;
    attr.c_cc[VMIN]  = 1;
    attr.c_cc[VTIME] = 0;
    tcflush(handle, TCIFLUSH);
    tcsetattr(handle, TCSANOW, &attr);

    logDebug("Device is a serial port, send the startup sequence.\n");

    // TODO rx/tx list set
    sbAppendFormat(&writeBuffer, "%s\r\n", TX_ONLINE_MSG);
  }

  for (;;)
  {
    uint8_t data[128];
    size_t  len;
    ssize_t r;

    int rd = isReady(handle, STDIN, STDOUT, timeout);

    if ((rd & FD1_ReadReady) > 0)
    {
      r = read(handle, data, sizeof data);
      if (r > 0)
      {
        sbAppendData(&readBuffer, data, r);
      }
      if (r < 0)
      {
        logAbort("Error reading device: %s\n", strerror(errno));
      }
      if (r == 0)
      {
        logAbort("EOF on device");
      }
    }

    if ((rd & FD2_ReadReady) > 0)
    {
      r = read(STDIN, data, sizeof data);
      if (r > 0 && !readonly) // if readonly we just ignore data coming from stdin
      {
        sbAppendData(&inBuffer, data, r);
        processInBuffer(&inBuffer, &writeBuffer);
      }
      if (r < 0)
      {
        logAbort("Error reading stdin: %s\n", strerror(errno));
      }
      if (r == 0)
      {
        logAbort("EOF on stdin");
      }
    }

    if ((rd & FD1_WriteReady) > 0 && sbGetLength(&writeBuffer) > 0)
    {
      r = write(handle, sbGet(&writeBuffer), sbGetLength(&writeBuffer));
      if (r > 0)
      {
        sbDelete(&writeBuffer, 0, (size_t) r); // Eliminate written bytes from buffer
      }
      if (r == 0)
      {
        logAbort("EOF on stdout");
      }
    }

    if ((rd & FD3_WriteReady) > 0 && sbGetLength(&readBuffer) > 0)
    {
      processReadBuffer(&readBuffer, STDOUT);
    }
  }

  close(handle);
  return 0;
}

/* Received data from stdin. Once it is a full command parse it as FORMAT_FAST then convert to
 * format desired by device.
 */
static void processInBuffer(StringBuffer *in, StringBuffer *out)
{
  RawMessage msg;
  char *     p = strchr(sbGet(in), '\n');

  if (!p)
  {
    return;
  }

  if (parseFastFormat(in, &msg))
  {
    // Format msg as iKonvert message
    sbAppendFormat(out, TX_PGN_MSG_PREFIX, msg.pgn, msg.dst);
    sbAppendEncodeBase64(out, msg.data, msg.len);
  }
  sbDelete(in, 0, p - sbGet(in));
}

static void computeIKonvertTime(RawMessage *msg, unsigned int t1, unsigned int t2)
{
  uint64_t ts = t1 * 1000 + t2;

  if (ts < lastTS) // Ooops, roll-around. Reset!
  {
    lastNow = 0;
  }
  if (lastNow == 0)
  {
    lastNow = getNow();
    lastTS  = ts;
  }
  // Compute the difference between lastTS and ts
  lastNow += ts - lastTS;
  storeTimestamp(msg->timestamp, lastNow);
}

static bool parseIKonvertFormat(StringBuffer *in, RawMessage *msg)
{
  char *       end = sbGet(in) + strlen(sbGet(in)); // not sbGetLength as 'in' has been truncated
  char *       p   = sbGet(in);
  int          r;
  unsigned int pgn;
  unsigned int prio;
  unsigned int src;
  unsigned int dst;
  unsigned int t1;
  unsigned int t2;
  int          i;

  r = sscanf(p, RX_PGN_MSG_PREFIX "%n", &pgn, &prio, &src, &dst, &t1, &t2, &i);
  if (r != 6)
  {
    logError("parseIKonvertFormat: %s not all fields found, r=%d\n", RX_PGN_MSG_PREFIX, r);
    return false;
  }

  msg->prio = prio;
  msg->pgn  = pgn;
  msg->src  = src;
  msg->dst  = dst;

  p += i;
  sbAppendDecodeBase64(&dataBuffer, p, end - p);
  msg->len = CB_MIN(sbGetLength(&dataBuffer), FASTPACKET_MAX_SIZE);
  memcpy(msg->data, sbGet(&dataBuffer), msg->len);
  sbEmpty(&dataBuffer);
  computeIKonvertTime(msg, t1, t2);
  return true;
}

static void parseIKonvertAsciiMessage(const char *msg)
{
  int error;
  int pgn;

  if (parseConst(&msg, "ACK,"))
  {
    logInfo("iKonvert acknowledge of %s\n", msg);
    return;
  }

  if (parseConst(&msg, "NAK,"))
  {
    if (parseInt(&msg, &error, -1))
    {
      logInfo("iKonvert NAK %d: %s\n", error, msg);
    }
    return;
  }
  if (parseInt(&msg, &pgn, -1) && pgn == 0)
  {
    int load, errors, count, uptime, addr, rejected;
    // Network status message
    if (parseInt(&msg, &load, 0) && load != 0)
    {
      logInfo("CAN Bus load %d%%\n", load);
    }
    if (parseInt(&msg, &errors, 0) && errors != 0)
    {
      logInfo("CAN Bus errors %d\n", errors);
    }
    if (parseInt(&msg, &count, 0) && count != 0)
    {
      logInfo("CAN device count %d\n", count);
    }
    if (parseInt(&msg, &uptime, 0) && uptime != 0)
    {
      logInfo("iKonvert uptime %ds\n", uptime);
    }
    if (parseInt(&msg, &addr, 0) && addr != 0)
    {
      logInfo("iKonvert address %d\n", addr);
    }
    if (parseInt(&msg, &rejected, 0) && rejected != 0)
    {
      logInfo("iKonvert rejected %d TX message requests\n", rejected);
    }
    return;
  }
  logError("Unknown iKonvert message: %s\n", msg);
}

static void processReadBuffer(StringBuffer *in, int out)
{
  RawMessage  msg;
  char *      p;
  const char *w = sbGet(in);

  while ((p = strchr(w, '\n')) != 0)
  {
    memset(&msg, 0, sizeof msg);

    p[0] = 0;
    if (p > w + 1 && p[-1] == '\r')
    {
      p[-1] = 0;
    }

    if (parseConst(&w, IKONVERT_ASCII_PREFIX))
    {
      parseIKonvertAsciiMessage(w);
      w = sbGet(in);
    }
    else if (parseIKonvertFormat(in, &msg))
    {
      ssize_t r;

      // Format msg as FAST message
      sbAppendFormat(&dataBuffer, "%s,%u,%u,%u,%u,%u,", msg.timestamp, msg.prio, msg.pgn, msg.src, msg.dst, msg.len);
      sbAppendDecodeHex(&dataBuffer, msg.data, msg.len);
      sbAppendString(&dataBuffer, "\n");

      r = write(out, sbGet(&dataBuffer), sbGetLength(&dataBuffer));
      if (r <= 0)
      {
        logAbort("Cannot write to output\n");
      }

      sbEmpty(&dataBuffer);
    }
    else
    {
      logError("Ignoring invalid message '%s'\n", sbGet(in));
    }
    sbDelete(in, 0, p + 1 - w);
  }
}
