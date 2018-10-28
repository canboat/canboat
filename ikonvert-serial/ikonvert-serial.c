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

#define IKONVERT_BEM 0x40100

static bool verbose;
static bool readonly;
static bool writeonly;
static bool passthru;
static long timeout;
static bool isFile;

int baudRate = B230400;

// Yeah globals. We're trying to avoid malloc()/free() to run quickly on limited memory hardware
StringBuffer writeBuffer; // What we still have to write to device
StringBuffer readBuffer;  // What we have already read from device
StringBuffer inBuffer;    // What we have already read from stdin but is not complete yet
StringBuffer dataBuffer;  // Temporary buffer during parse or generate
StringBuffer txList;      // TX list to send to iKonvert
StringBuffer rxList;      // RX list to send to iKonvert

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
      writeonly = true;
    }
    else if (strcasecmp(argv[1], "-p") == 0)
    {
      passthru = true;
    }
    else if (strcasecmp(argv[1], "-r") == 0)
    {
      readonly = true;
    }
    else if (strcasecmp(argv[1], "-v") == 0)
    {
      verbose = true;
    }
    else if (strcasecmp(argv[1], "-rx") == 0 && argc > 2)
    {
      argc--;
      argv++;
      if (sbGetLength(&rxList) > 0)
      {
        sbAppendString(&rxList, ",");
      }
      sbAppendFormat(&rxList, "%s", argv[1]);
    }
    else if (strcasecmp(argv[1], "-tx") == 0 && argc > 2)
    {
      argc--;
      argv++;
      if (sbGetLength(&txList) > 0)
      {
        sbAppendString(&txList, ",");
      }
      sbAppendFormat(&txList, "%s", argv[1]);
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
            "  -w      writeonly mode, data from device is not sent to stdout\n"
            "  -r      readonly mode, data from stdin is not sent to device\n"
            "  -p      passthru mode, data from stdin is sent to stdout\n"
            "  -v      verbose\n"
            "  -d      debug\n"
            "  -rx <list> Set PGN receive list\n"
            "  -tx <list> Set PGN transmit list\n"
            "  -s <n>  set baudrate to 38400, 57600, 115200 or 230400\n"
            "  -t <n>  timeout, if no message is received after <n> seconds the program quits\n"
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

    sbAppendFormat(&writeBuffer, "%s,%s\r\n", TX_SET_RX_LIST_MSG, sbGet(&rxList));
    sbAppendFormat(&writeBuffer, "%s,%s\r\n", TX_SET_TX_LIST_MSG, sbGet(&txList));
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
      if (r > 0)
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

  if (!readonly && parseFastFormat(in, &msg))
  {
    // Format msg as iKonvert message
    sbAppendFormat(out, TX_PGN_MSG_PREFIX, msg.pgn, msg.dst);
    sbAppendEncodeBase64(out, msg.data, msg.len);
  }
  if (passthru)
  {
    ssize_t r = write(STDOUT, sbGet(in), p + 1 - sbGet(in));

    if (r <= 0)
    {
      logAbort("Cannot write to output\n");
    }
  }

  sbDelete(in, 0, p + 1 - sbGet(in));
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

static bool parseIKonvertAsciiMessage(const char *msg, RawMessage *n2k)
{
  int error;
  int pgn;

  if (parseConst(&msg, "ACK,"))
  {
    if (verbose)
    {
      logInfo("iKonvert acknowledge of %s\n", msg);
    }
    return false;
  }

  if (parseConst(&msg, "NAK,"))
  {
    if (parseInt(&msg, &error, -1) && verbose)
    {
      logInfo("iKonvert NAK %d: %s\n", error, msg);
    }
    return false;
  }
  if (parseInt(&msg, &pgn, -1) && pgn == 0)
  {
    n2k->pgn  = IKONVERT_BEM;
    n2k->prio = 7;
    n2k->src  = 0;
    n2k->dst  = 255;
    storeTimestamp(n2k->timestamp, getNow());

    int    load, errors, count, uptime, addr, rejected;
    size_t off;

    n2k->len = 15;
    memset(n2k->data, 0xff, n2k->len);

    if (parseInt(&msg, &load, 0xff))
    {
      n2k->data[0] = (uint8_t) load;
      if (verbose)
      {
        logInfo("CAN Bus load %d%%\n", load);
      }
    }
    if (parseInt(&msg, &errors, -1))
    {
      n2k->data[1] = (uint8_t)(errors >> 0);
      n2k->data[2] = (uint8_t)(errors >> 8);
      n2k->data[3] = (uint8_t)(errors >> 16);
      n2k->data[4] = (uint8_t)(errors >> 24);
      if (verbose)
      {
        logInfo("CAN Bus errors %d\n", errors);
      }
    }
    if (parseInt(&msg, &count, 0) && count != 0)
    {
      n2k->data[5] = (uint8_t) count;
      if (verbose)
      {
        logInfo("CAN device count %d\n", count);
      }
    }
    if (parseInt(&msg, &uptime, 0) && uptime != 0)
    {
      n2k->data[6] = (uint8_t)(uptime >> 0);
      n2k->data[7] = (uint8_t)(uptime >> 8);
      n2k->data[8] = (uint8_t)(uptime >> 16);
      n2k->data[9] = (uint8_t)(uptime >> 24);
      if (verbose)
      {
        logInfo("iKonvert uptime %ds\n", uptime);
      }
    }
    if (parseInt(&msg, &addr, 0) && addr != 0)
    {
      n2k->data[10] = (uint8_t) addr;
      if (verbose)
      {
        logInfo("iKonvert address %d\n", addr);
      }
    }
    if (parseInt(&msg, &rejected, 0) && rejected != 0)
    {
      n2k->data[11] = (uint8_t)(rejected >> 0);
      n2k->data[12] = (uint8_t)(rejected >> 8);
      n2k->data[13] = (uint8_t)(rejected >> 16);
      n2k->data[14] = (uint8_t)(rejected >> 24);
      if (verbose)
      {
        logInfo("iKonvert rejected %d TX message requests\n", rejected);
      }
    }
    return true;
  }
  logError("Unknown iKonvert message: %s\n", msg);
  return false;
}

static void processReadBuffer(StringBuffer *in, int out)
{
  RawMessage  msg;
  char *      p;
  bool        sendMessage;
  const char *w = sbGet(in);

  while ((p = strchr(w, '\n')) != 0)
  {
    memset(&msg, 0, sizeof msg);
    sendMessage = false;

    p[0] = 0;
    if (p > w + 1 && p[-1] == '\r')
    {
      p[-1] = 0;
    }

    if (writeonly)
    {
      // ignore message
    }
    else if (parseConst(&w, IKONVERT_ASCII_PREFIX))
    {
      if (parseIKonvertAsciiMessage(w, &msg))
      {
        sendMessage = true;
      }
    }
    else if ((parseConst(&w, IKONVERT_ASCII_PREFIX) && parseIKonvertAsciiMessage(w, &msg)) || parseIKonvertFormat(in, &msg))
    {
      sendMessage = true;
    }
    else
    {
      logError("Ignoring unknown or invalid message '%s'\n", sbGet(in));
    }

    if (sendMessage)
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
    sbDelete(in, 0, p + 1 - sbGet(in));
  }
}
