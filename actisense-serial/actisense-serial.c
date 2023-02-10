/*
Read and write to an Actisense NGT-1 over its serial device.
This can be a serial version connected to an actual serial port
or an USB version connected to the virtual serial port.

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

#include "actisense.h"
#include "common.h"
#include "license.h"

/* The following startup command reverse engineered from Actisense NMEAreader.
 * It instructs the NGT1 to clear its PGN message TX list, thus it starts
 * sending all PGNs.
 */
static unsigned char NGT_STARTUP_SEQ[] = {
    0x11, /* msg byte 1, meaning ? */
    0x02, /* msg byte 2, meaning ? */
    0x00  /* msg byte 3, meaning ? */
};

#define BUFFER_SIZE 900

static int  verbose        = 0;
static int  readonly       = 0;
static int  writeonly      = 0;
static int  passthru       = 0;
static long timeout        = 0;
static int  outputCommands = 0;
static bool isFile;

enum MSG_State
{
  MSG_START,
  MSG_ESCAPE,
  MSG_MESSAGE
};

int baudRate = B115200;

static bool readIn(void);
static bool getInMsg(unsigned char *msg, size_t len);
static void parseAndWriteIn(int handle, const unsigned char *cmd);
static void writeMessage(int handle, unsigned char command, const unsigned char *cmd, const size_t len);
static bool readNGT1Byte(unsigned char c);
static int  readNGT1(int handle);
static void messageReceived(const unsigned char *msg, size_t msgLen);
static void n2kMessageReceived(const unsigned char *msg, size_t msgLen);
static void ngtMessageReceived(const unsigned char *msg, size_t msgLen);

int main(int argc, char **argv)
{
  int            handle;
  struct termios attr;
  char *         name   = argv[0];
  char *         device = 0;
  struct stat    statbuf;
  int            speed = 115200;
  int            i;
  int            wait;
  time_t         lastPing = time(0);

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
#ifdef B460800
        case 460800:
          baudRate = B460800;
          break;
#endif
#ifdef B921600
        case 921600:
          baudRate = B921600;
          break;
#endif
        default:
          baudRate = speed;
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
            "  -s <n>  set baudrate to 38400, 57600, 115200, 230400"
#ifdef B460800
            ", 460800"
#endif
#ifdef B921600
            ", 921600"
#endif
            "\n"
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

  logDebug("Opening %s\n", device);
  if (strncmp(device, "tcp:", STRSIZE("tcp:")) == 0)
  {
    handle = open_socket_stream(device);
    logDebug("socket = %d\n", handle);
    isFile = true;
    if (handle < 0)
    {
      fprintf(stderr, "Cannot open NGT-1-A TCP stream %s\n", device);
      exit(1);
    }
  }
  else
  {
    handle = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    logDebug("fd = %d\n", handle);
    if (handle < 0)
    {
      logAbort("Cannot open NGT-1-A device %s\n", device);
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
    if (cfsetspeed(&attr, baudRate) < 0)
    {
      logAbort("Could not set baudrate %d\n", speed);
    }
    attr.c_cflag |= CS8 | CLOCAL | CREAD;

    attr.c_iflag |= IGNPAR;
    attr.c_cc[VMIN]  = 1;
    attr.c_cc[VTIME] = 0;
    tcflush(handle, TCIFLUSH);
    tcsetattr(handle, TCSANOW, &attr);

    logDebug("Device is a serial port, send the startup sequence.\n");

    writeMessage(handle, NGT_MSG_SEND, NGT_STARTUP_SEQ, sizeof(NGT_STARTUP_SEQ));
    sleep(2);
  }

  // Do not read anything until we have seen 10 messages on bus
  for (i = 0; i < 10;)
  {
    int r = isReady(handle, INVALID_SOCKET, INVALID_SOCKET, timeout);

    if ((r & FD1_ReadReady) > 0)
    {
      if (readNGT1(handle) <= 0)
      {
        break;
      }
      i++;
    }
  }

  for (wait = timeout;;)
  {
    unsigned char msg[BUFFER_SIZE];
    int           r = isReady(writeonly ? INVALID_SOCKET : handle, readonly ? INVALID_SOCKET : STDIN_FILENO, INVALID_SOCKET, timeout);

    if ((r & FD1_ReadReady) > 0)
    {
      if (readNGT1(handle) <= 0)
      {
        break;
      }
    }
    if ((r & FD2_ReadReady) > 0)
    {
      if (!readIn())
      {
        break;
      }
    }

    if (getInMsg(msg, sizeof(msg)))
    {
      if (!passthru)
      {
        parseAndWriteIn(handle, msg);
      }
      if (outputCommands)
      {
        fprintf(stdout, "%s", msg);
        fflush(stdout);
      }
      wait = 0;
    }
    else
    {
      wait = timeout;
      if (writeonly)
      {
        break;
      }
      if (time(0) - lastPing > 20)
      {
        writeMessage(handle, NGT_MSG_SEND, NGT_STARTUP_SEQ, sizeof(NGT_STARTUP_SEQ));
        lastPing = time(0);
      }
    }
  }

  close(handle);
  return 0;
}

static void parseAndWriteIn(int handle, const unsigned char *cmd)
{
  unsigned char  msg[500];
  unsigned char *m;

  unsigned int prio;
  unsigned int pgn;
  unsigned int src;
  unsigned int dst;
  unsigned int bytes;

  char *       p;
  int          i;
  int          b;
  unsigned int byt;
  int          r;

  if (!cmd || !*cmd || *cmd == '\n')
  {
    return;
  }

  // Skip the time field, content is not used
  p = strchr((char *) cmd, ',');
  if (!p)
  {
    return;
  }

  r = sscanf(p, ",%u,%u,%u,%u,%u,%n", &prio, &pgn, &src, &dst, &bytes, &i);
  logDebug("parseAndWriteIn %.20s = %d\n", p, r);
  if (r == 5)
  {
    if (pgn >= ACTISENSE_BEM)
    { // Ignore synthetic CANboat PGNs that report original device status.
      return;
    }

    p += i - 1;
    m    = msg;
    *m++ = (unsigned char) prio;
    *m++ = (unsigned char) pgn;
    *m++ = (unsigned char) (pgn >> 8);
    *m++ = (unsigned char) (pgn >> 16);
    *m++ = (unsigned char) dst;
    //*m++ = (unsigned char) 0;
    *m++ = (unsigned char) bytes;
    for (b = 0; m < msg + sizeof(msg) && b < bytes; b++)
    {
      if ((sscanf(p, ",%x%n", &byt, &i) == 1) && (byt < 256))
      {
        *m++ = byt;
      }
      else
      {
        logError("Unable to parse incoming message '%s' at offset %u\n", cmd, b);
        return;
      }
      p += i;
    }
  }
  else
  {
    logError("Unable to parse incoming message '%s', r = %d\n", cmd, r);
    return;
  }

  logDebug("About to write:  %s\n", cmd);
  writeMessage(handle, N2K_MSG_SEND, msg, m - msg);
}

/*
 * Wrap the PGN or NGT message and send to NGT
 */
static void writeMessage(int handle, unsigned char command, const unsigned char *cmd, const size_t len)
{
  unsigned char  bst[255];
  unsigned char *b = bst;
  unsigned char *r = bst;
  unsigned char  crc;

  int i;

  *b++ = DLE;
  *b++ = STX;
  *b++ = command;
  crc  = command;
  *b++ = len;
  if (len == DLE)
  {
    *b++ = DLE;
  }

  for (i = 0; i < len; i++)
  {
    if (cmd[i] == DLE)
    {
      *b++ = DLE;
    }
    *b++ = cmd[i];
    crc += (unsigned char) cmd[i];
  }

  crc += i;

  crc = 256 - (int) crc;
  if (crc == DLE)
  {
    *b++ = DLE;
  }
  *b++ = crc;
  *b++ = DLE;
  *b++ = ETX;

  int retryCount    = 5;
  int needs_written = b - bst;
  int written;
  do
  {
    written = write(handle, r, needs_written);
    if (written != -1)
    {
      r += written;
      needs_written -= written;
    }
    else if (errno == EAGAIN)
    {
      retryCount--;
      usleep(25000);
    }
    else
    {
      break;
    }

  } while (needs_written > 0 && retryCount >= 0);

  if (written == -1)
  {
    logError("Unable to write command '%.*s' to NGT-1-A device\n", (int) len, cmd);
  }

  logDebug("Written command %X len %d\n", command, (int) len);
}

static StringBuffer inBuffer;

/**
 * Read from stdin, until we have a complete message and store it in msg.
 *
 * This is called when select() has seen that stdin is ready.
 * We can only do a single read, which we do into a separate buffer.
 *
 * Return false on error
 */
static bool readIn(void)
{
  unsigned char buf[BUFFER_SIZE];
  ssize_t       r;

  r = read(STDIN_FILENO, buf, sizeof(buf));

  if (r <= 0)
  {
    logAbort("EOF on reading stdin\n");
    return false;
  }

  sbAppendData(&inBuffer, buf, r);
  return true;
}

/**
 * After readIn returns data, you can retrieve messages from the
 * internal buffer until getInMsg() returns false.
 */
static bool getInMsg(unsigned char *msg, size_t msgLen)
{
  char * p;
  size_t len;

  p = strchr(sbGet(&inBuffer), '\n');
  if (!p)
  {
    return false;
  }
  len = p - sbGet(&inBuffer) + 1;
  memcpy(msg, sbGet(&inBuffer), len);
  msg[len] = 0;
  sbDelete(&inBuffer, 0, len);

  return true;
}

/**
 * Handle a byte coming in from the NGT1.
 *
 */

static bool readNGT1Byte(unsigned char c)
{
  static enum MSG_State state    = MSG_START;
  static bool           noEscape = false;
  static unsigned char  buf[500];
  static unsigned char *head = buf;

  if (state == MSG_START)
  {
    if ((c == ESC) && isFile)
    {
      noEscape = true;
    }
  }

  if (state == MSG_ESCAPE)
  {
    if (c == ETX)
    {
      messageReceived(buf, head - buf);
      head  = buf;
      state = MSG_START;
    }
    else if (c == STX)
    {
      head  = buf;
      state = MSG_MESSAGE;
    }
    else if ((c == DLE) || ((c == ESC) && isFile) || noEscape)
    {
      *head++ = c;
      state   = MSG_MESSAGE;
    }
    else
    {
      logError("DLE followed by unexpected char %02X, ignore message\n", c);
      state = MSG_START;
    }
  }
  else if (state == MSG_MESSAGE)
  {
    if (c == DLE)
    {
      state = MSG_ESCAPE;
    }
    else if (isFile && (c == ESC) && !noEscape)
    {
      state = MSG_ESCAPE;
    }
    else
    {
      *head++ = c;
    }
  }
  else
  {
    if (c == DLE)
    {
      state = MSG_ESCAPE;
    }
  }

  return state == MSG_START;
}

static int readNGT1(int handle)
{
  size_t        i;
  ssize_t       r;
  unsigned char c;
  unsigned char buf[500];
  bool          finish;

  do
  {
    r = read(handle, buf, sizeof(buf));
    logDebug("NGT read = %d\n", (int) r);

    if (r < 0 && errno == EAGAIN)
    {
      usleep(25000);
      continue;
    }
    if (r <= 0) /* No char read, abort message read */
    {
      logAbort("Unable to read from NGT1 device, errno=%d\n", errno);
    }

    if (isLogLevelEnabled(LOGLEVEL_DEBUG))
    {
      StringBuffer sb = sbNew;

      sbAppendEncodeHex(&sb, buf, r, ' ');
      logDebug("NGT data: %s\n", sbGet(&sb));
      sbClean(&sb);
    }

    for (i = 0; i < r; i++)
    {
      c      = buf[i];
      finish = readNGT1Byte(c);
    }
  } while (!finish);

  return r;
}

static void messageReceived(const unsigned char *msg, size_t msgLen)
{
  unsigned char command;
  unsigned char checksum = 0;
  unsigned char payloadLen;
  size_t        i;

  if (msgLen < 3)
  {
    logError("Ignore short command len = %zu\n", msgLen);
    return;
  }

  for (i = 0; i < msgLen; i++)
  {
    checksum += msg[i];
  }
  if (checksum)
  {
    logError("Ignoring message with invalid checksum\n");
    return;
  }

  command    = msg[0];
  payloadLen = msg[1];

  logDebug("message command = %02x len = %u\n", command, payloadLen);

  if (command == N2K_MSG_RECEIVED)
  {
    n2kMessageReceived(msg + 2, payloadLen);
  }
  else if (command == NGT_MSG_RECEIVED)
  {
    ngtMessageReceived(msg + 2, payloadLen);
  }
}

static void ngtMessageReceived(const unsigned char *msg, size_t msgLen)
{
  size_t i;
  char   line[1000];
  char * p;
  char   dateStr[DATE_LENGTH];

  if (msgLen < 12)
  {
    logError("Ignore short msg len = %zu\n", msgLen);
    return;
  }

  sprintf(line, "%s,%u,%u,%u,%u,%u", now(dateStr), 0, ACTISENSE_BEM + msg[0], 0, 0, (unsigned int) msgLen - 1);
  p = line + strlen(line);
  for (i = 1; i < msgLen && p < line + sizeof(line) - 5; i++)
  {
    sprintf(p, ",%02x", msg[i]);
    p += 3;
  }
  *p++ = 0;

  puts(line);
  fflush(stdout);
}

static void n2kMessageReceived(const unsigned char *msg, size_t msgLen)
{
  unsigned int prio, src, dst;
  unsigned int pgn;
  size_t       i;
  unsigned int len;
  char         line[800];
  char *       p;
  char         dateStr[DATE_LENGTH];

  if (msgLen < 11)
  {
    logError("Ignoring N2K message - too short\n");
    return;
  }
  prio = msg[0];
  pgn  = (unsigned int) msg[1] + 256 * ((unsigned int) msg[2] + 256 * (unsigned int) msg[3]);
  dst  = msg[4];
  src  = msg[5];
  /* Skip the timestamp logged by the NGT-1-A in bytes 6-9 */
  len = msg[10];

  if (len > 223)
  {
    logError("Ignoring N2K message - too long (%u)\n", len);
    return;
  }

  p = line;

  snprintf(p, sizeof(line), "%s,%u,%u,%u,%u,%u", now(dateStr), prio, pgn, src, dst, len);
  p += strlen(line);

  len += 11;
  for (i = 11; i < len; i++)
  {
    snprintf(p, line + sizeof(line) - p, ",%02x", msg[i]);
    p += strlen(p);
  }

  puts(line);
  fflush(stdout);
}
