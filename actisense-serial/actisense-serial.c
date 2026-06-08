/*
Read and write to an Actisense NGT-1 over its serial device.
This can be a serial version connected to an actual serial port
or an USB version connected to the virtual serial port.

(C) 2009-2025, Kees Verruijt, Harlingen, The Netherlands.

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
#include "parse.h"

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

static int      verbose        = 0;
static int      readonly       = 0;
static int      writeonly      = 0;
static int      passthru       = 0;
static long     timeout        = 0;
static int      outputCommands = 0;
static bool     isFile;
static bool     isEBL;
static uint64_t timestamp = 0;

enum MSG_State
{
  MSG_START,
  MSG_ESCAPE,
  MSG_HEADER,
  MSG_MESSAGE
};

int baudRate = B115200;

static bool readIn(void);
static bool getInMsg(char *msg, size_t len);
static void parseAndWriteIn(int handle, const char *cmd);
static void writeMessage(int handle, unsigned char command, const unsigned char *cmd, const size_t len, uint64_t when);
static bool readNGT1Byte(unsigned char c);
static int  readNGT1(int handle);
static void headerReceived(const unsigned char *msg, size_t msgLen);
static void messageReceived(const unsigned char *msg, size_t msgLen);
static void n2kMessageReceived(const unsigned char *msg, size_t msgLen, unsigned char command);
static void ngtMessageReceived(const unsigned char *msg, size_t msgLen);

int main(int argc, char **argv)
{
  int            handle;
  struct termios attr;
  char          *name   = argv[0];
  char          *device = 0;
  struct stat    statbuf;
  int            speed = 115200;
  int            i;
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
            "  -p      passthru mode, data on stdin is also sent to stdout (in addition to the device)\n"
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
            "  -o      alias for -p (kept for backward compatibility; -p is preferred)\n"
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

  fputs(CANBOAT_FORMAT_FAST_HEADER, stdout);
  emitCanboatStartupRecord("actisense-serial", device);

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
    int  oflag  = O_NOCTTY | O_NONBLOCK;
    bool exists = (stat(device, &statbuf) == 0);
    isFile      = exists && S_ISREG(statbuf.st_mode);

    if (isFile || (!exists && writeonly))
    {
      // Regular file (replay) or non-existent path opened for
      // capture (-w on a new file). The buffer-stall problem
      // below doesn't apply to regular files, so keep the
      // mode-specific flags here.
      if (writeonly)
      {
        oflag |= O_WRONLY | O_CREAT;
        isFile = true;
      }
      else if (readonly)
      {
        oflag |= O_RDONLY;
      }
      else
      {
        oflag |= O_RDWR;
      }
    }
    else
    {
      // Serial / character device. Always R/W:
      //  -r mode still needs to write NGT_STARTUP_SEQ and the
      //   20s ping so the NGT-1 stays in "emit all PGNs". Under
      //   the old O_RDONLY those writes failed silently and we
      //   only got away with it because the TX-list config is
      //   persistent across power cycles.
      //  -w mode without reading lets the device's output pile
      //   up in the FTDI/USB RX buffer until bytes start
      //   getting dropped (CLOCAL = no flow-control
      //   backpressure). Open R/W and drain in the loop below.
      oflag |= O_RDWR;
    }
    handle = open(device, oflag, 0777);

    logDebug("fd = %d\n", handle);
    if (handle < 0)
    {
      logAbort("Cannot open NGT-1-A device/file %s\n", device);
    }
  }

  if (isFile)
  {
    if (strncmp(device + strlen(device) - STRSIZE(".ebl"), ".ebl", STRSIZE(".ebl")) == 0)
    {
      isEBL = true;
      logDebug("EBL mode selected\n");
    }
    else
    {
      logDebug("Device is a normal file, do not set the attributes.\n");
    }
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

    writeMessage(handle, NGT_MSG_SEND, NGT_STARTUP_SEQ, sizeof(NGT_STARTUP_SEQ), UINT64_C(0));
    sleep(2);
  }

  if (readonly)
  {
    // Defensive: pin fd 0 to /dev/null. The main loop's isReady
    // already skips STDIN_FILENO in readonly mode, but this
    // guards against any future code path that calls read(0,..)
    // and against the kernel handing fd 0 to a subsequent
    // open() if stdin were closed outright.
    int devnull = open("/dev/null", O_RDONLY);
    if (devnull >= 0)
    {
      dup2(devnull, STDIN_FILENO);
      close(devnull);
    }
  }

  if (!isFile && !writeonly)
  {
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
  }

  for (;;)
  {
    char msg[BUFFER_SIZE];
    // File-capture mode (-w to a regular file) has an O_WRONLY
    // handle — don't poll it for read. Serial -w opens R/W and
    // does need polling so we can drain the device's output.
    int  r = isReady((writeonly && isFile) ? INVALID_SOCKET : handle,
                     readonly ? INVALID_SOCKET : STDIN_FILENO,
                     INVALID_SOCKET,
                     timeout);

    if ((r & FD1_ReadReady) > 0)
    {
      if (writeonly)
      {
        // Drain & discard. We must read the device or its
        // output backs up in the kernel buffer; we deliberately
        // don't pass it through the NGT-1 framing parser (no
        // stdout output in -w).
        char    drain[BUFFER_SIZE];
        ssize_t n = read(handle, drain, sizeof drain);
        if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
        {
          break;
        }
      }
      else if (readNGT1(handle) <= 0)
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

    while (getInMsg(msg, sizeof(msg)))
    {
      parseAndWriteIn(handle, msg);
      if (passthru || outputCommands)
      {
        // -p / -o: also echo the command line to stdout, in
        // addition to encoding it for the device. Matches the
        // ikonvert-serial / maretron-ipg -p semantics. -o is
        // kept as an alias for backward compatibility.
        fprintf(stdout, "%s", msg);
        fflush(stdout);
      }
    }
    if (time(0) - lastPing > 20)
    {
      writeMessage(handle, NGT_MSG_SEND, NGT_STARTUP_SEQ, sizeof(NGT_STARTUP_SEQ), UINT64_C(0));
      lastPing = time(0);
    }
  }

  close(handle);
  return 0;
}

static void parseAndWriteIn(int handle, const char *cmd)
{
  unsigned char  msg[500];
  unsigned char *m;

  unsigned int prio;
  unsigned int pgn;
  unsigned int src;
  unsigned int dst;
  unsigned int bytes;

  char        *p;
  int          i;
  int          b;
  unsigned int byt;
  int          r;
  uint64_t     when = 0;

  if (!cmd || !*cmd || *cmd == '\n')
  {
    return;
  }

  parseTimestamp(cmd, &when);
  p = strchr(cmd, ',');
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
  writeMessage(handle, N2K_MSG_SEND, msg, m - msg, when);
}

static size_t writeUint64(uint64_t v, unsigned char *buf)
{
  size_t out = 0;
  for (int byte = 0; byte < 8; byte++)
  {
    uint8_t c = (uint8_t) v;
    if (c == ESC)
    {
      *buf++ = c;
      out++;
    }
    *buf++ = c;
    out++;
    v = v >> 8;
  }
  return out;
}

/*
 * Wrap the PGN or NGT message and send to NGT
 */
static void writeMessage(int handle, unsigned char command, const unsigned char *cmd, const size_t len, uint64_t when)
{
  unsigned char  bst[255];
  unsigned char *b = bst;
  unsigned char *r = bst;
  unsigned char  crc;

  int i;

  if (isEBL)
  {
    if (when == 0)
    {
      when = getNow();
    }
    // Prepend with timestamp
    when = (when + UINT64_C(11644473600000)) * UINT64_C(10000);

    *b++ = ESC;
    *b++ = SOH;
    *b++ = EBL_TIMESTAMP;
    b += writeUint64(when, b);
    *b++ = ESC;
    *b++ = LF;
  }

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
    if (!isFile)
    {
      logAbort("EOF on reading stdin\n");
    }
    else
    {
      exit(0);
    }
  }

  sbAppendData(&inBuffer, buf, r);
  return true;
}

/**
 * After readIn returns data, you can retrieve messages from the
 * internal buffer until getInMsg() returns false.
 */
static bool getInMsg(char *msg, size_t msgLen)
{
  char  *p;
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

  logDebug("getInMsg => '%s'\n", msg);

  return true;
}

/**
 * Handle a byte coming in from the NGT1.
 *
 */

static bool readNGT1Byte(unsigned char c)
{
  static enum MSG_State prev_state = MSG_MESSAGE;
  static enum MSG_State state      = MSG_START;
  static bool           noEscape   = false;
  static unsigned char  buf[500];
  static unsigned char *head = buf;

  logDebug("readNGT1Byte isFile=%d isEBL=%d state=%d c=0x%02x\n", isFile, isEBL, state, c);

  if (state == MSG_START && isFile && !isEBL && c == ESC)
  {
    noEscape = true;
  }

  if (state == MSG_ESCAPE)
  {
    if (c == SOH && isEBL)
    {
      head  = buf;
      state = MSG_HEADER;
    }
    else if (c == LF && isEBL)
    {
      headerReceived(buf, head - buf);
      head  = buf;
      state = MSG_START;
    }
    else if (c == ETX)
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
      if (head < buf + sizeof(buf))
      {
        *head++ = c;
      }
      state = prev_state;
    }
    else
    {
      logError("DLE followed by unexpected char %02X, ignore message\n", c);
      state = MSG_START;
    }
  }
  else if (state == MSG_MESSAGE)
  {
    if (c == DLE || (isFile && (c == ESC) && !noEscape))
    {
      prev_state = state;
      state      = MSG_ESCAPE;
    }
    else if (head < buf + sizeof(buf))
    {
      *head++ = c;
    }
  }
  else if (state == MSG_HEADER)
  {
    if (c == ESC)
    {
      prev_state = state;
      state      = MSG_ESCAPE;
    }
    else if (head < buf + sizeof(buf))
    {
      *head++ = c;
    }
  }
  else
  {
    if (c == DLE || (isFile && (c == ESC) && !noEscape))
    {
      prev_state = state;
      state      = MSG_ESCAPE;
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
      if (!isFile)
      {
        logAbort("Unable to read from NGT1 device, errno=%d\n", errno);
      }
      exit(0);
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

static void headerReceived(const unsigned char *msg, size_t msgLen)
{
  unsigned char command;
  unsigned char payloadLen;

  command    = msg[0];
  payloadLen = msgLen - 1;

  logDebug("header command = %02x len = %zu\n", command, payloadLen);

  if (command == EBL_TIMESTAMP)
  {
    if (payloadLen != 8)
    {
      logError("Invalid EBL timestamp length %zu\n", payloadLen);
      exit(3);
    }
    else
    {
      // Filetime to Unix epoch millis, see
      // https://devblogs.microsoft.com/oldnewthing/20220602-00/?p=106706
      uint64_t ft = *(uint64_t *) (msg + 1);

      ft = ft / 10000;
      ft -= 11644473600000;
      timestamp = ft;
      logDebug("EBL timestamp %" PRIu64 "\n", timestamp);
    }
  }
  else if (command == EBL_VERSION)
  {
    logDebug("EBL version\n");
  }
  else
  {
    logError("EBL unknown message type %02x\n", command);
  }
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

  if (command == N2K_MSG_RECEIVED || (isFile && command == N2K_MSG_SEND))
  {
    n2kMessageReceived(msg + 2, payloadLen, command);
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
  char  *p;
  char   dateStr[DATE_LENGTH];

  if (msgLen < 12)
  {
    logError("Ignore short msg len = %zu\n", msgLen);
    return;
  }

  snprintf(line, sizeof(line), "%s,%u,%u,%u,%u,%u", fmtTimestamp(dateStr, timestamp), 0, ACTISENSE_BEM + msg[0], 0, 0, (unsigned int) msgLen - 1);
  p = line + strlen(line);
  for (i = 1; i < msgLen && p < line + sizeof(line) - 5; i++)
  {
    snprintf(p, 4, ",%02x", msg[i]);
    p += 3;
  }
  *p++ = 0;

  puts(line);
  fflush(stdout);
}

static void n2kMessageReceived(const unsigned char *msg, size_t msgLen, const unsigned char command)
{
  unsigned int prio, src, dst;
  unsigned int pgn;
  size_t       i;
  unsigned int len;
  char         line[800];
  char        *p;
  char         dateStr[DATE_LENGTH];
  size_t       headerLen = (command == N2K_MSG_SEND) ? 6 : 11;

  if (msgLen < headerLen)
  {
    logError("Ignoring N2K message - too short\n");
    return;
  }
  prio = msg[0];
  pgn  = (unsigned int) msg[1] + 256 * ((unsigned int) msg[2] + 256 * (unsigned int) msg[3]);
  dst  = msg[4];
  if (command == N2K_MSG_SEND)
  {
    src = 0;
    len = msg[5];
  }
  else
  {
    src = msg[5];
    /* Skip the timestamp logged by the NGT-1-A in bytes 6-9 */
    len = msg[10];
  }

  if (len > 223)
  {
    logError("Ignoring N2K message - too long (%u)\n", len);
    return;
  }

  p = line;
  snprintf(p, sizeof(line), "%s,%u,%u,%u,%u,%u", fmtTimestamp(dateStr, timestamp), prio, pgn, src, dst, len);
  p += strlen(line);

  i = headerLen;
  len += i;
  if (len > msgLen)
  {
    len = msgLen;
  }
  for (; i < len; i++)
  {
    snprintf(p, line + sizeof(line) - p, ",%02x", msg[i]);
    p += strlen(p);
  }

  puts(line);
  fflush(stdout);
}
