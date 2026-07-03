/*
Read and write to a Maretron IPG100 over TCP.

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

#include "maretron-ipg.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "common.h"
#include "license.h"
#include "parse.h"

typedef enum
{
  MARETRON_FRAME_OK,
  MARETRON_FRAME_NEED_MORE,
  MARETRON_FRAME_INVALID
} MaretronParseResult;

typedef struct
{
  uint32_t       pgn;
  uint8_t        prio;
  uint8_t        src;
  uint8_t        dst;
  uint8_t        msg_type;
  uint16_t       payload_len;
  const uint8_t *payload;
} MaretronFrame;

enum IpgState
{
  IPG_AWAIT_HANDSHAKE,
  IPG_STREAMING
};

static bool          verbose;
static bool          readonly;
static bool          writeonly;
static bool          passthru;
static long          timeout;
static enum IpgState state    = IPG_AWAIT_HANDSHAKE;
static const char   *password = "";

// Yeah globals. We're trying to avoid malloc()/free() to run quickly on limited memory hardware
StringBuffer writeBuffer; // What we still have to write to device
StringBuffer readBuffer;  // What we have already read from device
StringBuffer inBuffer;    // What we have already read from stdin but is not complete yet

static void                sendConnect(void);
static void                sendSetModeBinary(void);
static void                processInBuffer(StringBuffer *in, StringBuffer *out);
static bool                processReadBuffer(StringBuffer *in, int out);
static void                handleTextMessage(char *line);
static char               *ensureDefaultPort(const char *url);
static MaretronParseResult parseMaretronFrame(const uint8_t *buf, size_t buf_len, MaretronFrame *out, size_t *consumed);
static bool                buildMaretronFrame(uint8_t       *out,
                                              size_t         out_cap,
                                              size_t        *out_len,
                                              uint32_t       pgn,
                                              uint8_t        prio,
                                              uint8_t        dst,
                                              const uint8_t *payload,
                                              size_t         payload_len);
static void                writeCsvFrame(int out, uint64_t timestamp_ms, const MaretronFrame *frame);

int main(int argc, char **argv)
{
  int    handle;
  char  *name   = argv[0];
  char  *device = 0;
  int    ac     = argc;
  char **av     = argv;
  char  *urlWithPort;

  setProgName(av[0]);
  while (ac > 1)
  {
    if (strcasecmp(av[1], "-version") == 0)
    {
      printf("%s\n", VERSION);
      exit(0);
    }
    else if (strcasecmp(av[1], "-w") == 0)
    {
      writeonly = true;
    }
    else if (strcasecmp(av[1], "-p") == 0)
    {
      passthru = true;
    }
    else if (strcasecmp(av[1], "-r") == 0)
    {
      readonly = true;
    }
    else if (strcasecmp(av[1], "-v") == 0)
    {
      verbose = true;
    }
    else if (strcasecmp(av[1], "-t") == 0 && ac > 2)
    {
      ac--;
      av++;
      timeout = strtol(av[1], 0, 10);
      logDebug("timeout set to %ld seconds\n", timeout);
    }
    else if (strcasecmp(av[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
    }
    else if (strcasecmp(av[1], "-q") == 0)
    {
      setLogLevel(LOGLEVEL_ERROR);
    }
    else if (strncmp(av[1], "--password=", STRSIZE("--password=")) == 0)
    {
      password = av[1] + STRSIZE("--password=");
    }
    else if (strcasecmp(av[1], "--password") == 0 && ac > 2)
    {
      ac--;
      av++;
      password = av[1];
    }
    else if (strcasecmp(av[1], "-fixtime") == 0 && ac > 2)
    {
      ac--;
      av++;
      setFixedTimestamp(av[1]);
    }
    else if (!device)
    {
      device = av[1];
    }
    else
    {
      device = 0;
      break;
    }
    ac--;
    av++;
  }

  if (!device)
  {
    fprintf(stderr,
            "Usage: %s [-w] [-p] [-r] [-v] [-d] [-q] [-t <n>] [--password=<str>] tcp://<host>[:<port>]\n"
            "\n"
            "Options:\n"
            "  -w                    writeonly mode, data from device is not sent to stdout\n"
            "  -r                    readonly mode, data from stdin is not sent to device\n"
            "  -p                    passthru mode, data from stdin is also sent to stdout\n"
            "  -v                    verbose\n"
            "  -d                    debug\n"
            "  -q                    quiet, suppress INFO logs\n"
            "  -t <n>                timeout, if no message is received after <n> seconds the program quits\n"
            "  --password=<str>      IPG login password (default empty)\n"
            "  -fixtime <str>        use fixed timestamp <str> for all output (testing)\n"
            "  <device> is the address of a Maretron IPG100 in the format tcp://<host>[:<port>]\n"
            "  Default TCP port is " MARETRON_DEFAULT_PORT " (bus 0); use :6553 for bus 1.\n"
            "\n"
            "  Examples: %s tcp://ipg100.local\n"
            "            %s --password=secret tcp://192.168.1.100:6543\n"
            "\n" COPYRIGHT,
            name,
            name,
            name);
    exit(1);
  }

  logDebug("Opening %s\n", device);
  if (strncmp(device, "tcp:", STRSIZE("tcp:")) != 0)
  {
    logAbort("Only TCP URLs are supported: %s\n", device);
  }
  urlWithPort = ensureDefaultPort(device);
  handle      = open_socket_stream(urlWithPort);
  logDebug("socket = %d\n", handle);
  if (handle < 0)
  {
    logAbort("Cannot open TCP stream %s\n", urlWithPort);
  }

  fputs(CANBOAT_FORMAT_FAST_HEADER, stdout);
  emitCanboatStartupRecord("maretron-ipg", device);

  sendConnect();

  if (readonly)
  {
    // Defensive: pin fd 0 to /dev/null so any stray read(0,..)
    // gets immediate EOF. The main loop's inHandle gate
    // already excludes STDIN_FILENO in readonly mode; this
    // guards against any future code path that calls read on
    // fd 0 and against the kernel handing fd 0 to a subsequent
    // open() if stdin were closed outright. Matches the
    // behaviour of actisense-serial and ikonvert-serial.
    int devnull = open("/dev/null", O_RDONLY);
    if (devnull >= 0)
    {
      dup2(devnull, STDIN_FILENO);
      close(devnull);
    }
  }

  for (;;)
  {
    uint8_t data[1024];
    ssize_t r;

    int writeHandle = (sbGetLength(&writeBuffer) > 0) ? handle : INVALID_SOCKET;
    int inHandle    = (!readonly && state == IPG_STREAMING && writeHandle == INVALID_SOCKET) ? STDIN : INVALID_SOCKET;

    int rd = isReady(handle, inHandle, writeHandle, timeout);

    logDebug("isReady(%d, %d, %d, %d) = %d\n", handle, inHandle, writeHandle, timeout, rd);

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
        logAbort("EOF on device\n");
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
        logAbort("EOF on stdin\n");
      }
    }

    if ((rd & FD3_WriteReady) > 0 && sbGetLength(&writeBuffer) > 0)
    {
      r = write(handle, sbGet(&writeBuffer), sbGetLength(&writeBuffer));
      if (r > 0)
      {
        if (verbose)
        {
          logInfo("Sent [%-1.*s]\n", r, sbGet(&writeBuffer));
        }
        sbDelete(&writeBuffer, 0, (size_t) r); // Eliminate written bytes from buffer
      }
      if (r == 0)
      {
        logAbort("EOF on device\n");
      }
    }

    if (sbGetLength(&readBuffer) > 0)
    {
      logDebug("readBuffer len=%zu\n", sbGetLength(&readBuffer));
      processReadBuffer(&readBuffer, STDOUT);
    }
  }

  close(handle);
  free(urlWithPort);
  return 0;
}

/* Received data from stdin. Once it is a full canboat plain-CSV line, parse it as
 * FORMAT_FAST then encode as a Maretron 0xA5 frame and append to the writeBuffer.
 */
static void processInBuffer(StringBuffer *in, StringBuffer *out)
{
  RawMessage msg;
  char      *p;

  while ((p = strchr(sbGet(in), '\n')) != 0)
  {
    if (!readonly && parseFastFormat(in, &msg) && msg.pgn < CANBOAT_PGN_START)
    {
      uint8_t frame[6 + FASTPACKET_MAX_SIZE];
      size_t  frame_len = 0;

      if (buildMaretronFrame(frame, sizeof frame, &frame_len, msg.pgn, msg.prio, msg.dst, msg.data, msg.len))
      {
        sbAppendData(out, frame, frame_len);
        logDebug("Queued tx frame pgn=%u len=%zu\n", msg.pgn, frame_len);
      }
      else
      {
        logError("Cannot encode tx frame pgn=%u len=%u (oversize?)\n", msg.pgn, msg.len);
      }
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

  if (!p)
  {
    if (sbGetLength(in) > sizeof("2019-01-20T14:42:04.636Z,0,129540,") + 3 * FASTPACKET_MAX_SIZE)
    {
      sbEmpty(in);
    }
    return;
  }
}

static MaretronParseResult parseMaretronFrame(const uint8_t *buf, size_t buf_len, MaretronFrame *out, size_t *consumed)
{
  if (buf_len < 1)
  {
    return MARETRON_FRAME_NEED_MORE;
  }
  if (buf[0] != MARETRON_FRAME_SYNC)
  {
    return MARETRON_FRAME_INVALID;
  }
  if (buf_len < 6)
  {
    return MARETRON_FRAME_NEED_MORE;
  }

  uint8_t f1 = buf[1];
  if ((f1 & MARETRON_F1_SYNC_BIT) == 0)
  {
    return MARETRON_FRAME_INVALID;
  }

  uint8_t pf       = buf[2];
  uint8_t ps       = buf[3];
  uint8_t sa       = buf[4];
  uint8_t prio     = (f1 >> 4) & 0x07;
  uint8_t msg_type = (f1 >> 1) & 0x03;
  uint8_t dp       = f1 & 0x01;

  size_t   payload_start;
  uint16_t payload_len;

  if (msg_type == 3)
  {
    if (buf_len < 7)
    {
      return MARETRON_FRAME_NEED_MORE;
    }
    payload_len   = (uint16_t) buf[5] | ((uint16_t) buf[6] << 8);
    payload_start = 7;
  }
  else
  {
    payload_len   = buf[5];
    payload_start = 6;
  }

  size_t total = payload_start + payload_len;
  if (buf_len < total)
  {
    return MARETRON_FRAME_NEED_MORE;
  }

  uint32_t pgn;
  uint8_t  dst;

  if (pf < 0xF0)
  {
    pgn = ((uint32_t) dp << 16) | ((uint32_t) pf << 8);
    dst = ps;
  }
  else
  {
    pgn = ((uint32_t) dp << 16) | ((uint32_t) pf << 8) | ps;
    dst = 0xFF;
  }

  out->pgn         = pgn;
  out->prio        = prio;
  out->src         = sa;
  out->dst         = dst;
  out->msg_type    = msg_type;
  out->payload_len = payload_len;
  out->payload     = buf + payload_start;

  *consumed = total;
  return MARETRON_FRAME_OK;
}

/*
 * Encode a Maretron IPG100 binary frame into out[0..*out_len).
 * Returns true on success. Returns false if payload_len exceeds the
 * canboat plain-CSV limit (FASTPACKET_MAX_SIZE) or out_cap is too small.
 *
 * SA on the wire is always 0xFF; the IPG substitutes its claimed source
 * address. msg_type is 1 when payload_len <= 8 else 2.
 */
static bool buildMaretronFrame(uint8_t       *out,
                               size_t         out_cap,
                               size_t        *out_len,
                               uint32_t       pgn,
                               uint8_t        prio,
                               uint8_t        dst,
                               const uint8_t *payload,
                               size_t         payload_len)
{
  if (payload_len > FASTPACKET_MAX_SIZE)
  {
    return false;
  }
  if (out_cap < 6 + payload_len)
  {
    return false;
  }

  uint8_t msg_type = (payload_len <= 8) ? 1 : 2;
  uint8_t dp       = (uint8_t) ((pgn >> 16) & 0x01);
  uint8_t pf       = (uint8_t) ((pgn >> 8) & 0xFF);
  uint8_t ps;

  if (pf < 0xF0)
  {
    ps = dst;
  }
  else
  {
    ps = (uint8_t) (pgn & 0xFF);
  }

  out[0] = MARETRON_FRAME_SYNC;
  out[1] = (uint8_t) (MARETRON_F1_SYNC_BIT | ((prio & 0x07) << 4) | ((msg_type & 0x03) << 1) | (dp & 0x01));
  out[2] = pf;
  out[3] = ps;
  out[4] = 0xFF;
  out[5] = (uint8_t) payload_len;
  if (payload_len > 0)
  {
    memcpy(out + 6, payload, payload_len);
  }

  *out_len = 6 + payload_len;
  return true;
}

static void writeCsvFrame(int out, uint64_t timestamp_ms, const MaretronFrame *frame)
{
  char         dateStr[DATE_LENGTH];
  StringBuffer sb = sbNew;
  ssize_t      r;

  sbAppendFormat(&sb,
                 "%s,%u,%u,%u,%u,%u,",
                 fmtTimestamp(dateStr, timestamp_ms),
                 frame->prio,
                 frame->pgn,
                 frame->src,
                 frame->dst,
                 frame->payload_len);
  sbAppendEncodeHex(&sb, frame->payload, frame->payload_len, ',');
  sbAppendString(&sb, "\n");

  r = write(out, sbGet(&sb), sbGetLength(&sb));
  if (r <= 0)
  {
    logAbort("Cannot write to output\n");
  }
  sbClean(&sb);
}

static void sendConnect(void)
{
  sbAppendFormat(&writeBuffer, TX_CONNECT_MSG, password);
  sbAppendChar(&writeBuffer, '\0');
  logDebug("Queued CONNECT message (%zu bytes)\n", sbGetLength(&writeBuffer));
}

static void sendSetModeBinary(void)
{
  sbAppendString(&writeBuffer, TX_SET_MODE_BINARY_MSG);
  sbAppendChar(&writeBuffer, '\0');
  logDebug("Queued SET_MODE BINARY (%zu bytes pending)\n", sbGetLength(&writeBuffer));
}

static void handleTextMessage(char *line)
{
  char *tab = strchr(line, '\t');
  char *rest;
  if (tab != NULL)
  {
    *tab = '\0';
    rest = tab + 1;
  }
  else
  {
    rest = (char *) "";
  }

  if (strcmp(line, RX_CONNECTED) == 0)
  {
    logInfo("Connected to IPG (serial %s)\n", rest);
    sendSetModeBinary();
    state = IPG_STREAMING;
  }
  else if (strcmp(line, RX_NO) == 0)
  {
    logAbort("Authentication rejected by IPG\n");
  }
  else if (strcmp(line, RX_SERVER_VERSION) == 0)
  {
    logInfo("IPG server version %s\n", rest);
  }
  else if (strcmp(line, RX_INSTANCE_DATA) == 0)
  {
    logInfo("IPG instance data %s\n", rest);
  }
  else if (strcmp(line, RX_LICENSES_USED) == 0 || strcmp(line, RX_DETAILED_LICENSES_USED) == 0)
  {
    logDebug("%s %s\n", line, rest);
  }
  else
  {
    logDebug("Unhandled IPG control message: %s %s\n", line, rest);
  }
}

/*
 * Append :MARETRON_DEFAULT_PORT to the URL if no port is present. Returns malloc'd
 * string the caller owns (or a strdup of the original if it already had one).
 */
static char *ensureDefaultPort(const char *url)
{
  const char *body = url;
  if (strncmp(body, "tcp:", STRSIZE("tcp:")) == 0)
  {
    body += STRSIZE("tcp:");
  }
  while (*body == '/')
  {
    body++;
  }
  /* Bracketed IPv6 literals supply their own port-delimiter unambiguity */
  if (strchr(body, ':') != NULL)
  {
    return strdup(url);
  }
  size_t len = strlen(url) + 1 + STRSIZE(MARETRON_DEFAULT_PORT) + 1;
  char  *out = malloc(len);
  if (!out)
  {
    die("Out of memory");
  }
  snprintf(out, len, "%s:%s", url, MARETRON_DEFAULT_PORT);
  return out;
}

static bool processReadBuffer(StringBuffer *in, int out)
{
  bool receivedData = false;

  logDebug("processReadBuffer len=%zu\n", sbGetLength(in));
  while (sbGetLength(in) > 0)
  {
    const uint8_t *p     = (const uint8_t *) sbGet(in);
    size_t         rem   = sbGetLength(in);
    uint8_t        first = p[0];

    if (state == IPG_AWAIT_HANDSHAKE)
    {
      char *nul = memchr(p, 0, rem);
      if (nul == NULL)
      {
        return receivedData;
      }
      handleTextMessage((char *) sbGet(in));
      sbDelete(in, 0, (size_t) ((const uint8_t *) nul + 1 - p));
      continue;
    }

    /* IPG_STREAMING */
    if (first == MARETRON_FRAME_SYNC)
    {
      MaretronFrame       frame;
      size_t              consumed = 0;
      MaretronParseResult r        = parseMaretronFrame(p, rem, &frame, &consumed);

      if (r == MARETRON_FRAME_NEED_MORE)
      {
        return receivedData;
      }
      if (r == MARETRON_FRAME_INVALID)
      {
        logDebug("Invalid sync at start of frame, dropping 1 byte\n");
        sbDelete(in, 0, 1);
        continue;
      }
      if (!writeonly)
      {
        writeCsvFrame(out, getNow(), &frame);
      }
      sbDelete(in, 0, consumed);
      receivedData = true;
      continue;
    }

    if (first == MARETRON_VIDEO_PREFIX || first == MARETRON_ASCII_PREFIX)
    {
      char *nul = memchr(p, 0, rem);
      if (nul == NULL)
      {
        return receivedData;
      }
      sbDelete(in, 0, (size_t) ((const uint8_t *) nul + 1 - p));
      continue;
    }

    if ((first & 0x80) == 0)
    {
      char *nul = memchr(p, 0, rem);
      if (nul == NULL)
      {
        return receivedData;
      }
      handleTextMessage((char *) sbGet(in));
      sbDelete(in, 0, (size_t) ((const uint8_t *) nul + 1 - p));
      continue;
    }

    logDebug("Out-of-sync byte 0x%02x, dropping\n", first);
    sbDelete(in, 0, 1);
  }

  return receivedData;
}
