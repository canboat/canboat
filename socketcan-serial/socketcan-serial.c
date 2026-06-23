/*

Read and write NMEA 2000 data on a Linux SocketCAN interface, behaving as
much like actisense-serial and ikonvert-serial as possible:

  - received CAN frames are reassembled into whole PGNs and written to stdout
    in canboat FAST format (one line per PGN, with a "# format=FAST" header);
  - canboat format lines read from stdin are split into CAN frames and sent
    to the bus;
  - the program is a real node on the bus: it claims an ISO source address
    (PGN 60928), backs off / re-claims on conflict, and answers ISO Requests
    (PGN 59904) for its address claim.

Unlike the NGT-1 / iKonvert, a raw SocketCAN socket only ever delivers
individual <= 8 byte CAN frames, so the fast-packet reassembly that those
gateways do in hardware is done here in software. Which PGNs are fast-packet
is decided by range (see common.h) except for the mixed 0x1F000..0x1FFFF
range, for which fastpacket-table.h holds a per-PGN lookup generated from
canboat.json at build time.

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

#define _GNU_SOURCE
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "common.h"
#include "fastpacket-table.h"
#include "parse.h"

#define PGN_ISO_ADDRESS_CLAIM (60928)
#define PGN_ISO_REQUEST (59904)

#define N2K_ADDR_GLOBAL (255)
#define N2K_ADDR_NULL (254) /* "cannot claim address" / no source */
#define N2K_MAX_ADDR (253)  /* highest claimable address */

#define ADDRESS_CLAIM_TIMEOUT (250)  /* ms to wait for a contender before we own the address */
#define ADDRESS_SCAN_TIMEOUT (1000)  /* ms to listen for existing claims before picking an address */

#define PGN_HEARTBEAT (126993)
#define HEARTBEAT_DEFAULT_INTERVAL (60000) /* ms, the NMEA 2000 default heartbeat rate */

#define MAX_FASTPACKETS (64)

enum ClaimState
{
  CLAIM_DISABLED, /* -n: behave as a passive bridge, do not claim */
  CLAIM_SCANNING, /* ISO Request sent, listening for existing claims before we pick */
  CLAIM_PENDING,  /* claim sent, waiting out the 250ms contention window */
  CLAIM_CLAIMED,  /* address is ours */
  CLAIM_FAILED    /* could not claim (not arbitrary-address-capable and lost) */
};

/* Command-line / behaviour flags, named as in actisense-serial. */
static int  verbose   = 0;
static int  readonly  = 0;
static int  writeonly = 0;
static int  passthru  = 0;
static long timeout   = 0;

/* ISO device identity (NAME, see PGN 60928) and address-claim state. */
static uint32_t        uniqueNumber     = 0;
static unsigned int    manufacturerCode = 999; /* Signal K */
static unsigned int    deviceFunction   = 130;  /* PC Gateway */
static unsigned int    deviceClass      = 25;   /* Inter/Intranetwork Device */
static unsigned int    industryGroup    = 4;    /* Marine */
static unsigned int    deviceInstance   = 0;
static unsigned int    systemInstance   = 0;
static bool            arbitraryCapable = true;
static uint64_t        deviceName       = 0;
static unsigned int    preferredAddress = 0;
static unsigned int    address          = 0;
static enum ClaimState claimState       = CLAIM_PENDING;
static uint64_t        claimDeadline    = 0;        /* ms; end of contention window */
static bool            addressUsed[N2K_MAX_ADDR + 1]; /* addresses seen claimed by others */

/* Heartbeat (PGN 126993): emitted every heartbeatInterval ms once claimed. */
static long     heartbeatInterval = HEARTBEAT_DEFAULT_INTERVAL; /* 0 disables */
static uint8_t  heartbeatSeq      = 0;
static uint64_t nextHeartbeat     = 0; /* ms; when to send the next heartbeat */

typedef struct
{
  bool     used;
  uint32_t pgn;
  uint8_t  src;
  uint8_t  seq;     /* 3-bit sequence id (high bits of frame[0]) */
  uint8_t  size;    /* declared total payload length */
  uint8_t  fill;    /* bytes received so far */
  uint8_t  nextIdx; /* expected frame index */
  uint64_t when;    /* timestamp of the first frame */
  uint8_t  data[FASTPACKET_MAX_SIZE];
} FastPacket;

static FastPacket fastPackets[MAX_FASTPACKETS];

static int  openCanDevice(const char *device, int *canSocket);
static int  readCan(int sock);
static void handleFrame(uint32_t canId, const uint8_t *data, uint8_t len, uint64_t when);
static void emitMessage(uint64_t when, uint8_t prio, uint32_t pgn, uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len);
static bool isFastPacket(uint32_t pgn);
static void parseAndWriteIn(int sock, const char *cmd);
static void sendN2k(int sock, uint8_t prio, uint32_t pgn, uint8_t src, uint8_t dst, const uint8_t *data, size_t len);
static void sendCanFrame(int sock, uint32_t canId, const uint8_t *data, uint8_t len);
static bool readIn(void);
static bool getInMsg(char *msg, size_t len);

/* Address claim */
static void     buildDeviceName(void);
static void     startAddressClaim(int sock);
static void     beginAddressClaim(int sock);
static void     sendAddressClaim(int sock, uint8_t dst);
static void     sendIsoRequest(int sock, uint8_t src, uint8_t dst, uint32_t pgn);
static void     handleAddressClaim(int sock, uint8_t src, const uint8_t *data, uint8_t len);
static void     handleIsoRequest(int sock, uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len);
static void     tickAddressClaim(int sock, uint64_t now);
static int      pickFreeAddress(void);
static uint64_t nameFromBytes(const uint8_t *data);
static void     sendHeartbeat(int sock);

int main(int argc, char **argv)
{
  const char *name   = argv[0];
  const char *device = NULL;
  int         sock;

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
    else if (strcasecmp(argv[1], "-r") == 0)
    {
      readonly = 1;
    }
    else if (strcasecmp(argv[1], "-p") == 0)
    {
      passthru = 1;
    }
    else if (strcasecmp(argv[1], "-v") == 0)
    {
      verbose = 1;
    }
    else if (strcasecmp(argv[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
    }
    else if (strcasecmp(argv[1], "-n") == 0)
    {
      claimState = CLAIM_DISABLED;
    }
    else if (strcasecmp(argv[1], "-t") == 0 && argc > 2)
    {
      argc--, argv++;
      timeout = strtol(argv[1], 0, 10);
    }
    else if (strcasecmp(argv[1], "-a") == 0 && argc > 2)
    {
      argc--, argv++;
      preferredAddress = (unsigned int) strtol(argv[1], 0, 10) & 0xff;
    }
    else if (strcasecmp(argv[1], "-u") == 0 && argc > 2)
    {
      argc--, argv++;
      uniqueNumber = (uint32_t) strtoul(argv[1], 0, 10);
    }
    else if (strcasecmp(argv[1], "-m") == 0 && argc > 2)
    {
      argc--, argv++;
      manufacturerCode = (unsigned int) strtoul(argv[1], 0, 10);
    }
    else if (strcasecmp(argv[1], "-hb") == 0 && argc > 2)
    {
      argc--, argv++;
      heartbeatInterval = strtol(argv[1], 0, 10);
    }
    else if (!device)
    {
      device = argv[1];
    }
    else
    {
      device = NULL;
      break;
    }
    argc--, argv++;
  }

  if (!device)
  {
    fprintf(stderr,
            "Usage: %s [-w] [-r] [-p] [-v] [-d] [-n] [-t <n>] [-a <addr>] [-u <n>] [-m <n>] [-hb <ms>] <can-device>\n"
            "\n"
            "Bridge a Linux SocketCAN interface to/from canboat FAST format.\n"
            "\n"
            "Options:\n"
            "  -w        writeonly mode, received frames are not sent to stdout\n"
            "  -r        readonly mode, data from stdin is not sent to the device\n"
            "  -p        passthru mode, data from stdin is also echoed to stdout\n"
            "  -v        verbose\n"
            "  -d        debug\n"
            "  -n        do not claim an address (passive bridge only)\n"
            "  -t <n>    timeout, quit if no frame is received for <n> seconds\n"
            "  -a <addr> preferred source address to claim (default 0)\n"
            "  -u <n>    unique number for the ISO NAME (default derived from pid)\n"
            "  -m <n>    manufacturer code for the ISO NAME (default %u)\n"
            "  -hb <ms>  heartbeat (PGN 126993) interval in ms, default %d, 0 disables\n"
            "\n"
            "  <can-device> is a SocketCAN interface name, e.g. can0 or nmea2000.\n"
            "\n"
            "  Example: %s nmea2000 | analyzer\n"
            "\n" COPYRIGHT,
            name,
            manufacturerCode,
            HEARTBEAT_DEFAULT_INTERVAL,
            name);
    exit(1);
  }

  if (uniqueNumber == 0)
  {
    uniqueNumber = (uint32_t) (getpid() & 0x1fffff);
  }
  address = preferredAddress;

  fputs(CANBOAT_FORMAT_FAST_HEADER, stdout);
  emitCanboatStartupRecord("socketcan-serial", device);

  if (openCanDevice(device, &sock))
  {
    exit(1);
  }

  if (claimState != CLAIM_DISABLED)
  {
    buildDeviceName();
    startAddressClaim(sock);
  }

  uint64_t lastFrame = getNow();

  for (;;)
  {
    fd_set         fds;
    struct timeval tv;
    uint64_t       now = getNow();
    int            r;
    int            maxfd = sock;

    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    if (!readonly)
    {
      FD_SET(STDIN_FILENO, &fds);
      if (STDIN_FILENO > maxfd)
      {
        maxfd = STDIN_FILENO;
      }
    }

    /* Wake at the soonest of: the claim deadline, the next heartbeat, or a
     * 1s poll, so the timers advance even on an otherwise silent bus. */
    uint64_t waitMs = 1000;
    if ((claimState == CLAIM_PENDING || claimState == CLAIM_SCANNING) && claimDeadline > now)
    {
      waitMs = claimDeadline - now;
    }
    else if (claimState == CLAIM_CLAIMED && heartbeatInterval > 0)
    {
      waitMs = (nextHeartbeat > now) ? nextHeartbeat - now : 0;
    }
    tv.tv_sec  = waitMs / 1000;
    tv.tv_usec = (waitMs % 1000) * 1000;

    r = select(maxfd + 1, &fds, NULL, NULL, &tv);
    if (r < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      logAbort("select failed: %s\n", strerror(errno));
    }
    if (r == 0 && timeout && (getNow() - lastFrame) >= (uint64_t) timeout * 1000)
    {
      logAbort("Timeout %ld seconds; no data received\n", timeout);
    }

    if (FD_ISSET(sock, &fds))
    {
      lastFrame = getNow();
      if (readCan(sock) < 0)
      {
        break;
      }
    }
    if (!readonly && FD_ISSET(STDIN_FILENO, &fds))
    {
      char line[2000];

      if (!readIn())
      {
        /* stdin closed: stop reading it but keep bridging the bus,
         * rather than quitting like the serial tools do. */
        logDebug("EOF on stdin, continuing read-only\n");
        readonly = 1;
      }
      else
      {
        while (getInMsg(line, sizeof(line)))
        {
          parseAndWriteIn(sock, line);
          if (passthru)
          {
            fputs(line, stdout);
            fflush(stdout);
          }
        }
      }
    }

    tickAddressClaim(sock, getNow());

    if (claimState == CLAIM_CLAIMED && heartbeatInterval > 0 && getNow() >= nextHeartbeat)
    {
      sendHeartbeat(sock);
      nextHeartbeat = getNow() + (uint64_t) heartbeatInterval;
    }
  }

  close(sock);
  return 0;
}

static int openCanDevice(const char *device, int *canSocket)
{
  struct sockaddr_can addr;
  struct ifreq        ifr;
  int                 sock;
  int                 on = 1;

  if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    logError("socket: %s\n", strerror(errno));
    return 1;
  }

  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, device, IFNAMSIZ - 1);
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex)
  {
    logError("Cannot find CAN interface '%s': %s\n", device, strerror(errno));
    return 1;
  }

  memset(&addr, 0, sizeof(addr));
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(sock, (struct sockaddr *) &addr, sizeof(addr)) < 0)
  {
    logError("bind: %s\n", strerror(errno));
    return 1;
  }

  /* Kernel RX timestamps, so the emitted line carries the time the
   * frame actually arrived rather than when we got round to it. */
  if (setsockopt(sock, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0)
  {
    logError("SO_TIMESTAMP: %s (continuing without kernel timestamps)\n", strerror(errno));
  }

  *canSocket = sock;
  return 0;
}

/*
 * Drain all currently-readable CAN frames. recvmsg() also yields the
 * kernel SO_TIMESTAMP ancillary data.
 */
static int readCan(int sock)
{
  /* Drain at most a batch per call, then yield to the main loop so the
   * address-claim timers keep advancing on a busy bus. */
  int budget = 256;
  while (budget-- > 0)
  {
    struct can_frame frame;
    struct iovec     iov;
    struct msghdr    msg;
    char             control[CMSG_SPACE(sizeof(struct timeval))];
    struct cmsghdr  *cmsg;
    ssize_t          n;
    uint64_t         when = 0;

    memset(&msg, 0, sizeof(msg));
    iov.iov_base       = &frame;
    iov.iov_len        = sizeof(frame);
    msg.msg_iov        = &iov;
    msg.msg_iovlen     = 1;
    msg.msg_control    = control;
    msg.msg_controllen = sizeof(control);

    n = recvmsg(sock, &msg, 0);
    if (n < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        return 0;
      }
      if (errno == EINTR)
      {
        continue;
      }
      logError("recvmsg: %s\n", strerror(errno));
      return -1;
    }
    if (n < (ssize_t) sizeof(struct can_frame))
    {
      continue;
    }
    if ((frame.can_id & CAN_ERR_FLAG) != 0)
    {
      continue; /* error frame, not a real message */
    }

    for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL; cmsg = CMSG_NXTHDR(&msg, cmsg))
    {
      if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP)
      {
        struct timeval tv;
        memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
        when = (uint64_t) tv.tv_sec * 1000 + tv.tv_usec / 1000;
      }
    }
    if (when == 0)
    {
      when = getNow();
    }

    handleFrame(frame.can_id & CAN_EFF_MASK, frame.data, frame.can_dlc, when);
  }
  return 0;
}

static void handleFrame(uint32_t canId, const uint8_t *data, uint8_t len, uint64_t when)
{
  unsigned int prio, pgn, src, dst;
  int          slot;
  FastPacket  *fp;
  uint8_t      idx, seq, bucket, offset;

  getISO11783BitsFromCanId(canId, &prio, &pgn, &src, &dst);

  /* Act on the messages that drive the address-claim protocol before
   * anything else, but still emit them to stdout like any other frame. */
  if (claimState != CLAIM_DISABLED)
  {
    if (pgn == PGN_ISO_ADDRESS_CLAIM)
    {
      handleAddressClaim(0 /* sock set below */, (uint8_t) src, data, len);
    }
    else if (pgn == PGN_ISO_REQUEST)
    {
      handleIsoRequest(0, (uint8_t) src, (uint8_t) dst, data, len);
    }
  }

  if (writeonly)
  {
    return;
  }

  if (!isFastPacket(pgn))
  {
    emitMessage(when, (uint8_t) prio, pgn, (uint8_t) src, (uint8_t) dst, data, len);
    return;
  }

  /* Fast packet: frame[0] = (seq << 5) | index, frame 0 also has the
   * total size in frame[1] and 6 payload bytes; later frames carry 7. */
  if (len < 1)
  {
    return;
  }
  idx = data[FASTPACKET_INDEX] & FASTPACKET_MAX_INDEX;
  seq = data[FASTPACKET_INDEX] >> 5;

  /* Find an existing reassembly for this PGN+source. */
  slot = -1;
  for (int i = 0; i < MAX_FASTPACKETS; i++)
  {
    if (fastPackets[i].used && fastPackets[i].pgn == pgn && fastPackets[i].src == src)
    {
      slot = i;
      break;
    }
  }

  if (idx == 0)
  {
    if (len < 2)
    {
      return;
    }
    if (slot < 0)
    {
      for (int i = 0; i < MAX_FASTPACKETS; i++)
      {
        if (!fastPackets[i].used)
        {
          slot = i;
          break;
        }
      }
    }
    if (slot < 0)
    {
      logError("No free fast-packet slot, dropping pgn %u src %u\n", pgn, src);
      return;
    }
    fp          = &fastPackets[slot];
    fp->used    = true;
    fp->pgn     = pgn;
    fp->src     = (uint8_t) src;
    fp->seq     = seq;
    fp->size    = data[FASTPACKET_SIZE];
    fp->when    = when;
    fp->nextIdx = 1;
    bucket      = (uint8_t) CB_MIN(FASTPACKET_BUCKET_0_SIZE, len - FASTPACKET_BUCKET_0_OFFSET);
    bucket      = (uint8_t) CB_MIN(bucket, fp->size);
    memcpy(fp->data, data + FASTPACKET_BUCKET_0_OFFSET, bucket);
    fp->fill = bucket;
  }
  else
  {
    if (slot < 0)
    {
      logDebug("Fast-packet continuation without start, pgn %u src %u idx %u\n", pgn, src, idx);
      return;
    }
    fp = &fastPackets[slot];
    if (seq != fp->seq || idx != fp->nextIdx)
    {
      logDebug("Fast-packet out of order pgn %u src %u: got seq %u idx %u, want seq %u idx %u\n",
               pgn,
               src,
               seq,
               idx,
               fp->seq,
               fp->nextIdx);
      fp->used = false; /* give up on this one */
      return;
    }
    offset = (uint8_t) (FASTPACKET_BUCKET_0_SIZE + (idx - 1) * FASTPACKET_BUCKET_N_SIZE);
    bucket = (uint8_t) CB_MIN(FASTPACKET_BUCKET_N_SIZE, len - FASTPACKET_BUCKET_N_OFFSET);
    bucket = (uint8_t) CB_MIN(bucket, fp->size - fp->fill);
    if (offset + bucket > FASTPACKET_MAX_SIZE)
    {
      fp->used = false;
      return;
    }
    memcpy(fp->data + offset, data + FASTPACKET_BUCKET_N_OFFSET, bucket);
    fp->fill += bucket;
    fp->nextIdx++;
  }

  if (fp->fill >= fp->size)
  {
    emitMessage(fp->when, (uint8_t) prio, pgn, (uint8_t) src, (uint8_t) dst, fp->data, fp->size);
    fp->used = false;
  }
}

static void emitMessage(uint64_t when, uint8_t prio, uint32_t pgn, uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len)
{
  char  line[800];
  char  dateStr[DATE_LENGTH];
  char *p = line;
  char *end = line + sizeof(line);

  snprintf(p, end - p, "%s,%u,%u,%u,%u,%u", fmtTimestamp(dateStr, when), prio, pgn, src, dst, len);
  p += strlen(p);
  for (uint8_t i = 0; i < len && p < end - 4; i++)
  {
    snprintf(p, end - p, ",%02x", data[i]);
    p += 3;
  }

  puts(line);
  fflush(stdout);
}

static bool isFastPacket(uint32_t pgn)
{
  if (pgn >= FASTPACKET_MIXED_START && pgn < FASTPACKET_MIXED_END)
  {
    return FASTPACKET_MIXED[pgn - FASTPACKET_MIXED_START];
  }
  if (pgn >= 0x10000 && pgn < FASTPACKET_MIXED_START)
  {
    return true; /* fast-packet-only range */
  }
  if (pgn >= CANBOAT_PGN_START)
  {
    return true; /* synthetic CANboat fast PGNs (not seen on the wire) */
  }
  return false; /* single-frame */
}

/*
 * Parse a canboat format line from stdin and transmit it as one or more
 * CAN frames. The source address is forced to our claimed address.
 */
static void parseAndWriteIn(int sock, const char *cmd)
{
  RawMessage m;
  char       copy[2000];

  if (!cmd || !*cmd || *cmd == '\n' || *cmd == '#')
  {
    return;
  }

  strncpy(copy, cmd, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  if (parseRawFormatFast(copy, &m, false))
  {
    logError("Unable to parse incoming message '%s'\n", cmd);
    return;
  }
  if (m.pgn >= CANBOAT_PGN_START)
  {
    return; /* synthetic status PGNs are not for the bus */
  }

  sendN2k(sock, m.prio, m.pgn, (uint8_t) address, m.dst, m.data, m.len);
}

/*
 * Send an NMEA 2000 message, splitting into a fast packet when needed.
 * (Same wire layout as socketcan-writer.c.)
 */
static void sendN2k(int sock, uint8_t prio, uint32_t pgn, uint8_t src, uint8_t dst, const uint8_t *data, size_t len)
{
  uint32_t canId = getCanIdFromISO11783Bits(prio, pgn, src, dst);

  if (len <= 8)
  {
    sendCanFrame(sock, canId, data, (uint8_t) len);
    return;
  }

  size_t  remaining = len;
  uint8_t index     = 0;
  uint8_t frame[8];

  while (remaining > 0)
  {
    size_t chunk;
    frame[0] = index;
    if (index == 0)
    {
      frame[1] = (uint8_t) len;
      chunk    = CB_MIN(FASTPACKET_BUCKET_0_SIZE, remaining);
      memcpy(frame + FASTPACKET_BUCKET_0_OFFSET, data, chunk);
      sendCanFrame(sock, canId, frame, (uint8_t) (FASTPACKET_BUCKET_0_OFFSET + chunk));
    }
    else
    {
      chunk = CB_MIN(FASTPACKET_BUCKET_N_SIZE, remaining);
      memcpy(frame + FASTPACKET_BUCKET_N_OFFSET, data + (len - remaining), chunk);
      sendCanFrame(sock, canId, frame, (uint8_t) (FASTPACKET_BUCKET_N_OFFSET + chunk));
    }
    remaining -= chunk;
    index++;
  }
}

static void sendCanFrame(int sock, uint32_t canId, const uint8_t *data, uint8_t len)
{
  struct can_frame frame;

  memset(&frame, 0, sizeof(frame));
  frame.can_id  = canId | CAN_EFF_FLAG;
  frame.can_dlc = len;
  memcpy(frame.data, data, len);

  if (write(sock, &frame, sizeof(frame)) != (ssize_t) sizeof(frame))
  {
    logError("write to CAN: %s\n", strerror(errno));
  }
}

static StringBuffer inBuffer;

static bool readIn(void)
{
  unsigned char buf[2000];
  ssize_t       r = read(STDIN_FILENO, buf, sizeof(buf));

  if (r <= 0)
  {
    return false; /* EOF on stdin */
  }
  sbAppendData(&inBuffer, buf, r);
  return true;
}

static bool getInMsg(char *msg, size_t msgLen)
{
  char  *p = strchr(sbGet(&inBuffer), '\n');
  size_t len;

  if (!p)
  {
    return false;
  }
  len = p - sbGet(&inBuffer) + 1;
  if (len >= msgLen)
  {
    len = msgLen - 1;
  }
  memcpy(msg, sbGet(&inBuffer), len);
  msg[len] = 0;
  sbDelete(&inBuffer, 0, (p - sbGet(&inBuffer)) + 1);
  return true;
}

/*
 * --- ISO address claim -------------------------------------------------
 *
 * The CAN socket has the global send pointer; handleFrame() calls the
 * claim handlers with sock==0, so we keep the socket here.
 */
static int claimSock = -1;

static uint64_t nameFromBytes(const uint8_t *data)
{
  uint64_t name = 0;
  for (int i = 0; i < 8; i++)
  {
    name |= (uint64_t) data[i] << (8 * i);
  }
  return name;
}

static void buildDeviceName(void)
{
  deviceName = ((uint64_t) (uniqueNumber & 0x1fffff)) | ((uint64_t) (manufacturerCode & 0x7ff) << 21)
               | ((uint64_t) (deviceInstance & 0x07) << 32)         /* Device Instance Lower */
               | ((uint64_t) ((deviceInstance >> 3) & 0x1f) << 35)  /* Device Instance Upper */
               | ((uint64_t) (deviceFunction & 0xff) << 40) | ((uint64_t) (deviceClass & 0x7f) << 49)
               | ((uint64_t) (systemInstance & 0x0f) << 56) | ((uint64_t) (industryGroup & 0x07) << 60)
               | ((uint64_t) (arbitraryCapable ? 1 : 0) << 63);
}

static void startAddressClaim(int sock)
{
  claimSock     = sock;
  claimState    = CLAIM_SCANNING;
  claimDeadline = getNow() + ADDRESS_SCAN_TIMEOUT;
  /* Ask every node to (re)announce its address claim so we learn which
   * addresses are taken before we pick one. Sent from the null address
   * because we have not claimed yet. */
  sendIsoRequest(sock, N2K_ADDR_NULL, N2K_ADDR_GLOBAL, PGN_ISO_ADDRESS_CLAIM);
  logDebug("Scanning bus %d ms before claiming (NAME 0x%016" PRIx64 ")\n", ADDRESS_SCAN_TIMEOUT, deviceName);
}

static void beginAddressClaim(int sock)
{
  if (addressUsed[preferredAddress])
  {
    int next = pickFreeAddress();
    if (next < 0)
    {
      logError("No free address available; cannot claim\n");
      address    = N2K_ADDR_NULL;
      claimState = CLAIM_FAILED;
      sendAddressClaim(sock, N2K_ADDR_GLOBAL);
      return;
    }
    logDebug("Preferred address %u is taken, using %d\n", preferredAddress, next);
    address = (unsigned int) next;
  }
  else
  {
    address = preferredAddress;
  }
  claimState    = CLAIM_PENDING;
  claimDeadline = getNow() + ADDRESS_CLAIM_TIMEOUT;
  sendAddressClaim(sock, N2K_ADDR_GLOBAL);
  logDebug("Claiming address %u with NAME 0x%016" PRIx64 "\n", address, deviceName);
}

static void sendIsoRequest(int sock, uint8_t src, uint8_t dst, uint32_t pgn)
{
  uint8_t data[3];
  data[0] = (uint8_t) pgn;
  data[1] = (uint8_t) (pgn >> 8);
  data[2] = (uint8_t) (pgn >> 16);
  sendN2k(sock, 6, PGN_ISO_REQUEST, src, dst, data, sizeof(data));
}

static void sendAddressClaim(int sock, uint8_t dst)
{
  uint8_t data[8];
  for (int i = 0; i < 8; i++)
  {
    data[i] = (uint8_t) (deviceName >> (8 * i));
  }
  sendN2k(sock, 6, PGN_ISO_ADDRESS_CLAIM, (uint8_t) address, dst, data, sizeof(data));
}

static int pickFreeAddress(void)
{
  for (int a = 0; a <= N2K_MAX_ADDR; a++)
  {
    if (!addressUsed[a])
    {
      return a;
    }
  }
  return -1;
}

static void handleAddressClaim(int sock, uint8_t src, const uint8_t *data, uint8_t len)
{
  uint64_t theirName;

  (void) sock;
  if (len < 8 || src > N2K_MAX_ADDR)
  {
    return;
  }
  theirName = nameFromBytes(data);

  /* While scanning we own no address yet — just learn what is in use. */
  if (claimState == CLAIM_SCANNING)
  {
    addressUsed[src] = true;
    return;
  }

  if (src != address)
  {
    addressUsed[src] = true; /* remember it so we don't pick it */
    return;
  }

  /* Someone is claiming our address. Lowest NAME wins (ISO 11783-5). */
  if (deviceName < theirName)
  {
    logDebug("Won address %u conflict (our NAME lower), re-claiming\n", address);
    sendAddressClaim(claimSock, N2K_ADDR_GLOBAL);
    claimState    = CLAIM_PENDING;
    claimDeadline = getNow() + ADDRESS_CLAIM_TIMEOUT;
    return;
  }

  /* We lost. */
  addressUsed[src] = true;
  if (arbitraryCapable)
  {
    int next = pickFreeAddress();
    if (next < 0)
    {
      logError("Lost address %u and no free address left; cannot claim\n", address);
      address    = N2K_ADDR_NULL;
      claimState = CLAIM_FAILED;
      sendAddressClaim(claimSock, N2K_ADDR_GLOBAL);
      return;
    }
    logDebug("Lost address %u conflict, moving to %d\n", address, next);
    address       = (unsigned int) next;
    claimState    = CLAIM_PENDING;
    claimDeadline = getNow() + ADDRESS_CLAIM_TIMEOUT;
    sendAddressClaim(claimSock, N2K_ADDR_GLOBAL);
  }
  else
  {
    logError("Lost address %u conflict and not arbitrary-address-capable; cannot claim\n", address);
    address    = N2K_ADDR_NULL;
    claimState = CLAIM_FAILED;
    sendAddressClaim(claimSock, N2K_ADDR_GLOBAL);
  }
}

static void handleIsoRequest(int sock, uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len)
{
  uint32_t requested;

  (void) sock;
  (void) src;
  if (len < 3)
  {
    return;
  }
  requested = (uint32_t) data[0] | ((uint32_t) data[1] << 8) | ((uint32_t) data[2] << 16);

  if (requested == PGN_ISO_ADDRESS_CLAIM && (dst == address || dst == N2K_ADDR_GLOBAL))
  {
    if (claimState == CLAIM_CLAIMED || claimState == CLAIM_PENDING)
    {
      sendAddressClaim(claimSock, N2K_ADDR_GLOBAL);
    }
  }
}

static void tickAddressClaim(int sock, uint64_t now)
{
  (void) sock;
  if (claimState == CLAIM_SCANNING && now >= claimDeadline)
  {
    beginAddressClaim(claimSock);
    return;
  }
  if (claimState == CLAIM_PENDING && now >= claimDeadline)
  {
    claimState = CLAIM_CLAIMED;
    logInfo("Address %u claimed\n", address);
    if (heartbeatInterval > 0)
    {
      nextHeartbeat = now + (uint64_t) heartbeatInterval;
    }
  }
}

/*
 * NMEA 2000 Heartbeat, PGN 126993. Sent every heartbeatInterval ms once we
 * own an address, so other nodes know we are alive.
 */
static void sendHeartbeat(int sock)
{
  uint8_t  data[8];
  uint16_t offset = (uint16_t) (heartbeatInterval / 10); /* field resolution is 0.01s */

  data[0] = (uint8_t) offset;
  data[1] = (uint8_t) (offset >> 8);
  data[2] = heartbeatSeq;
  /* Controller 1 State = Error Active (0, normal), Controller 2 State = not
   * available (3), Equipment Status = Operational (0), reserved bits = 1. */
  data[3] = 0xCC;
  data[4] = 0xff;
  data[5] = 0xff;
  data[6] = 0xff;
  data[7] = 0xff;

  sendN2k(sock, 7, PGN_HEARTBEAT, (uint8_t) address, N2K_ADDR_GLOBAL, data, sizeof(data));

  heartbeatSeq = (heartbeatSeq >= 252) ? 0 : heartbeatSeq + 1;
}
