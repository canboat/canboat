/*

Read and write NMEA 2000 data on a Linux SocketCAN interface, behaving as
much like actisense-serial and ikonvert-serial as possible:

  - received CAN frames are reassembled into whole PGNs and written to stdout
    in canboat FAST format (one line per PGN, with a "# format=FAST" header);
  - canboat format lines read from stdin are split into CAN frames and sent
    to the bus;
  - the program is a real node on the bus: it claims an ISO source address
    (PGN 60928), backs off / re-claims on conflict, and answers ISO Requests
    (PGN 59904) for its address claim;
  - every few seconds it writes a synthetic "NMEA 2000 gateway: network
    status" PGN (262400) to stdout, the same shape the iKonvert emits from
    its $PDGY heartbeat, populated from the Linux kernel's per-interface CAN
    statistics (bus load, errors, device count, uptime, rejected TX).

Unlike the NGT-1 / iKonvert, a raw SocketCAN socket only ever delivers
individual <= 8 byte CAN frames, so the fast-packet reassembly that those
gateways do in hardware is done here in software. Which PGNs are fast-packet
is decided by range (see common.h) except for the mixed 0x1F000..0x1FFFF
range, for which fastpacket-table.h holds a per-PGN lookup generated from
canboat.json at build time.

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

#define _GNU_SOURCE
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "common.h"
#include "fastpacket-table.h"
#include "parse.h"

#define PGN_ISO_ACK (59392)
#define PGN_ISO_REQUEST (59904)
#define PGN_ISO_ADDRESS_CLAIM (60928)
#define PGN_GROUP_FUNCTION (126208)
#define PGN_PGN_LIST (126464)
#define PGN_PRODUCT_INFO (126996)

/* Product Information (PGN 126996) content. */
#define N2K_DB_VERSION (2100)      /* NMEA 2000 database version 2.100 (0.001 res) */
#define PRODUCT_CODE (1)
#define CERTIFICATION_LEVEL (0)    /* Level A */
#define LOAD_EQUIVALENCY (1)       /* 1 LEN = 50 mA */
#define MODEL_ID "socketcan-serial"

/* Group Function (PGN 126208) function codes and DURATION sentinels. */
#define GROUP_FUNCTION_REQUEST (0)
#define GROUP_FUNCTION_ACK (2)
#define TX_INTERVAL_NO_CHANGE (0xffffffffUL)
#define TX_INTERVAL_RESTORE_DEFAULT (0xfffffffeUL)

#define N2K_ADDR_GLOBAL (255)
#define N2K_ADDR_NULL (254) /* "cannot claim address" / no source */
#define N2K_MAX_ADDR (253)  /* highest claimable address */

#define ADDRESS_CLAIM_TIMEOUT (250)  /* ms to wait for a contender before we own the address */
#define ADDRESS_SCAN_TIMEOUT (1000)  /* ms to listen for existing claims before picking an address */

#define PGN_HEARTBEAT (126993)
#define HEARTBEAT_DEFAULT_INTERVAL (60000) /* ms, the NMEA 2000 default heartbeat rate */

/* Synthetic "NMEA 2000 gateway: network status" PGN (IKONVERT_BEM = 262400).
 * Like ikonvert-serial's $PDGY heartbeat, this is written to stdout only — it
 * is a BEM-range PGN by design and never goes on the wire. We synthesize the
 * same 15-byte shape from the Linux kernel's per-interface CAN statistics so a
 * downstream consumer sees a uniform per-gateway status record. */
#define PGN_NETWORK_STATUS (IKONVERT_BEM)
#define NETWORK_STATUS_INTERVAL_MS (5000) /* emit cadence; 5s is plenty for status */

/* CAN bus-load estimation from /sys/class/net/<iface>/statistics byte counters.
 * Those count data bytes only, so we add the per-frame protocol overhead and a
 * flat bit-stuffing inflation to approximate the true on-wire bit count. */
#define FALLBACK_BITRATE_BPS (250000) /* NMEA 2000 is fixed at 250 kbit/s */
#define CAN_EFF_OVERHEAD_BITS (67)    /* SOF+arb+ctrl+CRC+ACK+EOF+IFS, 29-bit frame */
#define CAN_STUFFING_NUMER (120)      /* ~20% average bit-stuffing inflation */
#define CAN_STUFFING_DENOM (100)

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
/* Default System Instance to 15 (max). Real sensors leave this at 0, so
 * a higher value pushes our NAME up in the ISO 11783-5 lower-NAME-wins
 * arbitration order — we yield to any well-behaved device on the bus
 * rather than steal addresses from real hardware. Override with -si. */
static unsigned int    systemInstance   = 15;
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

/* Network status (synthetic PGN 262400): emitted to stdout every
 * NETWORK_STATUS_INTERVAL_MS, populated from kernel CAN counters. */
static const char *canIface          = NULL; /* interface name, for /sys reads */
static uint64_t    startTime         = 0;     /* ms; for the uptime field */
static uint64_t    nextNetworkStatus = 0;     /* ms; when to emit next */
static uint32_t    busBitrate        = FALLBACK_BITRATE_BPS; /* bits/s, read once */
static bool        seenAddr[256];             /* distinct source addresses seen */

/* One read of the four kernel counters needed for the bus-load delta. The
 * sysfs counters are 32-bit here; deltas use modular subtraction so a counter
 * wrap is a small delta rather than a phantom 4 GiB burst. */
typedef struct
{
  uint32_t rxBytes;
  uint32_t txBytes;
  uint32_t rxPackets;
  uint32_t txPackets;
  uint64_t when; /* ms */
} LoadSample;
static LoadSample prevLoadSample;
static bool       havePrevLoadSample = false;

/* Outbound buffer drained one frame per select() POLLOUT, so a burst of
 * frames (fast-packet PGN, claim flurry, …) never blocks the main loop
 * waiting for the kernel CAN qdisc to drain. The queue holds about 1s of
 * bus time at 250 kbit/s with default txqueuelen, which is enough to
 * survive realistic write bursts and still keep RX / claim timers
 * responsive. */
#define TX_BUFFER_CAPACITY 1024
static struct can_frame txBuffer[TX_BUFFER_CAPACITY];
static size_t           txHead       = 0; /* oldest unsent frame */
static size_t           txTail       = 0; /* next free slot */
static size_t           txCount      = 0;
static size_t           txOverflowed = 0;

/* PGNs we originate / consume, reported via PGN 126464 on request. */
static const uint32_t txPgnList[] = {PGN_ISO_ACK, PGN_ISO_REQUEST, PGN_ISO_ADDRESS_CLAIM, PGN_GROUP_FUNCTION,
                                     PGN_PGN_LIST, PGN_HEARTBEAT, PGN_PRODUCT_INFO};
static const uint32_t rxPgnList[] = {PGN_ISO_REQUEST, PGN_ISO_ADDRESS_CLAIM, PGN_GROUP_FUNCTION};

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

/* Per-canId outbound fast-packet sequence counter. The upper 3 bits of
 * every fast-packet `frame[0]` carry this value; a strict receiver
 * uses it to distinguish back-to-back instances of the same PGN from
 * continuations of an in-flight reassembly. Without it consecutive
 * Product-Info responses (for example) all look like seq=0 first
 * frames and can be folded into one corrupted reassembly. The number
 * of distinct outbound fast-packet PGNs we emit is small (heartbeat
 * Group-Function ACK, PGN List, Product Info), so a 16-slot linear-
 * scan table is plenty. */
#define TX_FAST_SEQ_SLOTS 16
typedef struct
{
  bool     used;
  uint32_t canId;
  uint8_t  seq; /* next value to emit, 0..7 */
} TxFastSeq;
static TxFastSeq txFastSeq[TX_FAST_SEQ_SLOTS];

static uint8_t getNextFastSeq(uint32_t canId)
{
  TxFastSeq *first_free = NULL;
  for (size_t i = 0; i < TX_FAST_SEQ_SLOTS; i++)
  {
    TxFastSeq *slot = &txFastSeq[i];
    if (slot->used && slot->canId == canId)
    {
      uint8_t s = slot->seq;
      slot->seq = (uint8_t) ((s + 1) & 0x07);
      return s;
    }
    if (!slot->used && first_free == NULL)
    {
      first_free = slot;
    }
  }
  if (first_free)
  {
    first_free->used  = true;
    first_free->canId = canId;
    first_free->seq   = 1; /* we hand out 0 below, next will be 1 */
    return 0;
  }
  /* Table full — should never happen with only a handful of outbound
   * fast-packet PGNs, but fall back to seq 0 so we still emit a valid
   * frame rather than crashing. */
  logError("TxFastSeq table full, falling back to seq=0 for canId %08x\n", canId);
  return 0;
}

static int  openCanDevice(const char *device, int *canSocket);
static int  readCan(int sock);
static void handleFrame(uint32_t canId, const uint8_t *data, uint8_t len, uint64_t when);
static void emitMessage(uint64_t when, uint8_t prio, uint32_t pgn, uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len);
static bool isFastPacket(uint32_t pgn);
static void parseAndWriteIn(int sock, const char *cmd);
static void sendN2k(int sock, uint8_t prio, uint32_t pgn, uint8_t src, uint8_t dst, const uint8_t *data, size_t len, bool echo);
static void sendCanFrame(int sock, uint32_t canId, const uint8_t *data, uint8_t len);
static bool txDrainOne(int sock);
static bool readIn(void);
static bool getInMsg(char *msg, size_t len);

/* Address claim */
static void     buildDeviceName(void);
static void     startAddressClaim(int sock);
static void     beginAddressClaim(int sock);
static void     sendAddressClaim(int sock, uint8_t dst);
static void     sendIsoRequest(int sock, uint8_t src, uint8_t dst, uint32_t pgn);
static void     handleAddressClaim(int sock, uint8_t src, const uint8_t *data, uint8_t len);
static void     handleIsoRequest(uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len);
static void     handleGroupFunction(uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len);
static void     tickAddressClaim(int sock, uint64_t now);
static int      pickFreeAddress(void);
static uint64_t nameFromBytes(const uint8_t *data);
static void     sendHeartbeat(int sock);
static void     sendProductInfo(int sock);
static void     sendPgnList(int sock, uint8_t dst);
static void     sendIsoAck(int sock, uint8_t dst, uint8_t control, uint32_t pgn);
static void     sendAckGroupFunction(int sock, uint8_t dst, uint32_t pgn, uint8_t pgnError, uint8_t paramError);

/* Network status (synthetic PGN 262400) */
static void     sendNetworkStatus(void);
static bool     readSysfsCounter(const char *iface, const char *name, uint32_t *out);
static uint32_t readSysfsBitrate(const char *iface);
static bool     readLoadSample(const char *iface, uint64_t now, LoadSample *out);
static bool     computeLoadPct(const LoadSample *prev, const LoadSample *curr, uint32_t bitrate, uint8_t *out);

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
    else if (strcasecmp(argv[1], "-si") == 0 && argc > 2)
    {
      argc--, argv++;
      systemInstance = (unsigned int) strtoul(argv[1], 0, 10) & 0x0f;
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
            "Usage: %s [-w] [-r] [-p] [-v] [-d] [-n] [-t <n>] [-a <addr>] [-u <n>] [-m <n>] [-si <n>] [-hb <ms>] <can-device>\n"
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
            "  -si <n>   ISO NAME System Instance, 0..15 (default 15 = yield to other devices)\n"
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

  /* Network-status emission state: read the (fixed) bitrate once and arm
   * the first emission one interval out. Disabled in writeonly mode, which
   * suppresses all stdout output. */
  canIface          = device;
  startTime         = getNow();
  busBitrate        = readSysfsBitrate(device);
  nextNetworkStatus = startTime + NETWORK_STATUS_INTERVAL_MS;

  uint64_t lastFrame = getNow();

  for (;;)
  {
    fd_set         rfds, wfds;
    struct timeval tv;
    uint64_t       now = getNow();
    int            r;
    int            maxfd = sock;

    FD_ZERO(&rfds);
    FD_ZERO(&wfds);
    FD_SET(sock, &rfds);
    if (txCount > 0)
    {
      FD_SET(sock, &wfds);
    }
    if (!readonly)
    {
      FD_SET(STDIN_FILENO, &rfds);
      if (STDIN_FILENO > maxfd)
      {
        maxfd = STDIN_FILENO;
      }
    }

    /* Wake at the soonest of: the claim deadline, the next heartbeat, or a
     * 1s poll, so the timers advance even on an otherwise silent bus.
     * When the TX buffer is non-empty, also clamp to a short timeout as a
     * safety net in case POLLOUT lags qdisc availability on some kernels. */
    uint64_t waitMs = 1000;
    if ((claimState == CLAIM_PENDING || claimState == CLAIM_SCANNING) && claimDeadline > now)
    {
      waitMs = claimDeadline - now;
    }
    else if (claimState == CLAIM_CLAIMED && heartbeatInterval > 0)
    {
      waitMs = (nextHeartbeat > now) ? nextHeartbeat - now : 0;
    }
    if (!writeonly)
    {
      uint64_t nsWait = (nextNetworkStatus > now) ? nextNetworkStatus - now : 0;
      if (nsWait < waitMs)
      {
        waitMs = nsWait;
      }
    }
    if (txCount > 0 && waitMs > 5)
    {
      waitMs = 5;
    }
    tv.tv_sec  = waitMs / 1000;
    tv.tv_usec = (waitMs % 1000) * 1000;

    r = select(maxfd + 1, &rfds, &wfds, NULL, &tv);
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

    /* Drain one frame per writability wakeup (or per safety-net timeout)
     * so a single producer can't monopolise the loop; RX, stdin and the
     * claim timers stay responsive across long fast-packet bursts. */
    if (txCount > 0)
    {
      txDrainOne(sock);
    }

    if (FD_ISSET(sock, &rfds))
    {
      lastFrame = getNow();
      if (readCan(sock) < 0)
      {
        break;
      }
    }
    if (!readonly && FD_ISSET(STDIN_FILENO, &rfds))
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

    if (!writeonly && getNow() >= nextNetworkStatus)
    {
      sendNetworkStatus();
      nextNetworkStatus = getNow() + NETWORK_STATUS_INTERVAL_MS;
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

  /* Non-blocking writes: a full netdev qdisc must not stall RX or the
   * claim timers. The main loop buffers TX frames and drains them via
   * POLLOUT instead of busy-waiting in write(). */
  int flags = fcntl(sock, F_GETFL, 0);
  if (flags < 0 || fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0)
  {
    logError("setting CAN socket non-blocking: %s\n", strerror(errno));
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
    if ((frame.can_id & CAN_EFF_FLAG) == 0)
    {
      continue; /* CAN 1.0 / 2.0A standard (11-bit) frame, not NMEA 2000 */
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

  /* Remember every source we hear from, for the network-status device
   * count (independent of the address-claim table, which undercounts). */
  seenAddr[src & 0xff] = true;

  /* Act on the single-frame messages that drive the address-claim
   * protocol, but still emit them to stdout like any other frame. */
  if (claimState != CLAIM_DISABLED)
  {
    if (pgn == PGN_ISO_ADDRESS_CLAIM)
    {
      handleAddressClaim(0 /* uses claimSock */, (uint8_t) src, data, len);
    }
    else if (pgn == PGN_ISO_REQUEST)
    {
      handleIsoRequest((uint8_t) src, (uint8_t) dst, data, len);
    }
  }

  if (!isFastPacket(pgn))
  {
    if (!writeonly)
    {
      emitMessage(when, (uint8_t) prio, pgn, (uint8_t) src, (uint8_t) dst, data, len);
    }
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
    if (!writeonly)
    {
      emitMessage(fp->when, (uint8_t) prio, pgn, (uint8_t) src, (uint8_t) dst, fp->data, fp->size);
    }
    if (claimState != CLAIM_DISABLED && pgn == PGN_GROUP_FUNCTION)
    {
      handleGroupFunction((uint8_t) src, (uint8_t) dst, fp->data, fp->size);
    }
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

  sendN2k(sock, m.prio, m.pgn, (uint8_t) address, m.dst, m.data, m.len, false);
}

/*
 * Send an NMEA 2000 message, splitting into a fast packet when needed.
 * (Same wire layout as socketcan-writer.c.)
 */
static void sendN2k(int sock, uint8_t prio, uint32_t pgn, uint8_t src, uint8_t dst, const uint8_t *data, size_t len, bool echo)
{
  uint32_t canId = getCanIdFromISO11783Bits(prio, pgn, src, dst);

  /* Single-frame PGNs go out as one frame; fast-packet PGNs are always
   * fast-framed even when short, since receivers key on the PGN type. */
  if (len <= 8 && !isFastPacket(pgn))
  {
    sendCanFrame(sock, canId, data, (uint8_t) len);
  }
  else
  {
    size_t  remaining = len;
    uint8_t index     = 0;
    uint8_t frame[8];
    /* Per ISO 11783-3: every fast-packet CAN frame is exactly 8 bytes;
     * the last chunk is padded with 0xff. The first byte is
     * `(seq << 5) | index`, with `seq` incrementing per-canId per
     * instance so consecutive first frames are distinguishable from
     * continuations. */
    uint8_t seq = getNextFastSeq(canId);

    while (remaining > 0)
    {
      size_t chunk;
      memset(frame, 0xff, sizeof(frame));
      frame[0] = (uint8_t) ((seq << 5) | (index & 0x1f));
      if (index == 0)
      {
        frame[1] = (uint8_t) len;
        chunk    = CB_MIN(FASTPACKET_BUCKET_0_SIZE, remaining);
        memcpy(frame + FASTPACKET_BUCKET_0_OFFSET, data, chunk);
      }
      else
      {
        chunk = CB_MIN(FASTPACKET_BUCKET_N_SIZE, remaining);
        memcpy(frame + FASTPACKET_BUCKET_N_OFFSET, data + (len - remaining), chunk);
      }
      sendCanFrame(sock, canId, frame, (uint8_t) sizeof(frame));
      remaining -= chunk;
      index++;
    }
  }

  /* Echo our own generated PGNs to stdout too, so a downstream consumer
   * sees a complete picture of the bus including this node. The stdin
   * bridge passes echo=false; its -p passthru handles echoing instead. */
  if (echo && !writeonly)
  {
    emitMessage(getNow(), prio, pgn, src, dst, data, (uint8_t) len);
  }
}

/*
 * Enqueue a CAN frame for asynchronous transmission. The main loop drains
 * the buffer one frame at a time when select() reports POLLOUT on the CAN
 * socket. We never write() inline because the netdev qdisc is shallow
 * (txqueuelen often 10) and a single fast-packet PGN can easily exceed
 * it — a blocking retry loop would stall RX and the claim timers for
 * tens to hundreds of milliseconds per burst.
 */
static void sendCanFrame(int sock, uint32_t canId, const uint8_t *data, uint8_t len)
{
  (void) sock; /* the socket is drained by the main loop, not used here */
  if (txCount == TX_BUFFER_CAPACITY)
  {
    /* Rate-limit the warning so a sustained overrun doesn't flood stderr. */
    if (txOverflowed == 0)
    {
      logError("CAN TX buffer full (%d frames), dropping outbound frame\n", TX_BUFFER_CAPACITY);
    }
    txOverflowed++;
    return;
  }
  struct can_frame *slot = &txBuffer[txTail];
  memset(slot, 0, sizeof(*slot));
  slot->can_id  = canId | CAN_EFF_FLAG;
  slot->can_dlc = len;
  memcpy(slot->data, data, len);
  txTail = (txTail + 1) % TX_BUFFER_CAPACITY;
  txCount++;
}

/*
 * Drain one frame from the TX buffer if the kernel has room. Returns true
 * iff a frame was actually written; false on empty buffer or backpressure.
 */
static bool txDrainOne(int sock)
{
  if (txCount == 0)
  {
    return false;
  }
  const struct can_frame *f = &txBuffer[txHead];
  ssize_t                 n = write(sock, f, sizeof(*f));
  if (n == (ssize_t) sizeof(*f))
  {
    txHead = (txHead + 1) % TX_BUFFER_CAPACITY;
    txCount--;
    if (txOverflowed > 0 && txCount < TX_BUFFER_CAPACITY / 2)
    {
      logInfo("CAN TX buffer recovered (%zu frames had been dropped)\n", txOverflowed);
      txOverflowed = 0;
    }
    return true;
  }
  if (n < 0 && (errno == EAGAIN || errno == ENOBUFS || errno == EINTR))
  {
    /* Backpressure — leave the frame queued; the next POLLOUT-driven
     * wakeup or the safety-net select() timeout will retry. */
    return false;
  }
  /* Hard error — drop the frame so the buffer can't get stuck. */
  logError("write to CAN: %s (dropping frame)\n", strerror(errno));
  txHead = (txHead + 1) % TX_BUFFER_CAPACITY;
  txCount--;
  return false;
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
  sendN2k(sock, 6, PGN_ISO_REQUEST, src, dst, data, sizeof(data), true);
}

static void sendAddressClaim(int sock, uint8_t dst)
{
  uint8_t data[8];
  for (int i = 0; i < 8; i++)
  {
    data[i] = (uint8_t) (deviceName >> (8 * i));
  }
  sendN2k(sock, 6, PGN_ISO_ADDRESS_CLAIM, (uint8_t) address, dst, data, sizeof(data), true);
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

static void handleIsoRequest(uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len)
{
  uint32_t requested;
  bool     addressed;

  if (len < 3)
  {
    return;
  }
  requested = (uint32_t) data[0] | ((uint32_t) data[1] << 8) | ((uint32_t) data[2] << 16);
  addressed = (dst == address);
  if (!addressed && dst != N2K_ADDR_GLOBAL)
  {
    return; /* request is for some other node */
  }

  /* The address claim must be answerable even before we are fully claimed. */
  if (requested == PGN_ISO_ADDRESS_CLAIM)
  {
    if (claimState == CLAIM_CLAIMED || claimState == CLAIM_PENDING)
    {
      sendAddressClaim(claimSock, N2K_ADDR_GLOBAL);
    }
    return;
  }

  /* Everything else needs a claimed address to answer from. */
  if (claimState != CLAIM_CLAIMED)
  {
    return;
  }

  switch (requested)
  {
    case PGN_PRODUCT_INFO:
      sendProductInfo(claimSock);
      break;
    case PGN_PGN_LIST:
      sendPgnList(claimSock, src);
      break;
    case PGN_HEARTBEAT:
      sendHeartbeat(claimSock);
      break;
    default:
      /* ISO 11783-3: NAK an addressed request for a PGN we do not send;
       * silently ignore an unsupported global request. */
      if (addressed)
      {
        sendIsoAck(claimSock, src, 1 /* NAK */, requested);
      }
      break;
  }
}

/*
 * NMEA Request Group Function (PGN 126208, function 0). We only act on a
 * request that targets our Heartbeat (PGN 126993): it sets the heartbeat
 * transmission interval (or disables it), then we reply with an Acknowledge
 * Group Function. "whatever of the two is normal": the Request form is what
 * configures a PGN's transmission rate.
 */
static void handleGroupFunction(uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len)
{
  uint32_t targetPgn;
  uint32_t interval;
  uint8_t  paramError = 0; /* 0 = Acknowledge */

  (void) dst;
  if (len < 8 || data[0] != GROUP_FUNCTION_REQUEST)
  {
    return;
  }
  if (claimState != CLAIM_CLAIMED)
  {
    return;
  }
  targetPgn = (uint32_t) data[1] | ((uint32_t) data[2] << 8) | ((uint32_t) data[3] << 16);
  if (targetPgn != PGN_HEARTBEAT)
  {
    return; /* not a PGN whose rate we control */
  }

  /* Transmission interval: 32-bit, 0.001s resolution => milliseconds. */
  interval = (uint32_t) data[4] | ((uint32_t) data[5] << 8) | ((uint32_t) data[6] << 16) | ((uint32_t) data[7] << 24);

  if (interval == TX_INTERVAL_NO_CHANGE)
  {
    /* leave the interval as-is */
  }
  else if (interval == TX_INTERVAL_RESTORE_DEFAULT)
  {
    heartbeatInterval = HEARTBEAT_DEFAULT_INTERVAL;
    nextHeartbeat     = getNow() + (uint64_t) heartbeatInterval;
    logInfo("Heartbeat interval restored to default %d ms\n", HEARTBEAT_DEFAULT_INTERVAL);
  }
  else if (interval == 0)
  {
    heartbeatInterval = 0; /* disable */
    logInfo("Heartbeat disabled by group function\n");
  }
  else if (interval >= 1000 && interval <= 60000)
  {
    heartbeatInterval = (long) interval;
    nextHeartbeat     = getNow() + (uint64_t) heartbeatInterval;
    logInfo("Heartbeat interval set to %u ms by group function\n", interval);
  }
  else
  {
    paramError = 2; /* transmission interval too low/high */
    logError("Requested heartbeat interval %u ms out of range\n", interval);
  }

  sendAckGroupFunction(claimSock, src, PGN_HEARTBEAT, 0 /* PGN ok */, paramError);
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
    sendProductInfo(claimSock); /* announce ourselves once on the bus */
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

  sendN2k(sock, 7, PGN_HEARTBEAT, (uint8_t) address, N2K_ADDR_GLOBAL, data, sizeof(data), true);

  heartbeatSeq = (heartbeatSeq >= 252) ? 0 : heartbeatSeq + 1;
}

/* Copy a C string into a fixed-width NUL-padded STRING_FIX field. */
static void putStringFix(uint8_t *dst, size_t width, const char *s)
{
  size_t n = strlen(s);
  if (n > width)
  {
    n = width;
  }
  memset(dst, 0, width);
  memcpy(dst, s, n);
}

/*
 * Product Information, PGN 126996. Sent once when we claim an address and in
 * reply to an ISO Request, so the node is identifiable on the network.
 */
static void sendProductInfo(int sock)
{
  static uint64_t lastSent = 0;
  uint64_t        now = getNow();
  uint8_t         data[134];
  char            serial[32];

  /* 126996 is broadcast (PDU2), so a single reply answers every requester.
   * Collapse the discovery burst that several nodes make when we appear,
   * which would otherwise overflow the small CAN TX queue. */
  if (now - lastSent < 1000)
  {
    return;
  }
  lastSent = now;

  memset(data, 0, sizeof(data));
  data[0] = (uint8_t) N2K_DB_VERSION;
  data[1] = (uint8_t) (N2K_DB_VERSION >> 8);
  data[2] = (uint8_t) PRODUCT_CODE;
  data[3] = (uint8_t) (PRODUCT_CODE >> 8);
  putStringFix(data + 4, 32, MODEL_ID);
  putStringFix(data + 36, 32, VERSION); /* software version code */
  putStringFix(data + 68, 32, "");      /* model version */
  snprintf(serial, sizeof(serial), "%u", uniqueNumber);
  putStringFix(data + 100, 32, serial);
  data[132] = CERTIFICATION_LEVEL;
  data[133] = LOAD_EQUIVALENCY;

  sendN2k(sock, 6, PGN_PRODUCT_INFO, (uint8_t) address, N2K_ADDR_GLOBAL, data, sizeof(data), true);
}

/*
 * PGN List (Transmit and Receive), PGN 126464. Sent in reply to an ISO
 * Request for 126464: one message for the transmit list, one for receive.
 */
static void sendOnePgnList(int sock, uint8_t dst, uint8_t functionCode, const uint32_t *list, size_t count)
{
  uint8_t data[1 + 7 * 3];
  size_t  n = 0;

  data[n++] = functionCode;
  for (size_t i = 0; i < count && n + 3 <= sizeof(data); i++)
  {
    data[n++] = (uint8_t) list[i];
    data[n++] = (uint8_t) (list[i] >> 8);
    data[n++] = (uint8_t) (list[i] >> 16);
  }
  sendN2k(sock, 6, PGN_PGN_LIST, (uint8_t) address, dst, data, n, true);
}

static void sendPgnList(int sock, uint8_t dst)
{
  sendOnePgnList(sock, dst, 0 /* Transmit PGN list */, txPgnList, ARRAY_SIZE(txPgnList));
  sendOnePgnList(sock, dst, 1 /* Receive PGN list */, rxPgnList, ARRAY_SIZE(rxPgnList));
}

/* ISO Acknowledgement, PGN 59392. */
static void sendIsoAck(int sock, uint8_t dst, uint8_t control, uint32_t pgn)
{
  uint8_t data[8];

  data[0] = control;
  data[1] = 0xff; /* Group Function: not applicable */
  data[2] = 0xff; /* Reserved */
  data[3] = 0xff;
  data[4] = 0xff;
  data[5] = (uint8_t) pgn;
  data[6] = (uint8_t) (pgn >> 8);
  data[7] = (uint8_t) (pgn >> 16);

  sendN2k(sock, 6, PGN_ISO_ACK, (uint8_t) address, dst, data, sizeof(data), true);
}

/* Acknowledge Group Function, PGN 126208 function 2. */
static void sendAckGroupFunction(int sock, uint8_t dst, uint32_t pgn, uint8_t pgnError, uint8_t paramError)
{
  uint8_t data[6];

  data[0] = GROUP_FUNCTION_ACK;
  data[1] = (uint8_t) pgn;
  data[2] = (uint8_t) (pgn >> 8);
  data[3] = (uint8_t) (pgn >> 16);
  data[4] = (uint8_t) ((pgnError & 0x0f) | ((paramError & 0x0f) << 4));
  data[5] = 0; /* Number of parameters */

  sendN2k(sock, 6, PGN_GROUP_FUNCTION, (uint8_t) address, dst, data, sizeof(data), true);
}

/*
 * --- NMEA 2000 gateway: network status (synthetic PGN 262400) -----------
 *
 * Read a single unsigned decimal counter from
 * /sys/class/net/<iface>/statistics/<name>. Returns false (leaving *out
 * untouched) when the file is missing or unparseable, so each field
 * degrades to the canboat "no data" sentinel rather than reporting a wrong
 * value.
 */
static bool readSysfsCounter(const char *iface, const char *name, uint32_t *out)
{
  char          path[128];
  FILE         *f;
  unsigned long v;
  int           n;

  snprintf(path, sizeof(path), "/sys/class/net/%s/statistics/%s", iface, name);
  if ((f = fopen(path, "r")) == NULL)
  {
    return false;
  }
  n = fscanf(f, "%lu", &v);
  fclose(f);
  if (n != 1)
  {
    return false;
  }
  *out = (uint32_t) v;
  return true;
}

/*
 * Read the CAN bitrate (bits/s) from /sys/class/net/<iface>/can_bittiming/
 * bitrate, falling back to NMEA 2000's fixed 250 kbit/s when the file is
 * missing, unreadable or zero (e.g. the kernel has not filled in bittiming
 * yet, or this is a virtual interface).
 */
static uint32_t readSysfsBitrate(const char *iface)
{
  char          path[128];
  FILE         *f;
  unsigned long v;
  int           n;

  snprintf(path, sizeof(path), "/sys/class/net/%s/can_bittiming/bitrate", iface);
  if ((f = fopen(path, "r")) == NULL)
  {
    return FALLBACK_BITRATE_BPS;
  }
  n = fscanf(f, "%lu", &v);
  fclose(f);
  if (n != 1 || v == 0)
  {
    return FALLBACK_BITRATE_BPS;
  }
  return (uint32_t) v;
}

/*
 * Take one snapshot of the four kernel counters needed for the bus-load
 * delta. Returns false if any are unreadable, in which case the caller
 * reports load as "no data".
 */
static bool readLoadSample(const char *iface, uint64_t now, LoadSample *out)
{
  if (!readSysfsCounter(iface, "rx_bytes", &out->rxBytes) || !readSysfsCounter(iface, "tx_bytes", &out->txBytes)
      || !readSysfsCounter(iface, "rx_packets", &out->rxPackets) || !readSysfsCounter(iface, "tx_packets", &out->txPackets))
  {
    return false;
  }
  out->when = now;
  return true;
}

/*
 * CAN bus load percentage from two counter samples and the bus bitrate.
 * The sysfs byte counters cover data bytes only, so we add the per-frame
 * protocol overhead (CAN_EFF_OVERHEAD_BITS) and a flat bit-stuffing
 * inflation to approximate the true on-wire bit count:
 *
 *   bits_on_wire = (d_bytes * 8 + d_packets * 67) * 1.20
 *   pct          = bits_on_wire * 100 / (bitrate * dt_seconds)
 *
 * Returns false (load = "no data") when there is no usable interval.
 * Counter deltas use modular uint32 subtraction so a counter wrap shows
 * up as a small delta rather than a phantom multi-GiB burst.
 */
static bool computeLoadPct(const LoadSample *prev, const LoadSample *curr, uint32_t bitrate, uint8_t *out)
{
  uint64_t dtMs = curr->when - prev->when;
  uint64_t dBytes, dPackets, bitsRaw, bitsOnWire, denom, pct;

  if (dtMs == 0 || bitrate == 0)
  {
    return false;
  }
  dBytes   = (uint64_t) (uint32_t) (curr->rxBytes - prev->rxBytes) + (uint64_t) (uint32_t) (curr->txBytes - prev->txBytes);
  dPackets = (uint64_t) (uint32_t) (curr->rxPackets - prev->rxPackets) + (uint64_t) (uint32_t) (curr->txPackets - prev->txPackets);

  bitsRaw    = dBytes * 8 + dPackets * CAN_EFF_OVERHEAD_BITS;
  bitsOnWire = bitsRaw * CAN_STUFFING_NUMER / CAN_STUFFING_DENOM;
  denom      = (uint64_t) bitrate * dtMs;
  if (denom == 0)
  {
    return false;
  }
  /* pct = bits * 100 / (bitrate * dt_s) = bits * 100000 / (bitrate * dt_ms) */
  pct  = bitsOnWire * 100000 / denom;
  *out = (uint8_t) (pct > 100 ? 100 : pct);
  return true;
}

/*
 * Emit the synthetic "NMEA 2000 gateway: network status" PGN (262400) to
 * stdout. Same 15-byte layout as ikonvert-serial's $PDGY heartbeat:
 *   [0]     CAN network load (%)
 *   [1..4]  Errors (u32 LE)
 *   [5]     Device count
 *   [6..9]  Uptime (u32 LE, seconds)
 *   [10]    Gateway address
 *   [11..14] Rejected TX requests (u32 LE)
 * Fields we cannot measure are left at the 0xff / 0xffffffff "no data"
 * sentinel. This is a BEM-range PGN: it is written to stdout only, never
 * sent on the bus.
 */
static void sendNetworkStatus(void)
{
  uint64_t   now = getNow();
  uint8_t    data[15];
  uint8_t    src = 0;
  uint8_t    load;
  uint32_t   counter;
  LoadSample curr;
  unsigned   count = 0;

  memset(data, 0xff, sizeof(data));

  /* [0] CAN network load: delta-sampled against the previous emission.
   * The first emission has no baseline, so load stays at the sentinel. */
  if (readLoadSample(canIface, now, &curr))
  {
    if (havePrevLoadSample && computeLoadPct(&prevLoadSample, &curr, busBitrate, &load))
    {
      data[0] = load;
    }
    prevLoadSample     = curr;
    havePrevLoadSample = true;
  }

  /* [1..4] Errors: kernel RX error counter. */
  if (readSysfsCounter(canIface, "rx_errors", &counter))
  {
    data[1] = (uint8_t) (counter >> 0);
    data[2] = (uint8_t) (counter >> 8);
    data[3] = (uint8_t) (counter >> 16);
    data[4] = (uint8_t) (counter >> 24);
  }

  /* [5] Device count: distinct source addresses seen on the bus. This is
   * strictly more than the address-claim table, which only updates on PGN
   * 60928 — many real devices never re-announce. */
  for (int a = 0; a < 256; a++)
  {
    if (seenAddr[a])
    {
      count++;
    }
  }
  data[5] = (uint8_t) (count > 255 ? 255 : count);

  /* [6..9] Uptime in whole seconds since startup. */
  {
    uint32_t uptime = (uint32_t) ((now - startTime) / 1000);
    data[6]         = (uint8_t) (uptime >> 0);
    data[7]         = (uint8_t) (uptime >> 8);
    data[8]         = (uint8_t) (uptime >> 16);
    data[9]         = (uint8_t) (uptime >> 24);
  }

  /* [10] Gateway address: our claimed CAN address, if we own one. Also
   * used as the emitted message's source so downstream tools attribute
   * the status to this gateway rather than bucketing it at src=0. */
  if (claimState == CLAIM_CLAIMED)
  {
    data[10] = (uint8_t) address;
    src      = (uint8_t) address;
  }

  /* [11..14] Rejected TX requests: kernel TX-dropped counter. */
  if (readSysfsCounter(canIface, "tx_dropped", &counter))
  {
    data[11] = (uint8_t) (counter >> 0);
    data[12] = (uint8_t) (counter >> 8);
    data[13] = (uint8_t) (counter >> 16);
    data[14] = (uint8_t) (counter >> 24);
  }

  emitMessage(now, 7, PGN_NETWORK_STATUS, src, N2K_ADDR_GLOBAL, data, sizeof(data));
}
