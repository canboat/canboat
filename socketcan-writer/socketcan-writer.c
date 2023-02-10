/*

Reads raw n2k ASCII data from stdin and writes it to a Linux SocketCAN device (e.g. can0).

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
#define _GNU_SOURCE
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define GLOBALS
#include "common.h"
#include "parse.h"

static int    openCanDevice(char *device, int *socket);
static void   writeRawPGNToCanSocket(RawMessage *msg, int socket);
static void   sendCanFrame(struct can_frame *frame, int socket);
static void   sendN2kFastPacket(RawMessage *msg, struct can_frame *frame, int socket);
unsigned long time_diff(struct timeval x, struct timeval y, char *timestamp);

int main(int argc, char **argv)
{
  FILE          *file = stdin;
  char           msg[2000];
  char          *milliSecond;
  int            socket;
  struct timeval frameTime;
  struct timeval prevFrameTime;
  struct tm      ctime;
  unsigned long  usWait;

  setProgName(argv[0]);
  if (argc != 2)
  {
    puts("Usage: socketcan-writer <can-device>");
    exit(1);
  }

  if (openCanDevice(argv[1], &socket))
  {
    exit(1);
  }

  prevFrameTime.tv_sec  = 0;
  prevFrameTime.tv_usec = 0;

  while (fgets(msg, sizeof(msg) - 1, file))
  {
    RawMessage m;
    if (parseRawFormatFast(msg, &m, false))
    {
      continue; // Parsing failed -> skip the line
    }
    if (strlen(m.timestamp) >= 19)
    {
      m.timestamp[10] = 'T'; // to support 'T', '-' and ' ' separators
      memset(&ctime, 0, sizeof(struct tm));
      milliSecond = strptime(m.timestamp, "%Y-%m-%dT%H:%M:%S", &ctime);
      if ((milliSecond != NULL) && (milliSecond - m.timestamp >= 19)) // convert in tm struct => OK
      {
        frameTime.tv_sec  = mktime(&ctime);
        frameTime.tv_usec = (sscanf(milliSecond, ".%3ld", &frameTime.tv_usec) == 1) ? frameTime.tv_usec * 1000 : 0;
        usWait
            = ((prevFrameTime.tv_sec == 0) && (prevFrameTime.tv_usec == 0)) ? 0 : time_diff(prevFrameTime, frameTime, m.timestamp);
        prevFrameTime = frameTime;
      }
      else // convert in tm struct failed
      {
        usWait = 0;
      }
    }
    else // bad timestamp format YYYY-mm-dd[T|-| ]HH:MM:SS[.xxx] & min length 19 chrs
    {
      usWait = 0;
    }
    if (usWait > 0)
    {
      usleep(usWait);
    }
    writeRawPGNToCanSocket(&m, socket);
  }

  close(socket);
  exit(0);
}

/*
  Opens SocketCAN socket to given device, see: https://www.kernel.org/doc/Documentation/networking/can.txt
*/
static int openCanDevice(char *device, int *canSocket)
{
  struct sockaddr_can addr;
  struct ifreq        ifr;

  if (strcmp(device, "stdout") == 0 || strcmp(device, "-") == 0)
  {
    *canSocket = STDOUT_FILENO;
    return 0;
  }

  if ((*canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("socket");
    return 1;
  }

  strncpy(ifr.ifr_name, device, IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex            = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex)
  {
    perror("if_nametoindex");
    return 1;
  }

  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(*canSocket, (struct sockaddr *) &addr, sizeof(addr)) < 0)
  {
    perror("bind");
    return 1;
  }

  return 0;
}

static void writeRawPGNToCanSocket(RawMessage *msg, int socket)
{
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));

  if (msg->pgn >= (1 << 18)) // PGNs can't have more than 18 bits, otherwise it overwrites priority bits
  {
    logError("Invalid PGN, too big (0x%x). Skipping.\n", msg->pgn);
    return;
  }

  frame.can_id = getCanIdFromISO11783Bits(msg->prio, msg->pgn, msg->src, msg->dst);

  if (msg->len <= 8) // 8 or less bytes of data -> PGN fits into a single CAN frame
  {
    frame.can_dlc = msg->len;
    memcpy(frame.data, msg->data, msg->len);
    sendCanFrame(&frame, socket);
  }
  else
  { // Send PGN as n2k fast packet (spans multiple CAN frames, but CAN ID is still same for each frame)
    sendN2kFastPacket(msg, &frame, socket);
  }
}

static void sendCanFrame(struct can_frame *frame, int socket)
{
  if (write(socket, frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
  {
    perror("write");
  }
}

/*
  See pgn.h for n2k fast packet data format
*/
static void sendN2kFastPacket(RawMessage *msg, struct can_frame *frame, int socket)
{
  int index              = 0;
  int remainingDataBytes = msg->len;
  while (remainingDataBytes > 0)
  {
    frame->data[0]
        = index; // fast packet index (increases by 1 for every CAN frame), 'order' (the 3 uppermost bits) is left as 0 for now

    if (index == 0) // 1st frame
    {
      frame->data[1] = msg->len;             // fast packet payload size
      memcpy(frame->data + 2, msg->data, 6); // 6 first data bytes
      frame->can_dlc = 8;
      remainingDataBytes -= 6;
    }
    else // further frames
    {
      if (remainingDataBytes > 7)
      {
        memcpy(frame->data + 1, msg->data + 6 + (index - 1) * 7, 7); // 7 next data bytes
        frame->can_dlc = 8;
        remainingDataBytes -= 7;
      }
      else
      {
        memcpy(frame->data + 1, msg->data + 6 + (index - 1) * 7, remainingDataBytes); // 7 next data bytes
        frame->can_dlc     = 1 + remainingDataBytes;
        remainingDataBytes = 0;
      }
    }
    sendCanFrame(frame, socket);
    index++;
  }
}

unsigned long time_diff(struct timeval x, struct timeval y, char *timestamp)
{
  double x_ms, y_ms, diff;

  x_ms = (double) x.tv_sec * 1000000 + (double) x.tv_usec;
  y_ms = (double) y.tv_sec * 1000000 + (double) y.tv_usec;

  diff = (double) y_ms - (double) x_ms;
  if (diff < 0.0)
  {
    logError("Timestamp back in time at %s\n", timestamp);
    return 0;
  }

  return (unsigned long) diff;
}
