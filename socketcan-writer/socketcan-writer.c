/*

Reads raw n2k ASCII data from stdin and writes it to a Linux SocketCAN device (e.g. can0).

(C) 2009-2015, Kees Verruijt, Harlingen, The Netherlands.

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

#include <stdio.h>
#include <stdlib.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>

#define  GLOBALS
#include "pgn.h"

static int openCanDevice(char * device, int * socket);
static void writeRawPGNToCanSocket(RawMessage* msg, int socket);
static void sendCanFrame(struct can_frame* frame, int socket);
static void sendN2kFastPacket(RawMessage* msg, struct can_frame* frame, int socket);


int main(int argc, char ** argv)
{
  FILE * file = stdin;
  char msg[2000];
  int socket;

  if(argc != 2)
  {
    puts("Usage: socketcan-writer <can-device>");
    exit(1);
  }

  if(openCanDevice(argv[1], &socket))
  {
    exit(1);
  }

  while (fgets(msg, sizeof(msg) - 1, file))
  {
    RawMessage m;
    if(parseRawFormatFast(msg, &m, false))
    {
      continue;  // Parsing failed -> skip the line
    }
    writeRawPGNToCanSocket(&m, socket);
  }

  close(socket);
  exit(0);
}

/*
  Opens SocketCAN socket to given device, see: https://www.kernel.org/doc/Documentation/networking/can.txt
*/
static int openCanDevice(char * device, int * canSocket)
{
  struct sockaddr_can addr;
  struct ifreq ifr;

  if ((*canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("socket");
    return 1;
  }

  strncpy(ifr.ifr_name, device, IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex)
  {
    perror("if_nametoindex");
    return 1;
  }

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(*canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    perror("bind");
    return 1;
  }

  return 0;
}

static void writeRawPGNToCanSocket(RawMessage * msg, int socket)
{
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.can_id = getCanIdFromISO11783Bits(msg->prio, msg->pgn, msg->src, msg->dst);

  if(msg->len <= 8)  // 8 or less bytes of data -> PGN fits into a single CAN frame
  {
    frame.can_dlc = msg->len;
    memcpy(frame.data, msg->data, msg->len);
    sendCanFrame(&frame, socket);
  }
  else
  {           // Send PGN as n2k fast packet (spans multiple CAN frames, but CAN ID is still same for each frame)
    sendN2kFastPacket(msg, &frame, socket);
  }
}

static void sendCanFrame(struct can_frame * frame, int socket)
{
  if (write(socket, frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
  {
    perror("write");
  }
}

/*
  See pgn.h for n2k fast packet data format
*/
static void sendN2kFastPacket(RawMessage * msg, struct can_frame* frame, int socket)
{
  int index = 0;
  int remainingDataBytes = msg->len;
  while(remainingDataBytes > 0)
  {
    frame->data[0] = index;  // fast packet index (increases by 1 for every CAN frame), 'order' (the 3 uppermost bits) is left as 0 for now

    if(index == 0)    // 1st frame
    {
      frame->data[1] = msg->len;  // fast packet payload size
      memcpy(frame->data + 2, msg->data, 6);  // 6 first data bytes
      frame->can_dlc = 8;
      remainingDataBytes -= 6;
    }
    else              // further frames
    {
      if(remainingDataBytes > 7)
      {
        memcpy(frame->data + 1, msg->data + 6 + (index-1) * 7, 7);  // 7 next data bytes
        frame->can_dlc = 8;
        remainingDataBytes -= 7;
      }
      else
      {
        memcpy(frame->data + 1, msg->data + 6 + (index-1) * 7, remainingDataBytes);  // 7 next data bytes
        frame->can_dlc = 1 + remainingDataBytes;
        remainingDataBytes = 0;
      }
    }
    sendCanFrame(frame, socket);
    index++;
  }
}