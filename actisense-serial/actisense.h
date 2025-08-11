/*

Defines to interface with an Actisense NGT-1

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

/* ASCII characters used to mark packet start/stop */

#define SOH (0x01) /* Start of Heading */
#define STX (0x02) /* Start of Text (packet) */
#define ETX (0x03) /* End of Text (packet) */
#define LF (0x0A)  /* Line Feed (end of heading) */
#define DLE (0x10) /* Data Link Escape: to encode a STX or ETX send DLE+STX or DLE+ETX */
#define ESC (0x1B) /* Escape */

/* Actisense message structure is:

   DLE STX <command> <len> [<data> ...]  <checksum> DLE ETX

   <command> is a byte from the list below.
   In <data> any DLE characters are double escaped (DLE DLE).
   <len> encodes the unescaped length.
   <checksum> is such that the sum of all unescaped data bytes plus the command
              byte plus the length adds up to zero, modulo 256.
*/

// From https://github.com/aldas/go-nmea-client/blob/75bc78b9f6f828f93bc7c51ce4cd87e738f89c12/actisense/eblreader.go#L12
// EBL log file format used by Actisense W2K-1. Probably called "CAN-Raw (BST-95) message format"
// NGT1 ebl files are probably in different format.
//
// Example data frame from one EBL file:
// 1b 01 07 95 0e 28 9a 00 01 f8 09 3d 0d b3 22 48 32 59 0d 1b 0a
//
// 1b 01 <-- start of data frame (ESC+SOH)
//
//	07 95 <-- "95" is maybe frame type. Actisense EBL Reader v2.027 says
//	          "now has added support for the new CAN-Raw (BST-95) message format
//	          that is used for all data logging on Actisense W2K-1"
//	     0e <-- lengths 14 bytes till end
//	       28 9a <-- timestamp 39464 (hex 9A28) (little endian)
//	            00 01 f8 09  <--- 0x09f80100 = src:0, dst:255, pgn:129025 (1f801), prio:2 (little endian)
//	                       3d 0d b3 22 48 32 59 0d <-- CAN payload,
//	                                               1b 0a <-- end of data frame (ESC+LF)
//
// Timestamp is offset from time found in first data frame in file
// Example: first frame in file:
// 1B 01 03 00 10 E7 A7 84 83 D9 01 1B 0A
//
//	03 <--- "03" maybe frame type
//	   00 10 E7 A7 84 83 D9 01 <-- 8 byte Windows FILETIME

#define N2K_MSG_RECEIVED (0x93) /* Receive standard N2K message */
#define N2K_MSG_SEND (0x94)     /* Send N2K message */
#define NGT_MSG_RECEIVED (0xA0) /* Receive NGT specific message */
#define NGT_MSG_SEND (0xA1)     /* Send NGT message */

#define EBL_TIMESTAMP (0x03) /* 8 byte Windows FILETIME */
#define EBL_VERSION (0x01)   /* Some sort of version ID */
