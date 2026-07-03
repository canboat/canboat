/*

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

#ifndef MARETRON_IPG_H_INCLUDED
#define MARETRON_IPG_H_INCLUDED

#define MARETRON_DEFAULT_PORT "6543"
#define MARETRON_FRAME_SYNC 0xA5
#define MARETRON_F1_SYNC_BIT 0x80
#define MARETRON_VIDEO_PREFIX 0x33 /* '3' */
#define MARETRON_ASCII_PREFIX 0x32 /* '2' */

#define TX_CONNECT_MSG "CONNECT\t\"%s\"\t\tMOBILE"
#define TX_SET_MODE_BINARY_MSG "SET_MODE\tBINARY"

#define RX_CONNECTED "CONNECTED"
#define RX_NO "NO"
#define RX_SERVER_VERSION "SERVER_VERSION"
#define RX_INSTANCE_DATA "INSTANCE_DATA"
#define RX_LICENSES_USED "LICENSES_USED"
#define RX_DETAILED_LICENSES_USED "DETAILED_LICENSES_USED"

#endif
