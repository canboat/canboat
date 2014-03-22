/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2014, Kees Verruijt, Harlingen, The Netherlands.

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

#ifdef WIN32
# include <stdio.h>
# include <windows.h>
# include "nonstdbool.h"
#else
# include <stdint.h>
# include <stdio.h>
# include <stdbool.h>
# include <stdlib.h>
# include <stdio.h>
# ifndef __APPLE__
#  include <malloc.h>
# endif
# include <string.h>
# include <stdarg.h>
# include <inttypes.h>
# include <sys/time.h>
# include <ctype.h>
#endif

#include <math.h>
#include <time.h>

#ifdef WIN32
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
# define UINT64_C(x) ((uint64_t)(x))
# define INT64_C(x) ((int64_t)(x))
# define PRId64 "I64d"
# define PRIu64 "I64u"
# define PRIx64 "I64x"
# define PRIX64 "I64X"
# define strcasecmp _stricmp
# define UINT16_MAX (0xffff)
# define UINT32_MAX (0xffffffff)
# define SKIP_SETSYSTEMCLOCK
#endif

#ifndef WIN32
# ifndef __CYGWIN__
#  define HAS_ADJTIME
# endif
#endif

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#endif

#ifndef min
# define min(x,y) ((x)<=(y)?(x):(y))
#endif
#ifndef max
# define max(x,y) ((x)>=(y)?(x):(y))
#endif

#include "pgn.h"

#define DST_GLOBAL (0xff) /* The address used when a message is addressed to -all- stations */

typedef struct
{
  char timestamp[30 + 1];
  uint8_t prio;
  uint32_t pgn;
  uint8_t dst;
  uint8_t src;
  uint8_t len;
  uint8_t data[FASTPACKET_MAX_SIZE];
} RawMessage;

typedef struct
{
  size_t lastFastPacket;
  size_t size;
  size_t allocSize;
  uint8_t * data;
} Packet;

typedef struct
{
  Packet packetList[ARRAY_SIZE(pgnList)];
} DevicePackets;


