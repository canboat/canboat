/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2021, Kees Verruijt, Harlingen, The Netherlands.

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

#ifdef WIN32
#include <stdio.h>
#include <windows.h>

#include "nonstdbool.h"
#else
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef __APPLE__
#include <malloc.h>
#endif
#include <ctype.h>
#include <inttypes.h>
#include <stdarg.h>
#include <string.h>
#include <sys/time.h>
#endif

#include <math.h>
#include <time.h>

#ifdef WIN32
typedef unsigned char      uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int       uint32_t;
typedef signed int         int32_t;
typedef __int64            int64_t;
typedef unsigned __int64   uint64_t;
#define UINT64_C(x) ((uint64_t) (x))
#define INT64_C(x) ((int64_t) (x))
#define PRId64 "I64d"
#define PRIu64 "I64u"
#define PRIx64 "I64x"
#define PRIX64 "I64X"
#define strcasecmp _stricmp
#define UINT16_MAX (0xffff)
#define UINT32_MAX (0xffffffff)
#define SKIP_SETSYSTEMCLOCK
#endif

#ifndef WIN32
#ifndef __CYGWIN__
#define HAS_ADJTIME
#endif
#endif

#ifndef min
#define min(x, y) ((x) <= (y) ? (x) : (y))
#endif
#ifndef max
#define max(x, y) ((x) >= (y) ? (x) : (y))
#endif

#include "pgn.h"

#define DST_GLOBAL (0xff) /* The address used when a message is addressed to -all- stations */

/* There are max five reserved values according to ISO 11873-9 (that I gather from indirect sources)
 * but I don't yet know which datafields reserve the reserved values.
 */
#define DATAFIELD_UNKNOWN (0)
#define DATAFIELD_ERROR (-1)
#define DATAFIELD_RESERVED1 (-2)
#define DATAFIELD_RESERVED2 (-3)
#define DATAFIELD_RESERVED3 (-4)

typedef enum GeoFormats
{
  GEO_DD,
  GEO_DM,
  GEO_DMS
} GeoFormats;

extern bool       showRaw;
extern bool       showData;
extern bool       showJson;
extern bool       showJsonEmpty;
extern bool       showJsonValue;
extern bool       showSI;
extern GeoFormats showGeo;
extern char      *sep;
extern char       closingBraces[16]; // } and ] chars to close sentence in JSON mode, otherwise empty string
extern bool       g_skip;

/* analyzer.c */

/* print.c */

extern char  *getSep(void);
extern void   mprintf(const char *format, ...);
extern void   mreset(void);
extern void   mwrite(FILE *stream);
extern size_t mlocation(void);
extern void   printEmpty(const char *name, int64_t exceptionValue);
extern bool   adjustDataLenStart(uint8_t **data, size_t *dataLen, size_t *startBit);
