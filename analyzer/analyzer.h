/*

Analyzes NMEA 2000 PGNs.

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

#ifdef J1939
#include "pgn-j1939.h"
#else
#include "pgn.h"
#endif

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

extern bool         showRaw;
extern bool         showData;
extern bool         showJson;
extern bool         showJsonEmpty;
extern bool         showJsonValue;
extern bool         showBytes;
extern bool         showSI;
extern GeoFormats   showGeo;
extern char        *sep;
extern char         closingBraces[16]; // } and ] chars to close sentence in JSON mode, otherwise empty string
extern bool         g_skip;
extern const Field *g_ftf;
extern int64_t      g_length;

/* analyzer.c */

/* print.c */

extern char  *getSep(void);
extern void   mprintf(const char *format, ...);
extern void   mreset(void);
extern void   mwrite(FILE *stream);
extern size_t mlocation(void);
extern void   mset(size_t location);
extern char   mchr(size_t location);
extern void   minsert(size_t location, const char *str);
extern void   printEmpty(const char *name, int64_t exceptionValue);
extern bool   adjustDataLenStart(const uint8_t **data, size_t *dataLen, size_t *startBit);
