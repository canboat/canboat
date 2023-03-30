/*

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

#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED

#include "license.h"

#ifdef WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#ifndef WIN32
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <syslog.h>
#include <termios.h>
#define HAS_SYSLOG
typedef int SOCKET;
#else
#include <winsock2.h>

#include "winport.h"
#endif

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
#endif

#ifndef CB_MAX
#define CB_MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif

#ifndef CB_MIN
#define CB_MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

#define STRSIZE(x) (sizeof(x) - 1)
#define STRNULL(x) ((x != NULL) ? (x) : "NULL")

#ifndef INVALID_SOCKET
#define INVALID_SOCKET (-1)
#endif

#define STDIN (0)
#define STDOUT (1)
#define STDERR (2)

typedef enum LogLevel
{
  LOGLEVEL_FATAL,
  LOGLEVEL_ERROR,
  LOGLEVEL_INFO,
  LOGLEVEL_DEBUG
} LogLevel;

int  logDebug(const char *format, ...);
int  logInfo(const char *format, ...);
int  logError(const char *format, ...);
void logAbort(const char *format, ...);
void die(const char *t);
void setLogLevel(LogLevel level);
bool isLogLevelEnabled(LogLevel level);
void setProgName(char *name);
void setFixedTimestamp(char *fixedStr);

typedef struct StringBuffer
{
  char  *data;
  size_t len;
  size_t alloc;
} StringBuffer;

enum Base64Encoding
{
  BASE64_RFC,
  BASE64_AIS
};

extern StringBuffer sbNew;

void  sbAppendEncodeHex(StringBuffer *sb, const void *data, size_t len, char separator);                     // binary to hex
void  sbAppendEncodeBase64(StringBuffer *sb, const uint8_t *data, size_t len, enum Base64Encoding encoding); // binary to Base64
void  sbAppendDecodeHex(StringBuffer *sb, const char *data, size_t len);                                     // hex to binary
void  sbAppendDecodeBase64(StringBuffer *sb, const char *data, size_t len, enum Base64Encoding encoding);    // base64 to binary
void  sbAppendData(StringBuffer *sb, const void *data, size_t len);
void  sbAppendString(StringBuffer *sb, const char *string);
void  sbAppendFormat(StringBuffer *const sb, const char *const format, ...);
void  sbAppendFormatV(StringBuffer *const sb, const char *const format, va_list ap);
void  sbDelete(StringBuffer *sb, size_t start, size_t end);
void  sbEnsureCapacity(StringBuffer *const sb, size_t len);
char *sbSearchChar(const StringBuffer *const sb, char c);

#define sbGet(sb) ((sb)->data ? (sb)->data : "")
#define sbGetLength(sb) ((sb)->len)
#define sbTerminate(sb)             \
  {                                 \
    if ((sb)->data)                 \
    {                               \
      (sb)->data[(sb)->len] = '\0'; \
    }                               \
  }
#define sbTruncate(sb, newLen)             \
  {                                        \
    (sb)->len = CB_MIN(newLen, (sb)->len); \
    sbTerminate(sb);                       \
  }
#define sbEmpty(sb)    \
  {                    \
    sbTruncate(sb, 0); \
  }
#define sbClean(sb)     \
  {                     \
    if ((sb)->data)     \
    {                   \
      free((sb)->data); \
      (sb)->data = 0;   \
    }                   \
    (sb)->alloc = 0;    \
    (sb)->len   = 0;    \
  }

bool         getJSONValue(const char *message, const char *fieldName, char *value, size_t len);
bool         getJSONLookupValue(const char *message, const char *fieldName, int64_t *value);
bool         getJSONLookupName(const char *message, const char *fieldName, char *value, size_t len);
void         getISO11783BitsFromCanId(unsigned int id, unsigned int *prio, unsigned int *pgn, unsigned int *src, unsigned int *dst);
unsigned int getCanIdFromISO11783Bits(unsigned int prio, unsigned int pgn, unsigned int src, unsigned int dst);

SOCKET open_socket_stream(const char *url);

#define DATE_LENGTH 60
const char *now(char str[DATE_LENGTH]);
uint64_t    getNow(void);
void        storeTimestamp(char str[DATE_LENGTH], uint64_t when);

uint8_t scanNibble(char c);
int     scanHex(char **p, uint8_t *m);

enum ReadyDescriptor
{
  FD1_ReadReady  = 0x0001,
  FD2_ReadReady  = 0x0002,
  FD3_WriteReady = 0x0004
};

/*
 * Wait for R/W fd1, Read fd2 or Write fd3
 */
int isReady(int fd1, int fd2, int fd3, int timeout);

int writeSerial(int handle, const uint8_t *data, size_t len);

#define UINT16_OUT_OF_RANGE (MAX_UINT16 - 1)
#define UINT16_UNKNOWN (MAX_UINT16)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#endif

// C stringify operations with macro names
#define xstr(s) str(s)
#define str(s) #s

/*
 * Notes on the NMEA 2000 packet structure
 * ---------------------------------------
 *
 * http://www.nmea.org/Assets/pgn059392.pdf tells us that:
 * - All messages shall set the reserved bit in the CAN ID field to zero on transmit.
 * - Data field reserve bits or reserve bytes shall be filled with ones. i.e. a reserve
 *   byte will be set to a hex value of FF, a single reservie bit would be set to a value of 1.
 * - Data field extra bytes shall be illed with a hex value of FF.
 * - If the PGN in a Command or Request is not recognized by the destination it shall
 *   reply with the PGN 059392 ACK or NACK message using a destination specific address.
 *
 */

/*
 * Some packets include a "SID", explained by Maretron as follows:
 * SID: The sequence identifier field is used to tie related PGNs together. For example,
 * the DST100 will transmit identical SIDs for Speed (PGN 128259) and Water depth
 * (128267) to indicate that the readings are linked together (i.e., the data from each
 * PGN was taken at the same time although reported at slightly different times).
 */

/*
 * NMEA 2000 uses the 8 'data' bytes for fast framed PGNs as follows:
 * data[0] is an 'order' that increments, or not (depending a bit on implementation).
 * If the size of the packet <= 7 then the data follows in data[1..7]
 * If the size of the packet > 7 then the next byte data[1] is the size of the payload
 * and data[0] is divided into 5 bits index into the fast packet, and 3 bits 'order
 * that increases.
 * This means that for 'fast packets' the first bucket (sub-packet) contains 6 payload
 * bytes and 7 for remaining. Since the max index is 31, the maximal payload is
 * 6 + 31 * 7 = 223 bytes
 */

#define FASTPACKET_INDEX (0)
#define FASTPACKET_SIZE (1)
#define FASTPACKET_BUCKET_0_SIZE (6)
#define FASTPACKET_BUCKET_N_SIZE (7)
#define FASTPACKET_BUCKET_0_OFFSET (2)
#define FASTPACKET_BUCKET_N_OFFSET (1)
#define FASTPACKET_MAX_INDEX (0x1f)
#define FASTPACKET_MAX_SIZE (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * FASTPACKET_MAX_INDEX)

#define IS_PGN_PROPRIETARY(n)                                                   \
  (((n) >= 0xEF00 && (n) <= 0xEFFF)      /* PDU1 (addressed) single-frame */    \
   || ((n) >= 0xFF00 && (n) <= 0xFFFF)   /* PDU2 (nonaddressed) single-frame */ \
   || ((n) >= 0x1EF00 && (n) <= 0x1EFFF) /* PDU1 (addressed) fast-packet */     \
   || ((n) >= 0x1FF00 && (n) <= 0x1FFFF) /* PDU2 (nonaddressed) fast-packet */  \
  )

#define ALLOW_PGN_FAST_PACKET(n) (((n) >= 0x10000 && (n) < 0x1FFFF) || (n) >= CANBOAT_PGN_START)
#define ALLOW_PGN_SINGLE_FRAME(n) ((n) < 0x10000 || (n) >= 0x1F000)

#define MAP_PGN_TO_CONTINUOUS_RANGE(n) ((n) - (0xE800))
#define PGN_MAX_CONTINUOUS_RANGE (MAP_PGN_TO_CONTINUOUS_RANGE(0x20000))

#define Pi (3.141592654)
#define RadianToDegree (360.0 / 2 / Pi)
#define BITS(x) (x)
#define BYTES(x) ((x) * (8))
#define BITS_TO_BYTES(x) ((x) >> 3)

int  logInfo(const char *format, ...);
int  logDebug(const char *format, ...);
int  logError(const char *format, ...);
void logAbort(const char *format, ...);

/*
 * The 'converter' programs generate fake PGNs containing data that they generate
 * themselves or via proprietary non-PGN serial messages.
 * These need unique fake PGNs.
 */
#define CANBOAT_PGN_START 0x40000
#define CANBOAT_PGN_END 0x401FF
#define ACTISENSE_BEM 0x40000 /* Actisense specific fake PGNs */
#define IKONVERT_BEM 0x40100  /* iKonvert specific fake PGNs */

#endif
