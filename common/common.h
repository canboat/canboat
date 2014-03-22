/*

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

#ifndef CANBOAT_COMMON

#include "license.h"

#ifdef WIN32
# define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#include <stdarg.h>
#include <ctype.h>

#ifndef WIN32
# include <arpa/inet.h>
# include <stdint.h>
# include <syslog.h>
# include <sys/ioctl.h>
# include <sys/select.h>
# include <sys/socket.h>
# include <sys/time.h>
# include <sys/types.h>
# include <netdb.h>
# include <netinet/in.h>
# include <stdbool.h>
# include <termios.h>
# define HAS_SYSLOG
typedef int SOCKET;
#else
# include <winsock2.h>
# include "winport.h"
#endif

#ifndef CB_MAX
# define CB_MAX(x,y) (((x)>(y))?(x):(y))
#endif

#ifndef CB_MIN
# define CB_MIN(x,y) (((x)<(y))?(x):(y))
#endif

#define STRSIZE(x) (sizeof(x) - 1)

#ifndef INVALID_SOCKET
# define INVALID_SOCKET (-1)
#endif

typedef enum LogLevel
{ LOGLEVEL_FATAL
, LOGLEVEL_ERROR
, LOGLEVEL_INFO
, LOGLEVEL_DEBUG
} LogLevel;

int logDebug(const char * format, ...);
int logInfo(const char * format, ...);
int logError(const char * format, ...);
void logAbort(const char * format, ...);
void die(const char * t);
void setLogLevel(LogLevel level);
void setProgName(char * name);

typedef struct StringBuf
{
  char * data;
  size_t len;
  size_t alloc;
} StringBuffer;

StringBuffer sbNew;
#define sbGet(sb) (sb->data)
void sbAppendData(StringBuffer * sb, const void * data, size_t len);
void sbAppendString(StringBuffer * sb, const char * string);

int getJSONValue( const char * message, const char * fieldName, char * value, size_t len );
void getISO11783BitsFromCanId(unsigned int id, unsigned int * prio, unsigned int * pgn, unsigned int * src, unsigned int * dst);

SOCKET open_socket_stream(const char * url);

# define CANBOAT_COMMON
#endif
