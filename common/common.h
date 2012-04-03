
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
void die (char * t);
void setLogLevel(LogLevel level);
void setProgName(char * name);
