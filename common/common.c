/*

(C) 2009-2012, Kees Verruijt, Harlingen, The Netherlands.

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


#include "common.h"

static const char * logLevels[] =
{ "FATAL"
, "ERROR"
, "INFO"
, "DEBUG"
};

static LogLevel logLevel = LOGLEVEL_INFO;

static char * progName;

#ifndef WIN32

static int logBase(LogLevel level, const char * format, va_list ap)
{
  struct timeval tv;
  time_t t;
  struct tm tm;
  int msec;
  char strTmp[60];

  if (level > logLevel)
  {
    return 0;
  }


  if (gettimeofday(&tv, (void *) 0) == 0)
  {
    t = tv.tv_sec;
    msec = tv.tv_usec / 1000L;
    localtime_r(&t, &tm);
    strftime(strTmp, 60, "%Y-%m-%d %H:%M:%S", &tm);
    fprintf(stderr, "%s %s.%3.3d [%s] ", logLevels[level], strTmp, msec, progName);
  }
  else
  {
    fprintf(stderr, "%s [%s] ", logLevels[level], progName);
  }

  return vfprintf(stderr, format, ap);
}

#else

static int logBase(LogLevel level, const char * format, va_list ap)
{
  struct _timeb timebuffer;
  struct tm tm;
  char strTmp[60];

  if (level > logLevel)
  {
    return 0;
  }

  _ftime_s(&timebuffer);
  localtime_s(&tm, &timebuffer.time);
  strftime(strTmp, 60, "%Y-%m-%d %H:%M:%S", &tm);
  fprintf(stderr, "%s %s.%3.3d [%s] ", logLevels[level], strTmp, timebuffer.millitm, progName);

  return vfprintf(stderr, format, ap);
}

#endif

int logInfo(const char * format, ...)
{
  va_list ap;
  va_start(ap, format);

  return logBase(LOGLEVEL_INFO, format, ap);
}

int logDebug(const char * format, ...)
{
  va_list ap;
  va_start(ap, format);

  return logBase(LOGLEVEL_DEBUG, format, ap);
}

int logError(const char * format, ...)
{
  va_list ap;
  va_start(ap, format);

  return logBase(LOGLEVEL_ERROR, format, ap);
}

void logAbort(const char * format, ...)
{
  va_list ap;
  va_start(ap, format);

  logBase(LOGLEVEL_FATAL, format, ap);
  exit(2);
}

void die (char * t)
{
  int e = errno;
  char * s = 0;

  if (e)
  {
    s = strerror(e);
  }

  if (s)
  {
    logAbort("%s: %s\n", t, s);
  }
  else
  {
    logAbort("%s\n", t);
  }
}

void setLogLevel(LogLevel level)
{
  logLevel = CB_MIN(CB_MAX(level, LOGLEVEL_FATAL), LOGLEVEL_DEBUG);
  logDebug("Loglevel now %d\n", logLevel);
}

void setProgName(char * name)
{
  progName = strrchr(name, '/');
  if (!progName)
  {
    progName = strrchr(name, '\\');
  }
  if (!progName)
  {
    progName = name;
  }
  else
  {
    progName++;
  }
}

