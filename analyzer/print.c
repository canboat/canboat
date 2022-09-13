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

#include <math.h>

#include "analyzer.h"
#include "common.h"

extern int g_variableFieldRepeat[2]; // Actual number of repetitions
extern int g_variableFieldIndex;

static bool unhandledStartOffset(const char *fieldName, size_t startBit)
{
  logError("Field '%s' cannot start on bit %u\n", fieldName, startBit);
  return false;
}

static bool unhandledBitLength(const char *fieldName, size_t length)
{
  logError("Field '%s' cannot have size %u\n", fieldName, length);
  return false;
}

static char  mbuf[8192];
static char *mp = mbuf;

extern void mprintf(const char *format, ...)
{
  va_list ap;
  int     remain;

  va_start(ap, format);
  remain = sizeof(mbuf) - (mp - mbuf) - 1;
  if (remain > 0)
  {
    mp += vsnprintf(mp, remain, format, ap);
  }
  va_end(ap);
}

extern void mreset(void)
{
  mp = mbuf;
}

extern void mwrite(FILE *stream)
{
  fwrite(mbuf, sizeof(char), mp - mbuf, stream);
  fflush(stream);
  mreset();
}

extern char *getSep(void)
{
  char *s = sep;

  if (showJson)
  {
    sep = ",";
    if (strchr(s, '{'))
    {
      if (strlen(closingBraces) >= sizeof(closingBraces) - 2)
      {
        logError("Too many braces\n");
        exit(2);
      }
      strcat(closingBraces, "}");
    }
  }
  else
  {
    sep = ";";
  }

  return s;
}

extern void printEmpty(const char *fieldName, int64_t exceptionValue)
{
  if (showJsonEmpty)
  {
    mprintf("%s\"%s\": null", getSep(), fieldName);
  }
  else if (!showJson)
  {
    switch (exceptionValue)
    {
      case DATAFIELD_UNKNOWN:
        mprintf("%s %s = Unknown", getSep(), fieldName);
        break;
      case DATAFIELD_ERROR:
        mprintf("%s %s = ERROR", getSep(), fieldName);
        break;
      case DATAFIELD_RESERVED1:
        mprintf("%s %s = RESERVED1", getSep(), fieldName);
        break;
      case DATAFIELD_RESERVED2:
        mprintf("%s %s = RESERVED2", getSep(), fieldName);
        break;
      case DATAFIELD_RESERVED3:
        mprintf("%s %s = RESERVED3", getSep(), fieldName);
        break;
      default:
        mprintf("%s %s = Unhandled value %ld", getSep(), fieldName, exceptionValue);
    }
  }
}

static bool extractNumberNotEmpty(const Field *field,
                                  const char  *fieldName,
                                  uint8_t     *data,
                                  size_t       dataLen,
                                  size_t       startBit,
                                  size_t       bits,
                                  int64_t     *value,
                                  int64_t     *maxValue)
{
  int64_t reserved;

  if (!extractNumber(field, data, dataLen, startBit, bits, value, maxValue))
  {
    return false;
  }

  if (*maxValue >= 15)
  {
    reserved = 2; /* DATAFIELD_ERROR and DATAFIELD_UNKNOWN */
  }
  else if (*maxValue > 1)
  {
    reserved = 1; /* DATAFIELD_UNKNOWN */
  }
  else
  {
    reserved = 0;
  }

  if (fieldName[0] == '#')
  {
    logDebug("g_variableFieldRepeat[%d]=%d\n", g_variableFieldIndex, *value);
    g_variableFieldRepeat[g_variableFieldIndex++] = *value;
  }

  if (*value > *maxValue - reserved)
  {
    printEmpty(fieldName, *value - *maxValue);
    return false;
  }

  if (showBytes)
  {
    mprintf("(%" PRIx64 " = %" PRId64 ") ", value, value);
  }
  return true;
}

extern bool fieldPrintNumber(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  int64_t value;
  int64_t maxValue;
  double  a;

  if (!extractNumberNotEmpty(field, fieldName, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }

  if (field->resolution == 1.0)
  {
    if (showJson)
    {
      mprintf("%s\"%s\":%" PRId64 "", getSep(), fieldName, value);
    }
    else
    {
      mprintf("%s %s = %" PRId64, getSep(), fieldName, value);
    }
  }
  else
  {
    int         precision;
    double      r;
    const char *units = field->units;

    a = (double) value * field->resolution;

    precision = 0;
    for (r = field->resolution; (r > 0.0) && (r < 1.0); r *= 10.0)
    {
      precision++;
    }

    if (field->resolution == RES_RADIANS)
    {
      units = "rad";
      if (!showSI)
      {
        a *= RadianToDegree;
        precision -= 3;
        units = "deg";
      }
    }
    else if (field->resolution == RES_ROTATION || field->resolution == RES_HIRES_ROTATION)
    {
      units = "rad/s";
      if (!showSI)
      {
        a *= RadianToDegree;
        precision -= 3;
        units = "deg/s";
      }
    }
    else if (units && showSI)
    {
      if (strcmp(units, "kWh") == 0)
      {
        a *= 3.6e6; // 1 kWh = 3.6 MJ.
      }
      else if (strcmp(units, "Ah") == 0)
      {
        a *= 3600.0; // 1 Ah = 3600 C.
      }

      // Many more to follow, but pgn.h is not yet complete enough...
    }
    else if (units && !showSI)
    {
      if (strcmp(units, "C") == 0)
      {
        a /= 3600.0; // 3600 C = 1 Ah
        units = "Ah";
      }
    }

    if (showJson)
    {
      mprintf("%s\"%s\":%.*f", getSep(), fieldName, precision, a);
    }
    else if (units && strcmp(units, "m") == 0 && a >= 1000.0)
    {
      mprintf("%s %s = %.*f km", getSep(), fieldName, precision + 3, a / 1000);
    }
    else if (units && units[0] != '=')
    {
      mprintf("%s %s = %.*f", getSep(), fieldName, precision, a);
      if (units)
      {
        mprintf(" %s", units);
      }
    }
  }

  return true;
}

extern bool fieldPrintFloat(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return true;
}
extern bool fieldPrintDecimal(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return true;
}

extern bool fieldPrintLookup(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  int64_t value;
  int64_t maxValue;

  if (!extractNumberNotEmpty(field, fieldName, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }

  if (field->units && field->units[0] == '=' && isdigit(field->units[1]))
  {
    char        lookfor[20];
    const char *s;

    sprintf(lookfor, "=%" PRId64, value);
    if (strcmp(lookfor, field->units) != 0)
    {
      if (showBytes)
        logError("Field %s value %" PRId64 " does not match %s\n", fieldName, value, field->units + 1);
      return false;
    }
    s = field->description;
    if (!s)
    {
      s = lookfor + 1;
    }
    if (showJson)
    {
      mprintf("%s\"%s\":\"%s\"", getSep(), fieldName, s);
    }
    else
    {
      mprintf("%s %s = %s", getSep(), fieldName, s);
    }
    return true;
  }

  const char *s = NULL;

  if (field->lookupValue != NULL && value >= 0)
  {
    s = field->lookupValue[value];
  }

  if (s != NULL)
  {
    if (showJsonValue)
    {
      mprintf("%s\"%s\":{\"value\":%" PRId64 ",\"name\":\"%s\"}", getSep(), fieldName, value, s);
    }
    else if (showJson)
    {
      mprintf("%s\"%s\":\"%s\"", getSep(), fieldName, s);
    }
    else
    {
      mprintf("%s %s = %s", getSep(), fieldName, s);
    }
  }
  else
  {
    if (showJson)
    {
      mprintf("%s\"%s\":\"%" PRId64 "\"", getSep(), fieldName, value);
    }
    else
    {
      mprintf("%s %s = %" PRId64 "", getSep(), fieldName, value);
    }
  }

  return true;
}

extern bool fieldPrintBitLookup(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  int64_t value;
  int64_t maxValue;
  int64_t bitValue;
  size_t  bit;
  char    sep;

  if (!extractNumberNotEmpty(field, fieldName, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }

  for (bitValue = 1, bit = 0; bitValue <= maxValue; (bitValue *= 2), bit++)
  {
    logDebug("RES_BITFIELD is bit %u value %" PRIx64 " set %d\n", bit, bitValue, (value & value) >= 0);
    if ((value & bitValue) != 0)
    {
      const char *s = field->lookupValue[value];

      if (s)
      {
        if (showJson)
        {
          mprintf("%c\"%s\"", sep, s);
        }
        else
        {
          mprintf("%c%s", sep, s);
        }
      }
      else
      {
        mprintf("%c\"%" PRIu64 "\"", sep, bitValue);
      }
      sep = ',';
    }
  }
  if (showJson)
  {
    if (sep != '[')
    {
      mprintf("]");
    }
    else
    {
      mprintf("[]");
    }
  }
  return true;
}

extern bool fieldPrintLatLon(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  uint64_t absVal;
  int64_t  value;
  int64_t  maxValue;
  size_t   bytes       = *bits >> 3;
  bool     isLongitude = (strstr(fieldName, "ongit") != NULL);
  double   dd;
  double   degrees;
  double   remainder;
  double   minutes;
  double   seconds;
  double   scale;

  logDebug("fieldPrintLatLon for '%s' startbit=%zu bits=%zu\n", fieldName, startBit, *bits);

  if (!extractNumberNotEmpty(field, fieldName, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }

  absVal = (value < 0) ? -value : value;

  if (showGeo == GEO_DD)
  {
    scale = log10(1.0 / field->resolution);
    dd    = (double) value * field->resolution;
    logDebug("float %g resolution %g scale %g\n", dd, field->resolution, scale);

    if (showJson)
    {
      mprintf("%s\"%s\":%.*g", getSep(), fieldName, (int) scale, dd);
    }
    else
    {
      mprintf("%s %s = %.*g", getSep(), fieldName, (int) scale, dd);
    }
  }
  else if (showGeo == GEO_DM)
  {
    dd        = (double) absVal * field->resolution;
    degrees   = floor(dd);
    remainder = dd - degrees;
    minutes   = remainder * 60.;

    mprintf((showJson ? "%s\"%s\":\"%02u&deg; %6.3f %c\"" : "%s %s = %02ud %6.3f %c"),
            getSep(),
            fieldName,
            (uint32_t) degrees,
            minutes,
            (isLongitude ? ((value >= 0) ? 'E' : 'W') : ((value >= 0) ? 'N' : 'S')));
  }
  else
  {
    scale     = floor(log10(1.0 / field->resolution / 3600.));
    dd        = (double) absVal * field->resolution;
    degrees   = floor(dd);
    remainder = dd - degrees;
    minutes   = floor(remainder * 60.);
    seconds   = floor(remainder * 3600.) - 60. * minutes;

    mprintf((showJson ? "%s\"%s\":\"%02u&deg;%02u&rsquo;%06.*f&rdquo;%c\"" : "%s %s = %02ud %02u' %06.*f\"%c"),
            getSep(),
            fieldName,
            (int) degrees,
            (int) minutes,
            (int) scale,
            seconds,
            (isLongitude ? ((value >= 0) ? 'E' : 'W') : ((value >= 0) ? 'N' : 'S')));
    if (showJson)
    {
      scale = log10(1.0 / field->resolution);
      dd    = (double) value * field->resolution;
      logDebug("float %g resolution %g scale %g\n", dd, field->resolution, scale);

      mprintf("%s\"%s_dd\":%.*g", getSep(), fieldName, (int) scale, dd);
    }
  }
  return true;
}

extern bool fieldPrintTime(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  uint64_t unitspersecond;
  uint32_t hours;
  uint32_t minutes;
  uint32_t seconds;
  uint32_t units;
  int64_t  value;
  int64_t  maxValue;
  uint64_t t;

  if (!extractNumberNotEmpty(field, fieldName, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }

  t = (uint64_t) value;

  logDebug("fieldPrintTime(<%s>, \"%s\") t=%" PRIu64 " res=%g max=0x%" PRIx64 "\n",
           field->name,
           fieldName,
           t,
           field->resolution,
           maxValue);

  if (field->resolution < 1.0)
  {
    unitspersecond = (uint64_t) (1.0 / field->resolution);
  }
  else
  {
    unitspersecond = 1;
    t *= (uint64_t) field->resolution;
  }

  seconds = t / unitspersecond;
  units   = t % unitspersecond;
  minutes = seconds / 60;
  seconds = seconds % 60;
  hours   = minutes / 60;
  minutes = minutes % 60;

  if (showJson)
  {
    if (units != 0)
    {
      mprintf("%s \"%s\": \"%02u:%02u:%02u.%05u\"", getSep(), fieldName, hours, minutes, seconds, units);
    }
    else
    {
      mprintf("%s \"%s\": \"%02u:%02u:%02u\"", getSep(), fieldName, hours, minutes, seconds);
    }
  }
  else
  {
    if (units)
    {
      mprintf("%s %s = %02u:%02u:%02u.%05u", getSep(), fieldName, hours, minutes, seconds, units);
    }
    else
    {
      mprintf("%s %s = %02u:%02u:%02u", getSep(), fieldName, hours, minutes, seconds);
    }
  }
  return true;
}

extern bool fieldPrintDate(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  char       buf[sizeof("2008.03.10") + 1];
  time_t     t;
  struct tm *tm;
  uint16_t   d;

  if (startBit != 0)
  {
    return unhandledStartOffset(fieldName, startBit);
  }
  if (*bits != 16)
  {
    return unhandledBitLength(fieldName, *bits);
  }
  if (dataLen < *bits / 8)
  {
    return true;
  }

  d = data[0] + (data[1] << 8);

  if (d >= 0xfffd)
  {
    printEmpty(fieldName, d - INT64_C(0xffff));
    return true;
  }

  t  = d * 86400;
  tm = gmtime(&t);
  if (!tm)
  {
    logAbort("Unable to convert %u to gmtime\n", (unsigned int) t);
  }
  strftime(buf, sizeof(buf), "%Y.%m.%d", tm);
  if (showJson)
  {
    mprintf("%s\"%s\":\"%s\"", getSep(), fieldName, buf);
  }
  else
  {
    mprintf("%s %s = %s", getSep(), fieldName, buf);
  }
  return true;
}

extern bool fieldPrintStringFix(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return true;
}
extern bool fieldPrintStringVar(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return true;
}
extern bool fieldPrintStringLZ(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return true;
}
extern bool fieldPrintStringLAU(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return true;
}
extern bool fieldPrintMMSI(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return true;
}
extern bool fieldPrintBinary(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return true;
}
extern bool fieldPrintVariable(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return true;
}
