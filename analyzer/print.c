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
#include "utf.h"

extern int g_variableFieldRepeat[2]; // Actual number of repetitions
bool       g_skip;

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

extern size_t mlocation(void)
{
  return mp - mbuf;
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

/*
 * Find a field by Order. This will only work for a field that
 * is at a predefined bit offset, so no variable fields before
 * it.
 *
 * It is currently only used for LOOKUP_TYPE_TRIPLET.
 */
static size_t getFieldOffsetByOrder(Pgn *pgn, size_t order)
{
  uint8_t i;
  size_t  bitOffset = 0;

  for (i = 0; i < order; i++)
  {
    Field *field = &pgn->fieldList[i];

    if (i + 1 == order)
    {
      return bitOffset;
    }
    bitOffset += field->size;
  }
  return 0;
}

bool adjustDataLenStart(uint8_t **data, size_t *dataLen, size_t *startBit)
{
  size_t bytes = *startBit >> 3;

  if (bytes < *dataLen)
  {
    *data += bytes;
    *dataLen += bytes;
    *startBit = *startBit & 7;
    return true;
  }

  return false;
}

static bool extractNumberByOrder(Pgn *pgn, size_t order, uint8_t *data, size_t dataLen, int64_t *value)
{
  Field *field     = &pgn->fieldList[order - 1];
  size_t bitOffset = getFieldOffsetByOrder(pgn, order);

  size_t  startBit;
  int64_t maxValue;

  startBit = bitOffset & 7;
  data += bitOffset >> 3;
  dataLen -= bitOffset >> 3;

  return extractNumber(field, data, dataLen, startBit, field->size, value, &maxValue);
}

extern void printEmpty(const char *fieldName, int64_t exceptionValue)
{
  if (showJson)
  {
    if (showJsonEmpty)
    {
      mprintf("%s\"%s\":null", getSep(), fieldName);
    }
    else
    {
      g_skip = true;
    }
  }
  else
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

  if (*maxValue >= 7)
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

  if (field->pgn->repeatingField1 == field->order)
  {
    logDebug("The first repeating fieldset repeats %" PRId64 " times\n", *value);
    g_variableFieldRepeat[0] = *value;
  }

  if (field->pgn->repeatingField2 == field->order)
  {
    logDebug("The second repeating fieldset repeats %" PRId64 " times\n", *value);
    g_variableFieldRepeat[1] = *value;
  }

  if (*value > *maxValue - reserved)
  {
    printEmpty(fieldName, *value - *maxValue);
    return false;
  }

  return true;
}

// This is only a different printer than fieldPrintNumber so the JSON can contain a string value
extern bool fieldPrintMMSI(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  int64_t value;
  int64_t maxValue;

  if (!extractNumberNotEmpty(field, fieldName, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }

  if (showJson)
  {
    mprintf("%s\"%s\":\"%09u\"", getSep(), fieldName, value);
  }
  else
  {
    mprintf("%s %s = \"%09u\"", getSep(), fieldName, value);
  }

  return true;
}

extern bool fieldPrintNumber(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  int64_t     value;
  int64_t     maxValue;
  double      a;
  const char *unit = field->unit;

  if (!extractNumberNotEmpty(field, fieldName, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }

  logDebug("fieldPrintNumber <%s> resolution=%g unit='%s'\n", fieldName, field->resolution, (field->unit ? field->unit : "-"));
  if (field->resolution == 1.0 && field->unitOffset == 0.0)
  {
    if (showJson)
    {
      mprintf("%s\"%s\":%" PRId64, getSep(), fieldName, value);
    }
    else
    {
      mprintf("%s %s = %" PRId64, getSep(), fieldName, value);
      if (unit != NULL)
      {
        mprintf(" %s", unit);
      }
    }
  }
  else
  {
    int    precision;
    double r;

    a = (double) value * field->resolution + field->unitOffset;

    precision = field->precision;
    if (precision == 0)
    {
      for (r = field->resolution; (r > 0.0) && (r < 1.0); r *= 10.0)
      {
        precision++;
      }
    }

    if (showJson)
    {
      mprintf("%s\"%s\":%.*f", getSep(), fieldName, precision, a);
    }
    else if (unit != NULL && strcmp(unit, "m") == 0 && a >= 1000.0)
    {
      mprintf("%s %s = %.*f km", getSep(), fieldName, precision + 3, a / 1000);
    }
    else
    {
      mprintf("%s %s = %.*f", getSep(), fieldName, precision, a);
      if (unit != NULL)
      {
        mprintf(" %s", unit);
      }
    }
  }

  return true;
}

extern bool fieldPrintFloat(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  union
  {
    float    a;
    uint32_t w;
    uint8_t  b[4];
  } f;

  if (!adjustDataLenStart(&data, &dataLen, &startBit))
  {
    return false;
  }

  if (*bits != sizeof(f) || startBit != 0)
  {
    logError("field '%s' FLOAT value unhandled bits=%zu startBit=%zu\n", fieldName, *bits, startBit);
    return false;
  }
  if (dataLen < sizeof(f))
  {
    return false;
  }
#ifdef __BIG_ENDIAN__
  f.b[3] = data[0];
  f.b[2] = data[1];
  f.b[1] = data[2];
  f.b[0] = data[3];
#else
  memcpy(&f.w, data, sizeof(f));
#endif

  if (showJson)
  {
    mprintf("%s\"%s\":%g", getSep(), fieldName, f.a);
  }
  else
  {
    mprintf("%s %s = %g", getSep(), fieldName, f.a);
    if (field->unit)
    {
      mprintf(" %s", field->unit);
    }
  }

  return true;
}
extern bool fieldPrintDecimal(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  uint8_t  value = 0;
  uint8_t  bitMask;
  uint64_t bitMagnitude = 1;
  size_t   bit;
  char     buf[128];

  if (!adjustDataLenStart(&data, &dataLen, &startBit))
  {
    return false;
  }

  bitMask = 1 << startBit;

  if (startBit + *bits > dataLen * 8)
  {
    *bits = dataLen * 8 - startBit;
  }

  if (showJson)
  {
    mprintf("%s\"%s\":\"", getSep(), fieldName);
  }
  else
  {
    mprintf("%s %s = ", getSep(), fieldName);
  }

  for (bit = 0; bit < *bits && bit < sizeof(buf) * 8; bit++)
  {
    /* Act on the current bit */
    bool bitIsSet = (*data & bitMask) > 0;
    if (bitIsSet)
    {
      value |= bitMagnitude;
    }

    /* Find the next bit */
    if (bitMask == 128)
    {
      bitMask = 1;
      data++;
    }
    else
    {
      bitMask = bitMask << 1;
    }
    bitMagnitude = bitMagnitude << 1;

    if (bit % 8 == 7)
    {
      if (value < 100)
      {
        mprintf("%02u", value);
      }
      value        = 0;
      bitMagnitude = 1;
    }
  }
  if (showJson)
  {
    mprintf("\"");
  }
  return true;
}

extern bool fieldPrintLookup(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  int64_t value;
  int64_t maxValue;

  // Can't use extractNumberNotEmpty when the lookup key might use the 'error/unknown' values.
  if (!extractNumber(field, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }

  if (field->unit && field->unit[0] == '=' && isdigit(field->unit[1]))
  {
    char        lookfor[20];
    const char *s;

    sprintf(lookfor, "=%" PRId64, value);
    if (strcmp(lookfor, field->unit) != 0)
    {
      logDebug("Field %s value %" PRId64 " does not match %s\n", fieldName, value, field->unit + 1);
      g_skip = true;
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

  if (field->lookup.type != LOOKUP_TYPE_NONE && value >= 0)
  {
    if (field->lookup.type == LOOKUP_TYPE_PAIR)
    {
      s = (*field->lookup.function.pair)((size_t) value);
    }
    else /* TRIPLET */
    {
      int64_t val1;

      logDebug("Triplet extraction for field '%s'\n", field->name);

      if (extractNumberByOrder(field->pgn, field->lookup.val1Order, data, dataLen, &val1))
      {
        s = (*field->lookup.function.triplet)((size_t) val1, (size_t) value);
      }
    }
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
    if (*bits > 1 && (value >= maxValue - (*bits > 2 ? 2 : 1)))
    {
      printEmpty(fieldName, value - maxValue);
    }
    else if (showJson)
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

/*
 * Only print reserved fields if they are NOT all ones, in that case we have an incorrect
 * PGN definition.
 */
extern bool fieldPrintReserved(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  int64_t value;
  int64_t maxValue;

  if (!extractNumber(field, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }
  if (value == maxValue)
  {
    g_skip = true;
    return true;
  }

  return fieldPrintBinary(field, fieldName, data, dataLen, startBit, bits);
}

/*
 * Only print spare fields if they are NOT all zeroes, in that case we have an incorrect
 * PGN definition.
 */
extern bool fieldPrintSpare(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  int64_t value;
  int64_t maxValue;

  if (!extractNumber(field, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }
  if (value == 0)
  {
    g_skip = true;
    return true;
  }

  return fieldPrintBinary(field, fieldName, data, dataLen, startBit, bits);
}

extern bool fieldPrintBitLookup(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  int64_t value;
  int64_t maxValue;
  int64_t bitValue;
  size_t  bit;
  char    sep;

  if (!extractNumber(field, data, dataLen, startBit, *bits, &value, &maxValue))
  {
    return true;
  }
  if (value == 0)
  {
    printEmpty(fieldName, value - maxValue);
    return true;
  }

  logDebug("RES_BITFIELD length %u value %" PRIx64 "\n", *bits, value);

  if (showJson)
  {
    mprintf("%s\"%s\": ", getSep(), fieldName);
    sep = '[';
  }
  else
  {
    mprintf("%s %s =", getSep(), fieldName);
    sep = ' ';
  }

  for (bitValue = 1, bit = 0; bitValue <= maxValue; (bitValue *= 2), bit++)
  {
    bool isSet = (value & bitValue) != 0;
    logDebug("RES_BITFIELD is bit %u value %" PRIx64 " set? = %d\n", bit, bitValue, isSet);
    if (isSet)
    {
      const char *s = (*field->lookup.function.pair)(bit);

      if (s != NULL)
      {
        if (showJsonValue)
        {
          mprintf("%c{\"value\":%" PRId64 ",\"name\":\"%s\"}", sep, bitValue, s);
        }
        else if (showJson)
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
    dd = (double) value * field->resolution;

    if (showJson)
    {
      mprintf("%s\"%s\":%10.7f", getSep(), fieldName, dd);
    }
    else
    {
      mprintf("%s %s = %10.7f", getSep(), fieldName, dd);
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
    dd        = (double) absVal * field->resolution;
    degrees   = floor(dd);
    remainder = dd - degrees;
    minutes   = floor(remainder * 60.);
    seconds   = floor(remainder * 3600.) - 60. * minutes;

    mprintf((showJson ? "%s\"%s\":\"%02u&deg;%02u&rsquo;%06.3f&rdquo;%c\"" : "%s %s = %02ud %02u' %06.3f\"%c"),
            getSep(),
            fieldName,
            (int) degrees,
            (int) minutes,
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

  if (!adjustDataLenStart(&data, &dataLen, &startBit))
  {
    return false;
  }

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

static void print_ascii_json_escaped(uint8_t *data, int len)
{
  int c;
  int k;

  for (k = 0; k < len; k++)
  {
    c = data[k];
    switch (c)
    {
      case '\b':
        mprintf("%s", "\\b");
        break;

      case '\n':
        mprintf("%s", "\\n");
        break;

      case '\r':
        mprintf("%s", "\\r");
        break;

      case '\t':
        mprintf("%s", "\\t");
        break;

      case '\f':
        mprintf("%s", "\\f");
        break;

      case '"':
        mprintf("%s", "\\\"");
        break;

      case '\\':
        mprintf("%s", "\\\\");
        break;

      case '/':
        mprintf("%s", "\\/");
        break;

      case '\377':
        // 0xff has been seen on recent Simrad VHF systems, and it seems to indicate
        // end-of-field, with noise following. Assume this does not break other systems.
        return;

      default:
        if (c >= ' ' && c <= '~')
          mprintf("%c", c);
    }
  }
}

static bool printString(char *fieldName, uint8_t *data, size_t len)
{
  int     k;
  uint8_t lastbyte;

  if (len > 0)
  {
    // rtrim funny stuff from end, we see all sorts
    lastbyte = data[len - 1];
    if (lastbyte == 0xff || isspace(lastbyte) || lastbyte == 0 || lastbyte == '@')
    {
      while (len > 0 && (data[len - 1] == lastbyte))
      {
        len--;
      }
    }
  }

  if (len == 0)
  {
    printEmpty(fieldName, DATAFIELD_UNKNOWN);
    return true;
  }

  if (showJson)
  {
    mprintf("%s\"%s\":\"", getSep(), fieldName);
    print_ascii_json_escaped(data, len);
    mprintf("\"");
  }
  else
  {
    mprintf("%s %s = ", getSep(), fieldName);
    for (k = 0; k < len; k++)
    {
      if (data[k] == 0xff)
      {
        break;
      }
      if (data[k] >= ' ' && data[k] <= '~')
      {
        mprintf("%c", data[k]);
      }
    }
  }

  return true;
}

/**
 * Fixed length string where the length is defined by the field definition.
 */
extern bool fieldPrintStringFix(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  size_t len = field->size / 8;

  if (!adjustDataLenStart(&data, &dataLen, &startBit))
  {
    return false;
  }

  logDebug("fieldPrintStringFix('%s',%zu) size=%zu\n", fieldName, dataLen, len);

  len   = CB_MIN(len, dataLen); // Cap length to remaining bytes in message
  *bits = BYTES(len);
  return printString(fieldName, data, len);
}

extern bool fieldPrintStringVar(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  size_t len;

  if (!adjustDataLenStart(&data, &dataLen, &startBit))
  {
    return false;
  }

  // No space in message for string, don't print anything
  if (dataLen == 0)
  {
    len = 0;
  }
  else
  {
    // STRINGVAR format is <start> [ <data> ... ] <stop>
    //                  <len> [ <data> ... ] (with len > 2)
    //                  <stop>                                 zero length data
    //                  <#00>  ???
    if (*data == 0x02)
    {
      data++;
      dataLen--;
      for (len = 0; len < dataLen && data[len] != 0x01; len++)
        ;
      dataLen = len + 2;
    }
    else if (*data > 0x02)
    {
      dataLen = *data++;
      len     = dataLen - 1;

      // This is actually more like a STRINGLAU control byte, not sure
      // whether these fields are actually just STRINGLAU?
      if (*data == 0x01)
      {
        logDebug("field '%s' looks like STRING_LAU not STRING_VAR format\n", fieldName);
        data++;
        len--;
      }
    }
    else
    {
      dataLen = 1;
      len     = 0;
    }
  }

  *bits = BYTES(dataLen);

  return printString(fieldName, data, len);
}

extern bool fieldPrintStringLZ(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  // STRINGLZ format is <len> [ <data> ... ]
  size_t len;

  if (!adjustDataLenStart(&data, &dataLen, &startBit))
  {
    return false;
  }

  // Cap to dataLen
  len   = *data++;
  len   = CB_MIN(len, dataLen - 1);
  *bits = BYTES(len + 1);

  return printString(fieldName, data, len);
}

extern bool fieldPrintStringLAU(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  // STRINGLAU format is <len> <control> [ <data> ... ]
  // where <control> == 0 = UTF16
  //       <control> == 1 = ASCII(?) or maybe UTF8?
  int     control;
  size_t  len;
  size_t  utf8_len;
  utf8_t *utf8 = NULL;
  bool    r;

  if (!adjustDataLenStart(&data, &dataLen, &startBit))
  {
    return false;
  }

  len     = *data++;
  control = *data++;
  if (len < 2 || dataLen < 2)
  {
    logError("field '%s': Invalid string length %u in STRING_LAU field\n", fieldName, len);
    return false;
  }
  len = len - 2;
  // Cap to dataLen
  len   = CB_MIN(len, dataLen - 2);
  *bits = BYTES(len + 2);

  if (control == 0)
  {
    utf8_len = utf16_to_utf8((const utf16_t *) data, len / 2, NULL, 0);
    utf8     = malloc(utf8_len + 1);

    if (utf8 == NULL)
    {
      die("Out of memory");
    }
    len = utf16_to_utf8((const utf16_t *) data, len / 2, utf8, utf8_len + 1);
  }
  else if (control > 1)
  {
    logError("Unhandled string type %d in PGN\n", control);
    return false;
  }

  r = printString(fieldName, data, len);
  if (utf8 != NULL)
  {
    free(utf8);
  }
  return r;
}

extern bool fieldPrintBinary(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  size_t      i;
  size_t      remaining_bits;
  const char *s;

  if (!adjustDataLenStart(&data, &dataLen, &startBit))
  {
    return false;
  }

  if (startBit + *bits > dataLen * 8)
  {
    *bits = dataLen * 8 - startBit;
  }

  if (showJson)
  {
    mprintf("%s\"%s\":\"", getSep(), fieldName);
  }
  else
  {
    mprintf("%s %s = ", getSep(), fieldName);
  }
  remaining_bits = *bits;
  s              = "";
  for (i = 0; i < (*bits + 7) >> 3; i++)
  {
    uint8_t byte = data[i];

    if (i == 0 && startBit != 0)
    {
      byte = byte >> startBit; // Shift off older bits
      if (remaining_bits + startBit < 8)
      {
        byte = byte & ((1 << remaining_bits) - 1);
      }
      byte = byte << startBit; // Shift zeros back in
      remaining_bits -= (8 - startBit);
    }
    else
    {
      if (remaining_bits < 8)
      {
        // only the lower remaining_bits should be used
        byte = byte & ((1 << remaining_bits) - 1);
      }
      remaining_bits -= 8;
    }
    mprintf("%s%2.02X", s, byte);
    s = " ";
  }
  if (showJson)
  {
    mprintf("\"");
  }
  return true;
}
