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
    mprintf("%s\"%s\":null", getSep(), fieldName);
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
    const char *fmt = "%" PRId64;

    fmt = (field->ft->format != NULL) ? field->ft->format : "%" PRId64;

    if (showJson)
    {
      mprintf("%s\"%s\":", getSep(), fieldName);
      mprintf(fmt, value);
    }
    else
    {
      mprintf("%s %s = ", getSep(), fieldName);
      mprintf(fmt, value);
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
  union
  {
    float    a;
    uint32_t w;
    uint8_t  b[4];
  } f;

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
    if (field->units)
    {
      mprintf(" %s", field->units);
    }
  }

  return true;
}
extern bool fieldPrintDecimal(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  uint8_t  value        = 0;
  uint8_t  bitMask      = 1 << startBit;
  uint64_t bitMagnitude = 1;
  size_t   bit;
  char     buf[128];

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
      logDebug("Field %s value %" PRId64 " does not match %s\n", fieldName, value, field->units + 1);
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

  logDebug("fieldPrintStringFix('%s',%zu) size=%zu\n", fieldName, dataLen, len);

  len   = CB_MIN(len, dataLen); // Cap length to remaining bytes in message
  *bits = BYTES(len);
  return printString(fieldName, data, len);
}

extern bool fieldPrintStringVar(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  size_t len;

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
  size_t len = *data++;

  // Cap to dataLen
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

extern bool fieldPrintVariable(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return true;
}
