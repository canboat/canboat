
#include "analyzer.h"
#include <common.h>

Pgn *searchForPgn(int pgn)
{
  size_t start = 0;
  size_t end = pgnListSize;
  size_t mid;

  while (start <= end)
  {
    mid = (start + end) / 2;
    if (pgn == pgnList[mid].pgn)
    {
      while (mid && pgn == pgnList[mid - 1].pgn)
      {
        mid--;
      }
      return &pgnList[mid];
    }
    if (pgn < pgnList[mid].pgn)
    {
      end = mid - 1;
    }
    else
    {
      start = mid + 1;
    }
  }
  return 0;
}

/**
 * Return the last Pgn entry for which unknown == true && prn is smaller than requested.
 * This is slower, but is not used often.
 */
static Pgn *searchForUnknownPgn(int pgnId)
{
  Pgn *unknown = pgnList;
  Pgn *pgn;

  for (pgn = pgnList; pgn < pgnList + pgnListSize; pgn++)
  {
    if (pgn->unknownPgn)
    {
      unknown = pgn;
    }
    if (pgn->pgn > pgnId)
    {
      break;
    }
  }
  return unknown;
}

Pgn* getMatchingPgn(int pgnId, uint8_t *dataStart, int length)
{
  Pgn *pgn = searchForPgn(pgnId);
  int prn;
  int i;

  if (!pgn)
  {
    pgn = searchForUnknownPgn(pgnId);
  }

  prn = pgn->pgn;

  if (pgn == pgnListEnd() - 1 || pgn[1].pgn != prn)
  {
    // Don't bother complex search if there is only one PGN with this PRN.
    return pgn;
  }

  for (; pgn->pgn == prn; pgn++) // we never get here for the last pgn, so no need to check for end of list
  {
    int startBit = 0;
    uint8_t *data = dataStart;

    bool matchedFixedField = true;
    bool hasFixedField = false;

    /* There is a next index that we can use as well. We do so if the 'fixed' fields don't match */

    if (!pgn->fieldCount)
    {
      logError("Internal error: %p PGN %d offset %u '%s' has no fields\n", pgn, prn, (unsigned) (pgn - pgnList), pgn->description);
      for (i = 0; pgn->fieldList[i].name; i++)
      {
        logInfo("Field %d: %s\n", i, pgn->fieldList[i].name);
      }
      // exit(2);
      pgn->fieldCount = i;
    }

    // Iterate over fields
    for (i = 0, startBit = 0, data = dataStart; i < pgn->fieldCount; i++)
    {
      const Field *field = &pgn->fieldList[i];
      int bits = field->size;

      if (field->units && field->units[0] == '=')
      {
        int64_t value, desiredValue;
        int64_t maxValue;

        hasFixedField = true;
        extractNumber(field, data, startBit, field->size, &value, &maxValue);
        desiredValue = strtol(field->units + 1, 0, 10);
        if (value != desiredValue)
        {
          matchedFixedField = false;
          break;
        }
      }
      startBit += bits;
      data += startBit / 8;
      startBit %= 8;
    }
    if (!hasFixedField)
    {
      logDebug("Cant determine prn choice, return prn=%d variation '%s'\n", prn, pgn->description);
      return pgn;
    }
    if (matchedFixedField)
    {
      return pgn;
    }
  }
  return 0;
}

void checkPgnList(void)
{
  size_t i;
  int prn = 0;

  for (i = 0; i < pgnListSize; i++)
  {
    Pgn * pgn;

    if (pgnList[i].pgn < prn)
    {
      logError("Internal error: PGN %d is not sorted correctly\n", pgnList[i].pgn);
      exit(2);
    }
    if (pgnList[i].pgn == prn)
    {
      continue;
    }
    prn = pgnList[i].pgn;
    pgn = searchForPgn(prn);
    if (pgn != &pgnList[i])
    {
      logError("Internal error: PGN %d is not found correctly\n", prn);
      exit(2);
    }
  }
}

Field * getField(uint32_t pgnId, uint32_t field)
{

  Pgn* pgn = searchForPgn(pgnId);

  if (!pgn)
  {
    return 0;
  }
  if (field < pgn->fieldCount)
  {
    return pgn->fieldList + field;
  }
  if (pgn->repeatingFields)
  {
    uint32_t startOfRepeatingFields = pgn->fieldCount - pgn->repeatingFields;
    uint32_t index = startOfRepeatingFields + ((field - startOfRepeatingFields) % pgn->repeatingFields);

    return pgn->fieldList + index;
  }
  return 0;
}

/*
 *
 * This is perhaps as good a place as any to explain how CAN messages are layed out by the
 * NMEA. Basically, it's a mess once the bytes are recomposed into bytes (the on-the-wire
 * format is fine).
 *
 * For fields that are aligned on bytes there isn't much of an issue, they appear in our
 * buffers in standard Intel 'least endian' format.
 * For instance the MMSI # 244050447 is, in hex: 0x0E8BEA0F. This will be found in the CAN data as:
 * byte x+0: 0x0F
 * byte x+1: 0xEA
 * byte x+2: 0x8B
 * byte x+3: 0x0e
 *
 * To gather together we loop over the bytes, and keep increasing the magnitude of what we are
 * adding:
 *    for (i = 0, magnitude = 0; i < 4; i++)
 *    {
 *      value += data[i] << magnitude;
 *      magnitude += 8;
 *    }
 *
 * However, when there are two bit fields after each other, lets say A of 2 and then B of 6 bits:
 * then that is layed out MSB first, so the bit mask is 0b11000000 for the first
 * field and 0b00111111 for the second field.
 *
 * This means that if we have a bit field that crosses a byte boundary and does not start on
 * a byte boundary, the bit masks are like this (for a 16 bit field starting at the 3rd bit):
 *
 * 0b00111111 0b11111111 0b11000000
 *     ------   --------   --
 *     000000   11110000   11
 *     543210   32109876   54
 *
 * So we are forced to mask bits 0 and 1 of the first byte. Since we need to process the previous
 * field first, we cannot repeatedly shift bits out of the byte: if we shift left we get the first
 * field first, but in MSB order. We need bit values in LSB order, as the next byte will be more
 * significant. But we can't shift right as that will give us bits in LSB order but then we get the
 * two fields in the wrong order...
 *
 * So for that reason we explicitly test, per byte, how many bits we need and how many we have already
 * used.
 *
 */

void extractNumber(const Field * field, uint8_t * data, size_t startBit, size_t bits, int64_t * value, int64_t * maxValue)
{
  bool hasSign = field->hasSign;

  size_t firstBit = startBit;
  size_t bitsRemaining = bits;
  size_t magnitude = 0;
  size_t bitsInThisByte;
  uint64_t bitMask;
  uint64_t allOnes;
  uint64_t valueInThisByte;
  uint64_t maxv;

  *value = 0;
  maxv = 0;

  while (bitsRemaining)
  {
    bitsInThisByte = min(8 - firstBit, bitsRemaining);
    allOnes = (uint64_t) ((((uint64_t) 1) << bitsInThisByte) - 1);

    //How are bits ordered in bytes for bit fields? There are two ways, first field at LSB or first
    //field as MSB.
    //Experimentation, using the 129026 PGN, has shown that the most likely candidate is LSB.
    bitMask = allOnes << firstBit;
    valueInThisByte = (*data & bitMask) >> firstBit;

    *value |= valueInThisByte << magnitude;
    maxv |= allOnes << magnitude;

    magnitude += bitsInThisByte;
    bitsRemaining -= bitsInThisByte;
    firstBit += bitsInThisByte;
    if (firstBit >= 8)
    {
      firstBit -= 8;
      data++;
    }
  }

  if (hasSign)
  {
    maxv >>= 1;

    if (field->offset) /* J1939 Excess-K notation */
    {
      *value += field->offset;
    }
    else
    {
      bool negative = (*value & (((uint64_t) 1) << (bits - 1))) > 0;

      if (negative)
      {
        /* Sign extend value for cases where bits < 64 */
        /* Assume we have bits = 16 and value = -2 then we do: */
        /* 0000.0000.0000.0000.0111.1111.1111.1101 value    */
        /* 0000.0000.0000.0000.0111.1111.1111.1111 maxvalue */
        /* 1111.1111.1111.1111.1000.0000.0000.0000 ~maxvalue */
        *value |= ~*maxValue;
      }
    }
  }

  *maxValue = (int64_t) maxv;
}

static char * findFirstOccurrence(char * msg, char c)
{
  if (*msg == 0 || *msg == '\n')
  {
    return 0;
  }
  return strchr(msg, c);
}

static int setParsedValues(RawMessage * m, unsigned int prio, unsigned int pgn, unsigned int dst, unsigned int src, unsigned int len)
{
  m->prio = prio;
  m->pgn  = pgn;
  m->dst  = dst;
  m->src  = src;
  m->len  = len;

  return 0;
}

int parseRawFormatPlain(char * msg, RawMessage * m, bool showJson)
{
  unsigned int prio, pgn, dst, src, len, junk, r, i;
  char * p;
  unsigned int data[8];

  p = findFirstOccurrence(msg, ',');
  if(!p)
  {
    return 1;
  }

  memcpy(m->timestamp, msg, p - msg);
  m->timestamp[p - msg] = 0;

  /* Moronic Windows does not support %hh<type> so we use intermediate variables */
  r = sscanf( p
    , ",%u,%u,%u,%u,%u"
    ",%x,%x,%x,%x,%x,%x,%x,%x,%x"
    , &prio
    , &pgn
    , &src
    , &dst
    , &len
    , &data[0]
    , &data[1]
    , &data[2]
    , &data[3]
    , &data[4]
    , &data[5]
    , &data[6]
    , &data[7]
    , &junk
  );
  if (r < 5)
  {
    logError("Error reading message, scanned %u from %s", r, msg);
    if (!showJson) fprintf(stdout, "%s", msg);
    return 2;
  }

  if (r <= 5 + 8)
  {
    for (i = 0; i < len; i++)
    {
      m->data[i] = data[i];
    }
  }
  else
  {
    return -1;
  }

  return setParsedValues(m, prio, pgn, dst, src, len);
}

int parseRawFormatFast(char * msg, RawMessage * m, bool showJson)
{
  unsigned int prio, pgn, dst, src, len, r, i;
  char * p;

  p = findFirstOccurrence(msg, ',');
  if(!p)
  {
    return 1;
  }

  memcpy(m->timestamp, msg, p - msg);
  m->timestamp[p - msg] = 0;

  /* Moronic Windows does not support %hh<type> so we use intermediate variables */
  r = sscanf( p
    , ",%u,%u,%u,%u,%u "
    , &prio
    , &pgn
    , &src
    , &dst
    , &len
  );
  if (r < 5)
  {
    logError("Error reading message, scanned %u from %s", r, msg);
    if (!showJson) fprintf(stdout, "%s", msg);
    return 2;
  }
  for (i = 0; *p && i < 5;)
  {
    if (*++p == ',')
    {
      i++;
    }
  }
  if (!p)
  {
    logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
    if (!showJson) fprintf(stdout, "%s", msg);
    return 2;
  }
  p++;
  for (i = 0; i < len; i++)
  {
    if (scanHex(&p, &m->data[i]))
    {
      logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
      if (!showJson) fprintf(stdout, "%s", msg);
      return 2;
    }
    if (i < len)
    {
      if (*p != ',' && !isspace(*p))
      {
        logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
        if (!showJson) fprintf(stdout, "%s", msg);
        return 2;
      }
      p++;
    }
  }

  return setParsedValues(m, prio, pgn, dst, src, len);
}

int parseRawFormatAirmar(char * msg, RawMessage * m, bool showJson)
{
  unsigned int prio, pgn, dst, src, len, i;
  char * p;
  unsigned int id;

  p = findFirstOccurrence(msg, ' ');
  if (p < msg + 4 || p >= msg + sizeof(m->timestamp))
  {
    return 1;
  }

  memcpy(m->timestamp, msg, p - msg - 1);
  m->timestamp[p - msg - 1] = 0;
  p += 3;

  /* Moronic Windows does not support %hh<type> so we use intermediate variables */
  pgn = strtoul(p, &p, 10);
  if (*p == ' ')
  {
    id = strtoul(++p, &p, 16);
  }
  if (*p != ' ')
  {
    logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
    if (!showJson) fprintf(stdout, "%s", msg);
    return 2;
  }

  getISO11783BitsFromCanId(id, &prio, &pgn, &src, &dst);

  p++;
  len = strlen(p) / 2;
  for (i = 0; i < len; i++)
  {
    if (scanHex(&p, &m->data[i]))
    {
      logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
      if (!showJson) fprintf(stdout, "%s", msg);
      return 2;
    }
    if (i < len)
    {
      if (*p != ',' && *p != ' ')
      {
        logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
        if (!showJson) fprintf(stdout, "%s", msg);
        return 2;
      }
      p++;
    }
  }

  return setParsedValues(m, prio, pgn, dst, src, len);
}

int parseRawFormatChetco(char * msg, RawMessage * m, bool showJson)
{
  unsigned int pgn, src, i;
  unsigned int tstamp;
  time_t t;
  struct tm tm;
  char * p;

  if (*msg == 0 || *msg == '\n')
  {
    return 1;
  }

  if (sscanf(msg, "$PCDIN,%x,%x,%x,", &pgn, &tstamp, &src) < 3)
  {
    logError("Error reading Chetco message: %s", msg);
    if (!showJson) fprintf(stdout, "%s", msg);
    return 2;
  }

  t = (time_t) tstamp / 1000;
  localtime_r(&t, &tm);
  strftime(m->timestamp, sizeof(m->timestamp), "%Y-%m-%d-%H:%M:%S", &tm);
  sprintf(m->timestamp + strlen(m->timestamp), ",%u", tstamp % 1000);

  p = msg + STRSIZE("$PCDIN,01FD07,089C77D!,03,"); // Fixed length where data bytes start;

  for (i = 0; *p != '*'; i++)
  {
    if (scanHex(&p, &m->data[i]))
    {
      logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
      if (!showJson) fprintf(stdout, "%s", msg);
      return 2;
    }
  }

  return setParsedValues(m, 0, pgn, 255, src, i + 1);
}
