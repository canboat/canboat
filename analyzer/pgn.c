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

#include <common.h>

#include "analyzer.h"

/**
 * Return the first Pgn entry for which the pgn is found.
 * There can be multiple (with differing 'match' fields).
 */
Pgn *searchForPgn(int pgn)
{
  size_t start = 0;
  size_t end   = pgnListSize;
  size_t mid;

  while (start <= end)
  {
    mid = (start + end) / 2;
    if (pgn == pgnList[mid].pgn)
    {
      // Return the first one, unless it is the catch-all
      while (mid > 0 && pgn == pgnList[mid - 1].pgn)
      {
        mid--;
      }
      if (pgnList[mid].fallback)
      {
        mid++;
        if (pgn != pgnList[mid].pgn)
        {
          return NULL;
        }
      }
      return &pgnList[mid];
    }
    if (pgn < pgnList[mid].pgn)
    {
      if (mid == 0) {
        return NULL;
      }
      end = mid - 1;
    }
    else
    {
      start = mid + 1;
    }
  }
  return NULL;
}

/**
 * Return the last Pgn entry for which fallback == true && prn is smaller than requested.
 * This is slower, but is not used often.
 */
Pgn *searchForUnknownPgn(int pgnId)
{
  Pgn *fallback = pgnList;
  Pgn *pgn;

  for (pgn = pgnList; pgn < pgnList + pgnListSize; pgn++)
  {
    if (pgn->fallback)
    {
      fallback = pgn;
    }
    if (pgn->pgn > pgnId)
    {
      break;
    }
  }
  if (fallback == NULL)
  {
    logAbort("Cannot find catch-all PGN definition for PGN %d; internal definition error\n", pgnId);
  }
  logDebug("Found catch-all PGN %u for PGN %d\n", fallback->pgn, pgnId);
  return fallback;
}

/*
 * Return the best match for this pgnId.
 * If all else fails, return an 'fallback' match-all PGN that
 * matches the fast/single frame, PDU1/PDU2 and proprietary/generic range.
 */
Pgn *getMatchingPgn(int pgnId, uint8_t *data, int length)
{
  Pgn *pgn = searchForPgn(pgnId);
  int  prn;
  int  i;

  if (pgn == NULL)
  {
    pgn = searchForUnknownPgn(pgnId);
    logDebug("getMatchingPgn: Unknown PGN %u -> fallback %u\n", pgnId, (pgn != NULL) ? pgn->pgn : 0);
    return pgn;
  }

  if (!pgn->hasMatchFields)
  {
    logDebug("getMatchingPgn: PGN %u has no match fields, returning '%s'\n", pgnId, pgn->description);
    return pgn;
  }

  // Here if we have a PGN but it must be matched to the list of match fields.
  // This might end up without a solution, in that case return the catch-all fallback PGN.

  for (prn = pgn->pgn; pgn->pgn == prn; pgn++)
  {
    int  startBit          = 0;
    bool matchedFixedField = true;
    bool hasFixedField     = false;

    logDebug("getMatchingPgn: PGN %u matching with manufacturer specific '%s'\n", prn, pgn->description);

    // Iterate over fields
    for (i = 0, startBit = 0; i < pgn->fieldCount; i++)
    {
      const Field *field = &pgn->fieldList[i];
      int          bits  = field->size;

      if (field->unit != NULL && field->unit[0] == '=')
      {
        int64_t value, desiredValue;
        int64_t maxValue;

        hasFixedField = true;
        desiredValue  = strtol(field->unit + 1, 0, 10);
        if (!extractNumber(field, data, length, startBit, field->size, &value, &maxValue) || value != desiredValue)
        {
          logDebug("getMatchingPgn: PGN %u field '%s' value %" PRId64 " does not match %" PRId64 "\n",
                   prn,
                   field->name,
                   value,
                   desiredValue);
          matchedFixedField = false;
          break;
        }
        logDebug(
            "getMatchingPgn: PGN %u field '%s' value %" PRId64 " matches %" PRId64 "\n", prn, field->name, value, desiredValue);
      }
      startBit += bits;
    }
    if (!hasFixedField)
    {
      logDebug("getMatchingPgn: Cant determine prn choice, return prn=%d variation '%s'\n", prn, pgn->description);
      return pgn;
    }
    if (matchedFixedField)
    {
      logDebug("getMatchingPgn: PGN %u selected manufacturer specific '%s'\n", prn, pgn->description);
      return pgn;
    }
  }

  return searchForUnknownPgn(pgnId);
}

void checkPgnList(void)
{
  size_t i;
  int    prev_prn = 0;

  for (i = 0; i < pgnListSize; i++)
  {
    int  pgnRangeIndex = 0;
    int  prn           = pgnList[i].pgn;
    Pgn *pgn;

    if (prn < prev_prn)
    {
      logError("Internal error: PGN %d is not sorted correctly\n", prn);
      exit(2);
    }

    if (prn < ACTISENSE_BEM)
    {
      while (prn > pgnRange[pgnRangeIndex].pgnEnd && pgnRangeIndex < pgnRangeSize)
      {
        pgnRangeIndex++;
      }
      if (prn < pgnRange[pgnRangeIndex].pgnStart || prn > pgnRange[pgnRangeIndex].pgnEnd)
      {
        logError("Internal error: PGN %d is not part of a valid PRN range\n", prn);
        exit(2);
      }
      if (pgnRange[pgnRangeIndex].pgnStep == 256 && (prn & 0xff) != 0)
      {
        logError("Internal error: PGN %d (0x%x) is PDU1 and must have a PGN ending in 0x00\n", prn, prn);
        exit(2);
      }
      if (!(pgnRange[pgnRangeIndex].type == pgnList[i].type || pgnRange[pgnRangeIndex].type == PACKET_MIXED
            || pgnList[i].type == PACKET_ISO_TP))
      {
        logError("Internal error: PGN %d (0x%x) is in range 0x%x-0x%x and must have packet type %s\n",
                 prn,
                 prn,
                 pgnRange[pgnRangeIndex].pgnStart,
                 pgnRange[pgnRangeIndex].pgnEnd,
                 PACKET_TYPE_STR[pgnRange[pgnRangeIndex].type]);
        exit(2);
      }
    }

    if (prn == prev_prn || pgnList[i].fallback)
    {
      continue;
    }
    prev_prn = prn;
    pgn      = searchForPgn(prev_prn);
    if (pgn != &pgnList[i])
    {
      logError("Internal error: PGN %d is not found correctly\n", prev_prn);
      exit(2);
    }
  }
}

Field *getField(uint32_t pgnId, uint32_t field)
{
  Pgn *pgn = searchForPgn(pgnId);

  if (!pgn)
  {
    logDebug("PGN %u is unknown\n", pgnId);
    return 0;
  }
  if (field < pgn->fieldCount)
  {
    return pgn->fieldList + field;
  }
  logDebug("PGN %u does not have field %u\n", pgnId, field);
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

bool extractNumber(const Field *field,
                   uint8_t     *data,
                   size_t       dataLen,
                   size_t       startBit,
                   size_t       bits,
                   int64_t     *value,
                   int64_t     *maxValue)
{
  const bool  hasSign = field ? field->hasSign : false;
  const char *name    = field ? field->name : "<bits>";

  size_t   firstBit;
  size_t   bitsRemaining = bits;
  size_t   magnitude     = 0;
  size_t   bitsInThisByte;
  uint64_t bitMask;
  uint64_t allOnes;
  uint64_t valueInThisByte;
  uint64_t maxv;

  logDebug("extractNumber <%s> startBit=%zu bits=%zu\n", name, startBit, bits);

  if (!adjustDataLenStart(&data, &dataLen, &startBit))
  {
    return false;
  }

  firstBit = startBit;
  *value   = 0;
  maxv     = 0;

  while (bitsRemaining > 0 && dataLen > 0)
  {
    bitsInThisByte = min(8 - firstBit, bitsRemaining);
    allOnes        = (uint64_t) ((((uint64_t) 1) << bitsInThisByte) - 1);

    // How are bits ordered in bytes for bit fields? There are two ways, first field at LSB or first
    // field as MSB.
    // Experimentation, using the 129026 PGN, has shown that the most likely candidate is LSB.
    bitMask         = allOnes << firstBit;
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
      dataLen--;
    }
  }
  if (bitsRemaining > 0)
  {
    logDebug("Insufficient length in PGN to fill field '%s'\n", name);
    return false;
  }

  if (hasSign)
  {
    maxv >>= 1;

    if (field && field->offset) /* J1939 Excess-K notation */
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
        *value |= ~maxv;
      }
    }
  }

  *maxValue = (int64_t) maxv;

  logDebug("extractNumber <%s> startBit=%zu bits=%zu value=%" PRId64 " max=%" PRId64 "\n", name, startBit, bits, *value, *maxValue);

  return true;
}

static char *camelize(const char *str, bool upperCamelCase, int order)
{
  size_t      len         = strlen(str);
  char       *ptr         = malloc(len + 4);
  char       *p           = ptr;
  const char *s           = str;
  bool        lastIsAlpha = !upperCamelCase;

  if (p == NULL)
  {
    return NULL;
  }

  for (; *s; s++)
  {
    if (isalpha(*s) || isdigit(*s))
    {
      if (lastIsAlpha)
      {
        *p = tolower(*s);
      }
      else
      {
        *p          = toupper(*s);
        lastIsAlpha = true;
      }
      p++;
    }
    else
    {
      lastIsAlpha = false;
    }
  }

  if (order > 0 && (strcmp(str, "Reserved") == 0 || strcmp(str, "Spare") == 0))
  {
    sprintf(p, "%u", order);
  }
  else
  {
    *p = 0;
  }
  return ptr;
}

void camelCase(bool upperCamelCase)
{
  int  i, j;
  bool haveEarlierSpareOrReserved;

  for (i = 0; i < pgnListSize; i++)
  {
    pgnList[i].camelDescription = camelize(pgnList[i].description, upperCamelCase, 0);
    haveEarlierSpareOrReserved  = false;
    for (j = 0; j < ARRAY_SIZE(pgnList[i].fieldList) && pgnList[i].fieldList[j].name; j++)
    {
      const char *name = pgnList[i].fieldList[j].name;

      pgnList[i].fieldList[j].camelName = camelize(name, upperCamelCase, haveEarlierSpareOrReserved ? j + 1 : 0);
      if (strcmp(name, "Reserved") == 0 || strcmp(name, "Spare") == 0)
      {
        haveEarlierSpareOrReserved = true;
      }
    }
  }
}

