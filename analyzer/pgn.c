/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2025, Kees Verruijt, Harlingen, The Netherlands.

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
const Pgn *searchForPgn(int pgn)
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
      if (mid == 0)
      {
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
const Pgn *searchForUnknownPgn(int pgnId)
{
  Pgn *fallback = pgnList;
  Pgn *pgn;

  for (pgn = pgnList; pgn < pgnList + pgnListSize; pgn++)
  {
    if (pgn->fallback)
    {
      fallback = pgn;
    }
    if (pgn->pgn >= pgnId)
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
const Pgn *getMatchingPgn(int pgnId, const uint8_t *data, int length)
{
  const Pgn *pgn = searchForPgn(pgnId);
  int        prn;
  int        i;

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
    int        pgnRangeIndex = 0;
    int        prn           = pgnList[i].pgn;
    const Pgn *pgn;

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

const Field *getField(uint32_t pgnId, uint32_t field)
{
  const Pgn *pgn = searchForPgn(pgnId);

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
    if (isalpha((unsigned char) *s) || isdigit((unsigned char) *s))
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

      if ( pgnList[i].fieldList[j].camelName == NULL )
      {
          pgnList[i].fieldList[j].camelName = camelize(name, upperCamelCase, haveEarlierSpareOrReserved ? j + 1 : 0);
      }
      
      if (strcmp(name, "Reserved") == 0 || strcmp(name, "Spare") == 0)
      {
        haveEarlierSpareOrReserved = true;
      }
    }
  }
}
