/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

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
  // Half-open range [start, end): end is the count, not the last index, so an
  // out-of-range pgn converges to start == end and returns NULL rather than
  // dereferencing pgnList[pgnListSize] one past the array.
  size_t start = 0;
  size_t end   = pgnListSize;
  size_t mid;

  while (start < end)
  {
    mid = start + (end - start) / 2;
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
        if (mid >= pgnListSize || pgn != pgnList[mid].pgn)
        {
          return NULL;
        }
      }
      return &pgnList[mid];
    }
    if (pgn < pgnList[mid].pgn)
    {
      end = mid;
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
    /* Stop before taking a catch-all that starts above pgnId: when nothing is
     * defined between a range's catch-all and the next one (J1939 has nothing
     * past 0xFF00), the next one is a different range, e.g. a fast-packet
     * catch-all for a single-frame PGN, which then never completes.
     */
    if (pgn->pgn > pgnId)
    {
      break;
    }
    if (pgn->fallback)
    {
      fallback = pgn;
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

      if (field->hasMatchValue)
      {
        int64_t value, desiredValue;
        int64_t maxValue;

        hasFixedField = true;
        desiredValue  = field->matchValue;
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

/*
 * Return the best PGN for this prn, based on the PRN and possibly the ISO request/command
 * style data containing fields in the requested PGN.
 *
 * Note that CompanyId and IndustryCode are just normal "match" parameters, so do not
 * need to be treated really differently.
 */
/*
 * The field of `pgn` that a group function parameter addresses: `index` is the
 * parameter's field number, 1-based. A number past the fixed fields continues
 * into the definition's repeating set, as its fields repeat, when it has one
 * set and that set ends the definition. NULL when there is no such field.
 */
const Field *getParameterField(const Pgn *pgn, int index)
{
  int start = pgn->repeatingStart1;
  int count = pgn->repeatingCount1;

  if (index < 1)
  {
    return NULL;
  }
  if (index <= (int) pgn->fieldCount)
  {
    return &pgn->fieldList[index - 1];
  }
  if (count == 0 || pgn->repeatingCount2 != 0 || start + count - 1 != (int) pgn->fieldCount)
  {
    return NULL;
  }
  return &pgn->fieldList[(index - start) % count + start - 1];
}

/*
 * The definition of PGN `pgnId` that a 126208 group function's parameter list
 * describes. `data` starts at the parameter count, followed by that many
 * {field index, value} pairs. The first definition whose match fields all
 * agree with the parameters given wins. For a proprietary PGN the Manufacturer
 * (parameter 1) is required; its Industry Code comes from parameter 3 or, when
 * no parameter 3 is given, from parameter 1's top bits. Returns NULL when
 * nothing matches.
 */
const Pgn *getMatchingPgnByParameters(int pgnId, const uint8_t *data, int length)
{
  const Pgn *pgn = searchForPgn(pgnId);
  int        prn;
  int        d;
  int        industry = -1; // the Industry Code, when it comes inside parameter 1 rather than as parameter 3

  if (pgn == NULL)
  {
    return pgn;
  }

  if (IS_PGN_PROPRIETARY(pgnId))
  {
    // For proprietary PGNs we need to do more work, skip through the list until we
    // get to the correct company
    //
    // Data should be at least:
    // [0]    = # of fields, at least 2
    // [1]    = 0x01 = field 1 = Company Id
    // [2..3] = company id
    // [4]    = 0x03 = field 3 = Industry Code
    // [5]    = industry code
    //
    // Some devices (an Airmar setup tool, say) give only parameter 1, its two
    // bytes holding the whole header word: the 11-bit company, 2 reserved bits
    // and the 3-bit Industry Code in the top bits. Then the Industry Code is
    // taken from there.
    if (length < 4 || data[0] < 1 || data[1] != 0x01)
    {
      logError("PGN %u: refers to proprietary PGN but does not contain Company and Industry field values\n", pgnId);
      return NULL;
    }
    if (length < 6 || data[0] < 2 || data[4] != 3)
    {
      industry = (data[3] >> 5) & 0x07;
    }
  }

  if (!pgn->hasMatchFields)
  {
    logDebug("getMatchingPgnByParameters: PGN %u has no match fields, returning '%s'\n", pgnId, pgn->description);
    return pgn;
  }

  // Here if we have a PGN but it must be matched to the list of match fields.
  // This might end up without a solution, in that case return NULL.

  for (prn = pgn->pgn; pgn->pgn == prn; pgn++)
  {
    bool matchedFixedField = true;
    bool explicitIndustry  = false; // a parameter 3 anywhere in the list

    logDebug("getMatchingPgnByParameters: PGN %u parameters %d try match with manufacturer specific '%s'\n",
             prn,
             data[0],
             pgn->description);

    // Iterate over the parameter list and try to find a matching definition where all match
    // parameters are found; we can stop after the first non-match parameter.
    //
    // data[0] is the parameter count: iterate exactly that many {field index, value} pairs and no
    // further. Walking to the end of the data instead would misread trailing bytes (e.g. a wider
    // value field than this definition assumes, or fast-packet padding) as a bogus extra parameter
    // whose "field index" is out of range, spuriously rejecting an otherwise-matching definition.
    int nparams = data[0];
    int param;

    for (d = 1, param = 0; param < nparams && d < length; param++)
    {
      int index = data[d++];

      if (index == 3)
      {
        explicitIndustry = true;
      }
      logDebug("getMatchingPgnByParameters: offset %d parameter #%d\n", d, index);
      const Field *field = getParameterField(pgn, index);
      if (field == NULL)
      {
        matchedFixedField = false;
        break;
      }

      int          bits  = field->size;
      int          bytes = (bits + 7) >> 3;

      logDebug("getMatchingPgnByParameters: parameter #%d = '%s' length %d\n", index, field->description, bytes);
      if (field->hasMatchValue)
      {
        int64_t value, desiredValue;
        int64_t maxValue;

        desiredValue = field->matchValue;
        if (!extractNumber(field, data, length, d << 3, field->size, &value, &maxValue) || value != desiredValue)
        {
          logDebug("getMatchingPgnByParameters: PGN %u field '%s' value %" PRId64 " does not match %" PRId64 "\n",
                   prn,
                   field->name,
                   value,
                   desiredValue);
          matchedFixedField = false;
          break;
        }
        logDebug("getMatchingPgnByParameters: PGN %u field '%s' value %" PRId64 " matches %" PRId64 "\n",
                 prn,
                 field->name,
                 value,
                 desiredValue);
      }
      d += bytes;
    }
    if (matchedFixedField && industry >= 0 && !explicitIndustry)
    {
      const Field *field = getParameterField(pgn, 3);

      if (field != NULL && field->hasMatchValue && field->matchValue != industry)
      {
        logDebug("getMatchingPgnByParameters: PGN %u industry %d in parameter 1 does not match %" PRId64 "\n",
                 prn,
                 industry,
                 field->matchValue);
        matchedFixedField = false;
      }
    }
    if (matchedFixedField)
    {
      logDebug("getMatchingPgnByParameters: PGN %u selected manufacturer specific '%s'\n", prn, pgn->description);
      return pgn;
    }
  }

  return NULL;
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
