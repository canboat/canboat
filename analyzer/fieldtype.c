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

#define FIELDTYPE_GLOBALS

#include "analyzer.h"

extern FieldType *getFieldType(const char *name)
{
  for (size_t i = 0; i < fieldTypeCount; i++)
  {
    if (strcmp(name, fieldTypeList[i].name) == 0)
    {
      return &fieldTypeList[i];
    }
  }
  logError("fieldType '%s' not found\n", name);
  return NULL;
}

static bool isPhysicalQuantityListed(const PhysicalQuantity *pq)
{
  for (size_t i = 0; i < ARRAY_SIZE(PhysicalQuantityList); i++)
  {
    if (PhysicalQuantityList[i] == pq)
    {
      return true;
    }
  }
  return false;
}

static double getMinRange(const char *name, uint32_t size, double resolution, bool sign, int32_t offset)
{
  uint32_t highbit = (sign && offset == 0) ? (size - 1) : size;
  int64_t  minValue;
  double   r;

  if (!sign || offset != 0)
  {
    minValue = INT64_C(0) + offset;
    r        = minValue * resolution;
  }
  else
  {
    minValue = (UINT64_C(1) << highbit) - 1;
    r        = minValue * resolution * -1.0;
  }
  logDebug(
      "%s bits=%llu sign=%u minValue=%lld res=%g offset=%d -> rangeMin %g\n", name, highbit, sign, minValue, resolution, offset, r);
  return r;
}

#ifdef EXPLAIN
uint64_t g_max;

static void fillMaxRangeLookup(size_t n, const char *s)
{
  g_max = CB_MAX(g_max, n);
}
#endif

static double getMaxRange(const char *name, uint32_t size, double resolution, bool sign, int32_t offset, LookupInfo *lookup)
{
  uint64_t specialvalues = (size >= 4) ? 2 : (size >= 2) ? 1 : 0;
  uint32_t highbit       = (sign && offset == 0) ? (size - 1) : size;
  uint64_t maxValue;
  double   r;

  maxValue = (UINT64_C(1) << highbit) - 1 - specialvalues;
  if (offset != 0)
  {
    maxValue += offset;
  }

#ifdef EXPLAIN
  if (lookup != NULL && lookup->type == LOOKUP_TYPE_PAIR)
  {
    // maybe the specialValues are actually lookups, correct these.
    // Remember, when EXPLAIN is set the lookup function is like:
    // void lookupYES_NO(EnumPairCallback cb)
    // {
    //   (cb)(0, "No");
    //   (cb)(1, "Yes");
    // }
    g_max = maxValue;
    (*lookup->function.pairEnumerator)(fillMaxRangeLookup);
    maxValue = g_max;
  }
#endif

  r = maxValue * resolution;
  logDebug(
      "%s bits=%llu sign=%u maxValue=%lld res=%g offset=%d -> rangeMax %g\n", name, highbit, sign, maxValue, resolution, offset, r);
  return r;
}

static void fixupUnit(Field *f)
{
  if (showSI)
  {
    if (strcmp(f->unit, "kWh") == 0)
    {
      f->resolution *= 3.6e6; // 1 kWh = 3.6 MJ.
      f->rangeMin *= 3.6e6;
      f->rangeMax *= 3.6e6;
      f->unit = "J";
    }
    else if (strcmp(f->unit, "Ah") == 0)
    {
      f->resolution *= 3600.0; // 1 Ah = 3600 C.
      f->rangeMin *= 3600.0;
      f->rangeMax *= 3600.0;
      f->unit = "C";
    }

    // Many more to follow, but pgn.h is not yet complete enough...
  }
  else // NOT SI
  {
    if (strcmp(f->unit, "C") == 0)
    {
      f->resolution /= 3600.0; // 3600 C = 1 Ah
      f->rangeMin /= 3600.0;
      f->rangeMax /= 3600.0;
      f->unit = "Ah";
      logDebug("fixup <%s> to '%s'\n", f->name, f->unit);
    }
    else if (strcmp(f->unit, "Pa") == 0)
    {
      f->resolution /= 100000.0;
      f->rangeMin /= 100000.0;
      f->rangeMax /= 100000.0;
      f->precision = 3;
      f->unit      = "bar";
      logDebug("fixup <%s> to '%s'\n", f->name, f->unit);
    }
    else if (strcmp(f->unit, "K") == 0)
    {
      f->unitOffset = -273.15;
      f->rangeMin += -273.15;
      f->rangeMax += -275.15;
      f->precision = 2;
      f->unit      = "C";
      logDebug("fixup <%s> to '%s'\n", f->name, f->unit);
    }
    else if (strcmp(f->unit, "rad") == 0)
    {
      f->resolution *= RadianToDegree;
      f->rangeMin *= RadianToDegree;
      f->rangeMax *= RadianToDegree;
      f->unit      = "deg";
      f->precision = 1;
      logDebug("fixup <%s> to '%s'\n", f->name, f->unit);
    }
    else if (strcmp(f->unit, "rad/s") == 0)
    {
      f->resolution *= RadianToDegree;
      f->rangeMin *= RadianToDegree;
      f->rangeMax *= RadianToDegree;
      f->unit = "deg/s";
      logDebug("fixup <%s> to '%s'\n", f->name, f->unit);
    }
  }
}

extern void fillFieldType(bool doUnitFixup)
{
  // Percolate fields from physical quantity to fieldtype
  for (size_t i = 0; i < fieldTypeCount; i++)
  {
    FieldType *ft = &fieldTypeList[i];

    if (ft->physical != NULL)
    {
      if (!isPhysicalQuantityListed(ft->physical))
      {
        logAbort("FieldType '%s' contains an unlisted physical quantity '%s'\n", ft->name, ft->physical->name);
      }
      if (ft->unit == NULL)
      {
        ft->unit = ft->physical->abbreviation;
      }
      if (ft->url == NULL)
      {
        ft->url = ft->physical->url;
      }
    }
  }

  // Percolate fields from base to derived field
  for (size_t i = 0; i < fieldTypeCount; i++)
  {
    FieldType *ft = &fieldTypeList[i];

    logDebug("filling '%s'\n", ft->name);

    if (ft->baseFieldType != NULL)
    {
      FieldType *base = getFieldType(ft->baseFieldType);

      if (base == NULL)
      {
        logAbort("invalid baseFieldType '%s' found in FieldType '%s'\n", ft->baseFieldType, ft->name);
      }
      if (base > ft)
      {
        logAbort("invalid baseFieldType '%s' must be ordered before FieldType '%s'\n", ft->baseFieldType, ft->name);
      }
      ft->baseFieldTypePtr = base;

      // Inherit parent fields
      if (ft->physical == NULL)
      {
        ft->physical = base->physical;
      }
      if (ft->hasSign == Null && base->hasSign != Null)
      {
        ft->hasSign = base->hasSign;
      }
      if (ft->size == 0 && base->size != 0)
      {
        ft->size = base->size;
      }
      if (ft->resolution == 0.0 && base->resolution != 0.0)
      {
        ft->resolution = base->resolution;
      }
      else if (ft->resolution != 0.0 && base->resolution != 0.0 && ft->resolution != base->resolution)
      {
        logAbort("Cannot overrule resolution %g in '%s' with %g in '%s'\n", base->resolution, base->name, ft->resolution, ft->name);
      }
      if (ft->pf == NULL)
      {
        ft->pf = base->pf;
      }
    }

    if (ft->pf == NULL)
    {
      logAbort("FieldType '%s' has no print function\n", ft->name);
    }

    // Set the field range
    if (ft->size != 0 && ft->resolution != 0.0 && ft->hasSign != Null && ft->rangeMax == 0.0)
    {
      ft->rangeMin = getMinRange(ft->name, ft->size, ft->resolution, ft->hasSign == True, ft->offset);
      ft->rangeMax = getMaxRange(ft->name, ft->size, ft->resolution, ft->hasSign == True, ft->offset, NULL);
    }
    else
    {
      ft->rangeMin = nan("");
      ft->rangeMax = nan("");
    }
  }

  for (size_t i = 0; i < pgnListSize; i++)
  {
    uint32_t    pgn   = pgnList[i].pgn;
    const char *pname = pgnList[i].description;
    size_t      j;

    for (j = 0; pgnList[i].fieldList[j].name != NULL; j++)
    {
      Field     *f = &pgnList[i].fieldList[j];
      FieldType *ft;

      if (f->fieldType == NULL)
      {
        logAbort("PGN %u '%s' field '%s' contains NULL fieldType\n", pgn, pname, f->name);
      }
      ft = getFieldType(f->fieldType);
      if (ft == NULL)
      {
        logAbort("PGN %u '%s' field '%s' contains invalid fieldType '%s'\n", pgn, pname, f->name, f->fieldType);
      }
      f->ft = ft;

      if ((ft->hasSign == True && f->hasSign == false) || (ft->hasSign == False && f->hasSign == true))
      {
        logAbort(
            "PGN %u '%s' field '%s' contains different sign attribute than fieldType '%s'\n", pgn, pname, f->name, f->fieldType);
      }

      if (f->resolution == 0.0)
      {
        f->resolution = ft->resolution;
      }
      if (ft->resolution != 0.0 && ft->resolution != f->resolution)
      {
        logAbort("Cannot overrule resolution %g in '%s' with %g in PGN %u field '%s'\n",
                 ft->resolution,
                 ft->name,
                 f->resolution,
                 pgnList[i].pgn,
                 f->name);
      }

      if (ft->size != 0 && f->size == 0)
      {
        f->size = ft->size;
      }
      if (ft->size != 0 && ft->size != f->size)
      {
        logAbort(
            "Cannot overrule size %d in '%s' with %d in PGN %u field '%s'\n", ft->size, ft->name, f->size, pgnList[i].pgn, f->name);
      }

      if (ft->offset != 0 && f->offset == 0)
      {
        f->offset = ft->offset;
      }
      if (ft->offset != f->offset)
      {
        logAbort("Cannot overrule offset %d in '%s' with %d in PGN %u field '%s'\n",
                 ft->offset,
                 ft->name,
                 f->offset,
                 pgnList[i].pgn,
                 f->name);
      }

      if (ft->unit != NULL && f->unit == NULL)
      {
        f->unit = ft->unit;
      }
      if (f->unit != NULL && ft->unit != NULL && strcmp(f->unit, ft->unit) != 0
          && !(strcmp(f->unit, "deg") == 0 && strcmp(ft->unit, "rad") == 0))
      {
        logAbort("PGN %u '%s' field '%s' contains different unit attribute ('%s') than fieldType '%s' ('%s')\n",
                 pgn,
                 pname,
                 f->name,
                 f->unit,
                 f->fieldType,
                 ft->unit);
      }

      if (isnan(f->rangeMax) || f->rangeMax == 0.0)
      {
        f->rangeMin = ft->rangeMin;
        f->rangeMax = ft->rangeMax;
      }
      if (doUnitFixup && f->unit != NULL && f->resolution != 0.0)
      {
        fixupUnit(f);
      }
      if (f->unit != NULL && f->unit[0] == '=') // Is a match field
      {
        pgnList[i].hasMatchFields = true;
      }

      if (f->size != 0 && f->resolution != 0.0 && ft->hasSign != Null && isnan(f->rangeMax))
      {
        f->rangeMin = getMinRange(f->name, f->size, f->resolution, f->hasSign, f->offset);
        f->rangeMax = getMaxRange(f->name, f->size, f->resolution, f->hasSign, f->offset, &f->lookup);
      }

      f->pgn   = &pgnList[i];
      f->order = j + 1;
    }
    if (pgnList[i].type == PACKET_FAST && !ALLOW_PGN_FAST_PACKET(pgn))
    {
      logAbort("PGN %u '%s' is outside fast-packet range\n", pgn, pgnList[i].description);
    }
    if (pgnList[i].type != PACKET_FAST && !ALLOW_PGN_SINGLE_FRAME(pgn))
    {
      logError("PGN %u '%s' is outside single-frame range\n", pgn, pgnList[i].description);
    }
    if (pgnList[i].repeatingCount1 != 0 && pgnList[i].repeatingStart1 == 0)
    {
      logAbort("PGN %u '%s' has no way to determine repeating field set 1\n", pgn, pgnList[i].description);
    }
    if (pgnList[i].repeatingCount2 != 0 && pgnList[i].repeatingStart2 == 0)
    {
      logAbort("PGN %u '%s' has no way to determine repeating field set 2\n", pgn, pgnList[i].description);
    }

    if (pgnList[i].interval == 0)
    {
      pgnList[i].complete |= PACKET_INTERVAL_UNKNOWN;
    }

    if (j == 0 && pgnList[i].complete == PACKET_COMPLETE)
    {
      logError("Internal error: PGN %d '%s' does not have fields.\n", pgnList[i].pgn, pgnList[i].description);
      exit(2);
    }
    pgnList[i].fieldCount = j;
    logDebug("PGN %u has %u fields\n", pgnList[i].pgn, j);
  }

  logDebug("Filled all fieldtypes\n");
}
