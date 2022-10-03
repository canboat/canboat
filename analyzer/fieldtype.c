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

static void fixupUnit(Field *f)
{
  if (showSI)
  {
    if (strcmp(f->unit, "kWh") == 0)
    {
      f->resolution *= 3.6e6; // 1 kWh = 3.6 MJ.
      f->unit = "J";
    }
    else if (strcmp(f->unit, "Ah") == 0)
    {
      f->resolution *= 3600.0; // 1 Ah = 3600 C.
      f->unit = "C";
    }

    // Many more to follow, but pgn.h is not yet complete enough...
  }
  else // NOT SI
  {
    if (strcmp(f->unit, "C") == 0)
    {
      f->resolution /= 3600.0; // 3600 C = 1 Ah
      f->unit = "Ah";
      logDebug("fixup <%s> to '%s'\n", f->name, f->unit);
    }
    else if (strcmp(f->unit, "Pa") == 0)
    {
      f->resolution /= 100000.0;
      f->precision = 3;
      f->unit      = "bar";
      logDebug("fixup <%s> to '%s'\n", f->name, f->unit);
    }
    else if (strcmp(f->unit, "K") == 0)
    {
      f->unitOffset = -273.15;
      f->precision  = 2;
      f->unit       = "C";
      logDebug("fixup <%s> to '%s'\n", f->name, f->unit);
    }
    else if (strcmp(f->unit, "rad") == 0)
    {
      f->resolution *= RadianToDegree;
      f->unit      = "deg";
      f->precision = 1;
      logDebug("fixup <%s> to '%s'\n", f->name, f->unit);
    }
    else if (strcmp(f->unit, "rad/s") == 0)
    {
      f->resolution *= RadianToDegree;
      f->unit = "deg/s";
      logDebug("fixup <%s> to '%s'\n", f->name, f->unit);
    }
  }
}

extern void fillFieldType(bool doUnitFixup)
{
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
      ft->baseFieldTypePtr = base;

      // Inherit parent fields
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
      uint64_t specialvalues = (ft->size >= 4) ? 2 : (ft->size >= 2) ? 1 : 0;
      uint64_t highbit       = ft->size;

      if (ft->hasSign == False)
      {
        uint64_t maxValue = (UINT64_C(1) << highbit) - 1 - specialvalues;
        logDebug("%s bits=%llu sign=%u maxValue=%llu res=%g\n", ft->name, highbit, ft->hasSign, maxValue, ft->resolution);
        ft->rangeMin = 0.0;
        ft->rangeMax = maxValue * ft->resolution;
      }
      else
      {
        int64_t maxValue;

        highbit--;
        maxValue = (UINT64_C(1) << highbit) - 1;
        logDebug("%s bits=%llu sign=%u maxValue=%lld res=%g\n", ft->name, highbit, ft->hasSign, maxValue, ft->resolution);
        ft->rangeMin = maxValue * ft->resolution * -1.0;
        ft->rangeMax = (maxValue - specialvalues) * ft->resolution;
      }
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

      if (ft->unit != NULL && f->unit == NULL)
      {
        f->unit = ft->unit;
      }
      if (f->unit != NULL && ft->unit != NULL && strcmp(f->unit, ft->unit) != 0)
      {
        logAbort("PGN %u '%s' field '%s' contains different unit attribute ('%s') than fieldType '%s' ('%s')\n",
                 pgn,
                 pname,
                 f->name,
                 f->unit,
                 f->fieldType,
                 ft->unit);
      }

      if (doUnitFixup && f->unit != NULL && f->resolution != 0.0)
      {
        fixupUnit(f);
      }
      f->pgn = &pgnList[i];
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
