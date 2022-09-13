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

#include "pgn.h"

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

extern void fillFieldType(void)
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

    for (size_t j = 0;; j++)
    {
      Field     *f = &pgnList[i].fieldList[j];
      FieldType *ft;

      if (f->name == NULL) // End of field list
      {
        break;
      }

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
        logAbort("PGN %u '%s' field '%s' contains different sign field as fieldType '%s'\n", pgn, pname, f->name, f->fieldType);
      }
    }
  }

  logDebug("Filled all fieldtypes\n");
}
