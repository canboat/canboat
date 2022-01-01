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

#include <common.h>

#include "analyzer.h"

#define LOOKUP_TYPE(type, length)               \
  const char *lookupValue##type[1 << (length)]; \
  uint32_t    lookupLength##type = 1 << (length);
#define LOOKUP_TYPE_BITFIELD(type, length) \
  const char *lookupValue##type[length];   \
  uint32_t    lookupLength##type = length;

#include "lookup.h"

#define FILL(a, x, y)                                                 \
  if (a[(x)] != NULL)                                                 \
    logAbort("Non-unique value %u for lookup type %s\n", x, xstr(a)); \
  a[(x)] = y;

#define FILL_IF_NOT_SET(a, x, y) \
  if (a[(x)] == NULL)            \
  a[(x)] = y

// Fill the lookup arrays

void fillLookups(void)
{
  // We don't need to check array index, the C compiler will do that
  // for us with -Warray-bounds.
#define LOOKUP(type, id, value) FILL(lookupValue##type, id, value)

#include "lookup.h"

  /*
#define LOOKUP_TYPE(type, length)                                       \
  if (length > 1)                                                       \
  {                                                                     \
    FILL_IF_NOT_SET(lookupValue##type, (1 << (length)) - 1, "Unknown"); \
    FILL_IF_NOT_SET(lookupValue##type, (1 << (length)) - 2, "Error");   \
  }

#include "lookup.h"
*/

  // Iterate over the PGNs and fill the description of company-code fixed values
  {
    int i;

    for (i = 0; i < pgnListSize; i++)
    {
      Field *f = &pgnList[i].fieldList[0];

      if (f->name && f->units && strcmp(f->name, "Manufacturer Code") == 0)
      {
        int id = 0;

        if (sscanf(f->units, "=%d", &id) > 0)
        {
          f->description = lookupValueMANUFACTURER_CODE[id];
        }
      }
    }
  }
}
