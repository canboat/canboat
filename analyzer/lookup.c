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

/*
 The source below expands to many C functions, depending on whether EXPLAIN is set or
 not. When compiled for `analyzer-explain`, the macro is set, and the code will generate
 a function for every lookup that looks like this:

    void lookupYES_NO(EnumPairCallback cb)
    {
      (cb)(0, "No");
      (cb)(1, "Yes");
    }

 When the EXPLAIN macro is not set, and this is compiled for `analyzer`, the code generated
 will look like this:

    const char *lookupYES_NO(size_t val)
    {
      switch (val)
      {
        case 0: return "No";
        case 1: return "Yes";
      }
      return NULL;
    }

 This does away with all long sparse arrays that we had before this, and the C optimizers
 generally do an excellent job of creating jump tables to create this code, as this is a
 very typical pattern of code used by code generators (like this one :-) )
*/

#define LOOKUP_PAIR_FUNCTION function.pair
#define LOOKUP_BIT_FUNCTION function.pair
#define LOOKUP_TRIPLET_FUNCTION function.triplet
#define LOOKUP_FIELDTYPE_FUNCTION function.pair


// Generate functions that lookup the value using switch statements. Compilers
// are really good at optimizing long switch statements, as lex/yacc style generated
// code uses that a lot.

#define LOOKUP_TYPE(type, length)      \
  const char *lookup##type(size_t val) \
  {                                    \
    switch (val)                       \
    {
#define LOOKUP(type, n, str) \
  case n:                    \
    return str;

#define LOOKUP_TYPE_BITFIELD(type, length) \
  const char *lookup##type(size_t val)     \
  {                                        \
    switch (val)                           \
    {
#define LOOKUP_BITFIELD(type, n, str) \
  case n:                             \
    return str;

#define LOOKUP_TYPE_TRIPLET(type, length)            \
  const char *lookup##type(size_t val1, size_t val2) \
  {                                                  \
    switch (val1 * 256 + val2)                       \
    {
#define LOOKUP_TRIPLET(type, n1, n2, str) \
  case n1 * 256 + n2:                     \
    return str;

#define LOOKUP_TYPE_TRIPLET(type, length)            \
  const char *lookup##type(size_t val1, size_t val2) \
  {                                                  \
    switch (val1 * 256 + val2)                       \
    {
#define LOOKUP_TRIPLET(type, n1, n2, str) \
  case n1 * 256 + n2:                     \
    return str;

#define LOOKUP_TYPE_FIELDTYPE(type, length) \
  const char *lookup##type(size_t val)      \
  {                                         \
    switch (val)                            \
    {
#define LOOKUP_FIELDTYPE(ftype, n, str, ft)                  \
  case n: {                                                  \
    static Field f;                                          \
    if (f.name == NULL)                                      \
    {                                                        \
      fillFieldTypeLookupField(&f, xstr(ftype), n, str, ft); \
    }                                                        \
    g_ftf = &f;                                              \
    return str;                                              \
  }
#define LOOKUP_FIELDTYPE_LOOKUP(ftype, n, str, ft, bits, lt, ln) \
  case n: {                                                      \
    static Field f;                                              \
    if (f.name == NULL)                                          \
    {                                                            \
      f.lookup.name                   = xstr(ln);                \
      f.size                          = bits;                    \
      f.lookup.size                   = bits;                    \
      f.lookup.type                   = LOOKUP_TYPE_##lt;        \
      f.lookup.LOOKUP_##lt##_FUNCTION = lookup##ln;              \
      fillFieldTypeLookupField(&f, xstr(ftype), n, str, ft);     \
    }                                                            \
    g_ftf = &f;                                                  \
    return str;                                                  \
  }

#define LOOKUP_END \
  }                \
  return NULL;     \
  }


#include "lookup.h"

// Fill the lookup arrays


void fillLookups(void)
{
  // Iterate over the PGNs and fill the description of company-code fixed values
  int i;

  for (i = 0; i < pgnListSize; i++)
  {
    Field *f = &pgnList[i].fieldList[0];

    if (f->name != NULL && f->unit != NULL && strcmp(f->name, "Manufacturer Code") == 0)
    {
      int id = 0;

      if (sscanf(f->unit, "=%d", &id) > 0)
      {
        f->description = (lookupMANUFACTURER_CODE) (id);
      }
    }
  }
}
