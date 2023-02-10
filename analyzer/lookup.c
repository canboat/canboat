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

#ifdef EXPLAIN

// Generate functions that are usable as callbacks that will loop over all
// possible values.

#define LOOKUP_TYPE(type, length)        \
  void lookup##type(EnumPairCallback cb) \
  {
#define LOOKUP(type, n, str) (cb)(n, str);

#define LOOKUP_TYPE_BITFIELD(type, length) \
  void lookup##type(BitPairCallback cb)    \
  {
#define LOOKUP_BITFIELD(type, n, str) (cb)(n, str);

#define LOOKUP_TYPE_TRIPLET(type, length)   \
  void lookup##type(EnumTripletCallback cb) \
  {
#define LOOKUP_TRIPLET(type, n1, n2, str) (cb)(n1, n2, str);

#define LOOKUP_END }

#else

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

#define LOOKUP_END \
  }                \
  return NULL;     \
  }

#endif

#include "lookup.h"

// Fill the lookup arrays

#ifdef EXPLAIN
static size_t l_id;
static Field *l_field;

static void fillFieldDescription(size_t n, const char *s)
{
  if (n == l_id)
  {
    l_field->description = s;
  }
}
#endif

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
#ifdef EXPLAIN
        // The callback will enumerate all fields, set globals to indicate which
        // id to filter and where to fill the description.
        l_id    = id;
        l_field = f;
        (lookupMANUFACTURER_CODE)(fillFieldDescription);
#else
        f->description = (lookupMANUFACTURER_CODE) (id);
#endif
      }
    }
  }
}

