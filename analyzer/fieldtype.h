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

#ifndef FIELD_H_INCLUDED
#define FIELD_H_INCLUDED

#include <math.h>

#include "common.h"

typedef bool (*FieldPrintFunctionType)(const Field   *field,
                                       const char    *fieldName,
                                       const uint8_t *data,
                                       size_t         dataLen,
                                       size_t         startBit,
                                       size_t        *bits);

extern bool fieldPrintBinary(const Field   *field,
                             const char    *fieldName,
                             const uint8_t *data,
                             size_t         dataLen,
                             size_t         startBit,
                             size_t        *bits);
extern bool fieldPrintBitLookup(const Field   *field,
                                const char    *fieldName,
                                const uint8_t *data,
                                size_t         dataLen,
                                size_t         startBit,
                                size_t        *bits);
extern bool fieldPrintDate(const Field   *field,
                           const char    *fieldName,
                           const uint8_t *data,
                           size_t         dataLen,
                           size_t         startBit,
                           size_t        *bits);
extern bool fieldPrintDecimal(const Field   *field,
                              const char    *fieldName,
                              const uint8_t *data,
                              size_t         dataLen,
                              size_t         startBit,
                              size_t        *bits);
extern bool fieldPrintFloat(const Field   *field,
                            const char    *fieldName,
                            const uint8_t *data,
                            size_t         dataLen,
                            size_t         startBit,
                            size_t        *bits);
extern bool fieldPrintLatLon(const Field   *field,
                             const char    *fieldName,
                             const uint8_t *data,
                             size_t         dataLen,
                             size_t         startBit,
                             size_t        *bits);
extern bool fieldPrintLookup(const Field   *field,
                             const char    *fieldName,
                             const uint8_t *data,
                             size_t         dataLen,
                             size_t         startBit,
                             size_t        *bits);
extern bool fieldPrintMMSI(const Field   *field,
                           const char    *fieldName,
                           const uint8_t *data,
                           size_t         dataLen,
                           size_t         startBit,
                           size_t        *bits);
extern bool fieldPrintNumber(const Field   *field,
                             const char    *fieldName,
                             const uint8_t *data,
                             size_t         dataLen,
                             size_t         startBit,
                             size_t        *bits);
extern bool fieldPrintReserved(const Field   *field,
                               const char    *fieldName,
                               const uint8_t *data,
                               size_t         dataLen,
                               size_t         startBit,
                               size_t        *bits);
extern bool fieldPrintSpare(const Field   *field,
                            const char    *fieldName,
                            const uint8_t *data,
                            size_t         dataLen,
                            size_t         startBit,
                            size_t        *bits);
extern bool fieldPrintStringFix(const Field   *field,
                                const char    *fieldName,
                                const uint8_t *data,
                                size_t         dataLen,
                                size_t         startBit,
                                size_t        *bits);
extern bool fieldPrintStringLAU(const Field   *field,
                                const char    *fieldName,
                                const uint8_t *data,
                                size_t         dataLen,
                                size_t         startBit,
                                size_t        *bits);
extern bool fieldPrintStringLZ(const Field   *field,
                               const char    *fieldName,
                               const uint8_t *data,
                               size_t         dataLen,
                               size_t         startBit,
                               size_t        *bits);
extern bool fieldPrintTime(const Field   *field,
                           const char    *fieldName,
                           const uint8_t *data,
                           size_t         dataLen,
                           size_t         startBit,
                           size_t        *bits);
extern bool fieldPrintVariable(const Field   *field,
                               const char    *fieldName,
                               const uint8_t *data,
                               size_t         dataLen,
                               size_t         startBit,
                               size_t        *bits);
extern bool fieldPrintPGN(const Field   *field,
                          const char    *fieldName,
                          const uint8_t *data,
                          size_t         dataLen,
                          size_t         startBit,
                          size_t        *bits);
extern bool fieldPrintKeyValue(const Field   *field,
                               const char    *fieldName,
                               const uint8_t *data,
                               size_t         dataLen,
                               size_t         startBit,
                               size_t        *bits);
extern bool fieldPrintName(const Field   *field,
                           const char    *fieldName,
                           const uint8_t *data,
                           size_t         dataLen,
                           size_t         startBit,
                           size_t        *bits);
extern void fixupUnit(Field *f);

typedef enum Bool
{
  Null,
  False,
  True
} Bool;

// How a field type signals "data not available" (and, for integers, out-of-range/reserved).
// Emitted per field type as <Sentinels>; for TOP_OF_RANGE the actual reserved raw values are
// given per field as <UnknownValue>/<OutOfRangeValue>/<ReservedValue> (they vary by bit width).
typedef enum Sentinels
{
  SENTINEL_NONE = 0,     // No unavailable encoding (identifier, filler, or structured field)
  SENTINEL_TOP_OF_RANGE, // The top 1-3 raw integer values are reserved (Unknown/OutOfRange/Reserved)
  SENTINEL_NAN,          // IEEE-754 Not-a-Number
  SENTINEL_EMPTY_STRING, // An empty string (after trimming end-of-string bytes) means unavailable
  SENTINEL_VARIABLE      // Depends on the data type resolved through the companion DYNAMIC_FIELD_KEY
} Sentinels;

extern const char *sentinelsName(Sentinels s);

typedef struct PhysicalQuantity
{
  const char *name;        // Name, UPPERCASE_WITH_UNDERSCORE
  const char *description; // English description, shortish
  const char *comment;     // Other observations
  const char *abbreviation;
  const char *unit;
  const char *url; // Website explaining this
} PhysicalQuantity;

#ifdef FIELDTYPE_GLOBALS

#include "physicalquantity-data.h"

#else /* FIELDTYPE_GLOBALS */

extern const PhysicalQuantity *const PhysicalQuantityList[];

#endif

/**
 * The FieldType structure encapsulates the different datatypes in a PGN field.
 */

struct FieldType
{
  const char *name;                // Name, UPPERCASE_WITH_UNDERSCORE
  const char *description;         // English description, shortish
  const char *encodingDescription; // How the value is encoded
  const char *comment;             // Other observations
  const char *url;                 // Website explaining this
  uint32_t    size;                // Size in bits
  Bool        variableSize;        // True if size varies per instance of PGN
  char       *baseFieldType;       // Some field types are variations of others
  char       *v1Type;              // Type as printed in v1 xml/json
  bool        external;            // True when printed in XML

  // The following are only set for numbers
  const char *unit;       // String containing the 'Dimension' (e.g. s, h, m/s, etc.)
  int32_t     offset;     // For numbers with excess-K offset
  double      resolution; // A positive real value, or 1 for integral values
  Bool        hasSign;    // Is the value signed, e.g. has both positive and negative values?

  // These are derived from size, variableSize, resolution and hasSign
  double rangeMin;
  double rangeMax;

  // How this field type signals "data not available"
  Sentinels sentinels;

  // How to print this field
  FieldPrintFunctionType  pf;
  const PhysicalQuantity *physical;

  // Filled by initializer
  FieldType *baseFieldTypePtr;
};

#ifdef FIELDTYPE_GLOBALS
#include "fieldtype-data.h"

const size_t fieldTypeCount = ARRAY_SIZE(fieldTypeList);

#else
extern FieldType    fieldTypeList[];
extern const size_t fieldTypeCount;
#endif // FIELDTYPE_GLOBALS

extern FieldType *getFieldType(const char *name);
extern void       fillFieldType(bool doUnitFixup);
extern void       fillFieldTypeLookupField(Field *f, const char *lookup, const size_t key, const char *str, const char *ft);

#endif // FIELD_H_INCLUDED
