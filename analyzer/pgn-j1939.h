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

#include <float.h>

#include "common.h"
#include "parse.h"
#include "pow.h"

#define LEN_VARIABLE (0)

#define RES_LAT_LONG_PRECISION (10000000) /* 1e7 */
#define RES_LAT_LONG (1.0e-7)
#define RES_LAT_LONG_64 (1.0e-16)
#define RES_PERCENTAGE (100.0 / 25000.0)

#define RES_RADIANS (1e-4)
#define RES_ROTATION (1e-3 / 32.0)
#define RES_HIRES_ROTATION (1e-6 / 32.0)

typedef struct FieldType  FieldType;
typedef struct Pgn        Pgn;
typedef struct LookupInfo LookupInfo;

typedef void (*EnumPairCallback)(size_t value, const char *name);
typedef void (*BitPairCallback)(size_t value, const char *name);
typedef void (*EnumTripletCallback)(size_t value1, size_t value2, const char *name);
typedef void (*EnumFieldtypeCallback)(size_t value, const char *name, const char *ft, const LookupInfo *lookup);

typedef enum LookupType
{
  LOOKUP_TYPE_NONE,
  LOOKUP_TYPE_PAIR,
  LOOKUP_TYPE_TRIPLET,
  LOOKUP_TYPE_BIT,
  LOOKUP_TYPE_FIELDTYPE
} LookupType;

struct LookupInfo
{
  const char *name;
  LookupType  type;
  union
  {
    const char *(*pair)(size_t val);
    const char *(*triplet)(size_t val1, size_t val2);
    void (*pairEnumerator)(EnumPairCallback);
    void (*bitEnumerator)(BitPairCallback);
    void (*tripletEnumerator)(EnumTripletCallback);
    void (*fieldtypeEnumerator)(EnumFieldtypeCallback);
  } function;
  uint8_t val1Order; // Which field is the first field in a tripletEnumerator
  size_t  size;      // Used in analyzer only
};

#ifdef EXPLAIN
#define LOOKUP_PAIR_MEMBER .lookup.function.pairEnumerator
#define LOOKUP_BIT_MEMBER .lookup.function.bitEnumerator
#define LOOKUP_TRIPLET_MEMBER .lookup.function.tripletEnumerator
#define LOOKUP_FIELDTYPE_MEMBER .lookup.function.fieldtypeEnumerator
#else
#define LOOKUP_PAIR_MEMBER .lookup.function.pair
#define LOOKUP_BIT_MEMBER .lookup.function.pair
#define LOOKUP_TRIPLET_MEMBER .lookup.function.triplet
#define LOOKUP_FIELDTYPE_MEMBER .lookup.function.pair
#endif

typedef struct
{
  const char *name;
  const char *fieldType;

  uint32_t    size; /* Size in bits. All fields are contiguous in message; use 'reserved' fields to fill in empty bits. */
  const char *unit; /* String containing the 'Dimension' (e.g. s, h, m/s, etc.) */
  const char *description;

  int32_t offset;          /* Only used for SAE J1939 values with sign; these are in Offset/Excess-K notation instead
                            *    of two's complement as used by NMEA 2000.
                            *    See http://en.wikipedia.org/wiki/Offset_binary
                            */
  double resolution;       /* Either a positive real value or zero */
  int    precision;        /* How many decimal digits after the decimal point to print; usually 0 = automatic */
  double unitOffset;       /* Only used for K->C conversion in non-SI print */
  bool   proprietary;      /* Field is only present if earlier PGN field is in proprietary range */
  bool   hasSign;          /* Is the value signed, e.g. has both positive and negative values? */
  bool   partOfPrimaryKey; /* Is the value part of the primary key for the message */
  int8_t reservedOverride; /* Override reserved (special) value count; 0 = auto, else (count + 1). See SPECIAL_VALUES(). */
  bool    dynamicFieldLength;          /* True if this field's value is the byte length of a following DYNAMIC_FIELD_VALUE field. */
  uint8_t dynamicFieldLengthOverhead;  /* Non-value header bytes counted in the length that must be subtracted (see pgn.h). */

  /* The following fields are filled by C, no need to set in initializers */
  uint8_t    order;
  uint8_t    reservedCount; /* Resolved number of reserved special values (0..3): Unknown, OutOfRange, Reserved. */
  size_t     bitOffset;     // Bit offset from start of data, e.g. lower 3 bits = bit#, bit 4.. is byte offset
  char      *camelName;
  LookupInfo lookup;
  FieldType *ft;
  Pgn       *pgn;
  double     rangeMin;
  double     rangeMax;
} Field;

#include "fieldtype.h"

#define END_OF_FIELDS {0}

#define LOOKUP_FIELD(nam, len, typ)       \
  {.name              = nam,              \
   .size              = len,              \
   .resolution        = 1,                \
   .hasSign           = false,            \
   .lookup.type       = LOOKUP_TYPE_PAIR, \
   LOOKUP_PAIR_MEMBER = lookup##typ,      \
   .lookup.name       = xstr(typ),        \
   .fieldType         = "LOOKUP"}

#define LOOKUP_FIELDTYPE_FIELD(nam, len, typ)       \
  {.name                   = nam,                   \
   .size                   = len,                   \
   .resolution             = 1,                     \
   .hasSign                = false,                 \
   .lookup.type            = LOOKUP_TYPE_FIELDTYPE, \
   LOOKUP_FIELDTYPE_MEMBER = lookup##typ,           \
   .lookup.name            = xstr(typ),             \
   .fieldType              = "FIELDTYPE_LOOKUP"}

#define LOOKUP_TRIPLET_FIELD(nam, len, typ, desc, order) \
  {.name                 = nam,                          \
   .size                 = len,                          \
   .resolution           = 1,                            \
   .hasSign              = false,                        \
   .lookup.type          = LOOKUP_TYPE_TRIPLET,          \
   LOOKUP_TRIPLET_MEMBER = lookup##typ,                  \
   .lookup.name          = xstr(typ),                    \
   .lookup.val1Order     = order,                        \
   .fieldType            = "INDIRECT_LOOKUP",            \
   .description          = desc}

#define LOOKUP_FIELD_DESC(nam, len, typ, desc) \
  {.name              = nam,                   \
   .size              = len,                   \
   .resolution        = 1,                     \
   .hasSign           = false,                 \
   .lookup.type       = LOOKUP_TYPE_PAIR,      \
   LOOKUP_PAIR_MEMBER = lookup##typ,           \
   .lookup.name       = xstr(typ),             \
   .fieldType         = "LOOKUP",              \
   .description       = desc}

#define BITLOOKUP_FIELD(nam, len, typ)  \
  {.name             = nam,             \
   .size             = len,             \
   .resolution       = 1,               \
   .hasSign          = false,           \
   .lookup.type      = LOOKUP_TYPE_BIT, \
   LOOKUP_BIT_MEMBER = lookup##typ,     \
   .lookup.name      = xstr(typ),       \
   .fieldType        = "BITLOOKUP"}

#define FIELDTYPE_LOOKUP(nam, len, typ)             \
  {.name                   = nam,                   \
   .size                   = len,                   \
   .resolution             = 1,                     \
   .hasSign                = false,                 \
   .lookup.type            = LOOKUP_TYPE_FIELDTYPE, \
   LOOKUP_FIELDTYPE_MEMBER = lookup##typ,           \
   .lookup.name            = xstr(typ),             \
   .fieldType              = "LOOKUP_TYPE_FIELDTYPE"}

#define UNKNOWN_LOOKUP_FIELD(nam, len) \
  {.name = nam, .size = len, .resolution = 1, .hasSign = false, .lookup.type = LOOKUP_TYPE_PAIR, .fieldType = "LOOKUP"}

#define SPARE_NAMED_FIELD(nam, len) {.name = nam, .size = (len), .resolution = 1, .fieldType = "SPARE"}

#define SPARE_FIELD(len) SPARE_NAMED_FIELD("Spare", len)

#define RESERVED_FIELD(len) {.name = "Reserved", .size = (len), .resolution = 1, .fieldType = "RESERVED"}

#define RESERVED_PROP_FIELD(len, desc) \
  {.name = "Reserved", .size = (len), .resolution = 1, .description = desc, .fieldType = "RESERVED", .proprietary = true}

#define BINARY_FIELD(nam, len, desc) {.name = nam, .size = (len), .resolution = 1, .description = desc, .fieldType = "BINARY"}

#define BINARY_UNIT_FIELD(nam, len, unt, desc, prop) \
  {.name = nam, .size = (len), .resolution = 1, .unit = unt, .description = desc, .proprietary = prop, .fieldType = "BINARY"}

#define LATITUDE_I32_FIELD(nam) \
  {.name = nam, .size = BYTES(4), .resolution = 1e-7, .hasSign = true, .unit = "deg", .fieldType = "GEO_FIX32"}

#define LATITUDE_I64_FIELD(nam) \
  {.name = nam, .size = BYTES(8), .resolution = 1e-16, .hasSign = true, .unit = "deg", .fieldType = "GEO_FIX64"}

#define LONGITUDE_I32_FIELD(nam) \
  {.name = nam, .size = BYTES(4), .resolution = 1e-7, .hasSign = true, .unit = "deg", .fieldType = "GEO_FIX32"}

#define LONGITUDE_I64_FIELD(nam) \
  {.name = nam, .size = BYTES(8), .resolution = 1e-16, .hasSign = true, .unit = "deg", .fieldType = "GEO_FIX64"}

#define ANGLE_U16_FIELD(nam, desc) \
  {.name        = nam,             \
   .size        = BYTES(2),        \
   .resolution  = RES_RADIANS,     \
   .hasSign     = false,           \
   .unit        = "rad",           \
   .description = desc,            \
   .fieldType   = "ANGLE_UFIX16"}

#define ANGLE_I16_FIELD(nam, desc) \
  {.name        = nam,             \
   .size        = BYTES(2),        \
   .resolution  = RES_RADIANS,     \
   .hasSign     = true,            \
   .unit        = "rad",           \
   .description = desc,            \
   .fieldType   = "ANGLE_FIX16"}

#define INT32_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(4), .resolution = 1, .hasSign = true, .fieldType = "INT32", .description = desc}

// A whole bunch of different NUMBER fields, with variing resolutions

#define UNSIGNED_ALMANAC_PARAMETER_FIELD(nam, len, res, ft, desc) \
  {.name        = nam,                                            \
   .size        = len,                                            \
   .resolution  = res,                                            \
   .hasSign     = false,                                          \
   .description = desc,                                           \
   .fieldType   = ft}

#define SIGNED_ALMANAC_PARAMETER_FIELD(nam, len, res, ft, desc) \
  {.name        = nam,                                          \
   .size        = len,                                          \
   .resolution  = res,                                          \
   .hasSign     = true,                                         \
   .description = desc,                                         \
   .fieldType   = ft}

#define DILUTION_OF_PRECISION_UFIX16_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 0.01, .fieldType = "DILUTION_OF_PRECISION_UFIX16", .description = desc}

#define DILUTION_OF_PRECISION_FIX16_FIELD(nam, desc) \
  {.name        = nam,                               \
   .size        = BYTES(2),                          \
   .resolution  = 0.01,                              \
   .hasSign     = true,                              \
   .fieldType   = "DILUTION_OF_PRECISION_FIX16",     \
   .description = desc}

#define SIGNALTONOISERATIO_UFIX16_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 0.01, .fieldType = "SIGNALTONOISERATIO_UFIX16", .description = desc}

#define SIGNALTONOISERATIO_FIX16_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 0.01, .hasSign = true, .fieldType = "SIGNALTONOISERATIO_FIX16", .description = desc}

#define VERSION_FIELD(nam) {.name = nam, .size = BYTES(2), .resolution = 0.001, .fieldType = "VERSION"}

#define VOLTAGE_U16_V_FIELD(nam) {.name = nam, .size = BYTES(2), .resolution = 1.0, .unit = "V", .fieldType = "VOLTAGE_UFIX16_V"}

#define VOLTAGE_U16_10MV_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "V", .fieldType = "VOLTAGE_UFIX16_10MV"}

#define VOLTAGE_U16_50MV_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 0.05, .unit = "V", .fieldType = "VOLTAGE_UFIX16_50MV"}

#define VOLTAGE_U16_100MV_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 0.1, .unit = "V", .fieldType = "VOLTAGE_UFIX16_100MV"}

#define VOLTAGE_UFIX8_200MV_FIELD(nam) \
  {.name = nam, .size = BYTES(1), .resolution = 0.2, .unit = "V", .fieldType = "VOLTAGE_UFIX8_200MV"}

#define VOLTAGE_I16_10MV_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "V", .hasSign = true, .fieldType = "VOLTAGE_FIX16_10MV"}

#define RADIO_FREQUENCY_FIELD(nam, res) \
  {.name = nam, .size = BYTES(4), .resolution = res, .unit = "Hz", .fieldType = "RADIO_FREQUENCY_UFIX32"}

#define FREQUENCY_FIELD(nam, res) {.name = nam, .size = BYTES(2), .resolution = res, .unit = "Hz", .fieldType = "FREQUENCY_UFIX16"}

#define SPEED_I16_MM_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 0.001, .unit = "m/s", .hasSign = true, .fieldType = "SPEED_FIX16_MM"}

#define SPEED_I16_CM_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "m/s", .hasSign = true, .fieldType = "SPEED_FIX16_CM"}

#define SPEED_U16_CM_FIELD(nam) {.name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "m/s", .fieldType = "SPEED_UFIX16_CM"}

#define SPEED_U16_DM_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 0.1, .unit = "m/s", .fieldType = "SPEED_UFIX16_DM", .description = desc}

#define DISTANCE_FIX16_M_FIELD(nam, desc) \
  {.name        = nam,                    \
   .size        = BYTES(2),               \
   .resolution  = 1,                      \
   .hasSign     = true,                   \
   .unit        = "m",                    \
   .description = desc,                   \
   .fieldType   = "DISTANCE_FIX16_M"}

#define DISTANCE_FIX16_CM_FIELD(nam, desc) \
  {.name        = nam,                     \
   .size        = BYTES(2),                \
   .resolution  = 0.01,                    \
   .hasSign     = true,                    \
   .unit        = "m",                     \
   .description = desc,                    \
   .fieldType   = "DISTANCE_FIX16_CM"}

#define DISTANCE_FIX16_MM_FIELD(nam, desc) \
  {.name        = nam,                     \
   .size        = BYTES(2),                \
   .resolution  = 0.001,                   \
   .hasSign     = true,                    \
   .unit        = "m",                     \
   .description = desc,                    \
   .fieldType   = "DISTANCE_FIX16_MM"}

#define DISTANCE_FIX32_MM_FIELD(nam, desc) \
  {.name        = nam,                     \
   .size        = BYTES(4),                \
   .resolution  = 0.001,                   \
   .hasSign     = true,                    \
   .unit        = "m",                     \
   .description = desc,                    \
   .fieldType   = "DISTANCE_FIX32_MM"}

#define DISTANCE_FIX32_CM_FIELD(nam, desc) \
  {.name        = nam,                     \
   .size        = BYTES(4),                \
   .resolution  = 0.01,                    \
   .hasSign     = true,                    \
   .unit        = "m",                     \
   .description = desc,                    \
   .fieldType   = "DISTANCE_FIX32_CM"}

#define DISTANCE_FIX64_FIELD(nam, desc) \
  {.name        = nam,                  \
   .size        = BYTES(8),             \
   .resolution  = 1e-6,                 \
   .hasSign     = true,                 \
   .unit        = "m",                  \
   .description = desc,                 \
   .fieldType   = "DISTANCE_FIX64"}

#define LENGTH_UFIX8_DAM_FIELD(nam, desc) \
  {.name = nam, .size = 8, .resolution = 10, .unit = "m", .fieldType = "LENGTH_UFIX8_DAM", .description = desc}

#define LENGTH_UFIX16_CM_FIELD(nam) {.name = nam, .size = 16, .resolution = 0.01, .unit = "m", .fieldType = "LENGTH_UFIX16_CM"}

#define LENGTH_UFIX16_DM_FIELD(nam) {.name = nam, .size = 16, .resolution = 0.1, .unit = "m", .fieldType = "LENGTH_UFIX16_DM"}

#define LENGTH_UFIX32_M_FIELD(nam, desc) \
  {.name = nam, .size = 32, .resolution = 1, .unit = "m", .fieldType = "LENGTH_UFIX32_M", .description = desc}

#define LENGTH_UFIX32_CM_FIELD(nam, desc) \
  {.name = nam, .size = 32, .resolution = 0.01, .unit = "m", .fieldType = "LENGTH_UFIX32_CM", .description = desc}

#define LENGTH_UFIX32_MM_FIELD(nam) {.name = nam, .size = 32, .resolution = 0.001, .unit = "m", .fieldType = "LENGTH_UFIX32_MM"}

#define CURRENT_UFIX8_A_FIELD(nam) {.name = nam, .size = BYTES(1), .resolution = 1, .unit = "A", .fieldType = "CURRENT_UFIX8_A"}

#define CURRENT_UFIX16_A_FIELD(nam) {.name = nam, .size = BYTES(2), .resolution = 1, .unit = "A", .fieldType = "CURRENT_UFIX16_A"}

#define CURRENT_UFIX16_DA_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 0.1, .unit = "A", .fieldType = "CURRENT_UFIX16_DA"}

#define CURRENT_FIX16_DA_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 0.1, .hasSign = true, .unit = "A", .fieldType = "CURRENT_FIX16_DA"}

#define CURRENT_FIX24_CA_FIELD(nam) \
  {.name = nam, .size = BYTES(3), .resolution = 0.01, .hasSign = true, .unit = "A", .fieldType = "CURRENT_FIX24_CA"}

#define ELECTRIC_CHARGE_UFIX16_AH(nam) {.name = nam, .fieldType = "ELECTRIC_CHARGE_UFIX16_AH"}

#define PEUKERT_FIELD(nam) {.name = nam, .fieldType = "PEUKERT_EXPONENT"}

// Fully defined NUMBER fields

#define PGN_FIELD(nam, desc) {.name = nam, .size = BYTES(3), .resolution = 1, .fieldType = "PGN", .description = desc}

#define INSTANCE_FIELD {.name = "Instance", .size = BYTES(1), .resolution = 1, .description = NULL, .fieldType = "UINT8"}

#define POWER_FACTOR_U16_FIELD \
  {.name = "Power factor", .size = BYTES(2), .resolution = 1 / 16384., .unit = "Cos Phi", .fieldType = "UFIX16"}

#define POWER_FACTOR_U8_FIELD \
  {.name = "Power factor", .size = BYTES(1), .resolution = 0.01, .unit = "Cos Phi", .fieldType = "UFIX8"}

// End of NUMBER fields

#define MANUFACTURER_FIELD(unt, desc, prop)      \
  {.name              = "Manufacturer Code",     \
   .size              = 11,                      \
   .resolution        = 1,                       \
   .description       = desc,                    \
   .unit              = unt,                     \
   .lookup.type       = LOOKUP_TYPE_PAIR,        \
   LOOKUP_PAIR_MEMBER = lookupMANUFACTURER_CODE, \
   .lookup.name       = "MANUFACTURER_CODE",     \
   .proprietary       = prop,                    \
   .fieldType         = "MANUFACTURER"}

#define INDUSTRY_FIELD(unt, desc, prop)      \
  {.name              = "Industry Code",     \
   .size              = 3,                   \
   .resolution        = 1,                   \
   .unit              = unt,                 \
   .description       = desc,                \
   .lookup.type       = LOOKUP_TYPE_PAIR,    \
   LOOKUP_PAIR_MEMBER = lookupINDUSTRY_CODE, \
   .lookup.name       = "INDUSTRY_CODE",     \
   .proprietary       = prop,                \
   .fieldType         = "INDUSTRY"}

#define MARINE_INDUSTRY_FIELD INDUSTRY_FIELD("=4", "Marine Industry", false)

#define COMPANY(id) MANUFACTURER_FIELD("=" xstr(id), NULL, false), RESERVED_FIELD(2), MARINE_INDUSTRY_FIELD

#define MANUFACTURER_FIELDS MANUFACTURER_FIELD(NULL, NULL, false), RESERVED_FIELD(2), INDUSTRY_FIELD(NULL, NULL, false)

#define MANUFACTURER_PROPRIETARY_FIELDS                                            \
  MANUFACTURER_FIELD(NULL, "Only in PGN when Commanded PGN is proprietary", true), \
      RESERVED_PROP_FIELD(2, "Only in PGN when Commanded PGN is proprietary"),     \
      INDUSTRY_FIELD(NULL, "Only in PGN when Commanded PGN is proprietary", true)

#define INTEGER_DESC_FIELD(nam, len, desc) {.name = nam, .size = len, .resolution = 1, .description = desc}

#define INTEGER_UNIT_FIELD(nam, len, unt) {.name = nam, .size = len, .resolution = 1, .unit = unt}

#define SIGNED_INTEGER_UNIT_FIELD(nam, len, unt) {.name = nam, .size = len, .resolution = 1, .unit = unt, .hasSign = true}

#define INTEGER_FIELD(nam, len) INTEGER_DESC_FIELD(nam, len, "")

#define UINT8_DESC_FIELD(nam, desc) {.name = nam, .size = BYTES(1), .resolution = 1, .fieldType = "UINT8", .description = desc}

#define FIELD_INDEX(nam, desc) {.name = nam, .size = BYTES(1), .resolution = 1, .fieldType = "FIELD_INDEX", .description = desc}

#define UINT8_FIELD(nam) UINT8_DESC_FIELD(nam, NULL)

#define UINT16_DESC_FIELD(nam, desc) {.name = nam, .size = BYTES(2), .resolution = 1, .fieldType = "UINT16", .description = desc}

#define UINT16_FIELD(nam) UINT16_DESC_FIELD(nam, NULL)

#define UINT32_DESC_FIELD(nam, desc) {.name = nam, .size = BYTES(4), .resolution = 1, .fieldType = "UINT32", .description = desc}

#define UINT32_FIELD(nam) UINT32_DESC_FIELD(nam, NULL)

#define MATCH_LOOKUP_FIELD(nam, len, id, typ) \
  {                                           \
      .name              = nam,               \
      .size              = len,               \
      .resolution        = 1,                 \
      .hasSign           = false,             \
      .lookup.type       = LOOKUP_TYPE_PAIR,  \
      LOOKUP_PAIR_MEMBER = lookup##typ,       \
      .lookup.name       = xstr(typ),         \
      .fieldType         = "LOOKUP",          \
      .unit              = "=" xstr(id),      \
  }

#define MATCH_FIELD(nam, len, id, desc) \
  {.name = nam, .size = len, .resolution = 1, .unit = "=" xstr(id), .description = desc, .fieldType = "UNSIGNED_INTEGER"}

#define SIMPLE_DESC_FIELD(nam, len, desc) \
  {.name = nam, .size = len, .resolution = 1, .description = desc, .fieldType = "UNSIGNED_INTEGER"}

#define SIMPLE_FIELD(nam, len) {.name = nam, .size = len, .resolution = 1, .fieldType = "UNSIGNED_INTEGER"}

#define SIMPLE_SIGNED_FIELD(nam, len) {.name = nam, .size = len, .resolution = 1, .hasSign = true, .fieldType = "INTEGER"}

#define MMSI_FIELD(nam)     \
  {.name       = nam,       \
   .size       = BYTES(4),  \
   .resolution = 1,         \
   .hasSign    = false,     \
   .rangeMin   = 2000000,   \
   .rangeMax   = 999999999, \
   .fieldType  = "MMSI"}

#define DECIMAL_FIELD(nam, len, desc) {.name = nam, .size = len, .resolution = 1, .description = desc, .fieldType = "DECIMAL"}

#define DECIMAL_UNIT_FIELD(nam, len, unt) {.name = nam, .size = len, .resolution = 1, .unit = unt, .fieldType = "DECIMAL"}

#define STRINGLZ_FIELD(nam, len) {.name = nam, .size = len, .resolution = 0, .fieldType = "STRING_LZ"}

#define STRING_FIX_DESC_FIELD(nam, len, desc) \
  {.name = nam, .size = len, .resolution = 0, .description = desc, .fieldType = "STRING_FIX"}

#define STRINGVAR_FIELD(nam) {.name = nam, .size = LEN_VARIABLE, .resolution = 0, .fieldType = "STRING_LZ"}

#define STRINGLAU_FIELD(nam) {.name = nam, .size = LEN_VARIABLE, .resolution = 0, .fieldType = "STRING_LAU"}

#define STRING_FIX_FIELD(nam, len) STRING_FIX_DESC_FIELD(nam, len, NULL)

#define TEMPERATURE_HIGH_FIELD(nam) {.name = nam, .size = BYTES(2), .resolution = 0.1, .unit = "K", .fieldType = "TEMPERATURE_HIGH"}

#define TEMPERATURE_FIELD(nam) {.name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "K", .fieldType = "TEMPERATURE"}

#define TEMPERATURE_UINT8_OFFSET_FIELD(nam) \
  {.name = nam, .size = BYTES(1), .offset = 233, .resolution = 1, .unit = "K", .fieldType = "TEMPERATURE_UINT8_OFFSET"}

#define TEMPERATURE_U24_FIELD(nam) \
  {.name = nam, .size = BYTES(3), .resolution = 0.001, .unit = "K", .fieldType = "TEMPERATURE_UFIX24"}

#define TEMPERATURE_DELTA_FIX16_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 0.001, .unit = "K", .hasSign = true, .fieldType = "FIX16", .description = desc}

#define VOLUMETRIC_FLOW_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 0.1, .unit = "L/h", .hasSign = true, .fieldType = "VOLUMETRIC_FLOW"}

#define CONCENTRATION_UINT16_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 1, .unit = "ppm", .fieldType = "CONCENTRATION_UINT16_PPM"}

#define VOLUME_UFIX16_L_FIELD(nam) {.name = nam, .size = BYTES(2), .resolution = 1, .unit = "L", .fieldType = "VOLUME_UFIX16_L"}

#define VOLUME_UFIX32_DL_FIELD(nam) {.name = nam, .size = BYTES(4), .resolution = 0.1, .unit = "L", .fieldType = "VOLUME_UFIX32_DL"}

#define TIME_UFIX16_S_FIELD(nam) {.name = nam, .size = BYTES(2), .resolution = 1, .unit = "s", .fieldType = "TIME_UFIX16_S"}

#define TIME_FIX32_MS_FIELD(nam, desc) \
  {.name        = nam,                 \
   .size        = BYTES(4),            \
   .resolution  = 0.001,               \
   .unit        = "s",                 \
   .hasSign     = true,                \
   .fieldType   = "TIME_FIX32_MS",     \
   .description = desc}

#define TIME_UFIX8_5MS_FIELD(nam, desc) \
  {.name        = nam,                  \
   .size        = BYTES(1),             \
   .resolution  = 0.005,                \
   .unit        = "s",                  \
   .hasSign     = false,                \
   .fieldType   = "TIME_UFIX8_5MS",     \
   .description = desc}

#define TIME_UFIX16_MIN_FIELD(nam, desc) \
  {.name        = nam,                   \
   .size        = BYTES(2),              \
   .resolution  = 60,                    \
   .unit        = "s",                   \
   .hasSign     = false,                 \
   .fieldType   = "TIME_UFIX16_MIN",     \
   .description = desc}

#define TIME_UFIX16_MS_FIELD(nam, desc) \
  {.name        = nam,                  \
   .size        = BYTES(2),             \
   .resolution  = 0.001,                \
   .unit        = "s",                  \
   .hasSign     = false,                \
   .fieldType   = "TIME_UFIX16_MS",     \
   .description = desc}

#define TIME_UFIX16_CS_FIELD(nam, desc) \
  {.name        = nam,                  \
   .size        = BYTES(2),             \
   .resolution  = 0.01,                 \
   .unit        = "s",                  \
   .hasSign     = false,                \
   .fieldType   = "TIME_UFIX16_CS",     \
   .description = desc}

#define TIME_FIX16_5CS_FIELD(nam, desc) \
  {.name        = nam,                  \
   .size        = BYTES(2),             \
   .resolution  = 0.05,                 \
   .unit        = "s",                  \
   .hasSign     = true,                 \
   .fieldType   = "TIME_FIX16_5CS",     \
   .description = desc}

#define TIME_FIX16_MIN_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 60., .unit = "s", .hasSign = true, .fieldType = "TIME_FIX16_MIN"}

#define TIME_UFIX24_MS_FIELD(nam, desc) \
  {.name        = nam,                  \
   .size        = BYTES(3),             \
   .resolution  = 0.001,                \
   .unit        = "s",                  \
   .hasSign     = false,                \
   .fieldType   = "TIME_UFIX24_MS",     \
   .description = desc}

#define TIME_UFIX32_S_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(4), .resolution = 1, .unit = "s", .hasSign = false, .fieldType = "TIME_UFIX32_S", .description = desc}

#define TIME_UFIX32_MS_FIELD(nam, desc) \
  {.name        = nam,                  \
   .size        = BYTES(4),             \
   .resolution  = 0.001,                \
   .unit        = "s",                  \
   .hasSign     = false,                \
   .fieldType   = "TIME_UFIX32_MS",     \
   .description = desc}

#define TIME_FIELD(nam)                     \
  {.name        = nam,                      \
   .size        = BYTES(4),                 \
   .resolution  = 0.0001,                   \
   .unit        = "s",                      \
   .hasSign     = false,                    \
   .fieldType   = "TIME",                   \
   .description = "Seconds since midnight", \
   .rangeMin    = 0,                        \
   .rangeMax    = 86402}

#define DATE_FIELD(nam) {.name = nam, .size = BYTES(2), .resolution = 1, .unit = "d", .hasSign = false, .fieldType = "DATE"}

#define VARIABLE_FIELD(nam, desc) {.name = nam, .size = LEN_VARIABLE, .description = desc, .fieldType = "VARIABLE"}

#define KEY_VALUE_FIELD(nam, desc) {.name = nam, .size = LEN_VARIABLE, .description = desc, .fieldType = "KEY_VALUE"}

#define ENERGY_UINT32_FIELD(nam) {.name = nam, .size = BYTES(4), .resolution = 1, .unit = "kWh", .fieldType = "ENERGY_UINT32"}

#define POWER_I32_OFFSET_FIELD(nam) {.name = nam, .hasSign = true, .fieldType = "POWER_FIX32_OFFSET"}

#define POWER_I32_VA_OFFSET_FIELD(nam) {.name = nam, .hasSign = true, .fieldType = "POWER_FIX32_VA_OFFSET"}

#define POWER_I32_VAR_OFFSET_FIELD(nam) {.name = nam, .hasSign = true, .fieldType = "POWER_FIX32_VAR_OFFSET"}

#define POWER_U16_FIELD(nam) {.name = nam, .size = BYTES(2), .resolution = 1, .unit = "W", .fieldType = "POWER_UINT16"}

#define POWER_U16_VAR_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 1, .unit = "VAR", .description = desc, .fieldType = "POWER_UINT16_VAR"}

#define POWER_I32_FIELD(nam) \
  {.name = nam, .size = BYTES(4), .resolution = 1, .hasSign = true, .unit = "W", .fieldType = "POWER_INT32"}

#define POWER_U32_FIELD(nam) {.name = nam, .size = BYTES(4), .resolution = 1, .unit = "W", .fieldType = "POWER_UINT32"}

#define POWER_U32_VA_FIELD(nam) {.name = nam, .size = BYTES(4), .resolution = 1, .unit = "VA", .fieldType = "POWER_UINT32_VA"}

#define POWER_U32_VAR_FIELD(nam) {.name = nam, .size = BYTES(4), .resolution = 1, .unit = "VAR", .fieldType = "POWER_UINT32_VAR"}

#define PERCENTAGE_U8_FIELD(nam) {.name = nam, .size = BYTES(1), .resolution = 1, .unit = "%", .fieldType = "PERCENTAGE_UINT8"}

#define PERCENTAGE_U8_HIGHRES_FIELD(nam) \
  {.name = nam, .size = BYTES(1), .resolution = .4, .unit = "%", .fieldType = "PERCENTAGE_UINT8_HIGHRES"}

#define PERCENTAGE_I8_FIELD(nam) \
  {.name = nam, .size = BYTES(1), .resolution = 1, .hasSign = true, .unit = "%", .fieldType = "PERCENTAGE_INT8"}

#define PERCENTAGE_I16_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = RES_PERCENTAGE, .hasSign = true, .unit = "%", .fieldType = "PERCENTAGE_FIX16"}

#define ROTATION_FIX16_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = (1e-3 / 32.0), .hasSign = true, .unit = "rad/s", .fieldType = "ROTATION_FIX16"}

#define ROTATION_UFIX16_RPM_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 0.25, .hasSign = false, .unit = "rpm", .fieldType = "ROTATION_UFIX16_RPM"}

#define ROTATION_UFIX16_RPM_HIGHRES_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 0.125, .hasSign = false, .unit = "rpm", .fieldType = "ROTATION_UFIX16_RPM_HIGHRES"}

#define ROTATION_FIX32_FIELD(nam) \
  {.name = nam, .size = BYTES(4), .resolution = (1e-6 / 32.0), .hasSign = true, .unit = "rad/s", .fieldType = "ROTATION_FIX32"}

#define PRESSURE_UFIX16_HPA_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 100, .unit = "Pa", .fieldType = "PRESSURE_UFIX16_HPA"}

#define PRESSURE_UINT8_KPA_FIELD(nam) \
  {.name = nam, .size = BYTES(1), .resolution = 500, .unit = "Pa", .fieldType = "PRESSURE_UINT8_KPA"}

#define PRESSURE_UINT8_2KPA_FIELD(nam) \
  {.name = nam, .size = BYTES(1), .resolution = 2000, .unit = "Pa", .fieldType = "PRESSURE_UINT8_2KPA"}

#define PRESSURE_UFIX16_KPA_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 1000, .hasSign = false, .unit = "Pa", .fieldType = "PRESSURE_UFIX16_KPA"}

#define PRESSURE_RATE_FIX16_PA_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 1, .hasSign = true, .unit = "Pa/hr", .fieldType = "PRESSURE_RATE_FIX16_PA"}

#define PRESSURE_FIX16_KPA_FIELD(nam) \
  {.name = nam, .size = BYTES(2), .resolution = 1000, .hasSign = true, .unit = "Pa", .fieldType = "PRESSURE_FIX16_KPA"}

#define PRESSURE_FIX32_DPA_FIELD(nam) \
  {.name = nam, .size = BYTES(4), .resolution = 0.1, .hasSign = true, .unit = "Pa", .fieldType = "PRESSURE_FIX32_DPA"}

#define PRESSURE_UFIX32_DPA_FIELD(nam) \
  {.name = nam, .size = BYTES(4), .resolution = 0.1, .hasSign = false, .unit = "Pa", .fieldType = "PRESSURE_UFIX32_DPA"}

#define GAIN_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 0.01, .hasSign = true, .fieldType = "GAIN_FIX16", .description = desc}

#define MAGNETIC_FIX16_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 0.01, .hasSign = true, .unit = "T", .fieldType = "MAGNETIC_FIELD_FIX16"}

#define ANGLE_FIX16_DDEG_FIELD(nam, desc) \
  {.name = nam, .size = BYTES(2), .resolution = 0.1, .hasSign = true, .unit = "deg", .fieldType = "ANGLE_FIX16_DDEG"}

#define FLOAT_FIELD(nam, ft, desc) \
  {.name        = nam,              \
   .size        = BYTES(4),         \
   .hasSign     = true,             \
   .fieldType   = ft,               \
   .description = desc,             \
   .resolution  = 1,                \
   .rangeMin    = -1 * FLT_MAX,     \
   .rangeMax    = FLT_MAX}

#ifdef EXPLAIN
#define LOOKUP_TYPE(type, length) extern void lookup##type(EnumPairCallback cb);
#define LOOKUP_TYPE_TRIPLET(type, length) extern void lookup##type(EnumTripletCallback cb);
#define LOOKUP_TYPE_BITFIELD(type, length) extern void lookup##type(BitPairCallback cb);
#define LOOKUP_TYPE_FIELDTYPE(type, length) extern void lookup##type(EnumFieldtypeCallback cb);
#else
#define LOOKUP_TYPE(type, length) extern const char *lookup##type(size_t val);
#define LOOKUP_TYPE_TRIPLET(type, length) extern const char *lookup##type(size_t val1, size_t val2);
#define LOOKUP_TYPE_BITFIELD(type, length) extern const char *lookup##type(size_t val);
#define LOOKUP_TYPE_FIELDTYPE(type, length) extern const char *lookup##type(size_t val);
#endif

#include "lookup.h"

typedef enum PacketComplete
{
  PACKET_COMPLETE               = 0,
  PACKET_FIELDS_UNKNOWN         = 1,
  PACKET_FIELD_LENGTHS_UNKNOWN  = 2,
  PACKET_RESOLUTION_UNKNOWN     = 4,
  PACKET_LOOKUPS_UNKNOWN        = 8,
  PACKET_NOT_SEEN               = 16,
  PACKET_INTERVAL_UNKNOWN       = 32,
  PACKET_MISSING_COMPANY_FIELDS = 64
} PacketComplete;

#define PACKET_INCOMPLETE (PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN)
#define PACKET_INCOMPLETE_LOOKUP (PACKET_INCOMPLETE | PACKET_LOOKUPS_UNKNOWN)
#define PACKET_PDF_ONLY (PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN | PACKET_LOOKUPS_UNKNOWN | PACKET_NOT_SEEN)

typedef enum PacketType
{
  PACKET_SINGLE,
  PACKET_FAST,
  PACKET_ISO_TP,
  PACKET_MIXED,
} PacketType;

#ifdef GLOBALS
const char *PACKET_TYPE_STR[PACKET_MIXED + 1] = {"Single", "Fast", "ISO", "Mixed"};
#else
extern const char *PACKET_TYPE_STR[];
#endif

struct Pgn
{
  char      *description;
  uint32_t   pgn;
  uint16_t   complete;      /* Either PACKET_COMPLETE or bit values set for various unknown items */
  PacketType type;          /* Single, Fast or ISO_TP */
  Field      fieldList[33]; /* Note fixed # of fields; increase if needed. RepeatingFields support means this is enough for now. */
  uint32_t   fieldCount;    /* Filled by C, no need to set in initializers. */
  // uint32_t    size;          /* Filled by C, no need to set in initializers. */
  char       *camelDescription; /* Filled by C, no need to set in initializers. */
  bool        fallback;         /* true = this is a catch-all for unknown PGNs */
  bool        hasMatchFields;   /* true = there are multiple PGNs with same PRN */
  const char *explanation;      /* Preferably the NMEA 2000 explanation from the NMEA PGN field list */
  const char *url;              /* External URL */
  const char *researchDoc;      /* Basename (no extension) of a local research HTML doc in docs/, rendered as a link */
  uint16_t    interval;         /* Milliseconds between transmissions, standard. 0 is: not known, UINT16_MAX = never */
  uint8_t     priority;         /* Default priority */
  uint8_t     repeatingCount1;  /* How many fields repeat in set 1? */
  uint8_t     repeatingCount2;  /* How many fields repeat in set 2? */
  uint8_t     repeatingStart1;  /* At which field does the first set start? */
  uint8_t     repeatingStart2;  /* At which field does the second set start? */
  uint8_t     repeatingField1;  /* Which field explains how often the repeating fields set #1 repeats? 255 = there is no field */
  uint8_t     repeatingField2;  /* Which field explains how often the repeating fields set #2 repeats? 255 = there is no field */
};

typedef struct PgnRange
{
  uint32_t    pgnStart;
  uint32_t    pgnEnd;
  uint32_t    pgnStep;
  const char *who;
  PacketType  type;
} PgnRange;

// Returns the first pgn that matches the given id, or NULL if not found.
const Pgn *searchForPgn(int pgn);

// Returns the catch-all PGN that matches the given id.
const Pgn *searchForUnknownPgn(int pgnId);

// Returns a pointer (potentially invalid) to the first pgn that does not match "first".
const Pgn *endPgn(const Pgn *first);

const Pgn *getMatchingPgn(int pgnId, const uint8_t *dataStart, int length);
const Pgn *getMatchingPgnByParameters(int pgnId, const uint8_t *data, int length);

bool printPgn(const RawMessage *msg, const uint8_t *dataStart, int length, bool showData, bool showJson);
void checkPgnList(void);

const Field *getField(uint32_t pgn, uint32_t field);
bool         extractNumber(const Field   *field,
                           const uint8_t *data,
                           size_t         dataLen,
                           size_t         startBit,
                           size_t         bits,
                           int64_t       *value,
                           int64_t       *maxValue);
bool         extractNumberByOrder(const Pgn *pgn, size_t order, const uint8_t *data, size_t dataLen, int64_t *value);

void camelCase(bool upperCamelCase);

/* lookup.c */
extern void fillLookups(void);

#define IS_MANUFACTURER_PGN(x) (((x) >= 0xff00 && (x) <= 0xffff) || (x) >= 0x1ef00 || ((x) >= 0x1ff00 && (x) <= 0x1ffff))

#ifdef GLOBALS
PgnRange pgnRange[] = {{0xe800, 0xee00, 256, "ISO 11783", PACKET_SINGLE},
                       {0xef00, 0xef00, 256, "NMEA", PACKET_SINGLE},
                       {0xf000, 0xfeff, 1, "NMEA", PACKET_SINGLE},
                       {0xff00, 0xffff, 1, "Manufacturer", PACKET_SINGLE},
                       {0x1ed00, 0x1ee00, 256, "NMEA", PACKET_FAST},
                       {0x1ef00, 0x1ef00, 256, "Manufacturer", PACKET_FAST},
                       {0x1f000, 0x1feff, 1, "NMEA", PACKET_MIXED},
                       {0x1ff00, 0x1ffff, 1, "Manufacturer", PACKET_FAST}};

#include "pgn-j1939-data.h"

const size_t pgnListSize  = ARRAY_SIZE(pgnList);
const size_t pgnRangeSize = ARRAY_SIZE(pgnRange);

#else
extern Pgn      pgnList[];
extern size_t   pgnListSize;
extern PgnRange pgnRange[];
extern size_t   pgnRangeSize;
#endif
