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
    void        (*pairEnumerator)(EnumPairCallback);
    void        (*bitEnumerator)(BitPairCallback);
    void        (*tripletEnumerator)(EnumTripletCallback);
    void        (*fieldtypeEnumerator)(EnumFieldtypeCallback);
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

  int32_t offset;     /* Only used for SAE J1939 values with sign; these are in Offset/Excess-K notation instead
                       *    of two's complement as used by NMEA 2000.
                       *    See http://en.wikipedia.org/wiki/Offset_binary
                       */
  double resolution;  /* Either a positive real value or zero */
  int    precision;   /* How many decimal digits after the decimal point to print; usually 0 = automatic */
  double unitOffset;  /* Only used for K->C conversion in non-SI print */
  bool   proprietary; /* Field is only present if earlier PGN field is in proprietary range */
  bool   hasSign;     /* Is the value signed, e.g. has both positive and negative values? */

  /* The following fields are filled by C, no need to set in initializers */
  uint8_t    order;
  size_t     bitOffset; // Bit offset from start of data, e.g. lower 3 bits = bit#, bit 4.. is byte offset
  char      *camelName;
  LookupInfo lookup;
  FieldType *ft;
  Pgn       *pgn;
  double     rangeMin;
  double     rangeMax;
} Field;

#include "fieldtype.h"

#define END_OF_FIELDS \
  {                   \
    0                 \
  }

#define LOOKUP_FIELD(nam, len, typ)                                                               \
  {                                                                                               \
    .name = nam, .size = len, .resolution = 1, .hasSign = false, .lookup.type = LOOKUP_TYPE_PAIR, \
    LOOKUP_PAIR_MEMBER = lookup##typ, .lookup.name = xstr(typ), .fieldType = "LOOKUP"             \
  }

#define LOOKUP_FIELDTYPE_FIELD(nam, len, typ)                                                          \
  {                                                                                                    \
    .name = nam, .size = len, .resolution = 1, .hasSign = false, .lookup.type = LOOKUP_TYPE_FIELDTYPE, \
    LOOKUP_FIELDTYPE_MEMBER = lookup##typ, .lookup.name = xstr(typ), .fieldType = "FIELDTYPE_LOOKUP"   \
  }

#define LOOKUP_TRIPLET_FIELD(nam, len, typ, desc, order)                                                                      \
  {                                                                                                                           \
    .name = nam, .size = len, .resolution = 1, .hasSign = false, .lookup.type = LOOKUP_TYPE_TRIPLET,                          \
    LOOKUP_TRIPLET_MEMBER = lookup##typ, .lookup.name = xstr(typ), .lookup.val1Order = order, .fieldType = "INDIRECT_LOOKUP", \
    .description = desc                                                                                                       \
  }

#define LOOKUP_FIELD_DESC(nam, len, typ, desc)                                                             \
  {                                                                                                        \
    .name = nam, .size = len, .resolution = 1, .hasSign = false, .lookup.type = LOOKUP_TYPE_PAIR,          \
    LOOKUP_PAIR_MEMBER = lookup##typ, .lookup.name = xstr(typ), .fieldType = "LOOKUP", .description = desc \
  }

#define BITLOOKUP_FIELD(nam, len, typ)                                                                                            \
  {                                                                                                                               \
    .name = nam, .size = len, .resolution = 1, .hasSign = false, .lookup.type = LOOKUP_TYPE_BIT, LOOKUP_BIT_MEMBER = lookup##typ, \
    .lookup.name = xstr(typ), .fieldType = "BITLOOKUP"                                                                            \
  }

#define FIELDTYPE_LOOKUP(nam, len, typ)                                                                   \
  {                                                                                                       \
    .name = nam, .size = len, .resolution = 1, .hasSign = false, .lookup.type = LOOKUP_TYPE_FIELDTYPE,    \
    LOOKUP_FIELDTYPE_MEMBER = lookup##typ, .lookup.name = xstr(typ), .fieldType = "LOOKUP_TYPE_FIELDTYPE" \
  }

#define UNKNOWN_LOOKUP_FIELD(nam, len)                                                                                  \
  {                                                                                                                     \
    .name = nam, .size = len, .resolution = 1, .hasSign = false, .lookup.type = LOOKUP_TYPE_PAIR, .fieldType = "LOOKUP" \
  }

#define SPARE_NAMED_FIELD(nam, len)                                   \
  {                                                                   \
    .name = nam, .size = (len), .resolution = 1, .fieldType = "SPARE" \
  }

#define SPARE_FIELD(len) SPARE_NAMED_FIELD("Spare", len)

#define RESERVED_FIELD(len)                                                     \
  {                                                                             \
    .name = "Reserved", .size = (len), .resolution = 1, .fieldType = "RESERVED" \
  }

#define RESERVED_PROP_FIELD(len, desc)                                                                                    \
  {                                                                                                                       \
    .name = "Reserved", .size = (len), .resolution = 1, .description = desc, .fieldType = "RESERVED", .proprietary = true \
  }

#define BINARY_FIELD(nam, len, desc)                                                        \
  {                                                                                         \
    .name = nam, .size = (len), .resolution = 1, .description = desc, .fieldType = "BINARY" \
  }

#define BINARY_UNIT_FIELD(nam, len, unt, desc, prop)                                                                          \
  {                                                                                                                           \
    .name = nam, .size = (len), .resolution = 1, .unit = unt, .description = desc, .proprietary = prop, .fieldType = "BINARY" \
  }

#define LATITUDE_I32_FIELD(nam)                                                                                 \
  {                                                                                                             \
    .name = nam, .size = BYTES(4), .resolution = 1e-7, .hasSign = true, .unit = "deg", .fieldType = "GEO_FIX32" \
  }

#define LATITUDE_I64_FIELD(nam)                                                                                  \
  {                                                                                                              \
    .name = nam, .size = BYTES(8), .resolution = 1e-16, .hasSign = true, .unit = "deg", .fieldType = "GEO_FIX64" \
  }

#define LONGITUDE_I32_FIELD(nam)                                                                                \
  {                                                                                                             \
    .name = nam, .size = BYTES(4), .resolution = 1e-7, .hasSign = true, .unit = "deg", .fieldType = "GEO_FIX32" \
  }

#define LONGITUDE_I64_FIELD(nam)                                                                                 \
  {                                                                                                              \
    .name = nam, .size = BYTES(8), .resolution = 1e-16, .hasSign = true, .unit = "deg", .fieldType = "GEO_FIX64" \
  }

#define ANGLE_U16_FIELD(nam, desc)                                                                                  \
  {                                                                                                                 \
    .name = nam, .size = BYTES(2), .resolution = RES_RADIANS, .hasSign = false, .unit = "rad", .description = desc, \
    .fieldType = "ANGLE_UFIX16"                                                                                     \
  }

#define ANGLE_I16_FIELD(nam, desc)                                                                                 \
  {                                                                                                                \
    .name = nam, .size = BYTES(2), .resolution = RES_RADIANS, .hasSign = true, .unit = "rad", .description = desc, \
    .fieldType = "ANGLE_FIX16"                                                                                     \
  }

#define INT32_FIELD(nam, desc)                                                                                 \
  {                                                                                                            \
    .name = nam, .size = BYTES(4), .resolution = 1, .hasSign = true, .fieldType = "INT32", .description = desc \
  }

// A whole bunch of different NUMBER fields, with variing resolutions

#define UNSIGNED_ALMANAC_PARAMETER_FIELD(nam, len, res, unt, desc)                                   \
  {                                                                                                  \
    .name = nam, .size = len, .resolution = res, .hasSign = false, .unit = unt, .description = desc, \
    .fieldType = "UNSIGNED_ALMANAC_PARAMETER"                                                        \
  }

#define SIGNED_ALMANAC_PARAMETER_FIELD(nam, len, res, unt, desc)                                    \
  {                                                                                                 \
    .name = nam, .size = len, .resolution = res, .hasSign = true, .unit = unt, .description = desc, \
    .fieldType = "SIGNED_ALMANAC_PARAMETER"                                                         \
  }

#define DILUTION_OF_PRECISION_UFIX16_FIELD(nam, desc)                                                                   \
  {                                                                                                                     \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .fieldType = "DILUTION_OF_PRECISION_UFIX16", .description = desc \
  }

#define DILUTION_OF_PRECISION_FIX16_FIELD(nam, desc)                                                                \
  {                                                                                                                 \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .hasSign = true, .fieldType = "DILUTION_OF_PRECISION_FIX16", \
    .description = desc                                                                                             \
  }

#define SIGNALTONOISERATIO_UFIX16_FIELD(nam, desc)                                                                   \
  {                                                                                                                  \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .fieldType = "SIGNALTONOISERATIO_UFIX16", .description = desc \
  }

#define SIGNALTONOISERATIO_FIX16_FIELD(nam, desc)                                                                \
  {                                                                                                              \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .hasSign = true, .fieldType = "SIGNALTONOISERATIO_FIX16", \
    .description = desc                                                                                          \
  }

#define VERSION_FIELD(nam)                                                     \
  {                                                                            \
    .name = nam, .size = BYTES(2), .resolution = 0.001, .fieldType = "VERSION" \
  }

#define VOLTAGE_U16_V_FIELD(nam)                                                                   \
  {                                                                                                \
    .name = nam, .size = BYTES(2), .resolution = 1.0, .unit = "V", .fieldType = "VOLTAGE_UFIX16_V" \
  }

#define VOLTAGE_U16_10MV_FIELD(nam)                                                                    \
  {                                                                                                    \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "V", .fieldType = "VOLTAGE_UFIX16_10MV" \
  }

#define VOLTAGE_U16_50MV_FIELD(nam)                                                                    \
  {                                                                                                    \
    .name = nam, .size = BYTES(2), .resolution = 0.05, .unit = "V", .fieldType = "VOLTAGE_UFIX16_50MV" \
  }

#define VOLTAGE_U16_100MV_FIELD(nam)                                                                   \
  {                                                                                                    \
    .name = nam, .size = BYTES(2), .resolution = 0.1, .unit = "V", .fieldType = "VOLTAGE_UFIX16_100MV" \
  }

#define VOLTAGE_UFIX8_200MV_FIELD(nam)                                                                \
  {                                                                                                   \
    .name = nam, .size = BYTES(1), .resolution = 0.2, .unit = "V", .fieldType = "VOLTAGE_UFIX8_200MV" \
  }

#define VOLTAGE_I16_10MV_FIELD(nam)                                                                                    \
  {                                                                                                                    \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "V", .hasSign = true, .fieldType = "VOLTAGE_FIX16_10MV" \
  }

#define RADIO_FREQUENCY_FIELD(nam, res)                                                                   \
  {                                                                                                       \
    .name = nam, .size = BYTES(4), .resolution = res, .unit = "Hz", .fieldType = "RADIO_FREQUENCY_UFIX32" \
  }

#define FREQUENCY_FIELD(nam, res)                                                                   \
  {                                                                                                 \
    .name = nam, .size = BYTES(2), .resolution = res, .unit = "Hz", .fieldType = "FREQUENCY_UFIX16" \
  }

#define SPEED_I16_MM_FIELD(nam)                                                                                       \
  {                                                                                                                   \
    .name = nam, .size = BYTES(2), .resolution = 0.001, .unit = "m/s", .hasSign = true, .fieldType = "SPEED_FIX16_MM" \
  }

#define SPEED_I16_CM_FIELD(nam)                                                                                      \
  {                                                                                                                  \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "m/s", .hasSign = true, .fieldType = "SPEED_FIX16_CM" \
  }

#define SPEED_U16_CM_FIELD(nam)                                                                      \
  {                                                                                                  \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "m/s", .fieldType = "SPEED_UFIX16_CM" \
  }

#define SPEED_U16_DM_FIELD(nam, desc)                                                                                    \
  {                                                                                                                      \
    .name = nam, .size = BYTES(2), .resolution = 0.1, .unit = "m/s", .fieldType = "SPEED_UFIX16_DM", .description = desc \
  }

#define DISTANCE_FIX16_M_FIELD(nam, desc)                                                              \
  {                                                                                                    \
    .name = nam, .size = BYTES(2), .resolution = 1, .hasSign = true, .unit = "m", .description = desc, \
    .fieldType = "DISTANCE_FIX16_M"                                                                    \
  }

#define DISTANCE_FIX16_CM_FIELD(nam, desc)                                                                \
  {                                                                                                       \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .hasSign = true, .unit = "m", .description = desc, \
    .fieldType = "DISTANCE_FIX16_CM"                                                                      \
  }

#define DISTANCE_FIX16_MM_FIELD(nam, desc)                                                                 \
  {                                                                                                        \
    .name = nam, .size = BYTES(2), .resolution = 0.001, .hasSign = true, .unit = "m", .description = desc, \
    .fieldType = "DISTANCE_FIX16_MM"                                                                       \
  }

#define DISTANCE_FIX32_MM_FIELD(nam, desc)                                                                 \
  {                                                                                                        \
    .name = nam, .size = BYTES(4), .resolution = 0.001, .hasSign = true, .unit = "m", .description = desc, \
    .fieldType = "DISTANCE_FIX32_MM"                                                                       \
  }

#define DISTANCE_FIX32_CM_FIELD(nam, desc)                                                                \
  {                                                                                                       \
    .name = nam, .size = BYTES(4), .resolution = 0.01, .hasSign = true, .unit = "m", .description = desc, \
    .fieldType = "DISTANCE_FIX32_CM"                                                                      \
  }

#define DISTANCE_FIX64_FIELD(nam, desc)                                                                   \
  {                                                                                                       \
    .name = nam, .size = BYTES(8), .resolution = 1e-6, .hasSign = true, .unit = "m", .description = desc, \
    .fieldType = "DISTANCE_FIX64"                                                                         \
  }

#define LENGTH_UFIX8_DAM_FIELD(nam, desc)                                                                       \
  {                                                                                                             \
    .name = nam, .size = 8, .resolution = 10, .unit = "m", .fieldType = "LENGTH_UFIX8_DAM", .description = desc \
  }

#define LENGTH_UFIX16_CM_FIELD(nam)                                                           \
  {                                                                                           \
    .name = nam, .size = 16, .resolution = 0.01, .unit = "m", .fieldType = "LENGTH_UFIX16_CM" \
  }

#define LENGTH_UFIX16_DM_FIELD(nam)                                                          \
  {                                                                                          \
    .name = nam, .size = 16, .resolution = 0.1, .unit = "m", .fieldType = "LENGTH_UFIX16_DM" \
  }

#define LENGTH_UFIX32_M_FIELD(nam, desc)                                                                       \
  {                                                                                                            \
    .name = nam, .size = 32, .resolution = 1, .unit = "m", .fieldType = "LENGTH_UFIX32_M", .description = desc \
  }

#define LENGTH_UFIX32_CM_FIELD(nam, desc)                                                                          \
  {                                                                                                                \
    .name = nam, .size = 32, .resolution = 0.01, .unit = "m", .fieldType = "LENGTH_UFIX32_CM", .description = desc \
  }

#define LENGTH_UFIX32_MM_FIELD(nam)                                                            \
  {                                                                                            \
    .name = nam, .size = 32, .resolution = 0.001, .unit = "m", .fieldType = "LENGTH_UFIX32_MM" \
  }

#define CURRENT_UFIX8_A_FIELD(nam)                                                              \
  {                                                                                             \
    .name = nam, .size = BYTES(1), .resolution = 1, .unit = "A", .fieldType = "CURRENT_UFIX8_A" \
  }

#define CURRENT_UFIX16_A_FIELD(nam)                                                              \
  {                                                                                              \
    .name = nam, .size = BYTES(2), .resolution = 1, .unit = "A", .fieldType = "CURRENT_UFIX16_A" \
  }

#define CURRENT_UFIX16_DA_FIELD(nam)                                                                \
  {                                                                                                 \
    .name = nam, .size = BYTES(2), .resolution = 0.1, .unit = "A", .fieldType = "CURRENT_UFIX16_DA" \
  }

#define CURRENT_FIX16_DA_FIELD(nam)                                                                                 \
  {                                                                                                                 \
    .name = nam, .size = BYTES(2), .resolution = 0.1, .hasSign = true, .unit = "A", .fieldType = "CURRENT_FIX16_DA" \
  }

#define CURRENT_FIX24_CA_FIELD(nam)                                                                                  \
  {                                                                                                                  \
    .name = nam, .size = BYTES(3), .resolution = 0.01, .hasSign = true, .unit = "A", .fieldType = "CURRENT_FIX24_CA" \
  }

#define ELECTRIC_CHARGE_UFIX16_AH(nam)                    \
  {                                                       \
    .name = nam, .fieldType = "ELECTRIC_CHARGE_UFIX16_AH" \
  }

#define PEUKERT_FIELD(nam)                       \
  {                                              \
    .name = nam, .fieldType = "PEUKERT_EXPONENT" \
  }

// Fully defined NUMBER fields

#define PGN_FIELD(nam, desc)                                                                \
  {                                                                                         \
    .name = nam, .size = BYTES(3), .resolution = 1, .fieldType = "PGN", .description = desc \
  }

#define INSTANCE_FIELD                                                                               \
  {                                                                                                  \
    .name = "Instance", .size = BYTES(1), .resolution = 1, .description = NULL, .fieldType = "UINT8" \
  }

#define POWER_FACTOR_U16_FIELD                                                                                   \
  {                                                                                                              \
    .name = "Power factor", .size = BYTES(2), .resolution = 1 / 16384., .unit = "Cos Phi", .fieldType = "UFIX16" \
  }

#define POWER_FACTOR_U8_FIELD                                                                             \
  {                                                                                                       \
    .name = "Power factor", .size = BYTES(1), .resolution = 0.01, .unit = "Cos Phi", .fieldType = "UFIX8" \
  }

// End of NUMBER fields

#define MANUFACTURER_FIELD(unt, desc, prop)                                                                                      \
  {                                                                                                                              \
    .name = "Manufacturer Code", .size = 11, .resolution = 1, .description = desc, .unit = unt, .lookup.type = LOOKUP_TYPE_PAIR, \
    LOOKUP_PAIR_MEMBER = lookupMANUFACTURER_CODE, .lookup.name = "MANUFACTURER_CODE", .proprietary = prop,                       \
    .fieldType = "MANUFACTURER"                                                                                                  \
  }

#define INDUSTRY_FIELD(unt, desc, prop)                                                                                     \
  {                                                                                                                         \
    .name = "Industry Code", .size = 3, .resolution = 1, .unit = unt, .description = desc, .lookup.type = LOOKUP_TYPE_PAIR, \
    LOOKUP_PAIR_MEMBER = lookupINDUSTRY_CODE, .lookup.name = "INDUSTRY_CODE", .proprietary = prop, .fieldType = "INDUSTRY"  \
  }

#define MARINE_INDUSTRY_FIELD INDUSTRY_FIELD("=4", "Marine Industry", false)

#define COMPANY(id) MANUFACTURER_FIELD("=" xstr(id), NULL, false), RESERVED_FIELD(2), MARINE_INDUSTRY_FIELD

#define MANUFACTURER_FIELDS MANUFACTURER_FIELD(NULL, NULL, false), RESERVED_FIELD(2), INDUSTRY_FIELD(NULL, NULL, false)

#define MANUFACTURER_PROPRIETARY_FIELDS                                            \
  MANUFACTURER_FIELD(NULL, "Only in PGN when Commanded PGN is proprietary", true), \
      RESERVED_PROP_FIELD(2, "Only in PGN when Commanded PGN is proprietary"),     \
      INDUSTRY_FIELD(NULL, "Only in PGN when Commanded PGN is proprietary", true)

#define INTEGER_DESC_FIELD(nam, len, desc)                         \
  {                                                                \
    .name = nam, .size = len, .resolution = 1, .description = desc \
  }

#define INTEGER_UNIT_FIELD(nam, len, unt)                  \
  {                                                        \
    .name = nam, .size = len, .resolution = 1, .unit = unt \
  }

#define SIGNED_INTEGER_UNIT_FIELD(nam, len, unt)                            \
  {                                                                         \
    .name = nam, .size = len, .resolution = 1, .unit = unt, .hasSign = true \
  }

#define INTEGER_FIELD(nam, len) INTEGER_DESC_FIELD(nam, len, "")

#define UINT8_DESC_FIELD(nam, desc)                                                           \
  {                                                                                           \
    .name = nam, .size = BYTES(1), .resolution = 1, .fieldType = "UINT8", .description = desc \
  }

#define FIELD_INDEX(nam, desc)                                                                      \
  {                                                                                                 \
    .name = nam, .size = BYTES(1), .resolution = 1, .fieldType = "FIELD_INDEX", .description = desc \
  }

#define UINT8_FIELD(nam) UINT8_DESC_FIELD(nam, NULL)

#define UINT16_DESC_FIELD(nam, desc)                                                           \
  {                                                                                            \
    .name = nam, .size = BYTES(2), .resolution = 1, .fieldType = "UINT16", .description = desc \
  }

#define UINT16_FIELD(nam) UINT16_DESC_FIELD(nam, NULL)

#define UINT32_DESC_FIELD(nam, desc)                                                           \
  {                                                                                            \
    .name = nam, .size = BYTES(4), .resolution = 1, .fieldType = "UINT32", .description = desc \
  }

#define UINT32_FIELD(nam) UINT32_DESC_FIELD(nam, NULL)

#define MATCH_LOOKUP_FIELD(nam, len, id, typ)                                                                \
  {                                                                                                          \
    .name = nam, .size = len, .resolution = 1, .hasSign = false, .lookup.type = LOOKUP_TYPE_PAIR,            \
    LOOKUP_PAIR_MEMBER = lookup##typ, .lookup.name = xstr(typ), .fieldType = "LOOKUP", .unit = "=" xstr(id), \
  }

#define MATCH_FIELD(nam, len, id, desc)                                                                                   \
  {                                                                                                                       \
    .name = nam, .size = len, .resolution = 1, .unit = "=" xstr(id), .description = desc, .fieldType = "UNSIGNED_INTEGER" \
  }

#define SIMPLE_DESC_FIELD(nam, len, desc)                                                           \
  {                                                                                                 \
    .name = nam, .size = len, .resolution = 1, .description = desc, .fieldType = "UNSIGNED_INTEGER" \
  }

#define SIMPLE_FIELD(nam, len)                                                 \
  {                                                                            \
    .name = nam, .size = len, .resolution = 1, .fieldType = "UNSIGNED_INTEGER" \
  }

#define SIMPLE_SIGNED_FIELD(nam, len)                                                  \
  {                                                                                    \
    .name = nam, .size = len, .resolution = 1, .hasSign = true, .fieldType = "INTEGER" \
  }

#define MMSI_FIELD(nam)                                                                                           \
  {                                                                                                               \
    .name = nam, .size = BYTES(4), .resolution = 1, .hasSign = false, .rangeMin = 2000000, .rangeMax = 999999999, \
    .fieldType = "MMSI"                                                                                           \
  }

#define DECIMAL_FIELD(nam, len, desc)                                                      \
  {                                                                                        \
    .name = nam, .size = len, .resolution = 1, .description = desc, .fieldType = "DECIMAL" \
  }

#define DECIMAL_UNIT_FIELD(nam, len, unt)                                          \
  {                                                                                \
    .name = nam, .size = len, .resolution = 1, .unit = unt, .fieldType = "DECIMAL" \
  }

#define STRINGLZ_FIELD(nam, len)                                        \
  {                                                                     \
    .name = nam, .size = len, .resolution = 0, .fieldType = "STRING_LZ" \
  }

#define STRING_FIX_DESC_FIELD(nam, len, desc)                                                 \
  {                                                                                           \
    .name = nam, .size = len, .resolution = 0, .description = desc, .fieldType = "STRING_FIX" \
  }

#define STRINGVAR_FIELD(nam)                                                     \
  {                                                                              \
    .name = nam, .size = LEN_VARIABLE, .resolution = 0, .fieldType = "STRING_LZ" \
  }

#define STRINGLAU_FIELD(nam)                                                      \
  {                                                                               \
    .name = nam, .size = LEN_VARIABLE, .resolution = 0, .fieldType = "STRING_LAU" \
  }

#define STRING_FIX_FIELD(nam, len) STRING_FIX_DESC_FIELD(nam, len, NULL)

#define TEMPERATURE_HIGH_FIELD(nam)                                                                \
  {                                                                                                \
    .name = nam, .size = BYTES(2), .resolution = 0.1, .unit = "K", .fieldType = "TEMPERATURE_HIGH" \
  }

#define TEMPERATURE_FIELD(nam)                                                                 \
  {                                                                                            \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "K", .fieldType = "TEMPERATURE" \
  }

#define TEMPERATURE_UINT8_OFFSET_FIELD(nam)                                                                             \
  {                                                                                                                     \
    .name = nam, .size = BYTES(1), .offset = 233, .resolution = 1, .unit = "K", .fieldType = "TEMPERATURE_UINT8_OFFSET" \
  }

#define TEMPERATURE_U24_FIELD(nam)                                                                     \
  {                                                                                                    \
    .name = nam, .size = BYTES(3), .resolution = 0.001, .unit = "K", .fieldType = "TEMPERATURE_UFIX24" \
  }

#define TEMPERATURE_DELTA_FIX16_FIELD(nam, desc)                                                                                \
  {                                                                                                                             \
    .name = nam, .size = BYTES(2), .resolution = 0.001, .unit = "K", .hasSign = true, .fieldType = "FIX16", .description = desc \
  }

#define VOLUMETRIC_FLOW_FIELD(nam)                                                                                   \
  {                                                                                                                  \
    .name = nam, .size = BYTES(2), .resolution = 0.1, .unit = "L/h", .hasSign = true, .fieldType = "VOLUMETRIC_FLOW" \
  }

#define CONCENTRATION_UINT16_FIELD(nam)                                                                    \
  {                                                                                                        \
    .name = nam, .size = BYTES(2), .resolution = 1, .unit = "ppm", .fieldType = "CONCENTRATION_UINT16_PPM" \
  }

#define VOLUME_UFIX16_L_FIELD(nam)                                                              \
  {                                                                                             \
    .name = nam, .size = BYTES(2), .resolution = 1, .unit = "L", .fieldType = "VOLUME_UFIX16_L" \
  }

#define VOLUME_UFIX32_DL_FIELD(nam)                                                                \
  {                                                                                                \
    .name = nam, .size = BYTES(4), .resolution = 0.1, .unit = "L", .fieldType = "VOLUME_UFIX32_DL" \
  }

#define TIME_UFIX16_S_FIELD(nam)                                                              \
  {                                                                                           \
    .name = nam, .size = BYTES(2), .resolution = 1, .unit = "s", .fieldType = "TIME_UFIX16_S" \
  }

#define TIME_FIX32_MS_FIELD(nam, desc)                                                                              \
  {                                                                                                                 \
    .name = nam, .size = BYTES(4), .resolution = 0.001, .unit = "s", .hasSign = true, .fieldType = "TIME_FIX32_MS", \
    .description = desc                                                                                             \
  }

#define TIME_UFIX8_5MS_FIELD(nam, desc)                                                                               \
  {                                                                                                                   \
    .name = nam, .size = BYTES(1), .resolution = 0.005, .unit = "s", .hasSign = false, .fieldType = "TIME_UFIX8_5MS", \
    .description = desc                                                                                               \
  }

#define TIME_UFIX16_MIN_FIELD(nam, desc)                                                                            \
  {                                                                                                                 \
    .name = nam, .size = BYTES(2), .resolution = 60, .unit = "s", .hasSign = false, .fieldType = "TIME_UFIX16_MIN", \
    .description = desc                                                                                             \
  }

#define TIME_UFIX16_MS_FIELD(nam, desc)                                                                               \
  {                                                                                                                   \
    .name = nam, .size = BYTES(2), .resolution = 0.001, .unit = "s", .hasSign = false, .fieldType = "TIME_UFIX16_MS", \
    .description = desc                                                                                               \
  }

#define TIME_UFIX16_CS_FIELD(nam, desc)                                                                              \
  {                                                                                                                  \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .unit = "s", .hasSign = false, .fieldType = "TIME_UFIX16_CS", \
    .description = desc                                                                                              \
  }

#define TIME_FIX16_5CS_FIELD(nam, desc)                                                                             \
  {                                                                                                                 \
    .name = nam, .size = BYTES(2), .resolution = 0.05, .unit = "s", .hasSign = true, .fieldType = "TIME_FIX16_5CS", \
    .description = desc                                                                                             \
  }

#define TIME_FIX16_MIN_FIELD(nam)                                                                                 \
  {                                                                                                               \
    .name = nam, .size = BYTES(2), .resolution = 60., .unit = "s", .hasSign = true, .fieldType = "TIME_FIX16_MIN" \
  }

#define TIME_UFIX24_MS_FIELD(nam, desc)                                                                               \
  {                                                                                                                   \
    .name = nam, .size = BYTES(3), .resolution = 0.001, .unit = "s", .hasSign = false, .fieldType = "TIME_UFIX24_MS", \
    .description = desc                                                                                               \
  }

#define TIME_UFIX32_S_FIELD(nam, desc)                                                                           \
  {                                                                                                              \
    .name = nam, .size = BYTES(4), .resolution = 1, .unit = "s", .hasSign = false, .fieldType = "TIME_UFIX32_S", \
    .description = desc                                                                                          \
  }

#define TIME_UFIX32_MS_FIELD(nam, desc)                                                                               \
  {                                                                                                                   \
    .name = nam, .size = BYTES(4), .resolution = 0.001, .unit = "s", .hasSign = false, .fieldType = "TIME_UFIX32_MS", \
    .description = desc                                                                                               \
  }

#define TIME_FIELD(nam)                                                                                      \
  {                                                                                                          \
    .name = nam, .size = BYTES(4), .resolution = 0.0001, .unit = "s", .hasSign = false, .fieldType = "TIME", \
    .description = "Seconds since midnight", .rangeMin = 0, .rangeMax = 86402                                \
  }

#define DATE_FIELD(nam)                                                                                \
  {                                                                                                    \
    .name = nam, .size = BYTES(2), .resolution = 1, .unit = "d", .hasSign = false, .fieldType = "DATE" \
  }

#define VARIABLE_FIELD(nam, desc)                                                   \
  {                                                                                 \
    .name = nam, .size = LEN_VARIABLE, .description = desc, .fieldType = "VARIABLE" \
  }

#define KEY_VALUE_FIELD(nam, desc)                                                   \
  {                                                                                  \
    .name = nam, .size = LEN_VARIABLE, .description = desc, .fieldType = "KEY_VALUE" \
  }

#define ENERGY_UINT32_FIELD(nam)                                                                \
  {                                                                                             \
    .name = nam, .size = BYTES(4), .resolution = 1, .unit = "kWh", .fieldType = "ENERGY_UINT32" \
  }

#define POWER_I32_OFFSET_FIELD(nam)                                 \
  {                                                                 \
    .name = nam, .hasSign = true, .fieldType = "POWER_FIX32_OFFSET" \
  }

#define POWER_I32_VA_OFFSET_FIELD(nam)                                 \
  {                                                                    \
    .name = nam, .hasSign = true, .fieldType = "POWER_FIX32_VA_OFFSET" \
  }

#define POWER_I32_VAR_OFFSET_FIELD(nam)                                 \
  {                                                                     \
    .name = nam, .hasSign = true, .fieldType = "POWER_FIX32_VAR_OFFSET" \
  }

#define POWER_U16_FIELD(nam)                                                                 \
  {                                                                                          \
    .name = nam, .size = BYTES(2), .resolution = 1, .unit = "W", .fieldType = "POWER_UINT16" \
  }

#define POWER_U16_VAR_FIELD(nam, desc)                                                                                  \
  {                                                                                                                     \
    .name = nam, .size = BYTES(2), .resolution = 1, .unit = "VAR", .description = desc, .fieldType = "POWER_UINT16_VAR" \
  }

#define POWER_I32_FIELD(nam)                                                                                 \
  {                                                                                                          \
    .name = nam, .size = BYTES(4), .resolution = 1, .hasSign = true, .unit = "W", .fieldType = "POWER_INT32" \
  }

#define POWER_U32_FIELD(nam)                                                                 \
  {                                                                                          \
    .name = nam, .size = BYTES(4), .resolution = 1, .unit = "W", .fieldType = "POWER_UINT32" \
  }

#define POWER_U32_VA_FIELD(nam)                                                                  \
  {                                                                                              \
    .name = nam, .size = BYTES(4), .resolution = 1, .unit = "VA", .fieldType = "POWER_UINT32_VA" \
  }

#define POWER_U32_VAR_FIELD(nam)                                                                   \
  {                                                                                                \
    .name = nam, .size = BYTES(4), .resolution = 1, .unit = "VAR", .fieldType = "POWER_UINT32_VAR" \
  }

#define PERCENTAGE_U8_FIELD(nam)                                                                 \
  {                                                                                              \
    .name = nam, .size = BYTES(1), .resolution = 1, .unit = "%", .fieldType = "PERCENTAGE_UINT8" \
  }

#define PERCENTAGE_U8_HIGHRES_FIELD(nam)                                                                  \
  {                                                                                                       \
    .name = nam, .size = BYTES(1), .resolution = .4, .unit = "%", .fieldType = "PERCENTAGE_UINT8_HIGHRES" \
  }

#define PERCENTAGE_I8_FIELD(nam)                                                                                 \
  {                                                                                                              \
    .name = nam, .size = BYTES(1), .resolution = 1, .hasSign = true, .unit = "%", .fieldType = "PERCENTAGE_INT8" \
  }

#define PERCENTAGE_I16_FIELD(nam)                                                                                              \
  {                                                                                                                            \
    .name = nam, .size = BYTES(2), .resolution = RES_PERCENTAGE, .hasSign = true, .unit = "%", .fieldType = "PERCENTAGE_FIX16" \
  }

#define ROTATION_FIX16_FIELD(nam)                                                                                               \
  {                                                                                                                             \
    .name = nam, .size = BYTES(2), .resolution = (1e-3 / 32.0), .hasSign = true, .unit = "rad/s", .fieldType = "ROTATION_FIX16" \
  }

#define ROTATION_UFIX16_RPM_FIELD(nam, desc)                                                                               \
  {                                                                                                                        \
    .name = nam, .size = BYTES(2), .resolution = 0.25, .hasSign = false, .unit = "rpm", .fieldType = "ROTATION_UFIX16_RPM" \
  }

#define ROTATION_UFIX16_RPM_HIGHRES_FIELD(nam, desc)                                     \
  {                                                                                      \
    .name = nam, .size = BYTES(2), .resolution = 0.125, .hasSign = false, .unit = "rpm", \
    .fieldType = "ROTATION_UFIX16_RPM_HIGHRES"                                           \
  }

#define ROTATION_FIX32_FIELD(nam)                                                                                               \
  {                                                                                                                             \
    .name = nam, .size = BYTES(4), .resolution = (1e-6 / 32.0), .hasSign = true, .unit = "rad/s", .fieldType = "ROTATION_FIX32" \
  }

#define PRESSURE_UFIX16_HPA_FIELD(nam)                                                                 \
  {                                                                                                    \
    .name = nam, .size = BYTES(2), .resolution = 100, .unit = "Pa", .fieldType = "PRESSURE_UFIX16_HPA" \
  }

#define PRESSURE_UINT8_KPA_FIELD(nam)                                                                 \
  {                                                                                                   \
    .name = nam, .size = BYTES(1), .resolution = 500, .unit = "Pa", .fieldType = "PRESSURE_UINT8_KPA" \
  }

#define PRESSURE_UINT8_2KPA_FIELD(nam)                                                                  \
  {                                                                                                     \
    .name = nam, .size = BYTES(1), .resolution = 2000, .unit = "Pa", .fieldType = "PRESSURE_UINT8_2KPA" \
  }

#define PRESSURE_UFIX16_KPA_FIELD(nam)                                                                                    \
  {                                                                                                                       \
    .name = nam, .size = BYTES(2), .resolution = 1000, .hasSign = false, .unit = "Pa", .fieldType = "PRESSURE_UFIX16_KPA" \
  }

#define PRESSURE_RATE_FIX16_PA_FIELD(nam)                                                                                   \
  {                                                                                                                         \
    .name = nam, .size = BYTES(2), .resolution = 1, .hasSign = true, .unit = "Pa/hr", .fieldType = "PRESSURE_RATE_FIX16_PA" \
  }

#define PRESSURE_FIX16_KPA_FIELD(nam)                                                                                   \
  {                                                                                                                     \
    .name = nam, .size = BYTES(2), .resolution = 1000, .hasSign = true, .unit = "Pa", .fieldType = "PRESSURE_FIX16_KPA" \
  }

#define PRESSURE_FIX32_DPA_FIELD(nam)                                                                                  \
  {                                                                                                                    \
    .name = nam, .size = BYTES(4), .resolution = 0.1, .hasSign = true, .unit = "Pa", .fieldType = "PRESSURE_FIX32_DPA" \
  }

#define PRESSURE_UFIX32_DPA_FIELD(nam)                                                                                   \
  {                                                                                                                      \
    .name = nam, .size = BYTES(4), .resolution = 0.1, .hasSign = false, .unit = "Pa", .fieldType = "PRESSURE_UFIX32_DPA" \
  }

#define GAIN_FIELD(nam, desc)                                                                                          \
  {                                                                                                                    \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .hasSign = true, .fieldType = "GAIN_FIX16", .description = desc \
  }

#define MAGNETIC_FIX16_FIELD(nam, desc)                                                                                  \
  {                                                                                                                      \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .hasSign = true, .unit = "T", .fieldType = "MAGNETIC_FIELD_FIX16" \
  }

#define ANGLE_FIX16_DDEG_FIELD(nam, desc)                                                                             \
  {                                                                                                                   \
    .name = nam, .size = BYTES(2), .resolution = 0.1, .hasSign = true, .unit = "deg", .fieldType = "ANGLE_FIX16_DDEG" \
  }

#define FLOAT_FIELD(nam, unt, desc)                                                                                          \
  {                                                                                                                          \
    .name = nam, .size = BYTES(4), .hasSign = true, .unit = unt, .fieldType = "FLOAT", .description = desc, .resolution = 1, \
    .rangeMin = -1 * FLT_MAX, .rangeMax = FLT_MAX                                                                            \
  }

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

#ifdef GLOBALS
PgnRange pgnRange[] = {{0xe800, 0xee00, 256, "ISO 11783", PACKET_SINGLE},
                       {0xef00, 0xef00, 256, "NMEA", PACKET_SINGLE},
                       {0xf000, 0xfeff, 1, "NMEA", PACKET_SINGLE},
                       {0xff00, 0xffff, 1, "Manufacturer", PACKET_SINGLE},
                       {0x1ed00, 0x1ee00, 256, "NMEA", PACKET_FAST},
                       {0x1ef00, 0x1ef00, 256, "Manufacturer", PACKET_FAST},
                       {0x1f000, 0x1feff, 1, "NMEA", PACKET_MIXED},
                       {0x1ff00, 0x1ffff, 1, "Manufacturer", PACKET_FAST}};

Pgn pgnList[] = {

    /* PDU1 (addressed) single-frame PGN range 0E800 to 0xEEFF (59392 - 61183) */

    {"0xE800-0xEEFF: Standardized single-frame addressed",
     0xe800,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {BINARY_FIELD("Data", BYTES(8), NULL), END_OF_FIELDS},
     .fallback    = true,
     .explanation = "Standardized PGNs in PDU1 (addressed) single-frame PGN range 0xE800 to "
                    "0xEE00 (59392 - 60928). "
                    "When this is shown during analysis it means the PGN is not reverse engineered yet."}

    /************ Protocol PGNs ************/
    /* http://www.nmea.org/Assets/july%202010%20nmea2000_v1-301_app_b_pgn_field_list.pdf */
    /* http://www.maretron.com/products/pdf/J2K100-Data_Sheet.pdf */
    /* http://www.nmea.org/Assets/pgn059392.pdf */
    /* http://www8.garmin.com/manuals/GPSMAP4008_NMEA2000NetworkFundamentals.pdf */
    /* http://www.furunousa.com/Furuno/Doc/0/8JT2BMDDIB249FCNUK64DKLV67/GP330B%20NMEA%20PGNs.pdf */
    /* http://www.nmea.org/Assets/20140710%20nmea-2000-060928%20iso%20address%20claim%20pgn%20corrigendum.pdf */
    ,
    {"ISO Acknowledgement",
     59392,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {LOOKUP_FIELD("Control", BYTES(1), ISO_CONTROL),
      UINT8_FIELD("Group Function"),
      RESERVED_FIELD(24),
      PGN_FIELD("PGN", "Parameter Group Number of requested information"),
      END_OF_FIELDS},
     .interval    = UINT16_MAX,
     .explanation = "This message is provided by ISO 11783 for a handshake mechanism between transmitting and receiving devices. "
                    "This message is the possible response to acknowledge the reception of a 'normal broadcast' message or the "
                    "response to a specific command to indicate compliance or failure."}

    ,
    {"ISO Request",
     59904,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {PGN_FIELD("PGN", NULL), END_OF_FIELDS},
     .interval    = UINT16_MAX,
     .explanation = "As defined by ISO, this message has a data length of 3 bytes with no padding added to complete the single "
                    "frame. The appropriate response to this message is based on the PGN being requested, and whether the receiver "
                    "supports the requested PGN."}

    /* For a good explanation of ISO 11783 Transport Protocol (as used in J1939) see
     * http://www.simmasoftware.com/j1939-presentation.pdf
     *
     * First: Transmit a RTS message to the specific address that says:
     *   1. I'm about to send the following PGN in multiple packets.
     *   2. I'm sending X amount of data.
     *   3. I'm sending Y number of packets.
     *   4. I can send Z number of packets at once.
     * Second: Wait for CTS: CTS says:
     *   1. I can receive M number of packets at once.
     *   2. Start sending with sequence number N.
     * Third: Send data. Then repeat steps starting with #2. When all data sent, wait for ACK.
     */

    // ISO 11783 defines this PGN as part of the Transport Protocol method used for transmitting messages that have 9 or more data
    // bytes. This PGN represents a single packet of a multipacket message.
    ,
    {"ISO Transport Protocol, Data Transfer",
     60160,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {UINT8_FIELD("SID"), BINARY_FIELD("Data", BYTES(7), NULL), END_OF_FIELDS},
     .interval    = UINT16_MAX,
     .explanation = "ISO 11783 defines this PGN as part of the Transport Protocol method used for transmitting messages that have "
                    "9 or more data bytes. This PGN represents a single packet of a multipacket message."}

    // ''ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting messages that have
    // 9 or more data bytes. This PGN's role in the transport process is determined by the group function value found in the first
    // data byte of the PGN.''
    ,
    {"ISO Transport Protocol, Connection Management - Request To Send",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {MATCH_LOOKUP_FIELD("Group Function Code", BYTES(1), 16, ISO_COMMAND),
      SIMPLE_DESC_FIELD("Message size", BYTES(2), "bytes"),
      SIMPLE_DESC_FIELD("Packets", BYTES(1), "packets"),
      SIMPLE_DESC_FIELD("Packets reply", BYTES(1), "packets sent in response to CTS"), // This one is still mysterious to me...
      PGN_FIELD("PGN", NULL),
      END_OF_FIELDS},
     .interval    = UINT16_MAX,
     .url         = "https://embeddedflakes.com/j1939-transport-protocol/",
     .explanation = "ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting "
                    "messages that have 9 or more data bytes. This PGN's role in the transport process is to prepare the receiver "
                    "for the fact that this sender wants to transmit a long message. The receiver will respond with CTS."}

    ,
    {"ISO Transport Protocol, Connection Management - Clear To Send",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {MATCH_LOOKUP_FIELD("Group Function Code", BYTES(1), 17, ISO_COMMAND),
      SIMPLE_DESC_FIELD("Max packets", BYTES(1), "Number of frames that can be sent before another CTS is required"),
      SIMPLE_DESC_FIELD("Next SID", BYTES(1), "Number of next frame to be transmitted"),
      RESERVED_FIELD(BYTES(2)),
      PGN_FIELD("PGN", NULL),
      END_OF_FIELDS},
     .interval    = UINT16_MAX,
     .url         = "https://embeddedflakes.com/j1939-transport-protocol/",
     .explanation = "ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting "
                    "messages that have 9 or more data bytes. This PGN's role in the transport process is to signal to the sender "
                    "that the receive is ready to receive a number of frames."}

    ,
    {"ISO Transport Protocol, Connection Management - End Of Message",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {MATCH_LOOKUP_FIELD("Group Function Code", BYTES(1), 19, ISO_COMMAND),
      SIMPLE_DESC_FIELD("Total message size", BYTES(2), "bytes"),
      SIMPLE_DESC_FIELD("Total number of frames received", BYTES(1), "Total number of of frames received"),
      RESERVED_FIELD(BYTES(1)),
      PGN_FIELD("PGN", NULL),
      END_OF_FIELDS},
     .interval = UINT16_MAX,
     .url      = "https://embeddedflakes.com/j1939-transport-protocol/",
     .explanation
     = "ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting messages that "
       "have 9 or more data bytes. This PGN's role in the transport process is to mark the end of the message."}

    ,
    {"ISO Transport Protocol, Connection Management - Broadcast Announce",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {MATCH_LOOKUP_FIELD("Group Function Code", BYTES(1), 32, ISO_COMMAND),
      SIMPLE_DESC_FIELD("Message size", BYTES(2), "bytes"),
      SIMPLE_DESC_FIELD("Packets", BYTES(1), "frames"),
      RESERVED_FIELD(BYTES(1)),
      PGN_FIELD("PGN", NULL),
      END_OF_FIELDS},
     .interval    = UINT16_MAX,
     .url         = "https://embeddedflakes.com/j1939-transport-protocol/",
     .explanation = "ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting "
                    "messages that have 9 or more data bytes. This PGN's role in the transport process is to announce a broadcast "
                    "of a long message spanning multiple frames."}

    ,
    {"ISO Transport Protocol, Connection Management - Abort",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {MATCH_LOOKUP_FIELD("Group Function Code", BYTES(1), 255, ISO_COMMAND),
      BINARY_FIELD("Reason", BYTES(1), NULL),
      RESERVED_FIELD(BYTES(3)),
      PGN_FIELD("PGN", NULL),
      END_OF_FIELDS},
     .interval    = UINT16_MAX,
     .url         = "https://embeddedflakes.com/j1939-transport-protocol/",
     .explanation = "ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting "
                    "messages that have 9 or more data bytes. This PGN's role in the transport process is to announce an abort "
                    "of a long message spanning multiple frames."}

    ,
    {"ISO Address Claim",
     60928,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {SIMPLE_DESC_FIELD("Unique Number", 21, "ISO Identity Number"),
      MANUFACTURER_FIELD(NULL, NULL, false),
      SIMPLE_DESC_FIELD("Device Instance Lower", 3, "ISO ECU Instance"),
      SIMPLE_DESC_FIELD("Device Instance Upper", 5, "ISO Function Instance"),
      LOOKUP_TRIPLET_FIELD("Device Function", BYTES(1), DEVICE_FUNCTION, "ISO Function", 7 /*Device Class*/),
      SPARE_FIELD(1),
      LOOKUP_FIELD("Device Class", 7, DEVICE_CLASS),
      SIMPLE_DESC_FIELD("System Instance", 4, "ISO Device Class Instance"),
      LOOKUP_FIELD("Industry Group", 3, INDUSTRY_CODE),
      // "Arbitrary address capable" is explained at
      // https://embeddedflakes.com/network-management-in-sae-j1939/#Arbitrary_Address_Capable
      SIMPLE_DESC_FIELD("Arbitrary address capable",
                        1,
                        "Field indicates whether the device is capable to claim arbitrary source "
                        "address. Value is 1 for NMEA200 devices. Could be 0 for J1939 device claims"),
      END_OF_FIELDS},
     .interval    = UINT16_MAX,
     .explanation = "This network management message is used to claim network address, reply to devices requesting the claimed "
                    "address, and to respond with device information (NAME) requested by the ISO Request or Complex Request Group "
                    "Function. This PGN contains several fields that are requestable, either independently or in any combination."}

    /* PDU1 (addressed) single-frame PGN range 0EF00 to 0xEFFF (61184 - 61439) */

    ,
    {"0xEF00: Manufacturer Proprietary single-frame addressed",
     61184,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {MANUFACTURER_FIELDS, BINARY_FIELD("Data", BYTES(6), NULL), END_OF_FIELDS},
     .fallback    = true,
     .explanation = "Manufacturer proprietary PGNs in PDU1 (addressed) single-frame PGN 0xEF00 (61184). "
                    "When this is shown during analysis it means the PGN is not reverse engineered yet."}

    /* None defined at this time */

    /* PDU2 non-addressed single-frame PGN range 0xF000 - 0xFEFF (61440 - 65279) */

    ,
    {"0xF000-0xFEFF: Standardized single-frame non-addressed",
     61440,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {MANUFACTURER_FIELDS, BINARY_FIELD("Data", BYTES(6), NULL), END_OF_FIELDS},
     .fallback    = true,
     .explanation = "PGNs in PDU2 (non-addressed) single-frame PGN range 0xF000 to "
                    "0xFEFF (61440 - 65279). "
                    "When this is shown during analysis it means the PGN is not reverse engineered yet."}

    /* J1939 ECU #2 PGN 61443 */

    ,
    {"ECU #2",
     61443,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {RESERVED_FIELD(BYTES(1)), PERCENTAGE_U8_HIGHRES_FIELD("Throttle Lever"), RESERVED_FIELD(BYTES(6)), END_OF_FIELDS}}

    /* J1939 ECU #1 PGN 61444 */

    ,
    {"ECU #1",
     61444,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {RESERVED_FIELD(BYTES(3)), ROTATION_UFIX16_RPM_HIGHRES_FIELD("Engine RPM", NULL), RESERVED_FIELD(BYTES(3)), END_OF_FIELDS}}

    /* Maretron ACM 100 manual documents PGN 65001-65030 */

    ,
    {"Bus #1 Phase C Basic AC Quantities",
     65001,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      RESERVED_FIELD(BYTES(2)),
      END_OF_FIELDS}}

    ,
    {"Bus #1 Phase B Basic AC Quantities",
     65002,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      RESERVED_FIELD(BYTES(2)),
      END_OF_FIELDS}}

    ,
    {"Bus #1 Phase A Basic AC Quantities",
     65003,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      RESERVED_FIELD(BYTES(2)),
      END_OF_FIELDS}}

    ,
    {"Bus #1 Average Basic AC Quantities",
     65004,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      RESERVED_FIELD(BYTES(2)),
      END_OF_FIELDS}}

    ,
    {"Utility Total AC Energy",
     65005,
     PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {ENERGY_UINT32_FIELD("Total Energy Export"), ENERGY_UINT32_FIELD("Total Energy Import"), END_OF_FIELDS}}

    ,
    {"Utility Phase C AC Reactive Power",
     65006,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_U16_VAR_FIELD("Reactive Power", NULL),
      POWER_FACTOR_U16_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      RESERVED_FIELD(BYTES(3) + 6),
      END_OF_FIELDS}}

    ,
    {"Utility Phase C AC Power",
     65007,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_OFFSET_FIELD("Real Power"), POWER_I32_VA_OFFSET_FIELD("Apparent Power"), END_OF_FIELDS}}

    ,
    {"Utility Phase C Basic AC Quantities",
     65008,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      CURRENT_UFIX16_A_FIELD("AC RMS Current"),
      END_OF_FIELDS}}

    ,
    {"Utility Phase B AC Reactive Power",
     65009,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_U16_VAR_FIELD("Reactive Power", NULL),
      POWER_FACTOR_U16_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      RESERVED_FIELD(BYTES(3) + 6),
      END_OF_FIELDS}}

    ,
    {"Utility Phase B AC Power",
     65010,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_OFFSET_FIELD("Real Power"), POWER_I32_VA_OFFSET_FIELD("Apparent Power"), END_OF_FIELDS}}

    ,
    {"Utility Phase B Basic AC Quantities",
     65011,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      CURRENT_UFIX16_A_FIELD("AC RMS Current"),
      END_OF_FIELDS}}

    ,
    {"Utility Phase A AC Reactive Power",
     65012,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_VAR_OFFSET_FIELD("Reactive Power"),
      POWER_FACTOR_U16_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      RESERVED_FIELD(BYTES(1) + 6),
      END_OF_FIELDS}}

    ,
    {"Utility Phase A AC Power",
     65013,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_OFFSET_FIELD("Real Power"), POWER_I32_VA_OFFSET_FIELD("Apparent Power"), END_OF_FIELDS}}

    ,
    {"Utility Phase A Basic AC Quantities",
     65014,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      CURRENT_UFIX16_A_FIELD("AC RMS Current"),
      END_OF_FIELDS}}

    ,
    {"Utility Total AC Reactive Power",
     65015,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_VAR_OFFSET_FIELD("Reactive Power"),
      POWER_FACTOR_U16_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      RESERVED_FIELD(BYTES(1) + 6),
      END_OF_FIELDS}}

    ,
    {"Utility Total AC Power",
     65016,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_OFFSET_FIELD("Real Power"), POWER_I32_VA_OFFSET_FIELD("Apparent Power"), END_OF_FIELDS}}

    ,
    {"Utility Average Basic AC Quantities",
     65017,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      CURRENT_UFIX16_A_FIELD("AC RMS Current"),
      END_OF_FIELDS}}

    ,
    {"Generator Total AC Energy",
     65018,
     PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {ENERGY_UINT32_FIELD("Total Energy Export"), ENERGY_UINT32_FIELD("Total Energy Import"), END_OF_FIELDS}}

    ,
    {"Generator Phase C AC Reactive Power",
     65019,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_VAR_OFFSET_FIELD("Reactive Power"),
      POWER_FACTOR_U16_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      RESERVED_FIELD(BYTES(1) + 6),
      END_OF_FIELDS}}

    ,
    {"Generator Phase C AC Power",
     65020,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_OFFSET_FIELD("Real Power"), POWER_I32_VAR_OFFSET_FIELD("Apparent Power"), END_OF_FIELDS}}

    ,
    {"Generator Phase C Basic AC Quantities",
     65021,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      CURRENT_UFIX16_A_FIELD("AC RMS Current"),
      END_OF_FIELDS}}

    ,
    {"Generator Phase B AC Reactive Power",
     65022,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_VAR_OFFSET_FIELD("Reactive Power"),
      POWER_FACTOR_U16_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      RESERVED_FIELD(BYTES(1) + 6),
      END_OF_FIELDS}}

    ,
    {"Generator Phase B AC Power",
     65023,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_OFFSET_FIELD("Real Power"), POWER_I32_VA_OFFSET_FIELD("Apparent Power"), END_OF_FIELDS}}

    ,
    {"Generator Phase B Basic AC Quantities",
     65024,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      CURRENT_UFIX16_A_FIELD("AC RMS Current"),
      END_OF_FIELDS}}

    ,
    {"Generator Phase A AC Reactive Power",
     65025,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_VAR_OFFSET_FIELD("Reactive Power"),
      POWER_FACTOR_U16_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      RESERVED_FIELD(BYTES(1) + 6),
      END_OF_FIELDS}}

    ,
    {"Generator Phase A AC Power",
     65026,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_OFFSET_FIELD("Real Power"), POWER_I32_VA_OFFSET_FIELD("Apparent Power"), END_OF_FIELDS}}

    ,
    {"Generator Phase A Basic AC Quantities",
     65027,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      CURRENT_UFIX16_A_FIELD("AC RMS Current"),
      END_OF_FIELDS}}

    ,
    {"Generator Total AC Reactive Power",
     65028,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_VAR_OFFSET_FIELD("Reactive Power"),
      POWER_FACTOR_U16_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      RESERVED_FIELD(BYTES(1) + 6),
      END_OF_FIELDS}}

    ,
    {"Generator Total AC Power",
     65029,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {POWER_I32_OFFSET_FIELD("Real Power"), POWER_I32_VA_OFFSET_FIELD("Apparent Power"), END_OF_FIELDS}}

    ,
    {"Generator Average Basic AC Quantities",
     65030,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {VOLTAGE_U16_V_FIELD("Line-Line AC RMS Voltage"),
      VOLTAGE_U16_V_FIELD("Line-Neutral AC RMS Voltage"),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      CURRENT_UFIX16_A_FIELD("AC RMS Current"),
      END_OF_FIELDS}}

    ,
    {"Active Trouble Codes", /* J1939 PGN 65226 See https://embeddedflakes.com/j1939-diagnostics-part-1/ */
     65226,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {BINARY_FIELD("Malfunction Lamp Status", 2, "Fault Lamps"), /* Lamp modes are: 0 = off, 01 = on		*/
      BINARY_FIELD("Red Stop Lamp Status", 2, "Fault Lamps"),    /* 10 = flashing 1Hz, 11 = flashing 2Hz	*/
      BINARY_FIELD("Amber Warning Lamp Status", 2, "Fault Lamps"),
      BINARY_FIELD("Protect Lamp Status", 2, "Fault Lamps"),
      RESERVED_FIELD(BYTES(1)),
      BINARY_FIELD("SPN", 19, "Suspect Parameter Number"), /* These four fields comprise a Diagnostic Trouble Code (DTC) */
      BINARY_FIELD("FMI", 5, "Fault Mode Indicator"),      /* If there is more han one DTC the message is sent using TP  */
      BINARY_FIELD("CM", 1, "SPN Conversion Method"),      /* Not sure how to handle that actually... */
      BINARY_FIELD("OC", 7, "Occurance Count"),
      END_OF_FIELDS}}

    ,
    {"ISO Commanded Address",
     65240,
     PACKET_COMPLETE,
     PACKET_ISO_TP,
     /* ISO 11783 defined this message to provide a mechanism for assigning a network address to a node. The NAME information in
     the data portion of the message must match the name information of the node whose network address is to be set. */
     {BINARY_FIELD("Unique Number", 21, "ISO Identity Number"),
      MANUFACTURER_FIELD("Manufacturer Code", NULL, false),
      SIMPLE_DESC_FIELD("Device Instance Lower", 3, "ISO ECU Instance"),
      SIMPLE_DESC_FIELD("Device Instance Upper", 5, "ISO Function Instance"),
      LOOKUP_TRIPLET_FIELD("Device Function", BYTES(1), DEVICE_FUNCTION, "ISO Function", 7 /*Device Class*/),
      RESERVED_FIELD(1),
      LOOKUP_FIELD("Device Class", 7, DEVICE_CLASS),
      SIMPLE_DESC_FIELD("System Instance", 4, "ISO Device Class Instance"),
      LOOKUP_FIELD("Industry Code", 3, INDUSTRY_CODE),
      RESERVED_FIELD(1),
      UINT8_FIELD("New Source Address"),
      END_OF_FIELDS}}

    ,
    {"Engine Temp #1",
     65262,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {TEMPERATURE_UINT8_OFFSET_FIELD("Engine Coolant Temp"), END_OF_FIELDS}}

    ,
    {"Fuel Economy",
     65266,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {RESERVED_FIELD(BYTES(6)), PERCENTAGE_U8_HIGHRES_FIELD("Throttle Position"), END_OF_FIELDS}}

    ,
    {"Ambient Conditions",
     65269,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {PRESSURE_UINT8_KPA_FIELD("Barometric Pressure"), END_OF_FIELDS}}

    ,
    {"Inlet/Exhaust Conditions",
     65270,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {RESERVED_FIELD(BYTES(2)),
      TEMPERATURE_UINT8_OFFSET_FIELD("Intake Manifold Temp"),
      PRESSURE_UINT8_2KPA_FIELD("Air Inlet Pressure"),
      END_OF_FIELDS}}

    ,
    {"Vehicle Electrical Power",
     65271,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {RESERVED_FIELD(BYTES(4)), VOLTAGE_U16_50MV_FIELD("Battery Voltage"), END_OF_FIELDS}}

    /* proprietary PDU2 (non addressed) single-frame range 0xFF00 to 0xFFFF (65280 - 65535) */

    ,
    {"0xFF00-0xFFFF: Manufacturer Proprietary single-frame non-addressed",
     65280,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {MANUFACTURER_FIELDS, BINARY_FIELD("Data", BYTES(6), NULL), END_OF_FIELDS},
     .fallback    = true,
     .explanation = "Manufacturer proprietary PGNs in PDU2 (non-addressed) single-frame PGN range 0xFF00 to "
                    "0xFFFF (65280 - 65535). "
                    "When this is shown during analysis it means the PGN is not reverse engineered yet."}

    /* PDU1 (addressed) fast-packet PGN range 0x1ED00 to 0x1EE00 (126208 - 126464) */
    /* Only 0x1ED00 and 0x1EE00 seem to be used? */

    ,
    {"0x1ED00 - 0x1EE00: Standardized fast-packet addressed",
     0x1ed00,
     PACKET_INCOMPLETE_LOOKUP,
     PACKET_FAST,
     {BINARY_FIELD("Data", BYTES(FASTPACKET_MAX_SIZE), NULL), END_OF_FIELDS},
     .fallback    = true,
     .explanation = "Standardized PGNs in PDU1 (addressed) fast-packet PGN range 0x1ED00 to "
                    "0x1EE00 (65536 - 126464). "
                    "When this is shown during analysis it means the PGN is not reverse engineered yet."}

    /* proprietary PDU1 (addressed) fast-packet PGN range 0x1EF00 to 0x1EFFF (126720 - 126975) */

    ,
    {"0x1EF00-0x1EFFF: Manufacturer Proprietary fast-packet addressed",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     {MANUFACTURER_FIELDS, BINARY_FIELD("Data", BYTES(221), NULL), END_OF_FIELDS},
     .fallback    = true,
     .explanation = "Manufacturer Proprietary PGNs in PDU1 (addressed) fast-packet PGN range 0x1EF00 to "
                    "0x1EFFF (126720 - 126975). "
                    "When this is shown during analysis it means the PGN is not reverse engineered yet."}

    /* PDU2 (non addressed) mixed single/fast packet PGN range 0x1F000 to 0x1FEFF (126976 - 130815) */

    ,
    {"0x1F000-0x1FEFF: Standardized mixed single/fast packet non-addressed",
     126976,
     PACKET_INCOMPLETE,
     PACKET_MIXED,
     {BINARY_FIELD("Data", BYTES(FASTPACKET_MAX_SIZE), NULL), END_OF_FIELDS},
     .fallback    = true,
     .explanation = "Standardized PGNs in PDU2 (non-addressed) mixed single/fast packet PGN range 0x1F000 to "
                    "0x1FEFF (126976 - 130815). "
                    "When this is shown during analysis it means the PGN is not reverse engineered yet."}

    /* proprietary PDU2 (non addressed) fast packet PGN range 0x1FF00 to 0x1FFFF (130816 - 131071) */

    ,
    {"0x1FF00-0x1FFFF: Manufacturer Specific fast-packet non-addressed",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     {BINARY_FIELD("Data", BYTES(FASTPACKET_MAX_SIZE), NULL), END_OF_FIELDS},
     .fallback = true,
     .explanation
     = "This definition is used for Manufacturer Specific PGNs in PDU2 (non-addressed) fast-packet PGN range 0x1FF00 to "
       "0x1FFFF (130816 - 131071). "
       "When this is shown during analysis it means the PGN is not reverse engineered yet."}

};

const size_t pgnListSize  = ARRAY_SIZE(pgnList);
const size_t pgnRangeSize = ARRAY_SIZE(pgnRange);

#else
extern Pgn      pgnList[];
extern size_t   pgnListSize;
extern PgnRange pgnRange[];
extern size_t   pgnRangeSize;
#endif
