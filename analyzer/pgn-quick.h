/*

Quick CAN Protocol PGN definitions.

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

#ifndef PGN_QUICK_H_INCLUDED
#define PGN_QUICK_H_INCLUDED

#include <float.h>

#include "common.h"
#include "parse.h"
#include "pow.h"

#define LEN_VARIABLE (0)

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
  uint8_t val1Order;
  size_t  size;
};

#define LOOKUP_PAIR_MEMBER .lookup.function.pair
#define LOOKUP_BIT_MEMBER .lookup.function.pair
#define LOOKUP_TRIPLET_MEMBER .lookup.function.triplet
#define LOOKUP_FIELDTYPE_MEMBER .lookup.function.pair

typedef struct
{
  const char *name;
  const char *fieldType;
  uint32_t    size;
  const char *unit;
  const char *description;

  const char *encoding;  /* Character set to read this field's bytes as when they are not well-formed UTF-8.
                          *    NULL means Latin-1. Mirrors the member in pgn.h; no Quick field sets it today,
                          *    but print.c is shared between the builds. See keel/src/charset.rs. */

  bool    hasMatchValue;
  int64_t matchValue;
  int32_t offset;
  double  resolution;
  int     precision;
  double  unitOffset;
  bool    proprietary;
  bool    hasSign;
  bool    partOfPrimaryKey;
  int8_t  reservedOverride;
  bool    dynamicFieldLength;
  uint8_t dynamicFieldLengthOverhead;

  uint8_t    order;
  uint8_t    reservedCount;
  size_t     bitOffset;
  char      *camelName;
  LookupInfo lookup;
  FieldType *ft;
  Pgn       *pgn;
  double     rangeMin;
  double     rangeMax;
} Field;

#include "fieldtype.h"

#define LOOKUP_TYPE(type, length) extern const char *lookup##type(size_t val);
#define LOOKUP_TYPE_TRIPLET(type, length) extern const char *lookup##type(size_t val1, size_t val2);
#define LOOKUP_TYPE_BITFIELD(type, length) extern const char *lookup##type(size_t val);
#define LOOKUP_TYPE_FIELDTYPE(type, length) extern const char *lookup##type(size_t val);

/* The Quick tree's own lookups, as its own generated table. */
#include "lookup-quick-generated-data.h"

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
  uint16_t   complete;
  PacketType type;
  Field      fieldList[33];
  uint32_t   fieldCount;
  char       *camelDescription;
  bool        fallback;
  bool        hasMatchFields;
  const char *explanation;
  const char *url;
  const char *researchDoc;
  uint16_t    interval;
  uint8_t     priority;
  uint8_t     repeatingCount1;
  uint8_t     repeatingCount2;
  uint8_t     repeatingStart1;
  uint8_t     repeatingStart2;
  uint8_t     repeatingField1;
  uint8_t     repeatingField2;
};

typedef struct PgnRange
{
  uint32_t    pgnStart;
  uint32_t    pgnEnd;
  uint32_t    pgnStep;
  const char *who;
  PacketType  type;
} PgnRange;

const Pgn *searchForPgn(int pgn);
const Pgn *searchForUnknownPgn(int pgnId);
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

/* lookup.c */
extern void fillLookups(void);

/* Quick CAN IDs don't match any NMEA 2000 / ISO 11783 range.
 * There are no proprietary PGNs in the Quick space.
 */
#define IS_MANUFACTURER_PGN(x) (0)

#ifdef GLOBALS
/* Quick CAN IDs: 0x000 - 0x7FF, single frame, direct mapping */
PgnRange pgnRange[] = {{0, 0x7ff, 1, "Quick", PACKET_SINGLE}};

#include "pgn-quick-generated-data.h"

const size_t pgnListSize  = ARRAY_SIZE(pgnList);
const size_t pgnRangeSize = ARRAY_SIZE(pgnRange);

#else
extern Pgn      pgnList[];
extern size_t   pgnListSize;
extern PgnRange pgnRange[];
extern size_t   pgnRangeSize;
#endif

#endif /* PGN_QUICK_H_INCLUDED */
