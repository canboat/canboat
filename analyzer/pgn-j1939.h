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

#define LOOKUP_PAIR_MEMBER .lookup.function.pair
#define LOOKUP_BIT_MEMBER .lookup.function.pair
#define LOOKUP_TRIPLET_MEMBER .lookup.function.triplet
#define LOOKUP_FIELDTYPE_MEMBER .lookup.function.pair

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

/* The field-definition macro DSL that used to live here died at the
 * keel switchover: pgn-data.h is generated with plain designated
 * initializers from database/ (see keel/DESIGN.md).
 */

#define LOOKUP_TYPE(type, length) extern const char *lookup##type(size_t val);
#define LOOKUP_TYPE_TRIPLET(type, length) extern const char *lookup##type(size_t val1, size_t val2);
#define LOOKUP_TYPE_BITFIELD(type, length) extern const char *lookup##type(size_t val);
#define LOOKUP_TYPE_FIELDTYPE(type, length) extern const char *lookup##type(size_t val);

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
