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

#include "common.h"
#include "parse.h"

#define PROPRIETARY_PGN_ONLY "=proprietary pgn only"

#define LEN_VARIABLE (0)

#define RES_LAT_LONG_PRECISION (10000000) /* 1e7 */
#define RES_LAT_LONG (1.0e-7)
#define RES_LAT_LONG_64 (1.0e-16)
#define RES_PERCENTAGE (100.0 / 25000.0)

#define RES_RADIANS (1e-4)
#define RES_ROTATION (1e-3 / 32.0)
#define RES_HIRES_ROTATION (1e-6 / 32.0)
#define RES_ASCII (-1.0)
#define RES_LATITUDE (-2.0)
#define RES_LONGITUDE (-3.0)
#define RES_DATE (-4.0)
#define RES_TIME (-5.0)
#define RES_TEMPERATURE (-6.0)
#define RES_6BITASCII (-7.0) /* Actually not used in N2K, only in N183 AIS */
#define RES_INTEGER (-8.0)
#define RES_LOOKUP (-9.0)
#define RES_BINARY (-10.0)
#define RES_MANUFACTURER (-11.0)
#define RES_STRING (-12.0)
#define RES_FLOAT (-13.0)
#define RES_PRESSURE (-14.0)
#define RES_STRINGLZ (-15.0)  /* ASCII string starting with length byte and terminated by zero byte */
#define RES_STRINGLAU (-16.0) /* ASCII or UNICODE string starting with length byte and ASCII/Unicode byte */
#define RES_DECIMAL (-17.0)
#define RES_BITFIELD (-18.0)
#define RES_TEMPERATURE_HIGH (-19.0)
#define RES_TEMPERATURE_HIRES (-20.0)
#define RES_PRESSURE_HIRES (-21.0)
#define RES_VARIABLE (-22.0)
#define MAX_RESOLUTION_LOOKUP 22

typedef struct
{
  char    *name;
  uint32_t size;       /* Size in bits. All fields are contiguous in message; use 'reserved' fields to fill in empty bits. */
  double   resolution; /* Either a positive real value or one of the following RES_ special values */

  bool        hasSign; /* Is the value signed, e.g. has both positive and negative values? */
  const char *units;   /* String containing the 'Dimension' (e.g. s, h, m/s, etc.) unless it starts with , in which
                        * case it contains a set of lookup values.
                        */
  const char *description;
  int32_t     offset; /* Only used for SAE J1939 values with sign; these are in Offset/Excess-K notation instead
                       * of two's complement as used by NMEA 2000.
                       * See http://en.wikipedia.org/wiki/Offset_binary
                       */

  /* The following fields are filled by C, no need to set in initializers */
  char        *camelName;
  const char **lookupValue;
  const char  *lookupName;
} Field;

#define LOOKUP_FIELD(nam, len, typ)                                                                              \
  {                                                                                                              \
    .name = nam, .size = len, .resolution = RES_LOOKUP, .lookupValue = lookupValue##typ, .lookupName = xstr(typ) \
  }

#define LOOKUP_FIELD_DESC(nam, len, typ, desc)                                                                    \
  {                                                                                                               \
    .name = nam, .size = len, .resolution = RES_LOOKUP, .lookupValue = lookupValue##typ, .lookupName = xstr(typ), \
    .description = desc                                                                                           \
  }

#define LOOKUP_BITFIELD(nam, len, typ)                                                                             \
  {                                                                                                                \
    .name = nam, .size = len, .resolution = RES_BITFIELD, .lookupValue = lookupValue##typ, .lookupName = xstr(typ) \
  }

#define RESERVED_FIELD(len)                                                        \
  {                                                                                \
    .name = "Reserved", .size = (len), .resolution = RES_BINARY, .description = "" \
  }

#define BINARY_FIELD(nam, len, desc)                                          \
  {                                                                           \
    .name = nam, .size = (len), .resolution = RES_BINARY, .description = desc \
  }

#define ANGLE_POS_FIELD(nam, desc)                                                                                  \
  {                                                                                                                 \
    .name = nam, .size = BYTES(2), .resolution = RES_RADIANS, .hasSign = false, .units = "rad", .description = desc \
  }

#define ANGLE_REL_FIELD(nam, desc)                                                                                 \
  {                                                                                                                \
    .name = nam, .size = BYTES(2), .resolution = RES_RADIANS, .hasSign = true, .units = "rad", .description = desc \
  }

#define VOLTAGE_FIELD(nam, res)                                                       \
  {                                                                                   \
    .name = nam, .size = BYTES(2), .resolution = res, .units = "V", .description = "" \
  }

#define VOLTAGE_MEDIUM_FIELD(nam, res)                                                \
  {                                                                                   \
    .name = nam, .size = BYTES(2), .resolution = res, .units = "V", .description = "" \
  }

#define RADIO_FREQUENCY_FIELD(nam, res)                                                \
  {                                                                                    \
    .name = nam, .size = BYTES(4), .resolution = res, .units = "Hz", .description = "" \
  }

#define FREQUENCY_FIELD(nam, res)                                                      \
  {                                                                                    \
    .name = nam, .size = BYTES(2), .resolution = res, .units = "Hz", .description = "" \
  }

#define MILLIMETER_SPEED_FIELD(nam)                                                                        \
  {                                                                                                        \
    .name = nam, .size = BYTES(2), .resolution = 0.001, .hasSign = true, .units = "m/s", .description = "" \
  }

#define SPEED_FIELD(nam)                                                                                   \
  {                                                                                                        \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .hasSign = false, .units = "m/s", .description = "" \
  }

#define SPEED_REL_FIELD(nam)                                                                              \
  {                                                                                                       \
    .name = nam, .size = BYTES(2), .resolution = 0.01, .hasSign = true, .units = "m/s", .description = "" \
  }

#define MATCH_FIELD(nam, len, id, desc)                                                   \
  {                                                                                       \
    .name = nam, .size = len, .resolution = 1, .units = "=" xstr(id), .description = desc \
  }

#define COMPANY(id)                                                                                               \
  {                                                                                                               \
      .name       = "Manufacturer Code",                                                                          \
      .size       = 11,                                                                                           \
      .resolution = RES_MANUFACTURER,                                                                             \
      .units      = "=" xstr(id),                                                                                 \
  },                                                                                                              \
      RESERVED_FIELD(2),                                                                                          \
  {                                                                                                               \
    .name = "Industry Code", .size = 3, .resolution = RES_LOOKUP, .units = "=4", .description = "Marine Industry" \
  }

#define MANUFACTURER_FIELDS                                                                                        \
  {.name = "Manufacturer Code", .size = 11, .resolution = RES_MANUFACTURER, .description = ""}, RESERVED_FIELD(2), \
      LOOKUP_FIELD("Industry Code", 3, INDUSTRY_CODE)

#define MANUFACTURER_PROPRIETARY_FIELDS                                                                                          \
  {.name        = "Manufacturer Code",                                                                                           \
   .size        = 11,                                                                                                            \
   .resolution  = RES_MANUFACTURER,                                                                                              \
   .units       = PROPRIETARY_PGN_ONLY,                                                                                          \
   .description = "Only in PGN when PRN is proprietary",                                                                         \
   .lookupValue = lookupValueMANUFACTURER_CODE,                                                                                  \
   .lookupName  = "MANUFACTURER_CODE"},                                                                                           \
      {.name        = "Reserved",                                                                                                \
       .size        = 2,                                                                                                         \
       .resolution  = RES_BINARY,                                                                                                \
       .units       = PROPRIETARY_PGN_ONLY,                                                                                      \
       .description = "Only in PGN when PRN is proprietary"},                                                                    \
  {                                                                                                                              \
    .name = "Industry code", .size = 3, .resolution = RES_LOOKUP, .units = PROPRIETARY_PGN_ONLY,                                 \
    .description = "Only in PGN when PRN is proprietary", .lookupValue = lookupValueINDUSTRY_CODE, .lookupName = "INDUSTRY_CODE" \
  }

#define INSTANCE_FIELD                                                       \
  {                                                                          \
    .name = "Instance", .size = BYTES(1), .resolution = 1, .description = "" \
  }

#define INTEGER_DESC_FIELD(nam, len, desc)                                   \
  {                                                                          \
    .name = nam, .size = len, .resolution = RES_INTEGER, .description = desc \
  }

#define INTEGER_FIELD(nam, len) INTEGER_DESC_FIELD(nam, len, "")

#define SIMPLE_DESC_FIELD(nam, len, desc)                          \
  {                                                                \
    .name = nam, .size = len, .resolution = 1, .description = desc \
  }

#define SIMPLE_FIELD(nam, len) SIMPLE_DESC_FIELD(nam, len, "")

#define ONE_BYTE_FIELD(nam) SIMPLE_FIELD(nam, BYTES(1))

#define STRINGLZ_FIELD(nam, len)                                            \
  {                                                                         \
    .name = nam, .size = len, .resolution = RES_STRINGLZ, .description = "" \
  }

#define ASCII_DESC_FIELD(nam, len, desc)                                   \
  {                                                                        \
    .name = nam, .size = len, .resolution = RES_ASCII, .description = desc \
  }

#define STRINGVAR_FIELD(nam)                                                       \
  {                                                                                \
    .name = nam, .size = LEN_VARIABLE, .resolution = RES_STRING, .description = "" \
  }

#define STRINGLAU_FIELD(nam)                                                          \
  {                                                                                   \
    .name = nam, .size = LEN_VARIABLE, .resolution = RES_STRINGLAU, .description = "" \
  }

#define ASCII_FIELD(nam, len) ASCII_DESC_FIELD(nam, len, "")

#define TEMPERATURE_HIGH_FIELD(nam)                                                                    \
  {                                                                                                    \
    .name = nam, .size = BYTES(2), .resolution = RES_TEMPERATURE_HIGH, .units = "K", .description = "" \
  }

#define TEMPERATURE_FIELD(nam)                                                                    \
  {                                                                                               \
    .name = nam, .size = BYTES(2), .resolution = RES_TEMPERATURE, .units = "K", .description = "" \
  }

#define VARIABLE_FIELD(nam, desc)                                                      \
  {                                                                                    \
    .name = nam, .size = LEN_VARIABLE, .resolution = RES_VARIABLE, .description = desc \
  }

#define DISTANCE_FIELD(nam, len, res, desc)                                                         \
  {                                                                                                 \
    .name = nam, .size = len, .resolution = res, .hasSign = true, .units = "m", .description = desc \
  }

#define LENGTH_FIELD(nam, len, res, desc)                                          \
  {                                                                                \
    .name = nam, .size = len, .resolution = res, .units = "m", .description = desc \
  }

#define DECIMETERS_FIELD(nam) LENGTH_FIELD(nam, BYTES(2), 0.1, "")

#define HIRES_LENGTH_FIELD(nam, res) LENGTH_FIELD(nam, BYTES(4), res, "")

#define SHORT_TIME_FIELD(nam)                                                              \
  {                                                                                        \
    .name = nam, .size = BYTES(2), .resolution = RES_TIME, .units = "s", .description = "" \
  }

#define TIME_FIELD(nam, desc)                                                                \
  {                                                                                          \
    .name = nam, .size = BYTES(4), .resolution = RES_TIME, .units = "s", .description = desc \
  }

#define ELAPSED_FIELD(nam, len, res)                                             \
  {                                                                              \
    .name = nam, .size = len, .resolution = res, .units = "s", .description = "" \
  }

#define ENERGY_FIELD(nam)                                                             \
  {                                                                                   \
    .name = nam, .size = BYTES(4), .resolution = 1, .units = "kWh", .description = "" \
  }

#define AC_OFFSET_POWER_FIELD(nam, unit)                                                                                     \
  {                                                                                                                          \
    .name = nam, .size = BYTES(4), .resolution = 1, .hasSign = true, .units = unit, .offset = -2000000000, .description = "" \
  }

#define ISO_POWER_FACTOR_FIELD                                                                               \
  {                                                                                                          \
    .name = "Power Factor", .size = BYTES(2), .resolution = 1 / 16384, .units = "Cos Phi", .description = "" \
  }

#define POWER_FACTOR_FIELD                                                                              \
  {                                                                                                     \
    .name = "Power Factor", .size = BYTES(1), .resolution = 0.01, .units = "Cos Phi", .description = "" \
  }

#define SHORT_PERCENTAGE_FIELD(nam)                                                           \
  {                                                                                           \
    .name = nam, .size = BYTES(1), .resolution = RES_INTEGER, .units = "%", .description = "" \
  }

#define PERCENTAGE_FIELD(nam)                                                                    \
  {                                                                                              \
    .name = nam, .size = BYTES(2), .resolution = RES_PERCENTAGE, .units = "%", .description = "" \
  }

typedef struct
{
  const char *name;
  const char *resolution;
} Resolution;

static const Resolution types[MAX_RESOLUTION_LOOKUP] = {{"ASCII text", 0},
                                                        {"Latitude", 0},
                                                        {"Longitude", 0},
                                                        {"Date", "1"},
                                                        {"Time", "0.0001"},
                                                        {"Temperature", "0.01"},
                                                        {"6 Bit ASCII text", 0},
                                                        {"Integer", "1"},
                                                        {"Lookup table", 0},
                                                        {"Binary data", 0},
                                                        {"Manufacturer code", 0},
                                                        {"String with start/stop byte", 0},
                                                        {"IEEE Float", 0},
                                                        {"Pressure", 0},
                                                        {"ASCII string starting with length byte", 0},
                                                        {"ASCII or UNICODE string starting with length and control byte", 0},
                                                        {"Decimal encoded number", 0},
                                                        {"Bitfield", 0},
                                                        {"Temperature", "0.1"},
                                                        {"Temperature (hires)", "0.001"},
                                                        {"Pressure (hires)", "0.1"}};

#define LOOKUP_TYPE(type, length)                      \
  extern const char *lookupValue##type[1 << (length)]; \
  extern uint32_t    lookupLength##type;
#define LOOKUP_TYPE_BITFIELD(type, length)      \
  extern const char *lookupValue##type[length]; \
  extern uint32_t    lookupLength##type;

#include "lookup.h"

typedef enum PacketComplete
{
  PACKET_COMPLETE              = 0,
  PACKET_FIELDS_UNKNOWN        = 1,
  PACKET_FIELD_LENGTHS_UNKNOWN = 2,
  PACKET_PRECISION_UNKNOWN     = 4,
  PACKET_LOOKUPS_UNKNOWN       = 8,
  PACKET_NOT_SEEN              = 16
} PacketComplete;

#define PACKET_INCOMPLETE (PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_PRECISION_UNKNOWN)
#define PACKET_INCOMPLETE_LOOKUP (PACKET_INCOMPLETE | PACKET_LOOKUPS_UNKNOWN)

typedef enum PacketType
{
  PACKET_SINGLE,
  PACKET_FAST,
  PACKET_ISO11783
} PacketType;

typedef struct
{
  char      *description;
  uint32_t   pgn;
  uint16_t   complete;        /* Either PACKET_COMPLETE or bit values set for various unknown items */
  PacketType type;            /* Single, Fast or ISO11783 */
  uint32_t   size;            /* (Minimal) size of this PGN. Helps to determine initial malloc */
  uint32_t   repeatingFields; /* How many fields at the end repeat until the PGN is exhausted? */
  Field      fieldList[30]; /* Note fixed # of fields; increase if needed. RepeatingFields support means this is enough for now. */
  uint32_t   fieldCount;    /* Filled by C, no need to set in initializers. */
  char      *camelDescription; /* Filled by C, no need to set in initializers. */
  bool       unknownPgn;       /* true = this is a catch-all for unknown PGNs */
} Pgn;

// Returns the first pgn that matches the given id, or 0 if not found.
Pgn *searchForPgn(int pgn);

// Returns a pointer (potentially invalid) to the first pgn that does not match "first".
Pgn *endPgn(Pgn *first);

Pgn *getMatchingPgn(int pgnId, uint8_t *dataStart, int length);

bool printPgn(RawMessage *msg, uint8_t *dataStart, int length, bool showData, bool showJson);
void checkPgnList(void);

Field *getField(uint32_t pgn, uint32_t field);
void   extractNumber(const Field *field, uint8_t *data, size_t startBit, size_t bits, int64_t *value, int64_t *maxValue);

void camelCase(bool upperCamelCase);

/* lookup.c */
extern void fillLookups(void);

#ifdef GLOBALS
Pgn pgnList[] = {

    /* PDU1 (addressed) single-frame PGN range 0E800 to 0xEEFF (59392 - 61183) */

    {"Unknown single-frame addressed",
     0,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {BINARY_FIELD("Data", BYTES(8), ""), {0}},
     0,
     0,
     true}

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
     8,
     0,
     {LOOKUP_FIELD("Control", BYTES(1), ISO_CONTROL),
      ONE_BYTE_FIELD("Group Function"),
      RESERVED_FIELD(24),
      INTEGER_DESC_FIELD("PGN", 24, "Parameter Group Number of requested information"),
      {0}}}

    ,
    {"ISO Request", 59904, PACKET_COMPLETE, PACKET_SINGLE, 3, 0, {INTEGER_FIELD("PGN", 24), {0}}}

    /* For a good explanation of ISO 11783 transport protocol (as used in J1939) see
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

    // ISO 11783 defines this PGN as part of the transport protocol method used for transmitting messages that have 9 or more data
    // bytes. This PGN represents a single packet of a multipacket message.
    ,
    {"ISO Transport Protocol, Data Transfer",
     60160,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {ONE_BYTE_FIELD("SID"), SIMPLE_FIELD("Data", BYTES(7)), {0}}}

    // ''ISO 11783 defines this group function PGN as part of the transport protocol method used for transmitting messages that have
    // 9 or more data bytes. This PGN's role in the transport process is determined by the group function value found in the first
    // data byte of the PGN.''
    ,
    {"ISO Transport Protocol, Connection Management - Request To Send",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {MATCH_FIELD("Group Function Code", BYTES(1), 16, "RTS"),
      SIMPLE_DESC_FIELD("Message size", BYTES(2), "bytes"),
      SIMPLE_DESC_FIELD("Packets", BYTES(1), "packets"),
      SIMPLE_DESC_FIELD("Packets reply", BYTES(1), "packets sent in response to CTS"), // This one is still mysterious to me...
      INTEGER_DESC_FIELD("PGN", BYTES(3), "PGN"),
      {0}}}

    ,
    {"ISO Transport Protocol, Connection Management - Clear To Send",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {MATCH_FIELD("Group Function Code", BYTES(1), 17, "CTS"),
      SIMPLE_DESC_FIELD("Max packets", BYTES(1), "packets before waiting for next CTS"),
      SIMPLE_DESC_FIELD("Next SID", BYTES(1), "packet"),
      RESERVED_FIELD(BYTES(2)),
      INTEGER_DESC_FIELD("PGN", BYTES(3), "PGN"),
      {0}}}

    ,
    {"ISO Transport Protocol, Connection Management - End Of Message",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {MATCH_FIELD("Group Function Code", BYTES(1), 19, "EOM"),
      SIMPLE_DESC_FIELD("Total message size", BYTES(2), "bytes"),
      SIMPLE_DESC_FIELD("Total number of packets received", BYTES(1), "packets"),
      RESERVED_FIELD(BYTES(1)),
      INTEGER_DESC_FIELD("PGN", BYTES(3), "PGN"),
      {0}}}

    ,
    {"ISO Transport Protocol, Connection Management - Broadcast Announce",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {MATCH_FIELD("Group Function Code", BYTES(1), 32, "BAM"),
      SIMPLE_DESC_FIELD("Message size", BYTES(2), "bytes"),
      SIMPLE_DESC_FIELD("Packets", BYTES(1), "frames"),
      RESERVED_FIELD(BYTES(1)),
      INTEGER_DESC_FIELD("PGN", BYTES(3), "PGN"),
      {0}}},
    {"ISO Transport Protocol, Connection Management - Abort",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {MATCH_FIELD("Group Function Code", BYTES(1), 255, "Abort"),
      BINARY_FIELD("Reason", BYTES(1), ""),
      RESERVED_FIELD(BYTES(2)),
      INTEGER_DESC_FIELD("PGN", BYTES(3), "PGN"),
      {0}}}

    ,
    {"ISO Address Claim",
     60928,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {BINARY_FIELD("Unique Number", 21, "ISO Identity Number"),
      {"Manufacturer Code", 11, RES_MANUFACTURER, false, 0, ""},
      SIMPLE_DESC_FIELD("Device Instance Lower", 3, "ISO ECU Instance"),
      SIMPLE_DESC_FIELD("Device Instance Upper", 5, "ISO Function Instance"),
      SIMPLE_DESC_FIELD("Device Function", 8, "ISO Function"),
      RESERVED_FIELD(1),
      LOOKUP_FIELD("Device Class", 7, DEVICE_CLASS),
      SIMPLE_DESC_FIELD("System Instance", 4, "ISO Device Class Instance"),
      LOOKUP_FIELD("Industry Group", 3, INDUSTRY_CODE),
      RESERVED_FIELD(1),
      {0}}}

    /* PDU1 (addressed) single-frame PGN range 0EF00 to 0xEFFF (61184 - 61439) */

    /* The following probably have the wrong Proprietary ID */
    ,
    {"Seatalk: Wireless Keypad Light Control",
     61184,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1851),
      MATCH_FIELD("Proprietary ID", BYTES(1), 1, "Wireless Keypad Light Control"),
      ONE_BYTE_FIELD("Variant"),
      ONE_BYTE_FIELD("Wireless Setting"),
      ONE_BYTE_FIELD("Wired Setting"),
      {0}}}

    ,
    {"Seatalk: Wireless Keypad Light Control",
     61184,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1851), ONE_BYTE_FIELD("PID"), ONE_BYTE_FIELD("Variant"), ONE_BYTE_FIELD("Beep Control"), {0}}}

    ,
    {"Victron Battery Register",
     61184,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(358), SIMPLE_FIELD("Register Id", BYTES(2)), SIMPLE_FIELD("Payload", BYTES(4)), {0}}}

    ,
    {"Manufacturer Proprietary single-frame addressed",
     61184,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {MANUFACTURER_FIELDS, BINARY_FIELD("Data", BYTES(6), ""), {0}},
     0,
     0,
     true}

    /* PDU2 non-addressed single-frame PGN range 0xF000 - 0xFEFF (61440 - 65279) */

    ,
    {"Unknown single-frame non-addressed",
     61440,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {MANUFACTURER_FIELDS, BINARY_FIELD("Data", BYTES(6), ""), {0}},
     0,
     0,
     true}

    /* Maretron ACM 100 manual documents PGN 65001-65030 */

    ,
    {"Bus #1 Phase C Basic AC Quantities",
     65001,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {0}}}

    ,
    {"Bus #1 Phase B Basic AC Quantities",
     65002,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {0}}}

    ,
    {"Bus #1 Phase A Basic AC Quantities",
     65003,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {0}}}

    ,
    {"Bus #1 Average Basic AC Quantities",
     65004,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {0}}}

    ,
    {"Utility Total AC Energy",
     65005,
     PACKET_PRECISION_UNKNOWN,
     PACKET_SINGLE,
     8,
     0,
     {ENERGY_FIELD("Total Energy Export"), ENERGY_FIELD("Total Energy Import"), {0}}}

    ,
    {"Utility Phase C AC Reactive Power",
     65006,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Reactive Power", BYTES(2), 1, false, "var", ""},
      ISO_POWER_FACTOR_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      {0}}}

    ,
    {"Utility Phase C AC Power",
     65007,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Real Power", "W"), AC_OFFSET_POWER_FIELD("Apparent Power", "VA"), {0}}}

    ,
    {"Utility Phase C Basic AC Quantities",
     65008,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Utility Phase B AC Reactive Power",
     65009,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Reactive Power", BYTES(2), 1, false, "var", ""},
      ISO_POWER_FACTOR_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      {0}}}

    ,
    {"Utility Phase B AC Power",
     65010,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Real Power", "W"), AC_OFFSET_POWER_FIELD("Apparent Power", "VA"), {0}}}

    ,
    {"Utility Phase B Basic AC Quantities",
     65011,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Utility Phase A AC Reactive Power",
     65012,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Reactive Power", "var"),
      ISO_POWER_FACTOR_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      {0}}}

    ,
    {"Utility Phase A AC Power",
     65013,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Real Power", "W"), AC_OFFSET_POWER_FIELD("Apparent Power", "VA"), {0}}}

    ,
    {"Utility Phase A Basic AC Quantities",
     65014,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Utility Total AC Reactive Power",
     65015,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Reactive Power", "var"),
      ISO_POWER_FACTOR_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      {0}}}

    ,
    {"Utility Total AC Power",
     65016,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Real Power", "W"), AC_OFFSET_POWER_FIELD("Apparent Power", "VA"), {0}}}

    ,
    {"Utility Average Basic AC Quantities",
     65017,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Generator Total AC Energy",
     65018,
     PACKET_PRECISION_UNKNOWN,
     PACKET_SINGLE,
     8,
     0,
     {ENERGY_FIELD("Total Energy Export"), ENERGY_FIELD("Total Energy Import"), {0}}}

    ,
    {"Generator Phase C AC Reactive Power",
     65019,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Reactive Power", "var"),
      ISO_POWER_FACTOR_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      {0}}}

    ,
    {"Generator Phase C AC Power",
     65020,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Real Power", "W"), AC_OFFSET_POWER_FIELD("Apparent Power", "VA"), {0}}}

    ,
    {"Generator Phase C Basic AC Quantities",
     65021,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Generator Phase B AC Reactive Power",
     65022,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Reactive Power", "var"),
      ISO_POWER_FACTOR_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      {0}}}

    ,
    {"Generator Phase B AC Power",
     65023,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Real Power", "W"), AC_OFFSET_POWER_FIELD("Apparent Power", "VA"), {0}}}

    ,
    {"Generator Phase B Basic AC Quantities",
     65024,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Generator Phase A AC Reactive Power",
     65025,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Reactive Power", "var"),
      ISO_POWER_FACTOR_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      {0}}}

    ,
    {"Generator Phase A AC Power",
     65026,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Real Power", "W"), AC_OFFSET_POWER_FIELD("Apparent Power", "VA"), {0}}}

    ,
    {"Generator Phase A Basic AC Quantities",
     65027,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Generator Total AC Reactive Power",
     65028,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Reactive Power", "var"),
      ISO_POWER_FACTOR_FIELD,
      LOOKUP_FIELD("Power Factor Lagging", 2, POWER_FACTOR),
      {0}}}

    ,
    {"Generator Total AC Power",
     65029,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {AC_OFFSET_POWER_FIELD("Real Power", "W"), AC_OFFSET_POWER_FIELD("Apparent Power", "VA"), {0}}}

    ,
    {"Generator Average Basic AC Quantities",
     65030,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {VOLTAGE_FIELD("Line-Line AC RMS Voltage", 1),
      VOLTAGE_FIELD("Line-Neutral AC RMS Voltage", 1),
      FREQUENCY_FIELD("AC Frequency", 1 / 128.0),
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"ISO Commanded Address",
     65240,
     PACKET_COMPLETE,
     PACKET_ISO11783,
     9,
     0,
     /* ISO 11783 defined this message to provide a mechanism for assigning a network address to a node. The NAME information in the
     data portion of the message must match the name information of the node whose network address is to be set. */
     {BINARY_FIELD("Unique Number", 21, "ISO Identity Number"),
      SIMPLE_FIELD("Manufacturer Code", 11),
      SIMPLE_DESC_FIELD("Device Instance Lower", 3, "ISO ECU Instance"),
      SIMPLE_DESC_FIELD("Device Instance Upper", 5, "ISO Function Instance"),
      SIMPLE_DESC_FIELD("Device Function", BYTES(1), "ISO Function"),
      RESERVED_FIELD(1),
      LOOKUP_FIELD("Device Class", 7, DEVICE_CLASS),
      SIMPLE_DESC_FIELD("System Instance", 4, "ISO Device Class Instance"),
      LOOKUP_FIELD("Industry Code", 3, INDUSTRY_CODE),
      RESERVED_FIELD(1),
      ONE_BYTE_FIELD("New Source Address"),
      {0}}}

    /* proprietary PDU2 (non addressed) single-frame range 0xFF00 to 0xFFFF (65280 - 65535) */

    ,
    {"Furuno: Heave",
     65280,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1855), DISTANCE_FIELD("Heave", BYTES(4), 0.001, ""), RESERVED_FIELD(BYTES(2)), {0}}}

    ,
    {"Manufacturer Proprietary single-frame non-addressed",
     65280,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {MANUFACTURER_FIELDS, BINARY_FIELD("Data", BYTES(6), ""), {0}},
     0,
     0,
     true}

    ,
    {"Maretron: Proprietary DC Breaker Current",
     65284,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {COMPANY(137),
      ONE_BYTE_FIELD("Bank Instance"),
      ONE_BYTE_FIELD("Indicator Number"),
      {"Breaker Current", BYTES(2), 0.1, true, "A", ""},
      RESERVED_FIELD(BYTES(2)),
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Boot State Acknowledgment",
     65285,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {COMPANY(135), LOOKUP_FIELD("Boot State", 4, BOOT_STATE), {0}}}

    ,
    {"Lowrance: Temperature",
     65285,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {COMPANY(140), LOOKUP_FIELD("Temperature Source", BYTES(1), TEMPERATURE_SOURCE), TEMPERATURE_FIELD("Actual Temperature"), {0}}}

    ,
    {"Chetco: Dimmer",
     65286,
     PACKET_INCOMPLETE_LOOKUP,
     PACKET_SINGLE,
     8,
     0,
     {COMPANY(409),
      INSTANCE_FIELD,
      ONE_BYTE_FIELD("Dimmer1"),
      ONE_BYTE_FIELD("Dimmer2"),
      ONE_BYTE_FIELD("Dimmer3"),
      ONE_BYTE_FIELD("Dimmer4"),
      ONE_BYTE_FIELD("Control"),
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Boot State Request", 65286, PACKET_COMPLETE, PACKET_SINGLE, 8, 0, {COMPANY(135), {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Access Level",
     65287,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {COMPANY(135),
      INTEGER_FIELD("Format Code", 3),
      LOOKUP_FIELD("Access Level", 3, ACCESS_LEVEL),
      RESERVED_FIELD(2),
      INTEGER_DESC_FIELD(
          "Access Seed/Key",
          BYTES(4),
          "When transmitted, it provides a seed for an unlock operation. It is used to provide the key during PGN 126208."),
      {0}}}

    ,
    {"Simnet: Configure Temperature Sensor", 65287, PACKET_INCOMPLETE, PACKET_SINGLE, 0x08, 0, {COMPANY(1857), {0}}}

    ,
    {"Seatalk: Alarm",
     65288,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1851),
      BINARY_FIELD("SID", BYTES(1), ""),
      LOOKUP_FIELD("Alarm Status", BYTES(1), SEATALK_ALARM_STATUS),
      LOOKUP_FIELD("Alarm ID", BYTES(1), SEATALK_ALARM_ID),
      LOOKUP_FIELD("Alarm Group", BYTES(1), SEATALK_ALARM_GROUP),
      BINARY_FIELD("Alarm Priority", BYTES(2), ""),
      {0}}},
    {"Simnet: Trim Tab Sensor Calibration", 65289, PACKET_INCOMPLETE, PACKET_SINGLE, 0x08, 0, {COMPANY(1857), {0}}}

    ,
    {"Simnet: Paddle Wheel Speed Configuration", 65290, PACKET_INCOMPLETE, PACKET_SINGLE, 0x08, 0, {COMPANY(1857), {0}}}

    ,
    {"Simnet: Clear Fluid Level Warnings", 65292, PACKET_INCOMPLETE, PACKET_SINGLE, 0x08, 0, {COMPANY(1857), {0}}}

    ,
    {"Simnet: LGC-2000 Configuration", 65293, PACKET_INCOMPLETE, PACKET_SINGLE, 0x08, 0, {COMPANY(1857), {0}}}

    ,
    {"Navico: Wireless Battery Status",
     65309,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(275),
      ONE_BYTE_FIELD("Status"),
      SHORT_PERCENTAGE_FIELD("Battery Status"),
      SHORT_PERCENTAGE_FIELD("Battery Charge Status"),
      RESERVED_FIELD(BYTES(3)),
      {0}}}

    ,
    {"Navico: Wireless Signal Status",
     65312,
     PACKET_FIELDS_UNKNOWN,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(275), ONE_BYTE_FIELD("Unknown"), SHORT_PERCENTAGE_FIELD("Signal Strength"), RESERVED_FIELD(BYTES(3)), {0}}}

    ,
    {"Simnet: Reprogram Status", 65325, PACKET_INCOMPLETE, PACKET_SINGLE, 0x08, 0, {COMPANY(1857), {0}}}

    ,
    {"Simnet: Autopilot Mode", 65341, PACKET_INCOMPLETE, PACKET_SINGLE, 0x08, 0, {COMPANY(1857), {0}}}

    ,
    {"Seatalk: Pilot Wind Datum",
     65345,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1851),
      ANGLE_POS_FIELD("Wind Datum", ""),
      ANGLE_POS_FIELD("Rolling Average Wind Angle", ""),
      RESERVED_FIELD(BYTES(2)),
      {0}}},
    {"Seatalk: Pilot Heading",
     65359,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1851),
      BINARY_FIELD("SID", BYTES(1), ""),
      ANGLE_POS_FIELD("Heading True", ""),
      ANGLE_POS_FIELD("Heading Magnetic", ""),
      RESERVED_FIELD(BYTES(1)),
      {0}}}

    ,
    {"Seatalk: Pilot Locked Heading",
     65360,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1851),
      BINARY_FIELD("SID", BYTES(1), ""),
      ANGLE_POS_FIELD("Target Heading True", ""),
      ANGLE_POS_FIELD("Target Heading Magnetic", ""),
      RESERVED_FIELD(BYTES(1)),
      {0}}}

    ,
    {"Seatalk: Silence Alarm",
     65361,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1851),
      LOOKUP_FIELD("Alarm ID", BYTES(1), SEATALK_ALARM_ID),
      LOOKUP_FIELD("Alarm Group", BYTES(1), SEATALK_ALARM_GROUP),
      RESERVED_FIELD(32),
      {0}}}

    ,
    {"Seatalk: Keypad Message",
     65371,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1851),
      ONE_BYTE_FIELD("Proprietary ID"),
      ONE_BYTE_FIELD("First key"),
      ONE_BYTE_FIELD("Second key"),
      SIMPLE_FIELD("First key state", 2),
      SIMPLE_FIELD("Second key state", 2),
      RESERVED_FIELD(4),
      ONE_BYTE_FIELD("Encoder Position"),
      {0}}}

    ,
    {"SeaTalk: Keypad Heartbeat",
     65374,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1851), ONE_BYTE_FIELD("Proprietary ID"), ONE_BYTE_FIELD("Variant"), ONE_BYTE_FIELD("Status"), {0}}}

    ,
    {"Seatalk: Pilot Mode",
     65379,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(1851),
      BINARY_FIELD("Pilot Mode", BYTES(1), ""),
      BINARY_FIELD("Sub Mode", BYTES(1), ""),
      BINARY_FIELD("Pilot Mode Data", BYTES(1), ""),
      RESERVED_FIELD(BYTES(3)),
      {0}}} /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Depth Quality Factor",
     65408,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {COMPANY(135), ONE_BYTE_FIELD("SID"), LOOKUP_FIELD("Depth Quality Factor", 4, AIRMAR_DEPTH_QUALITY_FACTOR), {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Speed Pulse Count",
     65409,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {COMPANY(135),
      ONE_BYTE_FIELD("SID"),
      ELAPSED_FIELD("Duration of interval", BYTES(2), 0.001),
      SIMPLE_FIELD("Number of pulses received", BYTES(2)),
      RESERVED_FIELD(BYTES(1)),
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Device Information",
     65410,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {COMPANY(135),
      ONE_BYTE_FIELD("SID"),
      TEMPERATURE_FIELD("Internal Device Temperature"),
      VOLTAGE_FIELD("Supply Voltage", 0.01),
      RESERVED_FIELD(BYTES(1)),
      {0}}}

    ,
    {"Simnet: Autopilot Mode", 65480, PACKET_INCOMPLETE, PACKET_SINGLE, 0x08, 0, {COMPANY(1857), {0}}}

    /* PDU1 (addressed) fast-packet PGN range 0x10000 to 0x1EEFF (65536 - 126719) */
    ,
    {"Unknown fast-packet addressed",
     65536,
     PACKET_INCOMPLETE_LOOKUP,
     PACKET_FAST,
     255,
     0,
     {BINARY_FIELD("Data", BYTES(255), ""), {0}},
     0,
     0,
     true}

    /* http://www.maretron.com/support/manuals/DST100UM_1.2.pdf */
    /* http://www.nmea.org/Assets/20140109%20nmea-2000-corrigendum-tc201401031%20pgn%20126208.pdf */
    ,
    {"NMEA - Request group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     12,
     2,
     {MATCH_FIELD("Function Code", BYTES(1), 0, "Request"),
      INTEGER_DESC_FIELD("PGN", BYTES(3), "Requested PGN"),
      ELAPSED_FIELD("Transmission interval", BYTES(4), 0.001),
      ELAPSED_FIELD("Transmission interval offset", BYTES(2), 0.01),
      SIMPLE_DESC_FIELD("# of Parameters", BYTES(1), "How many parameter pairs will follow"),
      INTEGER_DESC_FIELD("Parameter", BYTES(1), "Parameter index"),
      VARIABLE_FIELD("Value", "Parameter value, variable length"),
      {0}}}

    ,
    {"NMEA - Command group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     2,
     {MATCH_FIELD("Function Code", BYTES(1), 1, "Command"),
      INTEGER_DESC_FIELD("PGN", BYTES(3), "Commanded PGN"),
      LOOKUP_FIELD("Priority", 4, PRIORITY),
      RESERVED_FIELD(4),
      SIMPLE_DESC_FIELD("# of Parameters", BYTES(1), "How many parameter pairs will follow"),
      INTEGER_DESC_FIELD("Parameter", BYTES(1), "Parameter index"),
      VARIABLE_FIELD("Value", "Parameter value, variable length"),
      {0}}}

    ,
    {"NMEA - Acknowledge group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     1,
     {MATCH_FIELD("Function Code", BYTES(1), 2, "Acknowledge"),
      INTEGER_DESC_FIELD("PGN", 24, "Commanded PGN"),
      LOOKUP_FIELD("PGN error code", 4, PGN_ERROR_CODE),
      LOOKUP_FIELD("Transmission interval/Priority error code", 4, TRANSMISSION_INTERVAL),
      SIMPLE_FIELD("# of Parameters", 8),
      LOOKUP_FIELD("Parameter", 4, PARAMETER_FIELD),
      {0}}}

    ,
    {"NMEA - Read Fields group function",
     126208,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     102,
     {MATCH_FIELD("Function Code", BYTES(1), 3, "Read Fields"),
      INTEGER_DESC_FIELD("PGN", 24, "Commanded PGN"),
      MANUFACTURER_PROPRIETARY_FIELDS,
      INTEGER_FIELD("Unique ID", 8),
      SIMPLE_FIELD("# of Selection Pairs", 8),
      SIMPLE_FIELD("# of Parameters", 8),
      INTEGER_FIELD("Selection Parameter", BYTES(1)),
      VARIABLE_FIELD("Selection Value", ""),
      INTEGER_FIELD("Parameter", BYTES(1)),
      {0}}}

    ,
    {"NMEA - Read Fields reply group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     202,
     {MATCH_FIELD("Function Code", BYTES(1), 4, "Read Fields Reply"),
      INTEGER_DESC_FIELD("PGN", 24, "Commanded PGN"),
      MANUFACTURER_PROPRIETARY_FIELDS,
      INTEGER_FIELD("Unique ID", 8),
      SIMPLE_FIELD("# of Selection Pairs", 8),
      SIMPLE_FIELD("# of Parameters", 8),
      INTEGER_FIELD("Selection Parameter", BYTES(1)),
      VARIABLE_FIELD("Selection Value", ""),
      INTEGER_FIELD("Parameter", BYTES(1)),
      VARIABLE_FIELD("Value", ""),
      {0}}}

    ,
    {"NMEA - Write Fields group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     202,
     {MATCH_FIELD("Function Code", BYTES(1), 5, "Write Fields"),
      INTEGER_DESC_FIELD("PGN", 24, "Commanded PGN"),
      MANUFACTURER_PROPRIETARY_FIELDS,
      INTEGER_FIELD("Unique ID", 8),
      SIMPLE_FIELD("# of Selection Pairs", 8),
      SIMPLE_FIELD("# of Parameters", 8),
      INTEGER_FIELD("Selection Parameter", BYTES(1)),
      VARIABLE_FIELD("Selection Value", ""),
      INTEGER_FIELD("Parameter", BYTES(1)),
      VARIABLE_FIELD("Value", ""),
      {0}}}

    ,
    {"NMEA - Write Fields reply group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     202,
     {MATCH_FIELD("Function Code", BYTES(1), 6, "Write Fields Reply"),
      INTEGER_DESC_FIELD("PGN", 24, "Commanded PGN"),
      MANUFACTURER_PROPRIETARY_FIELDS,
      INTEGER_FIELD("Unique ID", 8),
      SIMPLE_FIELD("# of Selection Pairs", 8),
      SIMPLE_FIELD("# of Parameters", 8),
      INTEGER_FIELD("Selection Parameter", BYTES(1)),
      VARIABLE_FIELD("Selection Value", ""),
      INTEGER_FIELD("Parameter", BYTES(1)),
      VARIABLE_FIELD("Value", ""),
      {0}}}

    /************ RESPONSE TO REQUEST PGNS **************/

    ,
    {"PGN List (Transmit and Receive)",
     126464,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     1,
     {LOOKUP_FIELD("Function Code", BYTES(1), PGN_LIST_FUNCTION),
      {
          "PGN",
          24,
          RES_INTEGER,
          false,
          0,
      },
      {0}}}

    /* proprietary PDU1 (addressed) fast-packet PGN range 0x1EF00 to 0x1EFFF (126720 - 126975) */

    ,
    {"Seatalk1: Pilot Mode",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     21,
     0,
     {COMPANY(1851),
      MATCH_FIELD("Proprietary ID", BYTES(2), 33264, "0x81f0"),
      MATCH_FIELD("command", BYTES(1), 132, "0x84"),
      BINARY_FIELD("Unknown 1", BYTES(3), ""),
      LOOKUP_FIELD("Pilot Mode", BYTES(1), SEATALK_PILOT_MODE),
      INTEGER_FIELD("Sub Mode", BYTES(1)),
      BINARY_FIELD("Pilot Mode Data", BYTES(1), ""),
      BINARY_FIELD("Unknown 2", BYTES(10), ""),
      {0}}}

    ,
    {"Fusion: Media Control",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {COMPANY(419),
      MATCH_FIELD("Proprietary ID", BYTES(1), 3, "Media Control"),
      INTEGER_FIELD("Unknown", BYTES(1)),
      INTEGER_FIELD("Source ID", BYTES(1)),
      LOOKUP_FIELD("Command", BYTES(1), FUSION_COMMAND),
      {0}}}

    ,
    {"Fusion: Sirius Control",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     7,
     0,
     {COMPANY(419),
      MATCH_FIELD("Proprietary ID", BYTES(1), 30, "Sirius Control"),
      INTEGER_FIELD("Unknown", BYTES(1)),
      INTEGER_FIELD("Source ID", BYTES(1)),
      LOOKUP_FIELD("Command", BYTES(1), FUSION_SIRIUS_COMMAND),
      {0}}}

    ,
    {"Fusion: Request Status",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     3,
     0,
     {COMPANY(419), MATCH_FIELD("Proprietary ID", BYTES(1), 1, "Request Status"), INTEGER_FIELD("Unknown", BYTES(1)), {0}}}

    ,
    {"Fusion: Set Source",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     3,
     0,
     {COMPANY(419),
      MATCH_FIELD("Proprietary ID", BYTES(1), 2, "Set Source"),
      INTEGER_FIELD("Unknown", BYTES(1)),
      INTEGER_FIELD("Source ID", BYTES(1)),
      {0}}}

    ,
    {"Fusion: Mute",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     3,
     0,
     {COMPANY(419),
      MATCH_FIELD("Proprietary ID", BYTES(1), 17, "Mute"),
      LOOKUP_FIELD("Command", BYTES(1), FUSION_MUTE_COMMAND),
      {0}}}

    ,
    {"Fusion: Set Zone Volume",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {COMPANY(419),
      MATCH_FIELD("Proprietary ID", BYTES(1), 24, "Set Zone Volume"),
      INTEGER_FIELD("Unknown", BYTES(1)),
      INTEGER_FIELD("Zone", BYTES(1)),
      INTEGER_FIELD("Volume", BYTES(1)),
      {0}}}

    ,
    {"Fusion: Set All Volumes",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {COMPANY(419),
      MATCH_FIELD("Proprietary ID", BYTES(1), 25, "Set All Volumes"),
      INTEGER_FIELD("Unknown", BYTES(1)),
      INTEGER_FIELD("Zone1", BYTES(1)),
      INTEGER_FIELD("Zone2", BYTES(1)),
      INTEGER_FIELD("Zone3", BYTES(1)),
      INTEGER_FIELD("Zone4", BYTES(1)),
      {0}}}

    /* Seatalk1 code from http://thomasknauf.de/rap/seatalk2.htm */
    ,
    {"Seatalk1: Keystroke",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     21,
     0,
     {COMPANY(1851),
      MATCH_FIELD("Proprietary ID", BYTES(2), 33264, "0x81f0"),
      MATCH_FIELD("command", BYTES(1), 134, "0x86"),
      INTEGER_FIELD("device", BYTES(1)),
      LOOKUP_FIELD("key", BYTES(1), SEATALK_KEYSTROKE),
      INTEGER_DESC_FIELD("keyInverted", BYTES(1), "Bit negated version of key"),
      BINARY_FIELD("Unknown data", BYTES(14), ""),
      // xx xx xx xx xx c1 c2 cd 64 80 d3 42 f1 c8 (if xx=0xff =>working or xx xx xx xx xx = [A5 FF FF FF FF | 00 00 00 FF FF |
      // FF FF FF FF FF | 42 00 F8 02 05])
      {0}}}

    ,
    {"Seatalk1: Device Identification",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {COMPANY(1851),
      MATCH_FIELD("Proprietary ID", BYTES(2), 33264, "0x81f0"),
      MATCH_FIELD("command", BYTES(1), 144, "0x90"),
      RESERVED_FIELD(BYTES(1)),
      LOOKUP_FIELD("device", BYTES(1), SEATALK_DEVICE_ID),
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/PB200UserManual.pdf */
    ,
    {"Airmar: Attitude Offset",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     9,
     0,
     {COMPANY(135),
      MATCH_FIELD("Proprietary ID", BYTES(1), 32, "Attitude Offsets"),
      {"Azimuth offset",
       BYTES(2),
       RES_RADIANS,
       true,
       "rad",
       "Positive: sensor rotated to port, negative: sensor rotated to starboard"},
      ANGLE_REL_FIELD("Pitch offset", "Positive: sensor tilted to bow, negative: sensor tilted to stern"),
      ANGLE_REL_FIELD("Roll offset", "Positive: sensor tilted to port, negative: sensor tilted to starboard"),
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/PB200UserManual.pdf */
    ,
    {"Airmar: Calibrate Compass",
     126720,
     PACKET_FIELDS_UNKNOWN,
     PACKET_FAST,
     24,
     0,
     {COMPANY(135),
      MATCH_FIELD("Proprietary ID", BYTES(1), 33, "Calibrate Compass"),
      LOOKUP_FIELD("Calibrate Function", BYTES(1), AIRMAR_CALIBRATE_FUNCTION),
      LOOKUP_FIELD("Calibration Status", BYTES(1), AIRMAR_CALIBRATE_STATUS),
      INTEGER_DESC_FIELD("Verify Score", BYTES(1), "TBD"),
      {"X-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"Y-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"Z-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"X-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"Y-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"Z-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"X-axis angular offset", BYTES(2), 0.1, true, "deg", "default 0, range 0 to 3600"},
      {"Pitch and Roll damping", BYTES(2), 0.05, true, "s", "default 30, range 0 to 200"},
      {"Compass/Rate gyro damping",
       BYTES(2),
       0.05,
       true,
       "s",
       "default -30, range -2400 to 2400, negative indicates rate gyro is to be used in compass calculations"},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/PB200UserManual.pdf */
    ,
    {"Airmar: True Wind Options",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {COMPANY(135),
      MATCH_FIELD("Proprietary ID", BYTES(1), 34, "True Wind Options"),
      LOOKUP_FIELD_DESC("COG substitution for HDG", 2, YES_NO, "Allow use of COG when HDG not available?"),
      LOOKUP_FIELD("Calibration Status", BYTES(1), AIRMAR_CALIBRATE_STATUS),
      INTEGER_DESC_FIELD("Verify Score", BYTES(1), "TBD"),
      {"X-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"Y-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"Z-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"X-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"Y-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"Z-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"X-axis angular offset", BYTES(2), 0.1, true, "deg", "default 0, range 0 to 3600"},
      {"Pitch and Roll damping", BYTES(2), 0.05, true, "s", "default 30, range 0 to 200"},
      {"Compass/Rate gyro damping",
       BYTES(2),
       0.05,
       true,
       "s",
       "default -30, range -2400 to 2400, negative indicates rate gyro is to be used in compass calculations"},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Simulate Mode",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     6,
     0,
     {COMPANY(135),
      MATCH_FIELD("Proprietary ID", BYTES(1), 35, "Simulate Mode"),
      LOOKUP_FIELD("Simulate Mode", 2, OFF_ON),
      RESERVED_FIELD(22),
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Calibrate Depth",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     6,
     0,
     {COMPANY(135),
      MATCH_FIELD("Proprietary ID", BYTES(1), 40, "Calibrate Depth"),
      {"Speed of Sound Mode", BYTES(2), 0.1, false, "m/s", "actual allowed range is 1350.0 to 1650.0 m/s"},
      RESERVED_FIELD(8),
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Calibrate Speed",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     12,
     2,
     {COMPANY(135),
      MATCH_FIELD("Proprietary ID", BYTES(1), 41, "Calibrate Speed"),
      {"Number of pairs of data points",
       BYTES(1),
       RES_INTEGER,
       false,
       0,
       "actual range is 0 to 25. 254=restore default speed curve"},
      FREQUENCY_FIELD("Input frequency", 0.1),
      SPEED_FIELD("Output speed"),
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Calibrate Temperature",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     6,
     2,
     {COMPANY(135),
      MATCH_FIELD("Proprietary ID", BYTES(1), 42, "Calibrate Temperature"),
      LOOKUP_FIELD("Temperature instance", 2, AIRMAR_TEMPERATURE_INSTANCE),
      RESERVED_FIELD(6),
      {"Temperature offset", BYTES(2), 0.001, true, "K", "actual range is -9.999 to +9.999 K"},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Speed Filter",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     2,
     {COMPANY(135),
      MATCH_FIELD("Proprietary ID", BYTES(1), 43, "Speed Filter"),
      LOOKUP_FIELD("Filter type", 4, AIRMAR_FILTER),
      RESERVED_FIELD(4),
      ELAPSED_FIELD("Sample interval", BYTES(2), 0.01),
      ELAPSED_FIELD("Filter duration", BYTES(2), 0.01),
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Temperature Filter",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     2,
     {COMPANY(135),
      MATCH_FIELD("Proprietary ID", BYTES(1), 44, "Temperature Filter"),
      LOOKUP_FIELD("Filter type", 4, AIRMAR_FILTER),
      RESERVED_FIELD(4),
      ELAPSED_FIELD("Sample interval", BYTES(2), 0.01),
      ELAPSED_FIELD("Filter duration", BYTES(2), 0.01),
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: NMEA 2000 options",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     6,
     2,
     {COMPANY(135),
      MATCH_FIELD("Proprietary ID", BYTES(1), 46, "NMEA 2000 options"),
      LOOKUP_FIELD("Transmission Interval", 2, AIRMAR_TRANSMISSION_INTERVAL),
      RESERVED_FIELD(22),
      {0}}}

    ,
    {"Airmar: Addressable Multi-Frame",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     4,
     0,
     {COMPANY(135), INTEGER_FIELD("Proprietary ID", BYTES(1)), {0}}}

    ,
    {"Maretron: Slave Response",
     126720,
     PACKET_LOOKUPS_UNKNOWN,
     PACKET_FAST,
     8,
     0,
     {COMPANY(137),
      SIMPLE_DESC_FIELD("Product code", BYTES(2), "0x1b2=SSC200"),
      SIMPLE_FIELD("Software code", BYTES(2)),
      SIMPLE_DESC_FIELD("Command", BYTES(1), "0x50=Deviation calibration result"),
      ONE_BYTE_FIELD("Status"),
      {0}}}

    ,
    {"Manufacturer Proprietary fast-packet addressed",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     223,
     0,
     {MANUFACTURER_FIELDS, BINARY_FIELD("Data", BYTES(221), ""), {0}},
     0,
     0,
     true}

    /* PDU2 (non addressed) fast packet PGN range 0x1F000 to 0x1FEFF (126976 - 130815) */
    ,
    {"Unknown fast-packet non-addressed",
     126976,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     255,
     0,
     {BINARY_FIELD("Data", BYTES(255), ""), {0}},
     0,
     0,
     true}

    ,
    {"Alert",
     126983,
     PACKET_COMPLETE,
     PACKET_FAST,
     28,
     0,
     {LOOKUP_FIELD("Alert Type", 4, ALERT_TYPE),
      LOOKUP_FIELD("Alert Category", 4, ALERT_CATEGORY),
      ONE_BYTE_FIELD("Alert System"),
      ONE_BYTE_FIELD("Alert Sub-System"),
      SIMPLE_FIELD("Alert ID", BYTES(2)),
      SIMPLE_FIELD("Data Source Network ID NAME", BYTES(8)),
      ONE_BYTE_FIELD("Data Source Instance"),
      ONE_BYTE_FIELD("Data Source Index-Source"),
      ONE_BYTE_FIELD("Alert Occurrence Number"),
      LOOKUP_FIELD("Temporary Silence Status", 1, YES_NO),
      LOOKUP_FIELD("Acknowledge Status", 1, YES_NO),
      LOOKUP_FIELD("Escalation Status", 1, YES_NO),
      LOOKUP_FIELD("Temporary Silence Support", 1, YES_NO),
      LOOKUP_FIELD("Acknowledge Support", 1, YES_NO),
      LOOKUP_FIELD("Escalation Support", 1, YES_NO),
      BINARY_FIELD("NMEA Reserved", 2, ""),
      SIMPLE_FIELD("Acknowledge Source Network ID NAME", BYTES(8)),
      LOOKUP_FIELD("Trigger Condition", 4, ALERT_TRIGGER_CONDITION),
      LOOKUP_FIELD("Threshold Status", 4, ALERT_THRESHOLD_STATUS),
      ONE_BYTE_FIELD("Alert Priority"),
      LOOKUP_FIELD("Alert State", BYTES(1), ALERT_STATE),
      {0}}}

    ,
    {"Alert Response",
     126984,
     PACKET_COMPLETE,
     PACKET_FAST,
     25,
     0,
     {LOOKUP_FIELD("Alert Type", 4, ALERT_TYPE),
      LOOKUP_FIELD("Alert Category", 4, ALERT_CATEGORY),
      ONE_BYTE_FIELD("Alert System"),
      ONE_BYTE_FIELD("Alert Sub-System"),
      SIMPLE_FIELD("Alert ID", BYTES(2)),
      SIMPLE_FIELD("Data Source Network ID NAME", BYTES(8)),
      ONE_BYTE_FIELD("Data Source Instance"),
      ONE_BYTE_FIELD("Data Source Index-Source"),
      ONE_BYTE_FIELD("Alert Occurrence Number"),
      SIMPLE_FIELD("Acknowledge Source Network ID NAME", BYTES(8)),
      LOOKUP_FIELD("Response Command", 2, ALERT_RESPONSE_COMMAND),
      BINARY_FIELD("NMEA Reserved", 6, ""),
      {0}}}

    ,
    {"Alert Text",
     126985,
     PACKET_COMPLETE,
     PACKET_FAST,
     49,
     0,
     {LOOKUP_FIELD("Alert Type", 4, ALERT_TYPE),
      LOOKUP_FIELD("Alert Category", 4, ALERT_CATEGORY),
      ONE_BYTE_FIELD("Alert System"),
      ONE_BYTE_FIELD("Alert Sub-System"),
      SIMPLE_FIELD("Alert ID", BYTES(2)),
      SIMPLE_FIELD("Data Source Network ID NAME", BYTES(8)),
      ONE_BYTE_FIELD("Data Source Instance"),
      ONE_BYTE_FIELD("Data Source Index-Source"),
      ONE_BYTE_FIELD("Alert Occurrence Number"),
      LOOKUP_FIELD("Language ID", BYTES(1), ALERT_LANGUAGE_ID),
      STRINGLAU_FIELD("Alert Text Description"),
      STRINGLAU_FIELD("Alert Location Text Description"),
      {0}}}

    ,
    {"Alert Configuration", 126986, PACKET_INCOMPLETE, PACKET_FAST, 8, 0, {{0}}}

    ,
    {"Alert Threshold", 126987, PACKET_INCOMPLETE, PACKET_FAST, 8, 0, {{0}}}

    ,
    {"Alert Value", 126988, PACKET_INCOMPLETE, PACKET_FAST, 8, 0, {{0}}}

    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    ,
    {"System Time",
     126992,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      LOOKUP_FIELD("Source", 4, SYSTEM_TIME),
      RESERVED_FIELD(4),
      {"Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      TIME_FIELD("Time", "Seconds since midnight"),
      {0}}}

    /* http://www.nmea.org/Assets/20140102%20nmea-2000-126993%20heartbeat%20pgn%20corrigendum.pdf */
    /* http://www.nmea.org/Assets/20190624%20NMEA%20Heartbeat%20Information%20Amendment%20AT%2020190623HB.pdf */
    ,
    {"Heartbeat",
     126993,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Data transmit offset",
       BYTES(2),
       0.001,
       false,
       "s",
       "Offset in transmit time from time of request command: 0x0 = transmit immediately, 0xFFFF = Do not change offset."},
      INTEGER_FIELD("Sequence Counter", BYTES(1)),
      LOOKUP_FIELD("Controller 1 State", 2, CONTROLLER_STATE),
      LOOKUP_FIELD("Controller 2 State", 2, CONTROLLER_STATE),
      LOOKUP_FIELD("Equipment Status", 2, EQUIPMENT_STATUS),
      RESERVED_FIELD(34),
      {0}}}

    ,
    {"Product Information",
     126996,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x86,
     0,
     {SIMPLE_FIELD("NMEA 2000 Version", BYTES(2)),
      SIMPLE_FIELD("Product Code", BYTES(2)),
      ASCII_FIELD("Model ID", BYTES(32)),
      ASCII_FIELD("Software Version Code", BYTES(32)),
      ASCII_FIELD("Model Version", BYTES(32)),
      ASCII_FIELD("Model Serial Code", BYTES(32)),
      ONE_BYTE_FIELD("Certification Level"),
      ONE_BYTE_FIELD("Load Equivalency"),
      {0}}}

    ,
    {"Configuration Information",
     126998,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x2a,
     0,
     {STRINGLAU_FIELD("Installation Description #1"),
      STRINGLAU_FIELD("Installation Description #2"),
      STRINGLAU_FIELD("Manufacturer Information"),
      {0}}}

    /************ PERIODIC DATA PGNs **************/
    /* http://www.nmea.org/Assets/july%202010%20nmea2000_v1-301_app_b_pgn_field_list.pdf */
    /* http://www.maretron.com/support/manuals/USB100UM_1.2.pdf */
    /* http://www8.garmin.com/manuals/GPSMAP4008_NMEA2000NetworkFundamentals.pdf */

    /* http://www.nmea.org/Assets/20130906%20nmea%202000%20%20man%20overboard%20notification%20%28mob%29%20pgn%20127233%20amendment.pdf
     */
    ,
    {"Man Overboard Notification",
     127233,
     PACKET_COMPLETE,
     PACKET_FAST,
     35,
     0,
     {ONE_BYTE_FIELD("SID"),
      INTEGER_DESC_FIELD("MOB Emitter ID", BYTES(4), "Identifier for each MOB emitter, unique to the vessel"),
      LOOKUP_FIELD("Man Overboard Status", 3, MOB_STATUS),
      RESERVED_FIELD(5),
      TIME_FIELD("Activation Time", "Time of day (UTC) when MOB was activated"),
      LOOKUP_FIELD("Position Source", 3, MOB_POSITION_SOURCE),
      RESERVED_FIELD(5),
      {"Position Date", BYTES(2), RES_DATE, false, "", "Date of MOB position"},
      TIME_FIELD("Position Time", "Time of day of MOB position (UTC)"),
      {"Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      LOOKUP_FIELD("COG Reference", 2, DIRECTION_REFERENCE),
      RESERVED_FIELD(6),
      ANGLE_POS_FIELD("COG", ""),
      SPEED_FIELD("SOG"),
      {"MMSI of vessel of origin", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      LOOKUP_FIELD("MOB Emitter Battery Low Status", 3, LOW_BATTERY),
      RESERVED_FIELD(5),
      {0}}}

    ,
    {"Heading/Track control",
     127237,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x15,
     0,
     {LOOKUP_FIELD("Rudder Limit Exceeded", 2, YES_NO),
      LOOKUP_FIELD("Off-Heading Limit Exceeded", 2, YES_NO),
      LOOKUP_FIELD("Off-Track Limit Exceeded", 2, YES_NO),
      LOOKUP_FIELD("Override", 2, YES_NO),
      LOOKUP_FIELD("Steering Mode", 3, STEERING_MODE),
      LOOKUP_FIELD("Turn Mode", 3, TURN_MODE),
      LOOKUP_FIELD("Heading Reference", 2, DIRECTION_REFERENCE),
      RESERVED_FIELD(5),
      LOOKUP_FIELD("Commanded Rudder Direction", 3, DIRECTION_RUDDER),
      ANGLE_REL_FIELD("Commanded Rudder Angle", ""),
      ANGLE_POS_FIELD("Heading-To-Steer (Course)", ""),
      ANGLE_POS_FIELD("Track", ""),
      ANGLE_POS_FIELD("Rudder Limit", ""),
      ANGLE_POS_FIELD("Off-Heading Limit", ""),
      ANGLE_REL_FIELD("Radius of Turn Order", ""),
      {"Rate of Turn Order", BYTES(2), RES_ROTATION, true, "rad/s", ""},
      DISTANCE_FIELD("Off-Track Limit", BYTES(2), 1, ""),
      ANGLE_POS_FIELD("Vessel Heading", ""),
      {0}}}

    /* http://www.maretron.com/support/manuals/RAA100UM_1.0.pdf */
    ,
    {"Rudder",
     127245,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {INSTANCE_FIELD,
      LOOKUP_FIELD("Direction Order", 3, DIRECTION_RUDDER),
      RESERVED_FIELD(5),
      ANGLE_REL_FIELD("Angle Order", ""),
      ANGLE_REL_FIELD("Position", ""),
      RESERVED_FIELD(BYTES(2)),
      {0}}}

    /* NMEA + Simrad AT10 */
    /* http://www.maretron.com/support/manuals/SSC200UM_1.7.pdf */
    /* molly_rose_E80start.kees */
    ,
    {"Vessel Heading",
     127250,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      ANGLE_POS_FIELD("Heading", ""),
      ANGLE_REL_FIELD("Deviation", ""),
      ANGLE_REL_FIELD("Variation", ""),
      LOOKUP_FIELD("Reference", 2, DIRECTION_REFERENCE),
      RESERVED_FIELD(6),
      {0}}}

    /* http://www.maretron.com/support/manuals/SSC200UM_1.7.pdf */
    /* Lengths observed from Simrad RC42 */
    ,
    {"Rate of Turn",
     127251,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     5,
     0,
     {ONE_BYTE_FIELD("SID"), {"Rate", BYTES(4), RES_HIRES_ROTATION, true, "rad/s", ""}, {0}}}

    ,
    {"Heave",
     127252,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {ONE_BYTE_FIELD("SID"), DISTANCE_FIELD("Heave", BYTES(2), 0.01, ""), RESERVED_FIELD(BYTES(5)), {0}}}

    ,
    {"Attitude",
     127257,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     7,
     0,
     {ONE_BYTE_FIELD("SID"), ANGLE_REL_FIELD("Yaw", ""), ANGLE_REL_FIELD("Pitch", ""), ANGLE_REL_FIELD("Roll", ""), {0}}}

    /* NMEA + Simrad AT10 */
    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    ,
    {"Magnetic Variation",
     127258,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     6,
     0,
     {ONE_BYTE_FIELD("SID"),
      LOOKUP_FIELD("Source", 4, MAGNETIC_VARIATION),
      RESERVED_FIELD(4),
      {"Age of service", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      ANGLE_REL_FIELD("Variation", ""),
      {0}}}

    /* Engine group PGNs all derived PGN Numbers from              */
    /* http://www.maretron.com/products/pdf/J2K100-Data_Sheet.pdf  */
    /* http://www.floscan.com/html/blue/NMEA2000.php               */
    /* http://www.osukl.com/wp-content/uploads/2015/04/3155-UM.pdf */
    ,
    {"Engine Parameters, Rapid Update",
     127488,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {LOOKUP_FIELD("Instance", BYTES(1), ENGINE_INSTANCE),
      {"Speed", BYTES(2), 0.25, false, "rpm", ""},
      {"Boost Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Tilt/Trim", BYTES(1), 1, true, "", ""},
      RESERVED_FIELD(BYTES(2)),
      {0}}}

    // http://www.osukl.com/wp-content/uploads/2015/04/3155-UM.pdf
    ,
    {"Engine Parameters, Dynamic",
     127489,
     PACKET_COMPLETE,
     PACKET_FAST,
     26,
     0,
     {LOOKUP_FIELD("Instance", BYTES(1), ENGINE_INSTANCE),
      {"Oil pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      TEMPERATURE_HIGH_FIELD("Oil temperature"),
      TEMPERATURE_FIELD("Temperature"),
      VOLTAGE_FIELD("Alternator Potential", 0.01),
      {"Fuel Rate", BYTES(2), 0.1, true, "L/h", ""},
      ELAPSED_FIELD("Total Engine hours", BYTES(4), 1.0),
      {"Coolant Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Fuel Pressure", BYTES(2), 1, false, "kPa", ""},
      RESERVED_FIELD(BYTES(1)),
      LOOKUP_BITFIELD("Discrete Status 1", BYTES(2), ENGINE_STATUS_1),
      LOOKUP_BITFIELD("Discrete Status 2", BYTES(2), ENGINE_STATUS_2),
      SHORT_PERCENTAGE_FIELD("Engine Load"),
      SHORT_PERCENTAGE_FIELD("Engine Torque"),
      {0}}}

    ,
    {"Transmission Parameters, Dynamic",
     127493,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {LOOKUP_FIELD("Instance", 8, ENGINE_INSTANCE),
      LOOKUP_FIELD("Transmission Gear", 2, GEAR_STATUS),
      RESERVED_FIELD(6),
      {"Oil pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      TEMPERATURE_HIGH_FIELD("Oil temperature"),
      INTEGER_FIELD("Discrete Status 1", BYTES(1)),
      RESERVED_FIELD(BYTES(1)),
      {0}}}

    ,
    {"Trip Parameters, Vessel",
     127496,
     PACKET_COMPLETE,
     PACKET_FAST,
     10,
     0,
     {ELAPSED_FIELD("Time to Empty", BYTES(4), 0.001),
      HIRES_LENGTH_FIELD("Distance to Empty", 0.01),
      {"Estimated Fuel Remaining", BYTES(2), 1, false, "L", ""},
      ELAPSED_FIELD("Trip Run Time", BYTES(4), 0.001),
      {0}}}

    ,
    {"Trip Parameters, Engine",
     127497,
     PACKET_COMPLETE,
     PACKET_FAST,
     9,
     0,
     {LOOKUP_FIELD("Instance", BYTES(1), ENGINE_INSTANCE),
      {"Trip Fuel Used", BYTES(2), 1, false, "L", ""},
      {"Fuel Rate, Average", BYTES(2), 0.1, true, "L/h", ""},
      {"Fuel Rate, Economy", BYTES(2), 0.1, true, "L/h", ""},
      {"Instantaneous Fuel Economy", BYTES(2), 0.1, true, "L/h", ""},
      {0}}}

    ,
    {"Engine Parameters, Static",
     127498,
     PACKET_COMPLETE,
     PACKET_FAST,
     52,
     0,
     {LOOKUP_FIELD("Instance", BYTES(1), ENGINE_INSTANCE),
      {"Rated Engine Speed", BYTES(2), 0.25, false, 0, "rpm"},
      ASCII_FIELD("VIN", BYTES(17)),
      ASCII_FIELD("Software ID", BYTES(32)),
      {0}}}

    ,
    {"Load Controller Connection State/Control",
     127500,
     PACKET_COMPLETE,
     PACKET_FAST,
     10,
     0,
     {INTEGER_FIELD("Sequence ID", BYTES(1)),
      INTEGER_FIELD("Connection ID", BYTES(1)),
      INTEGER_FIELD("State", BYTES(1)),
      INTEGER_FIELD("Status", BYTES(1)),
      INTEGER_FIELD("Operational Status & Control", BYTES(1)),
      INTEGER_FIELD("PWM Duty Cycle", BYTES(1)),
      INTEGER_FIELD("TimeON", BYTES(2)),
      INTEGER_FIELD("TimeOFF", BYTES(2)),
      {0}}}

    ,
    {"Binary Switch Bank Status",
     127501,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {INSTANCE_FIELD,
      LOOKUP_FIELD("Indicator1", 2, OFF_ON),
      LOOKUP_FIELD("Indicator2", 2, OFF_ON),
      LOOKUP_FIELD("Indicator3", 2, OFF_ON),
      LOOKUP_FIELD("Indicator4", 2, OFF_ON),
      LOOKUP_FIELD("Indicator5", 2, OFF_ON),
      LOOKUP_FIELD("Indicator6", 2, OFF_ON),
      LOOKUP_FIELD("Indicator7", 2, OFF_ON),
      LOOKUP_FIELD("Indicator8", 2, OFF_ON),
      LOOKUP_FIELD("Indicator9", 2, OFF_ON),
      LOOKUP_FIELD("Indicator10", 2, OFF_ON),
      LOOKUP_FIELD("Indicator11", 2, OFF_ON),
      LOOKUP_FIELD("Indicator12", 2, OFF_ON),
      LOOKUP_FIELD("Indicator13", 2, OFF_ON),
      LOOKUP_FIELD("Indicator14", 2, OFF_ON),
      LOOKUP_FIELD("Indicator15", 2, OFF_ON),
      LOOKUP_FIELD("Indicator16", 2, OFF_ON),
      LOOKUP_FIELD("Indicator17", 2, OFF_ON),
      LOOKUP_FIELD("Indicator18", 2, OFF_ON),
      LOOKUP_FIELD("Indicator19", 2, OFF_ON),
      LOOKUP_FIELD("Indicator20", 2, OFF_ON),
      LOOKUP_FIELD("Indicator21", 2, OFF_ON),
      LOOKUP_FIELD("Indicator22", 2, OFF_ON),
      LOOKUP_FIELD("Indicator23", 2, OFF_ON),
      LOOKUP_FIELD("Indicator24", 2, OFF_ON),
      LOOKUP_FIELD("Indicator25", 2, OFF_ON),
      LOOKUP_FIELD("Indicator26", 2, OFF_ON),
      LOOKUP_FIELD("Indicator27", 2, OFF_ON),
      LOOKUP_FIELD("Indicator28", 2, OFF_ON),
      {0}}},
    {"Switch Bank Control",
     127502,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {INSTANCE_FIELD,
      LOOKUP_FIELD("Switch1", 2, OFF_ON),
      LOOKUP_FIELD("Switch2", 2, OFF_ON),
      LOOKUP_FIELD("Switch3", 2, OFF_ON),
      LOOKUP_FIELD("Switch4", 2, OFF_ON),
      LOOKUP_FIELD("Switch5", 2, OFF_ON),
      LOOKUP_FIELD("Switch6", 2, OFF_ON),
      LOOKUP_FIELD("Switch7", 2, OFF_ON),
      LOOKUP_FIELD("Switch8", 2, OFF_ON),
      LOOKUP_FIELD("Switch9", 2, OFF_ON),
      LOOKUP_FIELD("Switch10", 2, OFF_ON),
      LOOKUP_FIELD("Switch11", 2, OFF_ON),
      LOOKUP_FIELD("Switch12", 2, OFF_ON),
      LOOKUP_FIELD("Switch13", 2, OFF_ON),
      LOOKUP_FIELD("Switch14", 2, OFF_ON),
      LOOKUP_FIELD("Switch15", 2, OFF_ON),
      LOOKUP_FIELD("Switch16", 2, OFF_ON),
      LOOKUP_FIELD("Switch17", 2, OFF_ON),
      LOOKUP_FIELD("Switch18", 2, OFF_ON),
      LOOKUP_FIELD("Switch19", 2, OFF_ON),
      LOOKUP_FIELD("Switch20", 2, OFF_ON),
      LOOKUP_FIELD("Switch21", 2, OFF_ON),
      LOOKUP_FIELD("Switch22", 2, OFF_ON),
      LOOKUP_FIELD("Switch23", 2, OFF_ON),
      LOOKUP_FIELD("Switch24", 2, OFF_ON),
      LOOKUP_FIELD("Switch25", 2, OFF_ON),
      LOOKUP_FIELD("Switch26", 2, OFF_ON),
      LOOKUP_FIELD("Switch27", 2, OFF_ON),
      LOOKUP_FIELD("Switch28", 2, OFF_ON),
      {0}}}

    /* http://www.nmea.org/Assets/nmea-2000-corrigendum-1-2010-1.pdf */
    ,
    {"AC Input Status",
     127503,
     PACKET_COMPLETE,
     PACKET_FAST,
     2 + 3 * 18,
     10,
     {INSTANCE_FIELD,
      ONE_BYTE_FIELD("Number of Lines")

          ,
      INTEGER_FIELD("Line", 2),
      LOOKUP_FIELD("Acceptability", 2, ACCEPTABILITY),
      RESERVED_FIELD(4),
      VOLTAGE_FIELD("Voltage", 0.01),
      {"Current", BYTES(2), 0.1, false, "A", ""},
      FREQUENCY_FIELD("Frequency", 0.01),
      {"Breaker Size", BYTES(2), 0.1, false, "A", ""},
      {"Real Power", BYTES(4), RES_INTEGER, false, "W", ""},
      {"Reactive Power", BYTES(4), RES_INTEGER, false, "VAR", ""},
      POWER_FACTOR_FIELD,
      {0}}}

    /* http://www.nmea.org/Assets/nmea-2000-corrigendum-1-2010-1.pdf */
    ,
    {"AC Output Status",
     127504,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     2 + 3 * 18,
     10,
     {INSTANCE_FIELD,
      ONE_BYTE_FIELD("Number of Lines")

          ,
      INTEGER_FIELD("Line", 2),
      LOOKUP_FIELD("Waveform", 3, WAVEFORM),
      RESERVED_FIELD(3),
      VOLTAGE_FIELD("Voltage", 0.01),
      {"Current", BYTES(2), 0.1, false, "A", ""},
      FREQUENCY_FIELD("Frequency", 0.01),
      {"Breaker Size", BYTES(2), 0.1, false, "A", ""},
      {"Real Power", BYTES(4), RES_INTEGER, false, "W", ""},
      {"Reactive Power", BYTES(4), RES_INTEGER, false, "VAR", ""},
      POWER_FACTOR_FIELD,
      {0}}}

    /* http://www.maretron.com/support/manuals/TLA100UM_1.2.pdf */
    /* Observed from EP65R */
    ,
    {"Fluid Level",
     127505,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {SIMPLE_FIELD("Instance", 4),
      LOOKUP_FIELD("Type", 4, TANK_TYPE),
      PERCENTAGE_FIELD("Level"),
      {"Capacity", BYTES(4), 0.1, false, "L", ""},
      RESERVED_FIELD(BYTES(1)),
      {0}}}

    ,
    {"DC Detailed Status",
     127506,
     PACKET_COMPLETE,
     PACKET_FAST,
     11,
     0,
     {ONE_BYTE_FIELD("SID"),
      INSTANCE_FIELD,
      LOOKUP_FIELD("DC Type", BYTES(1), DC_SOURCE),
      ONE_BYTE_FIELD("State of Charge"),
      ONE_BYTE_FIELD("State of Health"),
      SIMPLE_FIELD("Time Remaining", BYTES(2)),
      VOLTAGE_FIELD("Ripple Voltage", 0.01),
      {"Amp Hours", BYTES(2), 3600, false, "C", ""},
      {0}}}

    // http://www.osukl.com/wp-content/uploads/2015/04/3155-UM.pdf
    ,
    {"Charger Status",
     127507,
     PACKET_COMPLETE,
     PACKET_FAST,
     6,
     0,
     {INSTANCE_FIELD,
      ONE_BYTE_FIELD("Battery Instance"),
      LOOKUP_FIELD("Operating State", 4, CHARGER_STATE),
      LOOKUP_FIELD("Charge Mode", 4, CHARGER_MODE),
      LOOKUP_FIELD("Enabled", 2, OFF_ON),
      LOOKUP_FIELD("Equalization Pending", 2, OFF_ON),
      RESERVED_FIELD(4),
      SIMPLE_FIELD("Equalization Time Remaining", BYTES(2)),
      {0}}}

    ,
    {"Battery Status",
     127508,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {INSTANCE_FIELD,
      VOLTAGE_FIELD("Voltage", 0.01),
      {"Current", BYTES(2), 0.1, true, "A", ""},
      TEMPERATURE_FIELD("Temperature"),
      ONE_BYTE_FIELD("SID"),
      {0}}}

    /* https://www.nmea.org/Assets/20140102%20nmea-2000-127509%20pgn%20corrigendum.pdf */
    ,
    {"Inverter Status",
     127509,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     4,
     0,
     {INSTANCE_FIELD,
      ONE_BYTE_FIELD("AC Instance"),
      ONE_BYTE_FIELD("DC Instance"),
      LOOKUP_FIELD("Operating State", 4, INVERTER_STATE),
      LOOKUP_FIELD("Inverter Enable", 2, OFF_ON),
      RESERVED_FIELD(2),
      {0}}}

    ,
    {"Charger Configuration Status",
     127510,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     13,
     0,
     {INSTANCE_FIELD,
      ONE_BYTE_FIELD("Battery Instance"),
      SIMPLE_FIELD("Charger Enable/Disable", 2),
      RESERVED_FIELD(6),
      {"Charge Current Limit", BYTES(2), 0.1, false, "A", ""},
      ONE_BYTE_FIELD("Charging Algorithm"),
      ONE_BYTE_FIELD("Charger Mode"),
      TEMPERATURE_FIELD("Estimated Temperature"),
      SIMPLE_FIELD("Equalize One Time Enable/Disable", 4),
      SIMPLE_FIELD("Over Charge Enable/Disable", 4),
      SIMPLE_FIELD("Equalize Time", BYTES(2)),
      {0}}}

    ,
    {"Inverter Configuration Status",
     127511,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {INSTANCE_FIELD,
      ONE_BYTE_FIELD("AC Instance"),
      ONE_BYTE_FIELD("DC Instance"),
      SIMPLE_FIELD("Inverter Enable/Disable", 2),
      ONE_BYTE_FIELD("Inverter Mode"),
      ONE_BYTE_FIELD("Load Sense Enable/Disable"),
      ONE_BYTE_FIELD("Load Sense Power Threshold"),
      ONE_BYTE_FIELD("Load Sense Interval"),
      {0}}}

    ,
    {"AGS Configuration Status",
     127512,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {INSTANCE_FIELD, ONE_BYTE_FIELD("Generator Instance"), ONE_BYTE_FIELD("AGS Mode"), {0}}}

    /* #143, @ksltd writes that it is definitely 10 bytes and that
     * nominal voltage is a lookup, Peukert Exponent and Charge Efficiency
     * are 8 bits. It follows that Temperature Coefficient must be 8 bits
     * as well to fit in 10 bytes.
     *
     * I'm now actually following https://github.com/ttlappalainen/NMEA2000/
     * The Supports Equalization is 2 bits, Battery Type, Chemistry and
     * Nominal voltage are all 4 bits. Capacity and Peukert are both 2 bytes.
     * but this only adds up to 8 bytes... Maybe the 10 was as this is transmitted
     * as FAST pgn?
     */
    ,
    {"Battery Configuration Status",
     127513,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     10,
     0,
     {INSTANCE_FIELD,
      LOOKUP_FIELD("Battery Type", 4, BATTERY_TYPE),
      LOOKUP_FIELD("Supports Equalization", 2, YES_NO),
      RESERVED_FIELD(2),
      LOOKUP_FIELD("Nominal Voltage", 4, BATTERY_VOLTAGE),
      LOOKUP_FIELD("Chemistry", 4, BATTERY_CHEMISTRY),
      {"Capacity", BYTES(2), 1, false, "C", ""},
      SHORT_PERCENTAGE_FIELD("Temperature Coefficient"),
      {"Peukert Exponent", BYTES(1), 0.002, false, 0, "Possibly in Excess-1 notation"},
      SHORT_PERCENTAGE_FIELD("Charge Efficiency Factor"),
      {0}}}

    ,
    {"AGS Status",
     127514,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {INSTANCE_FIELD,
      ONE_BYTE_FIELD("Generator Instance"),
      ONE_BYTE_FIELD("AGS Operating State"),
      ONE_BYTE_FIELD("Generator State"),
      ONE_BYTE_FIELD("Generator On Reason"),
      ONE_BYTE_FIELD("Generator Off Reason"),
      {0}}}

    ,
    {"AC Power / Current - Phase A",
     127744,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("Connection Number"),
      {"AC RMS Current", BYTES(2), 0.1, false, "A", ""},
      {"Power", BYTES(4), 1, true, "W", ""},
      {0}}}

    ,
    {"AC Power / Current - Phase B",
     127745,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("Connection Number"),
      {"AC RMS Current", BYTES(2), 0.1, false, "A", ""},
      {"Power", BYTES(4), 1, true, "W", ""},
      {0}}}

    ,
    {"AC Power / Current - Phase C",
     127746,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("Connection Number"),
      {"AC RMS Current", BYTES(2), 0.1, false, "A", ""},
      {"Power", BYTES(4), 1, true, "W", ""},
      {0}}}

    ,
    {"Converter Status",
     127750,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {BINARY_FIELD("SID", BYTES(1), ""),
      ONE_BYTE_FIELD("Connection Number"),
      LOOKUP_FIELD("Operating State", BYTES(1), CONVERTER_STATE),
      LOOKUP_FIELD("Temperature State", 2, GOOD_WARNING_ERROR),
      LOOKUP_FIELD("Overload State", 2, GOOD_WARNING_ERROR),
      LOOKUP_FIELD("Low DC Voltage State", 2, GOOD_WARNING_ERROR),
      LOOKUP_FIELD("Ripple State", 2, GOOD_WARNING_ERROR),
      RESERVED_FIELD(BYTES(4)),
      {0}}}

    ,
    {"DC Voltage/Current",
     127751,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {BINARY_FIELD("SID", BYTES(1), ""),
      ONE_BYTE_FIELD("Connection Number"),
      VOLTAGE_FIELD("DC Voltage", 0.1),
      {"DC Current", BYTES(3), 0.01, true, "A", ""},
      RESERVED_FIELD(BYTES(1)),
      {0}}}

    /* https://www.nmea.org/Assets/20170204%20nmea%202000%20leeway%20pgn%20final.pdf */
    ,
    {"Leeway Angle",
     128000,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"), ANGLE_REL_FIELD("Leeway Angle", ""), RESERVED_FIELD(BYTES(5)), {0}}}

    ,
    {"Thruster Control Status",
     128006,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("Identifier"),
      LOOKUP_FIELD("Direction Control", 4, THRUSTER_DIRECTION_CONTROL),
      LOOKUP_FIELD("Power Enabled", 2, OFF_ON),
      LOOKUP_FIELD("Retract Control", 2, THRUSTER_RETRACT_CONTROL),
      SHORT_PERCENTAGE_FIELD("Speed Control"),
      LOOKUP_BITFIELD("Control Events", BYTES(1), THRUSTER_CONTROL_EVENTS),
      {"Command Timeout", BYTES(1), 1e-3, false, 0, ""},
      ANGLE_POS_FIELD("Azimuth Control", ""),
      {0}}}

    ,
    {"Thruster Information",
     128007,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("Identifier"),
      LOOKUP_FIELD("Motor Type", 4, THRUSTER_MOTOR_TYPE),
      RESERVED_FIELD(4),
      {"Power Rating", BYTES(2), 1, false, "W", ""},
      TEMPERATURE_FIELD("Maximum Temperature Rating"),
      {"Maximum Rotational Speed", BYTES(2), 0.25, false, "rpm", ""},
      {0}}}

    ,
    {"Thruster Motor Status",
     128008,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("Identifier"),
      LOOKUP_BITFIELD("Motor Events", BYTES(1), THRUSTER_MOTOR_EVENTS),
      {"Current", BYTES(1), 1, false, "A", ""},
      TEMPERATURE_FIELD("Temperature"),
      {"Operating Time", BYTES(2), 1, false, "minutes", ""},
      {0}}}

    /* http://www.maretron.com/support/manuals/DST100UM_1.2.pdf */
    ,
    {"Speed",
     128259,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      SPEED_FIELD("Speed Water Referenced"),
      SPEED_FIELD("Speed Ground Referenced"),
      LOOKUP_FIELD("Speed Water Referenced Type", BYTES(1), WATER_REFERENCE),
      SIMPLE_FIELD("Speed Direction", 4),
      RESERVED_FIELD(12),
      {0}}}

    /* http://www.maretron.com/support/manuals/DST100UM_1.2.pdf */
    ,
    {"Water Depth",
     128267,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      LENGTH_FIELD("Depth", BYTES(4), 0.01, "Depth below transducer"),
      DISTANCE_FIELD("Offset", BYTES(2), 0.001, "Distance between transducer and surface (positive) or keel (negative)"),
      LENGTH_FIELD("Range", BYTES(1), 10, "Max measurement range"),
      {0}}}

    /* http://www.nmea.org/Assets/nmea-2000-digital-interface-white-paper.pdf */
    ,
    {"Distance Log",
     128275,
     PACKET_COMPLETE,
     PACKET_FAST,
     14,
     0,
     {{"Date", BYTES(2), RES_DATE, false, "days", "Timestamp of last reset in Days since January 1, 1970"},
      TIME_FIELD("Time", "Timestamp of last reset Seconds since midnight"),
      LENGTH_FIELD("Log", BYTES(4), 1, "Total cumulative distance"),
      LENGTH_FIELD("Trip Log", BYTES(4), 1, "Distance since last reset"),
      {0}}}

    ,
    {"Tracked Target Data",
     128520,
     PACKET_NOT_SEEN,
     PACKET_FAST,
     27,
     0,
     {ONE_BYTE_FIELD("SID"),
      SIMPLE_DESC_FIELD("Target ID #", BYTES(1), "Number of route, waypoint, event, mark, etc."),
      LOOKUP_FIELD("Track Status", 2, TRACKING),
      LOOKUP_FIELD("Reported Target", 1, YES_NO),
      LOOKUP_FIELD("Target Acquisition", 1, TARGET_ACQUISITION),
      LOOKUP_FIELD("Bearing Reference", 2, DIRECTION_REFERENCE),
      RESERVED_FIELD(2),
      ANGLE_POS_FIELD("Bearing", ""),
      HIRES_LENGTH_FIELD("Distance", 0.001),
      ANGLE_POS_FIELD("Course", ""),
      SPEED_FIELD("Speed"),
      HIRES_LENGTH_FIELD("CPA", 0.01),
      {"TCPA", BYTES(4), 0.001, false, "s", "negative = time elapsed since event, positive = time to go"},
      TIME_FIELD("UTC of Fix", "Seconds since midnight"),
      ASCII_FIELD("Name", BYTES(255)),
      {0}}}

    ,
    /* https://www.nmea.org/Assets/20190613%20windlass%20amendment,%20128776,%20128777,%20128778.pdf */
    {"Windlass Control Status",
     128776,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     7,
     0,
     {ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("Windlass ID"),
      LOOKUP_FIELD("Windlass Direction Control", 2, WINDLASS_DIRECTION),
      LOOKUP_FIELD("Anchor Docking Control", 2, OFF_ON),
      LOOKUP_FIELD("Speed Control Type", 2, SPEED_TYPE),
      RESERVED_FIELD(2),
      BINARY_FIELD("Speed Control", BYTES(1), "0=Off,Single speed:1-100=On,Dual Speed:1-49=Slow/50-100=Fast,Proportional:10-100"),
      LOOKUP_FIELD("Power Enable", 2, OFF_ON),
      LOOKUP_FIELD("Mechanical Lock", 2, OFF_ON),
      LOOKUP_FIELD("Deck and Anchor Wash", 2, OFF_ON),
      LOOKUP_FIELD("Anchor Light", 2, OFF_ON),
      {"Command Timeout",
       BYTES(1),
       0.005,
       false,
       "s",
       "If timeout elapses the thruster stops operating and reverts to static mode"},
      LOOKUP_FIELD("Windlass Control Events", 4, WINDLASS_CONTROL),
      RESERVED_FIELD(4),
      {0}}}

    ,
    /* https://www.nmea.org/Assets/20190613%20windlass%20amendment,%20128776,%20128777,%20128778.pdf */
    {"Anchor Windlass Operating Status",
     128777,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("Windlass ID"),
      LOOKUP_FIELD("Windlass Direction Control", 2, WINDLASS_DIRECTION),
      LOOKUP_FIELD("Windlass Motion Status", 2, WINDLASS_MOTION),
      LOOKUP_FIELD("Rode Type Status", 2, RODE_TYPE),
      RESERVED_FIELD(2),
      DECIMETERS_FIELD("Rode Counter Value"),
      SPEED_FIELD("Windlass Line Speed"),
      LOOKUP_FIELD("Anchor Docking Status", 2, DOCKING_STATUS),
      LOOKUP_BITFIELD("Windlass Operating Events", 6, WINDLASS_OPERATION),
      {0}}}

    ,
    /* https://www.nmea.org/Assets/20190613%20windlass%20amendment,%20128776,%20128777,%20128778.pdf */
    {"Anchor Windlass Monitoring Status",
     128778,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("Windlass ID"),
      LOOKUP_BITFIELD("Windlass Monitoring Events", 8, WINDLASS_MONITORING),
      VOLTAGE_MEDIUM_FIELD("Controller voltage", 0.2),
      {"Motor current", BYTES(1), 1, false, "A", ""},
      ELAPSED_FIELD("Total Motor Time", BYTES(2), 60),
      RESERVED_FIELD(BYTES(1)),
      {0}}}

    ,
    {"Position, Rapid Update",
     129025,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""}, {"Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""}, {0}}}

    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    ,
    {"COG & SOG, Rapid Update",
     129026,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      LOOKUP_FIELD("COG Reference", 2, DIRECTION_REFERENCE),
      RESERVED_FIELD(6),
      ANGLE_POS_FIELD("COG", ""),
      SPEED_FIELD("SOG"),
      RESERVED_FIELD(BYTES(2)),
      {0}}}

    ,
    {"Position Delta, Rapid Update",
     129027,
     PACKET_NOT_SEEN,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      SIMPLE_FIELD("Time Delta", BYTES(2)),
      {"Latitude Delta", BYTES(2), 1, true, 0, ""},
      {"Longitude Delta", BYTES(2), 1, true, 0, ""},
      {0}}}

    ,
    {"Altitude Delta, Rapid Update",
     129028,
     PACKET_NOT_SEEN,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      {"Time Delta", BYTES(2), 1, true, 0, ""},
      SIMPLE_FIELD("GNSS Quality", 2),
      SIMPLE_FIELD("Direction", 2),
      RESERVED_FIELD(4),
      ANGLE_POS_FIELD("COG", ""),
      {"Altitude Delta", BYTES(2), 1, true, 0, ""},
      {0}}}

    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    ,
    {"GNSS Position Data",
     129029,
     PACKET_COMPLETE,
     PACKET_FAST,
     43,
     3,
     {ONE_BYTE_FIELD("SID"),
      {"Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      TIME_FIELD("Time", "Seconds since midnight"),
      {"Latitude", BYTES(8), RES_LATITUDE, true, "deg", ""},
      {"Longitude", BYTES(8), RES_LONGITUDE, true, "deg", ""},
      DISTANCE_FIELD("Altitude", BYTES(8), 1e-6, "Altitude referenced to WGS-84"),
      LOOKUP_FIELD("GNSS type", 4, GNS),
      LOOKUP_FIELD("Method", 4, GNS_METHOD),
      LOOKUP_FIELD("Integrity", 2, GNS_INTEGRITY),
      RESERVED_FIELD(6),
      SIMPLE_DESC_FIELD("Number of SVs", BYTES(1), "Number of satellites used in solution"),
      {"HDOP", BYTES(2), 0.01, true, 0, "Horizontal dilution of precision"},
      {"PDOP", BYTES(2), 0.01, true, 0, "Positional dilution of precision"},
      DISTANCE_FIELD("Geoidal Separation", BYTES(4), 0.01, "Geoidal Separation"),
      SIMPLE_DESC_FIELD("Reference Stations", BYTES(1), "Number of reference stations"),
      LOOKUP_FIELD("Reference Station Type", 4, GNS),
      {"Reference Station ID", 12, 1, false, ""},
      ELAPSED_FIELD("Age of DGNSS Corrections", BYTES(2), 0.01),
      {0}}}

    ,
    {"Time & Date",
     129033,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      TIME_FIELD("Time", "Seconds since midnight"),
      {"Local Offset", BYTES(2), RES_INTEGER, true, "minutes", "Minutes"},
      {0}}}

    ,
    {"AIS Class A Position Report",
     129038,
     PACKET_COMPLETE,
     PACKET_FAST,
     28,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      LOOKUP_FIELD("Position Accuracy", 1, POSITION_ACCURACY),
      LOOKUP_FIELD("RAIM", 1, RAIM_FLAG),
      LOOKUP_FIELD_DESC("Time Stamp", 6, TIME_STAMP, "0-59 = UTC second when the report was generated"),
      ANGLE_POS_FIELD("COG", ""),
      SPEED_FIELD("SOG"),
      {"Communication State",
       19,
       RES_BINARY,
       false,
       0,
       "Information used by the TDMA slot allocation algorithm and synchronization information"},
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      ANGLE_POS_FIELD("Heading", "True heading"),
      {"Rate of Turn", BYTES(2), RES_ROTATION, true, "rad/s", ""},
      LOOKUP_FIELD("Nav Status", 4, NAV_STATUS),
      LOOKUP_FIELD("Special Maneuver Indicator", 2, AIS_SPECIAL_MANEUVER),
      RESERVED_FIELD(2),
      BINARY_FIELD("AIS Spare", 3, ""),
      RESERVED_FIELD(5),
      INTEGER_FIELD("Sequence ID", BYTES(1)),
      {0}}}

    ,
    {"AIS Class B Position Report",
     129039,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x1a,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      LOOKUP_FIELD("Position Accuracy", 1, POSITION_ACCURACY),
      LOOKUP_FIELD("RAIM", 1, RAIM_FLAG),
      LOOKUP_FIELD("Time Stamp", 6, TIME_STAMP),
      ANGLE_POS_FIELD("COG", ""),
      SPEED_FIELD("SOG"),
      {"Communication State",
       19,
       RES_BINARY,
       false,
       0,
       "Information used by the TDMA slot allocation algorithm and synchronization information"},
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      ANGLE_POS_FIELD("Heading", "True heading"),
      ONE_BYTE_FIELD("Regional Application"),
      SIMPLE_FIELD("Regional Application", 2),
      LOOKUP_FIELD("Unit type", 1, AIS_TYPE),
      LOOKUP_FIELD_DESC("Integrated Display", 1, YES_NO, "Whether the unit can show messages 12 and 14"),
      LOOKUP_FIELD("DSC", 1, YES_NO),
      LOOKUP_FIELD("Band", 1, AIS_BAND),
      LOOKUP_FIELD("Can handle Msg 22", 1, YES_NO),
      LOOKUP_FIELD("AIS mode", 1, AIS_MODE),
      LOOKUP_FIELD("AIS communication state", 1, AIS_COMMUNICATION_STATE),
      {0}}}

    ,
    {"AIS Class B Extended Position Report",
     129040,
     PACKET_COMPLETE,
     PACKET_FAST,
     33,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      LOOKUP_FIELD("Position Accuracy", 1, POSITION_ACCURACY),
      LOOKUP_FIELD("AIS RAIM flag", 1, RAIM_FLAG),
      LOOKUP_FIELD("Time Stamp", 6, TIME_STAMP),
      ANGLE_POS_FIELD("COG", ""),
      SPEED_FIELD("SOG"),
      ONE_BYTE_FIELD("Regional Application"),
      SIMPLE_FIELD("Regional Application", 4),
      RESERVED_FIELD(4),
      LOOKUP_FIELD("Type of ship", BYTES(1), SHIP_TYPE),
      ANGLE_POS_FIELD("True Heading", ""),
      RESERVED_FIELD(4),
      LOOKUP_FIELD("GNSS type", 4, POSITION_FIX_DEVICE),
      DECIMETERS_FIELD("Length"),
      DECIMETERS_FIELD("Beam"),
      DECIMETERS_FIELD("Position reference from Starboard"),
      DECIMETERS_FIELD("Position reference from Bow"),
      ASCII_FIELD("Name", BYTES(20)),
      LOOKUP_FIELD("DTE", 1, AVAILABLE),
      LOOKUP_FIELD("AIS mode", 1, AIS_MODE),
      RESERVED_FIELD(4),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      {0}}}

    ,
    {"AIS Aids to Navigation (AtoN) Report",
     129041,
     PACKET_COMPLETE,
     PACKET_FAST,
     60,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      LOOKUP_FIELD("Position Accuracy", 1, POSITION_ACCURACY),
      LOOKUP_FIELD("AIS RAIM Flag", 1, RAIM_FLAG),
      LOOKUP_FIELD("Time Stamp", 6, TIME_STAMP),
      DECIMETERS_FIELD("Length/Diameter"),
      DECIMETERS_FIELD("Beam/Diameter"),
      DECIMETERS_FIELD("Position Reference from Starboard Edge"),
      DECIMETERS_FIELD("Position Reference from True North Facing Edge"),
      LOOKUP_FIELD("AtoN Type", 5, ATON_TYPE),
      LOOKUP_FIELD("Off Position Indicator", 1, YES_NO),
      LOOKUP_FIELD("Virtual AtoN Flag", 1, YES_NO),
      LOOKUP_FIELD("Assigned Mode Flag", 1, AIS_ASSIGNED_MODE),
      BINARY_FIELD("AIS Spare", 1, ""),
      LOOKUP_FIELD("Position Fixing Device Type", 4, POSITION_FIX_DEVICE),
      RESERVED_FIELD(3),
      BINARY_FIELD("AtoN Status", 8, "00000000 = default"),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(3),
      STRINGLAU_FIELD("AtoN Name"),
      {0}}}

    ,
    {"Datum",
     129044,
     PACKET_COMPLETE,
     PACKET_FAST,
     20,
     0,
     {ASCII_DESC_FIELD("Local Datum",
                       BYTES(4),
                       "defined in IHO Publication S-60, Appendices B and C. First three chars are datum ID as per IHO tables."
                       " Fourth char is local datum subdivision code."),
      {"Delta Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Delta Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      DISTANCE_FIELD("Delta Altitude", BYTES(4), 1e-6, ""),
      ASCII_DESC_FIELD("Reference Datum",
                       BYTES(4),
                       "defined in IHO Publication S-60, Appendices B and C."
                       " First three chars are datum ID as per IHO tables."
                       " Fourth char is local datum subdivision code."),
      {0}}}

    ,
    {"User Datum",
     129045,
     PACKET_COMPLETE,
     PACKET_FAST,
     37,
     0,
     {DISTANCE_FIELD("Delta X", BYTES(4), 0.01, "Delta shift in X axis from WGS 84"),
      DISTANCE_FIELD("Delta Y", BYTES(4), 0.01, "Delta shift in Y axis from WGS 84"),
      DISTANCE_FIELD("Delta Z", BYTES(4), 0.01, "Delta shift in Z axis from WGS 84"),
      {"Rotation in X",
       BYTES(4),
       RES_FLOAT,
       true,
       0,
       "Rotational shift in X axis from WGS 84. Rotations presented use the geodetic sign convention.  When looking along the "
       "positive axis towards the origin, counter-clockwise rotations are positive."},
      {"Rotation in Y",
       BYTES(4),
       RES_FLOAT,
       true,
       0,
       "Rotational shift in Y axis from WGS 84. Rotations presented use the geodetic sign convention.  When looking along the "
       "positive axis towards the origin, counter-clockwise rotations are positive."},
      {"Rotation in Z",
       BYTES(4),
       RES_FLOAT,
       true,
       0,
       "Rotational shift in Z axis from WGS 84. Rotations presented use the geodetic sign convention.  When looking along the "
       "positive axis towards the origin, counter-clockwise rotations are positive."},
      {"Scale", BYTES(4), RES_FLOAT, true, "ppm", "Scale factor expressed in parts-per-million"},
      DISTANCE_FIELD("Ellipsoid Semi-major Axis", BYTES(4), 0.01, "Semi-major axis (a) of the User Datum ellipsoid"),
      {"Ellipsoid Flattening Inverse", BYTES(4), RES_FLOAT, true, 0, "Flattening (1/f) of the User Datum ellipsoid"},
      ASCII_DESC_FIELD("Datum Name",
                       BYTES(4),
                       "4 character code from IHO Publication S-60,Appendices B and C."
                       " First three chars are datum ID as per IHO tables."
                       " Fourth char is local datum subdivision code."),
      {0}}}

    ,
    {"Cross Track Error",
     129283,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      LOOKUP_FIELD("XTE mode", 4, RESIDUAL_MODE),
      RESERVED_FIELD(2),
      LOOKUP_FIELD("Navigation Terminated", 2, YES_NO),
      DISTANCE_FIELD("XTE", BYTES(4), 0.01, ""),
      RESERVED_FIELD(BYTES(2)),
      {0}}}

    ,
    {"Navigation Data",
     129284,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x22,
     0,
     {ONE_BYTE_FIELD("SID"),
      HIRES_LENGTH_FIELD("Distance to Waypoint", 0.01),
      LOOKUP_FIELD("Course/Bearing reference", 2, DIRECTION_REFERENCE),
      LOOKUP_FIELD("Perpendicular Crossed", 2, YES_NO),
      LOOKUP_FIELD("Arrival Circle Entered", 2, YES_NO),
      LOOKUP_FIELD("Calculation Type", 2, BEARING_MODE),
      TIME_FIELD("ETA Time", "Seconds since midnight"),
      {"ETA Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      ANGLE_POS_FIELD("Bearing, Origin to Destination Waypoint", ""),
      ANGLE_POS_FIELD("Bearing, Position to Destination Waypoint", ""),
      SIMPLE_FIELD("Origin Waypoint Number", BYTES(4)),
      SIMPLE_FIELD("Destination Waypoint Number", BYTES(4)),
      {"Destination Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Destination Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      SPEED_REL_FIELD("Waypoint Closing Velocity"),
      {0}}}

    ,
    {"Navigation - Route/WP Information",
     129285,
     PACKET_COMPLETE,
     PACKET_FAST,
     233,
     4,
     {SIMPLE_FIELD("Start RPS#", BYTES(2)),
      SIMPLE_FIELD("nItems", BYTES(2)),
      SIMPLE_FIELD("Database ID", BYTES(2)),
      SIMPLE_FIELD("Route ID", BYTES(2)),
      SIMPLE_FIELD("Navigation direction in route", 2),
      SIMPLE_FIELD("Supplementary Route/WP data available", 2),
      RESERVED_FIELD(4),
      STRINGVAR_FIELD("Route Name"),
      RESERVED_FIELD(BYTES(1)),
      SIMPLE_FIELD("WP ID", BYTES(2)),
      STRINGVAR_FIELD("WP Name"),
      {"WP Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"WP Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {0}}}

    ,
    {"Set & Drift, Rapid Update",
     129291,
     PACKET_NOT_SEEN,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      LOOKUP_FIELD("Set Reference", 2, DIRECTION_REFERENCE),
      RESERVED_FIELD(6),
      ANGLE_POS_FIELD("Set", ""),
      SPEED_FIELD("Drift"),
      {0}}}

    ,
    {"Navigation - Route / Time to+from Mark",
     129301,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     10,
     0,
     {ONE_BYTE_FIELD("SID"),
      {"Time to mark", BYTES(4), 0.001, true, "s", "negative = elapsed since event, positive = time to go"},
      LOOKUP_FIELD("Mark Type", 4, MARK_TYPE),
      RESERVED_FIELD(4),
      SIMPLE_FIELD("Mark ID", BYTES(4)),
      {0}}}

    ,
    {"Bearing and Distance between two Marks",
     129302,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      {"Bearing Reference", 4, RES_LOOKUP, false, 0, ""},
      {"Calculation Type", 2, RES_LOOKUP, false, 0, ""},
      RESERVED_FIELD(2),
      ANGLE_POS_FIELD("Bearing, Origin to Destination", ""),
      HIRES_LENGTH_FIELD("Distance", 0.01),
      {"Origin Mark Type", 4, RES_LOOKUP, false, 0, ""},
      LOOKUP_FIELD("Destination Mark Type", 4, MARK_TYPE),
      SIMPLE_FIELD("Origin Mark ID", BYTES(4)),
      SIMPLE_FIELD("Destination Mark ID", BYTES(4)),
      {0}}}

    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    /* Haven't seen this yet (no way to send PGN 059904 yet) so lengths unknown */
    ,
    {"GNSS Control Status",
     129538,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     13,
     0,
     {SIMPLE_DESC_FIELD("SV Elevation Mask", BYTES(2), "Will not use SV below this elevation"),
      {"PDOP Mask", BYTES(2), 0.01, false, 0, "Will not report position above this PDOP"},
      {"PDOP Switch", BYTES(2), 0.01, false, 0, "Will report 2D position above this PDOP"},
      {"SNR Mask", BYTES(2), 0.01, false, 0, "Will not use SV below this SNR"},
      LOOKUP_FIELD("GNSS Mode (desired)", 3, GNSS_MODE),
      LOOKUP_FIELD("DGNSS Mode (desired)", 3, DGNSS_MODE),
      SIMPLE_FIELD("Position/Velocity Filter", 2),
      SIMPLE_FIELD("Max Correction Age", BYTES(2)),
      LENGTH_FIELD("Antenna Altitude for 2D Mode", BYTES(2), 0.01, ""),
      LOOKUP_FIELD("Use Antenna Altitude for 2D Mode", 2, YES_NO),
      {0}}}

    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    ,
    {"GNSS DOPs",
     129539,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      LOOKUP_FIELD("Desired Mode", 3, GNSS_MODE),
      LOOKUP_FIELD("Actual Mode", 3, GNSS_MODE),
      RESERVED_FIELD(2),
      {"HDOP", BYTES(2), 0.01, true, 0, "Horizontal dilution of precision"},
      {"VDOP", BYTES(2), 0.01, true, 0, "Vertical dilution of precision"},
      {"TDOP", BYTES(2), 0.01, true, 0, "Time dilution of precision"},
      {0}}}

    ,
    {"GNSS Sats in View",
     129540,
     PACKET_COMPLETE,
     PACKET_FAST,
     233,
     7,
     {ONE_BYTE_FIELD("SID"),
      INTEGER_DESC_FIELD("Mode", 2, "Unknown lookup values"),
      RESERVED_FIELD(6),
      ONE_BYTE_FIELD("Sats in View"),
      ONE_BYTE_FIELD("PRN"),
      ANGLE_POS_FIELD("Elevation", ""),
      ANGLE_POS_FIELD("Azimuth", ""),
      {"SNR", BYTES(2), 0.01, false, "dB", ""},
      {"Range residuals", BYTES(4), 1, true, 0, ""},
      LOOKUP_FIELD("Status", 4, SATELLITE_STATUS),
      RESERVED_FIELD(4),
      {0}}}

    ,
    {"GPS Almanac Data",
     129541,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     26,
     0,
     {INTEGER_FIELD("PRN", BYTES(1)),
      INTEGER_FIELD("GPS Week number", BYTES(2)),
      BINARY_FIELD("SV Health Bits", BYTES(1), ""),
      {"Eccentricity", BYTES(2), 1e-21, false, "m/m", ""},
      ELAPSED_FIELD("Almanac Reference Time", BYTES(1), 1e12),
      {"Inclination Angle", BYTES(2), 1e-19, true, "semi-circle", ""},
      {"Rate of Right Ascension", BYTES(2), 1e-38, true, "semi-circle/s", ""},
      {"Root of Semi-major Axis", BYTES(3), 1e-11, false, "sqrt(m)", ""},
      {"Argument of Perigee", BYTES(3), 1e-23, true, "semi-circle", ""},
      {"Longitude of Ascension Node", BYTES(3), 1e-23, true, "semi-circle", ""},
      {"Mean Anomaly", BYTES(3), 1e-23, true, "semi-circle", ""},
      {"Clock Parameter 1", 11, 1e-20, true, "s", ""},
      {"Clock Parameter 2", 11, 1e-38, true, "s/s", ""},
      RESERVED_FIELD(2),
      {0}}}

    ,
    {"GNSS Pseudorange Noise Statistics",
     129542,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     9,
     0,
     {ONE_BYTE_FIELD("SID"),
      SIMPLE_FIELD("RMS of Position Uncertainty", BYTES(2)),
      ONE_BYTE_FIELD("STD of Major axis"),
      ONE_BYTE_FIELD("STD of Minor axis"),
      ONE_BYTE_FIELD("Orientation of Major axis"),
      ONE_BYTE_FIELD("STD of Lat Error"),
      ONE_BYTE_FIELD("STD of Lon Error"),
      ONE_BYTE_FIELD("STD of Alt Error"),
      {0}}}

    ,
    {"GNSS RAIM Output",
     129545,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {ONE_BYTE_FIELD("SID"),
      SIMPLE_FIELD("Integrity flag", 4),
      RESERVED_FIELD(4),
      ONE_BYTE_FIELD("Latitude expected error"),
      ONE_BYTE_FIELD("Longitude expected error"),
      ONE_BYTE_FIELD("Altitude expected error"),
      ONE_BYTE_FIELD("SV ID of most likely failed sat"),
      ONE_BYTE_FIELD("Probability of missed detection"),
      ONE_BYTE_FIELD("Estimate of pseudorange bias"),
      ONE_BYTE_FIELD("Std Deviation of bias"),
      {0}}}

    ,
    {"GNSS RAIM Settings",
     129546,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("Radial Position Error Maximum Threshold"),
      ONE_BYTE_FIELD("Probability of False Alarm"),
      ONE_BYTE_FIELD("Probability of Missed Detection"),
      ONE_BYTE_FIELD("Pseudorange Residual Filtering Time Constant"),
      {0}}}

    ,
    {"GNSS Pseudorange Error Statistics",
     129547,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     9,
     0,
     {ONE_BYTE_FIELD("SID"),
      SIMPLE_FIELD("RMS Std Dev of Range Inputs", BYTES(2)),
      ONE_BYTE_FIELD("Std Dev of Major error ellipse"),
      ONE_BYTE_FIELD("Std Dev of Minor error ellipse"),
      ONE_BYTE_FIELD("Orientation of error ellipse"),
      ONE_BYTE_FIELD("Std Dev Lat Error"),
      ONE_BYTE_FIELD("Std Dev Lon Error"),
      ONE_BYTE_FIELD("Std Dev Alt Error"),
      {0}}}

    ,
    {"DGNSS Corrections",
     129549,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     13,
     0,
     {ONE_BYTE_FIELD("SID"),
      SIMPLE_FIELD("Reference Station ID", BYTES(2)),
      SIMPLE_FIELD("Reference Station Type", BYTES(2)),
      ONE_BYTE_FIELD("Time of corrections"),
      ONE_BYTE_FIELD("Station Health"),
      BINARY_FIELD("Reserved Bits", BYTES(1), ""),
      ONE_BYTE_FIELD("Satellite ID"),
      ONE_BYTE_FIELD("PRC"),
      ONE_BYTE_FIELD("RRC"),
      ONE_BYTE_FIELD("UDRE"),
      ONE_BYTE_FIELD("IOD"),
      {0}}}

    ,
    {"GNSS Differential Correction Receiver Interface",
     129550,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {ONE_BYTE_FIELD("Channel"),
      ONE_BYTE_FIELD("Frequency"),
      ONE_BYTE_FIELD("Serial Interface Bit Rate"),
      ONE_BYTE_FIELD("Serial Interface Detection Mode"),
      ONE_BYTE_FIELD("Differential Source"),
      ONE_BYTE_FIELD("Differential Operation Mode"),
      {0}}}

    ,
    {"GNSS Differential Correction Receiver Signal",
     129551,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("Channel"),
      ONE_BYTE_FIELD("Signal Strength"),
      ONE_BYTE_FIELD("Signal SNR"),
      ONE_BYTE_FIELD("Frequency"),
      ONE_BYTE_FIELD("Station Type"),
      ONE_BYTE_FIELD("Station ID"),
      ONE_BYTE_FIELD("Differential Signal Bit Rate"),
      ONE_BYTE_FIELD("Differential Signal Detection Mode"),
      ONE_BYTE_FIELD("Used as Correction Source"),
      RESERVED_FIELD(BYTES(1)),
      ONE_BYTE_FIELD("Differential Source"),
      ONE_BYTE_FIELD("Time since Last Sat Differential Sync"),
      ONE_BYTE_FIELD("Satellite Service ID No."),
      {0}}}

    ,
    {"GLONASS Almanac Data",
     129556,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {ONE_BYTE_FIELD("PRN"),
      ONE_BYTE_FIELD("NA"),
      ONE_BYTE_FIELD("CnA"),
      ONE_BYTE_FIELD("HnA"),
      ONE_BYTE_FIELD("(epsilon)nA"),
      ONE_BYTE_FIELD("(deltaTnA)DOT"),
      ONE_BYTE_FIELD("(omega)nA"),
      ONE_BYTE_FIELD("(delta)TnA"),
      ONE_BYTE_FIELD("tnA"),
      ONE_BYTE_FIELD("(lambda)nA"),
      ONE_BYTE_FIELD("(delta)inA"),
      ONE_BYTE_FIELD("tcA"),
      ONE_BYTE_FIELD("tnA"),
      {0}}}

    ,
    {"AIS DGNSS Broadcast Binary Message",
     129792,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      SIMPLE_FIELD("Repeat Indicator", 2),
      {"Source ID", BYTES(4), 1, false, "MMSI", ""},
      BINARY_FIELD("NMEA 2000 Reserved", BYTES(1), ""),
      ONE_BYTE_FIELD("AIS Transceiver Information"),
      ONE_BYTE_FIELD("Spare"),
      SIMPLE_FIELD("Longitude", BYTES(4)),
      SIMPLE_FIELD("Latitude", BYTES(4)),
      BINARY_FIELD("NMEA 2000 Reserved", BYTES(1), ""),
      ONE_BYTE_FIELD("Spare"),
      ONE_BYTE_FIELD("Number of Bits in Binary Data Field"),
      BINARY_FIELD("Binary Data", BYTES(8), ""),
      {0}}}

    ,
    {"AIS UTC and Date Report",
     129793,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x1a,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      LOOKUP_FIELD("Position Accuracy", 1, POSITION_ACCURACY),
      LOOKUP_FIELD("RAIM", 1, RAIM_FLAG),
      BINARY_FIELD("Reserved", 6, "NMEA reserved to align next data on byte boundary"),
      TIME_FIELD("Position Time", "Seconds since midnight"),
      {"Communication State",
       19,
       RES_BINARY,
       false,
       0,
       "Information used by the TDMA slot allocation algorithm and synchronization information"},
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      {"Position Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      BINARY_FIELD("Reserved", 4, "NMEA reserved to align next data on byte boundary"),
      LOOKUP_FIELD("GNSS type", 4, POSITION_FIX_DEVICE),
      BINARY_FIELD("Spare", BYTES(1), ""),
      {0}}}

    /* http://www.navcen.uscg.gov/enav/ais/AIS_messages.htm */
    ,
    {"AIS Class A Static and Voyage Related Data",
     129794,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x18,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat indicator", 2, REPEAT_INDICATOR),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      INTEGER_DESC_FIELD("IMO number", BYTES(4), ",0=unavailable"),
      ASCII_FIELD("Callsign", BYTES(7)),
      ASCII_FIELD("Name", BYTES(20)),
      LOOKUP_FIELD("Type of ship", BYTES(1), SHIP_TYPE),
      DECIMETERS_FIELD("Length"),
      DECIMETERS_FIELD("Beam"),
      DECIMETERS_FIELD("Position reference from Starboard"),
      DECIMETERS_FIELD("Position reference from Bow"),
      {"ETA Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      TIME_FIELD("ETA Time", "Seconds since midnight"),
      LENGTH_FIELD("Draft", BYTES(2), 0.01, ""),
      ASCII_FIELD("Destination", BYTES(20)),
      LOOKUP_FIELD("AIS version indicator", 2, AIS_VERSION),
      LOOKUP_FIELD("GNSS type", 4, POSITION_FIX_DEVICE),
      LOOKUP_FIELD("DTE", 1, AVAILABLE),
      RESERVED_FIELD(1),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      {0}}}

    ,
    {"AIS Addressed Binary Message",
     129795,
     PACKET_COMPLETE,
     PACKET_FAST,
     13,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      {"Source ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      RESERVED_FIELD(1),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      SIMPLE_FIELD("Sequence Number", 2),
      {"Destination ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      RESERVED_FIELD(6),
      SIMPLE_FIELD("Retransmit flag", 1),
      RESERVED_FIELD(1),
      INTEGER_FIELD("Number of Bits in Binary Data Field", BYTES(2)),
      BINARY_FIELD("Binary Data", BYTES(8), ""),
      {0}}}

    ,
    {"AIS Acknowledge",
     129796,
     PACKET_COMPLETE,
     PACKET_FAST,
     12,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      {"Source ID", BYTES(4), 1, false, "MMSI", ""},
      RESERVED_FIELD(1),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(2),
      SIMPLE_FIELD("Destination ID #1", BYTES(4)),
      BINARY_FIELD("Sequence Number for ID 1", 2, "reserved"),
      RESERVED_FIELD(6),
      BINARY_FIELD("Sequence Number for ID n", 2, "reserved"),
      {0}}}

    ,
    {"AIS Binary Broadcast Message",
     129797,
     PACKET_COMPLETE,
     PACKET_FAST,
     233,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      SIMPLE_FIELD("Source ID", BYTES(4)),
      RESERVED_FIELD(1),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(2),
      SIMPLE_FIELD("Number of Bits in Binary Data Field", BYTES(2)),
      BINARY_FIELD("Binary Data", BYTES(255), ""),
      {0}}}

    ,
    {"AIS SAR Aircraft Position Report",
     129798,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat indicator", 2, REPEAT_INDICATOR),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      LOOKUP_FIELD("Position Accuracy", 1, POSITION_ACCURACY),
      LOOKUP_FIELD("RAIM", 1, RAIM_FLAG),
      LOOKUP_FIELD("Time Stamp", 6, TIME_STAMP),
      ANGLE_POS_FIELD("COG", ""),
      {"SOG", BYTES(2), 0.1, false, "m/s", ""},
      {"Communication State",
       19,
       RES_BINARY,
       false,
       0,
       "Information used by the TDMA slot allocation algorithm and synchronization information"},
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      DISTANCE_FIELD("Altitude", BYTES(8), 1e-6, ""),
      BINARY_FIELD("Reserved for Regional Applications", BYTES(1), ""),
      LOOKUP_FIELD("DTE", 1, AVAILABLE),
      RESERVED_FIELD(7),
      {0}}}

    ,
    {"Radio Frequency/Mode/Power",
     129799,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {RADIO_FREQUENCY_FIELD("Rx Frequency", 10),
      RADIO_FREQUENCY_FIELD("Tx Frequency", 10),
      ONE_BYTE_FIELD("Radio Channel"),
      ONE_BYTE_FIELD("Tx Power"),
      ONE_BYTE_FIELD("Mode"),
      ONE_BYTE_FIELD("Channel Bandwidth"),
      {0}}}

    ,
    {"AIS UTC/Date Inquiry",
     129800,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      SIMPLE_FIELD("Source ID", 30),
      RESERVED_FIELD(2),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(3),
      SIMPLE_FIELD("Destination ID", 30),
      RESERVED_FIELD(2),
      {0}}}

    ,
    {"AIS Addressed Safety Related Message",
     129801,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     12,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      {"Source ID", 30, 1, false, "MMSI", ""},
      RESERVED_FIELD(2),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      SIMPLE_FIELD("Sequence Number", 2),
      {"Destination ID", 30, 1, false, "MMSI", ""},
      RESERVED_FIELD(2),
      SIMPLE_FIELD("Retransmit flag", 1),
      RESERVED_FIELD(7),
      ASCII_FIELD("Safety Related Text", BYTES(255)),
      {0}}}

    ,
    {"AIS Safety Related Broadcast Message",
     129802,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      INTEGER_FIELD("Source ID", 30),
      RESERVED_FIELD(2),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(3),
      ASCII_FIELD("Safety Related Text", BYTES(36)),
      {0}}}

    ,
    {"AIS Interrogation",
     129803,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     8,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      INTEGER_FIELD("Source ID", 30),
      RESERVED_FIELD(2),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(3),
      INTEGER_FIELD("Destination ID", 30),
      RESERVED_FIELD(2),
      INTEGER_FIELD("Message ID A", BYTES(1)),
      INTEGER_FIELD("Slot Offset A", 14),
      RESERVED_FIELD(2),
      INTEGER_FIELD("Message ID B", BYTES(1)),
      INTEGER_FIELD("Slot Offset B", 14),
      RESERVED_FIELD(2),
      {0}}}

    ,
    {"AIS Assignment Mode Command",
     129804,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     23,
     3,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      {"Source ID", 30, RES_INTEGER, false, "MMSI", ""},
      RESERVED_FIELD(2),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(3)

          ,
      {"Destination ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      INTEGER_FIELD("Offset", BYTES(2)),
      INTEGER_FIELD("Increment", BYTES(2)),
      {0}}}

    ,
    {"AIS Data Link Management Message",
     129805,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     4,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      INTEGER_FIELD("Source ID", 30),
      RESERVED_FIELD(2),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(3)

          ,
      INTEGER_FIELD("Offset", 10),
      INTEGER_FIELD("Number of Slots", BYTES(1)),
      INTEGER_FIELD("Timeout", BYTES(1)),
      INTEGER_FIELD("Increment", BYTES(1)),
      {0}}}

    ,
    {"AIS Channel Management",
     129806,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      INTEGER_FIELD("Source ID", 30),
      RESERVED_FIELD(2),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(3),
      SIMPLE_FIELD("Channel A", 7),
      SIMPLE_FIELD("Channel B", 7),
      RESERVED_FIELD(2),
      SIMPLE_DESC_FIELD("Power", BYTES(1), "reserved"),
      INTEGER_FIELD("Tx/Rx Mode", BYTES(1)),
      {"North East Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"North East Latitude Corner 1", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"South West Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"South West Latitude Corner 2", BYTES(4), RES_LATITUDE, true, "deg", ""},
      RESERVED_FIELD(6),
      SIMPLE_FIELD("Addressed or Broadcast Message Indicator", 2),
      INTEGER_FIELD("Channel A Bandwidth", 7),
      INTEGER_FIELD("Channel B Bandwidth", 7),
      RESERVED_FIELD(2),
      ONE_BYTE_FIELD("Transitional Zone Size"),
      {0}}}

    ,
    {"AIS Class B Group Assignment",
     129807,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat Indicator", 2, REPEAT_INDICATOR),
      INTEGER_FIELD("Source ID", 30),
      RESERVED_FIELD(2),
      INTEGER_FIELD("Tx/Rx Mode", 2),
      RESERVED_FIELD(6),
      {"North East Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"North East Latitude Corner 1", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"South West Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"South West Latitude Corner 2", BYTES(4), RES_LATITUDE, true, "deg", ""},
      SIMPLE_FIELD("Station Type", 6),
      RESERVED_FIELD(2),
      SIMPLE_FIELD("Ship and Cargo Filter", 6),
      RESERVED_FIELD(2),
      SIMPLE_FIELD("Reporting Interval", BYTES(2)),
      SIMPLE_FIELD("Quiet Time", BYTES(2)),
      {0}}}

    /* http://www.nmea.org/Assets/2000_20150328%20dsc%20technical%20corrigendum%20database%20version%202.100.pdf */
    /* This is like the worst PGN ever.
     * 1. The "Nature of Distress or 1st Telecommand' field meaning depends on the 'DSC Category'.
     * 2. The "Message address" (the 'to' field) meaning depends on the 'DSC format'.
     * 3. Field 12 'MMSI of ship in destress' may have multiple interpretations.
     * 4. Field 22 'DSC expansion field data' depends on field 21 for its meaning.
     * 5. It contains a variable length field 'Telephone number', which means that bit offsets for subsequent fields
     *    depend on this field's length.
     *
     * We solve #1 here by having two definitions.
     */

    ,
    {"DSC Distress Call Information",
     129808,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {LOOKUP_FIELD("DSC Format", BYTES(1), DSC_FORMAT),
      MATCH_FIELD("DSC Category", BYTES(1), 112, "Distress"),
      {"DSC Message Address", BYTES(5), RES_DECIMAL, false, 0, "MMSI, Geographic Area or blank"},
      LOOKUP_FIELD("Nature of Distress", BYTES(1), DSC_NATURE),
      LOOKUP_FIELD("Subsequent Communication Mode or 2nd Telecommand", BYTES(1), DSC_SECOND_TELECOMMAND),
      ASCII_FIELD("Proposed Rx Frequency/Channel", BYTES(6)),
      ASCII_FIELD("Proposed Tx Frequency/Channel", BYTES(6)),
      STRINGLAU_FIELD("Telephone Number"),
      {"Latitude of Vessel Reported",
       BYTES(4),
       RES_LATITUDE,
       true,
       "deg",
       "offset depends on previous field, as do all following fields"},
      {"Longitude of Vessel Reported", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      TIME_FIELD("Time of Position", "Seconds since midnight"),
      {"MMSI of Ship In Distress", BYTES(5), RES_DECIMAL, false, "MMSI", ""},
      ONE_BYTE_FIELD("DSC EOS Symbol"),
      LOOKUP_FIELD("Expansion Enabled", 2, YES_NO),
      RESERVED_FIELD(6),
      ASCII_FIELD("Calling Rx Frequency/Channel", BYTES(6)),
      ASCII_FIELD("Calling Tx Frequency/Channel", BYTES(6)),
      TIME_FIELD("Time of Receipt", "Seconds since midnight"),
      {"Date of Receipt", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      SIMPLE_FIELD("DSC Equipment Assigned Message ID", BYTES(2)),
      LOOKUP_FIELD("DSC Expansion Field Symbol", BYTES(1), DSC_EXPANSION_DATA),
      STRINGLAU_FIELD("DSC Expansion Field Data"),
      {0}}}

    ,
    {"DSC Call Information",
     129808,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {LOOKUP_FIELD("DSC Format Symbol", BYTES(1), DSC_FORMAT),
      LOOKUP_FIELD("DSC Category Symbol", BYTES(1), DSC_CATEGORY),
      {"DSC Message Address", BYTES(5), RES_DECIMAL, false, 0, "MMSI, Geographic Area or blank"},
      LOOKUP_FIELD("1st Telecommand", BYTES(1), DSC_FIRST_TELECOMMAND),
      LOOKUP_FIELD("Subsequent Communication Mode or 2nd Telecommand", BYTES(1), DSC_SECOND_TELECOMMAND),
      ASCII_FIELD("Proposed Rx Frequency/Channel", BYTES(6)),
      ASCII_FIELD("Proposed Tx Frequency/Channel", BYTES(6)),
      STRINGLAU_FIELD("Telephone Number"),
      {"Latitude of Vessel Reported",
       BYTES(4),
       RES_LATITUDE,
       true,
       "deg",
       "offset depends on previous field, as do all following fields"},
      {"Longitude of Vessel Reported", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      TIME_FIELD("Time of Position", "Seconds since midnight"),
      {"MMSI of Ship In Distress", BYTES(5), RES_DECIMAL, false, "MMSI", ""},
      ONE_BYTE_FIELD("DSC EOS Symbol"),
      LOOKUP_FIELD("Expansion Enabled", 2, YES_NO),
      RESERVED_FIELD(6),
      ASCII_FIELD("Calling Rx Frequency/Channel", BYTES(6)),
      ASCII_FIELD("Calling Tx Frequency/Channel", BYTES(6)),
      TIME_FIELD("Time of Receipt", "Seconds since midnight"),
      {"Date of Receipt", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      SIMPLE_FIELD("DSC Equipment Assigned Message ID", BYTES(2)),
      LOOKUP_FIELD("DSC Expansion Field Symbol", BYTES(1), DSC_EXPANSION_DATA),
      STRINGLAU_FIELD("DSC Expansion Field Data"),
      {0}}}

    ,
    {"AIS Class B static data (msg 24 Part A)",
     129809,
     PACKET_COMPLETE,
     PACKET_FAST,
     27,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat indicator", 2, REPEAT_INDICATOR),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      ASCII_FIELD("Name", BYTES(20)),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(3),
      INTEGER_FIELD("Sequence ID", BYTES(1)),
      {0}}}

    ,
    {"AIS Class B static data (msg 24 Part B)",
     129810,
     PACKET_COMPLETE,
     PACKET_FAST,
     34,
     0,
     {SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat indicator", 2, REPEAT_INDICATOR),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      LOOKUP_FIELD("Type of ship", BYTES(1), SHIP_TYPE),
      ASCII_FIELD("Vendor ID", BYTES(7)),
      ASCII_FIELD("Callsign", BYTES(7)),
      DECIMETERS_FIELD("Length"),
      DECIMETERS_FIELD("Beam"),
      DECIMETERS_FIELD("Position reference from Starboard"),
      DECIMETERS_FIELD("Position reference from Bow"),
      {"Mothership User ID", BYTES(4), RES_INTEGER, false, "MMSI", "MMSI of mother ship sent by daughter vessels"},
      RESERVED_FIELD(2),
      INTEGER_DESC_FIELD("Spare", 6, ",0=unavailable"),
      LOOKUP_FIELD("AIS Transceiver information", 5, AIS_TRANSCEIVER),
      RESERVED_FIELD(3),
      INTEGER_FIELD("Sequence ID", BYTES(1)),
      {0}}}

    ,
    {"Label", 130060, PACKET_INCOMPLETE | PACKET_NOT_SEEN, PACKET_FAST, 0, 0, {{0}}}

    ,
    {"Channel Source Configuration", 130061, PACKET_INCOMPLETE | PACKET_NOT_SEEN, PACKET_FAST, 0, 0, {{0}}}

    ,
    {"Route and WP Service - Database List",
     130064,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     9,
     {ONE_BYTE_FIELD("Start Database ID"),
      ONE_BYTE_FIELD("nItems"),
      ONE_BYTE_FIELD("Number of Databases Available")

          ,
      ONE_BYTE_FIELD("Database ID"),
      ASCII_FIELD("Database Name", BYTES(8)),
      TIME_FIELD("Database Timestamp", "Seconds since midnight"),
      {"Database Datestamp", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      SIMPLE_FIELD("WP Position Resolution", 6),
      RESERVED_FIELD(2),
      SIMPLE_FIELD("Number of Routes in Database", BYTES(2)),
      SIMPLE_FIELD("Number of WPs in Database", BYTES(2)),
      SIMPLE_FIELD("Number of Bytes in Database", BYTES(2)),
      {0}}}

    ,
    {"Route and WP Service - Route List",
     130065,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     6,
     {ONE_BYTE_FIELD("Start Route ID"),
      ONE_BYTE_FIELD("nItems"),
      ONE_BYTE_FIELD("Number of Routes in Database")

          ,
      ONE_BYTE_FIELD("Database ID"),
      ONE_BYTE_FIELD("Route ID"),
      ASCII_FIELD("Route Name", BYTES(8)),
      RESERVED_FIELD(4),
      SIMPLE_FIELD("WP Identification Method", 2),
      SIMPLE_FIELD("Route Status", 2),
      {0}}}

    ,
    {"Route and WP Service - Route/WP-List Attributes",
     130066,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {ONE_BYTE_FIELD("Database ID"),
      ONE_BYTE_FIELD("Route ID"),
      ASCII_FIELD("Route/WP-List Name", BYTES(8)),
      TIME_FIELD("Route/WP-List Timestamp", "Seconds since midnight"),
      {"Route/WP-List Datestamp", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      ONE_BYTE_FIELD("Change at Last Timestamp"),
      SIMPLE_FIELD("Number of WPs in the Route/WP-List", BYTES(2)),
      ONE_BYTE_FIELD("Critical supplementary parameters"),
      SIMPLE_FIELD("Navigation Method", 2),
      SIMPLE_FIELD("WP Identification Method", 2),
      SIMPLE_FIELD("Route Status", 2),
      SIMPLE_FIELD("XTE Limit for the Route", BYTES(2)),
      {0}}}

    ,
    {"Route and WP Service - Route - WP Name & Position",
     130067,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     4,
     {ONE_BYTE_FIELD("Start RPS#"),
      ONE_BYTE_FIELD("nItems"),
      SIMPLE_FIELD("Number of WPs in the Route/WP-List", BYTES(2)),
      ONE_BYTE_FIELD("Database ID"),
      ONE_BYTE_FIELD("Route ID")

          ,
      ONE_BYTE_FIELD("WP ID"),
      ASCII_FIELD("WP Name", BYTES(8)),
      {"WP Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"WP Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {0}}}

    ,
    {"Route and WP Service - Route - WP Name",
     130068,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {ONE_BYTE_FIELD("Start RPS#"),
      ONE_BYTE_FIELD("nItems"),
      SIMPLE_FIELD("Number of WPs in the Route/WP-List", BYTES(2)),
      ONE_BYTE_FIELD("Database ID"),
      ONE_BYTE_FIELD("Route ID"),
      ONE_BYTE_FIELD("WP ID"),
      ASCII_FIELD("WP Name", BYTES(8)),
      {0}}}

    ,
    {"Route and WP Service - XTE Limit & Navigation Method",
     130069,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     6,
     {ONE_BYTE_FIELD("Start RPS#"),
      ONE_BYTE_FIELD("nItems"),
      SIMPLE_FIELD("Number of WPs with a specific XTE Limit or Nav. Method", BYTES(2))

          ,
      ONE_BYTE_FIELD("Database ID"),
      ONE_BYTE_FIELD("Route ID"),
      ONE_BYTE_FIELD("RPS#"),
      SIMPLE_FIELD("XTE limit in the leg after WP", BYTES(2)),
      SIMPLE_FIELD("Nav. Method in the leg after WP", 4),
      RESERVED_FIELD(4),
      {0}}}

    ,
    {"Route and WP Service - WP Comment",
     130070,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {ONE_BYTE_FIELD("Start ID"),
      ONE_BYTE_FIELD("nItems"),
      SIMPLE_FIELD("Number of WPs with Comments", BYTES(2)),
      ONE_BYTE_FIELD("Database ID"),
      ONE_BYTE_FIELD("Route ID")

          ,
      ONE_BYTE_FIELD("WP ID / RPS#"),
      ASCII_FIELD("Comment", BYTES(8)),
      {0}}}

    ,
    {"Route and WP Service - Route Comment",
     130071,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {ONE_BYTE_FIELD("Start Route ID"),
      ONE_BYTE_FIELD("nItems"),
      SIMPLE_FIELD("Number of Routes with Comments", BYTES(2)),
      ONE_BYTE_FIELD("Database ID")

          ,
      ONE_BYTE_FIELD("Route ID"),
      ASCII_FIELD("Comment", BYTES(8)),
      {0}}}

    ,
    {"Route and WP Service - Database Comment",
     130072,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {ONE_BYTE_FIELD("Start Database ID"),
      ONE_BYTE_FIELD("nItems"),
      SIMPLE_FIELD("Number of Databases with Comments", BYTES(2))

          ,
      ONE_BYTE_FIELD("Database ID"),
      ASCII_FIELD("Comment", BYTES(8)),
      {0}}}

    ,
    {"Route and WP Service - Radius of Turn",
     130073,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {ONE_BYTE_FIELD("Start RPS#"),
      ONE_BYTE_FIELD("nItems"),
      SIMPLE_FIELD("Number of WPs with a specific Radius of Turn", BYTES(2)),
      ONE_BYTE_FIELD("Database ID"),
      ONE_BYTE_FIELD("Route ID")

          ,
      ONE_BYTE_FIELD("RPS#"),
      SIMPLE_FIELD("Radius of Turn", BYTES(2)),
      {0}}}

    ,
    {"Route and WP Service - WP List - WP Name & Position",
     130074,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     4,
     {ONE_BYTE_FIELD("Start WP ID"),
      ONE_BYTE_FIELD("nItems"),
      SIMPLE_FIELD("Number of valid WPs in the WP-List", BYTES(2)),
      ONE_BYTE_FIELD("Database ID"),
      RESERVED_FIELD(BYTES(1)),
      ONE_BYTE_FIELD("WP ID"),
      ASCII_FIELD("WP Name", BYTES(8)),
      {"WP Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"WP Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {0}}}

    /* http://askjackrabbit.typepad.com/ask_jack_rabbit/page/7/ */
    ,
    {"Wind Data",
     130306,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      SPEED_FIELD("Wind Speed"),
      ANGLE_POS_FIELD("Wind Angle", ""),
      LOOKUP_FIELD("Reference", 3, WIND_REFERENCE),
      RESERVED_FIELD(5 + BYTES(2)),
      {0}}}

    /* Water temperature, Transducer Measurement */
    ,
    {"Environmental Parameters",
     130310,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      TEMPERATURE_FIELD("Water Temperature"),
      TEMPERATURE_FIELD("Outside Ambient Air Temperature"),
      {"Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      RESERVED_FIELD(BYTES(1)),
      {0}}}

    ,
    {"Environmental Parameters",
     130311,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      LOOKUP_FIELD("Temperature Source", 6, TEMPERATURE_SOURCE),
      LOOKUP_FIELD("Humidity Source", 2, HUMIDITY_SOURCE),
      TEMPERATURE_FIELD("Temperature"),
      PERCENTAGE_FIELD("Humidity"),
      {"Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {0}}}

    ,
    {"Temperature",
     130312,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      INSTANCE_FIELD,
      LOOKUP_FIELD("Source", BYTES(1), TEMPERATURE_SOURCE),
      TEMPERATURE_FIELD("Actual Temperature"),
      TEMPERATURE_FIELD("Set Temperature"),
      {0}}}

    ,
    {"Humidity",
     130313,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      INSTANCE_FIELD,
      LOOKUP_FIELD("Source", BYTES(1), HUMIDITY_SOURCE),
      PERCENTAGE_FIELD("Actual Humidity"),
      PERCENTAGE_FIELD("Set Humidity"),
      {0}}}

    ,
    {"Actual Pressure",
     130314,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      INSTANCE_FIELD,
      LOOKUP_FIELD("Source", BYTES(1), PRESSURE_SOURCE),
      {"Pressure", BYTES(4), RES_PRESSURE_HIRES, true, "dPa", ""},
      {0}}}

    ,
    {"Set Pressure",
     130315,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      INSTANCE_FIELD,
      LOOKUP_FIELD("Source", BYTES(1), PRESSURE_SOURCE),
      {"Pressure", BYTES(4), RES_PRESSURE_HIRES, false, "dPa", ""},
      {0}}}

    ,
    {"Temperature Extended Range",
     130316,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {ONE_BYTE_FIELD("SID"),
      INSTANCE_FIELD,
      LOOKUP_FIELD("Source", BYTES(1), TEMPERATURE_SOURCE),
      {"Temperature", BYTES(3), RES_TEMPERATURE_HIRES, false, "K", ""},
      TEMPERATURE_HIGH_FIELD("Set Temperature"),
      {0}}}

    ,
    {"Tide Station Data",
     130320,
     PACKET_COMPLETE,
     PACKET_FAST,
     20,
     0,
     {LOOKUP_FIELD("Mode", 4, RESIDUAL_MODE),
      LOOKUP_FIELD("Tide Tendency", 2, TIDE),
      RESERVED_FIELD(2),
      {"Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      TIME_FIELD("Measurement Time", "Seconds since midnight"),
      {"Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      DISTANCE_FIELD("Tide Level", BYTES(2), 0.001, "Relative to MLLW"),
      LENGTH_FIELD("Tide Level standard deviation", BYTES(2), 0.01, ""),
      STRINGVAR_FIELD("Station ID"),
      STRINGVAR_FIELD("Station Name"),
      {0}}}

    ,
    {"Salinity Station Data",
     130321,
     PACKET_COMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     22,
     0,
     {LOOKUP_FIELD("Mode", 4, RESIDUAL_MODE),
      RESERVED_FIELD(4),
      {"Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      TIME_FIELD("Measurement Time", "Seconds since midnight"),
      {"Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Salinity",
       BYTES(4),
       RES_FLOAT,
       true,
       "ppt",
       "The average Salinity of ocean water is about 35 grams of salts per kilogram of sea water (g/kg), usually written as 35 "
       "ppt "
       "which is read as 35 parts per thousand."},
      TEMPERATURE_FIELD("Water Temperature"),
      STRINGVAR_FIELD("Station ID"),
      STRINGVAR_FIELD("Station Name"),
      {0}}}

    ,
    {"Current Station Data",
     130322,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {SIMPLE_FIELD("Mode", 4),
      RESERVED_FIELD(4),
      {"Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      TIME_FIELD("Measurement Time", "Seconds since midnight"),
      {"Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      LENGTH_FIELD("Measurement Depth", BYTES(4), 0.01, "Depth below transducer"),
      SPEED_FIELD("Current speed"),
      ANGLE_POS_FIELD("Current flow direction", ""),
      TEMPERATURE_FIELD("Water Temperature"),
      STRINGVAR_FIELD("Station ID"),
      STRINGVAR_FIELD("Station Name"),
      {0}}}

    ,
    {"Meteorological Station Data",
     130323,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x1e,
     0,
     {SIMPLE_FIELD("Mode", 4),
      RESERVED_FIELD(4),
      {"Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      TIME_FIELD("Measurement Time", "Seconds since midnight"),
      {"Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      SPEED_FIELD("Wind Speed"),
      ANGLE_POS_FIELD("Wind Direction", ""),
      LOOKUP_FIELD("Wind Reference", 3, WIND_REFERENCE),
      RESERVED_FIELD(5),
      SPEED_FIELD("Wind Gusts"),
      {"Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      TEMPERATURE_FIELD("Ambient Temperature"),
      STRINGVAR_FIELD("Station ID"),
      STRINGVAR_FIELD("Station Name"),
      {0}}}

    ,
    {"Moored Buoy Station Data",
     130324,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {SIMPLE_FIELD("Mode", 4),
      RESERVED_FIELD(4),
      {"Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      TIME_FIELD("Measurement Time", "Seconds since midnight"),
      {"Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      SPEED_FIELD("Wind Speed"),
      ANGLE_POS_FIELD("Wind Direction", ""),
      LOOKUP_FIELD("Wind Reference", 3, WIND_REFERENCE),
      RESERVED_FIELD(5),
      SPEED_FIELD("Wind Gusts"),
      SIMPLE_FIELD("Wave Height", BYTES(2)),
      SIMPLE_FIELD("Dominant Wave Period", BYTES(2)),
      {"Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Pressure Tendency Rate", BYTES(2), 1, false, "", ""},
      TEMPERATURE_FIELD("Air Temperature"),
      TEMPERATURE_FIELD("Water Temperature"),
      ASCII_FIELD("Station ID", BYTES(8)),
      {0}}}

    ,
    {"Payload Mass", 130560, PACKET_INCOMPLETE | PACKET_NOT_SEEN, PACKET_FAST, 0, 0, {{0}}}

    /* http://www.nmea.org/Assets/20130905%20amendment%20at%202000%20201309051%20watermaker%20input%20setting%20and%20status%20pgn%20130567.pdf

    This PGN may be requested or used to command and configure a number of Watermaker controls. The Command Group Function PGN
    126208 is used perform the following: start/stop a production, start/stop rinse or flush operation , start/stop low and high
    pressure pump and perform an emergency stop. The Request Group Function PGN 126208 or ISO Request PGN 059904 may be used to
    request this PGN. This PGN also provides Watermaker status and measurement information. The PGN is broadcast periodically.

    */
    ,
    {"Watermaker Input Setting and Status",
     130567,
     PACKET_COMPLETE,
     PACKET_FAST,
     24,
     0,
     {LOOKUP_FIELD("Watermaker Operating State", 6, WATERMAKER_STATE),
      LOOKUP_FIELD("Production Start/Stop", 2, YES_NO),
      LOOKUP_FIELD("Rinse Start/Stop", 2, YES_NO),
      LOOKUP_FIELD("Low Pressure Pump Status", 2, YES_NO),
      LOOKUP_FIELD("High Pressure Pump Status", 2, YES_NO),
      LOOKUP_FIELD("Emergency Stop", 2, YES_NO),
      LOOKUP_FIELD("Product Solenoid Valve Status", 2, OK_WARNING),
      LOOKUP_FIELD("Flush Mode Status", 2, YES_NO),
      LOOKUP_FIELD("Salinity Status", 2, OK_WARNING),
      LOOKUP_FIELD("Sensor Status", 2, OK_WARNING),
      LOOKUP_FIELD("Oil Change Indicator Status", 2, OK_WARNING),
      LOOKUP_FIELD("Filter Status", 2, OK_WARNING),
      LOOKUP_FIELD("System Status", 2, OK_WARNING),
      RESERVED_FIELD(2),
      {"Salinity", BYTES(2), RES_INTEGER, false, "ppm", ""},
      TEMPERATURE_FIELD("Product Water Temperature"),
      {"Pre-filter Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Post-filter Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Feed Pressure", BYTES(2), RES_PRESSURE, true, "kPa", ""},
      {"System High Pressure", BYTES(2), RES_PRESSURE, false, "kPa", ""},
      {"Product Water Flow", BYTES(2), 0.1, true, "L/h", ""},
      {"Brine Water Flow", BYTES(2), 0.1, true, "L/h", ""},
      ELAPSED_FIELD("Run Time", BYTES(4), 1),
      {0}}}

    /* https://www.nmea.org/Assets/20160725%20corrigenda%20pgn%20130569%20published.pdf */
    ,
    {"Current Status and File",
     130569,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     0,
     {LOOKUP_FIELD("Zone", BYTES(1), ENTERTAINMENT_ZONE),
      LOOKUP_FIELD("Source", 8, ENTERTAINMENT_SOURCE),
      INTEGER_DESC_FIELD("Number", BYTES(1), "Source number per type"),
      INTEGER_DESC_FIELD("ID", BYTES(4), "Unique file ID"),
      LOOKUP_FIELD("Play status", BYTES(1), ENTERTAINMENT_PLAY_STATUS),
      SHORT_TIME_FIELD("Elapsed Track Time"),
      SHORT_TIME_FIELD("Track Time"),
      LOOKUP_FIELD("Repeat Status", 4, ENTERTAINMENT_REPEAT_STATUS),
      LOOKUP_FIELD("Shuffle Status", 4, ENTERTAINMENT_SHUFFLE_STATUS),
      INTEGER_DESC_FIELD("Save Favorite Number", BYTES(1), "Used to command AV to save current station as favorite"),
      INTEGER_DESC_FIELD("Play Favorite Number", BYTES(2), "Used to command AV to play indicated favorite station"),
      LOOKUP_FIELD("Thumbs Up/Down", BYTES(1), ENTERTAINMENT_LIKE_STATUS),
      SHORT_PERCENTAGE_FIELD("Signal Strength"),
      RADIO_FREQUENCY_FIELD("Radio Frequency", 10),
      INTEGER_DESC_FIELD("HD Frequency Multicast", BYTES(1), "Digital sub channel"),
      INTEGER_DESC_FIELD("Delete Favorite Number", BYTES(1), "Used to command AV to delete current station as favorite"),
      INTEGER_FIELD("Total Number of Tracks", BYTES(2)),
      {0}}}

    /* https://www.nmea.org/Assets/20160715%20corrigenda%20entertainment%20pgns%20.pdf */

    ,
    {"Library Data File",
     130570,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     0,
     {LOOKUP_FIELD("Source", 8, ENTERTAINMENT_SOURCE),
      INTEGER_DESC_FIELD("Number", BYTES(1), "Source number per type"),
      INTEGER_DESC_FIELD("ID", BYTES(4), "Unique file ID"),
      LOOKUP_FIELD("Type", BYTES(1), ENTERTAINMENT_TYPE),
      STRINGLAU_FIELD("Name"),
      INTEGER_FIELD("Track", BYTES(2)),
      INTEGER_FIELD("Station", BYTES(2)),
      INTEGER_FIELD("Favorite", BYTES(1)),
      RADIO_FREQUENCY_FIELD("Radio Frequency", 10.),
      INTEGER_FIELD("HD Frequency", BYTES(1)),
      LOOKUP_FIELD("Zone", BYTES(1), ENTERTAINMENT_ZONE),
      LOOKUP_FIELD("In play queue", 2, YES_NO),
      LOOKUP_FIELD("Locked", 2, YES_NO),
      RESERVED_FIELD(4),
      STRINGLAU_FIELD("Artist"),
      STRINGLAU_FIELD("Album"),
      STRINGLAU_FIELD("Station"),
      {0}}}

    ,
    {"Library Data Group",
     130571,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     2,
     {
         LOOKUP_FIELD("Source", 8, ENTERTAINMENT_SOURCE),
         INTEGER_DESC_FIELD("Number", BYTES(1), "Source number per type"),
         LOOKUP_FIELD("Zone", BYTES(1), ENTERTAINMENT_ZONE),
         INTEGER_DESC_FIELD("Group ID", BYTES(4), "Unique group ID"),
         INTEGER_DESC_FIELD("ID offset", BYTES(2), "First ID in this PGN"),
         INTEGER_DESC_FIELD("ID count", BYTES(2), "Number of IDs in this PGN"),
         INTEGER_DESC_FIELD("Total ID count", BYTES(2), "Total IDs in group"),
         LOOKUP_FIELD("ID type", BYTES(1), ENTERTAINMENT_ID_TYPE),
         INTEGER_FIELD("ID", BYTES(4)),
         STRINGLAU_FIELD("Name")
         // TODO: Add support for extra fields *after* the repeating fields.
         // The NMEA, in all its wisdom, suddenly feels a repeating field PGN can act to different rules. Sigh.
         // , { "Artist", BYTES(2), RES_STRINGLAU, false, 0, "" }
     }}

    ,
    {"Library Data Search",
     130572,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     0,
     {LOOKUP_FIELD("Source", 8, ENTERTAINMENT_SOURCE),
      INTEGER_DESC_FIELD("Number", BYTES(1), "Source number per type"),
      INTEGER_DESC_FIELD("Group ID", BYTES(4), "Unique group ID"),
      LOOKUP_FIELD("Group type 1", BYTES(1), ENTERTAINMENT_GROUP),
      STRINGLAU_FIELD("Group name 1"),
      LOOKUP_FIELD("Group type 2", BYTES(1), ENTERTAINMENT_GROUP),
      STRINGLAU_FIELD("Group name 2"),
      LOOKUP_FIELD("Group type 3", BYTES(1), ENTERTAINMENT_GROUP),
      STRINGLAU_FIELD("Group name 3"),
      {0}}}

    ,
    {"Supported Source Data",
     130573,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     10,
     {INTEGER_DESC_FIELD("ID offset", BYTES(2), "First ID in this PGN"),
      INTEGER_DESC_FIELD("ID count", BYTES(2), "Number of IDs in this PGN"),
      INTEGER_DESC_FIELD("Total ID count", BYTES(2), "Total IDs in group"),
      INTEGER_DESC_FIELD("ID", BYTES(1), "Source ID"),
      LOOKUP_FIELD("Source", 8, ENTERTAINMENT_SOURCE),
      INTEGER_DESC_FIELD("Number", BYTES(1), "Source number per type"),
      STRINGLAU_FIELD("Name"),
      LOOKUP_BITFIELD("Play support", BYTES(4), ENTERTAINMENT_PLAY_STATUS_BITFIELD),
      LOOKUP_BITFIELD("Browse support", BYTES(2), ENTERTAINMENT_GROUP_BITFIELD),
      LOOKUP_FIELD("Thumbs support", 2, YES_NO),
      LOOKUP_FIELD("Connected", 2, YES_NO),
      LOOKUP_BITFIELD("Repeat support", 2, ENTERTAINMENT_REPEAT_BITFIELD),
      LOOKUP_BITFIELD("Shuffle support", 2, ENTERTAINMENT_SHUFFLE_BITFIELD),
      {0}}}

    ,
    {"Supported Zone Data",
     130574,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     2,
     {INTEGER_DESC_FIELD("First zone ID", BYTES(1), "First Zone in this PGN"),
      INTEGER_DESC_FIELD("Zone count", BYTES(1), "Number of Zones in this PGN"),
      INTEGER_DESC_FIELD("Total zone count", BYTES(1), "Total Zones supported by this device"),
      LOOKUP_FIELD("Zone ID", BYTES(1), ENTERTAINMENT_ZONE),
      STRINGLAU_FIELD("Name"),
      {0}}}

    ,
    {"Small Craft Status",
     130576,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_SINGLE,
     2,
     0,
     {{"Port trim tab", BYTES(1), 1, true, 0, ""}, {"Starboard trim tab", BYTES(1), 1, true, 0, ""}, {0}}}

    ,
    {"Direction Data",
     130577,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     14,
     0,
     {LOOKUP_FIELD("Data Mode", 4, RESIDUAL_MODE),
      LOOKUP_FIELD("COG Reference", 2, DIRECTION_REFERENCE),
      RESERVED_FIELD(2),
      ONE_BYTE_FIELD("SID")
      /* So far, 2 bytes. Very sure of this given molly rose data */
      ,
      ANGLE_POS_FIELD("COG", ""),
      SPEED_FIELD("SOG"),
      ANGLE_POS_FIELD("Heading", ""),
      SPEED_FIELD("Speed through Water"),
      ANGLE_POS_FIELD("Set", ""),
      SPEED_FIELD("Drift"),
      {0}}}

    ,
    {"Vessel Speed Components",
     130578,
     PACKET_COMPLETE,
     PACKET_FAST,
     12,
     0,
     {MILLIMETER_SPEED_FIELD("Longitudinal Speed, Water-referenced"),
      MILLIMETER_SPEED_FIELD("Transverse Speed, Water-referenced"),
      MILLIMETER_SPEED_FIELD("Longitudinal Speed, Ground-referenced"),
      MILLIMETER_SPEED_FIELD("Transverse Speed, Ground-referenced"),
      MILLIMETER_SPEED_FIELD("Stern Speed, Water-referenced"),
      MILLIMETER_SPEED_FIELD("Stern Speed, Ground-referenced"),
      {0}}}

    ,
    {"System Configuration",
     130579,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     (48 / 8 + 2),
     0,
     {LOOKUP_FIELD("Power", 2, YES_NO),
      LOOKUP_FIELD("Default Settings", 2, ENTERTAINMENT_DEFAULT_SETTINGS),
      LOOKUP_FIELD("Tuner regions", 4, ENTERTAINMENT_REGIONS),
      INTEGER_FIELD("Max favorites", BYTES(1)),
      LOOKUP_BITFIELD("Video protocols", 4, VIDEO_PROTOCOLS),
      RESERVED_FIELD(44),
      {0}}}

    ,
    {"System Configuration (deprecated)",
     130580,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     2,
     0,
     {LOOKUP_FIELD("Power", 2, YES_NO),
      LOOKUP_FIELD("Default Settings", 2, ENTERTAINMENT_DEFAULT_SETTINGS),
      LOOKUP_FIELD("Tuner regions", 4, ENTERTAINMENT_REGIONS),
      INTEGER_FIELD("Max favorites", BYTES(1)),
      {0}}}

    ,
    {"Zone Configuration (deprecated)",
     130581,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     14,
     2,
     {INTEGER_DESC_FIELD("First zone ID", BYTES(1), "First Zone in this PGN"),
      INTEGER_DESC_FIELD("Zone count", BYTES(1), "Number of Zones in this PGN"),
      INTEGER_DESC_FIELD("Total zone count", BYTES(1), "Total Zones supported by this device"),
      LOOKUP_FIELD("Zone ID", BYTES(1), ENTERTAINMENT_ZONE),
      STRINGLAU_FIELD("Zone name"),
      {0}}}

    ,
    {"Zone Volume",
     130582,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     4,
     0,
     {LOOKUP_FIELD("Zone ID", BYTES(1), ENTERTAINMENT_ZONE),
      SHORT_PERCENTAGE_FIELD("Volume"),
      LOOKUP_FIELD_DESC("Volume change", 2, ENTERTAINMENT_VOLUME_CONTROL, "Write only"),
      LOOKUP_FIELD("Mute", 2, YES_NO),
      RESERVED_FIELD(4),
      LOOKUP_FIELD("Channel", 8, ENTERTAINMENT_CHANNEL),
      {0}}}

    ,
    {"Available Audio EQ presets",
     130583,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     2,
     {INTEGER_DESC_FIELD("First preset", BYTES(1), "First preset in this PGN"),
      INTEGER_FIELD("Preset count", BYTES(1)),
      INTEGER_FIELD("Total preset count", BYTES(1)),
      LOOKUP_FIELD("Preset type", BYTES(1), ENTERTAINMENT_EQ),
      STRINGLAU_FIELD("Preset name"),
      {0}}}

    ,
    {"Available Bluetooth addresses",
     130584,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     3,
     {INTEGER_DESC_FIELD("First address", BYTES(1), "First address in this PGN"),
      INTEGER_FIELD("Address count", BYTES(1)),
      INTEGER_FIELD("Total address count", BYTES(1)),
      INTEGER_FIELD("Bluetooth address", BYTES(6)),
      LOOKUP_FIELD("Status", BYTES(1), BLUETOOTH_STATUS),
      STRINGLAU_FIELD("Device name"),
      SHORT_PERCENTAGE_FIELD("Signal strength"),
      {0}}}

    ,
    {"Bluetooth source status",
     130585,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     0,
     {INTEGER_FIELD("Source number", BYTES(1)),
      LOOKUP_FIELD("Status", 4, BLUETOOTH_SOURCE_STATUS),
      LOOKUP_FIELD("Forget device", 2, YES_NO),
      LOOKUP_FIELD("Discovering", 2, YES_NO),
      INTEGER_FIELD("Bluetooth address", BYTES(6)),
      {0}}}

    ,
    {"Zone Configuration",
     130586,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     14,
     2,
     {LOOKUP_FIELD("Zone ID", BYTES(1), ENTERTAINMENT_ZONE),
      SHORT_PERCENTAGE_FIELD("Volume limit"),
      {"Fade", BYTES(1), RES_INTEGER, true, "%", ""},
      {"Balance", BYTES(1), RES_INTEGER, true, "%", ""},
      {"Sub volume", BYTES(1), RES_INTEGER, true, "%", ""},
      {"EQ - Treble", BYTES(1), RES_INTEGER, true, "%", ""},
      {"EQ - Mid range", BYTES(1), RES_INTEGER, true, "%", ""},
      {"EQ - Bass", BYTES(1), RES_INTEGER, true, "%", ""},
      LOOKUP_FIELD("Preset type", BYTES(1), ENTERTAINMENT_EQ),
      LOOKUP_FIELD("Audio filter", BYTES(1), ENTERTAINMENT_FILTER),
      FREQUENCY_FIELD("High pass filter frequency", RES_INTEGER),
      FREQUENCY_FIELD("Low pass filter frequency", RES_INTEGER),
      LOOKUP_FIELD("Channel", 8, ENTERTAINMENT_CHANNEL),
      {0}}}

    /* proprietary PDU2 (non addressed) fast packet PGN range 0x1FF00 to 0x1FFFF (130816 - 131071) */
    ,
    {"SonicHub: Init #2",
     130816,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     9,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 1, "Init #2"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("A", BYTES(2)),
      INTEGER_FIELD("B", BYTES(2)),
      {0}}}

    ,
    {"SonicHub: AM Radio",
     130816,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 4, "AM Radio"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      LOOKUP_FIELD("Item", BYTES(1), SONICHUB_TUNING),
      RADIO_FREQUENCY_FIELD("Frequency", 1),
      SIMPLE_FIELD("Noise level", 2),  // Not sure about this
      SIMPLE_FIELD("Signal level", 4), // ... and this, doesn't make complete sense compared to display
      RESERVED_FIELD(2),
      STRINGLZ_FIELD("Text", BYTES(32)),
      {0}}}

    ,
    {"SonicHub: Zone info",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 5, "Zone info"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("Zone", BYTES(1)),
      {0}}}

    ,
    {"SonicHub: Source",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 6, "Source"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      LOOKUP_FIELD("Source", BYTES(1), SONICHUB_SOURCE),
      {0}}}

    ,
    {"SonicHub: Source List",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 8, "Source list"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("Source ID", BYTES(1)),
      INTEGER_FIELD("A", 8),
      STRINGLZ_FIELD("Text", BYTES(32)),
      {0}}}

    ,
    {"SonicHub: Control",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 9, "Control"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      LOOKUP_FIELD("Item", BYTES(1), FUSION_MUTE_COMMAND),
      {0}}}

    ,
    {"SonicHub: Unknown",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 9, "Unknown"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("A", 8),
      INTEGER_FIELD("B", 8),
      {0}}}

    ,
    {"SonicHub: FM Radio",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 12, "FM Radio"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      LOOKUP_FIELD("Item", BYTES(1), SONICHUB_TUNING),
      RADIO_FREQUENCY_FIELD("Frequency", 1),
      SIMPLE_FIELD("Noise level", 2) // Not sure about this
      ,
      SIMPLE_FIELD("Signal level", 4) // ... and this, doesn't make complete sense compared to display
      ,
      RESERVED_FIELD(2),
      STRINGLZ_FIELD("Text", BYTES(32)),
      {0}}}

    ,
    {"SonicHub: Playlist",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 13, "Playlist"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      LOOKUP_FIELD("Item", BYTES(1), SONICHUB_PLAYLIST),
      INTEGER_FIELD("A", BYTES(1)),
      INTEGER_FIELD("Current Track", BYTES(4)),
      INTEGER_FIELD("Tracks", BYTES(4)),
      ELAPSED_FIELD("Length", BYTES(4), 0.001),
      ELAPSED_FIELD("Position in track", BYTES(4), 0.001),
      {0}}}

    ,
    {"SonicHub: Track",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 14, "Track"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("Item", BYTES(4)),
      STRINGLZ_FIELD("Text", BYTES(32)),
      {0}}}

    ,
    {"SonicHub: Artist",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 15, "Artist"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("Item", BYTES(4)),
      STRINGLZ_FIELD("Text", BYTES(32)),
      {0}}}

    ,
    {"SonicHub: Album",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 16, "Album"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("Item", BYTES(4)),
      STRINGLZ_FIELD("Text", BYTES(32)),
      {0}}}

    ,
    {"SonicHub: Menu Item",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 19, "Menu Item"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("Item", BYTES(4)),
      ONE_BYTE_FIELD("C"),
      ONE_BYTE_FIELD("D"),
      ONE_BYTE_FIELD("E"),
      STRINGLZ_FIELD("Text", BYTES(32)),
      {0}}}

    ,
    {"SonicHub: Zones",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 20, "Zones"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("Zones", BYTES(1)),
      {0}}}

    ,
    {"SonicHub: Max Volume",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 23, "Max Volume"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("Zone", BYTES(1)),
      INTEGER_FIELD("Level", BYTES(1)),
      {0}}}

    ,
    {"SonicHub: Volume",
     130816,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 24, "Volume"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("Zone", BYTES(1)),
      INTEGER_FIELD("Level", BYTES(1)),
      {0}}}

    ,
    {"SonicHub: Init #1",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 25, "Init #1"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      {0}}}

    ,
    {"SonicHub: Position",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 48, "Position"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      ELAPSED_FIELD("Position", BYTES(4), 0.001),
      {0}}}

    ,
    {"SonicHub: Init #3",
     130816,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     9,
     0,
     {COMPANY(275),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 50, "Init #3"),
      LOOKUP_FIELD("Control", BYTES(1), SONICHUB_CONTROL),
      INTEGER_FIELD("A", BYTES(1)),
      INTEGER_FIELD("B", BYTES(1)),
      {0}}}

    ,
    {"Simrad: Text Message",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {COMPANY(1857),
      RESERVED_FIELD(BYTES(1)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 50, "Init #3") // FIXME
      ,
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      ONE_BYTE_FIELD("C"),
      ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("Prio"),
      ASCII_FIELD("Text", BYTES(32)),
      {0}}}

    ,
    {"Manufacturer Proprietary fast-packet non-addressed",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     223,
     0,
     {MANUFACTURER_FIELDS, BINARY_FIELD("Data", BYTES(221), ""), {0}},
     0,
     0,
     true}

    ,
    {"Navico: Product Information",
     130817,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0e,
     0,
     {COMPANY(275),
      INTEGER_FIELD("Product Code", BYTES(2)),
      ASCII_FIELD("Model", BYTES(32)),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      ONE_BYTE_FIELD("C"),
      ASCII_FIELD("Firmware version", BYTES(10)),
      ASCII_FIELD("Firmware date", BYTES(32)),
      ASCII_FIELD("Firmware time", BYTES(32)),
      {0}}}

    ,
    {"Simnet: Reprogram Data",
     130818,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     223,
     0,
     {COMPANY(1857),
      INTEGER_FIELD("Version", BYTES(2)),
      INTEGER_FIELD("Sequence", BYTES(2)),
      BINARY_FIELD("Data", BYTES(217), ""),
      {0}}}

    ,
    {"Simnet: Request Reprogram",
     130819,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1857), {0}}}

    ,
    {"Simnet: Reprogram Status",
     130820,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1857), RESERVED_FIELD(BYTES(1)), ONE_BYTE_FIELD("Status"), RESERVED_FIELD(BYTES(3)), {0}}}

    /* M/V Dirona */
    ,
    {"Furuno: Unknown",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1855), ONE_BYTE_FIELD("A"), ONE_BYTE_FIELD("B"), ONE_BYTE_FIELD("C"), ONE_BYTE_FIELD("D"), ONE_BYTE_FIELD("E"), {0}}}

    /* Fusion */
    ,
    {"Fusion: Source Name",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     13,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 2, "Source"),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("Source ID"),
      ONE_BYTE_FIELD("Current Source ID"),
      ONE_BYTE_FIELD("D"),
      ONE_BYTE_FIELD("E"),
      STRINGLZ_FIELD("Source", BYTES(5)),
      {0}}}

    ,
    {"Fusion: Track Info",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     23,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 4, "Track Info"),
      SIMPLE_FIELD("A", BYTES(2)),
      LOOKUP_FIELD("Transport", 4, ENTERTAINMENT_PLAY_STATUS),
      SIMPLE_FIELD("X", 4),
      ONE_BYTE_FIELD("B"),
      SIMPLE_FIELD("Track #", BYTES(2)),
      SIMPLE_FIELD("C", BYTES(2)),
      SIMPLE_FIELD("Track Count", BYTES(2)),
      SIMPLE_FIELD("E", BYTES(2)),
      {"Track Length", BYTES(3), 0.001, false, 0, ""},
      {"G", BYTES(3), 0.001, false, 0, ""},
      SIMPLE_FIELD("H", BYTES(2))}},
    {"Fusion: Track",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x20,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 5, "Track Title"),
      ONE_BYTE_FIELD("A"),
      SIMPLE_FIELD("B", BYTES(5)),
      STRINGLZ_FIELD("Track", BYTES(10)),
      {0}}}

    ,
    {"Fusion: Artist",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x20,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 6, "Track Artist"),
      ONE_BYTE_FIELD("A"),
      SIMPLE_FIELD("B", BYTES(5)),
      STRINGLZ_FIELD("Artist", BYTES(10)),
      {0}}}

    ,
    {"Fusion: Album",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x20,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 7, "Track Album"),
      ONE_BYTE_FIELD("A"),
      SIMPLE_FIELD("B", BYTES(5)),
      STRINGLZ_FIELD("Album", BYTES(10)),
      {0}}}

    ,
    {"Fusion: Unit Name",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x20,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 33, "Unit Name"),
      ONE_BYTE_FIELD("A"),
      STRINGLZ_FIELD("Name", BYTES(14)),
      {0}}}

    ,
    {"Fusion: Zone Name",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x20,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 45, "Zone Name"),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("Number"),
      STRINGLZ_FIELD("Name", BYTES(13)),
      {0}}}

    ,
    {"Fusion: Play Progress",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 9, "Track Progress"),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      ELAPSED_FIELD("Progress", BYTES(3), 0.001),
      {0}}}

    ,
    {"Fusion: AM/FM Station",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0A,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 11, "AM/FM Station"),
      ONE_BYTE_FIELD("A"),
      LOOKUP_FIELD("AM/FM", BYTES(1), FUSION_RADIO_SOURCE),
      ONE_BYTE_FIELD("B"),
      RADIO_FREQUENCY_FIELD("Frequency", 1),
      ONE_BYTE_FIELD("C"),
      STRINGLZ_FIELD("Track", BYTES(10)),
      {0}}}

    ,
    {"Fusion: VHF",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 12, "VHF"),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      ONE_BYTE_FIELD("Channel"),
      SIMPLE_FIELD("D", BYTES(3)),
      {0}}}

    ,
    {"Fusion: Squelch",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 13, "Squelch"),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      ONE_BYTE_FIELD("Squelch"),
      {0}}}

    ,
    {"Fusion: Scan",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 14, "Scan"),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      LOOKUP_FIELD("Scan", BITS(2), YES_NO),
      SIMPLE_FIELD("C", BITS(6)),
      {0}}}

    ,
    {"Fusion: Menu Item",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     23,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 17, "Menu Item"),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      ONE_BYTE_FIELD("Line"),
      ONE_BYTE_FIELD("E"),
      ONE_BYTE_FIELD("F"),
      ONE_BYTE_FIELD("G"),
      ONE_BYTE_FIELD("H"),
      ONE_BYTE_FIELD("I"),
      STRINGLZ_FIELD("Text", BYTES(5)),
      {0}}}

    ,
    {"Fusion: Replay",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     23,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 20, "Replay"),
      ONE_BYTE_FIELD("A"),
      LOOKUP_FIELD("Mode", BYTES(1), FUSION_REPLAY_MODE),
      SIMPLE_FIELD("C", BYTES(3)),
      ONE_BYTE_FIELD("D"),
      ONE_BYTE_FIELD("E"),
      LOOKUP_FIELD("Status", BYTES(1), FUSION_REPLAY_STATUS),
      ONE_BYTE_FIELD("H"),
      ONE_BYTE_FIELD("I"),
      ONE_BYTE_FIELD("J"),
      {0}}}

    ,
    {"Fusion: Mute",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     5,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 23, "Mute"),
      ONE_BYTE_FIELD("A"),
      LOOKUP_FIELD("Mute", BYTES(1), FUSION_MUTE_COMMAND),
      {0}}}

    ,
    // Range: 0 to +24
    {"Fusion: Sub Volume",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 26, "Sub Volume"),
      ONE_BYTE_FIELD("A"),
      {"Zone 1", BYTES(1), 1, false, "vol", ""},
      {"Zone 2", BYTES(1), 1, false, "vol", ""},
      {"Zone 3", BYTES(1), 1, false, "vol", ""},
      {"Zone 4", BYTES(1), 1, false, "vol", ""},
      {0}}}

    ,
    // Range: -15 to +15
    {"Fusion: Tone",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 27, "Tone"),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      {"Bass", BYTES(1), 1, true, "vol", ""},
      {"Mid", BYTES(1), 1, true, "vol", ""},
      {"Treble", BYTES(1), 1, true, "vol", ""},
      {0}}}

    ,
    {"Fusion: Volume",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x0A,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 29, "Volume"),
      ONE_BYTE_FIELD("A"),
      {"Zone 1", BYTES(1), 1, false, "vol", ""},
      {"Zone 2", BYTES(1), 1, false, "vol", ""},
      {"Zone 3", BYTES(1), 1, false, "vol", ""},
      {"Zone 4", BYTES(1), 1, false, "vol", ""},
      {0}}}

    ,
    {"Fusion: Power State",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     5,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 32, "Power"),
      ONE_BYTE_FIELD("A"),
      LOOKUP_FIELD("State", BYTES(1), FUSION_POWER_STATE),
      {0}}}

    ,
    {"Fusion: SiriusXM Channel",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x20,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 36, "SiriusXM Channel"),
      SIMPLE_FIELD("A", BYTES(4)),
      STRINGLZ_FIELD("Channel", BYTES(12)),
      {0}}}

    ,
    {"Fusion: SiriusXM Title",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x20,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 37, "SiriusXM Title"),
      SIMPLE_FIELD("A", BYTES(4)),
      STRINGLZ_FIELD("Title", BYTES(12)),
      {0}}}

    ,
    {"Fusion: SiriusXM Artist",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x20,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 38, "SiriusXM Artist"),
      SIMPLE_FIELD("A", BYTES(4)),
      STRINGLZ_FIELD("Artist", BYTES(12)),
      {0}}}

    ,
    {"Fusion: SiriusXM Genre",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x20,
     0,
     {COMPANY(419),
      MATCH_FIELD("Message ID", BYTES(1), 40, "SiriusXM Genre"),
      SIMPLE_FIELD("A", BYTES(4)),
      STRINGLZ_FIELD("Genre", BYTES(12)),
      {0}}}

    /* M/V Dirona */
    ,
    {"Furuno: Unknown",
     130821,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0c,
     0,
     {COMPANY(1855),
      ONE_BYTE_FIELD("SID"),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      ONE_BYTE_FIELD("C"),
      ONE_BYTE_FIELD("D"),
      ONE_BYTE_FIELD("E"),
      ONE_BYTE_FIELD("F"),
      ONE_BYTE_FIELD("G"),
      ONE_BYTE_FIELD("H"),
      ONE_BYTE_FIELD("I"),
      {0}}}

    ,
    {"Maretron: Proprietary Temperature High Range",
     130823,
     PACKET_COMPLETE,
     PACKET_FAST,
     9,
     0,
     {COMPANY(137),
      ONE_BYTE_FIELD("SID"),
      INSTANCE_FIELD,
      LOOKUP_FIELD("Source", BYTES(1), TEMPERATURE_SOURCE),
      TEMPERATURE_HIGH_FIELD("Actual Temperature"),
      TEMPERATURE_HIGH_FIELD("Set Temperature"),
      {0}}}

    ,
    {"B&G: Wind data",
     130824,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {COMPANY(381),
      ONE_BYTE_FIELD("Field 4"),
      ONE_BYTE_FIELD("Field 5"),
      SIMPLE_DESC_FIELD("Timestamp", BYTES(4), "Increasing field, what else can it be?"),
      {0}}}

    /* M/V Dirona */
    ,
    {"Maretron: Annunciator",
     130824,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {COMPANY(137),
      ONE_BYTE_FIELD("Field 4"),
      ONE_BYTE_FIELD("Field 5"),
      SIMPLE_FIELD("Field 6", BYTES(2)),
      ONE_BYTE_FIELD("Field 7"),
      SIMPLE_FIELD("Field 8", BYTES(2)),
      {0}}}

    /* Uwe Lovas has seen this from EP-70R */
    ,
    {"Lowrance: unknown",
     130827,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     10,
     0,
     {COMPANY(140),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      ONE_BYTE_FIELD("C"),
      ONE_BYTE_FIELD("D"),
      SIMPLE_FIELD("E", BYTES(2)),
      SIMPLE_FIELD("F", BYTES(2)),
      {0}}}

    ,
    {"Simnet: Set Serial Number", 130828, PACKET_INCOMPLETE | PACKET_NOT_SEEN, PACKET_FAST, 0x08, 0, {COMPANY(1857), {0}}}

    ,
    {"Suzuki: Engine and Storage Device Config",
     130831,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(586), {0}}}

    ,
    {"Simnet: Fuel Used - High Resolution", 130832, PACKET_INCOMPLETE | PACKET_NOT_SEEN, PACKET_FAST, 0x08, 0, {COMPANY(1857), {0}}}

    ,
    {"Simnet: Engine and Tank Configuration",
     130834,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1857), {0}}}

    ,
    {"Simnet: Set Engine and Tank Configuration",
     130835,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1857), {0}}}

    /* Seen when HDS8 configures EP65R */
    ,
    {"Simnet: Fluid Level Sensor Configuration",
     130836,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0e,
     0,
     {COMPANY(1857),
      ONE_BYTE_FIELD("C"),
      INTEGER_FIELD("Device", BYTES(1)),
      INSTANCE_FIELD,
      SIMPLE_FIELD("F", 1 * 4),
      LOOKUP_FIELD("Tank type", 1 * 4, TANK_TYPE),
      {"Capacity", BYTES(4), 0.1, false, 0, ""},
      ONE_BYTE_FIELD("G"),
      {"H", BYTES(2), 1, true, 0, ""},
      {"I", BYTES(1), 1, true, 0, ""},
      {0}}}

    ,
    {"Maretron Proprietary Switch Status Counter",
     130836,
     PACKET_COMPLETE,
     PACKET_FAST,
     16,
     0,
     {COMPANY(137),
      INSTANCE_FIELD,
      ONE_BYTE_FIELD("Indicator Number"),
      {"Start Date", BYTES(2), RES_DATE, false, "days", "Timestamp of last reset in Days since January 1, 1970"},
      TIME_FIELD("Start Time", "Timestamp of last reset Seconds since midnight"),
      INTEGER_FIELD("OFF Counter", BYTES(1)),
      INTEGER_FIELD("ON Counter", BYTES(1)),
      INTEGER_FIELD("ERROR Counter", BYTES(1)),
      LOOKUP_FIELD("Switch Status", 2, OFF_ON),
      RESERVED_FIELD(BYTES(2)),
      {0}}}

    ,
    {"Simnet: Fuel Flow Turbine Configuration",
     130837,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1857), {0}}}

    ,
    {"Maretron Proprietary Switch Status Timer",
     130837,
     PACKET_COMPLETE,
     PACKET_FAST,
     23,
     0,
     {COMPANY(137),
      INSTANCE_FIELD,
      ONE_BYTE_FIELD("Indicator Number"),
      {"Start Date", BYTES(2), RES_DATE, false, "days", "Timestamp of last reset in Days since January 1, 1970"},
      TIME_FIELD("Start Time", "Timestamp of last reset Seconds since midnight"),
      {"Accumulated OFF Period", BYTES(4), RES_DECIMAL, false, "seconds", ""},
      {"Accumulated ON Period", BYTES(4), RES_DECIMAL, false, "seconds", ""},
      {"Accumulated ERROR Period", BYTES(4), RES_DECIMAL, false, "seconds", ""},
      LOOKUP_FIELD("Switch Status", 2, OFF_ON),
      RESERVED_FIELD(6),
      {0}}}

    ,
    {"Simnet: Fluid Level Warning", 130838, PACKET_INCOMPLETE | PACKET_NOT_SEEN, PACKET_FAST, 0x08, 0, {COMPANY(1857), {0}}}

    ,
    {"Simnet: Pressure Sensor Configuration",
     130839,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1857), {0}}}

    ,
    {"Simnet: Data User Group Configuration",
     130840,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1857), {0}}}

    ,
    {"Simnet: AIS Class B static data (msg 24 Part A)",
     130842,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x1d,
     0,
     {COMPANY(1857),
      MATCH_FIELD("Message ID", 6, 0, "Msg 24 Part A"),
      LOOKUP_FIELD("Repeat indicator", 2, REPEAT_INDICATOR),
      ONE_BYTE_FIELD("D"),
      ONE_BYTE_FIELD("E"),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      ASCII_FIELD("Name", BYTES(20)),
      {0}}}

    ,
    {"Furuno: Six Degrees Of Freedom Movement",
     130842,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     29,
     0,
     {COMPANY(1855),
      {"A", BYTES(4), 1, true, 0, ""},
      {"B", BYTES(4), 1, true, 0, ""},
      {"C", BYTES(4), 1, true, 0, ""},
      {"D", BYTES(1), 1, true, 0, ""},
      {"E", BYTES(4), 1, true, 0, ""},
      {"F", BYTES(4), 1, true, 0, ""},
      {"G", BYTES(2), 1, true, 0, ""},
      {"H", BYTES(2), 1, true, 0, ""},
      {"I", BYTES(2), 1, true, 0, ""},
      {0}}}

    ,
    {"Simnet: AIS Class B static data (msg 24 Part B)",
     130842,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x25,
     0,
     {COMPANY(1857),
      MATCH_FIELD("Message ID", 6, 1, "Msg 24 Part B"),
      LOOKUP_FIELD("Repeat indicator", 2, REPEAT_INDICATOR),
      ONE_BYTE_FIELD("D"),
      ONE_BYTE_FIELD("E"),
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      LOOKUP_FIELD("Type of ship", BYTES(1), SHIP_TYPE),
      ASCII_FIELD("Vendor ID", BYTES(7)),
      ASCII_FIELD("Callsign", BYTES(7)),
      DECIMETERS_FIELD("Length"),
      DECIMETERS_FIELD("Beam"),
      DECIMETERS_FIELD("Position reference from Starboard"),
      DECIMETERS_FIELD("Position reference from Bow"),
      {"Mothership User ID", BYTES(4), RES_INTEGER, false, "MMSI", "Id of mother ship sent by daughter vessels"},
      RESERVED_FIELD(2),
      INTEGER_DESC_FIELD("Spare", 6, ",0=unavailable"),
      {0}}}

    ,
    {"Furuno: Heel Angle, Roll Information",
     130843,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1855),
      ONE_BYTE_FIELD("A"),
      ONE_BYTE_FIELD("B"),
      ANGLE_REL_FIELD("Yaw", ""),
      ANGLE_REL_FIELD("Pitch", ""),
      ANGLE_REL_FIELD("Roll", ""),
      {0}}}

    ,
    {"Simnet: Sonar Status, Frequency and DSP Voltage",
     130843,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1857), {0}}}

    ,
    {"Simnet: Compass Heading Offset",
     130845,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0e,
     0,
     {COMPANY(1857),
      SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat indicator", 2, REPEAT_INDICATOR),
      SIMPLE_FIELD("Unused", BYTES(3)),
      MATCH_FIELD("Type", BYTES(2), 0, "Heading Offset"),
      SIMPLE_FIELD("Unused B", BYTES(2)),
      ANGLE_REL_FIELD("Angle", ""),
      SIMPLE_FIELD("Unused C", BYTES(2)),
      {0}}}

    ,
    {"Furuno: Multi Sats In View Extended", 130845, PACKET_INCOMPLETE, PACKET_FAST, 0x08, 0, {COMPANY(1855), {0}}}

    ,
    {"Simnet: Compass Local Field",
     130845,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x0e,
     0,
     {COMPANY(1857),
      SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat indicator", 2, REPEAT_INDICATOR),
      SIMPLE_FIELD("Unused", BYTES(3)),
      MATCH_FIELD("Type", BYTES(2), 768, "Local field"),
      SIMPLE_FIELD("Unused B", BYTES(2)),
      PERCENTAGE_FIELD("Local field"),
      SIMPLE_FIELD("Unused C", BYTES(2)),
      {0}}}

    ,
    {"Simnet: Compass Field Angle",
     130845,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x0e,
     0,
     {COMPANY(1857),
      SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat indicator", 2, REPEAT_INDICATOR),
      SIMPLE_FIELD("Unused", BYTES(3)),
      MATCH_FIELD("Type", BYTES(2), 1024, "Local field"),
      SIMPLE_FIELD("Unused B", BYTES(2)),
      ANGLE_REL_FIELD("Field angle", ""),
      SIMPLE_FIELD("Unused C", BYTES(2)),
      {0}}}

    ,
    {"Simnet: Parameter Handle",
     130845,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x0e,
     0,
     {COMPANY(1857),
      SIMPLE_FIELD("Message ID", 6),
      LOOKUP_FIELD("Repeat indicator", 2, REPEAT_INDICATOR),
      ONE_BYTE_FIELD("D"),
      ONE_BYTE_FIELD("Group"),
      ONE_BYTE_FIELD("F"),
      ONE_BYTE_FIELD("G"),
      ONE_BYTE_FIELD("H"),
      ONE_BYTE_FIELD("I"),
      ONE_BYTE_FIELD("J"),
      LOOKUP_FIELD("Backlight", BYTES(1), SIMNET_BACKLIGHT_LEVEL),
      SIMPLE_FIELD("L", BYTES(2)),
      {0}}}

    ,
    {"Furuno: Motion Sensor Status Extended", 130846, PACKET_INCOMPLETE, PACKET_FAST, 0x08, 0, {COMPANY(1855), {0}}}

    ,
    {"SeaTalk: Node Statistics",
     130847,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0,
     0,
     {COMPANY(1851),
      SIMPLE_FIELD("Product Code", BYTES(2)),
      ONE_BYTE_FIELD("Year"),
      ONE_BYTE_FIELD("Month"),
      SIMPLE_FIELD("Device Number", BYTES(2)),
      VOLTAGE_FIELD("Node Voltage", 0.01),
      {0}}}

    ,
    {"Simnet: Event Command: AP command",
     130850,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     12,
     0,
     {COMPANY(1857),
      MATCH_FIELD("Proprietary ID", BYTES(1), 2, "AP command"),
      SIMPLE_FIELD("Unused A", BYTES(2)),
      ONE_BYTE_FIELD("Controlling Device"),
      LOOKUP_FIELD("Event", BYTES(1), SIMNET_AP_EVENTS),
      SIMPLE_FIELD("Unused B", BYTES(1)),
      LOOKUP_FIELD("Direction", BYTES(1), SIMNET_DIRECTION),
      ANGLE_POS_FIELD("Angle", ""),
      SIMPLE_FIELD("Unused C", BYTES(1)),
      {0}}}

    ,
    {"Simnet: Event Command: Alarm?",
     130850,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     12,
     0,
     {COMPANY(1857),
      SIMPLE_FIELD("A", BYTES(2)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 1, "Alarm command"),
      ONE_BYTE_FIELD("C"),
      INTEGER_FIELD("Alarm", BYTES(2)),
      INTEGER_FIELD("Message ID", BYTES(2)),
      ONE_BYTE_FIELD("F"),
      ONE_BYTE_FIELD("G"),
      {0}}}

    ,
    {"Simnet: Event Command: Unknown",
     130850,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     12,
     0,
     {COMPANY(1857),
      SIMPLE_FIELD("A", BYTES(2)),
      MATCH_FIELD("Proprietary ID", BYTES(1), 1, "Alarm command"),
      ONE_BYTE_FIELD("B"),
      SIMPLE_FIELD("C", BYTES(2)),
      SIMPLE_FIELD("D", BYTES(2)),
      SIMPLE_FIELD("E", BYTES(2)),
      {0}}}

    ,
    {"Simnet: Event Reply: AP command",
     130851,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     12,
     0,
     {COMPANY(1857),
      MATCH_FIELD("Proprietary ID", BYTES(1), 2, "AP command"),
      SIMPLE_FIELD("B", BYTES(2)),
      ONE_BYTE_FIELD("Controlling Device"),
      LOOKUP_FIELD("Event", BYTES(1), SIMNET_AP_EVENTS),
      ONE_BYTE_FIELD("C"),
      LOOKUP_FIELD("Direction", BYTES(1), SIMNET_DIRECTION),
      ANGLE_POS_FIELD("Angle", ""),
      ONE_BYTE_FIELD("G"),
      {0}}}

    ,
    {"Simnet: Alarm Message",
     130856,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {COMPANY(1857),
      SIMPLE_FIELD("Message ID", BYTES(2)),
      ONE_BYTE_FIELD("B"),
      ONE_BYTE_FIELD("C"),
      ASCII_FIELD("Text", BYTES(255)),
      {0}}}

    ,
    {"Airmar: Additional Weather Data",
     130880,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x1e,
     0,
     {COMPANY(135),
      ONE_BYTE_FIELD("C"),
      TEMPERATURE_FIELD("Apparent Windchill Temperature"),
      TEMPERATURE_FIELD("True Windchill Temperature"),
      TEMPERATURE_FIELD("Dewpoint"),
      {0}}}

    ,
    {"Airmar: Heater Control",
     130881,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x9,
     0,
     {COMPANY(135),
      ONE_BYTE_FIELD("C"),
      TEMPERATURE_FIELD("Plate Temperature"),
      TEMPERATURE_FIELD("Air Temperature"),
      TEMPERATURE_FIELD("Dewpoint"),
      {0}}}

    ,
    {"Airmar: POST",
     130944,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x8,
     0,
     {COMPANY(135),
      LOOKUP_FIELD("Control", 1, AIRMAR_POST_CONTROL),
      RESERVED_FIELD(7),
      INTEGER_FIELD("Number of ID/test result pairs to follow", BYTES(1)),
      LOOKUP_FIELD_DESC("Test ID",
                        BYTES(1),
                        AIRMAR_POST_ID,
                        "See Airmar docs for table of IDs and failure codes; these lookup values are for DST200"),
      INTEGER_DESC_FIELD("Test result", BYTES(1), "Values other than 0 are failure codes"),
      {0}}}

    ,
    {"Actisense: Operating mode",
     ACTISENSE_BEM + 0x11,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     0x0e,
     0,
     {ONE_BYTE_FIELD("SID"),
      INTEGER_FIELD("Model ID", BYTES(2)),
      INTEGER_FIELD("Serial ID", BYTES(4)),
      INTEGER_FIELD("Error ID", BYTES(4)),
      SIMPLE_FIELD("Operating Mode", BYTES(2)),
      {0}}}

    ,
    {"Actisense: Startup status",
     ACTISENSE_BEM + 0xf0,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0f,
     0,
     {ONE_BYTE_FIELD("SID"),
      INTEGER_FIELD("Model ID", BYTES(2)),
      INTEGER_FIELD("Serial ID", BYTES(4)),
      INTEGER_FIELD("Error ID", BYTES(4)),
      {"Firmware version", BYTES(2), 0.001, false, 0, ""},
      ONE_BYTE_FIELD("Reset status"),
      ONE_BYTE_FIELD("A"),
      {0}}}

    ,
    {"Actisense: System status",
     ACTISENSE_BEM + 0xf2,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x22,
     0,
     {ONE_BYTE_FIELD("SID"),
      INTEGER_FIELD("Model ID", BYTES(2)),
      INTEGER_FIELD("Serial ID", BYTES(4)),
      INTEGER_FIELD("Error ID", BYTES(4)),
      ONE_BYTE_FIELD("Indi channel count"),
      ONE_BYTE_FIELD("Ch1 Rx Bandwidth"),
      ONE_BYTE_FIELD("Ch1 Rx Load"),
      ONE_BYTE_FIELD("Ch1 Rx Filtered"),
      ONE_BYTE_FIELD("Ch1 Rx Dropped"),
      ONE_BYTE_FIELD("Ch1 Tx Bandwidth"),
      ONE_BYTE_FIELD("Ch1 Tx Load"),
      ONE_BYTE_FIELD("Ch2 Rx Bandwidth"),
      ONE_BYTE_FIELD("Ch2 Rx Load"),
      ONE_BYTE_FIELD("Ch2 Rx Filtered"),
      ONE_BYTE_FIELD("Ch2 Rx Dropped"),
      ONE_BYTE_FIELD("Ch2 Tx Bandwidth"),
      ONE_BYTE_FIELD("Ch2 Tx Load"),
      ONE_BYTE_FIELD("Uni channel count"),
      ONE_BYTE_FIELD("Ch1 Bandwidth"),
      ONE_BYTE_FIELD("Ch1 Deleted"),
      ONE_BYTE_FIELD("Ch1 BufferLoading"),
      ONE_BYTE_FIELD("Ch1 PointerLoading"),
      ONE_BYTE_FIELD("Ch2 Bandwidth"),
      ONE_BYTE_FIELD("Ch2 Deleted"),
      ONE_BYTE_FIELD("Ch2 BufferLoading"),
      ONE_BYTE_FIELD("Ch2 PointerLoading"),
      {0}}}

    ,
    {"Actisense: ?",
     ACTISENSE_BEM + 0xf4,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     17,
     0,
     {ONE_BYTE_FIELD("SID"), INTEGER_FIELD("Model ID", BYTES(2)), INTEGER_FIELD("Serial ID", BYTES(4)), {0}}}

    ,
    {"iKonvert: Network status",
     IKONVERT_BEM,
     PACKET_COMPLETE,
     PACKET_FAST,
     15,
     0,
     {INTEGER_FIELD("CAN network load", BYTES(1)),
      INTEGER_FIELD("Errors", BYTES(4)),
      INTEGER_FIELD("Device count", BYTES(1)),
      INTEGER_DESC_FIELD("Uptime", BYTES(4), "s"),
      INTEGER_FIELD("Gateway address", BYTES(1)),
      INTEGER_FIELD("Rejected TX requests", BYTES(4)),
      {0}}}};

size_t pgnListSize = ARRAY_SIZE(pgnList);

#else
extern Pgn    pgnList[];
extern size_t pgnListSize;
#endif
