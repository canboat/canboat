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

#ifndef FIELD_H_INCLUDED
#define FIELD_H_INCLUDED

#include <math.h>

#include "common.h"

typedef bool (*FieldPrintFunctionType)(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);

extern bool fieldPrintBinary(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintBitLookup(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintDate(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintDecimal(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintFloat(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintLatLon(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintLookup(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintMMSI(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintNumber(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintReserved(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintSpare(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintStringFix(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintStringLAU(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintStringLZ(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintTime(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintVariable(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);

typedef enum Bool
{
  Null,
  False,
  True
} Bool;

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

static const PhysicalQuantity ELECTRICAL_CURRENT = {
    .name         = "ELECTRICAL_CURRENT",
    .description  = "Electrical current",
    .abbreviation = "A",
    .unit         = "Ampere",
    .url          = "https://en.wikipedia.org/wiki/Electric_current",
};

static const PhysicalQuantity ELECTRICAL_CHARGE = {.name         = "ELECTRICAL_CHARGE",
                                                   .description  = "Electrical charge",
                                                   .abbreviation = "C",
                                                   .unit         = "Coulomb",
                                                   .url          = "https://en.wikipedia.org/wiki/Electric_charge"};

static const PhysicalQuantity ELECTRICAL_ENERGY
    = {.name        = "ELECTRICAL_ENERGY",
       .description = "Electrical energy",
       .comment = "The amount of electricity used or stored. The base unit used in NMEA 2000 is the Kilo Watt Hour (kWh) which is "
                  "equivalent to 3.6e6 J (Joules).",
       .unit    = "Kilo Watt Hour",
       .abbreviation = "kWh",
       .url          = "https://en.wikipedia.org/wiki/Electrical_energy"};

static const PhysicalQuantity ELECTRICAL_POWER = {.name         = "ELECTRICAL_POWER",
                                                  .description  = "Electrical power",
                                                  .comment      = "The amount of energy transferred or converted per unit time.",
                                                  .unit         = "Watt",
                                                  .abbreviation = "W",
                                                  .url          = "https://en.wikipedia.org/wiki/Electrical_power"};

static const PhysicalQuantity ELECTRICAL_APPARENT_POWER
    = {.name         = "ELECTRICAL_APPARENT_POWER",
       .description  = "AC apparent power",
       .comment      = "The amount of power transferred where the current and voltage are in phase.",
       .unit         = "Volt Ampere",
       .abbreviation = "VA",
       .url          = "https://en.wikipedia.org/wiki/Volt-ampere"};

static const PhysicalQuantity ELECTRICAL_REACTIVE_POWER
    = {.name         = "ELECTRICAL_REACTIVE_POWER",
       .description  = "AC reactive power",
       .comment      = "The amount of power transferred where the current and voltage are not in phase.",
       .unit         = "Volt Ampere Reactive",
       .abbreviation = "VAR",
       .url          = "https://en.wikipedia.org/wiki/Volt-ampere#Reactive"};

static const PhysicalQuantity POTENTIAL_DIFFERENCE = {.name         = "POTENTIAL_DIFFERENCE",
                                                      .description  = "Potential difference",
                                                      .abbreviation = "V",
                                                      .unit         = "Volt",
                                                      .url          = "https://en.wikipedia.org/wiki/Voltage"};

static const PhysicalQuantity POWER_FACTOR
    = {.name        = "POWER_FACTOR",
       .description = "Power Factor",
       .comment = "Used in AC circuits only, the ratio of the real power absorbed by the load to the apparent power flowing in the "
                  "circuit. If less than one, the voltage and current are not in phase.",
       .unit    = "Cos(Phi)",
       .abbreviation = "Cos Phi",
       .url          = "https://en.wikipedia.org/wiki/Power_factor"};

static const PhysicalQuantity LENGTH = {.name         = "LENGTH",
                                        .description  = "Length",
                                        .comment      = "The physical size in one dimension of an object.",
                                        .unit         = "Meter",
                                        .abbreviation = "m",
                                        .url          = "https://en.wikipedia.org/wiki/Length"};

static const PhysicalQuantity DISTANCE = {.name         = "DISTANCE",
                                          .description  = "Distance",
                                          .comment      = "The amount of separation between two objects.",
                                          .unit         = "meter",
                                          .abbreviation = "m",
                                          .url          = "https://en.wikipedia.org/wiki/Distance"};

static const PhysicalQuantity SPEED = {.name         = "SPEED",
                                       .description  = "Speed",
                                       .comment      = "The velocity, or length per unit of time.",
                                       .abbreviation = "m/s",
                                       .unit         = "meter per second",
                                       .url          = "https://en.wikipedia.org/wiki/Speed"};

static const PhysicalQuantity ANGLE
    = {.name         = "ANGLE",
       .description  = "Angle",
       .comment      = "All standardized PGNs seen so far all use radians, but some manufacturer specific PGNs use degrees (deg).",
       .url          = "https://en.wikipedia.org/wiki/Angle",
       .abbreviation = "rad",
       .unit         = "radian"};

static const PhysicalQuantity ANGLE_DEG = {.name         = "ANGLE_DEG",
                                           .description  = "Angle",
                                           .url          = "https://en.wikipedia.org/wiki/Angle",
                                           .abbreviation = "deg",
                                           .unit         = "degree"};

static const PhysicalQuantity ANGULAR_VELOCITY = {.name         = "ANGULAR_VELOCITY",
                                                  .description  = "Angular velocity",
                                                  .comment      = "The speed at which a measured angle changes",
                                                  .url          = "https://en.wikipedia.org/wiki/Angular_velocity",
                                                  .abbreviation = "rad/s",
                                                  .unit         = "radians per second"};

static const PhysicalQuantity VOLUME = {.name         = "VOLUME",
                                        .description  = "Volume",
                                        .comment      = "A measure of occupied three-dimensional space.",
                                        .url          = "https://en.wikipedia.org/wiki/Volume",
                                        .abbreviation = "L",
                                        .unit         = "liter"};

static const PhysicalQuantity VOLUMETRIC_FLOW = {.name         = "VOLUMETRIC_FLOW",
                                                 .description  = "Volumetric flow",
                                                 .comment      = "The volume of fluid which passes per unit time.",
                                                 .abbreviation = "L/h",
                                                 .unit         = "liter per hour",
                                                 .url          = "https://en.wikipedia.org/wiki/Volumetric_flow_rate"};

static const PhysicalQuantity FREQUENCY = {.name         = "FREQUENCY",
                                           .description  = "Frequency",
                                           .abbreviation = "Hz",
                                           .unit         = "Hertz",
                                           .url          = "https://en.wikipedia.org/wiki/Radio_frequency"};

static const PhysicalQuantity DATE = {.name         = "DATE",
                                      .description  = "Date",
                                      .comment      = "A calendar date is a reference to a particular day in time, in NMEA 2000 "
                                                      "expressed as the number of days since 1970-01-01 (UNIX epoch).",
                                      .abbreviation = "d",
                                      .unit         = "days",
                                      .url          = "https://en.wikipedia.org/wiki/Calendar_date"};

static const PhysicalQuantity TIME
    = {.name        = "TIME",
       .description = "Time",
       .comment
       = "Time is what clocks measure. We use time to place events in sequence one after the other, and we use time to compare how "
         "long events last. Absolute times in NMEA2000 are expressed as seconds since midnight(in an undefined timezone)",
       .url          = "https://en.wikipedia.org/wiki/Time",
       .abbreviation = "s",
       .unit         = "Second"};

static const PhysicalQuantity MAGNETIC_FIELD = {.name         = "MAGNETIC_FIELD",
                                                .description  = "Magnetic field",
                                                .unit         = "Tesla",
                                                .abbreviation = "T",
                                                .url          = "https://en.wikipedia.org/wiki/Magnetic_field"};

static const PhysicalQuantity GEO_COORDINATE
    = {.name         = "GEOGRAPHICAL_COORDINATE",
       .description  = "Geographical coordinate",
       .comment      = "Latitude or longitude. Combined they form a unique point on earth, when height is disregarded.",
       .abbreviation = "deg",
       .unit         = "degree",
       .url          = "https://en.wikipedia.org/wiki/Geographic_coordinate_system"};

static const PhysicalQuantity TEMPERATURE = {.name         = "TEMPERATURE",
                                             .description  = "Temperature",
                                             .unit         = "Kelvin",
                                             .abbreviation = "K",
                                             .url          = "https://en.wikipedia.org/wiki/Temperature"};

static const PhysicalQuantity PRESSURE = {.name         = "PRESSURE",
                                          .description  = "Pressure",
                                          .abbreviation = "Pa",
                                          .unit         = "Pascal",
                                          .url          = "https://en.wikipedia.org/wiki/Pressure"};

static const PhysicalQuantity PRESSURE_RATE = {.name         = "PRESSURE_RATE",
                                               .description  = "Pressure rate",
                                               .comment      = "How the pressure changes over time.",
                                               .abbreviation = "Pa/hr",
                                               .unit         = "Pascal per hour",
                                               .url          = "https://en.wikipedia.org/wiki/Pressure"};

static const PhysicalQuantity CONCENTRATION
    = {.name         = "CONCENTRATION",
       .description  = "Concentration of one substance in another, in this marine context usually the amount of salts in water",
       .url          = "https://www.engineeringtoolbox.com/water-salinity-d_1251.html",
       .unit         = "parts per million",
       .abbreviation = "ppm"};

static const PhysicalQuantity SIGNAL_TO_NOISE_RATIO = {.name         = "SIGNAL_TO_NOISE_RATIO",
                                                       .description  = "Signal-to-noise ratio",
                                                       .url          = "https://en.wikipedia.org/wiki/Signal-to-noise_ratio",
                                                       .abbreviation = "dB",
                                                       .unit         = "decibel"};

const PhysicalQuantity *const PhysicalQuantityList[] = {&ELECTRICAL_CURRENT,
                                                        &ELECTRICAL_CHARGE,
                                                        &ELECTRICAL_ENERGY,
                                                        &ELECTRICAL_POWER,
                                                        &ELECTRICAL_APPARENT_POWER,
                                                        &ELECTRICAL_REACTIVE_POWER,
                                                        &POTENTIAL_DIFFERENCE,
                                                        &POWER_FACTOR,
                                                        &LENGTH,
                                                        &DISTANCE,
                                                        &SPEED,
                                                        &ANGLE,
                                                        &ANGULAR_VELOCITY,
                                                        &VOLUME,
                                                        &VOLUMETRIC_FLOW,
                                                        &MAGNETIC_FIELD,
                                                        &FREQUENCY,
                                                        &DATE,
                                                        &TIME,
                                                        &GEO_COORDINATE,
                                                        &TEMPERATURE,
                                                        &PRESSURE,
                                                        &PRESSURE_RATE,
                                                        &CONCENTRATION,
                                                        &SIGNAL_TO_NOISE_RATIO,
                                                        NULL};

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

  // How to print this field
  FieldPrintFunctionType  pf;
  const PhysicalQuantity *physical;

  // Filled by initializer
  FieldType *baseFieldTypePtr;
};

#ifdef FIELDTYPE_GLOBALS
FieldType fieldTypeList[] = {
    // Numeric types
    {.name        = "NUMBER",
     .description = "Number",
     .encodingDescription
     = "Binary numbers are little endian. Number fields that use two or three bits use one special encoding, for the maximum "
       "value.  When present, this means that the field is not present. Number fields that use four bits or more use two special "
       "encodings. The maximum positive value means that the field is not present. The maximum positive value minus 1 means that "
       "the field has an error. For instance, a broken sensor. For signed numbers the maximum values are the maximum positive "
       "value and that minus 1, not the all-ones bit encoding which is the maximum negative value.",
     .url = "https://en.wikipedia.org/wiki/Binary_number",
     .pf  = fieldPrintNumber},

    {.name          = "INTEGER",
     .description   = "Signed integral number",
     .hasSign       = True,
     .baseFieldType = "NUMBER",
     .url           = "https://en.wikipedia.org/wiki/Integer_%28computer_science%29",
     .v1Type        = "Integer"},

    {.name          = "UNSIGNED_INTEGER",
     .description   = "Unsigned integral number",
     .hasSign       = False,
     .baseFieldType = "NUMBER",
     .url           = "https://en.wikipedia.org/wiki/Integer_%28computer_science%29",
     .v1Type        = "Integer"},

    {.name = "INT8", .description = "8 bit signed integer", .size = 8, .hasSign = True, .baseFieldType = "INTEGER"},

    {.name = "UINT8", .description = "8 bit unsigned integer", .size = 8, .hasSign = False, .baseFieldType = "UNSIGNED_INTEGER"},

    {.name = "INT16", .description = "16 bit signed integer", .size = 16, .hasSign = True, .baseFieldType = "INTEGER"},

    {.name = "UINT16", .description = "16 bit unsigned integer", .size = 16, .hasSign = False, .baseFieldType = "UNSIGNED_INTEGER"},

    {.name = "UINT24", .description = "24 bit unsigned integer", .size = 24, .hasSign = False, .baseFieldType = "UNSIGNED_INTEGER"},

    {.name = "INT32", .description = "32 bit signed integer", .size = 32, .hasSign = True, .baseFieldType = "INTEGER"},

    {.name = "UINT32", .description = "32 bit unsigned integer", .size = 32, .hasSign = False, .baseFieldType = "UNSIGNED_INTEGER"},

    {.name = "INT64", .description = "64 bit signed integer", .size = 64, .hasSign = True, .baseFieldType = "INTEGER"},

    {.name = "UINT64", .description = "64 bit unsigned integer", .size = 64, .hasSign = False, .baseFieldType = "UNSIGNED_INTEGER"},

    {.name        = "UNSIGNED_FIXED_POINT_NUMBER",
     .description = "An unsigned numeric value where the Least Significant Bit does not encode the integer value 1",
     .encodingDescription
     = "The `Resolution` attribute indicates what the raw value 1 should represent. The `Signed` and `BitLength` attributes are "
       "always present. Together, this gives sufficient information to represent a fixed point number in a particular range where "
       "non-integral values can be encoded without requiring four or eight bytes for a floating point number.",
     .hasSign       = False,
     .url           = "https://en.wikipedia.org/wiki/Fixed-point_arithmetic",
     .baseFieldType = "NUMBER"},

    {.name        = "SIGNED_FIXED_POINT_NUMBER",
     .description = "A signed numeric value where the Least Significant Bit does not encode the integer value 1",
     .encodingDescription
     = "The `Resolution` attribute indicates what the raw value 1 should represent. The `Signed` and `BitLength` attributes are "
       "always present. Together, this gives sufficient information to represent a fixed point number in a particular range where "
       "non-integral values can be encoded without requiring four or eight bytes for a floating point number.",
     .hasSign       = True,
     .url           = "https://en.wikipedia.org/wiki/Fixed-point_arithmetic",
     .baseFieldType = "NUMBER"},

    {.name = "FIX8", .description = "8 bit signed fixed point number", .size = 8, .baseFieldType = "SIGNED_FIXED_POINT_NUMBER"},

    {.name          = "UFIX8",
     .description   = "8 bit unsigned fixed point number",
     .size          = 8,
     .baseFieldType = "UNSIGNED_FIXED_POINT_NUMBER"},

    {.name = "FIX16", .description = "16 bit signed fixed point number", .size = 16, .baseFieldType = "SIGNED_FIXED_POINT_NUMBER"},

    {.name          = "UFIX16",
     .description   = "16 bit unsigned fixed point number",
     .size          = 16,
     .baseFieldType = "UNSIGNED_FIXED_POINT_NUMBER"},

    {.name = "FIX24", .description = "24 bit signed fixed point number", .size = 24, .baseFieldType = "SIGNED_FIXED_POINT_NUMBER"},

    {.name          = "UFIX24",
     .description   = "24 bit unsigned fixed point number",
     .size          = 24,
     .baseFieldType = "UNSIGNED_FIXED_POINT_NUMBER"},

    {.name = "FIX32", .description = "32 bit signed fixed point number", .size = 32, .baseFieldType = "SIGNED_FIXED_POINT_NUMBER"},

    {.name          = "UFIX32",
     .description   = "32 bit unsigned fixed point number",
     .size          = 32,
     .baseFieldType = "UNSIGNED_FIXED_POINT_NUMBER"},

    {.name = "FIX64", .description = "64 bit signed fixed point number", .size = 64, .baseFieldType = "SIGNED_FIXED_POINT_NUMBER"},

    {.name          = "UFIX64",
     .description   = "64 bit unsigned fixed point number",
     .size          = 64,
     .baseFieldType = "UNSIGNED_FIXED_POINT_NUMBER"},

    {.name        = "FLOAT",
     .description = "32 bit IEEE-754 floating point number",
     .size        = 32,
     .hasSign     = True,
     .url         = "https://en.wikipedia.org/wiki/IEEE_754",
     .pf          = fieldPrintFloat},

    {.name                = "DECIMAL",
     .description         = "A unsigned numeric value represented with 2 decimal digits per byte",
     .encodingDescription = "Each byte represent 2 digits, so 1234 is represented by 2 bytes containing 0x12 and 0x34. A number "
                            "with an odd number of digits will have 0 as the first digit in the first byte.",
     .hasSign             = False,
     .url                 = "https://en.wikipedia.org/wiki/Binary-coded_decimal",
     .pf                  = fieldPrintDecimal},

    {.name                = "LOOKUP",
     .description         = "Number value where each value encodes for a distinct meaning",
     .encodingDescription = "Each lookup has a LookupEnumeration defining what the possible values mean",
     .comment = "For almost all lookups the list of values is known with some precision, but it is quite possible that a value "
                "occurs that has no corresponding textual explanation.",
     .hasSign = False,
     .pf      = fieldPrintLookup,
     .v1Type  = "Lookup table"},

    {.name = "INDIRECT_LOOKUP",
     .description
     = "Number value where each value encodes for a distinct meaning but the meaning also depends on the value in another field",
     .encodingDescription = "Each lookup has a LookupIndirectEnumeration defining what the possible values mean",
     .comment = "For almost all lookups the list of values is known with some precision, but it is quite possible that a value "
                "occurs that has no corresponding textual explanation.",
     .hasSign = False,
     .pf      = fieldPrintLookup,
     .v1Type  = "Integer"},

    {.name                = "BITLOOKUP",
     .description         = "Number value where each bit value encodes for a distinct meaning",
     .encodingDescription = "Each LookupBit has a LookupBitEnumeration defining what the possible values mean. A bitfield can have "
                            "any combination of bits set.",
     .comment = "For almost all lookups the list of values is known with some precision, but it is quite possible that a value "
                "occurs that has no corresponding textual explanation.",
     .pf      = fieldPrintBitLookup,
    .v1Type   = "Bitfield"},

    {.name          = "MANUFACTURER",
     .description   = "Manufacturer",
     .size          = 11,
     .pf            = fieldPrintLookup,
     .baseFieldType = "LOOKUP",
     .v1Type        = "Manufacturer code"},

    {.name = "INDUSTRY", .description = "Industry", .size = 3, .pf = fieldPrintLookup, .baseFieldType = "LOOKUP"},

    {.name = "VERSION", .description = "Version", .resolution = 0.001, .baseFieldType = "UFIX16"},

    // Specific typed numeric fields

    {.name          = "DILUTION_OF_PRECISION_FIX16",
     .description   = "Dilution of precision",
     .url           = "https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)",
     .resolution    = 0.01,
     .baseFieldType = "FIX16"},

    {.name          = "DILUTION_OF_PRECISION_UFIX16",
     .description   = "Dilution of precision",
     .url           = "https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)",
     .resolution    = 0.01,
     .baseFieldType = "UFIX16"},

    {.name          = "SIGNALTONOISERATIO_FIX16",
     .description   = "Signal-to-noise ratio",
     .url           = "https://en.wikipedia.org/wiki/Signal-to-noise_ratio",
     .resolution    = 0.01,
     .physical      = &SIGNAL_TO_NOISE_RATIO,
     .baseFieldType = "FIX16"},

    {.name          = "SIGNALTONOISERATIO_UFIX16",
     .description   = "Signal-to-noise ratio",
     .url           = "https://en.wikipedia.org/wiki/Signal-to-noise_ratio",
     .resolution    = 0.01,
     .physical      = &SIGNAL_TO_NOISE_RATIO,
     .baseFieldType = "UFIX16"},

    {.name = "ANGLE_FIX16", .description = "Angle", .resolution = 0.0001, .physical = &ANGLE, .baseFieldType = "FIX16"},

    {.name          = "ANGLE_FIX16_DDEG",
     .description   = "Angle",
     .resolution    = 0.1,
     .unit          = "deg",
     .physical      = &ANGLE,
     .baseFieldType = "FIX16"},

    {.name = "ANGLE_UFIX16", .description = "Angle", .resolution = 0.0001, .physical = &ANGLE, .baseFieldType = "UFIX16"},

    {.name        = "GEO_FIX32",
     .description = "Geographical latitude or longitude",
     .encodingDescription
     = "The `Resolution` for this field is 1.0e-7, so the resolution is 1/10 millionth of a degree, or about 1 "
       "cm when we refer to an Earth position",
     .resolution    = 1.0e-7,
     .physical      = &GEO_COORDINATE,
     .pf            = fieldPrintLatLon,
     .baseFieldType = "FIX32",
     .v1Type        = "Lat/Lon"},

    {.name                = "GEO_FIX64",
     .description         = "Geographical latitude or longitude, high resolution",
     .encodingDescription = "The `Resolution` for this field is 1.0e-16, so the resolution is about 0.01 nm (nanometer) when we "
                            "refer to an Earth position",
     .resolution          = 1.0e-16,
     .physical            = &GEO_COORDINATE,
     .pf                  = fieldPrintLatLon,
     .baseFieldType       = "FIX64",
     .v1Type              = "Lat/Lon"},

    {.name          = "LENGTH_UFIX8_DAM",
     .description   = "Length, in decameter resolution",
     .resolution    = 10,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX8"},

    {.name          = "LENGTH_UFIX16_DM",
     .description   = "Length, in decimeter resolution",
     .resolution    = 0.1,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX16"},

    {.name          = "LENGTH_UFIX32_CM",
     .description   = "Length, in centimeter resolution",
     .resolution    = 0.01,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX32_M",
     .description   = "Length, in meter resolution",
     .resolution    = 1,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX32_CM",
     .description   = "Length, in centimeter resolution",
     .resolution    = 0.01,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX32_MM",
     .description   = "Length, in millimeter resolution",
     .resolution    = 0.001,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX8_DAM",
     .description   = "Length, byte, unsigned decameters",
     .resolution    = 10.,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX8"},

    {.name          = "LENGTH_UFIX16_CM",
     .description   = "Length, unsigned centimeters",
     .resolution    = 0.01,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX16"},

    {.name          = "LENGTH_UFIX16_DM",
     .description   = "Length, unsigned decimeters",
     .resolution    = 0.1,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX16"},

    {.name          = "LENGTH_UFIX32_MM",
     .description   = "Length, high range, unsigned millimeters",
     .resolution    = 0.001,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX32_CM",
     .description   = "Length, high range, unsigned centimeters",
     .resolution    = 0.01,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX32_M",
     .description   = "Length, high range, meters",
     .resolution    = 1.,
     .physical      = &LENGTH,
     .baseFieldType = "UFIX32"},

    {.name          = "TEMPERATURE",
     .description   = "Temperature",
     .resolution    = 0.01,
     .physical      = &TEMPERATURE,
     .baseFieldType = "UFIX16",
     .v1Type        = "Temperature"},

    {.name                = "TEMPERATURE_HIGH",
     .description         = "Temperature, high range",
     .encodingDescription = "This has a higher range but lower resolution than TEMPERATURE",
     .resolution          = 0.1,
     .physical            = &TEMPERATURE,
     .baseFieldType       = "UFIX16",
     .v1Type              = "Temperature"},

    {.name                = "TEMPERATURE_UFIX24",
     .description         = "Temperature, high resolution",
     .encodingDescription = "This has a higher range and higher resolution than TEMPERATURE (but uses three bytes)",
     .resolution          = 0.001,
     .physical            = &TEMPERATURE,
     .baseFieldType       = "UFIX24",
     .v1Type              = "Temperature"},

    {.name          = "TEMPERATURE_DELTA_FIX16",
     .description   = "Temperature difference",
     .resolution    = 0.001,
     .physical      = &TEMPERATURE,
     .baseFieldType = "FIX16"},

    {.name          = "VOLUMETRIC_FLOW",
     .description   = "Volumetric flow",
     .resolution    = 0.1,
     .physical      = &VOLUMETRIC_FLOW,
     .baseFieldType = "FIX16"},

    {.name                = "CONCENTRATION_UINT16_PPM",
     .description         = "Concentration of one substance in another, in this context usually the amount of salts in water",
     .encodingDescription = "Expressed in parts per million",
     .resolution          = 1,
     .physical            = &CONCENTRATION,
     .baseFieldType       = "UINT16"},

    {.name = "VOLUME_UFIX16_L", .description = "Volume", .resolution = 1, .physical = &VOLUME, .baseFieldType = "UFIX16"},

    {.name = "VOLUME_UFIX32_DL", .description = "Volume", .resolution = 0.1, .physical = &VOLUME, .baseFieldType = "UFIX32"},

    {.name = "TIME", .description = "Time", .physical = &TIME, .pf = fieldPrintTime, .v1Type = "Time"},

    {.name                = "TIME_UFIX32",
     .description         = "Time",
     .encodingDescription = "When indicating a wall clock time, this is the amount of time passed since midnight",
     .size                = 32,
     .hasSign             = False,
     .resolution          = 0.0001,
     .baseFieldType       = "TIME"},

    {.name          = "TIME_UFIX16_S",
     .description   = "Time delta, 16 bits with 1 second resolution",
     .resolution    = 1,
     .size          = 16,
     .hasSign       = False,
     .baseFieldType = "TIME"},

    {.name          = "TIME_UFIX8_5MS",
     .description   = "Time delta, 8 bits with 5 millisecond resolution",
     .resolution    = 0.005,
     .size          = 8,
     .hasSign       = False,
     .baseFieldType = "TIME"},

    {.name          = "TIME_UFIX8_P12S",
     .description   = "Time delta, 8 bits with 2^12 second resolution",
     .resolution    = POW2(12),
     .size          = 8,
     .hasSign       = False,
     .baseFieldType = "TIME"},

    {.name          = "TIME_UFIX16_MS",
     .description   = "Time delta, 16 bits with millisecond resolution",
     .resolution    = 0.001,
     .size          = 16,
     .hasSign       = False,
     .baseFieldType = "TIME"},

    {.name          = "TIME_UFIX16_CS",
     .description   = "Time delta, 16 bits with centisecond resolution",
     .resolution    = 0.01,
     .size          = 16,
     .hasSign       = False,
     .baseFieldType = "TIME"},

    {.name          = "TIME_UFIX16_MIN",
     .description   = "Time delta, 16 bits with minute resolution",
     .resolution    = 60,
     .size          = 16,
     .hasSign       = False,
     .baseFieldType = "TIME"},

    {.name          = "TIME_UFIX24_MS",
     .description   = "Time delta, 24 bits with millisecond resolution",
     .resolution    = 0.001,
     .size          = 24,
     .hasSign       = False,
     .baseFieldType = "TIME"},

    {.name          = "TIME_UFIX32_S",
     .description   = "Time delta, 32 bits with second resolution",
     .resolution    = 1,
     .size          = 32,
     .hasSign       = False,
     .baseFieldType = "TIME"},

    {.name          = "TIME_UFIX32_MS",
     .description   = "Time delta, 32 bits with millisecond resolution",
     .resolution    = 0.001,
     .size          = 32,
     .hasSign       = False,
     .baseFieldType = "TIME"},

    {.name          = "TIME_FIX32_MS",
     .description   = "Time delta",
     .resolution    = 0.001,
     .size          = 32,
     .hasSign       = True,
     .baseFieldType = "TIME"},

    {.name          = "TIME_FIX16_5CS",
     .description   = "Time delta, 5 centisecond resolution",
     .resolution    = 0.05,
     .size          = 16,
     .hasSign       = True,
     .baseFieldType = "TIME"},

    {.name          = "TIME_FIX16_MIN",
     .description   = "Time delta, minute resolution",
     .resolution    = 60,
     .size          = 16,
     .hasSign       = True,
     .baseFieldType = "TIME",
     .v1Type        = "Integer"},

    {.name                = "DATE",
     .description         = "Date",
     .encodingDescription = "The date, in days since 1 January 1970.",
     .physical            = &DATE,
     .size                = 16,
     .hasSign             = False,
     .pf                  = fieldPrintDate,
     .v1Type              = "Date"},

    {.name          = "VOLTAGE_UFIX16_10MV",
     .description   = "Voltage",
     .resolution    = 0.01,
     .physical      = &POTENTIAL_DIFFERENCE,
     .baseFieldType = "UFIX16"},

    {.name          = "VOLTAGE_UFIX16_100MV",
     .description   = "Voltage",
     .resolution    = 0.1,
     .physical      = &POTENTIAL_DIFFERENCE,
     .baseFieldType = "UFIX16"},

    {.name          = "VOLTAGE_UFIX8_200MV",
     .description   = "Voltage",
     .resolution    = 0.2,
     .physical      = &POTENTIAL_DIFFERENCE,
     .baseFieldType = "UFIX8"},

    {.name          = "VOLTAGE_UFIX16_V",
     .description   = "Voltage",
     .resolution    = 1,
     .physical      = &POTENTIAL_DIFFERENCE,
     .baseFieldType = "UFIX16"},

    {.name          = "VOLTAGE_FIX16_10MV",
     .description   = "Voltage, signed",
     .resolution    = 0.01,
     .physical      = &POTENTIAL_DIFFERENCE,
     .baseFieldType = "FIX16"},

    {.name          = "CURRENT",
     .description   = "Electrical current",
     .hasSign       = False,
     .physical      = &ELECTRICAL_CURRENT,
     .baseFieldType = "UNSIGNED_FIXED_POINT_NUMBER"},

    {.name          = "CURRENT_UFIX8_A",
     .description   = "Electrical current",
     .resolution    = 1,
     .physical      = &ELECTRICAL_CURRENT,
     .baseFieldType = "UFIX8"},

    {.name          = "CURRENT_UFIX16_A",
     .description   = "Electrical current",
     .resolution    = 1,
     .physical      = &ELECTRICAL_CURRENT,
     .baseFieldType = "UFIX16"},

    {.name          = "CURRENT_UFIX16_DA",
     .description   = "Electrical current",
     .resolution    = .1,
     .physical      = &ELECTRICAL_CURRENT,
     .baseFieldType = "UFIX16"},

    {.name          = "CURRENT_FIX16_DA",
     .description   = "Electrical current",
     .resolution    = .1,
     .physical      = &ELECTRICAL_CURRENT,
     .baseFieldType = "FIX16"},

    {.name          = "CURRENT_FIX24_CA",
     .description   = "Electrical current",
     .resolution    = .01,
     .physical      = &ELECTRICAL_CURRENT,
     .baseFieldType = "FIX24"},

    {.name          = "ELECTRIC_CHARGE_UFIX16_AH",
     .description   = "Electrical charge",
     .resolution    = 1,
     .unit          = "Ah",
     .physical      = &ELECTRICAL_CHARGE,
     .baseFieldType = "UFIX16"},

    {.name          = "PEUKERT_EXPONENT",
     .description   = "Effect of discharge rate on usable battery capacity",
     .resolution    = 0.002,
     .offset        = 500, // = 1 / resolution
     .url           = "https://en.wikipedia.org/wiki/Peukert's_law",
     .baseFieldType = "UFIX8"},

    {.name          = "CURRENT_SIGNED",
     .description   = "Electrical current, signed",
     .physical      = &ELECTRICAL_CHARGE,
     .baseFieldType = "SIGNED_FIXED_POINT_NUMBER"},

    {.name = "ENERGY_UINT32", .description = "Electrical energy", .physical = &ELECTRICAL_ENERGY, .baseFieldType = "UINT32"},

    {.name        = "POWER_FIX32_OFFSET",
     .description = "Electrical power",
     .encodingDescription
     = "This uses an offset, so 0 encodes the maximum negative value -2000000000, and 0 is represented by 2000000000.",
     .resolution    = 1,
     .offset        = -2000000000,
     .physical      = &ELECTRICAL_POWER,
     .baseFieldType = "FIX32"},

    {.name        = "POWER_FIX32_OFFSET",
     .description = "Electrical power",
     .encodingDescription
     = "This uses an offset, so 0 encodes the maximum negative value -2000000000, and 0 is represented by 2000000000.",
     .resolution    = 1,
     .offset        = -2000000000,
     .physical      = &ELECTRICAL_POWER,
     .baseFieldType = "FIX32"},

    {.name        = "POWER_FIX32_VA_OFFSET",
     .description = "Electrical power, AC apparent power",
     .encodingDescription
     = "This uses an offset, so 0 encodes the maximum negative value -2000000000, and 0 is represented by 2000000000. Depending on "
       "the field it represents either real power in W, active power in VA or reactive power in VAR.",
     .resolution    = 1,
     .offset        = -2000000000,
     .physical      = &ELECTRICAL_APPARENT_POWER,
     .baseFieldType = "FIX32"},

    {.name        = "POWER_FIX32_VAR_OFFSET",
     .description = "Electrical power, AC reactive power",
     .encodingDescription
     = "This uses an offset, so 0 encodes the maximum negative value -2000000000, and 0 is represented by 2000000000. Depending on "
       "the field it represents either real power in W, active power in VA or reactive power in VAR.",
     .resolution    = 1,
     .offset        = -2000000000,
     .physical      = &ELECTRICAL_REACTIVE_POWER,
     .baseFieldType = "FIX32"},

    {.name          = "POWER_UINT16",
     .description   = "Electrical power, either DC or AC Real power, in Watts",
     .physical      = &ELECTRICAL_POWER,
     .resolution    = 1,
     .baseFieldType = "UINT16"},

    {.name          = "POWER_UINT16_VAR",
     .description   = "Electrical power, AC reactive",
     .physical      = &ELECTRICAL_REACTIVE_POWER,
     .unit          = "VAR",
     .resolution    = 1,
     .baseFieldType = "UINT16"},

    {.name          = "POWER_INT32",
     .description   = "Electrical power, either DC or AC Real power, in Watts",
     .physical      = &ELECTRICAL_POWER,
     .resolution    = 1,
     .baseFieldType = "INT32"},

    {.name          = "POWER_UINT32",
     .description   = "Electrical power, DC or AC Real power, in Watts",
     .physical      = &ELECTRICAL_POWER,
     .resolution    = 1,
     .baseFieldType = "UINT32"},

    {.name          = "POWER_UINT32_VA",
     .description   = "Electrical power, AC apparent power in VA.",
     .unit          = "VA",
     .resolution    = 1,
     .physical      = &ELECTRICAL_APPARENT_POWER,
     .baseFieldType = "UINT32"},

    {.name          = "POWER_UINT32_VAR",
     .description   = "Electrical power, AC reactive power in VAR.",
     .unit          = "VAR",
     .resolution    = 1,
     .physical      = &ELECTRICAL_REACTIVE_POWER,
     .baseFieldType = "UINT32"},

    {.name = "PERCENTAGE_UINT8", .description = "Percentage, unsigned", .unit = "%", .baseFieldType = "UINT8"},

    {.name = "PERCENTAGE_INT8", .description = "Percentage", .unit = "%", .baseFieldType = "INT8"},

    {.name          = "PERCENTAGE_UFIX16",
     .description   = "Percentage, unsigned high range",
     .resolution    = 0.004,
     .unit          = "%",
     .baseFieldType = "UFIX16"},

    {.name                = "ROTATION_FIX16",
     .description         = "Rotational speed",
     .encodingDescription = "Angular rotation in rad/s, in 1/32th of a thousandth radian",
     .comment             = "Whoever came up with 1/32th of 1/1000 of a radian?",
     .resolution          = (1e-3 / 32.0),
     .physical            = &ANGULAR_VELOCITY,
     .baseFieldType       = "FIX16"},

    {.name                = "ROTATION_FIX32",
     .description         = "Rotational speed, high resolution",
     .encodingDescription = "Angular rotation in rad/s, in 1/32th of a millionth radian",
     .comment             = "Whoever came up with 1/32th of 1e-6 of a radian?",
     .resolution          = (1e-6 / 32.0),
     .physical            = &ANGULAR_VELOCITY,
     .baseFieldType       = "FIX32"},

    {.name                = "ROTATION_UFIX16_RPM",
     .description         = "Rotational speed, RPM",
     .encodingDescription = "Angular rotation in 0.25 rpm",
     .resolution          = 0.25,
     .unit                = "rpm",
     .physical            = &ANGULAR_VELOCITY,
     .baseFieldType       = "UFIX16"},

    {.name          = "PRESSURE_UFIX16_HPA",
     .description   = "Pressure, 16 bit unsigned in hectopascal resolution",
     .resolution    = 100,
     .physical      = &PRESSURE,
     .baseFieldType = "UFIX16"},

    {.name          = "PRESSURE_UFIX16_KPA",
     .description   = "Pressure, 16 bit unsigned in kilopascal resolution.",
     .resolution    = 1000,
     .physical      = &PRESSURE,
     .baseFieldType = "UFIX16"},

    {.name          = "PRESSURE_RATE_FIX16_PA",
     .description   = "Pressure change rate, 16 bit signed in pascal resolution.",
     .resolution    = 1,
     .physical      = &PRESSURE_RATE,
     .baseFieldType = "FIX16"},

    {.name          = "PRESSURE_FIX16_KPA",
     .description   = "Pressure, 16 bit signed in kilopascal resolution.",
     .resolution    = 1000,
     .physical      = &PRESSURE,
     .baseFieldType = "FIX16"},

    {.name          = "PRESSURE_UFIX32_DPA",
     .description   = "Pressure, 32 bit unsigned in decipascal resolution.",
     .resolution    = 0.1,
     .physical      = &PRESSURE,
     .baseFieldType = "UFIX32"},

    {.name          = "PRESSURE_FIX32_DPA",
     .description   = "Pressure, 32 bit signed in decipascal resolution.",
     .resolution    = 0.1,
     .physical      = &PRESSURE,
     .baseFieldType = "FIX32"},

    {.name = "RADIO_FREQUENCY_UFIX32", .description = "Radio frequency", .physical = &FREQUENCY, .baseFieldType = "UFIX32"},

    {.name                = "FREQUENCY_UFIX16",
     .description         = "frequency",
     .encodingDescription = "Various resolutions are used, ranging from 0.01 Hz to 1 Hz",
     .physical            = &FREQUENCY,
     .baseFieldType       = "UFIX16"},

    {.name          = "SPEED_FIX16_MM",
     .description   = "Speed, with millimeter resolution",
     .resolution    = 0.001,
     .physical      = &SPEED,
     .baseFieldType = "FIX16"},

    {.name          = "SPEED_FIX16_CM",
     .description   = "Speed, with centimeter resolution",
     .resolution    = 0.01,
     .physical      = &SPEED,
     .baseFieldType = "FIX16"},

    {.name          = "SPEED_UFIX16_CM",
     .description   = "Speed, unsigned, with centimeter resolution",
     .resolution    = 0.01,
     .physical      = &SPEED,
     .baseFieldType = "UFIX16"},

    {.name          = "SPEED_UFIX16_DM",
     .description   = "Speed, unsigned, with decimeter resolution",
     .resolution    = 0.1,
     .physical      = &SPEED,
     .baseFieldType = "UFIX16"},

    {.name          = "DISTANCE_FIX16_M",
     .description   = "Distance, with meter resolution",
     .resolution    = 1,
     .physical      = &DISTANCE,
     .baseFieldType = "FIX16"},

    {.name          = "DISTANCE_FIX16_CM",
     .description   = "Distance, with centimeter resolution",
     .resolution    = 0.01,
     .physical      = &DISTANCE,
     .baseFieldType = "FIX16"},

    {.name          = "DISTANCE_FIX16_MM",
     .description   = "Distance, with millimeter resolution",
     .resolution    = 0.001,
     .physical      = &DISTANCE,
     .baseFieldType = "FIX16"},

    {.name          = "DISTANCE_FIX32_MM",
     .description   = "Distance, high range, with millimeter resolution",
     .resolution    = 0.001,
     .physical      = &DISTANCE,
     .baseFieldType = "FIX32"},

    {.name          = "DISTANCE_FIX32_CM",
     .description   = "Distance, high range, with centimeter resolution",
     .resolution    = 0.01,
     .physical      = &DISTANCE,
     .baseFieldType = "FIX32"},

    {.name = "DISTANCE_FIX64", .description = "Distance", .resolution = 1e-6, .physical = &DISTANCE, .baseFieldType = "FIX64"},

    {.name = "GAIN_FIX16", .description = "Gain", .resolution = 0.01, .baseFieldType = "FIX16"},

    {.name          = "MAGNETIC_FIELD_FIX16",
     .description   = "Magnetic field",
     .resolution    = 0.01,
     .physical      = &MAGNETIC_FIELD,
     .baseFieldType = "FIX16"},

    {.name          = "INSTANCE",
     .description   = "Instance",
     .comment       = "Devices that support multiple sensors TODO",
     .baseFieldType = "UINT8"},

    {.name = "PGN", .description = "PRN number", .resolution = 1, .baseFieldType = "UINT24"},

    {.name          = "POWER_FACTOR_UFIX16",
     .description   = "Power Factor",
     .resolution    = 1 / 16384.,
     .physical      = &POWER_FACTOR,
     .baseFieldType = "UFIX16"},

    {.name          = "POWER_FACTOR_UFIX8",
     .description   = "Power Factor",
     .resolution    = 0.01,
     .physical      = &POWER_FACTOR,
     .baseFieldType = "UFIX8"},

    {.name                = "SIGNED_ALMANAC_PARAMETER",
     .description         = "Almanac parameter, signed",
     .encodingDescription = "These encode various almanac parameters consisting of differing sizes and sign. They are all using an "
                            "interesting resolution/scale, which is always a number of bits that the value is shifted left or "
                            "right. This is reflected by resolution field containing some factor of 2^n or 2^-n.",
     .url                 = "https://www.gps.gov/technical/icwg/IS-GPS-200N.pdf",
     .baseFieldType       = "SIGNED_FIXED_POINT_NUMBER"},

    {.name                = "UNSIGNED_ALMANAC_PARAMETER",
     .description         = "Almanac parameter, unsigned",
     .encodingDescription = "These encode various almanac parameters consisting of differing sizes and sign. They are all using an "
                            "interesting resolution/scale, which is always a number of bits that the value is shifted left or "
                            "right. This is reflected by resolution field containing some factor of 2^n or 2^-n.",
     .url                 = "https://www.gps.gov/technical/icwg/IS-GPS-200N.pdf",
     .baseFieldType       = "UNSIGNED_FIXED_POINT_NUMBER"},

    // Stringy types
    {.name                = "STRING_FIX",
     .description         = "A fixed length string containing single byte codepoints.",
     .encodingDescription = "The length of the string is determined by the PGN field definition. Trailing bytes have been observed "
                            "as '@', ' ', 0x0 or 0xff.",
     .comment
     = "It is unclear what character sets are allowed/supported. Possibly UTF-8 but it could also be that only ASCII values "
       "are supported.",
     .pf     = fieldPrintStringFix,
     .v1Type = "ASCII text"},

    {.name        = "STRING_LZ",
     .description = "A varying length string containing single byte codepoints encoded with a length byte and terminating zero.",
     .encodingDescription = "The length of the string is determined by a starting length byte. It also contains a terminating "
                            "zero byte. The length byte includes the zero byte but not itself.",
     .comment
     = "It is unclear what character sets are allowed/supported. Possibly UTF-8 but it could also be that only ASCII values "
       "are supported.",
     .variableSize = True,
     .pf           = fieldPrintStringLZ,
     .v1Type       = "ASCII string starting with length byte"},

    {.name = "STRING_LAU",
     .description
     = "A varying length string containing double or single byte codepoints encoded with a length byte and terminating zero.",
     .encodingDescription
     = "The length of the string is determined by a starting length byte. The 2nd byte contains 0 for UNICODE or 1 for ASCII.",
     .comment
     = "It is unclear what character sets are allowed/supported. For single byte, assume ASCII. For UNICODE, assume UTF-16, "
       "but this has not been seen in the wild yet.",
     .variableSize = True,
     .pf           = fieldPrintStringLAU,
     .v1Type       = "ASCII or UNICODE string starting with length and control byte"},

    // Others
    {.name                = "BINARY",
     .description         = "Binary field",
     .encodingDescription = "Unspecified content consisting of any number of bits.",
     .pf                  = fieldPrintBinary,
     .v1Type              = "Binary data"},

    {.name                = "RESERVED",
     .description         = "Reserved field",
     .encodingDescription = "All reserved bits shall be 1",
     .comment             = "NMEA reserved for future expansion and/or to align next data on byte boundary",
     .pf                  = fieldPrintReserved},

    {.name                = "SPARE",
     .description         = "Spare field",
     .encodingDescription = "All spare bits shall be 0",
     .comment = "This is like a reserved field but originates from other sources where unused fields shall be 0, like the AIS "
                "ITU-1371 standard.",
     .pf      = fieldPrintSpare},

    {.name        = "MMSI",
     .description = "MMSI",
     .resolution  = 1,
     .size        = 32,
     .hasSign     = False,
     .rangeMin    = 2000000, // Minimal valid MMSI is coastal station (00) MID (2xx)
     .rangeMax    = 999999999,
     .encodingDescription
     = "The MMSI is encoded as a 32 bit number, but is always printed as a 9 digit number and should be considered as a string. "
       "The first three or four digits are special, see the USCG link for a detailed explanation.",
     .url = "https://navcen.uscg.gov/maritime-mobile-service-identity",
     .pf  = fieldPrintMMSI},

    {.name        = "VARIABLE",
     .description = "Variable",
     .encodingDescription
     = "The definition of the field is that of the reference PGN and reference field, this is totally variable.",
     .pf = fieldPrintVariable}};

const size_t fieldTypeCount = ARRAY_SIZE(fieldTypeList);

#else
extern FieldType                     fieldTypeList[];
extern const size_t                  fieldTypeCount;
#endif // FIELDTYPE_GLOBALS

extern FieldType *getFieldType(const char *name);
extern void       fillFieldType(bool doUnitFixup);

#endif // FIELD_H_INCLUDED
