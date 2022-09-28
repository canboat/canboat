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
extern bool fieldPrintStringVar(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintTime(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);
extern bool fieldPrintVariable(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits);

typedef enum Bool
{
  Null,
  False,
  True
} Bool;

/**
 * The FieldType structure encapsulates the different datatypes in a PGN field.
 */

struct FieldType
{
  const char *name;                // Name, UPPERCASE_WITH_UNDERSCORE
  const char *description;         // English description, shortish
  const char *encodingDescription; // How the value is encoded
  const char *comment;             // Other observations
  uint32_t    size;                // Size in bits
  Bool        variableSize;        // True if size varies per instance of PGN
  char       *baseFieldType;       // Some field types are variations of others
  char       *v1Type;              // Type as printed in v1 xml/json

  // The following are only set for numbers
  const char *unit;       // String containing the 'Dimension' (e.g. s, h, m/s, etc.)
  int32_t     offset;     // For numbers with excess-K offset
  double      resolution; // A positive real value, or 1 for integral values
  Bool        hasSign;    // Is the value signed, e.g. has both positive and negative values?
  const char *format;     // Format string for printf

  // These are derived from size, variableSize, resolution and hasSign
  double rangeMin;
  double rangeMax;
  // Sometimes we override the range so as to show a more useful value
  char *rangeMinText;
  char *rangeMaxText;

  // How to print this field
  FieldPrintFunctionType pf;

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
     .pf = fieldPrintNumber},

    {.name          = "INTEGER",
     .description   = "Integral number",
     .resolution    = 1,
     .hasSign       = True,
     .baseFieldType = "NUMBER",
     .v1Type        = "Integer"},

    {.name          = "UNSIGNED_INTEGER",
     .description   = "Unsigned integral number",
     .resolution    = 1,
     .hasSign       = False,
     .baseFieldType = "NUMBER",
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
     .description = "An unsigned numeric value where the Least Boolificant Bit does not encode the integer value 1",
     .encodingDescription
     = "The `Resolution` attribute indicates what the raw value 1 should represent. The `Signed` and `BitLength` attributes are "
       "always present. Together, this gives sufficient information to represent a fixed point number in a particular range where "
       "non-integral values can be encoded without requiring four or eight bytes for a floating point number.",
     .hasSign       = False,
     .baseFieldType = "NUMBER"},

    {.name        = "SIGNED_FIXED_POINT_NUMBER",
     .description = "A signed numeric value where the Least Boolificant Bit does not encode the integer value 1",
     .encodingDescription
     = "The `Resolution` attribute indicates what the raw value 1 should represent. The `Signed` and `BitLength` attributes are "
       "always present. Together, this gives sufficient information to represent a fixed point number in a particular range where "
       "non-integral values can be encoded without requiring four or eight bytes for a floating point number.",
     .hasSign       = True,
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

    {.name = "FLOAT", .description = "32 bit floating point number", .size = 32, .hasSign = True, .pf = fieldPrintFloat},

    {.name = "DECIMAL", .description = "Binary Coded Decimal number", .size = 32, .hasSign = False, .pf = fieldPrintDecimal},

    {.name                = "LOOKUP",
     .description         = "Number value where each value encodes for a distinct meaning",
     .encodingDescription = "Each lookup has a LookupEnumeration defining what the possible values mean",
     .comment = "For almost all lookups the list of values is known with some precision, but it is quite possible that a value "
                "occurs that has no corresponding textual explanation.",
     .pf      = fieldPrintLookup,
     .baseFieldType = "UNSIGNED_INTEGER",
     .v1Type        = "Lookup table"},

    {.name                = "BITLOOKUP",
     .description         = "Number value where each bit value encodes for a distinct meaning",
     .encodingDescription = "Each LookupBit has a LookupBitEnumeration defining what the possible values mean. A bitfield can have "
                            "any combination of bits set.",
     .comment = "For almost all lookups the list of values is known with some precision, but it is quite possible that a value "
                "occurs that has no corresponding textual explanation.",
     .pf      = fieldPrintBitLookup,
     .baseFieldType = "UNSIGNED_INTEGER"},

    {.name          = "MANUFACTURER",
     .description   = "Manufacturer",
     .size          = 11,
     .pf            = fieldPrintLookup,
     .baseFieldType = "LOOKUP",
     .v1Type        = "Manufacturer code"},

    {.name = "INDUSTRY", .description = "Industry", .size = 3, .pf = fieldPrintLookup, .baseFieldType = "LOOKUP"},

    {.name                = "DECIMAL",
     .description         = "A unsigned numeric value represented with 2 decimal digits per byte",
     .encodingDescription = "Each byte represent 2 digits, so 1234 is represented by 2 bytes containing 0x12 and 0x34. A number "
                            "with an odd number of digits will have 0 as the first digit in the first byte.",
     .hasSign             = False,
     .pf                  = fieldPrintDecimal},

    {.name = "VERSION", .description = "Version", .resolution = 0.001, .pf = fieldPrintNumber, .baseFieldType = "UFIX16"},

    // Specific typed numeric fields

    {.name          = "DILUTION_OF_PRECISION_FIX16",
     .description   = "Dilution of precision",
     .comment       = "See https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)",
     .resolution    = 0.01,
     .baseFieldType = "FIX16"},

    {.name          = "DILUTION_OF_PRECISION_UFIX16",
     .description   = "Dilution of precision",
     .comment       = "See https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)",
     .resolution    = 0.01,
     .baseFieldType = "UFIX16"},

    {.name          = "SIGNALTONOISERATIO_UFIX16",
     .description   = "Signal-to-noise ratio",
     .comment       = "See https://en.wikipedia.org/wiki/Signal-to-noise_ratio",
     .resolution    = 0.01,
     .unit          = "dB",
     .baseFieldType = "UFIX16"},

    {.name = "ANGLE_FIX16", .description = "Angular rotation", .resolution = 0.001, .unit = "rad", .baseFieldType = "FIX16"},
    {.name = "ANGLE_FIX16_DDEG", .description = "Angular rotation", .resolution = 0.1, .unit = "deg", .baseFieldType = "FIX16"},

    {.name          = "ANGLE_UFIX16",
     .description   = "Angular rotation",
     .resolution    = 0.001,
     .unit          = "rad",
     .pf            = fieldPrintNumber,
     .baseFieldType = "UFIX16"},

    {.name                = "GEO_FIX32",
     .description         = "Geographical latitude or longitude",
     .encodingDescription = "The `Resolution` for this field is 1.0e-7, so the precision is 1/10 millionth of a degree, or about 1 "
                            "cm when we refer to an Earth position",
     .resolution          = 1.0e-7,
     .unit                = "deg",
     .pf                  = fieldPrintLatLon,
     .baseFieldType       = "FIX32",
     .v1Type              = "Lat/Lon"},

    {.name                = "GEO_FIX64",
     .description         = "Geographical latitude or longitude, high precision",
     .encodingDescription = "The `Resolution` for this field is 1.0e-16, so the precision is about 0.01 nm (nanometer) when we "
                            "refer to an Earth position",
     .resolution          = 1.0e-16,
     .unit                = "deg",
     .pf                  = fieldPrintLatLon,
     .baseFieldType       = "FIX64",
     .v1Type              = "Lat/Lon"},

    {.name          = "LENGTH_UFIX8_DAM",
     .description   = "Length, in decameter precision",
     .resolution    = 10,
     .unit          = "m",
     .pf            = fieldPrintNumber,
     .baseFieldType = "UFIX8"},

    {.name          = "LENGTH_UFIX16_DM",
     .description   = "Length, in decimeter precision",
     .resolution    = 0.1,
     .unit          = "m",
     .pf            = fieldPrintNumber,
     .baseFieldType = "UFIX16"},

    {.name          = "LENGTH_UFIX32_CM",
     .description   = "Length, in centimeter precision",
     .resolution    = 0.01,
     .unit          = "m",
     .pf            = fieldPrintNumber,
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX32_M",
     .description   = "Length, in meter precision",
     .resolution    = 1,
     .unit          = "m",
     .pf            = fieldPrintNumber,
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX32_CM",
     .description   = "Length, in centimeter precision",
     .resolution    = 0.01,
     .unit          = "m",
     .pf            = fieldPrintNumber,
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX32_MM",
     .description   = "Length, in millimeter precision",
     .resolution    = 0.001,
     .unit          = "m",
     .pf            = fieldPrintNumber,
     .baseFieldType = "UFIX32"},

    {.name          = "TEMPERATURE",
     .description   = "Temperature",
     .resolution    = 0.01,
     .unit          = "K",
     .baseFieldType = "UFIX16",
     .v1Type        = "Temperature"},

    {.name                = "TEMPERATURE_HIGH",
     .description         = "Temperature, high range",
     .encodingDescription = "This has a higher range but lower precision than TEMPERATURE",
     .resolution          = 0.1,
     .unit                = "K",
     .baseFieldType       = "TEMPERATURE"},

    {.name                = "TEMPERATURE_UFIX24",
     .description         = "Temperature, high precision",
     .encodingDescription = "This has a higher range and higher precision than TEMPERATURE (but uses three bytes)",
     .size                = 24,
     .resolution          = 0.001,
     .unit                = "K",
     .baseFieldType       = "TEMPERATURE"},

    {.name          = "TEMPERATURE_DELTA_FIX16",
     .description   = "Temperature difference",
     .resolution    = 0.001,
     .unit          = "K",
     .baseFieldType = "FIX16"},

    {.name = "VOLUMETRIC_FLOW", .description = "Volumetric flow", .resolution = 0.1, .unit = "L/h", .baseFieldType = "FIX16"},

    {.name                = "CONCENTRATION_UINT16_PPM",
     .description         = "Concentration of one substance in another",
     .encodingDescription = "Expressed in parts per million",
     .resolution          = 1,
     .unit                = "ppm",
     .baseFieldType       = "UINT16"},

    {.name = "VOLUME_UFIX16_L", .description = "Volume", .resolution = 1, .unit = "L", .baseFieldType = "UFIX16"},

    {.name = "VOLUME_UFIX32_DL", .description = "Volume", .resolution = 0.1, .unit = "L", .baseFieldType = "UFIX32"},

    {.name                = "TIME",
     .description         = "Time",
     .encodingDescription = "Time since midnight.",
     .resolution          = 0.0001,
     .unit                = "s",
     .pf                  = fieldPrintTime,
     .baseFieldType       = "UFIX32"},

    {.name                = "SHORT_TIME",
     .description         = "Time, low range",
     .encodingDescription = "This encodes an elapsed time interval with a precision of 0.1 milliseconds.",
     .resolution          = 0.0001,
     .unit                = "s",
     .pf                  = fieldPrintTime,
     .baseFieldType       = "UFIX16"},

    {.name          = "TIME_UFIX8_5MS",
     .description   = "Time delta, 8 bits with 5 millisecond precision",
     .resolution    = 0.05,
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "UFIX8"},

    {.name          = "TIME_UFIX8_P12S",
     .description   = "Time delta, 8 bits with 2^12 second precision",
     .resolution    = POW2(12),
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "UFIX8"},

    {.name          = "TIME_UFIX16_MS",
     .description   = "Time delta, 16 bits with millisecond precision",
     .resolution    = 0.001,
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "UFIX16"},

    {.name          = "TIME_UFIX16_CS",
     .description   = "Time delta, 16 bits with centisecond precision",
     .resolution    = 0.01,
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "UFIX16"},

    {.name          = "TIME_UFIX16_MIN",
     .description   = "Time delta, 16 bits with minute precision",
     .resolution    = 60,
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "UFIX16"},

    {.name          = "TIME_UFIX24_MS",
     .description   = "Time delta, 24 bits with millisecond precision",
     .resolution    = 0.001,
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "UFIX24"},

    {.name          = "TIME_UFIX32_S",
     .description   = "Time delta, 32 bits with second precision",
     .resolution    = 1,
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "UFIX32"},

    {.name          = "TIME_UFIX32_MS",
     .description   = "Time delta, 32 bits with millisecond precision",
     .resolution    = 0.001,
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "UFIX32"},

    {.name          = "TIME_FIX32_MS",
     .description   = "Time delta",
     .resolution    = 0.001,
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "FIX32"},

    {.name          = "TIME_FIX16_5CS",
     .description   = "Time delta, 5 centisecond resolution",
     .resolution    = 0.05,
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "FIX16"},

    {.name          = "TIME_FIX16_MIN",
     .description   = "Time delta, minute resolution",
     .resolution    = 60,
     .unit          = "s",
     .pf            = fieldPrintTime,
     .baseFieldType = "FIX16"},

    {.name                = "DATE",
     .description         = "Date",
     .encodingDescription = "The date, in days since 1 January 1970.",
     .unit                = "days",
     .pf                  = fieldPrintDate,
     .baseFieldType       = "UINT16"},

    {.name                = "VOLTAGE_UFIX16",
     .description         = "Voltage",
     .encodingDescription = "Various resolutions are used, ranging from 0.01 V to 1 V.",
     .unit                = "V",
     .baseFieldType       = "UFIX16"},

    {.name                = "VOLTAGE_INT16",
     .description         = "Voltage, signed",
     .encodingDescription = "Various resolutions are used, ranging from 0.01 V to 1 V.",
     .unit                = "V",
     .baseFieldType       = "INT16"},

    {.name          = "CURRENT",
     .description   = "Electrical current",
     .hasSign       = False,
     .unit          = "A",
     .baseFieldType = "UNSIGNED_FIXED_POINT_NUMBER"},

    {.name = "CURRENT_UFIX8_A", .description = "Electrical current", .resolution = 1, .unit = "A", .baseFieldType = "UFIX8"},

    {.name = "CURRENT_UFIX16_A", .description = "Electrical current", .resolution = 1, .unit = "A", .baseFieldType = "UFIX16"},

    {.name = "CURRENT_UFIX16_DA", .description = "Electrical current", .resolution = .1, .unit = "A", .baseFieldType = "UFIX16"},

    {.name = "CURRENT_FIX16_DA", .description = "Electrical current", .resolution = .1, .unit = "A", .baseFieldType = "FIX16"},

    {.name = "CURRENT_FIX24_CA", .description = "Electrical current", .resolution = .01, .unit = "A", .baseFieldType = "FIX16"},

    {.name          = "ELECTRIC_CHARGE_UFIX16_AH",
     .description   = "Electrical current",
     .resolution    = 3600,
     .unit          = "C",
     .baseFieldType = "UFIX16"},

    {.name = "PEUKERT_EXPONENT", .description = "Electrical current", .resolution = 0.002, .offset = 1, .baseFieldType = "UFIX8"},

    {.name          = "CURRENT_SIGNED",
     .description   = "Electrical current, signed",
     .unit          = "A",
     .baseFieldType = "SIGNED_FIXED_POINT_NUMBER"},

    {.name = "ENERGY", .description = "Electrical energy consumption", .unit = "kWh", .baseFieldType = "UINT32"},

    {.name        = "POWER_INT32_OFFSET",
     .description = "Electrical energy consumption",
     .encodingDescription
     = "This uses an offset, so 0 encodes the maximum negative value -2000000000, and 0 is represented by 2000000000. Depending on "
       "the field it represents either real power in W, active power in VA or reactive power in VAR.",
     .offset        = -2000000000,
     .baseFieldType = "INT32"},

    {.name          = "POWER_UINT16",
     .description   = "Electrical power, either DC or AC Real power, in Watts",
     .unit          = "W",
     .baseFieldType = "UINT16"},

    {.name = "POWER_UINT16_VAR", .description = "Electrical power, AC reactive", .unit = "VAR", .baseFieldType = "UINT16"},

    {.name          = "POWER_INT32",
     .description   = "Electrical power, either DC or AC Real power, in Watts",
     .unit          = "W",
     .baseFieldType = "INT32"},

    {.name          = "POWER_UINT32",
     .description   = "Electrical power, DC or AC Real power, in Watts",
     .unit          = "W",
     .baseFieldType = "UINT32"},

    {.name = "POWER_UINT32_VA", .description = "Electrical power, AC active power in VA.", .unit = "VA", .baseFieldType = "UINT32"},

    {.name          = "POWER_UINT32_VAR",
     .description   = "Electrical power, AC reactive power in VAR.",
     .unit          = "VAR",
     .baseFieldType = "UINT32"},

    {.name = "PERCENTAGE_UINT8", .description = "Percentage, unsigned", .unit = "%", .baseFieldType = "UINT8"},

    {.name = "PERCENTAGE_INT8", .description = "Percentage", .unit = "%", .baseFieldType = "INT8"},

    {.name = "PERCENTAGE_UINT16", .description = "Percentage, unsigned high range", .unit = "%", .baseFieldType = "UINT16"},

    {.name                = "ROTATION_FIX16",
     .description         = "Rotational speed",
     .encodingDescription = "Angular rotation in rad/s, in 1/32th of a thousandth radian",
     .comment             = "Whoever came up with 1/32th of 1/1000 of a radian?",
     .resolution          = (1e-3 / 32.0),
     .unit                = "rad/s",
     .baseFieldType       = "FIX16"},

    {.name                = "ROTATION_FIX32",
     .description         = "Rotational speed, high resolution",
     .encodingDescription = "Angular rotation in rad/s, in 1/32th of a millionth radian",
     .comment             = "Whoever came up with 1/32th of 1e-6 of a radian?",
     .resolution          = (1e-6 / 32.0),
     .unit                = "rad/s",
     .baseFieldType       = "FIX32"},

    {.name                = "ROTATION_UFIX16_RPM",
     .description         = "Rotational speed, RPM",
     .encodingDescription = "Angular rotation in 0.25 rpm",
     .resolution          = 0.25,
     .unit                = "rpm",
     .baseFieldType       = "UFIX16"},

    {.name          = "PRESSURE_UFIX16_HPA",
     .description   = "Pressure, 16 bit unsigned in hectopascal precision",
     .resolution    = 100,
     .unit          = "Pa",
     .baseFieldType = "UFIX16"},

    {.name          = "PRESSURE_UFIX16_KPA",
     .description   = "Pressure, 16 bit unsigned in kilopascal precision.",
     .resolution    = 1000,
     .unit          = "Pa",
     .baseFieldType = "UFIX16"},

    {.name          = "PRESSURE_RATE_FIX16_PA",
     .description   = "Pressure change rate, 16 bit signed in pascal precision.",
     .resolution    = 1000,
     .unit          = "Pa/hr",
     .baseFieldType = "FIX16"},

    {.name          = "PRESSURE_FIX16_KPA",
     .description   = "Pressure, 16 bit signed in kilopascal precision.",
     .resolution    = 1000,
     .unit          = "Pa",
     .baseFieldType = "FIX16"},

    {.name          = "PRESSURE_UFIX32_DPA",
     .description   = "Pressure, 32 bit unsigned in decipascal precision.",
     .resolution    = 0.1,
     .unit          = "Pa",
     .baseFieldType = "UFIX32"},

    {.name          = "PRESSURE_FIX32_DPA",
     .description   = "Pressure, 32 bit signed in decipascal precision.",
     .resolution    = 0.1,
     .unit          = "Pa",
     .baseFieldType = "FIX32"},

    {.name = "RADIO_FREQUENCY_UFIX32", .description = "Radio frequency", .resolution = 10, .unit = "Hz", .baseFieldType = "UFIX32"},

    {.name                = "FREQUENCY_UFIX16",
     .description         = "frequency",
     .encodingDescription = "Various resolutions are used, ranging from 0.01 Hz to 1 Hz",
     .unit                = "Hz",
     .baseFieldType       = "UFIX16"},

    {.name          = "SPEED_FIX16_MM",
     .description   = "Speed, with millimeter precision",
     .resolution    = 0.001,
     .unit          = "m/s",
     .baseFieldType = "FIX16"},

    {.name          = "SPEED_FIX16_CM",
     .description   = "Speed, with centimeter precision",
     .resolution    = 0.01,
     .unit          = "m/s",
     .baseFieldType = "FIX16"},

    {.name          = "SPEED_UFIX16_CM",
     .description   = "Speed, unsigned, with centimeter precision",
     .resolution    = 0.01,
     .unit          = "m/s",
     .baseFieldType = "UFIX16"},

    {.name          = "SPEED_UFIX16_DM",
     .description   = "Speed, unsigned, with decimeter precision",
     .resolution    = 0.1,
     .unit          = "m/s",
     .baseFieldType = "UFIX16"},

    {.name          = "DISTANCE_FIX16_M",
     .description   = "Distance, with meter precision",
     .resolution    = 1,
     .unit          = "m",
     .baseFieldType = "FIX16"},

    {.name          = "DISTANCE_FIX16_CM",
     .description   = "Distance, with centimeter precision",
     .resolution    = 0.01,
     .unit          = "m",
     .baseFieldType = "FIX16"},

    {.name          = "DISTANCE_FIX16_MM",
     .description   = "Distance, with millimeter precision",
     .resolution    = 0.001,
     .unit          = "m",
     .baseFieldType = "FIX16"},

    {.name          = "DISTANCE_FIX32_MM",
     .description   = "Distance, high range, with millimeter precision",
     .resolution    = 0.001,
     .unit          = "m",
     .baseFieldType = "FIX32"},

    {.name          = "DISTANCE_FIX32_CM",
     .description   = "Distance, high range, with centimeter precision",
     .resolution    = 0.01,
     .unit          = "m",
     .baseFieldType = "FIX32"},

    {.name = "DISTANCE_FIX64", .description = "Distance", .resolution = 1e-6, .unit = "m", .baseFieldType = "FIX64"},

    {.name          = "LENGTH_UFIX8_DAM",
     .description   = "Length, byte, unsigned decameters",
     .resolution    = 10.,
     .unit          = "m",
     .baseFieldType = "UFIX8"},

    {.name          = "LENGTH_UFIX16_CM",
     .description   = "Length, unsigned centimeters",
     .resolution    = 0.01,
     .unit          = "m",
     .baseFieldType = "UFIX16"},

    {.name          = "LENGTH_UFIX16_DM",
     .description   = "Length, unsigned decimeters",
     .resolution    = 0.1,
     .unit          = "m",
     .baseFieldType = "UFIX16"},

    {.name          = "LENGTH_UFIX32_MM",
     .description   = "Length, high range, unsigned millimeters",
     .resolution    = 0.001,
     .unit          = "m",
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX32_CM",
     .description   = "Length, high range, unsigned centimeters",
     .resolution    = 0.01,
     .unit          = "m",
     .baseFieldType = "UFIX32"},

    {.name          = "LENGTH_UFIX32_M",
     .description   = "Length, high range, meters",
     .resolution    = 1.,
     .unit          = "m",
     .baseFieldType = "UFIX32"},

    {.name = "GAIN_FIX16", .description = "Gain", .resolution = 0.01, .baseFieldType = "FIX16"},

    {.name          = "MAGNETIC_FIELD_FIX16",
     .description   = "Magnetic field",
     .resolution    = 0.01,
     .unit          = "Tesla",
     .baseFieldType = "FIX16"},

    {.name          = "ELAPSED",
     .description   = "Elapsed time",
     .hasSign       = False,
     .unit          = "s",
     .pf            = fieldPrintNumber,
     .baseFieldType = "SIGNED_FIXED_POINT_NUMBER"},

    {.name          = "INSTANCE",
     .description   = "Instance",
     .comment       = "Devices that support multiple sensors TODO",
     .baseFieldType = "UINT8"},

    {.name = "PGN", .description = "PRN number", .resolution = 1, .baseFieldType = "UINT24"},

    {.name          = "POWER_FACTOR_UFIX16",
     .description   = "Power Factor",
     .resolution    = 1 / 16384.,
     .unit          = "Cos Phi",
     .baseFieldType = "UFIX16"},

    {.name = "POWER_FACTOR_UFIX8", .description = "Power Factor", .resolution = 0.01, .unit = "Cos Phi", .baseFieldType = "UFIX8"},

    {.name                = "SIGNED_ALMANAC_PARAMETER",
     .description         = "Almanac parameter, signed",
     .encodingDescription = "These encode various almanac parameters consisting of differing sizes and sign. They are all using an "
                            "interesting resolution/scale, which is always a number of bits that the value is shifted left or "
                            "right. This is reflected by resolution field containing some factor of 2^n or 2^-n.",
     .baseFieldType       = "SIGNED_FIXED_POINT_NUMBER"},

    {.name                = "UNSIGNED_ALMANAC_PARAMETER",
     .description         = "Almanac parameter, unsigned",
     .encodingDescription = "These encode various almanac parameters consisting of differing sizes and sign. They are all using an "
                            "interesting resolution/scale, which is always a number of bits that the value is shifted left or "
                            "right. This is reflected by resolution field containing some factor of 2^n or 2^-n.",
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
    {.name                = "STRING_VAR",
     .description         = "A varying length string containing single byte codepoints.",
     .encodingDescription = "The length of the string is determined either with a start (0x02) and stop (0x01) byte, or with a "
                            "starting length byte (> 0x02), or an indication that the string is empty which is encoded by either "
                            "0x01 or 0x00 as the first byte.",
     .comment
     = "It is unclear what character sets are allowed/supported. Possibly UTF-8 but it could also be that only ASCII values "
       "are supported.",
     .variableSize = True,
     .pf           = fieldPrintStringVar},

    {.name        = "STRING_LZ",
     .description = "A varying length string containing single byte codepoints encoded with a length byte and terminating zero.",
     .encodingDescription = "The length of the string is determined by a starting length byte. It also contains a terminating "
                            "zero byte. The length byte includes the zero byte but not itself.",
     .comment
     = "It is unclear what character sets are allowed/supported. Possibly UTF-8 but it could also be that only ASCII values "
       "are supported.",
     .variableSize = True,
     .pf           = fieldPrintStringLZ,
     .v1Type       = "String with start/stop byte"},

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
     .encodingDescription = "Any content consisting of any number of bits.",
     .pf                  = fieldPrintBinary,
     .v1Type              = "Binary data"},

    {.name                = "RESERVED",
     .description         = "Reserved field",
     .encodingDescription = "All reserved bits shall be 1",
     .comment             = "NMEA reserved for future expansion and/or to align next data on byte boundary",
     .pf                  = fieldPrintReserved,
     .baseFieldType       = "BINARY"},

    {.name                = "SPARE",
     .description         = "Spare field",
     .encodingDescription = "All reserved bits shall be 0",
     .comment = "This is like a reserved field but originates from other sources where unused fields shall be 0, like the AIS "
                "ITU-1371 standard.",
     .pf      = fieldPrintSpare,
     .baseFieldType = "BINARY"},

    {.name        = "MMSI",
     .description = "MMSI",
     .encodingDescription
     = "The MMSI is encoded as a 32 bit number, but is always printed as a 9 digit number and should be considered as a string",
     .format        = "\"%09u\"",
     .baseFieldType = "UINT32",
     .rangeMinText  = "000000000",
     .rangeMaxText  = "999999999"},

    {.name        = "VARIABLE",
     .description = "Variable",
     .encodingDescription
     = "The definition of the field is that of the reference PGN and reference field, this is totally variable.",
     .pf = fieldPrintVariable}};

const size_t fieldTypeCount = ARRAY_SIZE(fieldTypeList);

#else
extern FieldType    fieldTypeList[];
extern const size_t fieldTypeCount;
#endif // FIELDTYPE_GLOBALS

extern FieldType *getFieldType(const char *name);
extern void       fillFieldType(void);

#endif // FIELD_H_INCLUDED
