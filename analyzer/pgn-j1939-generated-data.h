/* ==========================================================================
*
*   GENERATED FILE - DO NOT EDIT.
*
*   Every line below is written by `keel` from the YAML database in
*   database/. Editing this file achieves nothing: the next `make generated`
*   overwrites it, and CI fails the build on the resulting diff.
*
*   To change a PGN, a lookup or a field type, edit the YAML:
*
*       database/pgns/<pgn>-<id>.yaml     one file per PGN variant
*       database/lookups/<NAME>.yaml      one file per enumeration
*       database/fieldtypes.yaml          the field-type hierarchy
*
*   then regenerate and check it in:
*
*       make generated
*
*   See keel/DESIGN.md, or run `keel edit` for the browser editor.
*
* ==========================================================================
*
* (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
* Part of CANboat; licensed under the Apache License, Version 2.0.
*/

Pgn pgnList[] = {
    {"0xE800-0xEEFF: Standardized single-frame addressed",
     59392,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 64, .resolution = 1.0}
     },
     .camelDescription = "0xe8000xeeffStandardizedSingleFrameAddressed",
     .fallback = true,
     .explanation = "Standardized PGNs in PDU1 (addressed) single-frame PGN range 0xE800 to 0xEE00 (59392 - 60928). When this is shown during analysis it means the PGN is not reverse engineered yet."},

    {"ISO Acknowledgement",
     59392,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Control", .camelName = "control", .fieldType = "LOOKUP", .size = 8, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupISO_CONTROL, .lookup.name = "ISO_CONTROL"},
      {.name = "Group Function", .camelName = "groupFunction", .fieldType = "UINT8", .resolution = 1.0},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 24, .resolution = 1.0},
      {.name = "PGN", .camelName = "pgn", .fieldType = "PGN", .size = 24, .description = "Parameter Group Number of requested information"}
     },
     .camelDescription = "isoAcknowledgement",
     .interval = UINT16_MAX,
     .explanation = "This message is provided by ISO 11783 for a handshake mechanism between transmitting and receiving devices. This message is the possible response to acknowledge the reception of a 'normal broadcast' message or the response to a specific command to indicate compliance or failure."},

    {"ISO Request",
     59904,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "PGN", .camelName = "pgn", .fieldType = "PGN", .size = 24}
     },
     .camelDescription = "isoRequest",
     .interval = UINT16_MAX,
     .explanation = "As defined by ISO, this message has a data length of 3 bytes with no padding added to complete the single frame. The appropriate response to this message is based on the PGN being requested, and whether the receiver supports the requested PGN."},

    {"ISO Transport Protocol, Data Transfer",
     60160,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "SID", .camelName = "sid", .fieldType = "UINT8", .resolution = 1.0},
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 56, .resolution = 1.0}
     },
     .camelDescription = "isoTransportProtocolDataTransfer",
     .interval = UINT16_MAX,
     .explanation = "ISO 11783 defines this PGN as part of the Transport Protocol method used for transmitting messages that have 9 or more data bytes. This PGN represents a single packet of a multipacket message."},

    {"ISO Transport Protocol, Connection Management - Request To Send",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Group Function Code", .camelName = "groupFunctionCode", .fieldType = "LOOKUP", .size = 8, .resolution = 1.0, .hasMatchValue = true, .matchValue = 16, .description = "RTS", .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupISO_COMMAND, .lookup.name = "ISO_COMMAND"},
      {.name = "Message size", .camelName = "messageSize", .fieldType = "UINT16", .resolution = 1.0, .description = "bytes"},
      {.name = "Packets", .camelName = "packets", .fieldType = "UINT8", .resolution = 1.0, .description = "packets"},
      {.name = "Packets reply", .camelName = "packetsReply", .fieldType = "UINT8", .resolution = 1.0, .description = "packets sent in response to CTS"},
      {.name = "PGN", .camelName = "pgn", .fieldType = "PGN", .size = 24}
     },
     .camelDescription = "isoTransportProtocolConnectionManagementRequestToSend",
     .interval = UINT16_MAX,
     .explanation = "ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting messages that have 9 or more data bytes. This PGN's role in the transport process is to prepare the receiver for the fact that this sender wants to transmit a long message. The receiver will respond with CTS.",
     .url = "https://embeddedflakes.com/j1939-transport-protocol/"},

    {"ISO Transport Protocol, Connection Management - Clear To Send",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Group Function Code", .camelName = "groupFunctionCode", .fieldType = "LOOKUP", .size = 8, .resolution = 1.0, .hasMatchValue = true, .matchValue = 17, .description = "CTS", .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupISO_COMMAND, .lookup.name = "ISO_COMMAND"},
      {.name = "Max packets", .camelName = "maxPackets", .fieldType = "UINT8", .resolution = 1.0, .description = "Number of frames that can be sent before another CTS is required"},
      {.name = "Next SID", .camelName = "nextSid", .fieldType = "UINT8", .resolution = 1.0, .description = "Number of next frame to be transmitted"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 16, .resolution = 1.0},
      {.name = "PGN", .camelName = "pgn", .fieldType = "PGN", .size = 24}
     },
     .camelDescription = "isoTransportProtocolConnectionManagementClearToSend",
     .interval = UINT16_MAX,
     .explanation = "ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting messages that have 9 or more data bytes. This PGN's role in the transport process is to signal to the sender that the receive is ready to receive a number of frames.",
     .url = "https://embeddedflakes.com/j1939-transport-protocol/"},

    {"ISO Transport Protocol, Connection Management - End Of Message",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Group Function Code", .camelName = "groupFunctionCode", .fieldType = "LOOKUP", .size = 8, .resolution = 1.0, .hasMatchValue = true, .matchValue = 19, .description = "EOM", .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupISO_COMMAND, .lookup.name = "ISO_COMMAND"},
      {.name = "Total message size", .camelName = "totalMessageSize", .fieldType = "UINT16", .resolution = 1.0, .description = "bytes"},
      {.name = "Total number of frames received", .camelName = "totalNumberOfFramesReceived", .fieldType = "UINT8", .resolution = 1.0, .description = "Total number of of frames received"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 8, .resolution = 1.0},
      {.name = "PGN", .camelName = "pgn", .fieldType = "PGN", .size = 24}
     },
     .camelDescription = "isoTransportProtocolConnectionManagementEndOfMessage",
     .interval = UINT16_MAX,
     .explanation = "ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting messages that have 9 or more data bytes. This PGN's role in the transport process is to mark the end of the message.",
     .url = "https://embeddedflakes.com/j1939-transport-protocol/"},

    {"ISO Transport Protocol, Connection Management - Broadcast Announce",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Group Function Code", .camelName = "groupFunctionCode", .fieldType = "LOOKUP", .size = 8, .resolution = 1.0, .hasMatchValue = true, .matchValue = 32, .description = "BAM", .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupISO_COMMAND, .lookup.name = "ISO_COMMAND"},
      {.name = "Message size", .camelName = "messageSize", .fieldType = "UINT16", .resolution = 1.0, .description = "bytes"},
      {.name = "Packets", .camelName = "packets", .fieldType = "UINT8", .resolution = 1.0, .description = "frames"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 8, .resolution = 1.0},
      {.name = "PGN", .camelName = "pgn", .fieldType = "PGN", .size = 24}
     },
     .camelDescription = "isoTransportProtocolConnectionManagementBroadcastAnnounce",
     .interval = UINT16_MAX,
     .explanation = "ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting messages that have 9 or more data bytes. This PGN's role in the transport process is to announce a broadcast of a long message spanning multiple frames.",
     .url = "https://embeddedflakes.com/j1939-transport-protocol/"},

    {"ISO Transport Protocol, Connection Management - Abort",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Group Function Code", .camelName = "groupFunctionCode", .fieldType = "LOOKUP", .size = 8, .resolution = 1.0, .hasMatchValue = true, .matchValue = 255, .description = "Abort", .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupISO_COMMAND, .lookup.name = "ISO_COMMAND"},
      {.name = "Reason", .camelName = "reason", .fieldType = "BINARY", .size = 8, .resolution = 1.0},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 24, .resolution = 1.0},
      {.name = "PGN", .camelName = "pgn", .fieldType = "PGN", .size = 24}
     },
     .camelDescription = "isoTransportProtocolConnectionManagementAbort",
     .interval = UINT16_MAX,
     .explanation = "ISO 11783 defines this group function PGN as part of the Transport Protocol method used for transmitting messages that have 9 or more data bytes. This PGN's role in the transport process is to announce an abort of a long message spanning multiple frames.",
     .url = "https://embeddedflakes.com/j1939-transport-protocol/"},

    {"ISO Address Claim",
     60928,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Unique Number", .camelName = "uniqueNumber", .fieldType = "UNSIGNED_INTEGER", .size = 21, .resolution = 1.0, .description = "ISO Identity Number"},
      {.name = "Manufacturer Code", .camelName = "manufacturerCode", .fieldType = "LOOKUP", .size = 11, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupMANUFACTURER_CODE, .lookup.name = "MANUFACTURER_CODE"},
      {.name = "Device Instance Lower", .camelName = "deviceInstanceLower", .fieldType = "UNSIGNED_INTEGER", .size = 3, .resolution = 1.0, .description = "ISO ECU Instance"},
      {.name = "Device Instance Upper", .camelName = "deviceInstanceUpper", .fieldType = "UNSIGNED_INTEGER", .size = 5, .resolution = 1.0, .description = "ISO Function Instance"},
      {.name = "Device Function", .camelName = "deviceFunction", .fieldType = "INDIRECT_LOOKUP", .size = 8, .resolution = 1.0, .description = "ISO Function", .lookup.type = LOOKUP_TYPE_TRIPLET, LOOKUP_TRIPLET_MEMBER = lookupDEVICE_FUNCTION, .lookup.name = "DEVICE_FUNCTION", .lookup.val1Order = 7},
      {.name = "Spare", .camelName = "spare", .fieldType = "SPARE", .size = 1, .resolution = 1.0},
      {.name = "Device Class", .camelName = "deviceClass", .fieldType = "LOOKUP", .size = 7, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupDEVICE_CLASS, .lookup.name = "DEVICE_CLASS"},
      {.name = "System Instance", .camelName = "systemInstance", .fieldType = "UNSIGNED_INTEGER", .size = 4, .resolution = 1.0, .description = "ISO Device Class Instance"},
      {.name = "Industry Group", .camelName = "industryGroup", .fieldType = "LOOKUP", .size = 3, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupINDUSTRY_CODE, .lookup.name = "INDUSTRY_CODE"},
      {.name = "Arbitrary address capable", .camelName = "arbitraryAddressCapable", .fieldType = "UNSIGNED_INTEGER", .size = 1, .resolution = 1.0, .description = "Field indicates whether the device is capable to claim arbitrary source address. Value is 1 for NMEA200 devices. Could be 0 for J1939 device claims"}
     },
     .camelDescription = "isoAddressClaim",
     .interval = UINT16_MAX,
     .explanation = "This network management message is used to claim network address, reply to devices requesting the claimed address, and to respond with device information (NAME) requested by the ISO Request or Complex Request Group Function. This PGN contains several fields that are requestable, either independently or in any combination."},

    {"0xEF00: Manufacturer Proprietary single-frame addressed",
     61184,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Manufacturer Code", .camelName = "manufacturerCode", .fieldType = "LOOKUP", .size = 11, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupMANUFACTURER_CODE, .lookup.name = "MANUFACTURER_CODE"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 2, .resolution = 1.0},
      {.name = "Industry Code", .camelName = "industryCode", .fieldType = "LOOKUP", .size = 3, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupINDUSTRY_CODE, .lookup.name = "INDUSTRY_CODE"},
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 48, .resolution = 1.0}
     },
     .camelDescription = "0xef00ManufacturerProprietarySingleFrameAddressed",
     .fallback = true,
     .explanation = "Manufacturer proprietary PGNs in PDU1 (addressed) single-frame PGN 0xEF00 (61184). When this is shown during analysis it means the PGN is not reverse engineered yet."},

    {"0xF000-0xFEFF: Standardized single-frame non-addressed",
     61440,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Manufacturer Code", .camelName = "manufacturerCode", .fieldType = "LOOKUP", .size = 11, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupMANUFACTURER_CODE, .lookup.name = "MANUFACTURER_CODE"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 2, .resolution = 1.0},
      {.name = "Industry Code", .camelName = "industryCode", .fieldType = "LOOKUP", .size = 3, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupINDUSTRY_CODE, .lookup.name = "INDUSTRY_CODE"},
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 48, .resolution = 1.0}
     },
     .camelDescription = "0xf0000xfeffStandardizedSingleFrameNonAddressed",
     .fallback = true,
     .explanation = "PGNs in PDU2 (non-addressed) single-frame PGN range 0xF000 to 0xFEFF (61440 - 65279). When this is shown during analysis it means the PGN is not reverse engineered yet."},

    {"Electronic Transmission Controller 1",
     61442,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Transmission Driveline Engaged", .camelName = "transmissionDrivelineEngaged", .fieldType = "UNSIGNED_INTEGER", .size = 2, .resolution = 1.0},
      {.name = "Torque Converter Lockup Engaged", .camelName = "torqueConverterLockupEngaged", .fieldType = "UNSIGNED_INTEGER", .size = 2, .resolution = 1.0},
      {.name = "Transmission Shift in Progress", .camelName = "transmissionShiftInProgress", .fieldType = "UNSIGNED_INTEGER", .size = 2, .resolution = 1.0},
      {.name = "Torque Converter Lockup Transition in Progress", .camelName = "torqueConverterLockupTransitionInProgress", .fieldType = "UNSIGNED_INTEGER", .size = 2, .resolution = 1.0},
      {.name = "Transmission Output Shaft Speed", .camelName = "transmissionOutputShaftSpeed", .fieldType = "ROTATION_UFIX16_RPM"},
      {.name = "Percent Clutch Slip", .camelName = "percentClutchSlip", .fieldType = "UINT8", .resolution = 1.0},
      {.name = "Engine Momentary Overspeed Enable", .camelName = "engineMomentaryOverspeedEnable", .fieldType = "LOOKUP", .size = 2, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupYES_NO, .lookup.name = "YES_NO"},
      {.name = "Progressive Shift Disable", .camelName = "progressiveShiftDisable", .fieldType = "LOOKUP", .size = 2, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupYES_NO, .lookup.name = "YES_NO"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 4, .resolution = 1.0},
      {.name = "Transmission Input Shaft Speed", .camelName = "transmissionInputShaftSpeed", .fieldType = "UINT16", .resolution = 1.0},
      {.name = "Source Address of Controlling Device for Transmission Control", .camelName = "sourceAddressOfControllingDeviceForTransmissionControl", .fieldType = "UINT8", .resolution = 1.0}
     },
     .camelDescription = "electronicTransmissionController1",
     .explanation = "J1939 ETC1 - Electronic Transmission Controller 1. Reports transmission status including driveline engagement, torque converter lockup, output/input shaft speeds, and clutch slip."},

    {"ECU #2",
     61443,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 8, .resolution = 1.0},
      {.name = "Throttle Lever", .camelName = "throttleLever", .fieldType = "PERCENTAGE_UINT8", .resolution = 0.4},
      {.name = "Reserved", .camelName = "reserved3", .fieldType = "RESERVED", .size = 48, .resolution = 1.0}
     },
     .camelDescription = "ecu2"},

    {"ECU #1",
     61444,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 24, .resolution = 1.0},
      {.name = "Engine RPM", .camelName = "engineRpm", .fieldType = "ROTATION_UFIX16_RPM_HIGHRES"},
      {.name = "Reserved", .camelName = "reserved3", .fieldType = "RESERVED", .size = 24, .resolution = 1.0}
     },
     .camelDescription = "ecu1"},

    {"Electronic Transmission Controller 2",
     61445,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Transmission Selected Gear", .camelName = "transmissionSelectedGear", .fieldType = "INT8", .resolution = 1.0, .hasSign = true, .description = "SPN 524, offset by -125 in J1939; negative gears are reverse, 0 is neutral"},
      {.name = "Transmission Actual Gear Ratio", .camelName = "transmissionActualGearRatio", .fieldType = "NUMBER", .size = 16, .resolution = 0.001, .description = "SPN 526, 0.001 per bit"},
      {.name = "Transmission Current Gear", .camelName = "transmissionCurrentGear", .fieldType = "INT8", .resolution = 1.0, .hasSign = true, .description = "SPN 523, offset by -125 in J1939; negative gears are reverse, 0 is neutral"},
      {.name = "Transmission Requested Range", .camelName = "transmissionRequestedRange", .fieldType = "STRING_FIX", .size = 16, .description = "SPN 162, two ASCII characters"},
      {.name = "Transmission Current Range", .camelName = "transmissionCurrentRange", .fieldType = "STRING_FIX", .size = 16, .description = "SPN 163, two ASCII characters"}
     },
     .camelDescription = "electronicTransmissionController2",
     .interval = 100,
     .priority = 6},

    {"Engine Gas Flow Rate 1",
     61450,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Exhaust Gas Recirculation 1 Mass Flow Rate", .camelName = "engineExhaustGasRecirculation1MassFlowRate", .fieldType = "NUMBER", .size = 16, .resolution = 0.05, .unit = "kg/h", .description = "SPN 2659, 0.05 kg/h per bit"},
      {.name = "Engine Intake Air Mass Flow Rate", .camelName = "engineIntakeAirMassFlowRate", .fieldType = "NUMBER", .size = 16, .resolution = 0.05, .unit = "kg/h", .description = "SPN 132, 0.05 kg/h per bit"},
      {.name = "Engine Exhaust Gas Recirculation 2 Mass Flow Rate", .camelName = "engineExhaustGasRecirculation2MassFlowRate", .fieldType = "NUMBER", .size = 16, .resolution = 0.05, .unit = "kg/h", .description = "SPN 5257, 0.05 kg/h per bit"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 16, .resolution = 1.0}
     },
     .camelDescription = "engineGasFlowRate1",
     .interval = 50,
     .priority = 6},

    {"Aftertreatment 1 SCR Exhaust Gas Temperature 2",
     64709,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Aftertreatment 1 SCR Catalyst Intake Gas Temperature 2", .camelName = "aftertreatment1ScrCatalystIntakeGasTemperature2", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 5862"},
      {.name = "Aftertreatment 1 SCR Catalyst Intake Gas Temperature 2 Preliminary FMI", .camelName = "aftertreatment1ScrCatalystIntakeGasTemperature2PreliminaryFmi", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 5863"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 43, .resolution = 1.0}
     },
     .camelDescription = "aftertreatment1ScrExhaustGasTemperature2",
     .interval = 1000,
     .priority = 6},

    {"Engine Fuel Properties",
     64740,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Fuel Dynamic Viscosity", .camelName = "engineFuelDynamicViscosity", .fieldType = "NUMBER", .size = 16, .resolution = 1.0, .description = "SPN 5537"},
      {.name = "Engine Fuel Density", .camelName = "engineFuelDensity", .fieldType = "NUMBER", .size = 16, .resolution = 1.0, .description = "SPN 5538"},
      {.name = "Engine Fuel Relative Dielectricity", .camelName = "engineFuelRelativeDielectricity", .fieldType = "NUMBER", .size = 16, .resolution = 1.0, .description = "SPN 5539"},
      {.name = "Engine Fuel Temperature 2", .camelName = "engineFuelTemperature2", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 5540"}
     },
     .camelDescription = "engineFuelProperties",
     .interval = 1000,
     .priority = 6},

    {"Engine Fluid Level/Pressure 11",
     64751,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Exhaust Gas Recirculation 1 Intake Absolute Pressure", .camelName = "engineExhaustGasRecirculation1IntakeAbsolutePressure", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 5430, 0.1 kPa per bit"},
      {.name = "Engine Exhaust Gas Recirculation 1 Outlet Absolute Pressure", .camelName = "engineExhaustGasRecirculation1OutletAbsolutePressure", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 5431, 0.1 kPa per bit"},
      {.name = "Engine Exhaust Gas Recirculation 2 Intake Absolute Pressure", .camelName = "engineExhaustGasRecirculation2IntakeAbsolutePressure", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 7468, 0.1 kPa per bit"},
      {.name = "Engine Exhaust Gas Recirculation 2 Outlet Absolute Pressure", .camelName = "engineExhaustGasRecirculation2OutletAbsolutePressure", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 7469, 0.1 kPa per bit"}
     },
     .camelDescription = "engineFluidLevelPressure11",
     .interval = 1000,
     .priority = 6},

    {"Engine Oil Message",
     64776,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Oil Viscosity", .camelName = "engineOilViscosity", .fieldType = "NUMBER", .size = 16, .resolution = 0.015625, .unit = "cP", .description = "SPN 5055, 0.015625 cP per bit"},
      {.name = "Engine Oil Density", .camelName = "engineOilDensity", .fieldType = "NUMBER", .size = 16, .resolution = 3.05175781e-5, .unit = "g/cm3", .description = "SPN 5056, 1/32768 g/cm3 per bit"},
      {.name = "Engine Oil Relative Dielectricity", .camelName = "engineOilRelativeDielectricity", .fieldType = "NUMBER", .size = 16, .resolution = 0.0001220703125, .description = "SPN 5468, 1/8192 per bit"},
      {.name = "Engine Oil Temperature 3", .camelName = "engineOilTemperature3", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 5925"}
     },
     .camelDescription = "engineOilMessage",
     .interval = 1000,
     .priority = 6},

    {"Aftertreatment 1 Diesel Oxidation Catalyst",
     64800,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Aftertreatment 1 Diesel Oxidation Catalyst Intake Gas Temperature", .camelName = "aftertreatment1DieselOxidationCatalystIntakeGasTemperature", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 4765"},
      {.name = "Aftertreatment 1 Diesel Oxidation Catalyst Outlet Gas Temperature", .camelName = "aftertreatment1DieselOxidationCatalystOutletGasTemperature", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 4766"},
      {.name = "Aftertreatment 1 Diesel Oxidation Catalyst Differential Pressure", .camelName = "aftertreatment1DieselOxidationCatalystDifferentialPressure", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 4767, 0.1 kPa per bit"},
      {.name = "Aftertreatment 1 Diesel Oxidation Catalyst Intake Gas Temperature Preliminary FMI", .camelName = "aftertreatment1DieselOxidationCatalystIntakeGasTemperaturePreliminaryFmi", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 4768"},
      {.name = "Aftertreatment 1 Diesel Oxidation Catalyst Outlet Gas Temperature Preliminary FMI", .camelName = "aftertreatment1DieselOxidationCatalystOutletGasTemperaturePreliminaryFmi", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 4769"},
      {.name = "Aftertreatment 1 Diesel Oxidation Catalyst Differential Pressure Preliminary FMI", .camelName = "aftertreatment1DieselOxidationCatalystDifferentialPressurePreliminaryFmi", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 4770"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 1, .resolution = 1.0}
     },
     .camelDescription = "aftertreatment1DieselOxidationCatalyst",
     .interval = 500,
     .priority = 6},

    {"Aftertreatment 1 Gas Parameters",
     64908,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Aftertreatment 1 Diesel Particulate Filter Intake Pressure", .camelName = "aftertreatment1DieselParticulateFilterIntakePressure", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 3609, 0.1 kPa per bit"},
      {.name = "Aftertreatment 1 Diesel Particulate Filter Outlet Pressure", .camelName = "aftertreatment1DieselParticulateFilterOutletPressure", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 3610, 0.1 kPa per bit"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 32, .resolution = 1.0}
     },
     .camelDescription = "aftertreatment1GasParameters",
     .interval = 500,
     .priority = 6},

    {"Engine Operating Information",
     64914,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Operating State", .camelName = "engineOperatingState", .fieldType = "NUMBER", .size = 4, .resolution = 1.0, .description = "SPN 3543"},
      {.name = "Fuel Pump Primer Control", .camelName = "fuelPumpPrimerControl", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 4082"},
      {.name = "Engine Automatic Start Enable Status", .camelName = "engineAutomaticStartEnableStatus", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 6385"},
      {.name = "Time Remaining in Engine Operating State", .camelName = "timeRemainingInEngineOperatingState", .fieldType = "DURATION_UFIX16_S", .description = "SPN 3544"},
      {.name = "Engine Fuel Shutoff Vent Control", .camelName = "engineFuelShutoffVentControl", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3608"},
      {.name = "Engine Fuel Shutoff 1 Control", .camelName = "engineFuelShutoff1Control", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 632"},
      {.name = "Engine Fuel Shutoff 2 Control", .camelName = "engineFuelShutoff2Control", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 2807"},
      {.name = "Engine Fuel Shutoff Valve Leak Test Control", .camelName = "engineFuelShutoffValveLeakTestControl", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3601"},
      {.name = "Engine Oil Priming Pump Control", .camelName = "engineOilPrimingPumpControl", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3589"},
      {.name = "Engine Oil Preheater Control", .camelName = "engineOilPreheaterControl", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3602"},
      {.name = "Engine Electrical System Power Conservation Control", .camelName = "engineElectricalSystemPowerConservationControl", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3603"},
      {.name = "Engine Block/Coolant Preheater Control", .camelName = "engineBlockCoolantPreheaterControl", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3604"},
      {.name = "Engine Coolant Circulating Pump Control", .camelName = "engineCoolantCirculatingPumpControl", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3605"},
      {.name = "Engine Controlled Shutdown Request", .camelName = "engineControlledShutdownRequest", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3606"},
      {.name = "Engine Emergency (Immediate) Shutdown Indication", .camelName = "engineEmergencyImmediateShutdownIndication", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3607"},
      {.name = "Engine Automatic Start Safety Interlock Status", .camelName = "engineAutomaticStartSafetyInterlockStatus", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 6884"},
      {.name = "Engine Automatic Start Enable Time Remaining", .camelName = "engineAutomaticStartEnableTimeRemaining", .fieldType = "NUMBER", .size = 8, .resolution = 1.0, .description = "SPN 6807"},
      {.name = "Engine Derate Request", .camelName = "engineDerateRequest", .fieldType = "PERCENTAGE_UINT8", .resolution = 0.4, .description = "SPN 3644"}
     },
     .camelDescription = "engineOperatingInformation",
     .interval = 1000,
     .priority = 6},

    {"Aftertreatment 1 Intermediate Gas",
     64946,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Aftertreatment 1 Exhaust Gas Temperature 2", .camelName = "aftertreatment1ExhaustGasTemperature2", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 3249"},
      {.name = "Aftertreatment 1 Diesel Particulate Filter Intermediate Temperature", .camelName = "aftertreatment1DieselParticulateFilterIntermediateTemperature", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 3250"},
      {.name = "Aftertreatment 1 Diesel Particulate Filter Differential Pressure", .camelName = "aftertreatment1DieselParticulateFilterDifferentialPressure", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 3251, 0.1 kPa per bit"},
      {.name = "Aftertreatment 1 Exhaust Gas Temperature 2 Preliminary FMI", .camelName = "aftertreatment1ExhaustGasTemperature2PreliminaryFmi", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 3252"},
      {.name = "Aftertreatment 1 Diesel Particulate Filter Intermediate Temperature Preliminary FMI", .camelName = "aftertreatment1DieselParticulateFilterIntermediateTemperaturePreliminaryFmi", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 3253"},
      {.name = "Aftertreatment 1 Diesel Particulate Filter Differential Pressure Preliminary FMI", .camelName = "aftertreatment1DieselParticulateFilterDifferentialPressurePreliminaryFmi", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 3254"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 1, .resolution = 1.0}
     },
     .camelDescription = "aftertreatment1IntermediateGas",
     .interval = 500,
     .priority = 6},

    {"Aftertreatment 1 Outlet Gas 2",
     64947,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Aftertreatment 1 Outlet Gas Temperature 2", .camelName = "aftertreatment1OutletGasTemperature2", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 3245"},
      {.name = "Aftertreatment 1 Outlet Gas Pressure 2", .camelName = "aftertreatment1OutletGasPressure2", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 3246, 0.1 kPa per bit"},
      {.name = "Aftertreatment 1 Outlet Gas Temperature 2 Preliminary FMI", .camelName = "aftertreatment1OutletGasTemperature2PreliminaryFMI", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 3247"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 3, .resolution = 1.0},
      {.name = "Aftertreatment 1 Outlet Gas Pressure 2 Preliminary FMI", .camelName = "aftertreatment1OutletGasPressure2PreliminaryFMI", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 3248"},
      {.name = "Reserved", .camelName = "reserved2", .fieldType = "RESERVED", .size = 19, .resolution = 1.0}
     },
     .camelDescription = "aftertreatment1OutletGas2",
     .interval = 500,
     .priority = 6},

    {"Aftertreatment 1 Intake Gas 2",
     64948,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Aftertreatment 1 Intake Gas Temperature 2", .camelName = "aftertreatment1IntakeGasTemperature2", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 3241"},
      {.name = "Aftertreatment 1 Intake Gas Pressure 2", .camelName = "aftertreatment1IntakeGasPressure2", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 3242, 0.1 kPa per bit"},
      {.name = "Aftertreatment 1 Intake Gas Temperature 2 Preliminary FMI", .camelName = "aftertreatment1IntakeGasTemperature2PreliminaryFMI", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 3243"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 3, .resolution = 1.0},
      {.name = "Aftertreatment 1 Intake Gas Pressure 2 Preliminary FMI", .camelName = "aftertreatment1IntakeGasPressure2PreliminaryFMI", .fieldType = "NUMBER", .size = 5, .resolution = 1.0, .description = "SPN 3244"},
      {.name = "Reserved", .camelName = "reserved2", .fieldType = "RESERVED", .size = 19, .resolution = 1.0}
     },
     .camelDescription = "aftertreatment1IntakeGas2",
     .interval = 500,
     .priority = 6},

    {"Intake/Exhaust Conditions 2",
     64976,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Air Filter 2 Differential Pressure", .camelName = "engineAirFilter2DifferentialPressure", .fieldType = "PRESSURE_UINT8_005KPA", .description = "SPN 2809, 0.05 kPa per bit"},
      {.name = "Engine Air Filter 3 Differential Pressure", .camelName = "engineAirFilter3DifferentialPressure", .fieldType = "PRESSURE_UINT8_005KPA", .description = "SPN 2810, 0.05 kPa per bit"},
      {.name = "Engine Air Filter 4 Differential Pressure", .camelName = "engineAirFilter4DifferentialPressure", .fieldType = "PRESSURE_UINT8_005KPA", .description = "SPN 2811, 0.05 kPa per bit"},
      {.name = "Engine Intake Manifold", .camelName = "engineIntakeManifold2Pressure", .fieldType = "PRESSURE_UINT8_2KPA", .description = "SPN 3562"},
      {.name = "Engine Intake Manifold", .camelName = "engineIntakeManifold1AbsolutePressure", .fieldType = "NUMBER", .size = 8, .resolution = 0.1, .unit = "kPa", .description = "SPN 3563, 0.1 kPa per bit"},
      {.name = "Engine Intake Manifold 1 Absolute Pressure (High Resolution)", .camelName = "engineIntakeManifold1AbsolutePressureHighResolution", .fieldType = "PRESSURE_UFIX16_HPA", .description = "SPN 4817, 0.1 kPa per bit"},
      {.name = "Engine Intake Manifold 2 Absolute Pressure", .camelName = "engineIntakeManifold2AbsolutePressure", .fieldType = "PRESSURE_UINT8_2KPA", .description = "SPN 5422"}
     },
     .camelDescription = "intakeExhaustConditions2",
     .interval = 500,
     .priority = 6},

    {"Bus #1 Phase C Basic AC Quantities",
     65001,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 16, .resolution = 1.0}
     },
     .camelDescription = "bus1PhaseCBasicAcQuantities"},

    {"Bus #1 Phase B Basic AC Quantities",
     65002,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 16, .resolution = 1.0}
     },
     .camelDescription = "bus1PhaseBBasicAcQuantities"},

    {"Bus #1 Phase A Basic AC Quantities",
     65003,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 16, .resolution = 1.0}
     },
     .camelDescription = "bus1PhaseABasicAcQuantities"},

    {"Bus #1 Average Basic AC Quantities",
     65004,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 16, .resolution = 1.0}
     },
     .camelDescription = "bus1AverageBasicAcQuantities"},

    {"Utility Total AC Energy",
     65005,
     PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Total Energy Export", .camelName = "totalEnergyExport", .fieldType = "ENERGY_UINT32", .resolution = 1.0},
      {.name = "Total Energy Import", .camelName = "totalEnergyImport", .fieldType = "ENERGY_UINT32", .resolution = 1.0}
     },
     .camelDescription = "utilityTotalAcEnergy"},

    {"Utility Phase C AC Reactive Power",
     65006,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Reactive Power", .camelName = "reactivePower", .fieldType = "POWER_UINT16_VAR"},
      {.name = "Power factor", .camelName = "powerFactor", .fieldType = "UINT16", .resolution = 6.103515625e-5, .unit = "Cos Phi"},
      {.name = "Power Factor Lagging", .camelName = "powerFactorLagging", .fieldType = "LOOKUP", .size = 2, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupPOWER_FACTOR, .lookup.name = "POWER_FACTOR"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 30, .resolution = 1.0}
     },
     .camelDescription = "utilityPhaseCAcReactivePower"},

    {"Utility Phase C AC Power",
     65007,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Real Power", .camelName = "realPower", .fieldType = "POWER_FIX32_OFFSET", .hasSign = true},
      {.name = "Apparent Power", .camelName = "apparentPower", .fieldType = "POWER_FIX32_VA_OFFSET", .hasSign = true}
     },
     .camelDescription = "utilityPhaseCAcPower"},

    {"Utility Phase C Basic AC Quantities",
     65008,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "AC RMS Current", .camelName = "acRmsCurrent", .fieldType = "CURRENT_UFIX16_A"}
     },
     .camelDescription = "utilityPhaseCBasicAcQuantities"},

    {"Utility Phase B AC Reactive Power",
     65009,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Reactive Power", .camelName = "reactivePower", .fieldType = "POWER_UINT16_VAR"},
      {.name = "Power factor", .camelName = "powerFactor", .fieldType = "UINT16", .resolution = 6.103515625e-5, .unit = "Cos Phi"},
      {.name = "Power Factor Lagging", .camelName = "powerFactorLagging", .fieldType = "LOOKUP", .size = 2, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupPOWER_FACTOR, .lookup.name = "POWER_FACTOR"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 30, .resolution = 1.0}
     },
     .camelDescription = "utilityPhaseBAcReactivePower"},

    {"Utility Phase B AC Power",
     65010,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Real Power", .camelName = "realPower", .fieldType = "POWER_FIX32_OFFSET", .hasSign = true},
      {.name = "Apparent Power", .camelName = "apparentPower", .fieldType = "POWER_FIX32_VA_OFFSET", .hasSign = true}
     },
     .camelDescription = "utilityPhaseBAcPower"},

    {"Utility Phase B Basic AC Quantities",
     65011,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "AC RMS Current", .camelName = "acRmsCurrent", .fieldType = "CURRENT_UFIX16_A"}
     },
     .camelDescription = "utilityPhaseBBasicAcQuantities"},

    {"Utility Phase A AC Reactive Power",
     65012,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Reactive Power", .camelName = "reactivePower", .fieldType = "POWER_FIX32_VAR_OFFSET", .hasSign = true},
      {.name = "Power factor", .camelName = "powerFactor", .fieldType = "UINT16", .resolution = 6.103515625e-5, .unit = "Cos Phi"},
      {.name = "Power Factor Lagging", .camelName = "powerFactorLagging", .fieldType = "LOOKUP", .size = 2, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupPOWER_FACTOR, .lookup.name = "POWER_FACTOR"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 14, .resolution = 1.0}
     },
     .camelDescription = "utilityPhaseAAcReactivePower"},

    {"Utility Phase A AC Power",
     65013,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Real Power", .camelName = "realPower", .fieldType = "POWER_FIX32_OFFSET", .hasSign = true},
      {.name = "Apparent Power", .camelName = "apparentPower", .fieldType = "POWER_FIX32_VA_OFFSET", .hasSign = true}
     },
     .camelDescription = "utilityPhaseAAcPower"},

    {"Utility Phase A Basic AC Quantities",
     65014,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "AC RMS Current", .camelName = "acRmsCurrent", .fieldType = "CURRENT_UFIX16_A"}
     },
     .camelDescription = "utilityPhaseABasicAcQuantities"},

    {"Utility Total AC Reactive Power",
     65015,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Reactive Power", .camelName = "reactivePower", .fieldType = "POWER_FIX32_VAR_OFFSET", .hasSign = true},
      {.name = "Power factor", .camelName = "powerFactor", .fieldType = "UINT16", .resolution = 6.103515625e-5, .unit = "Cos Phi"},
      {.name = "Power Factor Lagging", .camelName = "powerFactorLagging", .fieldType = "LOOKUP", .size = 2, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupPOWER_FACTOR, .lookup.name = "POWER_FACTOR"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 14, .resolution = 1.0}
     },
     .camelDescription = "utilityTotalAcReactivePower"},

    {"Utility Total AC Power",
     65016,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Real Power", .camelName = "realPower", .fieldType = "POWER_FIX32_OFFSET", .hasSign = true},
      {.name = "Apparent Power", .camelName = "apparentPower", .fieldType = "POWER_FIX32_VA_OFFSET", .hasSign = true}
     },
     .camelDescription = "utilityTotalAcPower"},

    {"Utility Average Basic AC Quantities",
     65017,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "AC RMS Current", .camelName = "acRmsCurrent", .fieldType = "CURRENT_UFIX16_A"}
     },
     .camelDescription = "utilityAverageBasicAcQuantities"},

    {"Generator Total AC Energy",
     65018,
     PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Total Energy Export", .camelName = "totalEnergyExport", .fieldType = "ENERGY_UINT32", .resolution = 1.0},
      {.name = "Total Energy Import", .camelName = "totalEnergyImport", .fieldType = "ENERGY_UINT32", .resolution = 1.0}
     },
     .camelDescription = "generatorTotalAcEnergy"},

    {"Generator Phase C AC Reactive Power",
     65019,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Reactive Power", .camelName = "reactivePower", .fieldType = "POWER_FIX32_VAR_OFFSET", .hasSign = true},
      {.name = "Power factor", .camelName = "powerFactor", .fieldType = "UINT16", .resolution = 6.103515625e-5, .unit = "Cos Phi"},
      {.name = "Power Factor Lagging", .camelName = "powerFactorLagging", .fieldType = "LOOKUP", .size = 2, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupPOWER_FACTOR, .lookup.name = "POWER_FACTOR"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 14, .resolution = 1.0}
     },
     .camelDescription = "generatorPhaseCAcReactivePower"},

    {"Generator Phase C AC Power",
     65020,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Real Power", .camelName = "realPower", .fieldType = "POWER_FIX32_OFFSET", .hasSign = true},
      {.name = "Apparent Power", .camelName = "apparentPower", .fieldType = "POWER_FIX32_VA_OFFSET", .hasSign = true}
     },
     .camelDescription = "generatorPhaseCAcPower"},

    {"Generator Phase C Basic AC Quantities",
     65021,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "AC RMS Current", .camelName = "acRmsCurrent", .fieldType = "CURRENT_UFIX16_A"}
     },
     .camelDescription = "generatorPhaseCBasicAcQuantities"},

    {"Generator Phase B AC Reactive Power",
     65022,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Reactive Power", .camelName = "reactivePower", .fieldType = "POWER_FIX32_VAR_OFFSET", .hasSign = true},
      {.name = "Power factor", .camelName = "powerFactor", .fieldType = "UINT16", .resolution = 6.103515625e-5, .unit = "Cos Phi"},
      {.name = "Power Factor Lagging", .camelName = "powerFactorLagging", .fieldType = "LOOKUP", .size = 2, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupPOWER_FACTOR, .lookup.name = "POWER_FACTOR"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 14, .resolution = 1.0}
     },
     .camelDescription = "generatorPhaseBAcReactivePower"},

    {"Generator Phase B AC Power",
     65023,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Real Power", .camelName = "realPower", .fieldType = "POWER_FIX32_OFFSET", .hasSign = true},
      {.name = "Apparent Power", .camelName = "apparentPower", .fieldType = "POWER_FIX32_VA_OFFSET", .hasSign = true}
     },
     .camelDescription = "generatorPhaseBAcPower"},

    {"Generator Phase B Basic AC Quantities",
     65024,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "AC RMS Current", .camelName = "acRmsCurrent", .fieldType = "CURRENT_UFIX16_A"}
     },
     .camelDescription = "generatorPhaseBBasicAcQuantities"},

    {"Generator Phase A AC Reactive Power",
     65025,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Reactive Power", .camelName = "reactivePower", .fieldType = "POWER_FIX32_VAR_OFFSET", .hasSign = true},
      {.name = "Power factor", .camelName = "powerFactor", .fieldType = "UINT16", .resolution = 6.103515625e-5, .unit = "Cos Phi"},
      {.name = "Power Factor Lagging", .camelName = "powerFactorLagging", .fieldType = "LOOKUP", .size = 2, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupPOWER_FACTOR, .lookup.name = "POWER_FACTOR"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 14, .resolution = 1.0}
     },
     .camelDescription = "generatorPhaseAAcReactivePower"},

    {"Generator Phase A AC Power",
     65026,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Real Power", .camelName = "realPower", .fieldType = "POWER_FIX32_OFFSET", .hasSign = true},
      {.name = "Apparent Power", .camelName = "apparentPower", .fieldType = "POWER_FIX32_VA_OFFSET", .hasSign = true}
     },
     .camelDescription = "generatorPhaseAAcPower"},

    {"Generator Phase A Basic AC Quantities",
     65027,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "AC RMS Current", .camelName = "acRmsCurrent", .fieldType = "CURRENT_UFIX16_A"}
     },
     .camelDescription = "generatorPhaseABasicAcQuantities"},

    {"Generator Total AC Reactive Power",
     65028,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Reactive Power", .camelName = "reactivePower", .fieldType = "POWER_FIX32_VAR_OFFSET", .hasSign = true},
      {.name = "Power factor", .camelName = "powerFactor", .fieldType = "UINT16", .resolution = 6.103515625e-5, .unit = "Cos Phi"},
      {.name = "Power Factor Lagging", .camelName = "powerFactorLagging", .fieldType = "LOOKUP", .size = 2, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupPOWER_FACTOR, .lookup.name = "POWER_FACTOR"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 14, .resolution = 1.0}
     },
     .camelDescription = "generatorTotalAcReactivePower"},

    {"Generator Total AC Power",
     65029,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Real Power", .camelName = "realPower", .fieldType = "POWER_FIX32_OFFSET", .hasSign = true},
      {.name = "Apparent Power", .camelName = "apparentPower", .fieldType = "POWER_FIX32_VA_OFFSET", .hasSign = true}
     },
     .camelDescription = "generatorTotalAcPower"},

    {"Generator Average Basic AC Quantities",
     65030,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Line-Line AC RMS Voltage", .camelName = "lineLineAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "Line-Neutral AC RMS Voltage", .camelName = "lineNeutralAcRmsVoltage", .fieldType = "VOLTAGE_UFIX16_V"},
      {.name = "AC Frequency", .camelName = "acFrequency", .fieldType = "FREQUENCY_UFIX16", .resolution = 0.0078125},
      {.name = "AC RMS Current", .camelName = "acRmsCurrent", .fieldType = "CURRENT_UFIX16_A"}
     },
     .camelDescription = "generatorAverageBasicAcQuantities"},

    {"Exhaust Temperature",
     65031,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Exhaust Manifold Bank 2 Temperature 1", .camelName = "engineExhaustManifoldBank2Temperature1", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 2433"},
      {.name = "Engine Exhaust Manifold Bank 1 Temperature 1", .camelName = "engineExhaustManifoldBank1Temperature1", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 2434"},
      {.name = "Engine Exhaust Manifold Bank 2 Temperature 2", .camelName = "engineExhaustManifoldBank2Temperature2", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 5969"},
      {.name = "Engine Exhaust Manifold Bank 1 Temperature 2", .camelName = "engineExhaustManifoldBank1Temperature2", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 5970"}
     },
     .camelDescription = "exhaustTemperature",
     .interval = 1000,
     .priority = 6},

    {"Engine Fuel/Lube Systems",
     65130,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Oil Level Remote Reservoir", .camelName = "engineOilLevelRemoteReservoir", .fieldType = "PERCENTAGE_UINT8", .resolution = 0.4, .description = "SPN 1380"},
      {.name = "Engine Fuel Supply Pump Intake Absolute Pressure", .camelName = "engineFuelSupplyPumpIntakeAbsolutePressure", .fieldType = "PRESSURE_UINT8_2KPA", .description = "SPN 1381"},
      {.name = "Engine Fuel Filter (suction side) Differential Pressure", .camelName = "engineFuelFilterSuctionSideDifferentialPressure", .fieldType = "PRESSURE_UINT8_2KPA", .description = "SPN 1382"},
      {.name = "Engine Waste Oil Reservoir Level", .camelName = "engineWasteOilReservoirLevel", .fieldType = "PERCENTAGE_UINT8", .resolution = 0.4, .description = "SPN 3548"},
      {.name = "Engine Oil Filter Outlet Pressure", .camelName = "engineOilFilterOutletPressure", .fieldType = "NUMBER", .size = 8, .resolution = 4.0, .unit = "kPa", .description = "SPN 3549, 4 kPa per bit"},
      {.name = "Engine Oil Priming Pump Switch", .camelName = "engineOilPrimingPumpSwitch", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3550"},
      {.name = "Engine Oil Priming State", .camelName = "engineOilPrimingState", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3551"},
      {.name = "Engine Oil Pre-Heated State", .camelName = "engineOilPreHeatedState", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3552"},
      {.name = "Engine Coolant Pre-heated State", .camelName = "engineCoolantPreHeatedState", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3553"},
      {.name = "Engine Ventilation Status", .camelName = "engineVentilationStatus", .fieldType = "NUMBER", .size = 3, .resolution = 1.0, .description = "SPN 3554"},
      {.name = "Fuel Pump Primer Status", .camelName = "fuelPumpPrimerStatus", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 4083"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 3, .resolution = 1.0},
      {.name = "Engine Fuel Supply Pump Intake Pressure", .camelName = "engineFuelSupplyPumpIntakePressure", .fieldType = "PRESSURE_UINT8_2KPA", .description = "SPN 7104"}
     },
     .camelDescription = "engineFuelLubeSystems",
     .interval = 500,
     .priority = 6},

    {"Engine Temperature 2",
     65188,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Oil Temperature 2", .camelName = "engineOilTemperature2", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 1135"},
      {.name = "Engine ECU Temperature", .camelName = "engineEcuTemperature", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 1136"},
      {.name = "Engine Exhaust Gas Recirculation 1 Differential Pressure", .camelName = "engineExhaustGasRecirculation1DifferentialPressure", .fieldType = "NUMBER", .size = 16, .resolution = 0.0078125, .unit = "kPa", .description = "SPN 411, 1/128 kPa per bit with a -250 kPa offset"},
      {.name = "Engine Exhaust Gas Recirculation 1 Temperature", .camelName = "engineExhaustGasRecirculation1Temperature", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 412"}
     },
     .camelDescription = "engineTemperature2",
     .interval = 500,
     .priority = 6},

    {"Intake Manifold Information 2",
     65189,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Intake Manifold 2 Temperature", .camelName = "engineIntakeManifold2Temperature", .fieldType = "TEMPERATURE_UINT8_OFFSET", .description = "SPN 1131"},
      {.name = "Engine Intake Manifold 3 Temperature", .camelName = "engineIntakeManifold3Temperature", .fieldType = "TEMPERATURE_UINT8_OFFSET", .description = "SPN 1132"},
      {.name = "Engine Intake Manifold 4 Temperature", .camelName = "engineIntakeManifold4Temperature", .fieldType = "TEMPERATURE_UINT8_OFFSET", .description = "SPN 1133"},
      {.name = "Engine Intake Manifold 5 Temperature", .camelName = "engineIntakeManifold5Temperature", .fieldType = "TEMPERATURE_UINT8_OFFSET", .description = "SPN 1802"},
      {.name = "Engine Intake Manifold 6 Temperature", .camelName = "engineIntakeManifold6Temperature", .fieldType = "TEMPERATURE_UINT8_OFFSET", .description = "SPN 1803"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 24, .resolution = 1.0}
     },
     .camelDescription = "intakeManifoldInformation2",
     .interval = 500,
     .priority = 6},

    {"Intake Manifold Information 1",
     65190,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Turbocharger 1 Boost Pressure", .camelName = "engineTurbocharger1BoostPressure", .fieldType = "NUMBER", .size = 16, .resolution = 0.125, .unit = "kPa", .description = "SPN 1127, 0.125 kPa per bit"},
      {.name = "Engine Turbocharger 2 Boost Pressure", .camelName = "engineTurbocharger2BoostPressure", .fieldType = "NUMBER", .size = 16, .resolution = 0.125, .unit = "kPa", .description = "SPN 1128, 0.125 kPa per bit"},
      {.name = "Engine Turbocharger 3 Boost Pressure", .camelName = "engineTurbocharger3BoostPressure", .fieldType = "NUMBER", .size = 16, .resolution = 0.125, .unit = "kPa", .description = "SPN 1129, 0.125 kPa per bit"},
      {.name = "Engine Turbocharger 4 Boost Pressure", .camelName = "engineTurbocharger4BoostPressure", .fieldType = "NUMBER", .size = 16, .resolution = 0.125, .unit = "kPa", .description = "SPN 1130, 0.125 kPa per bit"}
     },
     .camelDescription = "intakeManifoldInformation1",
     .interval = 500,
     .priority = 6},

    {"ECU History",
     65201,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Total ECU Distance", .camelName = "totalEcuDistance", .fieldType = "NUMBER", .size = 32, .resolution = 0.125, .unit = "km", .description = "SPN 1032, 0.125 kilometre per bit"},
      {.name = "Total ECU Run Time", .camelName = "totalEcuRunTime", .fieldType = "NUMBER", .size = 32, .resolution = 0.05, .unit = "h", .description = "SPN 1033, 0.05 hour per bit"}
     },
     .camelDescription = "ecuHistory",
     .interval = 1000,
     .priority = 6},

    {"Fan Drive",
     65213,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Estimated Percent Fan Speed", .camelName = "estimatedPercentFanSpeed", .fieldType = "PERCENTAGE_UINT8", .resolution = 0.4, .description = "SPN 975"},
      {.name = "Fan Drive State", .camelName = "fanDriveState", .fieldType = "NUMBER", .size = 4, .resolution = 1.0, .description = "SPN 977"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 4, .resolution = 1.0},
      {.name = "Fan Speed", .camelName = "fanSpeed", .fieldType = "ROTATION_UFIX16_RPM_HIGHRES", .description = "SPN 1639"},
      {.name = "Hydraulic Fan Motor Pressure", .camelName = "hydraulicFanMotorPressure", .fieldType = "PRESSURE_UFIX16_KPA", .description = "SPN 4211"},
      {.name = "Hydraulic Fan Motor Pressure 2", .camelName = "hydraulicFanMotorPressure2", .fieldType = "PRESSURE_UINT8_KPA", .description = "SPN 4212"},
      {.name = "Reserved", .camelName = "reserved2", .fieldType = "RESERVED", .size = 8, .resolution = 1.0}
     },
     .camelDescription = "fanDrive",
     .interval = 1000,
     .priority = 6},

    {"Electronic Engine Controller 4",
     65214,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Rated Power", .camelName = "engineRatedPower", .fieldType = "NUMBER", .size = 16, .resolution = 0.5, .unit = "kW", .description = "SPN 166, 0.5 kW per bit"},
      {.name = "Engine Rated Speed", .camelName = "engineRatedSpeed", .fieldType = "ROTATION_UFIX16_RPM_HIGHRES", .description = "SPN 189"},
      {.name = "Engine Rotation Direction", .camelName = "engineRotationDirection", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 3669"},
      {.name = "Engine Intake Manifold Pressure Control Mode", .camelName = "engineIntakeManifoldPressureControlMode", .fieldType = "NUMBER", .size = 2, .resolution = 1.0, .description = "SPN 5465"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 4, .resolution = 1.0},
      {.name = "Crank Attempt Count on Present Start Attempt", .camelName = "crankAttemptCountOnPresentStartAttempt", .fieldType = "NUMBER", .size = 8, .resolution = 1.0, .description = "SPN 3671"},
      {.name = "Reserved", .camelName = "reserved8", .fieldType = "RESERVED", .size = 16, .resolution = 1.0}
     },
     .camelDescription = "electronicEngineController4",
     .priority = 6},

    {"Active Trouble Codes",
     65226,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Malfunction Lamp Status", .camelName = "malfunctionLampStatus", .fieldType = "BINARY", .size = 2, .resolution = 1.0, .description = "Fault Lamps"},
      {.name = "Red Stop Lamp Status", .camelName = "redStopLampStatus", .fieldType = "BINARY", .size = 2, .resolution = 1.0, .description = "Fault Lamps"},
      {.name = "Amber Warning Lamp Status", .camelName = "amberWarningLampStatus", .fieldType = "BINARY", .size = 2, .resolution = 1.0, .description = "Fault Lamps"},
      {.name = "Protect Lamp Status", .camelName = "protectLampStatus", .fieldType = "BINARY", .size = 2, .resolution = 1.0, .description = "Fault Lamps"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 8, .resolution = 1.0},
      {.name = "SPN", .camelName = "spn", .fieldType = "BINARY", .size = 19, .resolution = 1.0, .description = "Suspect Parameter Number"},
      {.name = "FMI", .camelName = "fmi", .fieldType = "BINARY", .size = 5, .resolution = 1.0, .description = "Fault Mode Indicator"},
      {.name = "CM", .camelName = "cm", .fieldType = "BINARY", .size = 1, .resolution = 1.0, .description = "SPN Conversion Method"},
      {.name = "OC", .camelName = "oc", .fieldType = "BINARY", .size = 7, .resolution = 1.0, .description = "Occurance Count"}
     },
     .camelDescription = "activeTroubleCodes",
     .repeatingCount1 = 4,
     .repeatingStart1 = 6,
     .repeatingField1 = 255},

    {"ISO Commanded Address",
     65240,
     PACKET_COMPLETE,
     PACKET_ISO_TP,
     {
      {.name = "Unique Number", .camelName = "uniqueNumber", .fieldType = "BINARY", .size = 21, .resolution = 1.0, .description = "ISO Identity Number"},
      {.name = "Manufacturer Code", .camelName = "manufacturerCode", .fieldType = "LOOKUP", .size = 11, .resolution = 1.0, .unit = "Manufacturer Code", .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupMANUFACTURER_CODE, .lookup.name = "MANUFACTURER_CODE"},
      {.name = "Device Instance Lower", .camelName = "deviceInstanceLower", .fieldType = "UNSIGNED_INTEGER", .size = 3, .resolution = 1.0, .description = "ISO ECU Instance"},
      {.name = "Device Instance Upper", .camelName = "deviceInstanceUpper", .fieldType = "UNSIGNED_INTEGER", .size = 5, .resolution = 1.0, .description = "ISO Function Instance"},
      {.name = "Device Function", .camelName = "deviceFunction", .fieldType = "INDIRECT_LOOKUP", .size = 8, .resolution = 1.0, .description = "ISO Function", .lookup.type = LOOKUP_TYPE_TRIPLET, LOOKUP_TRIPLET_MEMBER = lookupDEVICE_FUNCTION, .lookup.name = "DEVICE_FUNCTION", .lookup.val1Order = 7},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 1, .resolution = 1.0},
      {.name = "Device Class", .camelName = "deviceClass", .fieldType = "LOOKUP", .size = 7, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupDEVICE_CLASS, .lookup.name = "DEVICE_CLASS"},
      {.name = "System Instance", .camelName = "systemInstance", .fieldType = "UNSIGNED_INTEGER", .size = 4, .resolution = 1.0, .description = "ISO Device Class Instance"},
      {.name = "Industry Code", .camelName = "industryCode", .fieldType = "LOOKUP", .size = 3, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupINDUSTRY_CODE, .lookup.name = "INDUSTRY_CODE"},
      {.name = "Reserved", .camelName = "reserved10", .fieldType = "RESERVED", .size = 1, .resolution = 1.0},
      {.name = "New Source Address", .camelName = "newSourceAddress", .fieldType = "UINT8", .resolution = 1.0}
     },
     .camelDescription = "isoCommandedAddress"},

    {"Engine Fluid Level/Pressure 2",
     65243,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Fuel Injection Control Pressure", .camelName = "engineFuelInjectionControlPressure", .fieldType = "NUMBER", .size = 16, .resolution = 0.00390625, .unit = "MPa", .description = "SPN 164, 1/256 MPa per bit"},
      {.name = "Engine Fuel 1 Injector Metering Rail 1 Pressure", .camelName = "engineFuel1InjectorMeteringRail1Pressure", .fieldType = "NUMBER", .size = 16, .resolution = 0.00390625, .unit = "MPa", .description = "SPN 157, 1/256 MPa per bit"},
      {.name = "Engine Fuel 1 Injector Timing Rail 1 Pressure", .camelName = "engineFuel1InjectorTimingRail1Pressure", .fieldType = "NUMBER", .size = 16, .resolution = 0.00390625, .unit = "MPa", .description = "SPN 156, 1/256 MPa per bit"},
      {.name = "Engine Fuel 1 Injector Metering Rail 2 Pressure", .camelName = "engineFuel1InjectorMeteringRail2Pressure", .fieldType = "NUMBER", .size = 16, .resolution = 0.00390625, .unit = "MPa", .description = "SPN 1349, 1/256 MPa per bit"}
     },
     .camelDescription = "engineFluidLevelPressure2",
     .interval = 500,
     .priority = 6},

    {"Idle Operation",
     65244,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Total Idle Fuel Used", .camelName = "engineTotalIdleFuelUsed", .fieldType = "VOLUME_UFIX32_HL", .description = "SPN 236, 0.5 litre per bit"},
      {.name = "Engine Total Idle Hours", .camelName = "engineTotalIdleHours", .fieldType = "DURATION_UFIX32_J1939_HOURS", .description = "SPN 235, 0.05 hour per bit"}
     },
     .camelDescription = "idleOperation",
     .interval = 1000,
     .priority = 6},

    {"Fuel Consumption (Liquid) 1",
     65257,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Engine Trip Fuel", .camelName = "engineTripFuel", .fieldType = "VOLUME_UFIX32_HL", .description = "SPN 182, 0.5 litre per bit"},
      {.name = "Engine Total Fuel Used", .camelName = "engineTotalFuelUsed", .fieldType = "VOLUME_UFIX32_HL", .description = "SPN 250, 0.5 litre per bit"}
     },
     .camelDescription = "fuelConsumptionLiquid1",
     .interval = 1000,
     .priority = 6},

    {"Engine Temp #1",
     65262,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Engine Coolant Temp", .camelName = "engineCoolantTemp", .fieldType = "TEMPERATURE_UINT8_OFFSET"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 56, .resolution = 1.0}
     },
     .camelDescription = "engineTemp1"},

    {"Fuel Economy",
     65266,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 48, .resolution = 1.0},
      {.name = "Throttle Position", .camelName = "throttlePosition", .fieldType = "PERCENTAGE_UINT8", .resolution = 0.4},
      {.name = "Reserved", .camelName = "reserved3", .fieldType = "RESERVED", .size = 8, .resolution = 1.0}
     },
     .camelDescription = "fuelEconomy"},

    {"Ambient Conditions",
     65269,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Barometric Pressure", .camelName = "barometricPressure", .fieldType = "PRESSURE_UINT8_KPA"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 56, .resolution = 1.0}
     },
     .camelDescription = "ambientConditions"},

    {"Inlet/Exhaust Conditions",
     65270,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 16, .resolution = 1.0},
      {.name = "Intake Manifold Temp", .camelName = "intakeManifoldTemp", .fieldType = "TEMPERATURE_UINT8_OFFSET"},
      {.name = "Air Inlet Pressure", .camelName = "airInletPressure", .fieldType = "PRESSURE_UINT8_2KPA"},
      {.name = "Reserved", .camelName = "reserved4", .fieldType = "RESERVED", .size = 32, .resolution = 1.0}
     },
     .camelDescription = "inletExhaustConditions"},

    {"Vehicle Electrical Power",
     65271,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 32, .resolution = 1.0},
      {.name = "Battery Voltage", .camelName = "batteryVoltage", .fieldType = "VOLTAGE_UFIX16_50MV"},
      {.name = "Reserved", .camelName = "reserved3", .fieldType = "RESERVED", .size = 16, .resolution = 1.0}
     },
     .camelDescription = "vehicleElectricalPower"},

    {"Transmission Fluids 1",
     65272,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Transmission Clutch 1 Pressure", .camelName = "transmissionClutch1Pressure", .fieldType = "NUMBER", .size = 8, .resolution = 16.0, .unit = "kPa", .description = "SPN 123, 16 kPa per bit"},
      {.name = "Transmission Oil Level 1", .camelName = "transmissionOilLevel1", .fieldType = "PERCENTAGE_UINT8", .resolution = 0.4, .description = "SPN 124"},
      {.name = "Transmission Filter Differential Pressure", .camelName = "transmissionFilterDifferentialPressure", .fieldType = "PRESSURE_UINT8_2KPA", .description = "SPN 126"},
      {.name = "Transmission 1 Oil Pressure", .camelName = "transmission1OilPressure", .fieldType = "NUMBER", .size = 8, .resolution = 16.0, .unit = "kPa", .description = "SPN 127, 16 kPa per bit"},
      {.name = "Transmission 1 Oil Temperature 1", .camelName = "transmission1OilTemperature1", .fieldType = "TEMPERATURE_UFIX16_J1939", .description = "SPN 177"},
      {.name = "Transmission Oil Level 1 High / Low", .camelName = "transmissionOilLevel1HighLow", .fieldType = "NUMBER", .size = 8, .resolution = 0.5, .unit = "L", .description = "SPN 3027, 0.5 litre per bit with a -62.5 litre offset"},
      {.name = "Transmission Oil Level 1 Countdown Timer", .camelName = "transmissionOilLevel1CountdownTimer", .fieldType = "NUMBER", .size = 4, .resolution = 1.0, .description = "SPN 3028"},
      {.name = "Transmission Oil Level 1 Measurement Status", .camelName = "transmissionOilLevel1MeasurementStatus", .fieldType = "NUMBER", .size = 4, .resolution = 1.0, .description = "SPN 3026"}
     },
     .camelDescription = "transmissionFluids1",
     .interval = 1000,
     .priority = 6},

    {"0xFF00-0xFFFF: Manufacturer Proprietary single-frame non-addressed",
     65280,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Manufacturer Code", .camelName = "manufacturerCode", .fieldType = "LOOKUP", .size = 11, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupMANUFACTURER_CODE, .lookup.name = "MANUFACTURER_CODE"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 2, .resolution = 1.0},
      {.name = "Industry Code", .camelName = "industryCode", .fieldType = "LOOKUP", .size = 3, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupINDUSTRY_CODE, .lookup.name = "INDUSTRY_CODE"},
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 48, .resolution = 1.0}
     },
     .camelDescription = "0xff000xffffManufacturerProprietarySingleFrameNonAddressed",
     .fallback = true,
     .explanation = "Manufacturer proprietary PGNs in PDU2 (non-addressed) single-frame PGN range 0xFF00 to 0xFFFF (65280 - 65535). When this is shown during analysis it means the PGN is not reverse engineered yet."},

    {"0x1ED00 - 0x1EE00: Standardized fast-packet addressed",
     126208,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN | PACKET_LOOKUPS_UNKNOWN,
     PACKET_FAST,
     {
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 1784, .resolution = 1.0}
     },
     .camelDescription = "0x1ed000x1ee00StandardizedFastPacketAddressed",
     .fallback = true,
     .explanation = "Standardized PGNs in PDU1 (addressed) fast-packet PGN range 0x1ED00 to 0x1EE00 (65536 - 126464). When this is shown during analysis it means the PGN is not reverse engineered yet."},

    {"0x1EF00-0x1EFFF: Manufacturer Proprietary fast-packet addressed",
     126720,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_FAST,
     {
      {.name = "Manufacturer Code", .camelName = "manufacturerCode", .fieldType = "LOOKUP", .size = 11, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupMANUFACTURER_CODE, .lookup.name = "MANUFACTURER_CODE"},
      {.name = "Reserved", .camelName = "reserved", .fieldType = "RESERVED", .size = 2, .resolution = 1.0},
      {.name = "Industry Code", .camelName = "industryCode", .fieldType = "LOOKUP", .size = 3, .resolution = 1.0, .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupINDUSTRY_CODE, .lookup.name = "INDUSTRY_CODE"},
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 1768, .resolution = 1.0}
     },
     .camelDescription = "0x1ef000x1efffManufacturerProprietaryFastPacketAddressed",
     .fallback = true,
     .explanation = "Manufacturer Proprietary PGNs in PDU1 (addressed) fast-packet PGN range 0x1EF00 to 0x1EFFF (126720 - 126975). When this is shown during analysis it means the PGN is not reverse engineered yet."},

    {"0x1F000-0x1FEFF: Standardized mixed single/fast packet non-addressed",
     126976,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_MIXED,
     {
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 1784, .resolution = 1.0}
     },
     .camelDescription = "0x1f0000x1feffStandardizedMixedSingleFastPacketNonAddressed",
     .fallback = true,
     .explanation = "Standardized PGNs in PDU2 (non-addressed) mixed single/fast packet PGN range 0x1F000 to 0x1FEFF (126976 - 130815). When this is shown during analysis it means the PGN is not reverse engineered yet."},

    {"0x1FF00-0x1FFFF: Manufacturer Specific fast-packet non-addressed",
     130816,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_FAST,
     {
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 1784, .resolution = 1.0}
     },
     .camelDescription = "0x1ff000x1ffffManufacturerSpecificFastPacketNonAddressed",
     .fallback = true,
     .explanation = "This definition is used for Manufacturer Specific PGNs in PDU2 (non-addressed) fast-packet PGN range 0x1FF00 to 0x1FFFF (130816 - 131071). When this is shown during analysis it means the PGN is not reverse engineered yet."}
};
