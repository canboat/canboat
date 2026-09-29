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
    {"Quick: Unknown Packet",
     0,
     PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_RESOLUTION_UNKNOWN,
     PACKET_SINGLE,
     {
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 64, .resolution = 1.0, .description = "Raw payload of an unidentified Quick message type"}
     },
     .camelDescription = "quickUnknownPgn",
     .fallback = true,
     .explanation = "Quick message type 0x000-0x7FF that is not reverse engineered yet. Any unrecognised 11-bit Quick identifier resolves here, with its payload left as raw bytes, rather than being misreported as a packet we do have a definition for."},

    {"Quick: Miscellaneous Flags Packet",
     1728,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Source Address", .camelName = "sourceAddress", .fieldType = "UINT16", .resolution = 1.0, .description = "Source address of the transmitting device"},
      {.name = "Flags", .camelName = "flags", .fieldType = "BINARY", .size = 48, .resolution = 1.0, .description = "Miscellaneous flag bits (layout TBD - placeholder for 6 remaining bytes)"}
     },
     .camelDescription = "miscFlagsPacket"},

    {"Quick: Chain Count Packet",
     1729,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Source Address", .camelName = "sourceAddress", .fieldType = "UINT16", .resolution = 1.0, .description = "Talker identifier of the transmitting device"},
      {.name = "Chain Deployed", .camelName = "chainDeployed", .fieldType = "UINT32", .resolution = 1.0, .description = "Length of chain currently deployed"},
      {.name = "Units", .camelName = "units", .fieldType = "LOOKUP", .size = 16, .resolution = 1.0, .description = "Measurement units for the deployed chain length", .lookup.type = LOOKUP_TYPE_PAIR, LOOKUP_PAIR_MEMBER = lookupQUICK_UNIT, .lookup.name = "QUICK_UNIT"}
     },
     .camelDescription = "chainCountPacket"},

    {"Quick: Unknown Packet Type 1",
     1730,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Source Address", .camelName = "sourceAddress", .fieldType = "UINT16", .resolution = 1.0, .description = "Source address of the transmitting device"},
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 48, .resolution = 1.0, .description = "Unknown payload (layout TBD)"}
     },
     .camelDescription = "unknownPacket1"},

    {"Quick: Unknown Packet Type 2",
     1731,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {
      {.name = "Source Address", .camelName = "sourceAddress", .fieldType = "UINT16", .resolution = 1.0, .description = "Source address of the transmitting device"},
      {.name = "Data", .camelName = "data", .fieldType = "BINARY", .size = 48, .resolution = 1.0, .description = "Unknown payload (layout TBD)"}
     },
     .camelDescription = "unknownPacket2"}
};
