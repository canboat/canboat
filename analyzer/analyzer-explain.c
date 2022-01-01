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

#define GLOBALS
#include "analyzer.h"
#include "common.h"

enum GeoFormats
{
  GEO_DD,
  GEO_DM,
  GEO_DMS
};

/* There are max five reserved values according to ISO 11873-9 (that I gather from indirect sources)
 * but I don't yet know which datafields reserve the reserved values.
 */
#define DATAFIELD_UNKNOWN (0)
#define DATAFIELD_ERROR (-1)
#define DATAFIELD_RESERVED1 (-2)
#define DATAFIELD_RESERVED2 (-3)
#define DATAFIELD_RESERVED3 (-4)

bool            showRaw       = false;
bool            showData      = false;
bool            showBytes     = false;
bool            showJson      = false;
bool            showJsonEmpty = false;
bool            showJsonValue = false;
bool            showSI        = false; // Output everything in strict SI units
char           *sep           = " ";
char            closingBraces[8]; // } and ] chars to close sentence in JSON mode, otherwise empty string
enum GeoFormats showGeo  = GEO_DD;
int             onlyPgn  = 0;
int             onlySrc  = -1;
int             clockSrc = -1;
size_t          heapSize = 0;

int g_variableFieldRepeat[2]; // Actual number of repetitions
int g_variableFieldIndex;

typedef struct
{
  const char  *name;
  uint32_t     size;
  const char **values;
} Enumeration;

Enumeration lookupEnums[] = {
#define LOOKUP_TYPE(type, length) {.name = xstr(type), .size = length, .values = lookupValue##type},
#include "lookup.h"
};

Enumeration bitfieldEnums[] = {
#define LOOKUP_TYPE_BITFIELD(type, length) {.name = xstr(type), .size = length, .values = lookupValue##type},
#include "lookup.h"
};

static void explain(void);
static void explainXML(bool, bool, bool);

static void usage(char **argv, char **av)
{
  if (av != NULL)
  {
    printf("Unknown or invalid argument %s\n", av[0]);
  }
  printf("Usage: %s -explain | -explain-xml | -explain-ngt-xml | -explain-ik-xml"
         " | [-camel] | [-upper-camel]"
         " | [-version]\n",
         argv[0]);
  printf("     -explain          Export the PGN database in text format\n");
  printf("     -explain-xml      Export the PGN database in XML format\n");
  printf("     -explain-ngt-xml  Export the Actisense PGN database in XML format\n");
  printf("     -explain-ik-xml   Export the iKonvert PGN database in XML format\n");
  printf("     -camel            Show fieldnames in normalCamelCase\n");
  printf("     -upper-camel      Show fieldnames in UpperCamelCase\n");
  printf("     -version          Print the version of the program and quit\n");
  printf("\n");
  exit(1);
}

int main(int argc, char **argv)
{
  int    ac           = argc;
  char **av           = argv;
  bool   doExplain    = false;
  bool   doExplainXML = false;
  bool   doExplainNGT = false;
  bool   doExplainIK  = false;

  setProgName(argv[0]);

  camelCase(false);
  fillLookups();

  for (; ac > 1; ac--, av++)
  {
    if (strcasecmp(av[1], "-version") == 0)
    {
      printf("%s\n", VERSION);
      exit(0);
    }
    else if (strcasecmp(av[1], "-camel") == 0)
    {
      camelCase(false);
    }
    else if (strcasecmp(av[1], "-upper-camel") == 0)
    {
      camelCase(true);
    }
    else if (strcasecmp(av[1], "-explain-xml") == 0)
    {
      doExplainXML = true;
    }
    else if (strcasecmp(av[1], "-explain-ngt-xml") == 0)
    {
      doExplainNGT = true;
    }
    else if (strcasecmp(av[1], "-explain-ik-xml") == 0)
    {
      doExplainIK = true;
    }
    else if (strcasecmp(av[1], "-explain") == 0)
    {
      doExplain = true;
    }
    else
    {
      usage(argv, av);
    }
  }

  if (doExplain)
  {
    explain();
    exit(0);
  }
  if (doExplainXML || doExplainNGT || doExplainIK)
  {
    explainXML(doExplainXML, doExplainNGT, doExplainIK);
    exit(0);
  }
  usage(argv, NULL);
  exit(1);
}

static void explainPGN(Pgn pgn)
{
  int i;

  printf("PGN: %d / %08o / %05X - %u - %s\n\n", pgn.pgn, pgn.pgn, pgn.pgn, pgn.size, pgn.description);

  if (pgn.repeatingFields >= 100)
  {
    printf("     The last %u and %u fields repeat until the data is exhausted.\n\n",
           pgn.repeatingFields % 100,
           pgn.repeatingFields / 100);
  }
  else if (pgn.repeatingFields)
  {
    printf("     The last %u fields repeat until the data is exhausted.\n\n", pgn.repeatingFields);
  }
  for (i = 0; i < ARRAY_SIZE(pgn.fieldList) && pgn.fieldList[i].name; i++)
  {
    Field f = pgn.fieldList[i];
    printf("  Field #%d: %s%s%s\n",
           i + 1,
           f.name,
           f.name[0] && (f.description && f.description[0] && f.description[0] != ',') ? " - " : "",
           (!f.description || f.description[0] == ',') ? "" : f.description);
    if (f.size == LEN_VARIABLE)
    {
      printf("                  Bits: variable\n");
    }
    else
    {
      printf("                  Bits: %u\n", f.size);
    }

    if (f.units && f.units[0] == '=')
    {
      printf("                  Match: %s\n", &f.units[1]);
    }
    else if (f.units && f.units[0] != ',')
    {
      printf("                  Units: %s\n", f.units);
    }

    if (f.resolution < 0.0)
    {
      Resolution t = types[-1 * (int) f.resolution - 1];
      if (t.name)
      {
        printf("                  Type: %s\n", t.name);
      }
      if (t.resolution)
      {
        printf("                  Resolution: %s\n", t.resolution);
      }
      else if (f.resolution == RES_LATITUDE || f.resolution == RES_LONGITUDE)
      {
        if (f.size == BYTES(8))
        {
          printf("                  Resolution: %.16f\n", 1e-16);
        }
        else
        {
          printf("                  Resolution: %.7f\n", 1e-7);
        }
      }
    }
    else if (f.resolution != 1.0)
    {
      printf("                  Resolution: %g\n", f.resolution);
    }
    printf("                  Signed: %s\n", (f.hasSign) ? "true" : "false");
    if (f.offset != 0)
    {
      printf("                  Offset: %d\n", f.offset);
    }

    if (f.lookupName)
    {
      if (f.resolution == RES_LOOKUP)
      {
        printf("                  Enumeration: %s\n", f.lookupName);
      }
      if (f.resolution == RES_BITFIELD)
      {
        printf("                  BitEnumeration: %s\n", f.lookupName);
      }
    }

    if (f.resolution == RES_LOOKUP && f.lookupValue)
    {
      uint32_t maxValue = (1 << f.size) - 1;
      printf("                  Range: 0..%u\n", maxValue);
      for (uint32_t i = 0; i <= maxValue; i++)
      {
        const char *s = f.lookupValue[i];

        if (s)
        {
          printf("                  Lookup: %u=%s\n", i, s);
        }
      }
    }

    if (f.resolution == RES_BITFIELD && f.lookupValue)
    {
      uint32_t maxValue = f.size;

      printf("           BitRange: 0..%u\n", maxValue);
      for (uint32_t i = 0; i < maxValue; i++)
      {
        const char *s = f.lookupValue[i];

        if (s)
        {
          printf("                  Bit: %u=%s\n", i, s);
        }
      }
    }
  }

  printf("\n\n");
}

/*
 * Print string but replace special characters by their XML entity.
 */
static void printXML(int indent, const char *element, const char *p)
{
  int i;

  if (p)
  {
    if (element)
    {
      for (i = 0; i < indent; i++)
      {
        fputs(" ", stdout);
      }
      printf("<%s>", element);
    }
    for (; *p; p++)
    {
      switch (*p)
      {
        case '&':
          fputs("&amp;", stdout);
          break;

        case '<':
          fputs("&lt;", stdout);
          break;

        case '>':
          fputs("&gt;", stdout);
          break;

        case '"':
          fputs("&quot;", stdout);
          break;

        default:
          putchar(*p);
      }
    }
    if (element)
    {
      printf("</%s>\n", element);
    }
  }
}

static void explainPGNXML(Pgn pgn)
{
  int      i;
  unsigned bitOffset     = 0;
  bool     showBitOffset = true;

  printf("    <PGNInfo>\n"
         "      <PGN>%u</PGN>\n",
         pgn.pgn);
  printXML(6, "Id", pgn.camelDescription);
  printXML(6, "Description", pgn.description);
  printXML(6, "Type", (pgn.type == PACKET_ISO11783 ? "ISO" : (pgn.type == PACKET_FAST ? "Fast" : "Single")));
  printf("      <Complete>%s</Complete>\n", (pgn.complete == PACKET_COMPLETE ? "true" : "false"));

  if (pgn.complete != PACKET_COMPLETE)
  {
    printf("      <Missing>\n");

    if ((pgn.complete & PACKET_FIELDS_UNKNOWN) != 0)
    {
      printXML(8, "MissingAttribute", "Fields");
    }
    if ((pgn.complete & PACKET_FIELD_LENGTHS_UNKNOWN) != 0)
    {
      printXML(8, "MissingAttribute", "FieldLengths");
    }
    if ((pgn.complete & PACKET_PRECISION_UNKNOWN) != 0)
    {
      printXML(8, "MissingAttribute", "Precision");
    }
    if ((pgn.complete & PACKET_LOOKUPS_UNKNOWN) != 0)
    {
      printXML(8, "MissingAttribute", "Lookups");
    }
    if ((pgn.complete & PACKET_NOT_SEEN) != 0)
    {
      printXML(8, "MissingAttribute", "SampleData");
    }

    printf("      </Missing>\n");
  }

  printf("      <Length>%u</Length>\n", pgn.size);

  if (pgn.repeatingFields >= 100)
  {
    printf("      <RepeatingFieldSet1>%u</RepeatingFieldSet1>\n", pgn.repeatingFields % 100);
    printf("      <RepeatingFieldSet2>%u</RepeatingFieldSet2>\n", pgn.repeatingFields / 100);
  }
  else
  {
    printf("      <RepeatingFields>%u</RepeatingFields>\n", pgn.repeatingFields);
  }

  if (pgn.fieldList[0].name)
  {
    printf("      <Fields>\n");

    for (i = 0; i < ARRAY_SIZE(pgn.fieldList) && pgn.fieldList[i].name; i++)
    {
      Field f = pgn.fieldList[i];

      printf("        <Field>\n"
             "          <Order>%d</Order>\n",
             i + 1);
      printXML(10, "Id", f.camelName);
      printXML(10, "Name", f.name);

      if (f.size == LEN_VARIABLE || (f.units && strcmp(f.units, PROPRIETARY_PGN_ONLY) == 0))
      {
        showBitOffset = false;
      }

      if (f.description && f.description[0] && f.description[0] != ',')
      {
        printXML(10, "Description", f.description);
      }
      if (f.size == LEN_VARIABLE)
      {
        printf("          <BitLengthVariable/>\n");
      }
      else
      {
        printf("          <BitLength>%u</BitLength>\n", f.size);
      }
      if (showBitOffset)
      {
        printf("          <BitOffset>%u</BitOffset>\n", bitOffset);
        printf("          <BitStart>%u</BitStart>\n", bitOffset % 8);
      }
      bitOffset = bitOffset + f.size;

      if (f.units && f.units[0] == '=')
      {
        printf("          <Match>%s</Match>\n", &f.units[1]);
      }
      else if (f.units && f.units[0] != ',')
      {
        printf("          <Units>%s</Units>\n", f.units);
      }

      if (f.resolution < 0.0)
      {
        Resolution t = types[-1 * (int) f.resolution - 1];
        if (t.name)
        {
          printf("          <Type>%s</Type>\n", t.name);
        }
        if (t.resolution)
        {
          printf("          <Resolution>%s</Resolution>\n", t.resolution);
        }
        else if (f.resolution == RES_LATITUDE || f.resolution == RES_LONGITUDE)
        {
          if (f.size == BYTES(8))
          {
            printf("          <Resolution>%.16f</Resolution>\n", 1e-16);
          }
          else
          {
            printf("          <Resolution>%.7f</Resolution>\n", 1e-7);
          }
        }
      }
      else if (f.resolution != 1.0)
      {
        printf("          <Resolution>%g</Resolution>\n", f.resolution);
      }
      printf("          <Signed>%s</Signed>\n", f.hasSign ? "true" : "false");
      if (f.offset != 0)
      {
        printf("          <Offset>%d</Offset>\n", f.offset);
      }

      if (f.resolution == RES_LOOKUP && f.lookupValue)
      {
        uint32_t maxValue = (1 << f.size) - 1;

        printf("          <EnumValues Name='%s'>\n", f.lookupName);

        for (uint32_t i = 0; i <= maxValue; i++)
        {
          const char *s = f.lookupValue[i];

          if (s)
          {
            printf("            <EnumPair Value='%u' Name='", i);
            printXML(0, 0, s);
            printf("' />\n");
          }
        }
        printf("          </EnumValues>\n");
      }

      if (f.resolution == RES_BITFIELD && f.lookupValue)
      {
        uint32_t maxValue = f.size;

        printf("          <EnumBitValues Name='%s'>\n", f.lookupName);

        for (uint32_t i = 0; i < maxValue; i++)
        {
          const char *s = f.lookupValue[i];

          if (s)
          {
            printf("            <EnumPair Bit='%u' Name='", i);
            printXML(0, 0, s);
            printf("' />\n");
          }
        }

        printf("          </EnumBitValues>\n");
      }

      if (f.resolution == RES_STRINGLZ || f.resolution == RES_STRINGLAU || f.resolution == RES_VARIABLE)
      {
        showBitOffset = false; // From here on there is no good bitoffset to be printed
      }
      printf("        </Field>\n");
    }
    printf("      </Fields>\n");
  }
  printf("    </PGNInfo>\n");
}

static void explain(void)
{
  int i;

  printf(COPYRIGHT "\n\nThis program can understand a number of N2K messages. What follows is an explanation of the messages\n"
                   "that it understands. First is a list of completely understood messages, as far as I can tell.\n"
                   "What follows is a list of messages that contain fields that have unknown content or size, or even\n"
                   "completely unknown fields. If you happen to know more, please tell me!\n\n");
  printf("_______ Complete PGNs _________\n\n");
  for (i = 1; i < ARRAY_SIZE(pgnList); i++)
  {
    if (pgnList[i].complete == PACKET_COMPLETE && pgnList[i].pgn < ACTISENSE_BEM)
    {
      explainPGN(pgnList[i]);
    }
  }
  printf("_______ Incomplete PGNs _________\n\n");
  for (i = 1; i < ARRAY_SIZE(pgnList); i++)
  {
    if (pgnList[i].complete != PACKET_COMPLETE && pgnList[i].pgn < ACTISENSE_BEM)
    {
      explainPGN(pgnList[i]);
    }
  }
}

static void explainXML(bool normal, bool actisense, bool ikonvert)
{
  int i;

  printf("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
         "<!--\n" COPYRIGHT "\n-->\n"
         "<PGNDefinitions xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\" "
         "Version=\"0.1\">\n"
         "  <Comment>See https://github.com/canboat/canboat for the full source code</Comment>\n"
         "  <CreatorCode>Canboat NMEA2000 Analyzer</CreatorCode>\n"
         "  <License>Apache License Version 2.0</License>\n"
         "  <Version>" VERSION "</Version>\n");

  if (normal)
  {
    printf("  <Enumerations>\n");

    for (i = 0; i < ARRAY_SIZE(lookupEnums); i++)
    {
      uint32_t maxValue = (1 << lookupEnums[i].size) - 1;
      printf("    <Enumeration Name='%s' MaxValue='%u'>\n", lookupEnums[i].name, maxValue);
      for (int j = 0; j <= maxValue; j++)
      {
        if (lookupEnums[i].values[j])
        {
          printf("      <EnumPair Value='%u' Name='", j);
          printXML(0, 0, lookupEnums[i].values[j]);
          printf("' />\n");
        }
      }
      printf("    </Enumeration>\n");
    }

    for (i = 0; i < ARRAY_SIZE(bitfieldEnums); i++)
    {
      uint32_t maxValue = bitfieldEnums[i].size - 1;
      printf("    <BitEnumeration Name='%s' MaxValue='%u'>\n", bitfieldEnums[i].name, maxValue);
      for (int j = 0; j <= maxValue; j++)
      {
        if (bitfieldEnums[i].values[j])
        {
          printf("      <EnumPair Bit='%u' Name='", j);
          printXML(0, 0, bitfieldEnums[i].values[j]);
          printf("' />\n");
        }
      }
      printf("    </BitEnumeration>\n");
    }

    printf("  </Enumerations>\n");
  }

  printf("  <PGNs>\n");
  for (i = 1; i < ARRAY_SIZE(pgnList); i++)
  {
    int pgn = pgnList[i].pgn;
    if ((normal && pgn < ACTISENSE_BEM) || (actisense && pgn >= ACTISENSE_BEM && pgn < IKONVERT_BEM)
        || (ikonvert && pgn >= IKONVERT_BEM))
    {
      explainPGNXML(pgnList[i]);
    }
  }

  printf("  </PGNs>\n"
         "</PGNDefinitions>\n");
}
