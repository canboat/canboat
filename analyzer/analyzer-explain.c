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

/* There are max five reserved values according to ISO 11873-9 (that I gather from indirect sources)
 * but I don't yet know which datafields reserve the reserved values.
 */
#define DATAFIELD_UNKNOWN (0)
#define DATAFIELD_ERROR (-1)
#define DATAFIELD_RESERVED1 (-2)
#define DATAFIELD_RESERVED2 (-3)
#define DATAFIELD_RESERVED3 (-4)

bool       showRaw       = false;
bool       showData      = false;
bool       showJson      = false;
bool       showJsonEmpty = false;
bool       showJsonValue = false;
bool       showSI        = false; // Output everything in strict SI units
GeoFormats showGeo       = GEO_DD;

bool  doExpandLookups = false;
char *sep             = " ";
char  closingBraces[16]; // } and ] chars to close sentence in JSON mode, otherwise empty string

int    onlyPgn  = 0;
int    onlySrc  = -1;
int    clockSrc = -1;
size_t heapSize = 0;

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
  printf("     -expand-lookups   Explain lookups everywhere they are used (historic format)\n");
  printf("     -camel            Show fieldnames in normalCamelCase\n");
  printf("     -upper-camel      Show fieldnames in UpperCamelCase\n");
  printf("     -version          Print the version of the program and quit\n");
  printf("     -d                Print logging from level ERROR, INFO and DEBUG\n");
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
    else if (strcasecmp(av[1], "-expand-lookups") == 0)
    {
      doExpandLookups = true;
    }
    else if (strcasecmp(av[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
      logDebug("Logging at debug level\n");
    }
    else
    {
      usage(argv, av);
    }
  }

  fillLookups();
  fillFieldType();

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
      printf("                  Unit: %s\n", f.units);
    }

    if (f.resolution != 1.0 && f.resolution != 0.0)
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
      if (strstr(f.fieldType, "BIT") == NULL)
      {
        printf("                  Enumeration: %s\n", f.lookupName);
      }
      else
      {
        printf("                  BitEnumeration: %s\n", f.lookupName);
      }
    }

    if (!(f.units && f.units[0] == '=') && f.lookupValue != NULL && strcmp(f.fieldType, "LOOKUP") == 0)
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

    if (f.lookupValue && strcmp(f.fieldType, "BITLOOKUP") == 0)
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
  printXML(6, "ID", pgn.camelDescription);
  printXML(6, "Description", pgn.description);
  printXML(6, "Explanation", pgn.explanation);
  printXML(6, "Type", (pgn.type == PACKET_ISO11783 ? "ISO" : (pgn.type == PACKET_FAST ? "Fast" : "Single")));
  printXML(6, "Complete", (pgn.complete == PACKET_COMPLETE ? "true" : "false"));

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
    if ((pgn.complete & PACKET_RESOLUTION_UNKNOWN) != 0)
    {
      printXML(8, "MissingAttribute", "Resolution");
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

      if (f.size == LEN_VARIABLE)
      {
        showBitOffset = false;
      }

      if (f.description && f.description[0] && f.description[0] != ',')
      {
        printXML(10, "Description", f.description);
      }
      if (f.size == LEN_VARIABLE)
      {
        printf("          <BitLengthVariable>true</BitLengthVariable>\n");
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

      if (f.proprietary)
      {
        if (doExpandLookups)
        {
          printf("          <Match>proprietary pgn only</Match>\n");
        }
        else
        {
          printf("          <Condition>PGNIsProprietary</Condition>\n");
        }
      }
      if (f.units && f.units[0] == '=')
      {
        printf("          <Match>%s</Match>\n", &f.units[1]);
      }
      else if (f.units && f.units[0] != ',')
      {
        printf("          <Unit>%s</Unit>\n", f.units);
      }

      if (f.resolution != 1.0 && f.resolution != 0.0)
      {
        printf("          <Resolution>%g</Resolution>\n", f.resolution);
      }
      printf("          <Signed>%s</Signed>\n", f.hasSign ? "true" : "false");
      if (f.offset != 0)
      {
        printf("          <Offset>%d</Offset>\n", f.offset);
      }

      if (f.fieldType != NULL)
      {
        printf("          <FieldType>%s</FieldType>\n", f.fieldType);
      }
      else
      {
        logError("PGN %u field '%s' has no fieldtype\n", pgn.pgn, f.name);
      }

      if (!(f.units && f.units[0] == '=') && f.lookupValue && f.fieldType != NULL && strcmp(f.fieldType, "LOOKUP") == 0)
      {
        if (doExpandLookups)
        {
          uint32_t maxValue = (1 << f.size) - 1;

          printf("          <EnumValues>\n");

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
        else
        {
          printf("          <LookupEnumeration>%s</LookupEnumeration>\n", f.lookupName);
        }
      }

      if (f.lookupValue && strcmp(f.fieldType, "BITLOOKUP") == 0)
      {
        if (doExpandLookups)
        {
          uint32_t maxValue = f.size;

          printf("          <EnumBitValues>\n");

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
        else
        {
          printf("          <LookupBitEnumeration>%s</LookupBitEnumeration>\n", f.lookupName);
        }
      }

      if ((f.ft != NULL && f.ft->variableSize) || f.proprietary)
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

static void explainMissingXML(void)
{
  printf("  <Missing>\n");
  printf("    <MissingAttribute Name=\"%s\">%s</MissingAttribute>\n",
         "Fields",
         "The list of fields is incomplete; some fields maybe be missing or their attributes may be incorrect");
  printf("    <MissingAttribute Name=\"%s\">%s</MissingAttribute>\n",
         "FieldLengths",
         "The length of one or more fields is likely incorrect");
  printf("    <MissingAttribute Name=\"%s\">%s</MissingAttribute>\n",
         "Resolution",
         "The resolution of one or more fields is likely incorrect");
  printf("    <MissingAttribute Name=\"%s\">%s</MissingAttribute>\n",
         "Lookups",
         "One or more of the lookup fields contain missing or incorrect values");
  printf(
      "    <MissingAttribute Name=\"%s\">%s</MissingAttribute>\n", "SampleData", "The PGN has not been seen in any logfiles yet");

  printf("  </Missing>\n");
}

static void explainFieldTypesXML(void)
{
  printf("  <FieldTypes>\n");
  for (size_t i = 0; i < fieldTypeCount; i++)
  {
    FieldType *ft = &fieldTypeList[i];

    printf("    <FieldType Name=\"%s\">\n", ft->name);
    if (ft->description != NULL)
    {
      printf("      <Description>%s</Description>\n", ft->description);
    }
    if (ft->encodingDescription != NULL)
    {
      printf("      <EncodingDescription>%s</EncodingDescription>\n", ft->encodingDescription);
    }
    if (ft->comment != NULL)
    {
      printf("      <Comment>%s</Comment>\n", ft->comment);
    }
    if (ft->baseFieldType != NULL)
    {
      printf("      <BaseFieldType>%s</BaseFieldType>\n", ft->baseFieldType);
    }
    if (ft->size != 0)
    {
      printf("      <Bits>%u</Bits>\n", ft->size);
    }
    if (ft->offset != 0)
    {
      printf("      <Offset>%d</Offset>\n", ft->offset);
    }
    if (ft->variableSize != Null)
    {
      printf("      <VariableSize>true</VariableSize>\n");
    }
    if (ft->unit != NULL)
    {
      printf("      <Unit>%s</Unit>\n", ft->unit);
    }
    if (ft->hasSign != Null)
    {
      printf("      <Signed>%s</Signed>\n", ft->hasSign == True ? "true" : "false");
    }
    if (ft->resolution != 1.0 && ft->resolution != 0.0)
    {
      printf("      <Resolution>%.16g</Resolution>\n", ft->resolution);
    }
    if (ft->format != NULL)
    {
      printf("      <Format>%s</Format>\n", ft->format);
    }
    if (ft->rangeMinText != NULL)
    {
      printf("      <RangeMin>%s</RangeMin>\n", ft->rangeMinText);
    }
    else if (!isnan(ft->rangeMin))
    {
      printf("      <RangeMin>%.16g</RangeMin>\n", ft->rangeMin);
    }
    if (ft->rangeMaxText != NULL)
    {
      printf("      <RangeMax>%s</RangeMax>\n", ft->rangeMaxText);
    }
    else if (!isnan(ft->rangeMax))
    {
      printf("      <RangeMax>%.16g</RangeMax>\n", ft->rangeMax);
    }
    printf("    </FieldType>\n");
  }
  printf("  </FieldTypes>\n");
}

static void explainXML(bool normal, bool actisense, bool ikonvert)
{
  int i;

  printf("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
         "<?xml-stylesheet type=\"text/xsl\" href=\"canboat.xsl\"?>"
         "<PGNDefinitions xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\" "
         "Version=\"0.1\">\n"
         "  <Comment>See https://github.com/canboat/canboat for the full source code</Comment>\n"
         "  <CreatorCode>Canboat NMEA2000 Analyzer</CreatorCode>\n"
         "  <License>Apache License Version 2.0</License>\n"
         "  <Version>" VERSION "</Version>\n"
         "  <Copyright>" COPYRIGHT "\n</Copyright>\n");

  if (normal && !doExpandLookups)
  {
    explainFieldTypesXML();
    explainMissingXML();

    printf("  <LookupEnumerations>\n");
    for (i = 0; i < ARRAY_SIZE(lookupEnums); i++)
    {
      uint32_t maxValue = (1 << lookupEnums[i].size) - 1;
      printf("    <LookupEnumeration Name='%s' MaxValue='%u'>\n", lookupEnums[i].name, maxValue);
      for (int j = 0; j <= maxValue; j++)
      {
        if (lookupEnums[i].values[j])
        {
          printf("      <EnumPair Value='%u' Name='", j);
          printXML(0, 0, lookupEnums[i].values[j]);
          printf("' />\n");
        }
      }
      printf("    </LookupEnumeration>\n");
    }
    printf("  </LookupEnumerations>\n");

    printf("  <LookupBitEnumerations>\n");
    for (i = 0; i < ARRAY_SIZE(bitfieldEnums); i++)
    {
      uint32_t maxValue = bitfieldEnums[i].size - 1;
      printf("    <LookupBitEnumeration Name='%s' MaxValue='%u'>\n", bitfieldEnums[i].name, maxValue);
      for (int j = 0; j <= maxValue; j++)
      {
        if (bitfieldEnums[i].values[j])
        {
          printf("      <BitPair Bit='%u' Name='", j);
          printXML(0, 0, bitfieldEnums[i].values[j]);
          printf("' />\n");
        }
      }
      printf("    </LookupBitEnumeration>\n");
    }
    printf("  </LookupBitEnumerations>\n");
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
