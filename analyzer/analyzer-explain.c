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
bool       showSI        = true; // Output everything in strict SI units
GeoFormats showGeo       = GEO_DD;

bool  doV1 = false;
char *sep  = " ";
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
  printf("     -v1               v1 format: Explain lookups everywhere they are used\n");
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
    else if (strcasecmp(av[1], "-v1") == 0)
    {
      doV1 = true;
    }
    else if (strcasecmp(av[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
      logDebug("Logging at debug level\n");
    }
    else
    {
      usage(argv, av + 1);
    }
  }

  fillLookups();
  fillFieldType(false);

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

/**
 * Count the number of bits in all fields up to the first
 * repeating field.
 * Variable length fields are counted as 0.
 */
static unsigned int getMinimalPgnLength(Pgn *pgn, bool *isVariable)
{
  // Count all fields until the first repeating field
  uint32_t     fieldCount = pgn->fieldCount;
  unsigned int length     = 0;

  *isVariable = false;
  if (pgn->repeatingCount1 > 0)
  {
    fieldCount -= pgn->repeatingCount1 + pgn->repeatingCount2;
    *isVariable = true;
  }

  logDebug("PGN %u fieldCount=%u (was %u)\n", pgn->pgn, fieldCount, pgn->fieldCount);

  for (uint32_t i = 0; i < fieldCount; i++)
  {
    Field *f = &pgn->fieldList[i];

    if (f->size == LEN_VARIABLE)
    {
      *isVariable = true;
    }
    else
    {
      length += f->size;
    }
  }

  length /= 8; // Bits to bytes
  logDebug("PGN %u len=%u\n", pgn->pgn, length);
  return length;
}

static void explainPGN(Pgn pgn)
{
  int  i;
  int  len;
  bool isVariable;

  printf("PGN: %d / %08o / %05X - %s\n\n", pgn.pgn, pgn.pgn, pgn.pgn, pgn.description);

  if (pgn.explanation != NULL)
  {
    printf("     %s\n", pgn.explanation);
  }
  if (pgn.url != NULL)
  {
    printf("     URL: %s\n", pgn.url);
  }
  len = getMinimalPgnLength(&pgn, &isVariable);
  if (isVariable)
  {
    printf("     The length is variable but at least %d bytes\n", len);
  }
  else
  {
    printf("     The length is %d bytes\n", len);
  }

  if (pgn.repeatingCount1 > 0)
  {
    if (pgn.repeatingField1 < 255)
    {
      printf("     Fields %u thru %u repeat n times, where n is the value contained in field %u.\n\n",
             pgn.repeatingStart1,
             pgn.repeatingStart1 + pgn.repeatingCount1,
             pgn.repeatingField1);
    }
    else
    {
      printf("     Fields %u thru %u repeat until the data in the PGN is exhausted.\n\n",
             pgn.repeatingStart1,
             pgn.repeatingStart1 + pgn.repeatingCount1);
    }
  }

  if (pgn.repeatingCount2 > 0)
  {
    if (pgn.repeatingField2 < 255)
    {
      printf("     Fields %u thru %u repeat n times, where n is the value contained in field %u.\n\n",
             pgn.repeatingStart2,
             pgn.repeatingStart2 + pgn.repeatingCount2,
             pgn.repeatingField2);
    }
    else
    {
      printf("     Fields %u thru %u repeat until the data in the PGN is exhausted.\n\n",
             pgn.repeatingStart2,
             pgn.repeatingStart2 + pgn.repeatingCount2);
    }
  }

  if (pgn.interval != 0 && pgn.interval < UINT16_MAX)
  {
    printf("     The PGN is normally transmitted every %u ms\n", pgn.interval);
  }
  if (pgn.interval == UINT16_MAX)
  {
    printf("     The PGN is transmitted on-demand or when data is available\n");
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

    if (f.unit && f.unit[0] == '=')
    {
      printf("                  Match: %s\n", &f.unit[1]);
    }
    else if (f.unit && f.unit[0] != ',')
    {
      printf("                  Unit: %s\n", f.unit);
    }

    if (f.resolution != 0.0)
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

    if (!(f.unit && f.unit[0] == '=') && f.lookupValue != NULL && strcmp(f.fieldType, "LOOKUP") == 0)
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

    if (f.lookupValue != NULL && strcmp(f.fieldType, "BITLOOKUP") == 0)
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

static void printXMLUnsigned(int indent, const char *element, const unsigned int p)
{
  int i;

  for (i = 0; i < indent; i++)
  {
    fputs(" ", stdout);
  }
  printf("<%s>%u</%s>\n", element, p, element);
}

static const char *getV1Type(Field *f)
{
  for (FieldType *ft = f->ft; ft != NULL; ft = ft->baseFieldTypePtr)
  {
    if (ft->v1Type != NULL)
    {
      if (strcmp(ft->v1Type, "Lat/Lon") == 0)
      {
        if (strstr(f->name, "ongitude") != NULL)
        {
          return "Longitude";
        }
        return "Latitude";
      }
      return ft->v1Type;
    }
  }
  return NULL;
}

static const char *getV2Type(Field *f)
{
  for (FieldType *ft = f->ft; ft != NULL; ft = ft->baseFieldTypePtr)
  {
    if (ft->baseFieldTypePtr == NULL)
    {
      return ft->name;
    }
  }
  return NULL;
}

static void explainPGNXML(Pgn pgn)
{
  int      i;
  unsigned bitOffset     = 0;
  bool     showBitOffset = true;
  int      len;
  bool     isVariable;

  if (pgn.fallback && doV1)
  {
    return;
  }

  printf("    <PGNInfo>\n"
         "      <PGN>%u</PGN>\n",
         pgn.pgn);
  printXML(6, "Id", pgn.camelDescription);
  printXML(6, "Description", pgn.description);
  if (!doV1)
  {
    printXML(6, "Explanation", pgn.explanation);
    printXML(6, "URL", pgn.url);
  }
  printXML(6, "Type", (pgn.type == PACKET_ISO11783 ? "ISO" : (pgn.type == PACKET_FAST ? "Fast" : "Single")));
  printXML(6, "Complete", (pgn.complete == PACKET_COMPLETE ? "true" : "false"));
  if (pgn.fallback)
  {
    printXML(6, "Fallback", "true");
  }

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
    if ((pgn.complete & PACKET_INTERVAL_UNKNOWN) != 0)
    {
      printXML(8, "MissingAttribute", "Interval");
    }

    printf("      </Missing>\n");
  }

  len = getMinimalPgnLength(&pgn, &isVariable);
  if (!doV1)
  {
    printXMLUnsigned(6, "FieldCount", pgn.fieldCount);
    if (isVariable)
    {
      printXMLUnsigned(6, "MinLength", len);
    }
    else
    {
      printXMLUnsigned(6, "Length", len);
    }
  }
  else
  {
    printXMLUnsigned(6, "Length", len);
  }

  if (pgn.repeatingCount1 > 0)
  {
    printXMLUnsigned(6, "RepeatingFieldSet1Size", pgn.repeatingCount1);
    printXMLUnsigned(6, "RepeatingFieldSet1StartField", pgn.repeatingStart1);
    if (pgn.repeatingField1 < 255)
    {
      printXMLUnsigned(6, "RepeatingFieldSet1CountField", pgn.repeatingField1);
    }
  }

  if (pgn.repeatingCount2 > 0)
  {
    printXMLUnsigned(6, "RepeatingFieldSet2Size", pgn.repeatingCount2);
    printXMLUnsigned(6, "RepeatingFieldSet2StartField", pgn.repeatingStart2);
    if (pgn.repeatingField2 < 255)
    {
      printXMLUnsigned(6, "RepeatingFieldSet2CountField", pgn.repeatingField2);
    }
  }

  if (!doV1)
  {
    if (pgn.interval != 0 && pgn.interval < UINT16_MAX)
    {
      printXMLUnsigned(6, "TransmissionInterval", pgn.interval);
    }
    if (pgn.interval == UINT16_MAX)
    {
      printXML(6, "TransmissionIrregular", "true");
    }
  }

  if (pgn.fieldList[0].name)
  {
    printf("      <Fields>\n");

    for (i = 0; i < ARRAY_SIZE(pgn.fieldList) && pgn.fieldList[i].name; i++)
    {
      Field      f  = pgn.fieldList[i];
      FieldType *ft = f.ft;

      printf("        <Field>\n"
             "          <Order>%d</Order>\n",
             f.order);
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
        printXMLUnsigned(10, "BitLength", f.size);
      }
      if (showBitOffset)
      {
        printXMLUnsigned(10, "BitOffset", bitOffset);
        printXMLUnsigned(10, "BitStart", bitOffset % 8);
      }
      bitOffset = bitOffset + f.size;

      if (f.proprietary)
      {
        printXML(10, "Condition", "PGNIsProprietary");
      }
      if (f.unit && f.unit[0] == '=')
      {
        printXML(10, "Match", &f.unit[1]);
      }
      else
      {
        if (doV1)
        {
          printXML(10, "Units", f.unit);
        }
        else
        {
          printXML(10, "Unit", f.unit);
        }
      }

      if (doV1)
      {
        const char *s = getV1Type(&f);

        if (s != NULL)
        {
          printXML(10, "Type", s);
        }
      }

      if (f.resolution != 0.0)
      {
        printf("          <Resolution>%g</Resolution>\n", f.resolution);
      }

      if (doV1)
      {
        printXML(10, "Signed", f.hasSign ? "true" : "false");
      }
      else if (ft->hasSign != Null)
      {
        printf("          <Signed>%s</Signed>\n", ft->hasSign == True ? "true" : "false");
      }

      if (f.offset != 0)
      {
        printf("          <Offset>%d</Offset>\n", f.offset);
      }

      if (!isnan(f.rangeMin))
      {
        printf("          <RangeMin>%.16g</RangeMin>\n", f.rangeMin);
      }
      else if (!doV1 && f.lookupValue != 0)
      {
        if (!(f.unit && f.unit[0] == '='))
        {
          printf("          <RangeMin>%.16g</RangeMin>\n", 0.0);
        }
      }

      if (!isnan(f.rangeMax))
      {
        printf("          <RangeMax>%.16g</RangeMax>\n", f.rangeMax);
      }
      else if (!doV1 && f.lookupValue != 0 && !(f.unit && f.unit[0] == '='))
      {
        printf("          <RangeMax>%.16g</RangeMax>\n", (double) ((1 << f.size) - 1));
      }

      if (!doV1)
      {
        printXML(10, "FieldType", getV2Type(&f));
        if (ft->physical != NULL)
        {
          printXML(10, "PhysicalQuantity", ft->physical->name);
        }
      }

      if (f.lookupValue != 0)
      {
        if (strcmp(f.fieldType, "BITLOOKUP") == 0)
        {
          if (doV1)
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
            printXML(10, "LookupBitEnumeration", f.lookupName);
          }
        }
        else
        {
          if (doV1 && !(f.unit && f.unit[0] == '='))
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
          else if (!doV1)
          {
            printXML(10, "LookupEnumeration", f.lookupName);
          }
        }
      }

      if ((ft != NULL && ft->variableSize) || f.proprietary)
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
  printf("  <MissingEnumerations>\n");
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
  printf("    <MissingAttribute Name=\"%s\">%s</MissingAttribute>\n", "Interval", "The transmission interval is not known");

  printf("  </MissingEnumerations>\n");
}

static void explainPhysicalQuantityXML(void)
{
  printf("  <PhysicalQuantities>\n");
  for (size_t i = 0; i < ARRAY_SIZE(PhysicalQuantityList); i++)
  {
    const PhysicalQuantity *pq = PhysicalQuantityList[i];

    printf("    <PhysicalQuantity Name=\"%s\">\n", pq->name);
    if (pq->description != NULL)
    {
      printf("      <Description>%s</Description>\n", pq->description);
    }
    if (pq->comment != NULL)
    {
      printf("      <Comment>%s</Comment>\n", pq->comment);
    }
    if (pq->url != NULL)
    {
      printf("      <URL>%s</URL>\n", pq->url);
    }
    if (pq->unit != NULL)
    {
      printf("      <UnitDescription>%s</UnitDescription>\n", pq->unit);
    }
    if (pq->abbreviation != NULL)
    {
      printf("      <Unit>%s</Unit>\n", pq->abbreviation);
    }
    printf("    </PhysicalQuantity>\n");
  }
  printf("  </PhysicalQuantities>\n");
}

static void explainFieldTypesXML(void)
{
  printf("  <FieldTypes>\n");
  for (size_t i = 0; i < fieldTypeCount; i++)
  {
    FieldType *ft = &fieldTypeList[i];

    if (ft->baseFieldType != NULL)
    {
      continue;
    }

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
    if (ft->url != NULL)
    {
      printf("      <URL>%s</URL>\n", ft->url);
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

    if (!isnan(ft->rangeMin))
    {
      printf("      <RangeMin>%.16g</RangeMin>\n", ft->rangeMin);
    }
    if (!isnan(ft->rangeMax))
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
         "<!--\n" COPYRIGHT "\n-->\n");
  if (!doV1)
  {
    printf("<?xml-stylesheet type=\"text/xsl\" href=\"canboat.xsl\"?>\n");
  }
  printf("<PGNDefinitions xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\" "
         "Version=\"0.1\">\n"
         "  <Comment>See https://github.com/canboat/canboat for the full source code</Comment>\n"
         "  <CreatorCode>Canboat NMEA2000 Analyzer</CreatorCode>\n"
         "  <License>Apache License Version 2.0</License>\n"
         "  <Version>" VERSION "</Version>\n");
  if (!doV1)
  {
    printf("  <Copyright>" COPYRIGHT "\n</Copyright>\n");
  }

  if (normal && !doV1)
  {
    explainPhysicalQuantityXML();
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
  for (i = 0; i < ARRAY_SIZE(pgnList); i++)
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

extern bool fieldPrintVariable(Field *field, char *fieldName, uint8_t *data, size_t dataLen, size_t startBit, size_t *bits)
{
  return false;
}
