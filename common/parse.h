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

#ifndef PARSE_H_INCLUDED
#define PARSE_H_INCLUDED

#include <common.h>

typedef struct
{
  char     timestamp[DATE_LENGTH];
  uint8_t  prio;
  uint32_t pgn;
  uint8_t  dst;
  uint8_t  src;
  uint8_t  len;
  uint8_t  data[FASTPACKET_MAX_SIZE];
} RawMessage;

bool parseFastFormat(StringBuffer *src, RawMessage *msg);
bool parseInt(const char **msg, int *value, int defValue);
bool parseConst(const char **msg, const char *str);

int parseRawFormatPlain(char *msg, RawMessage *m, bool showJson);
int parseRawFormatFast(char *msg, RawMessage *m, bool showJson);
int parseRawFormatAirmar(char *msg, RawMessage *m, bool showJson);
int parseRawFormatChetco(char *msg, RawMessage *m, bool showJson);
int parseRawFormatGarminCSV(char *msg, RawMessage *m, bool showJson, bool absolute);
int parseRawFormatYDWG02(char *msg, RawMessage *m, bool showJson);
int parseRawFormatActisenseN2KAscii(char *msg, RawMessage *m, bool showJson);

#endif
