/*

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

#include <signal.h>
#include <sys/select.h>

#include "common.h"
#include "nmea0183.h"

extern uint16_t port;
extern char *   srcFilter;
extern bool     rateLimit;
extern uint32_t protocol;
extern int      debug;
extern bool     unitSI;

extern int64_t epoch(void);

typedef enum
{
  U_ANGLE,
  U_ANGULAR_VELOCITY,
  U_VELOCITY,
  U_DISTANCE,
  U_TEMPERATURE,
  U_GEO,
  U_MAX
} Unit;

bool getJSONNumber(const char *message, const char *fieldName, double *value, Unit unit);
bool getJSONInteger(const char *message, const char *fieldName, int *value);
