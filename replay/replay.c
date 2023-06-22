/*

Delay incoming 'raw' format messages by looking at the timestamp
in the first field and delaying by the time difference between this
and the previous message, unless it is not in the range 0..10s;
in that case the message is sent immediately.

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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "common.h"
#include "license.h"

#define FORMAT_DT "%Y-%m-%dT%H:%M:%S"
#define FORMAT_MS "%d"

int main(int argc, char **argv)
{
  char      line[8192];
  struct tm tm;
  char     *r;
  int       ms;
  uint64_t  now;
  uint64_t  prev = UINT64_C(0);

  setProgName(argv[0]);
  while (argc > 1)
  {
    if (strcasecmp(argv[1], "-version") == 0)
    {
      printf("%s\n", VERSION);
      exit(0);
    }
    else if (strcasecmp(argv[1], "-d") == 0)
    {
      setLogLevel(LOGLEVEL_DEBUG);
    }
    argc--;
    argv++;
  }

  while (fgets(line, sizeof(line), stdin))
  {
    memset(&tm, 0, sizeof(tm));
    ms = 0;

    r = strptime(line, FORMAT_DT, &tm);
    if (r)
    {
      sscanf(r + 1, FORMAT_MS, &ms);
    }
    now = timegm(&tm);
    logDebug("%-1.24s = %" PRIu64 " s r='%-1.10s'\n", line, now, r);
    now = now * UINT64_C(1000) + ms;
    logDebug("%-1.24s = %" PRIu64 " ms\n", line, now);

    if (now > prev && now < prev + UINT64_C(10000))
    {
      uint64_t       diff    = now - prev;
      struct timeval timeout = {diff / UINT64_C(1000), (diff % UINT64_C(1000)) * UINT64_C(1000)};

      logDebug("%-1.24s = %" PRIu64 " zzz=%" PRIu64 "\n", line, now, diff);
      select(1, NULL, NULL, NULL, &timeout);
    }
    else
    {
      logDebug("%-1.24s = %" PRIu64 "\n", line, now);
    }
    prev = now;

    fprintf(stdout, "%s", line);
  }
  return 0;
}
