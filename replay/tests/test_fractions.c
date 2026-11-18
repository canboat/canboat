/* (C) 2009-2026 Kees Verruijt. Licensed under Apache-2.0. */
static unsigned long waits[8];
static int           waitCount;
#define main replay_program_main
#define select capture_select
#include "../replay.c"
#undef main
#undef select

int capture_select(int n, fd_set *readers, fd_set *writers, fd_set *errors, struct timeval *timeout)
{
  (void) n;
  (void) readers;
  (void) writers;
  (void) errors;
  if (waitCount < (int) (sizeof(waits) / sizeof(waits[0])))
  {
    waits[waitCount] = timeout->tv_sec * 1000000UL + timeout->tv_usec;
  }
  waitCount++;
  return 0;
}

int main(int argc, char **argv)
{
  /* Each fraction precision is 100 ms apart, and a whole second without a fraction
   * must not read the next CSV field (3, 6) as milliseconds. */
  static const unsigned long expected[] = {100000UL, 100000UL, 100000UL, 1000000UL};
  FILE                      *input      = tmpfile();
  int                        i;

  (void) argc;
  if (input == NULL)
  {
    perror("tmpfile");
    return 1;
  }
  fputs("2026-11-15T00:00:00.1,3,126993,0,255,1,00\n"
        "2026-11-15T00:00:00.2,3,126993,0,255,1,00\n"
        "2026-11-15T00:00:00.100,3,126993,0,255,1,00\n"
        "2026-11-15T00:00:00.200,3,126993,0,255,1,00\n"
        "2026-11-15T00:00:00.100000,3,126993,0,255,1,00\n"
        "2026-11-15T00:00:00.200000,3,126993,0,255,1,00\n"
        "2026-11-15T01:00:00,3,126993,0,255,1,00\n"
        "2026-11-15T01:00:01,6,126993,0,255,1,00\n",
        input);
  rewind(input);
  if (dup2(fileno(input), STDIN_FILENO) < 0)
  {
    perror("dup2");
    return 1;
  }
  if (replay_program_main(1, argv) != 0)
  {
    fprintf(stderr, "replay failed\n");
    return 1;
  }
  fclose(input);
  if (waitCount != (int) (sizeof(expected) / sizeof(expected[0])))
  {
    fprintf(stderr, "replay waited %d times, expected %d\n", waitCount, (int) (sizeof(expected) / sizeof(expected[0])));
    return 1;
  }
  for (i = 0; i < waitCount; i++)
  {
    if (waits[i] != expected[i])
    {
      fprintf(stderr, "wait %d was %lu us, expected %lu us\n", i, waits[i], expected[i]);
      return 1;
    }
  }
  return 0;
}
