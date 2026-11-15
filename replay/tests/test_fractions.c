/* (C) 2009-2026 Kees Verruijt. Licensed under Apache-2.0. */
static unsigned long waited;
#define main replay_program_main
#define select capture_select
#include "../replay.c"
#undef main
#undef select
#include <assert.h>

int capture_select(int n, fd_set *readers, fd_set *writers, fd_set *errors, struct timeval *timeout)
{
  (void) n;
  (void) readers;
  (void) writers;
  (void) errors;
  waited += timeout->tv_sec * 1000000UL + timeout->tv_usec;
  return 0;
}

int main(int argc, char **argv)
{
  FILE *input = tmpfile();
  (void) argc;
  assert(input != NULL);
  fputs("2026-11-15T00:00:00.1,3,126993,0,255,1,00\n"
        "2026-11-15T00:00:00.2,3,126993,0,255,1,00\n"
        "2026-11-15T00:00:00.100,3,126993,0,255,1,00\n"
        "2026-11-15T00:00:00.200,3,126993,0,255,1,00\n"
        "2026-11-15T00:00:00.100000,3,126993,0,255,1,00\n"
        "2026-11-15T00:00:00.200000,3,126993,0,255,1,00\n",
        input);
  rewind(input);
  assert(dup2(fileno(input), STDIN_FILENO) >= 0);
  assert(replay_program_main(1, argv) == 0);
  fclose(input);
  assert(waited == 300000UL);
  return 0;
}
