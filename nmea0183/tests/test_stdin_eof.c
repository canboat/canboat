/* (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0. */
#define main reader_main
#define read capture_read
#define isReady capture_ready
#include "../nmeareader.c"
#undef main
#undef read
#undef isReady

static int ready_calls;
static int failures;

static void check(int ok, const char *what)
{
  if (!ok)
  {
    fprintf(stderr, "FAIL: %s\n", what);
    failures++;
  }
}

/* The first wait offers stdin, which reads EOF. After that, stdin must no
 * longer be selected; the device then reports an error so the loop ends. */
int capture_ready(SOCKET device, SOCKET input, SOCKET extra, int timeout)
{
  (void) device;
  (void) extra;
  (void) timeout;
  if (ready_calls++ == 0)
  {
    check(input == STDIN, "first wait selects stdin");
    return FD2_ReadReady;
  }
  check(input == INVALID_SOCKET, "wait after stdin EOF does not select stdin");
  return FD1_ReadReady;
}

ssize_t capture_read(int fd, void *data, size_t len)
{
  (void) data;
  (void) len;
  return fd == STDIN ? 0 : -1;
}

int main(void)
{
  char path[] = "/tmp/canboat-stdin-eof-XXXXXX";
  int  fd     = mkstemp(path);
  if (fd < 0)
  {
    perror("mkstemp");
    return 1;
  }
  close(fd);
  char *args[] = {"nmea0183-serial", path, NULL};
  int   result = reader_main(2, args);
  unlink(path);
  check(result == 0, "reader exits with status 0");
  check(ready_calls == 2, "reader waits exactly twice");
  if (failures == 0)
  {
    printf("test-nmea-stdin-eof: OK\n");
  }
  return failures == 0 ? 0 : 1;
}
