/* (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0. */
#define main reader_main
#define read capture_read
#define isReady capture_ready
#include "../nmeareader.c"
#undef main
#undef read
#undef isReady
#include <assert.h>

static int ready_calls;
int        capture_ready(SOCKET device, SOCKET input, SOCKET extra, int timeout)
{
  (void) device;
  (void) extra;
  (void) timeout;
  if (ready_calls++ == 0)
  {
    assert(input == STDIN);
    return FD2_ReadReady;
  }
  assert(input == INVALID_SOCKET);
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
  assert(fd >= 0);
  close(fd);
  char *args[] = {"nmea0183-serial", path, NULL};
  int   result = reader_main(2, args);
  unlink(path);
  assert(ready_calls == 2);
  return result;
}
