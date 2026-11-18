/* (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0. */
/* The system headers come first, so the redirects below only rename the
 * calls in nmeareader.c and not a fortified inline read() in <unistd.h>. */
#include "common.h"

int     capture_ready(SOCKET device, SOCKET input, SOCKET extra, int timeout);
ssize_t capture_read(int fd, void *data, size_t len);

#define main reader_main
#define read capture_read
#define isReady capture_ready
#include "../nmeareader.c"
#undef main
#undef read
#undef isReady

static int    ready_calls;
static int    device_reads;
static int    stdin_reads;
static SOCKET device_fd = INVALID_SOCKET;
static int    failures;
static char   path[] = "/tmp/canboat-stdin-eof-XXXXXX";

static void check(int ok, const char *what)
{
  if (!ok)
  {
    fprintf(stderr, "FAIL: %s\n", what);
    failures++;
  }
}

/* The first wait offers stdin, which reads EOF. After that, stdin must no
 * longer be selected while the same device still is; the device then
 * reports an error so the loop ends. */
int capture_ready(SOCKET device, SOCKET input, SOCKET extra, int timeout)
{
  (void) extra;
  (void) timeout;
  if (ready_calls++ == 0)
  {
    device_fd = device;
    check(device != INVALID_SOCKET && device != STDIN, "first wait selects the device");
    check(input == STDIN, "first wait selects stdin");
    return FD2_ReadReady;
  }
  if (ready_calls > 2)
  {
    /* The device read did not end the loop; stop rather than spin forever. */
    fprintf(stderr, "FAIL: reader waits again after the device read error\n");
    unlink(path);
    exit(1);
  }
  check(device == device_fd, "wait after stdin EOF still selects the device");
  check(stdin_reads == 1, "stdin EOF is read before the second wait");
  check(input == INVALID_SOCKET, "wait after stdin EOF does not select stdin");
  return FD1_ReadReady;
}

ssize_t capture_read(int fd, void *data, size_t len)
{
  (void) data;
  (void) len;
  if (fd == STDIN)
  {
    stdin_reads++;
    return 0;
  }
  check(fd == device_fd, "device read uses the selected device");
  device_reads++;
  return -1;
}

int main(void)
{
  /* Held open until the reader returns, so the device the reader opens
   * cannot land on descriptor 0 even when the test starts without stdin. */
  int fd = mkstemp(path);
  if (fd < 0)
  {
    perror("mkstemp");
    return 1;
  }
  char *args[] = {"nmea0183-serial", path, NULL};
  int   result = reader_main(2, args);
  close(fd);
  unlink(path);
  check(result == 0, "reader exits with status 0");
  check(ready_calls == 2, "reader waits exactly twice");
  check(device_reads == 1, "reader reads the device once after stdin EOF");
  if (failures == 0)
  {
    printf("test-nmea-stdin-eof: OK\n");
  }
  return failures == 0 ? 0 : 1;
}
