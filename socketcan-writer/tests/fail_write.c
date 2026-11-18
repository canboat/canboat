/* (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0. */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/* Wraps write() to stdout, the test's stand-in for the CAN socket. Calls
 * CANBOAT_TEST_FAIL_WRITE up to (but not including) that number plus
 * CANBOAT_TEST_FAIL_COUNT (default 1) fail with CANBOAT_TEST_FAIL_ERRNO
 * (default ENOSPC). Every call is reported on stderr. */
ssize_t __real_write(int fd, const void *data, size_t length);
ssize_t __wrap_write(int fd, const void *data, size_t length)
{
  static unsigned long calls;
  if (fd == STDOUT_FILENO)
  {
    calls++;
    fprintf(stderr, "frame-attempt:%lu\n", calls);
    const char   *fail  = getenv("CANBOAT_TEST_FAIL_WRITE");
    const char   *count = getenv("CANBOAT_TEST_FAIL_COUNT");
    const char   *err   = getenv("CANBOAT_TEST_FAIL_ERRNO");
    unsigned long first = fail ? strtoul(fail, NULL, 10) : 0;
    unsigned long n     = count ? strtoul(count, NULL, 10) : 1;
    if (first != 0 && calls >= first && calls < first + n)
    {
      errno = err ? (int) strtol(err, NULL, 10) : ENOSPC;
      return -1;
    }
  }
  return __real_write(fd, data, length);
}
