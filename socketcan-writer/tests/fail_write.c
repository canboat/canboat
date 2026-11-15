/* (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0. */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

ssize_t __real_write(int fd, const void *data, size_t length);
ssize_t __wrap_write(int fd, const void *data, size_t length)
{
  static unsigned calls;
  if (fd == STDOUT_FILENO)
  {
    calls++;
    fprintf(stderr, "frame-attempt:%u\n", calls);
    const char *fail = getenv("CANBOAT_TEST_FAIL_WRITE");
    if (fail && calls == strtoul(fail, NULL, 10))
    {
      errno = ENOSPC;
      return -1;
    }
  }
  return __real_write(fd, data, length);
}
