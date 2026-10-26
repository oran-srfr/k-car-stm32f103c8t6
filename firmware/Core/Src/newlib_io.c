#include <errno.h>
#include <sys/types.h>

/* No console/serial transport is configured yet. Fail explicitly instead of
 * silently dropping output or relying on a debugger's semihosting service.
 * Replace _write/_read when UART logging is added. */
int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

off_t _lseek(int file, off_t offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;
    errno = ESPIPE;
    return (off_t)-1;
}

int _read(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    errno = ENOSYS;
    return -1;
}

int _write(int file, const char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    errno = ENOSYS;
    return -1;
}
