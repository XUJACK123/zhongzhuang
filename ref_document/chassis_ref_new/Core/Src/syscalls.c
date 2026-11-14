/* Bare-metal image has no POSIX console or files. Explicit stubs keep libc
 * error paths defined without implying UART/semihosting output is available. */
#include <errno.h>
int _close(int fd) { (void)fd; errno=ENOSYS; return -1; }
int _lseek(int fd, int offset, int whence) {
    (void)fd; (void)offset; (void)whence; errno=ENOSYS; return -1;
}
int _read(int fd, char *buffer, int count) {
    (void)fd; (void)buffer; (void)count; errno=ENOSYS; return -1;
}
int _write(int fd, const char *buffer, int count) {
    (void)fd; (void)buffer; (void)count; errno=ENOSYS; return -1;
}
