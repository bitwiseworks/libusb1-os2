#ifndef LIBUSB_POLL_POSIX_H
#define LIBUSB_POLL_POSIX_H

#include <limits.h>
#ifndef FD_SETSIZE
#define FD_SETSIZE OPEN_MAX
#endif
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/errno.h>
#include <unistd.h>
#include <stdlib.h>

typedef struct pollfd {
    int fd;                         /* file desc to poll */
    short events;                   /* events of interest on fd */
    short revents;                  /* events that occurred on fd */
} pollfd_t;


// poll flags
#define POLLIN  0x0001
#define POLLOUT 0x0004
#define POLLERR 0x0008

// synonyms
#define POLLNORM POLLIN
#define POLLPRI POLLIN
#define POLLRDNORM POLLIN
#define POLLRDBAND POLLIN
#define POLLWRNORM POLLOUT
#define POLLWRBAND POLLOUT

// ignored
#define POLLHUP 0x0010
#define POLLNVAL 0x0020

#define usbi_write write
#define usbi_read read
#define usbi_close close
int usbi_pipe(int pipefd[2]);
int usbi_poll(struct pollfd *pollSet, int pollCount, int pollTimeout);

#endif /* LIBUSB_POLL_POSIX_H */
