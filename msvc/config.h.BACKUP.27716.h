/* config.h.  Manual config for MSVC.  */

#ifndef _MSC_VER
#warn "msvc/config.h shouldn't be included for your development environment."
#error "Please make sure the msvc/ directory is removed from your build path."
#endif

/* Default visibility */
#define DEFAULT_VISIBILITY /**/

/* Debug message logging */
//#define ENABLE_DEBUG_LOGGING 1

/* Message logging */
#define ENABLE_LOGGING 1

/* Windows backend */
#define OS_WINDOWS 1

/* type of second poll() argument */
#define POLL_NFDS_TYPE unsigned int

<<<<<<< HEAD
/* no way to run git describe from MSVC? */
#define LIBUSB_DESCRIBE ""
=======
/* Define to 1 if you have the <signal.h> header file. */
#define HAVE_SIGNAL_H 1

/* Define to 1 if you have the <sys/types.h> header file. */
#define HAVE_SYS_TYPES_H 1
>>>>>>> 1e6928c... Core: Use HAVE_SYS_TYPES_H and HAVE_SIGNAL_H
