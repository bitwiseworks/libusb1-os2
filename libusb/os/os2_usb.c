/*
 * Copyright (c) 2013 Paul Smedley <paul@smedley.id.au>
 * Heavily influenced by openbsd_usb.c 
 * & the libusb 0.1.x port Copyright (c) 2006 Richard L Walsh
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <sys/time.h>
#include <sys/types.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>

#define INCL_DOS
#include <os2.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/errno.h>
#include "usbcalls.h"

#include "libusb.h"
#include "libusbi.h"

struct device_priv {
	char devnode[16];
	int fd;

	unsigned char *cdesc;			/* active config descriptor */
	struct libusb_device_descriptor ddesc;		/* usb device descriptor */
	char report[2048];
};

struct handle_priv {
	int pipe[2];				/* for event notification */
	int endpoints[USB_MAXENDPOINTS];
};

#if 1 /* directly call usbresmg */
HFILE g_hUSBDrv;
BOOL  g_fInit;
#define  IOCAT_USBRES            0x000000A0  // USB Resource device control
#define  IOCTLF_SENDCONTROLURB   0x00000036

typedef struct
{
  ULONG  ulHandle;
  UCHAR  bRequestType;
  UCHAR  bRequest;
  USHORT wValue;
  USHORT wIndex;
  USHORT wLength;
  ULONG  ulTimeout; /* in milliseconds */
  USHORT usStatus;
} USBCALLS_CTRL_REQ, *PUSBCALLS_CTRL_REQ;
#endif

#define UE_ADDR         0x0f
#define UE_GET_ADDR(a)  ((a) & UE_ADDR)


/*
 * Backend functions
 */
static int os2_get_device_list(struct libusb_context *,
    struct discovered_devs **);
static int os2_open(struct libusb_device_handle *);
static void os2_close(struct libusb_device_handle *);

static int os2_get_device_descriptor(struct libusb_device *, unsigned char *,
    int *);
static int os2_get_active_config_descriptor(struct libusb_device *,
    unsigned char *, size_t, int *);
static int os2_get_config_descriptor(struct libusb_device *, uint8_t,
    unsigned char *, size_t, int *);

static int os2_get_configuration(struct libusb_device_handle *, int *);
static int os2_set_configuration(struct libusb_device_handle *, int);

static int os2_claim_interface(struct libusb_device_handle *, int);
static int os2_release_interface(struct libusb_device_handle *, int);

static int os2_set_interface_altsetting(struct libusb_device_handle *, int,
    int);
static int os2_clear_halt(struct libusb_device_handle *, unsigned char);
static int os2_reset_device(struct libusb_device_handle *);
static void os2_destroy_device(struct libusb_device *);

static int os2_submit_transfer(struct usbi_transfer *);
static int os2_cancel_transfer(struct usbi_transfer *);
static void os2_clear_transfer_priv(struct usbi_transfer *);
static int os2_handle_events(struct libusb_context *ctx, struct pollfd *,
    unsigned int, int);
static int os2_clock_gettime(int, struct timespec *);
static int os2_kernel_driver_active(struct libusb_device_handle *dev_handle, int iface);
static int os2_attach_kernel_driver(struct libusb_device_handle *dev_handle, int iface);
static int os2_detach_kernel_driver(struct libusb_device_handle *dev_handle, int iface);

/*
 * Private functions
 */
static int _errno_to_libusb(int);
/*static int _cache_active_config_descriptor(struct libusb_device *, int);*/
static int _sync_control_transfer(struct usbi_transfer *);
static int _sync_gen_transfer(struct usbi_transfer *);
static int _access_endpoint(struct libusb_transfer *);

const struct usbi_os_backend os2_backend = {
	.name = "Synchronous OS/2 backend",
	.init = NULL,
	.exit = NULL,
	.get_device_list = os2_get_device_list,
	.get_device_descriptor = os2_get_device_descriptor,
	.get_active_config_descriptor = os2_get_active_config_descriptor,
	.get_config_descriptor = os2_get_config_descriptor,

	.open = os2_open,
	.close = os2_close,
	.get_configuration = os2_get_configuration,
	.set_configuration = os2_set_configuration,
	.claim_interface = os2_claim_interface,
	.release_interface = os2_release_interface,

	.set_interface_altsetting = os2_set_interface_altsetting,
	.clear_halt = os2_clear_halt,
	.reset_device = os2_reset_device,

	.kernel_driver_active = os2_kernel_driver_active,
	.detach_kernel_driver = os2_detach_kernel_driver,
	.attach_kernel_driver = os2_attach_kernel_driver,

	.destroy_device = os2_destroy_device,

	.submit_transfer = os2_submit_transfer,
	.cancel_transfer = os2_cancel_transfer,
	.clear_transfer_priv = os2_clear_transfer_priv,

	.handle_events = os2_handle_events,

	.clock_gettime = os2_clock_gettime,

	.device_priv_size = sizeof(struct device_priv),
	.device_handle_priv_size = sizeof(struct handle_priv),
	.transfer_priv_size = 0,
	.add_iso_packet_size = 0,
};

int
os2_get_device_list(struct libusb_context * ctx,
	struct discovered_devs **discdevs)
{
	APIRET    rc;
	ULONG     cntDev;
	ULONG     ctr;
	ULONG     len;
	struct libusb_device *dev;
	struct device_priv *dpriv;

	unsigned long session_id;
	char      report[2048];

	usbi_dbg("");

	rc = UsbQueryNumberDevices( &cntDev);
	if (rc) {
		usbi_dbg( "unable to query number of USB devices - rc= %x", (int)rc);
		return -1;
	}
	usbi_dbg( "%d devices detected", cntDev);
	for (ctr = 1; ctr <= cntDev; ctr++) {
		session_id = ctr;
		dev = usbi_get_device_by_session_id(ctx, session_id);
		if (dev == NULL) {
			dev = usbi_alloc_device(ctx, session_id);
			if (dev == NULL)
				return (LIBUSB_ERROR_NO_MEM);
			dev->bus_number = (uint8_t) 1;
			dev->device_address = ctr;
			dev->speed = LIBUSB_SPEED_UNKNOWN;

			/* add device_descriptor to dpriv->ddesc */
			len = sizeof(report);
			rc = UsbQueryDeviceReport( ctr, &len, report);
			if (rc) {
				usbi_dbg( "unable to query device report - device= %d  rc= %x",	(int)ctr, (int)rc);
				return -1;
			}
			dpriv = (struct device_priv *)dev->os_priv;
			memcpy( &dpriv->ddesc, report, sizeof(dpriv->ddesc));
			memcpy( &dpriv->report, report+sizeof(dpriv->ddesc), (len - sizeof(dpriv->ddesc)));

			usbi_sanitize_device(dev);
		}
		if (discovered_devs_append(*discdevs, dev) == NULL)
			return (LIBUSB_ERROR_NO_MEM);
	}
  return (LIBUSB_SUCCESS);
}

int
os2_open(struct libusb_device_handle *handle)
{
/* Adapted from usb_os_open() in Rich's libusb 0.1.x port */
	struct handle_priv *hpriv = (struct handle_priv *)handle->os_priv;
	struct device_priv *dpriv = (struct device_priv *)handle->dev->os_priv;
	int       rtn = 0;
	APIRET    rc;
	int       usbhandle;

	rc = UsbOpen( (PUSBHANDLE)&usbhandle,
		(USHORT)dpriv->ddesc.idVendor,
		(USHORT)dpriv->ddesc.idProduct,
		(USHORT)dpriv->ddesc.bcdDevice,
		(USHORT)USB_OPEN_FIRST_UNUSED);

	if (rc) {
		usbi_dbg( "unable to open device - id= %x/%x  rc= %x",
			dpriv->ddesc.idVendor, 
			dpriv->ddesc.idProduct, (int)rc);
		usbhandle = -1;
		rtn = -1;
	}

	dpriv->fd = usbhandle;
	usbi_dbg("open: rc = %x, fd %d", rc, dpriv->fd);
	/* set device configuration to 1st configuration */
	rc = UsbDeviceSetConfiguration (dpriv->fd,1);
	usbi_dbg("open, set device configuration: rc = %x, fd %d", rc, dpriv->fd);

	int fds[2];
	if (socketpair(AF_UNIX, SOCK_STREAM, 0, fds) < 0)
		return _errno_to_libusb(errno);
	hpriv->pipe[0]=fds[0];
	hpriv->pipe[1]=fds[1];
	return usbi_add_pollfd(HANDLE_CTX(handle), hpriv->pipe[0], POLLIN);

}

void
os2_close(struct libusb_device_handle *handle)
{
/* Adapted from usb_os_close() in Rich's libusb 0.1.x port */
	struct handle_priv *hpriv = (struct handle_priv *)handle->os_priv;
	struct device_priv *dpriv = (struct device_priv *)handle->dev->os_priv;
	APIRET    rc;

	usbi_dbg("close: fd %d", dpriv->fd);

	rc = UsbClose( (USBHANDLE)dpriv->fd);
	if (rc) {
		usbi_dbg( "unable to close device - id= %x/%x  handle= %x  rc= %d",
		    dpriv->ddesc.idVendor, 
		    dpriv->ddesc.idProduct,
		    dpriv->fd, (int)rc);
	}

	dpriv->fd = -1;
	usbi_remove_pollfd(HANDLE_CTX(handle), hpriv->pipe[0]);

	close(hpriv->pipe[0]);
	close(hpriv->pipe[1]);
}

int
os2_get_device_descriptor(struct libusb_device *dev, unsigned char *buf,
    int *host_endian)
{
	usbi_dbg("");
	struct device_priv *dpriv = (struct device_priv *)dev->os_priv;

	memcpy(buf, &dpriv->ddesc, DEVICE_DESC_LENGTH);

	*host_endian = 0;

	return (LIBUSB_SUCCESS);
}

int
os2_get_active_config_descriptor(struct libusb_device *dev,
    unsigned char *buf, size_t len, int *host_endian)
{
	usbi_dbg("");
	struct device_priv *dpriv = (struct device_priv *)dev->os_priv;
	struct libusb_config_descriptor *ucd;

	ucd = (struct libusb_config_descriptor*) (dpriv->report);
	len = MIN(len, ucd->wTotalLength);
	usbi_dbg("len %d", len);

	memcpy(buf, dpriv->report, len); /* added 28-04-2013 */

	*host_endian = 0;

	return (LIBUSB_SUCCESS);
}

int
os2_get_config_descriptor(struct libusb_device *dev, uint8_t idx,
    unsigned char *buf, size_t len, int *host_endian)
{
	usbi_dbg("");
	struct device_priv *dpriv = (struct device_priv *)dev->os_priv;
	struct libusb_config_descriptor *ucd;

	usbi_dbg("len %d", len);
	memcpy(buf, dpriv->report, len);

	*host_endian = 0;

	return (LIBUSB_SUCCESS);
}

int
os2_get_configuration(struct libusb_device_handle *handle, int *config)
{
	usbi_dbg("");
	return (LIBUSB_ERROR_NOT_SUPPORTED);
}

int
os2_set_configuration(struct libusb_device_handle *handle, int config)
{
/* based on OpenBSD code*/ 
	struct device_priv *dpriv = (struct device_priv *)handle->dev->os_priv;
	APIRET    rc;

	usbi_dbg("configuration %d", config);

	rc = UsbDeviceSetConfiguration( (USBHANDLE)dpriv->fd, (USHORT)config);
	if (rc) {
		usbi_dbg( "unable to set configuration - config= %d  rc= %x",
			config, (int)rc);
		return -1;
	}

	return 0;
}

int
os2_claim_interface(struct libusb_device_handle *handle, int iface)
{
/* USBRESM$ appears to handle this as part of opening the device */
	usbi_dbg("");
	return (LIBUSB_SUCCESS);
}

int
os2_release_interface(struct libusb_device_handle *handle, int iface)
{
/* USBRESM$ appears to handle this as part of opening the device */
	usbi_dbg("");
	return (LIBUSB_SUCCESS);
}

int
os2_set_interface_altsetting(struct libusb_device_handle *handle, int iface,
    int altsetting)
{
/* TO DO */
	usbi_dbg("");
	return (LIBUSB_SUCCESS);
}

int
os2_clear_halt(struct libusb_device_handle *handle, unsigned char endpoint)
{
/* TO DO */
	usbi_dbg("");
	return (LIBUSB_SUCCESS);
}

int
os2_reset_device(struct libusb_device_handle *handle)
{
/* TO DO */
	usbi_dbg("");
	return (LIBUSB_ERROR_NOT_SUPPORTED);
}

void
os2_destroy_device(struct libusb_device *dev)
{
/* copied from BSD */
	struct device_priv *dpriv = (struct device_priv *)dev->os_priv;

	usbi_dbg("");

	free(dpriv->cdesc);
}

int
os2_submit_transfer(struct usbi_transfer *itransfer)
{
/* Copied from BSD */
	struct libusb_transfer *transfer;
	struct handle_priv *hpriv;
	int err = 0;

	usbi_dbg("");

	transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	hpriv = (struct handle_priv *)transfer->dev_handle->os_priv;
	switch (transfer->type) {
	case LIBUSB_TRANSFER_TYPE_CONTROL:
		err = _sync_control_transfer(itransfer);
		break;
	case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
		if (IS_XFEROUT(transfer)) {
			/* Isochronous write is not supported */
			err = LIBUSB_ERROR_NOT_SUPPORTED;
			break;
		}
		err = _sync_gen_transfer(itransfer);
		break;
	case LIBUSB_TRANSFER_TYPE_BULK:
	case LIBUSB_TRANSFER_TYPE_INTERRUPT:
		if (IS_XFEROUT(transfer) &&
		    transfer->flags & LIBUSB_TRANSFER_ADD_ZERO_PACKET) {
			err = LIBUSB_ERROR_NOT_SUPPORTED;
			break;
		}
		err = _sync_gen_transfer(itransfer);
		break;
	}
	if (err)
		return (err);

	if (write(hpriv->pipe[1], &itransfer, sizeof(itransfer)) < 0)
		return _errno_to_libusb(errno);

	return (LIBUSB_SUCCESS);
}

int
os2_cancel_transfer(struct usbi_transfer *itransfer)
{
/* Not supported on bsd, so we won't either */
	usbi_dbg("");

	return (LIBUSB_ERROR_NOT_SUPPORTED);
}

void
os2_clear_transfer_priv(struct usbi_transfer *itransfer)
{
/* Copied from openbsd_usb.c */
	usbi_dbg("");

	/* Nothing to do */
}

int
os2_handle_events(struct libusb_context *ctx, struct pollfd *fds, unsigned int nfds,
    int num_ready)
{
/* copied from openbsd_usb.c */
	struct libusb_device_handle *handle;
	struct handle_priv *hpriv = NULL;
	struct usbi_transfer *itransfer;
	struct pollfd *pollfd;
	int i, err = 0;

	usbi_dbg("");

	pthread_mutex_lock(&ctx->open_devs_lock);
	for (i = 0; i < nfds && num_ready > 0; i++) {
		pollfd = &fds[i];

		if (!pollfd->revents)
			continue;

		hpriv = NULL;
		num_ready--;
		list_for_each_entry(handle, &ctx->open_devs, list,
		    struct libusb_device_handle) {
			hpriv = (struct handle_priv *)handle->os_priv;

			if (hpriv->pipe[0] == pollfd->fd)
				break;

			hpriv = NULL;
		}

		if (NULL == hpriv) {
			usbi_dbg("fd %d is not an event pipe!", pollfd->fd);
			err = ENOENT;
			break;
		}

		if (pollfd->revents & POLLERR) {
			usbi_remove_pollfd(HANDLE_CTX(handle), hpriv->pipe[0]);
			usbi_handle_disconnect(handle);
			continue;
		}

		if (read(hpriv->pipe[0], &itransfer, sizeof(itransfer)) < 0) {
			err = errno;
			break;
		}

		if ((err = usbi_handle_transfer_completion(itransfer,
		    LIBUSB_TRANSFER_COMPLETED)))
			break;
	}
	pthread_mutex_unlock(&ctx->open_devs_lock);

	if (err)
		return _errno_to_libusb(err);

	return (LIBUSB_SUCCESS);
}

inline int
clock_gettime(int clock_id, struct timespec *ts)
{
	struct timeval tv;

#if 0
	if (clock_id != CLOCK_REALTIME) {
		errno = EINVAL;
		return (-1);
	}
#endif
	if (gettimeofday(&tv, NULL) < 0)
		return (-1);
	ts->tv_sec = tv.tv_sec;
	ts->tv_nsec = tv.tv_usec * 1000;
	return (0);
}

int
os2_clock_gettime(int clkid, struct timespec *tp)
{
	usbi_dbg("clock %d", clkid);

	if (clkid == USBI_CLOCK_REALTIME)
		return clock_gettime(CLOCK_REALTIME, tp);

	if (clkid == USBI_CLOCK_MONOTONIC)
		return clock_gettime(CLOCK_MONOTONIC, tp);

	return (LIBUSB_ERROR_INVALID_PARAM);
}

int
_errno_to_libusb(int err)
{
	switch (err) {
	case EIO:
		return (LIBUSB_ERROR_IO);
	case EACCES:
		return (LIBUSB_ERROR_ACCESS);
	case ENOENT:
		return (LIBUSB_ERROR_NO_DEVICE);
	case ENOMEM:
		return (LIBUSB_ERROR_NO_MEM);
	}

	usbi_dbg("error: %s", strerror(err));

	return (LIBUSB_ERROR_OTHER);
}

int
_sync_control_transfer(struct usbi_transfer *itransfer)
{
	usbi_dbg("");
	struct libusb_transfer *transfer;
	struct libusb_control_setup *setup;
	struct device_priv *dpriv;
	APIRET    rc;

	transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	dpriv = (struct device_priv *)transfer->dev_handle->dev->os_priv;
	setup = (struct libusb_control_setup *)transfer->buffer;

	usbi_dbg("usbhandle = %d, type %d request %d value %d index %d length %d timeout %d",
	    dpriv->fd, setup->bmRequestType, setup->bRequest,
	    libusb_le16_to_cpu(setup->wValue),
	    libusb_le16_to_cpu(setup->wIndex),
	    libusb_le16_to_cpu(setup->wLength), transfer->timeout);

	rc = UsbCtrlMessage( (USBHANDLE) dpriv->fd, (UCHAR)setup->bmRequestType,
	    (UCHAR)setup->bRequest, (USHORT)setup->wValue, (USHORT)setup->wIndex,
	    (USHORT)setup->wLength, transfer->buffer + LIBUSB_CONTROL_SETUP_SIZE, (ULONG)transfer->timeout);


	if (rc) {
		usbi_dbg( "unable to send control message - rc= %d", (int)rc);
		return -1;
	}

	itransfer->transferred = setup->wLength; /* not right, should be length actually transferred */
	usbi_dbg("transferred %d", itransfer->transferred);
	return (0);
}

int
_access_endpoint(struct libusb_transfer *transfer)
{
	struct handle_priv *hpriv;
	struct device_priv *dpriv;
	char *s, devnode[16];
	int fd, endpt;
	mode_t mode;

	hpriv = (struct handle_priv *)transfer->dev_handle->os_priv;
	dpriv = (struct device_priv *)transfer->dev_handle->dev->os_priv;

	endpt = UE_GET_ADDR(transfer->endpoint);
	mode = IS_XFERIN(transfer) ? O_RDONLY : O_WRONLY;

	usbi_dbg("endpoint %d mode %d", endpt, mode);

	if (hpriv->endpoints[endpt] < 0) {
		/* Pick the right node given the control one */
		strlcpy(devnode, dpriv->devnode, sizeof(devnode));
		s = strchr(devnode, '.');
		snprintf(s, 4, ".%02d", endpt);

		/* We may need to read/write to the same endpoint later. */
		if (((fd = open(devnode, O_RDWR)) < 0) && (errno == ENXIO))
			if ((fd = open(devnode, mode)) < 0)
				return (-1);
		hpriv->endpoints[endpt] = fd;
	}

	return (hpriv->endpoints[endpt]);
}

int
_sync_gen_transfer(struct usbi_transfer *itransfer)
{
	usbi_dbg("");
	struct libusb_transfer *transfer;
	struct device_priv *dpriv;
	transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	dpriv = (struct device_priv *)transfer->dev_handle->dev->os_priv;

	int nr = transfer->length;
		usbi_dbg("_sync_gen_transfer nr = %d",nr);
	APIRET    rc;

	if (IS_XFERIN(transfer)) {
	usbi_dbg("_sync_gen_transfer - UsbBulkRead");
		rc = UsbBulkRead( (USBHANDLE) dpriv->fd, (UCHAR)transfer->endpoint, 0,
			(PULONG)&nr, transfer->buffer, (ULONG)transfer->timeout);
		usbi_dbg("usbcalls UsbBulkRead with dh = %p",dpriv->fd);
		usbi_dbg("usbcalls UsbBulkRead with bulk_in_ep = 0x%02x",transfer->endpoint);
		usbi_dbg("usbcalls UsbBulkRead with usbcalls_timeout = %d",transfer->timeout);
		usbi_dbg("usbcalls UsbBulkRead nr2 = %d",nr);
		usbi_dbg("usbcalls UsbBulkRead rc = %d",rc);

		if (rc == 640) {
			usbi_dbg("usbbulkread timeout");
			nr = -ETIMEDOUT;
		}
		// Froloff keeps changing the return code
		if (rc && rc != USB_ERROR_LESSTRANSFERED && rc != 0x8004 && rc != 7004  && rc!=32772 && rc !=28676) {
			usbi_dbg( "unable to read from bulk endpoint - size= %d  nr= %d  rc= %x",
				transfer->length, nr, (int)rc);
			nr = -1;
		}

	} else {
	usbi_dbg("_sync_gen_transfer - USBBulkWrite");

		rc = UsbBulkWrite( (USBHANDLE) dpriv->fd, (UCHAR)transfer->endpoint, 0,
			(PULONG)nr, transfer->buffer, (ULONG)transfer->timeout);
		usbi_dbg("usbcalls UsbBulkWrite with dh = %p",dpriv->fd);
		usbi_dbg("usbcalls UsbBulkWrite with bulk_out_ep = 0x%02x",transfer->endpoint);
		usbi_dbg("usbcalls UsbBulkWrite with usbcalls_timeout = %d",transfer->timeout);
		usbi_dbg("usbcalls UsbBulkWrite rc = %d",rc);
		if (rc) {
			usbi_dbg( "unable to write to bulk endpoint - size= %d  nr= %d  rc= %x",
				transfer->length, nr, (int)rc);
			nr = -1;
		}
	}
	if (nr < 0)
		return _errno_to_libusb(errno);
	itransfer->transferred = nr;
	usbi_dbg("itransfer->transferred = %d, nr =%d",itransfer->transferred, nr);
	return (0);
}

// The 3 functions below are unlikely to ever get supported on OS/2
static int os2_kernel_driver_active(struct libusb_device_handle *dev_handle, int iface)
{
	return LIBUSB_ERROR_NOT_SUPPORTED;
}

static int os2_attach_kernel_driver(struct libusb_device_handle *dev_handle, int iface)
{
	return LIBUSB_ERROR_NOT_SUPPORTED;
}

static int os2_detach_kernel_driver(struct libusb_device_handle *dev_handle, int iface)
{
	return LIBUSB_ERROR_NOT_SUPPORTED;
}
