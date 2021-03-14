/*
 * Copyright (c) 2013 Paul Smedley <paul@smedley.id.au>
 * Heavily influenced by openbsd_usb.c
 * & the libusb 0.1.x port Copyright (c) 2006 Richard L Walsh
 * Copyright (c) 2021 Lars Erdmann
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

#define INCL_DOS
#define INCL_DOSERRORS
#include <os2.h>

#include <config.h>

#include <sys/time.h>
#include <sys/types.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>

#include <stdint.h>
#include <usbcalls.h>

#include "libusbi.h"
#include "os2_usb.h"

struct device_priv {
   int fd;              /* device file descriptor */
   int altsetting;      /* alternate setting for the interface in use */
   struct libusb_context *ctx;
   struct libusb_device_descriptor ddesc; /* usb device descriptor */
   unsigned char cdesc[4096];    /* active config descriptor */
};


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
static int os2_handle_transfer_completion(struct usbi_transfer *);
static int os2_clock_gettime(int, struct timespec *);
static int os2_kernel_driver_active(struct libusb_device_handle *dev_handle, int iface);
static int os2_attach_kernel_driver(struct libusb_device_handle *dev_handle, int iface);
static int os2_detach_kernel_driver(struct libusb_device_handle *dev_handle, int iface);

/*
 * Private functions
 */
static int _apiret_to_libusb(int);
static int _sync_control_transfer(struct usbi_transfer *);
static int _sync_bulk_transfer(struct usbi_transfer *);
static int _sync_irq_transfer(struct usbi_transfer *);
static int _sync_iso_transfer(struct usbi_transfer *);

const struct usbi_os_backend os2_backend = {
   "Synchronous OS/2 backend",
   0,
   NULL,          /* init() */
   NULL,          /* exit() */
   os2_get_device_list,
   NULL,          /* hotplug_poll */
   os2_open,
   os2_close,

   os2_get_device_descriptor,
   os2_get_active_config_descriptor,
   os2_get_config_descriptor,
   NULL,          /* get_config_descriptor_by_value() */

   os2_get_configuration,
   os2_set_configuration,

   os2_claim_interface,
   os2_release_interface,

   os2_set_interface_altsetting,
   os2_clear_halt,
   os2_reset_device,

   NULL,          /* alloc_streams */
   NULL,          /* free_streams */

   NULL,          /* dev_mem_alloc() */
   NULL,          /* dev_mem_free() */

   os2_kernel_driver_active,
   os2_detach_kernel_driver,
   os2_attach_kernel_driver,

   os2_destroy_device,

   os2_submit_transfer,
   os2_cancel_transfer,
   os2_clear_transfer_priv,

   NULL,          /* handle_events() */
   os2_handle_transfer_completion,

   os2_clock_gettime,

#ifdef USBI_TIMERFD_AVAILABLE
   /* clock ID of the clock that should be used for timerfd */
   NULL,          /* get_timerfd_clockid() */
#endif

   sizeof(struct device_priv),
   0,          /* OS/2 does not access endpoints via file handle */
   0,          /* transfer_priv_size */
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
   char     scratchBuf[DEVICE_DESC_LENGTH+4096];
   GETDEVINFODATA info;


   usbi_dbg("");

   rc = UsbQueryNumberDevices( &cntDev);
   if (rc) {
      usbi_dbg( "unable to query number of USB devices - rc= %x", (int)rc);
      return(LIBUSB_ERROR_IO);
   }
   usbi_dbg( "%d devices detected", cntDev);
   for (ctr = 1; ctr <= cntDev; ctr++) {
      session_id = ctr;
      dev = usbi_get_device_by_session_id(ctx, session_id);
      if (dev == NULL) {
         dev = usbi_alloc_device(ctx, session_id);
         if (dev == NULL)
            return(LIBUSB_ERROR_NO_MEM);

         len = sizeof(info);
         rc = UsbQueryDeviceInfo( ctr,&len, (PUCHAR)&info);

         dev->bus_number     = info.ctrlID;        /* under OS/2, the used HC is equivalent to the bus number */
         dev->device_address = info.deviceAddress; /* the real USB address programmed into the USB device */
         dev->port_number    = info.portNum;       /* the real port number on the hub the device is attached to, is zero for a root hub device */
         switch (info.SpeedDevice) {
         case 0:
            dev->speed = LIBUSB_SPEED_FULL;
            break;
         case 1:
            dev->speed = LIBUSB_SPEED_LOW;
            break;
         case 2:
            dev->speed = LIBUSB_SPEED_HIGH;
            break;
         default:
            dev->speed = LIBUSB_SPEED_UNKNOWN;
            break;
         } /* endswitch */
         dev->parent_dev = NULL;                   /* under OS/2, we do have the parent hub identifier as info.parentHubIndex */
                                                   /* but only non-hub devices are exposed by USBD.SYS (and therefore USBRESMG.SYS) */
                                                   /* therefore, it is impossible to query the full bus hierarchy */

         /* add device_descriptor to dpriv->ddesc */
         len = sizeof(scratchBuf);
         rc = UsbQueryDeviceReport( ctr, &len, scratchBuf);
         if (rc) {
            usbi_dbg( "unable to query device report - device= %d  rc= %x",   (int)ctr, (int)rc);
            return(LIBUSB_ERROR_IO);
         }
         dpriv = (struct device_priv *)dev->os_priv;
         dpriv->fd = -1;
         dpriv->altsetting = 0;
         memcpy( &dpriv->ddesc, scratchBuf, sizeof(dpriv->ddesc));
         memcpy(  dpriv->cdesc, scratchBuf + sizeof(dpriv->ddesc), (len - sizeof(dpriv->ddesc)));

         usbi_sanitize_device(dev);
      }
      if (discovered_devs_append(*discdevs, dev) == NULL)
         return(LIBUSB_ERROR_NO_MEM);

      libusb_unref_device(dev);
   }
   return(LIBUSB_SUCCESS);
}

int
os2_open(struct libusb_device_handle *handle)
{
   struct device_priv *dpriv = (struct device_priv *)handle->dev->os_priv;
   APIRET    rc;
   int       usbhandle;

   rc = UsbOpen( (PUSBHANDLE)&usbhandle,
      (USHORT)dpriv->ddesc.idVendor,
      (USHORT)dpriv->ddesc.idProduct,
      (USHORT)dpriv->ddesc.bcdDevice,
      (USHORT)USB_OPEN_FIRST_UNUSED);

   /* if we can't attach the device, but have a fd already, this means */
   /* we know the device. so reset error and use the known handle      */
   if (rc && dpriv->fd != -1) {
      rc = LIBUSB_SUCCESS;
      usbhandle = dpriv->fd;
   }

   if (rc) {
      usbi_dbg( "unable to open device - id= %x/%x  rc= %x",
         dpriv->ddesc.idVendor,
         dpriv->ddesc.idProduct, (int)rc);
      dpriv->fd = -1;
      return( _apiret_to_libusb(rc));
   }

   dpriv->fd = usbhandle;
   usbi_dbg("open: fd %d", dpriv->fd);
   /* set device configuration to 1st configuration */
   rc = UsbDeviceSetConfiguration (dpriv->fd,1);
   usbi_dbg("open, set device configuration: rc = %x, fd %d", rc, dpriv->fd);

   return(LIBUSB_SUCCESS);
}

void
os2_close(struct libusb_device_handle *handle)
{
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
}

int
os2_get_device_descriptor(struct libusb_device *dev, unsigned char *buf,
    int *host_endian)
{
   struct device_priv *dpriv = (struct device_priv *)dev->os_priv;

   usbi_dbg("");

   memcpy(buf, &dpriv->ddesc, DEVICE_DESC_LENGTH);

   *host_endian = 0;

   return(LIBUSB_SUCCESS);
}

int
os2_get_active_config_descriptor(struct libusb_device *dev,
    unsigned char *buf, size_t len, int *host_endian)
{
   struct device_priv *dpriv            = (struct device_priv *)dev->os_priv;
   struct libusb_config_descriptor *ucd = (struct libusb_config_descriptor*) (dpriv->cdesc);

   len = MIN(len, ucd->wTotalLength);

   usbi_dbg("len %d", len);

   memcpy(buf, dpriv->cdesc, len); /* added 28-04-2013 */

   *host_endian = 0;

   return(len);
}

int
os2_get_config_descriptor(struct libusb_device *dev, uint8_t idx,
    unsigned char *buf, size_t len, int *host_endian)
{
   /* just return the active config descriptor, it's a bad idea to arbitrarily switch configuration anyway */
   return(os2_get_active_config_descriptor(dev,buf,len,host_endian));
}

int
os2_get_configuration(struct libusb_device_handle *handle, int *config)
{
   struct device_priv *dpriv = (struct device_priv *)handle->dev->os_priv;
   APIRET rc = LIBUSB_SUCCESS;

   usbi_dbg("get configuration");
   *config = 0;
   rc = UsbDeviceGetConfiguration( (USBHANDLE)dpriv->fd, (PUCHAR)config);
   if (rc) {
      usbi_dbg( "unable to get configuration rc= %x",
         (int)rc);
      return(LIBUSB_ERROR_IO);
   }

   return(LIBUSB_SUCCESS);
}

int
os2_set_configuration(struct libusb_device_handle *handle, int config)
{
/* based on OpenBSD code*/
   struct device_priv *dpriv = (struct device_priv *)handle->dev->os_priv;
   APIRET    rc = LIBUSB_SUCCESS;

   usbi_dbg("set configuration %d", config);

   rc = UsbDeviceSetConfiguration( (USBHANDLE)dpriv->fd, (USHORT)config);
   if (rc) {
      usbi_dbg( "unable to set configuration - config= %d  rc= %x",
         config, (int)rc);
      return(LIBUSB_ERROR_IO);
   }

   return(LIBUSB_SUCCESS);
}

int
os2_claim_interface(struct libusb_device_handle *handle, int iface)
{
/* USBRESM$ appears to handle this as part of opening the device */
   usbi_dbg("");
   return(LIBUSB_SUCCESS);
}

int
os2_release_interface(struct libusb_device_handle *handle, int iface)
{
/* USBRESM$ appears to handle this as part of opening the device */
   usbi_dbg("");
   return(LIBUSB_SUCCESS);
}

int
os2_set_interface_altsetting(struct libusb_device_handle *handle, int iface,
    int altsetting)
{
   struct device_priv *dpriv = (struct device_priv *)handle->dev->os_priv;
   APIRET rc = LIBUSB_SUCCESS;

   usbi_dbg("");
   dpriv->altsetting = altsetting;
   rc = UsbInterfaceSetAltSetting((USBHANDLE)dpriv->fd,(USHORT)iface,(USHORT)altsetting);
   return(_apiret_to_libusb(rc));
}

int
os2_clear_halt(struct libusb_device_handle *handle, unsigned char endpoint)
{
   struct device_priv *dpriv = (struct device_priv *)handle->dev->os_priv;
   APIRET rc = LIBUSB_SUCCESS;

   usbi_dbg("");
   rc = UsbEndpointClearHalt((USBHANDLE)dpriv->fd,(USHORT)endpoint);
   return(_apiret_to_libusb(rc));
}

int
os2_reset_device(struct libusb_device_handle *handle)
{
/* TO DO */
   usbi_dbg("");
   return(LIBUSB_ERROR_NOT_SUPPORTED);
}

void
os2_destroy_device(struct libusb_device *dev)
{
   usbi_dbg("");
}

int
os2_submit_transfer(struct usbi_transfer *itransfer)
{
/* Copied from BSD */
   struct libusb_transfer *transfer;
   int err = 0;

   usbi_dbg("");

   transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);

   switch (transfer->type) {
   case LIBUSB_TRANSFER_TYPE_CONTROL:
      err = _sync_control_transfer(itransfer);
      break;
   case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
      err = _sync_iso_transfer(itransfer);
      break;
   case LIBUSB_TRANSFER_TYPE_BULK:
      if (IS_XFEROUT(transfer) &&
          transfer->flags & LIBUSB_TRANSFER_ADD_ZERO_PACKET) {
         err = LIBUSB_ERROR_NOT_SUPPORTED;
         break;
      }
      err = _sync_bulk_transfer(itransfer);
      break;
   case LIBUSB_TRANSFER_TYPE_INTERRUPT:
      if (IS_XFEROUT(transfer) &&
          transfer->flags & LIBUSB_TRANSFER_ADD_ZERO_PACKET) {
         err = LIBUSB_ERROR_NOT_SUPPORTED;
         break;
      }
      err = _sync_irq_transfer(itransfer);
      break;
   case LIBUSB_TRANSFER_TYPE_BULK_STREAM:
      err = LIBUSB_ERROR_NOT_SUPPORTED;
      break;
   }
   if (err)
      return(err);

   usbi_signal_transfer_completion(itransfer);

   return(LIBUSB_SUCCESS);
}

int
os2_cancel_transfer(struct usbi_transfer *itransfer)
{
/* Not supported on bsd, so we won't either */
   usbi_dbg("");

   return(LIBUSB_ERROR_NOT_SUPPORTED);
}

void
os2_clear_transfer_priv(struct usbi_transfer *itransfer)
{
/* Copied from openbsd_usb.c */
   usbi_dbg("");

   /* Nothing to do */
}

int
os2_handle_transfer_completion(struct usbi_transfer *itransfer)
{
   return(usbi_handle_transfer_completion(itransfer, LIBUSB_TRANSFER_COMPLETED));
}

inline int
clock_gettime(int clock_id, struct timespec *ts)
{
   struct timeval tv;

   if (gettimeofday(&tv, NULL) < 0)
      return LIBUSB_ERROR_OTHER;
   ts->tv_sec = tv.tv_sec;
   ts->tv_nsec = tv.tv_usec * 1000;
   return(LIBUSB_SUCCESS);
}

int
os2_clock_gettime(int clkid, struct timespec *tp)
{
   usbi_dbg("clock %d", clkid);

   if (clkid == USBI_CLOCK_REALTIME)
      return(clock_gettime(CLOCK_REALTIME, tp));

   if (clkid == USBI_CLOCK_MONOTONIC)
      return(clock_gettime(CLOCK_MONOTONIC, tp));

   return(LIBUSB_ERROR_INVALID_PARAM);
}

int
_apiret_to_libusb(int err)
{
   usbi_dbg("error: %d", err);

   switch (err) {
   case NO_ERROR:
      return(LIBUSB_SUCCESS);
   case USB_IORB_FAILED:
      return(LIBUSB_ERROR_IO);
   case ERROR_INVALID_PARAMETER:
      return(LIBUSB_ERROR_INVALID_PARAM);
   case ERROR_DEV_NOT_EXIST:
      return(LIBUSB_ERROR_NO_DEVICE);
   case ERROR_NOT_ENOUGH_MEMORY:
      return(LIBUSB_ERROR_NO_MEM);
   case ERROR_TIMEOUT:
      return(LIBUSB_ERROR_TIMEOUT);
   }

   return(LIBUSB_ERROR_OTHER);
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
      return(LIBUSB_ERROR_IO);
   }

   itransfer->transferred = setup->wLength; /* not right, should be length actually transferred */
   usbi_dbg("transferred %d", itransfer->transferred);
   return(LIBUSB_SUCCESS);
}


int
_sync_bulk_transfer(struct usbi_transfer *itransfer)
{
   usbi_dbg("");

   struct libusb_transfer *transfer;
   struct device_priv *dpriv;
   transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   dpriv = (struct device_priv *)transfer->dev_handle->dev->os_priv;
   APIRET    rc = LIBUSB_SUCCESS;
   int nr = transfer->length;

   usbi_dbg("_sync_bulk_transfer nr = %d",nr);

   if (IS_XFERIN(transfer)) {
      usbi_dbg("_sync_bulk_transfer - UsbBulkRead");

      rc = UsbBulkRead( (USBHANDLE) dpriv->fd, (UCHAR)transfer->endpoint, 0,
         (PULONG)&nr, transfer->buffer, (ULONG)transfer->timeout);
      usbi_dbg("usbcalls UsbBulkRead with dh = %p",dpriv->fd);
      usbi_dbg("usbcalls UsbBulkRead with bulk_in_ep = 0x%02x",transfer->endpoint);
      usbi_dbg("usbcalls UsbBulkRead with usbcalls_timeout = %d",transfer->timeout);
      usbi_dbg("usbcalls UsbBulkRead nr2 = %d",nr);
      usbi_dbg("usbcalls UsbBulkRead rc = %d",rc);

      if (rc == ERROR_TIMEOUT) {
         usbi_dbg("UsbBulkRead timeout");
         nr = -ETIMEDOUT;
      }
      if (rc && rc != USB_ERROR_LESSTRANSFERED) {
         usbi_dbg( "unable to read from bulk endpoint - size= %d  nr= %d  rc= %x",
            transfer->length, nr, (int)rc);
         nr = -1;
      }

   } else {
      usbi_dbg("_sync_bulk_transfer - UsbBulkWrite");

      rc = UsbBulkWrite( (USBHANDLE) dpriv->fd, (UCHAR)transfer->endpoint, 0,
         (ULONG)nr, transfer->buffer, (ULONG)transfer->timeout);
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
      return(_apiret_to_libusb(rc));

   itransfer->transferred = nr;
   usbi_dbg("itransfer->transferred = %d, nr =%d",itransfer->transferred, nr);
   return(LIBUSB_SUCCESS);
}

int
_sync_irq_transfer(struct usbi_transfer *itransfer)
{
   usbi_dbg("");

   struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   struct device_priv *dpriv        = (struct device_priv *)transfer->dev_handle->dev->os_priv;
   APIRET    rc = LIBUSB_SUCCESS;
   int nr = transfer->length;

   usbi_dbg("_sync_irq_transfer nr = %d",nr);

   if (IS_XFERIN(transfer)) {
      usbi_dbg("_sync_irq_transfer - UsbIrqRead");

      rc = UsbIrqRead( (USBHANDLE) dpriv->fd, (UCHAR)transfer->endpoint, 0,
         (PULONG)&nr, transfer->buffer, (ULONG)transfer->timeout);
      usbi_dbg("usbcalls UsbIrqRead with dh = %p",dpriv->fd);
      usbi_dbg("usbcalls UsbIrqRead with interrupt_in_ep = 0x%02x",transfer->endpoint);
      usbi_dbg("usbcalls UsbIrqRead with usbcalls_timeout = %d",transfer->timeout);
      usbi_dbg("usbcalls UsbIrqRead nr2 = %d",nr);
      usbi_dbg("usbcalls UsbIrqRead rc = %d",rc);

      if (rc == ERROR_TIMEOUT) {
         usbi_dbg("UsbIrqRead timeout");
         nr = -ETIMEDOUT;
      }
      if (rc && rc != USB_ERROR_LESSTRANSFERED) {
         usbi_dbg( "unable to read from interrupt endpoint - size= %d  nr= %d  rc= %x",
            transfer->length, nr, (int)rc);
         nr = -1;
      }

   } else {
      usbi_dbg("_sync_irq_transfer - UsbIrqWrite");

      rc = UsbIrqWrite( (USBHANDLE) dpriv->fd, (UCHAR)transfer->endpoint, 0,
         (ULONG)nr, transfer->buffer, (ULONG)transfer->timeout);
      usbi_dbg("usbcalls UsbIrqWrite with dh = %p",dpriv->fd);
      usbi_dbg("usbcalls UsbIrqWrite with interrupt_out_ep = 0x%02x",transfer->endpoint);
      usbi_dbg("usbcalls UsbIrqWrite with usbcalls_timeout = %d",transfer->timeout);
      usbi_dbg("usbcalls UsbIrqWrite rc = %d",rc);
      if (rc) {
         usbi_dbg( "unable to write to interrupt endpoint - size= %d  nr= %d  rc= %x",
            transfer->length, nr, (int)rc);
         nr = -1;
      }
   }

   if (nr < 0)
      return(_apiret_to_libusb(rc));

   itransfer->transferred = nr;
   usbi_dbg("itransfer->transferred = %d, nr =%d",itransfer->transferred, nr);
   return(LIBUSB_SUCCESS);
}

int
_sync_iso_transfer(struct usbi_transfer *itransfer)
{
   struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   struct device_priv *dpriv        = (struct device_priv *)transfer->dev_handle->dev->os_priv;
   APIRET rc = NO_ERROR;
   HEV    hIsoComplete=0;
   unsigned int i = 0;
   unsigned int packet_len = 0;
   unsigned int num_packets = transfer->num_iso_packets;
   PUSBCALLS_MY_ISO_RSP pIsoResponse = NULL;

   itransfer->transferred = 0;

   pIsoResponse = (PUSBCALLS_MY_ISO_RSP)malloc(sizeof(USBCALLS_MY_ISO_RSP)+ num_packets * sizeof(unsigned short));
   if (!pIsoResponse) {
      return(LIBUSB_ERROR_NO_MEM);
   } /* endif */

   rc = DosCreateEventSem(NULL,&hIsoComplete,DC_SEM_SHARED,FALSE);

   for (i=0;i<num_packets ;i++) {
      packet_len = transfer->iso_packet_desc[i].length;
      pIsoResponse->usFrameSize[i] = (USHORT)packet_len;
   } /* endfor */
   pIsoResponse->usStatus = 0;
   pIsoResponse->usDataLength = (USHORT)transfer->length;

   rc = UsbIsoOpen((USBHANDLE)dpriv->fd,(UCHAR)transfer->endpoint,(UCHAR)dpriv->altsetting,(USHORT)num_packets,0);

   rc = UsbStartIsoTransfer((USBHANDLE)dpriv->fd,(UCHAR)transfer->endpoint,(UCHAR)dpriv->altsetting,(ULONG)hIsoComplete,(PUCHAR)pIsoResponse,(PUCHAR)transfer->buffer,0,(USHORT)num_packets);
   if (!rc) {
      rc = DosWaitEventSem(hIsoComplete,(ULONG)transfer->timeout);
      if (!rc) {
         for(i=0;i<num_packets;i++) {
            itransfer->transferred                    += pIsoResponse->usFrameSize[i];
            transfer->iso_packet_desc[i].actual_length = pIsoResponse->usFrameSize[i];
            transfer->iso_packet_desc[i].status        = (0 == pIsoResponse->usStatus) ? LIBUSB_TRANSFER_COMPLETED : LIBUSB_TRANSFER_ERROR;
         }
      } else {
         UsbCancelTransfer((USBHANDLE)dpriv->fd,(UCHAR)transfer->endpoint,(UCHAR)dpriv->altsetting,(ULONG)hIsoComplete);
      } /* endif */
   } /* endif */

   rc = UsbIsoClose((USBHANDLE)dpriv->fd,(UCHAR)transfer->endpoint,(UCHAR)dpriv->altsetting);

   rc = DosCloseEventSem(hIsoComplete);

   free((void *)pIsoResponse);

   return(LIBUSB_SUCCESS);
}

/* The 3 functions below are unlikely to ever get supported on OS/2 */
static int os2_kernel_driver_active(struct libusb_device_handle *dev_handle, int iface)
{
   return(LIBUSB_ERROR_NOT_SUPPORTED);
}

static int os2_attach_kernel_driver(struct libusb_device_handle *dev_handle, int iface)
{
   return(LIBUSB_ERROR_NOT_SUPPORTED);
}

static int os2_detach_kernel_driver(struct libusb_device_handle *dev_handle, int iface)
{
   return(LIBUSB_ERROR_NOT_SUPPORTED);
}

