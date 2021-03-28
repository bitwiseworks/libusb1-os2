/*
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

#include "libusb.h"
#include "libusbi.h"
#include "os2_usb.h"

/*
 * Backend functions
 */
static int os2_get_device_list(struct libusb_context *,
    struct discovered_devs **);
static int os2_open(struct libusb_device_handle *);
static void os2_close(struct libusb_device_handle *);

static int os2_get_active_config_descriptor(struct libusb_device *,
    void *, size_t);
static int os2_get_config_descriptor(struct libusb_device *, uint8_t,
    void *, size_t);

static int os2_get_configuration(struct libusb_device_handle *, uint8_t *);
static int os2_set_configuration(struct libusb_device_handle *, int);

static int os2_claim_interface(struct libusb_device_handle *, uint8_t);
static int os2_release_interface(struct libusb_device_handle *, uint8_t);

static int os2_set_interface_altsetting(struct libusb_device_handle *, uint8_t,
    uint8_t);
static int os2_clear_halt(struct libusb_device_handle *, unsigned char);
static int os2_reset_device(struct libusb_device_handle *);
static void os2_destroy_device(struct libusb_device *);

static int os2_submit_transfer(struct usbi_transfer *);
static int os2_cancel_transfer(struct usbi_transfer *);
static void os2_clear_transfer_priv(struct usbi_transfer *);
static int os2_handle_transfer_completion(struct usbi_transfer *);
static int os2_kernel_driver_active(struct libusb_device_handle *dev_handle, uint8_t iface);
static int os2_attach_kernel_driver(struct libusb_device_handle *dev_handle, uint8_t iface);
static int os2_detach_kernel_driver(struct libusb_device_handle *dev_handle, uint8_t iface);

/*
 * Private functions
 */
static int _is_streaming_interface(struct libusb_device *dev,uint8_t iface);
static void _call_iso_close(struct libusb_device *dev);
static int _interface_for_endpoint(struct libusb_device *dev,uint8_t endpoint);
static int _apiret_to_libusb(ULONG);
static int _sync_control_transfer(struct usbi_transfer *);
static int _sync_bulk_transfer(struct usbi_transfer *);
static int _sync_irq_transfer(struct usbi_transfer *);
static int _sync_iso_transfer(struct usbi_transfer *);


const struct usbi_os_backend usbi_backend = {
   "Synchronous OS/2 backend",
   0,
   NULL,          /* init() */
   NULL,          /* exit() */
   NULL,          /* set_option() TODO TODO TODO */
   os2_get_device_list,
   NULL,          /* hotplug_poll() */
   NULL,          /* wrap_sys_device() TODO TODO TODO */
   os2_open,
   os2_close,

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

   NULL,                         /* handle_events() */
   os2_handle_transfer_completion,

   0,                            /* context private data */
   sizeof(struct device_priv),   /* device private data */
   0,                            /* handle private data */
   sizeof(struct transfer_priv)  /* transfer private data */
};

static int
os2_get_device_list(struct libusb_context * ctx,
   struct discovered_devs **discdevs)
{
   APIRET    rc;
   ULONG     cntDev;
   ULONG     ctr;
   ULONG     len;
   struct libusb_device *dev;
   struct device_priv *dpriv;

   unsigned char scratchBuf[LIBUSB_DT_DEVICE_SIZE+4096];
   GETDEVINFODATA info;


   usbi_dbg(" ");

   rc = UsbQueryNumberDevices( &cntDev);
   if (rc) {
      usbi_dbg( "unable to query number of USB devices - rc= %x", (int)rc);
      return(LIBUSB_ERROR_IO);
   }
   usbi_dbg( "%lu devices detected", cntDev);
   for (ctr = 1; ctr <= cntDev; ctr++)
   {
      len = sizeof(info);
      rc = UsbQueryDeviceInfo( ctr,&len, (PUCHAR)&info);
      if (rc) {
         usbi_dbg( "unable to query device info - device= %d  rc= %x",   (int)ctr, (int)rc);
         return(LIBUSB_ERROR_IO);
      }

      len = sizeof(scratchBuf);
      rc = UsbQueryDeviceReport( ctr, &len, scratchBuf);
      if (rc) {
         usbi_dbg( "unable to query device report - device= %d  rc= %x",   (int)ctr, (int)rc);
         return(LIBUSB_ERROR_IO);
      }

      dev = usbi_get_device_by_session_id(ctx, info.rmDevHandle);
      if (dev == NULL) {
         dev = usbi_alloc_device(ctx, info.rmDevHandle);
         if (dev == NULL)
            return(LIBUSB_ERROR_NO_MEM);

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

         memcpy(&dev->device_descriptor,scratchBuf, LIBUSB_DT_DEVICE_SIZE);
         usbi_localize_device_descriptor(&dev->device_descriptor);

         dpriv = (struct device_priv *)usbi_get_device_priv(dev);
         dpriv->fd = -1;
         dpriv->rmDevHandle = info.rmDevHandle;     /* under OS/2, the Resource Manager device handle is a GUID, well suited to track a device */
         memset(dpriv->altsetting,0,sizeof(dpriv->altsetting));
         memset(dpriv->endpoint,0,sizeof(dpriv->endpoint));
         memcpy(dpriv->cdesc, scratchBuf + LIBUSB_DT_DEVICE_SIZE, (len - LIBUSB_DT_DEVICE_SIZE));
         if (LIBUSB_SUCCESS != libusb_get_config_descriptor(dev,0,&dpriv->curr_config_descriptor))
         {
             dpriv->curr_config_descriptor = NULL;
         }
         if (usbi_sanitize_device(dev)) {
            libusb_unref_device(dev);
            continue;
         }
      }
      if (discovered_devs_append(*discdevs, dev) == NULL)
         return(LIBUSB_ERROR_NO_MEM);

      libusb_unref_device(dev);
   }
   return(LIBUSB_SUCCESS);
}

static int
os2_open(struct libusb_device_handle *handle)
{
   struct libusb_device *dev = handle->dev;
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(dev);
   APIRET    rc = NO_ERROR;
   int       usbhandle;

   if (dev->refcnt == 2)
   {
      rc = UsbOpen( (PUSBHANDLE)&usbhandle,
         (USHORT)dev->device_descriptor.idVendor,
         (USHORT)dev->device_descriptor.idProduct,
         (USHORT)dev->device_descriptor.bcdDevice,
         (USHORT)USB_OPEN_FIRST_UNUSED);

      if (rc)
      {
         usbi_dbg( "unable to open device - id= %x:%x  rc= %x",
            dev->device_descriptor.idVendor,
            dev->device_descriptor.idProduct, (int)rc);
         dpriv->fd = -1;
         return( _apiret_to_libusb(rc));
      }

      dpriv->fd = usbhandle;

      /* set device configuration to 1st configuration */
      rc = UsbDeviceSetConfiguration (dpriv->fd,1);
      usbi_dbg("UsbDeviceSetConfiguration: rc = %lu", rc);
   } /* endif */

   usbi_dbg("fd %d", dpriv->fd);

   return(LIBUSB_SUCCESS);
}

static void
os2_close(struct libusb_device_handle *handle)
{
   struct libusb_device *dev = handle->dev;
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(dev);
   APIRET    rc;

   usbi_dbg("close: fd %d", dpriv->fd);

   if (dev->refcnt == 2) {
      rc = UsbClose( (USBHANDLE)dpriv->fd);
      if (rc) {
         usbi_dbg( "unable to close device - id= %x/%x  handle= %x  rc= %d",
             dev->device_descriptor.idVendor,
             dev->device_descriptor.idProduct,
             dpriv->fd, (int)rc);
      }

      dpriv->fd = -1;
   } /* endif */
}

static int
os2_get_active_config_descriptor(struct libusb_device *dev,
    void *buf, size_t len)
{
   struct device_priv *dpriv            = (struct device_priv *)usbi_get_device_priv(dev);
   struct libusb_config_descriptor *ucd = (struct libusb_config_descriptor*) (dpriv->cdesc);

   len = MIN(len, ucd->wTotalLength);

   usbi_dbg("len %d", len);

   memcpy(buf, dpriv->cdesc, len);

   return(len);
}

static int
os2_get_config_descriptor(struct libusb_device *dev, uint8_t idx,
    void *buf, size_t len)
{
   UNUSED(idx);
   /* just return the active config descriptor, it's a bad idea to arbitrarily switch configuration anyway */
   return(os2_get_active_config_descriptor(dev,buf,len));
}

/*
 * as requested in the function description in libusbi.h, make sure we avoid I/O to get the configuration value
 * fortunately, USBD.SYS will internally save the selected configuration which we query here via
 * UsbQueryDeviceInfo
 */
static int
os2_get_configuration(struct libusb_device_handle *handle, uint8_t *config)
{
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(handle->dev);
   ULONG cntDev,ctr,len;
   APIRET rc = LIBUSB_SUCCESS;
   GETDEVINFODATA info;

   usbi_dbg(" ");

   *config = 0;

   rc = UsbQueryNumberDevices( &cntDev);
   if (rc) {
      usbi_dbg( "unable to query number of USB devices - rc= %x", (int)rc);
      return(LIBUSB_ERROR_IO);
   }
   for (ctr = 1; ctr <= cntDev; ctr++) {
      len = sizeof(info);
      rc = UsbQueryDeviceInfo( ctr,&len, (PUCHAR)&info);
      if (rc) {
         usbi_dbg( "unable to query device info -  rc= %x", (int)rc);
         return(LIBUSB_ERROR_IO);
      } /* endif */
      if (info.rmDevHandle == dpriv->rmDevHandle) {
         *config = info.bConfigurationValue;
         return(LIBUSB_SUCCESS);
      } /* endif */
   }

   return(LIBUSB_ERROR_NO_DEVICE);
}

static int
os2_set_configuration(struct libusb_device_handle *handle, int config)
{
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(handle->dev);
   APIRET    rc = LIBUSB_SUCCESS;

   usbi_dbg("configuration %d", config);

   rc = UsbDeviceSetConfiguration( (USBHANDLE)dpriv->fd, (USHORT)config);
   if (rc) {
      usbi_dbg( "unable to set configuration - config= %d  rc= %x",
         config, (int)rc);
      return(LIBUSB_ERROR_IO);
   }

   return(LIBUSB_SUCCESS);
}

static int
os2_claim_interface(struct libusb_device_handle *handle, uint8_t iface)
{
   UNUSED(handle);
   UNUSED(iface);
/* USBRESM$ appears to handle this as part of opening the device */
   usbi_dbg(" ");
   return(LIBUSB_SUCCESS);
}

static int
os2_release_interface(struct libusb_device_handle *handle, uint8_t iface)
{
/* USBRESM$ appears to handle this as part of opening the device */
   usbi_dbg(" ");

   APIRET rc     = NO_ERROR;
   int errorcode = LIBUSB_SUCCESS;

   struct libusb_device *dev = handle->dev;
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(dev);
   if (_is_streaming_interface(dev,iface) && dpriv->endpoint[iface] && dpriv->altsetting[iface])
   {
      rc = UsbInterfaceSetAltSetting((USBHANDLE)dpriv->fd,(USHORT)iface,(USHORT)0);
      if (NO_ERROR != rc) {
         usbi_dbg("UsbInterfaceSetAltSetting failed for if %#02x with alt 0, apiret: %lu",iface,rc);
         errorcode = _apiret_to_libusb(rc);
      }
      dpriv->endpoint[iface] = 0;
      dpriv->altsetting[iface] = 0;
   }
   return(errorcode);
}

static int
os2_set_interface_altsetting(struct libusb_device_handle *handle, uint8_t iface,
    uint8_t altsetting)
{
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(handle->dev);
   APIRET rc = NO_ERROR;

   usbi_dbg(" ");
   if (dpriv->altsetting[iface] != altsetting)
   {
      rc = UsbInterfaceSetAltSetting((USBHANDLE)dpriv->fd,(USHORT)iface,(USHORT)altsetting);
      if (NO_ERROR != rc) {
         usbi_dbg("UsbInterfaceSetAltSetting cannot set alternate setting, apiret:%lu",rc);
         return(_apiret_to_libusb(rc));
      }
      dpriv->altsetting[iface] = altsetting;
      usbi_dbg("UsbInterfaceSetAltSetting interface %u set to alternate setting: %u",iface,altsetting);
   }
   return (LIBUSB_SUCCESS);
}

static int
os2_clear_halt(struct libusb_device_handle *handle, unsigned char endpoint)
{
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(handle->dev);
   APIRET rc = LIBUSB_SUCCESS;

   usbi_dbg(" ");
   rc = UsbEndpointClearHalt((USBHANDLE)dpriv->fd,(USHORT)endpoint);
   return(_apiret_to_libusb(rc));
}

static int
os2_reset_device(struct libusb_device_handle *handle)
{
/* TO DO */
   UNUSED(handle);

   usbi_dbg(" ");
   return(LIBUSB_ERROR_NOT_SUPPORTED);
}

static void
os2_destroy_device(struct libusb_device *dev)
{
   usbi_dbg(" ");

   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(dev);

   _call_iso_close(dev);
   libusb_free_config_descriptor(dpriv->curr_config_descriptor);
   dpriv->curr_config_descriptor = NULL;
}

static int
os2_submit_transfer(struct usbi_transfer *itransfer)
{
   struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   int err = 0;

   usbi_dbg(" ");

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

//   if (err)
//   {
//       return(err);
//   }
   //if (err)
   //{
   //    transfer->status = LIBUSB_TRANSFER_ERROR;
   //}
   //else
   //{
   //    transfer->status = LIBUSB_TRANSFER_COMPLETED;
   //}

   usbi_signal_transfer_completion(itransfer);

   return(LIBUSB_SUCCESS);
}

static int
os2_cancel_transfer(struct usbi_transfer *itransfer)
{
   UNUSED(itransfer);
   usbi_dbg(" ");

   return(LIBUSB_SUCCESS);
}

static void
os2_clear_transfer_priv(struct usbi_transfer *itransfer)
{
   UNUSED(itransfer);

   usbi_dbg(" ");
}

static int
os2_handle_transfer_completion(struct usbi_transfer *itransfer)
{
   //struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);

   //return(usbi_handle_transfer_completion(itransfer, transfer->status));

   return(usbi_handle_transfer_completion(itransfer, LIBUSB_TRANSFER_COMPLETED));
}


static int _is_streaming_interface(struct libusb_device *dev, uint8_t iface)
{
   struct device_priv *dpriv               = (struct device_priv *)usbi_get_device_priv(dev);
   struct libusb_config_descriptor *config    = dpriv->curr_config_descriptor;
   int foundStreamingInterface       = 0;
   int i=0,a=0,e=0;

   for (i=0;i<config->bNumInterfaces;i++) {
      for (a=0;a<config->interface[i].num_altsetting;a++) {
         for (e=0;e<config->interface[i].altsetting[a].bNumEndpoints;e++) {
            if ((config->interface[i].altsetting[a].bInterfaceNumber == iface) &&
                ((config->interface[i].altsetting[a].bInterfaceClass  == 1) || (config->interface[i].altsetting[a].bInterfaceClass == 14)) &&
                (config->interface[i].altsetting[a].bInterfaceSubClass == 2) &&
                (config->interface[i].altsetting[a].endpoint[e].bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
                   foundStreamingInterface = 1;
                   goto leave;
            } /* endif */
         } /* endfor */
      } /* endfor */
   } /* endfor */
leave:
   return(foundStreamingInterface);
}

static void _call_iso_close(struct libusb_device *dev)
{
   struct device_priv *dpriv               = (struct device_priv *)usbi_get_device_priv(dev);
   struct libusb_config_descriptor *config    = dpriv->curr_config_descriptor;
   APIRET rc = NO_ERROR;
   int i=0,a=0,e=0;

   for (i=0;i<config->bNumInterfaces;i++) {
      for (a=0;a<config->interface[i].num_altsetting;a++) {
         for (e=0;e<config->interface[i].altsetting[a].bNumEndpoints;e++) {
            if (((config->interface[i].altsetting[a].bInterfaceClass  == 1) || (config->interface[i].altsetting[a].bInterfaceClass == 14)) &&
                (config->interface[i].altsetting[a].bInterfaceSubClass == 2) &&
                (config->interface[i].altsetting[a].endpoint[e].bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
                   if (dpriv->endpoint[i] && dpriv->altsetting[i])
                   {
                      rc = UsbInterfaceSetAltSetting((USBHANDLE)dpriv->fd,(USHORT)i,(USHORT)0);
                      if (NO_ERROR != rc) {
                         usbi_dbg("UsbInterfaceSetAltSetting failed for if %#02x with alt 0, apiret: %lu",i,rc);
                      }
                      dpriv->endpoint[i] = 0;
                      dpriv->altsetting[i] = 0;
                   } /* endif */
            } /* endif */
         } /* endfor */
      } /* endfor */
   } /* endfor */
   return;
}


static int _interface_for_endpoint(struct libusb_device *dev,uint8_t endpoint)
{
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(dev);
   struct libusb_config_descriptor *config    = dpriv->curr_config_descriptor;
   int interface                              = -1;
   int i=0,a=0,e=0;

   for (i=0;i<config->bNumInterfaces;i++) {
      for (a=0;a<config->interface[i].num_altsetting;a++) {
         for (e=0;e<config->interface[i].altsetting[a].bNumEndpoints;e++) {
            if (((config->interface[i].altsetting[a].bInterfaceClass  == 1) || (config->interface[i].altsetting[a].bInterfaceClass == 14)) &&
                (config->interface[i].altsetting[a].bInterfaceSubClass == 2) &&
                (config->interface[i].altsetting[a].endpoint[e].bEndpointAddress == endpoint)) {
               interface = i;
               goto leave;
            } /* endif */
         } /* endfor */
      } /* endfor */
   } /* endfor */
leave:
   return(interface);
}



static int
_apiret_to_libusb(ULONG err)
{
   usbi_dbg("error: %lu", err);

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

static int
_sync_control_transfer(struct usbi_transfer *itransfer)
{
   usbi_dbg(" ");

   struct libusb_transfer *transfer;
   struct libusb_control_setup *setup;
   struct device_priv *dpriv;
   APIRET    rc;

   transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   dpriv = (struct device_priv *)usbi_get_device_priv(transfer->dev_handle->dev);
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


static int
_sync_bulk_transfer(struct usbi_transfer *itransfer)
{
   usbi_dbg(" ");

   struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   struct device_priv *dpriv        = (struct device_priv *)usbi_get_device_priv(transfer->dev_handle->dev);
   APIRET    rc = LIBUSB_SUCCESS;
   int nr = transfer->length;

   usbi_dbg("nr = %d",nr);

   if (IS_XFERIN(transfer)) {
      usbi_dbg("UsbBulkRead");

      rc = UsbBulkRead( (USBHANDLE) dpriv->fd, (UCHAR)transfer->endpoint, 0,
         (PULONG)&nr, transfer->buffer, (ULONG)transfer->timeout);
      usbi_dbg("UsbBulkRead with dh = %u",(unsigned int)dpriv->fd);
      usbi_dbg("UsbBulkRead with bulk_in_ep = %#02x",transfer->endpoint);
      usbi_dbg("UsbBulkRead with usbcalls_timeout = %d",transfer->timeout);
      usbi_dbg("UsbBulkRead nr = %d",nr);
      usbi_dbg("UsbBulkRead rc = %lu",rc);

      if (rc == ERROR_TIMEOUT) {
         usbi_dbg("UsbBulkRead timeout");
         nr = -ETIMEDOUT;
      }
      if (rc && rc != USB_ERROR_LESSTRANSFERED) {
         usbi_dbg( "unable to read from bulk endpoint - size= %d  nr= %d  rc= %#x",
            transfer->length, nr, (int)rc);
         nr = -1;
      }

   } else {
      usbi_dbg("UsbBulkWrite");

      rc = UsbBulkWrite( (USBHANDLE) dpriv->fd, (UCHAR)transfer->endpoint, 0,
         (ULONG)nr, transfer->buffer, (ULONG)transfer->timeout);
      usbi_dbg("UsbBulkWrite with dh = %u",(unsigned int)dpriv->fd);
      usbi_dbg("UsbBulkWrite with bulk_out_ep = %#02x",transfer->endpoint);
      usbi_dbg("UsbBulkWrite with usbcalls_timeout = %d",transfer->timeout);
      usbi_dbg("UsbBulkWrite rc = %lu",rc);
      if (rc) {
         usbi_dbg( "unable to write to bulk endpoint - size= %d  nr= %d  rc= %#x",
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

static int
_sync_irq_transfer(struct usbi_transfer *itransfer)
{
   usbi_dbg(" ");

   struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   struct device_priv *dpriv        = (struct device_priv *)usbi_get_device_priv(transfer->dev_handle->dev);
   APIRET    rc = LIBUSB_SUCCESS;
   int nr = transfer->length;

   usbi_dbg("nr = %d",nr);

   if (IS_XFERIN(transfer)) {
      usbi_dbg("UsbIrqRead");

      rc = UsbIrqRead( (USBHANDLE) dpriv->fd, (UCHAR)transfer->endpoint, 0,
         (PULONG)&nr, transfer->buffer, (ULONG)transfer->timeout);
      usbi_dbg("UsbIrqRead with dh = %u",(unsigned int)dpriv->fd);
      usbi_dbg("UsbIrqRead with interrupt_in_ep = %#02x",transfer->endpoint);
      usbi_dbg("UsbIrqRead with usbcalls_timeout = %d",transfer->timeout);
      usbi_dbg("UsbIrqRead nr = %d",nr);
      usbi_dbg("UsbIrqRead rc = %lu",rc);

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
      usbi_dbg("UsbIrqWrite");

      rc = UsbIrqWrite( (USBHANDLE) dpriv->fd, (UCHAR)transfer->endpoint, 0,
         (ULONG)nr, transfer->buffer, (ULONG)transfer->timeout);
      usbi_dbg("UsbIrqWrite with dh = %u",(unsigned int)dpriv->fd);
      usbi_dbg("UsbIrqWrite with interrupt_out_ep = %#02x",transfer->endpoint);
      usbi_dbg("UsbIrqWrite with usbcalls_timeout = %d",transfer->timeout);
      usbi_dbg("UsbIrqWrite rc = %lu",rc);
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

static int
_sync_iso_transfer(struct usbi_transfer *itransfer)
{
   usbi_dbg(" ");

   struct transfer_priv *tpriv      = (struct transfer_priv *)usbi_get_transfer_priv(itransfer);
   struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   struct libusb_device *dev        = transfer->dev_handle->dev;
   struct device_priv *dpriv        = (struct device_priv *)usbi_get_device_priv(dev);
   APIRET rc = NO_ERROR;
   unsigned int i = 0, j = 0;
   unsigned int packet_len = 0;
   unsigned int packet_index = 0;
   unsigned int num_max_packets_per_execution = 0;
   unsigned int num_max_executions = 0;
   unsigned int num_remaining_packets = 0;
   unsigned char *buffer = NULL;
   int length = 0;
   PUSBCALLS_MY_ISO_RSP pIsoResponse = &tpriv->IsoResponse;
   ULONG postCount = 0;
   int iface = 0;
   int errorcode = LIBUSB_SUCCESS;
   enum libusb_transfer_status status = LIBUSB_TRANSFER_ERROR;
   HEV hTransferSem = 0U;


   usbi_dbg("dev handle: %p, nr = %u, num packets = %u",transfer->dev_handle,transfer->length,transfer->num_iso_packets);


   itransfer->transferred = 0;
//   transfer->status = LIBUSB_TRANSFER_ERROR;

   do
   {

      if (transfer->num_iso_packets > MAX_NUM_ISO_PACKETS)
      {
         usbi_dbg("number of iso packets exceeds supported limit, num packets %u",transfer->num_iso_packets);
         errorcode = LIBUSB_ERROR_INVALID_PARAM;
         break;
      }

      if (transfer->num_iso_packets)
      {
         for (i=0,length=0;i<(unsigned int)transfer->num_iso_packets;i++,length += packet_len) {
         packet_len = transfer->iso_packet_desc[i].length;
         transfer->iso_packet_desc[i].actual_length = 0;
         transfer->iso_packet_desc[i].status        = LIBUSB_TRANSFER_ERROR;
         } /* endfor */
         if (transfer->length < length) {
            usbi_dbg("overall transfer length (%u) < sum packet lengths (%u)",transfer->length, length);
            errorcode = LIBUSB_ERROR_INVALID_PARAM;
            break;
         }
         /* endif */
         packet_len = transfer->iso_packet_desc[0].length;
      }
      else
      {
          length = transfer->length;
          packet_len = length;
      }

      iface = _interface_for_endpoint(dev,transfer->endpoint);
      if (iface < 0) {
         usbi_dbg("endpoint %#02x not associated with streaming interface",transfer->endpoint);
         errorcode = LIBUSB_ERROR_INVALID_PARAM;
         break;
      } /* endif */

      if (!dpriv->altsetting[iface]) {
         usbi_dbg("cannot do isochronous transfer with altsetting = 0");
         errorcode = LIBUSB_ERROR_INVALID_PARAM;
         break;
      }
      dpriv->endpoint[iface] = transfer->endpoint;

      /*
       * we HAVE TO protect per device the call to UsbIsoOpen !
       * UsbIsoOpen allocates bandwidth for the given altsetting/endpoint combination
       * this is a DEVICE property that has to be maintained across the call to
       * UsbStartIsoTransfer. Since libusb can call this function from more thread
       * than one, we need to serialize the access at this point with the per-device
       * mutex
       */
      usbi_mutex_lock(&dev->lock);

      rc = UsbIsoOpen((USBHANDLE)dpriv->fd,
                         (UCHAR)transfer->endpoint,
                         (UCHAR)dpriv->altsetting[iface],
                         2U, /* number of HC internal ISO buffer management structures */
                         (USHORT)packet_len);
      usbi_dbg("UsbIsoOpen for if %#02x, alt %#02x, ep %#02x, apiret:%lu",iface,dpriv->altsetting[iface],transfer->endpoint,rc);
      if (NO_ERROR != rc) {
         errorcode = _apiret_to_libusb(rc);
         goto cleanup2;
      }

      rc = DosCreateEventSem(NULL,&hTransferSem,DC_SEM_SHARED,FALSE);
      if (NO_ERROR != rc) {
         usbi_dbg("hTransferSem: DosCreateEventSem failed, apiret: %lu",rc);
         errorcode = LIBUSB_ERROR_OTHER;
         goto cleanup;
      }

      num_max_packets_per_execution = 65535U / packet_len;
      /*
       * IMPORTANT: for every invocation of UsbStartIsoTransfer in conjunction with EHCI, the number of packets
       * has to be a multiple of 8 !!!
       * for UHCI/OHCI this does not matter but it's also not wrong
       */
      num_max_packets_per_execution = (num_max_packets_per_execution / 8U) * 8U;
      num_max_executions = length / (num_max_packets_per_execution * packet_len);
      num_remaining_packets = transfer->num_iso_packets - (num_max_packets_per_execution * num_max_executions);


      for (i=0,buffer = transfer->buffer,packet_index = 0;i<num_max_executions;i++,buffer += packet_len*num_max_packets_per_execution,packet_index += num_max_packets_per_execution)
      {
         unsigned int tmp=0;
         for (j=0,length=0;j<num_max_packets_per_execution ;j++,length += tmp)
         {
            tmp = transfer->iso_packet_desc[packet_index+j].length;
            pIsoResponse->usFrameSize[j] = (USHORT)tmp;
         } /* endfor */
         pIsoResponse->usStatus = 0;
         pIsoResponse->usDataLength = (USHORT)length;

         rc = UsbStartIsoTransfer((USBHANDLE)dpriv->fd,
                                  (UCHAR)transfer->endpoint,
                                  (UCHAR)dpriv->altsetting[iface],
                                  (ULONG)hTransferSem,
                                  (PUCHAR)pIsoResponse,
                                  (PUCHAR)buffer,
                                  (USHORT)packet_len,
                                  (USHORT)num_max_packets_per_execution);
         usbi_dbg("UsbStartIsoTransfer with dh = %u, pass = %u",(unsigned int)dpriv->fd,i);
         usbi_dbg("UsbStartIsoTransfer with iso ep = %#02x",transfer->endpoint);
         usbi_dbg("UsbStartIsoTransfer with usbcalls_timeout = %d",transfer->timeout);
         usbi_dbg("UsbStartIsoTransfer num iso packets %u, packet len %u, buffer len %u",num_max_packets_per_execution,packet_len,length);
         usbi_dbg("UsbStartIsoTransfer rc = %lu",rc);

         if (!rc) {
            rc = DosWaitEventSem(hTransferSem,(ULONG)transfer->timeout);
            if (!rc)
            {
               status = (0 == pIsoResponse->usStatus) ? LIBUSB_TRANSFER_COMPLETED : LIBUSB_TRANSFER_ERROR;
               for(j=0;j<num_max_packets_per_execution;j++) {
                  itransfer->transferred                                 += pIsoResponse->usFrameSize[j];
                  transfer->iso_packet_desc[packet_index+j].actual_length = pIsoResponse->usFrameSize[j];
                  transfer->iso_packet_desc[packet_index+j].status        = status;
               }
            } else {
               usbi_dbg("DosWaitEventSem failed, apiret: %lu",rc);
               rc = UsbCancelTransfer((USBHANDLE)dpriv->fd,(UCHAR)transfer->endpoint,(UCHAR)dpriv->altsetting[iface],(ULONG)hTransferSem);
                    DosWaitEventSem(hTransferSem,(ULONG)transfer->timeout);
               if (NO_ERROR != rc) {
                  usbi_dbg("UsbCancelTransfer failed, apiret: %lu",rc);
               }
               else
               {
                  rc = DosWaitEventSem(hTransferSem,(ULONG)transfer->timeout);
               }
               errorcode = _apiret_to_libusb(rc);
            }
            rc = DosResetEventSem(hTransferSem,&postCount);
            if (ERROR_ALREADY_RESET != rc && NO_ERROR != rc) {
               usbi_dbg("DosResetEventSem failed, apiret: %lu",rc);
            }
         }
      }

      if (num_remaining_packets)
      {
         unsigned int tmp=0;
         for (j=0,length=0;j<num_remaining_packets ;j++,length += tmp)
         {
            tmp = transfer->iso_packet_desc[packet_index+j].length;
            pIsoResponse->usFrameSize[j] = (USHORT)tmp;
         }
         pIsoResponse->usStatus = 0;
         pIsoResponse->usDataLength = (USHORT)length;

         rc = UsbStartIsoTransfer((USBHANDLE)dpriv->fd,
                                  (UCHAR)transfer->endpoint,
                                  (UCHAR)dpriv->altsetting[iface],
                                  (ULONG)hTransferSem,
                                  (PUCHAR)pIsoResponse,
                                  (PUCHAR)buffer,
                                  (USHORT)packet_len,
                                  (USHORT)num_remaining_packets);
         usbi_dbg("UsbStartIsoTransfer with dh = %u, pass = %u",(unsigned int)dpriv->fd,i);
         usbi_dbg("UsbStartIsoTransfer with iso ep = %#02x",transfer->endpoint);
         usbi_dbg("UsbStartIsoTransfer with usbcalls_timeout = %d",transfer->timeout);
         usbi_dbg("UsbStartIsoTransfer num iso packets %u, packet len %u, buffer len %u",num_remaining_packets,packet_len,length);
         usbi_dbg("UsbStartIsoTransfer rc = %lu",rc);

         if (!rc) {
            rc = DosWaitEventSem(hTransferSem,(ULONG)transfer->timeout);
            if (!rc)
            {
               status = (0 == pIsoResponse->usStatus) ? LIBUSB_TRANSFER_COMPLETED : LIBUSB_TRANSFER_ERROR;
               for(j=0;j<num_remaining_packets;j++) {
                  itransfer->transferred                                 += pIsoResponse->usFrameSize[j];
                  transfer->iso_packet_desc[packet_index+j].actual_length = pIsoResponse->usFrameSize[j];
                  transfer->iso_packet_desc[packet_index+j].status        = status;
               }
            } else {
               usbi_dbg("DosWaitEventSem failed, apiret: %lu",rc);
               rc = UsbCancelTransfer((USBHANDLE)dpriv->fd,(UCHAR)transfer->endpoint,(UCHAR)dpriv->altsetting[iface],(ULONG)hTransferSem);
               if (NO_ERROR != rc) {
                  usbi_dbg("UsbCancelTransfer failed, apiret: %lu",rc);
               }
               else
               {
                  rc = DosWaitEventSem(hTransferSem,(ULONG)transfer->timeout);
               }
               errorcode = _apiret_to_libusb(rc);
            }
            rc = DosResetEventSem(hTransferSem,&postCount);
            if (ERROR_ALREADY_RESET != rc && NO_ERROR != rc) {
               usbi_dbg("DosResetEventSem failed, apiret: %lu",rc);
            }
         }
      }

      rc = DosCloseEventSem(hTransferSem);
      if (NO_ERROR != rc) {
         usbi_dbg("hTransferSem: DosCloseEventSem failed, apiret: %lu",rc);
      }

cleanup:
      rc = UsbIsoClose((USBHANDLE)dpriv->fd,(UCHAR)transfer->endpoint,(UCHAR)dpriv->altsetting[iface]);
      usbi_dbg("UsbIsoClose for if %#02x, alt %#02x, ep %#02x, apiret: %lu",iface,dpriv->altsetting[iface],transfer->endpoint,rc);
      if (NO_ERROR != rc) {
         errorcode = _apiret_to_libusb(rc);
      }

cleanup2:
      /*
       * free the per-device mutex protection, we are done
       */
      usbi_mutex_unlock(&dev->lock);

   } while (0); /* enddo */



   usbi_dbg("itransfer->transferred = %d, nr =%d",itransfer->transferred, transfer->length);
   return(errorcode);
}

/* The 3 functions below are unlikely to ever get supported on OS/2 */
static int os2_kernel_driver_active(struct libusb_device_handle *dev_handle, uint8_t iface)
{
   UNUSED(dev_handle);
   UNUSED(iface);
   return(LIBUSB_ERROR_NOT_SUPPORTED);
}

static int os2_attach_kernel_driver(struct libusb_device_handle *dev_handle, uint8_t iface)
{
   UNUSED(dev_handle);
   UNUSED(iface);
   return(LIBUSB_ERROR_NOT_SUPPORTED);
}

static int os2_detach_kernel_driver(struct libusb_device_handle *dev_handle, uint8_t iface)
{
   UNUSED(dev_handle);
   UNUSED(iface);
   return(LIBUSB_ERROR_NOT_SUPPORTED);
}

