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

#include <string.h>
#include <usbcalls.h>

#include "libusb.h"
#include "libusbi.h"
#include "os2_usb.h"

unsigned long _System _DLL_InitTerm(unsigned long hmod, unsigned long flag);
extern int  _CRT_init(void);
extern void _CRT_term(void);
extern void __ctordtorInit(void);
extern void __ctordtorTerm(void);

static void *GenericHandlingThread(void *arg);
static void ControlHandlingRoutine(struct usbi_transfer *itransfer);
static void BulkIrqHandlingRoutine(struct usbi_transfer *itransfer);
static void IsoStreamHandlingRoutine(struct usbi_transfer *itransfer);

/*
 * Backend functions
 */
static int os2_get_device_list(struct libusb_context *,struct discovered_devs **);
static int os2_open(struct libusb_device_handle *);
static void os2_close(struct libusb_device_handle *);
static int os2_get_active_config_descriptor(struct libusb_device *,void *, size_t);
static int os2_get_config_descriptor(struct libusb_device *, uint8_t,void *, size_t);
static int os2_get_config_descriptor_by_value(struct libusb_device *,uint8_t, void **);
static int os2_get_configuration(struct libusb_device_handle *, uint8_t *);
static int os2_set_configuration(struct libusb_device_handle *, int);
static int os2_claim_interface(struct libusb_device_handle *, uint8_t);
static int os2_release_interface(struct libusb_device_handle *, uint8_t);
static int os2_set_interface_altsetting(struct libusb_device_handle *, uint8_t, uint8_t);
static int os2_clear_halt(struct libusb_device_handle *, unsigned char);
static int os2_reset_device(struct libusb_device_handle *);
static void os2_destroy_device(struct libusb_device *);
static int os2_submit_transfer(struct usbi_transfer *);
static int os2_cancel_transfer(struct usbi_transfer *);
static void os2_clear_transfer_priv(struct usbi_transfer *);
static int os2_handle_transfer_completion(struct usbi_transfer *);

/*
 * Private functions
 */
static int _is_streaming_interface(struct libusb_device *dev,uint8_t iface);
static void _call_iso_close(struct libusb_device *dev);
static int _interface_for_endpoint(struct libusb_device *dev,uint8_t endpoint);
static uint8_t _endpoint_for_alternate_setting(struct libusb_device *dev,uint8_t iface, uint8_t altsetting, uint16_t *ppacket_len);
static int _apiret_to_libusb(ULONG);
static int _async_control_transfer(struct usbi_transfer *);
static int _async_bulkirq_transfer(struct usbi_transfer *);
static int _async_iso_transfer(struct usbi_transfer *);

const struct usbi_os_backend usbi_backend = {
   "Asynchronous OS/2 backend",
   USBI_CAP_HAS_HID_ACCESS, /* OS/2 USB driver stack makes no attempt to prevent arbitrary access to any USB device */
   NULL,          /* init() */
   NULL,          /* exit() */
   NULL,          /* set_option() */
   os2_get_device_list,
   NULL,          /* hotplug_poll() */
   NULL,          /* wrap_sys_device() */
   os2_open,
   os2_close,
   os2_get_active_config_descriptor,
   os2_get_config_descriptor,
   os2_get_config_descriptor_by_value,
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
   NULL,          /* kernel_driver_active() */
   NULL,          /* detach_kernel_driver() */
   NULL,          /* attach_kernel_driver() */
   os2_destroy_device,
   os2_submit_transfer,
   os2_cancel_transfer,
   os2_clear_transfer_priv,
   NULL,            /* handle_events() */
   os2_handle_transfer_completion,
   0,                            /* context private data */
   sizeof(struct device_priv),   /* device private data */
   0,                            /* handle private data */
   sizeof(struct transfer_priv)  /* transfer private data */
};

static HMUX ghMux               = NULLHANDLE;
static HEV  ghTerminateSem      = NULLHANDLE;
static pthread_t gThrd          = NULL;


unsigned long _System _DLL_InitTerm(unsigned long hmod, unsigned long flag)
{
    SEMRECORD semRec = {0};
    pthread_attr_t attr = NULL;

    UNUSED(hmod);

    if (0 == flag)
    {
        if (_CRT_init())
        {
            return 0;
        }

        if (NO_ERROR != DosCreateEventSem(NULL,&ghTerminateSem,DC_SEM_SHARED,FALSE))
        {
            _CRT_term();
            return 0;
        }

        semRec.hsemCur = (HSEM)ghTerminateSem;
        semRec.ulUser  = 0xFFFFFFFFUL;
        if (NO_ERROR != DosCreateMuxWaitSem(NULL,&ghMux,1,&semRec,DCMW_WAIT_ANY))
        {
            DosCloseEventSem(ghTerminateSem);
            _CRT_term();
            return 0;
        }

        if (pthread_attr_init(&attr))
        {
            DosDeleteMuxWaitSem(ghMux,(HSEM)ghTerminateSem);
            DosCloseEventSem(ghTerminateSem);
            DosCloseMuxWaitSem(ghMux);
            _CRT_term();
            return 0;
        }

        pthread_attr_setprio(&attr,PRTYD_MAXIMUM); /* can only change delta, not class */

        if (pthread_create(&gThrd,&attr,GenericHandlingThread,NULL))
        {
            pthread_attr_destroy(&attr);
            DosDeleteMuxWaitSem(ghMux,(HSEM)ghTerminateSem);
            DosCloseEventSem(ghTerminateSem);
            DosCloseMuxWaitSem(ghMux);
            _CRT_term();
            return 0;
        }
        pthread_attr_destroy(&attr);
    }
    else if (1 == flag)
    {
        DosDeleteMuxWaitSem(ghMux,(HSEM)ghTerminateSem);
        DosCloseEventSem(ghTerminateSem);
        DosCloseMuxWaitSem(ghMux);
        _CRT_term();
    }
    return 1;
}

static void *GenericHandlingThread(void *arg)
{
    UNUSED(arg);

    ULONG  ulUser = 0UL;
    struct usbi_transfer *itransfer = NULL;
    struct libusb_transfer *transfer = NULL;
    APIRET rc = NO_ERROR;

    do
    {
       rc = DosWaitMuxWaitSem(ghMux,SEM_INDEFINITE_WAIT,(PULONG)&ulUser);
       usbi_dbg("DosWaitMuxWaitSem,  rc = %lu",rc);

       /*
        * do NOT access access itransfer/transfer at this point
        * as it might no longer be accessible if "os2_close" has
        * triggered termination !
        */
       if (0xFFFFFFFFUL == ulUser)
       {
           break;
       }

       itransfer = (struct usbi_transfer *)ulUser;
       transfer  = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);

       if ((NO_ERROR == rc) && transfer->dev_handle)
       {
         /*
          * at this point, we always need to check for transfer type:
          * even though the itransfer/transfer pointer is the same
          * the transfer type might change from one invocation to
          * the next because the itransfer/transfer pointer might
          * point to a different transfer type
          */
          switch(transfer->type)
          {
              case LIBUSB_TRANSFER_TYPE_CONTROL:
                  ControlHandlingRoutine(itransfer);
                  break;
              case LIBUSB_TRANSFER_TYPE_INTERRUPT:
              case LIBUSB_TRANSFER_TYPE_BULK:
                  BulkIrqHandlingRoutine(itransfer);
                  break;
              case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
                  IsoStreamHandlingRoutine(itransfer);
                  break;
              default:
                  break;
          }
       }
    } while (1);

    usbi_dbg("I/O completion thread exiting");

    return NULL;
}

static void ControlHandlingRoutine(struct usbi_transfer *itransfer)
{
    struct transfer_priv *tpriv     = (struct transfer_priv *)usbi_get_transfer_priv(itransfer);
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);

    usbi_dbg("ctrl: Response.usStatus = %#x",(unsigned int)tpriv->Response.usStatus);

    if (TRUE == tpriv->toCancel)
    {
        tpriv->status = LIBUSB_TRANSFER_CANCELLED;
        itransfer->transferred = 0;
    }
    else
    {
        tpriv->Processed += tpriv->Response.usDataLength;

        tpriv->status = (0 == tpriv->Response.usStatus) ? LIBUSB_TRANSFER_COMPLETED : LIBUSB_TRANSFER_ERROR;
        itransfer->transferred = tpriv->Processed;
        usbi_dbg("transferred %u of %u bytes",itransfer->transferred,transfer->length);
    }

    DosDeleteMuxWaitSem(ghMux,(HSEM)tpriv->hEventSem);
    DosCloseEventSem(tpriv->hEventSem);
    tpriv->hEventSem = NULLHANDLE;

    usbi_signal_transfer_completion(itransfer);

    return;
}

static void BulkIrqHandlingRoutine(struct usbi_transfer *itransfer)
{
    struct transfer_priv *tpriv     = (struct transfer_priv *)usbi_get_transfer_priv(itransfer);
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    struct libusb_device *dev       = transfer->dev_handle->dev;
    struct device_priv *dpriv       = (struct device_priv *)usbi_get_device_priv(dev);
    APIRET rc;

    usbi_dbg("bulk/irq: Response.usStatus = %#x",(unsigned int)tpriv->Response.usStatus);

    if (TRUE == tpriv->toCancel)
    {
        tpriv->status = LIBUSB_TRANSFER_CANCELLED;
        itransfer->transferred = tpriv->Processed;

        DosDeleteMuxWaitSem(ghMux,(HSEM)tpriv->hEventSem);
        DosCloseEventSem(tpriv->hEventSem);
        tpriv->hEventSem = NULLHANDLE;

        usbi_signal_transfer_completion(itransfer);
    }
    else
    {
       tpriv->Processed += tpriv->Response.usDataLength;

       if ((tpriv->ToProcess == tpriv->Response.usDataLength) && (tpriv->Processed < (unsigned int)transfer->length))
       {
           usbi_dbg("received %u from %u bytes",tpriv->Processed,transfer->length);

           int diff = transfer->length - tpriv->Processed;
           tpriv->ToProcess = diff < MAX_TRANSFER_SIZE ? diff : MAX_TRANSFER_SIZE;

           tpriv->Response.usDataLength = (USHORT)tpriv->ToProcess;
           rc = UsbStartDataTransfer(dpriv->fd,transfer->endpoint,0,(PUCHAR)&tpriv->Response,transfer->buffer+tpriv->Processed,tpriv->hEventSem,IS_XFERIN(transfer) ? 0 : USB_TRANSFER_FULL_SIZE);
           usbi_dbg("UsbStartDataTransfer with fd = %#.8lx",dpriv->fd);
           usbi_dbg("UsbStartDataTransfer with ep = %#02x",transfer->endpoint);
           usbi_dbg("UsbStartDataTransfer with timeout = %d",transfer->timeout);
           usbi_dbg("UsbStartDataTransfer rc = %lu",rc);
           if (NO_ERROR != rc)
           {
              tpriv->status = LIBUSB_TRANSFER_ERROR;
              itransfer->transferred = tpriv->Processed;
              usbi_dbg("transferred %u of %u bytes (unfinished transfer)",itransfer->transferred,transfer->length);

              DosDeleteMuxWaitSem(ghMux,(HSEM)tpriv->hEventSem);
              DosCloseEventSem(tpriv->hEventSem);
              tpriv->hEventSem = NULLHANDLE;

              usbi_signal_transfer_completion(itransfer);
           }
           else
           {
              ULONG ulCount = 0;
              /* if NO_ERROR, wait for next termination of transfer and event semaphore notification */
              DosResetEventSem(tpriv->hEventSem,&ulCount);
           }
       }
       else
       {
          tpriv->status = (0 == tpriv->Response.usStatus) ? LIBUSB_TRANSFER_COMPLETED : LIBUSB_TRANSFER_ERROR;
          itransfer->transferred = tpriv->Processed;
          usbi_dbg("transferred %u of %u bytes",itransfer->transferred,transfer->length);

          DosDeleteMuxWaitSem(ghMux,(HSEM)tpriv->hEventSem);
          DosCloseEventSem(tpriv->hEventSem);
          tpriv->hEventSem = NULLHANDLE;

          usbi_signal_transfer_completion(itransfer);
       }
    }
    return;
}

static void IsoStreamHandlingRoutine(struct usbi_transfer *itransfer)
{
    struct transfer_priv *tpriv     = (struct transfer_priv *)usbi_get_transfer_priv(itransfer);
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    struct libusb_device *dev       = transfer->dev_handle->dev;
    struct device_priv *dpriv       = (struct device_priv *)usbi_get_device_priv(dev);
    unsigned int j;

    usbi_dbg("iso: Response.usStatus = %#x",(unsigned int)tpriv->Response.usStatus);

    if (TRUE == tpriv->toCancel)
    {
       usbi_dbg("transfer was cancelled !");

       tpriv->status = LIBUSB_TRANSFER_CANCELLED;
       itransfer->transferred = 0;
       for(j=0;j<(unsigned int)transfer->num_iso_packets;j++) {
           transfer->iso_packet_desc[j].actual_length = 0;
           transfer->iso_packet_desc[j].status        = tpriv->status;
       }
    }
    else
    {
      usbi_dbg("transfer succeeded !");

      tpriv->status = (0 == tpriv->Response.usStatus ) ? LIBUSB_TRANSFER_COMPLETED : LIBUSB_TRANSFER_ERROR;
      for(j=0,itransfer->transferred = 0;j<tpriv->ToProcess;j++) {
          itransfer->transferred                    += tpriv->Response.usFrameSize[j];
          transfer->iso_packet_desc[j].actual_length = tpriv->Response.usFrameSize[j];
          transfer->iso_packet_desc[j].status        = tpriv->status;
      }
      tpriv->Processed += tpriv->ToProcess;
    }

    usbi_mutex_lock(&dev->lock);
    dpriv->numIsoBuffsInUse -= 1;
    usbi_dbg("Num Iso Buffers in use:%u",dpriv->numIsoBuffsInUse);
    usbi_mutex_unlock(&dev->lock);

    usbi_dbg("transferred %u of %u bytes",itransfer->transferred,transfer->length);

    DosDeleteMuxWaitSem(ghMux,(HSEM)tpriv->hEventSem);
    DosCloseEventSem(tpriv->hEventSem);
    tpriv->hEventSem = NULLHANDLE;

    usbi_signal_transfer_completion(itransfer);

    return;
}



static int os2_get_device_list(struct libusb_context * ctx, struct discovered_devs **discdevs)
{
   APIRET    rc;
   ULONG     cntDev;
   ULONG     ctr;
   ULONG     len;
   struct libusb_device *dev;
   struct device_priv *dpriv;
   struct usbi_device_descriptor *pDeviceDescriptor = NULL;
   struct discovered_devs *ddd;

   unsigned char scratchBuf[LIBUSB_DT_DEVICE_SIZE+4096];
   GETDEVINFODATA info;


   usbi_dbg(" ");

   rc = UsbQueryNumberDevices( &cntDev);
   if (rc) {
      usbi_dbg( "unable to query number of USB devices - rc = %lu", rc);
      return(LIBUSB_ERROR_IO);
   }
   usbi_dbg( "%lu devices detected", cntDev);
   for (ctr = 1; ctr <= cntDev; ctr++)
   {
      len = sizeof(info);
      rc = UsbQueryDeviceInfo( ctr,&len, (PUCHAR)&info);
      if (rc) {
         usbi_dbg( "unable to query device info - device= %lu  rc = %lu", ctr, rc);
         return(LIBUSB_ERROR_IO);
      }

      len = sizeof(scratchBuf);
      rc = UsbQueryDeviceReport( ctr, &len, scratchBuf);
      if (rc) {
         usbi_dbg( "unable to query device report - device= %lu  rc= %lu", ctr, rc);
         return(LIBUSB_ERROR_IO);
      }

      /* under OS/2, the Resource Manager device handle is a GUID, well suited to track a device */
      dev = usbi_get_device_by_session_id(ctx, info.rmDevHandle);
      if (dev == NULL) {
         dev = usbi_alloc_device(ctx, info.rmDevHandle);
         if (dev == NULL)
            return(LIBUSB_ERROR_NO_MEM);

         dev->bus_number     = info.ctrlID;        /* under OS/2, the used HC is equivalent to the bus number */
         dev->device_address = info.deviceAddress; /* the real USB address programmed into the USB device */
         dev->port_number    = info.portNum;       /* the real port number on the hub the device is attached to, is zero for a root hub device */
         switch (info.SpeedDevice) {
         case USB_SPEED_FULL:
            dev->speed = LIBUSB_SPEED_FULL;
            break;
         case USB_SPEED_LOW:
            dev->speed = LIBUSB_SPEED_LOW;
            break;
         case USB_SPEED_HIGH:
            dev->speed = LIBUSB_SPEED_HIGH;
            break;
         case USB_SPEED_SUPER:
            dev->speed = LIBUSB_SPEED_SUPER;
            break;
         default:
            dev->speed = LIBUSB_SPEED_UNKNOWN;
            break;
         } /* endswitch */
         dev->parent_dev = NULL;                   /* under OS/2, we do have the parent hub identifier as info.parentHubIndex */
                                                   /* but only non-hub devices are exposed by USBD.SYS (and therefore USBRESMG.SYS) */
                                                   /* therefore, it is impossible to query the full bus hierarchy */

         /*
          * the libusb exposed device descriptor structure
          * is not necessarily byte packed. We therefore have
          * to copy over each individual field
          */
         pDeviceDescriptor = (struct usbi_device_descriptor *)scratchBuf;
         dev->device_descriptor.bLength = pDeviceDescriptor->bLength;
         dev->device_descriptor.bDescriptorType = pDeviceDescriptor->bDescriptorType;
         dev->device_descriptor.bcdUSB = pDeviceDescriptor->bcdUSB;
         dev->device_descriptor.bDeviceClass = pDeviceDescriptor->bDeviceClass;
         dev->device_descriptor.bDeviceSubClass = pDeviceDescriptor->bDeviceSubClass;
         dev->device_descriptor.bDeviceProtocol = pDeviceDescriptor->bDeviceProtocol;
         dev->device_descriptor.bMaxPacketSize0 = pDeviceDescriptor->bMaxPacketSize0;
         dev->device_descriptor.idVendor = pDeviceDescriptor->idVendor;
         dev->device_descriptor.idProduct = pDeviceDescriptor->idProduct;
         dev->device_descriptor.bcdDevice = pDeviceDescriptor->bcdDevice;
         dev->device_descriptor.iManufacturer = pDeviceDescriptor->iManufacturer;
         dev->device_descriptor.iProduct = pDeviceDescriptor->iProduct;
         dev->device_descriptor.iSerialNumber = pDeviceDescriptor->iSerialNumber;
         dev->device_descriptor.bNumConfigurations = pDeviceDescriptor->bNumConfigurations;
         usbi_localize_device_descriptor(&dev->device_descriptor);

         dpriv = (struct device_priv *)usbi_get_device_priv(dev);
         dpriv->fd = -1U;
         dpriv->numIsoBuffsInUse = 0;
         dpriv->numOpens = 0;
         memset(dpriv->altsetting,0,sizeof(dpriv->altsetting));
         memset(dpriv->endpoint,0,sizeof(dpriv->endpoint));
         memcpy(dpriv->cdesc, scratchBuf + LIBUSB_DT_DEVICE_SIZE, (len - LIBUSB_DT_DEVICE_SIZE));
         if (LIBUSB_SUCCESS != libusb_get_config_descriptor(dev,0,&dpriv->curr_config_descriptor))
         {
             dpriv->curr_config_descriptor = NULL;
         }

         usbi_dbg( "allocated libusb_get_config_descriptor: %p", dpriv->curr_config_descriptor);

         if (usbi_sanitize_device(dev)) {
            libusb_unref_device(dev);
            continue;
         }
      }
      ddd = discovered_devs_append(*discdevs, dev);
      libusb_unref_device(dev);
      if (ddd == NULL) {
         return(LIBUSB_ERROR_NO_MEM);
      }

      *discdevs = ddd;
   }
   return(LIBUSB_SUCCESS);
}

static int os2_open(struct libusb_device_handle *handle)
{
   struct libusb_device *dev = handle->dev;
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(dev);
   APIRET    rc = NO_ERROR;
   int       usbhandle;
   int       ret = LIBUSB_SUCCESS;

   /* take device mutex: we need to manipulate per device control var "numOpens" */
   usbi_mutex_lock(&dev->lock);

   usbi_dbg("on entry: fd %#.8lx, numOpens: %u", dpriv->fd,dpriv->numOpens);

   if (dpriv->numOpens == 0)
   {
      rc = UsbOpen( (PUSBHANDLE)&usbhandle,
         (USHORT)dev->device_descriptor.idVendor,
         (USHORT)dev->device_descriptor.idProduct,
         (USHORT)dev->device_descriptor.bcdDevice,
         (USHORT)USB_OPEN_FIRST_UNUSED);

      usbi_dbg( "UsbOpen: id = %#04x:%#04x  rc = %lu",
                dev->device_descriptor.idVendor,
                dev->device_descriptor.idProduct,
                rc);

      if (rc)
      {
         dpriv->fd = -1U;
         ret = _apiret_to_libusb(rc);
         goto leave;
      }

      dpriv->fd = usbhandle;

      /* set device configuration to 1st configuration */
      rc = UsbDeviceSetConfiguration (dpriv->fd,1);
      usbi_dbg("UsbDeviceSetConfiguration: rc = %lu", rc);
   }
   /* endif */
   dpriv->numOpens++;

leave:
   usbi_mutex_unlock(&dev->lock);

   return(ret);
}

static void os2_close(struct libusb_device_handle *handle)
{
   struct libusb_device *dev = handle->dev;
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(dev);
   APIRET    rc = NO_ERROR;
   unsigned int numOpens = 0;
   SEMRECORD semRec[64] = {0};
   ULONG numSems = 64;
   ULONG ulAttributes = 0;

   /* take device mutex: we need to manipulate per device control var "numOpens" */
   usbi_mutex_lock(&dev->lock);
   if (dpriv->numOpens)
   {
       dpriv->numOpens--;
       numOpens = dpriv->numOpens;
   }
   /*
    * free the dev mutex here, because it will be claimed by libusb_unref_dev further below !
    */
   usbi_mutex_unlock(&dev->lock);

   usbi_dbg("on entry: fd %#.8lx, numOpens: %u", dpriv->fd,numOpens);


   if (numOpens == 0)
   {
      rc = DosQueryMuxWaitSem(ghMux,&numSems,semRec,&ulAttributes);
      usbi_dbg("DosQueryMuxWaitSem, numSems = %lu, rc = %lu",numSems,rc);
      /*
       * decrement by the ghTerminateSem in order to find out how many
       * transfers are still flying
       */
      if (numSems) numSems--;

      /*
       * kludge: we only want the worker thread to stop on the very last close
       * the device refcnt is our kludge to find out
       */
      if (dev->refcnt > 2)
      {
          DosPostEventSem(ghTerminateSem);
          pthread_join(gThrd,NULL);
      }

      while(numSems)
      {
         usbi_dbg("unref device once");

         /*
          * libusb increments the ref cnt on every "libusb_submit_transfer"
          * and decrements it on every "usbi_handle_transfer_completion"
          * however a transfer that is never executed (and therefore, still chained)
          * will miss the decrement because it is never completed,
          * which will lead to the device not being properly destroyed in the libusb library
          * we unref it once so that the libusb ref cnt can finally reach zero
          */
          libusb_unref_device(dev);
          numSems--;
      }

      rc = UsbClose(dpriv->fd);

      usbi_dbg( "UsbClose: id = %#04x:%#04x  rc = %lu",
          dev->device_descriptor.idVendor,
          dev->device_descriptor.idProduct,
          rc);

      dpriv->fd = -1U;
   }
   /* endif */
}

static int os2_get_active_config_descriptor(struct libusb_device *dev,void *buf, size_t len)
{
   struct device_priv *dpriv            = (struct device_priv *)usbi_get_device_priv(dev);
   struct libusb_config_descriptor *ucd = (struct libusb_config_descriptor*) (dpriv->cdesc);

   len = MIN(len, ucd->wTotalLength);

   usbi_dbg("len %d", len);

   memcpy(buf, dpriv->cdesc, len);

   return(len);
}

static int os2_get_config_descriptor(struct libusb_device *dev, uint8_t idx,void *buf, size_t len)
{
   UNUSED(idx);
   /* just return the active config descriptor, it's a bad idea to arbitrarily switch configuration anyway */
   return(os2_get_active_config_descriptor(dev,buf,len));
}

static int os2_get_config_descriptor_by_value(struct libusb_device *dev,uint8_t bConfigurationValue, void **buffer)
{
   struct device_priv *dpriv            = (struct device_priv *)usbi_get_device_priv(dev);

   if (bConfigurationValue != 1)
   {
       return LIBUSB_ERROR_NOT_FOUND;
   }
   *buffer = dpriv->cdesc;

   return LIBUSB_SUCCESS;
}


/*
 * as requested in the function description in libusbi.h, make sure we avoid I/O to get the configuration value
 * fortunately, USBD.SYS will internally save the selected configuration which we query here via
 * UsbQueryDeviceInfo
 */
static int os2_get_configuration(struct libusb_device_handle *handle, uint8_t *config)
{
   struct libusb_device *dev = handle->dev;
   ULONG cntDev,ctr,len;
   APIRET rc = LIBUSB_SUCCESS;
   GETDEVINFODATA info;

   usbi_dbg(" ");

   *config = 0;

   rc = UsbQueryNumberDevices( &cntDev);
   if (rc) {
      usbi_dbg( "unable to query number of USB devices - rc= %lu", rc);
      return(LIBUSB_ERROR_IO);
   }
   for (ctr = 1; ctr <= cntDev; ctr++) {
      len = sizeof(info);
      rc = UsbQueryDeviceInfo( ctr,&len, (PUCHAR)&info);
      if (rc) {
         usbi_dbg( "unable to query device info - rc = %lu", rc);
         return(LIBUSB_ERROR_IO);
      } /* endif */
      if (info.rmDevHandle == dev->session_data) {
         *config = info.bConfigurationValue;
         return(LIBUSB_SUCCESS);
      } /* endif */
   }

   return(LIBUSB_ERROR_NO_DEVICE);
}

static int os2_set_configuration(struct libusb_device_handle *handle, int config)
{
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(handle->dev);
   APIRET    rc = LIBUSB_SUCCESS;

   usbi_dbg("configuration %d", config);

   rc = UsbDeviceSetConfiguration( dpriv->fd, (USHORT)config);
   if (rc) {
      usbi_dbg( "unable to set configuration - config= %d  rc= %lu",
         config, rc);
      return(LIBUSB_ERROR_IO);
   }

   return(LIBUSB_SUCCESS);
}

static int os2_claim_interface(struct libusb_device_handle *handle, uint8_t iface)
{
   UNUSED(handle);
   UNUSED(iface);
/* USBRESM$ appears to handle this as part of opening the device */
   usbi_dbg(" ");
   return(LIBUSB_SUCCESS);
}

static int os2_release_interface(struct libusb_device_handle *handle, uint8_t iface)
{
/* USBRESM$ appears to handle this as part of opening the device */
   usbi_dbg(" ");

   APIRET rc     = NO_ERROR;
   int errorcode = LIBUSB_SUCCESS;

   struct libusb_device *dev = handle->dev;
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(dev);
   if (_is_streaming_interface(dev,iface) && dpriv->endpoint[iface] && dpriv->altsetting[iface])
   {
      rc = UsbInterfaceSetAltSetting(dpriv->fd,iface,0);
      usbi_dbg("UsbInterfaceSetAltSetting for if %#02x with alt 0, rc = %lu",iface,rc);
      if (NO_ERROR != rc) {
         errorcode = _apiret_to_libusb(rc);
      }
      dpriv->endpoint[iface] = 0;
      dpriv->altsetting[iface] = 0;
   }
   return(errorcode);
}

static int os2_set_interface_altsetting(struct libusb_device_handle *handle, uint8_t iface,uint8_t altsetting)
{
   struct libusb_device *dev = handle->dev;
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(handle->dev);
   APIRET rc = NO_ERROR;
   int errorcode = LIBUSB_SUCCESS;

   usbi_dbg(" ");
//      uint8_t newsetting=0xFFU;

   rc = UsbInterfaceSetAltSetting(dpriv->fd,iface,altsetting);
   usbi_dbg("UsbInterfaceSetAltSetting for if %#02x, alt %#02x, rc = %lu",iface,altsetting,rc);

   if (NO_ERROR != rc) {
      return(_apiret_to_libusb(rc));
   }

//      check that we can successfully read back the value we just set
//      rc = UsbInterfaceGetAltSetting(dpriv->fd,iface,(PUCHAR)&newsetting);

   if (_is_streaming_interface(dev,iface))
   {
       if (0 != altsetting)
       {
          uint16_t packet_len = 0;
          uint8_t endpoint = _endpoint_for_alternate_setting(dev,iface,altsetting,&packet_len);

          rc = UsbIsoOpen(dpriv->fd,
                             endpoint,
                             altsetting,
                             NUM_ISO_BUFFS, /* number of HC internal ISO buffer management structures */
                             packet_len);
          usbi_dbg("UsbIsoOpen for if %#02x, alt %#02x, ep %#02x, packet length %u, rc =  %lu",iface,altsetting,endpoint,packet_len,rc);
          errorcode = _apiret_to_libusb(rc);
          dpriv->endpoint[iface] = endpoint;
       }
       else
       {
          rc = UsbIsoClose(dpriv->fd,dpriv->endpoint[iface],dpriv->altsetting[iface]);
          usbi_dbg("UsbIsoClose for if %#02x, alt %#02x, ep %#02x, rc = %lu",iface,dpriv->altsetting[iface],dpriv->endpoint[iface],rc);
          errorcode = _apiret_to_libusb(rc);
       }
   }
   dpriv->altsetting[iface] = altsetting;
   usbi_dbg("UsbInterfaceSetAltSetting interface %u set to alternate setting: %u",iface,altsetting);

   return (errorcode);
}

static int os2_clear_halt(struct libusb_device_handle *handle, unsigned char endpoint)
{
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(handle->dev);
   APIRET rc = LIBUSB_SUCCESS;

   usbi_dbg(" ");
   rc = UsbEndpointClearHalt(dpriv->fd,endpoint);
   return(_apiret_to_libusb(rc));
}

static int os2_reset_device(struct libusb_device_handle *handle)
{
/* TO DO */
   UNUSED(handle);

   usbi_dbg(" ");
   return(LIBUSB_ERROR_NOT_SUPPORTED);
}

static void os2_destroy_device(struct libusb_device *dev)
{
   usbi_dbg(" ");

   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(dev);

   _call_iso_close(dev);
   libusb_free_config_descriptor(dpriv->curr_config_descriptor);

   usbi_dbg( "freed libusb_get_config_descriptor: %p", dpriv->curr_config_descriptor);

   dpriv->curr_config_descriptor = NULL;
}

static int os2_submit_transfer(struct usbi_transfer *itransfer)
{
   struct libusb_context *ctx       = ITRANSFER_CTX(itransfer);
   struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   int err = LIBUSB_SUCCESS;

   usbi_dbg(" ");

   /*
    * protect the transfers that are submitted
    * from being incorrectly added to the list
    * of flying transfers: there is a chance
    * that the itransfer address of a transfer
    * to be freed matches the itransfer address
    * of a newly allocated transfer in case
    * the heap managment picks the very same memory
    * block for its allocation
    * We do this by requesting the flying transfer
    * mutex for the whole time that we are busy
    * submitting a transfer
    */
   usbi_mutex_lock(&ctx->flying_transfers_lock);

   switch (transfer->type) {
   case LIBUSB_TRANSFER_TYPE_CONTROL:
      err = _async_control_transfer(itransfer);
      break;
   case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
      err = _async_iso_transfer(itransfer);
      break;
   case LIBUSB_TRANSFER_TYPE_BULK:
   case LIBUSB_TRANSFER_TYPE_INTERRUPT:
      if (IS_XFEROUT(transfer) &&
          transfer->flags & LIBUSB_TRANSFER_ADD_ZERO_PACKET) {
         err = LIBUSB_ERROR_NOT_SUPPORTED;
         break;
      }
      err = _async_bulkirq_transfer(itransfer);
      break;
   case LIBUSB_TRANSFER_TYPE_BULK_STREAM:
      err = LIBUSB_ERROR_NOT_SUPPORTED;
      break;
   default:
      err = LIBUSB_ERROR_NOT_SUPPORTED;
      break;
   }

   usbi_mutex_unlock(&ctx->flying_transfers_lock);

   return(err);
}

static int os2_cancel_transfer(struct usbi_transfer *itransfer)
{
   struct transfer_priv *tpriv = (struct transfer_priv *)usbi_get_transfer_priv(itransfer);
   struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   struct libusb_device *dev        = NULL;
   struct device_priv *dpriv        = NULL;
   APIRET rc = NO_ERROR;
   int iface = 0;

   usbi_dbg(" ");

   if (!transfer->dev_handle)
   {
       return (LIBUSB_ERROR_INVALID_PARAM);
   }

   dev = transfer->dev_handle->dev;
   dpriv = (struct device_priv *)usbi_get_device_priv(dev);

   iface = _interface_for_endpoint(dev,transfer->endpoint);
   if (iface < 0) {
      usbi_dbg("no interface to endpoint %#02x found",transfer->endpoint);
      return(LIBUSB_ERROR_INVALID_PARAM);
   } /* endif */

   tpriv->toCancel = TRUE;

   /*
    * we call UsbCancelTransfer for each transfer to be cancelled, including isochronous transfers
    * that will lead to an error in UsbIsoClose (because that also tries to cancel any outstanding transfers)
    * but if we'd not cancel the transfers here, UsbIsoClose will not properly cleanup which leads to problems
    * when the device is reinserted
    */
   rc = UsbCancelTransfer(dpriv->fd,transfer->endpoint,dpriv->altsetting[iface],tpriv->hEventSem);
   usbi_dbg("UsbCancelTransfer with ep = %#02x, alt = %02u, rc = %lu",transfer->endpoint,dpriv->altsetting[iface],rc);

   return(_apiret_to_libusb(rc));
}

static void os2_clear_transfer_priv(struct usbi_transfer *itransfer)
{
   struct transfer_priv *tpriv = (struct transfer_priv *)usbi_get_transfer_priv(itransfer);

   /* TODO: free transfer related data */
   DosCloseEventSem(tpriv->hEventSem);

   return;
}

static int os2_handle_transfer_completion(struct usbi_transfer *itransfer)
{
   struct transfer_priv *tpriv = (struct transfer_priv *)usbi_get_transfer_priv(itransfer);
   struct libusb_transfer *transfer   = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   int result = LIBUSB_SUCCESS;

   usbi_dbg(" ");

   /*
    * transfers are freed behind libusb back. And this is one place where we have to catch
    * this error to prevent "usbi_handle_transfer_cancellation/usbi_handle_transfer_completion"
    * from trapping. In the specific error case, an attempt was made to remove this transfer element
    * multiple times. Therefore we check for "empty list".
    */
   if (!itransfer->list.next || !itransfer->list.prev)
   {
       usbi_dbg("transfer no longer chained in, aborting !");
       return (LIBUSB_ERROR_INVALID_PARAM);
   }

   if (TRUE == tpriv->toCancel)
   {
      tpriv->toCancel = FALSE;
      usbi_dbg("transfer %p, transfer cancelled", transfer);
      result = usbi_handle_transfer_cancellation(itransfer);
   }
   else
   {
      usbi_dbg("transfer %p, transfer completed", transfer);
      result = usbi_handle_transfer_completion(itransfer, tpriv->status);
   }

   return(result);
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
               ((config->interface[i].altsetting[a].endpoint[e].bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS)) {
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
   unsigned short i=0,a=0,e=0;

   for (i=0;i<config->bNumInterfaces;i++) {
      for (a=0;a<config->interface[i].num_altsetting;a++) {
         for (e=0;e<config->interface[i].altsetting[a].bNumEndpoints;e++) {
            if ((config->interface[i].altsetting[a].endpoint[e].bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
               if (dpriv->endpoint[i] && dpriv->altsetting[i])
               {
                  rc = UsbInterfaceSetAltSetting(dpriv->fd,i,0);
                  usbi_dbg("UsbInterfaceSetAltSetting for if %#02x, alt 0, rc = %lu",i,rc);
                  if (NO_ERROR != rc) {
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
            if (config->interface[i].altsetting[a].endpoint[e].bEndpointAddress == endpoint) {
               interface = i;
               goto leave;
            } /* endif */
         } /* endfor */
      } /* endfor */
   } /* endfor */
leave:
   return(interface);
}

static uint8_t _endpoint_for_alternate_setting(struct libusb_device *dev,uint8_t iface, uint8_t altsetting, uint16_t *ppacket_len)
{
   struct device_priv *dpriv = (struct device_priv *)usbi_get_device_priv(dev);
   struct libusb_config_descriptor *config    = dpriv->curr_config_descriptor;
   int i=0,a=0,e=0;
   uint8_t endpoint = 0;

   *ppacket_len = 0;
   for (i=0;i<config->bNumInterfaces;i++) {
      for (a=0;a<config->interface[i].num_altsetting;a++) {
         for (e=0;e<config->interface[i].altsetting[a].bNumEndpoints;e++) {
            if ((i == iface) && (a == altsetting))
            {
               uint16_t packet_len = config->interface[i].altsetting[a].endpoint[e].wMaxPacketSize;
               packet_len = (packet_len & PACKET_SIZE_MASK) * (((packet_len & PACKET_MUL_MASK) >> PACKET_MUL_SHIFT)+1);

               endpoint = config->interface[i].altsetting[a].endpoint[e].bEndpointAddress;
               *ppacket_len = packet_len;
               goto leave;
            } /* endif */
         } /* endfor */
      } /* endfor */
   } /* endfor */
leave:
   return(endpoint);
}


static int _apiret_to_libusb(ULONG err)
{
   usbi_dbg("error: %lu", err);

   switch (err) {
   case NO_ERROR:
      return(LIBUSB_SUCCESS);
   case (ERROR_USER_DEFINED_BASE | ERROR_ALREADY_ASSIGNED):
       return(LIBUSB_ERROR_INVALID_PARAM);
   case (ERROR_USER_DEFINED_BASE | ERROR_DEV_NOT_EXIST):
       return(LIBUSB_ERROR_NO_DEVICE);
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

static int _async_control_transfer(struct usbi_transfer *itransfer)
{
   usbi_dbg(" ");

   struct transfer_priv *tpriv        = (struct transfer_priv *)usbi_get_transfer_priv(itransfer);
   struct libusb_transfer *transfer   = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   struct libusb_control_setup *setup = (struct libusb_control_setup *)transfer->buffer;
   struct libusb_device *dev          = NULL;
   struct device_priv *dpriv          = NULL;
   APIRET rc = NO_ERROR;
   SEMRECORD semRec = {0};

   if (!transfer->dev_handle)
   {
       return (LIBUSB_ERROR_INVALID_PARAM);
   }

   tpriv->hEventSem = NULLHANDLE;
   rc = DosCreateEventSem(NULL,&tpriv->hEventSem,DC_SEM_SHARED,FALSE);
   usbi_dbg("DosCreateEventSem, rc:%lu",rc);
   if (NO_ERROR != rc)
   {
       return(_apiret_to_libusb(rc));
   }

   dev = transfer->dev_handle->dev;
   dpriv = (struct device_priv *)usbi_get_device_priv(dev);

   tpriv->Processed                = 0;
   tpriv->ToProcess                = setup->wLength;
   tpriv->Response.usStatus        = 0;
   tpriv->Response.usDataLength    = tpriv->ToProcess;
   tpriv->status                   = LIBUSB_TRANSFER_COMPLETED; /* preset with a safe value, different from LIBUSB_TRANSFER_CANCELLED */
   tpriv->toCancel                 = FALSE;


   semRec.hsemCur = (HSEM)tpriv->hEventSem;
   semRec.ulUser  = (ULONG)itransfer;
   rc = DosAddMuxWaitSem(ghMux,&semRec);

   rc = UsbStartCtrlTransfer(   dpriv->fd, 0, 0, setup->bmRequestType,
                                setup->bRequest,setup->wValue,setup->wIndex,
                                (PUCHAR)&tpriv->Response, transfer->buffer + LIBUSB_CONTROL_SETUP_SIZE, tpriv->hEventSem);
   usbi_dbg("UsbStartCtrlTransfer with fd = %#.8lx",dpriv->fd);
   usbi_dbg("UsbStartCtrlTransfer type %#02x request %#02x value %d index %d length %d",setup->bmRequestType,setup->bRequest,setup->wValue,setup->wIndex,setup->wLength);
   usbi_dbg("UsbStartCtrlTransfer with timeout = %d",transfer->timeout);
   usbi_dbg("UsbStartCtrlTransfer rc = %lu",rc);

   if (NO_ERROR != rc)
   {
       DosDeleteMuxWaitSem(ghMux,(HSEM)tpriv->hEventSem);
       DosCloseEventSem(tpriv->hEventSem);
   }

   return(_apiret_to_libusb(rc));
}


static int _async_bulkirq_transfer(struct usbi_transfer *itransfer)
{
   usbi_dbg(" ");

   struct transfer_priv *tpriv      = (struct transfer_priv *)usbi_get_transfer_priv(itransfer);
   struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   struct libusb_device *dev        = NULL;
   struct device_priv *dpriv        = NULL;
   APIRET rc = NO_ERROR;
   SEMRECORD semRec = {0};

   if (!transfer->dev_handle)
   {
       return (LIBUSB_ERROR_INVALID_PARAM);
   }

   tpriv->hEventSem = NULLHANDLE;
   rc = DosCreateEventSem(NULL,&tpriv->hEventSem,DC_SEM_SHARED,FALSE);
   usbi_dbg("DosCreateEventSem, rc:%lu",rc);
   if (NO_ERROR != rc)
   {
       return(_apiret_to_libusb(rc));
   }

   dev = transfer->dev_handle->dev;
   dpriv = (struct device_priv *)usbi_get_device_priv(dev);

   tpriv->Processed                = 0;
   tpriv->ToProcess                = transfer->length < MAX_TRANSFER_SIZE ? transfer->length : MAX_TRANSFER_SIZE;
   tpriv->Response.usStatus        = 0;
   tpriv->Response.usDataLength    = (USHORT)tpriv->ToProcess;
   tpriv->status                   = LIBUSB_TRANSFER_COMPLETED; /* preset with a safe value, different from LIBUSB_TRANSFER_CANCELLED */
   tpriv->toCancel                 = FALSE;


   semRec.hsemCur = (HSEM)tpriv->hEventSem;
   semRec.ulUser  = (ULONG)itransfer;
   rc = DosAddMuxWaitSem(ghMux,&semRec);

   rc = UsbStartDataTransfer(dpriv->fd,transfer->endpoint,0,(PUCHAR)&tpriv->Response,transfer->buffer,tpriv->hEventSem,IS_XFERIN(transfer) ? 0 : USB_TRANSFER_FULL_SIZE);
   usbi_dbg("UsbStartDataTransfer with fd = %#.8lx",dpriv->fd);
   usbi_dbg("UsbStartDataTransfer with ep = %#02x",transfer->endpoint);
   usbi_dbg("UsbStartDataTransfer with timeout = %d",transfer->timeout);
   usbi_dbg("UsbStartDataTransfer rc = %lu",rc);

   if (NO_ERROR != rc)
   {
       DosDeleteMuxWaitSem(ghMux,(HSEM)tpriv->hEventSem);
       DosCloseEventSem(tpriv->hEventSem);
   }
   return(_apiret_to_libusb(rc));
}


static int _async_iso_transfer(struct usbi_transfer *itransfer)
{
   usbi_dbg(" ");

   struct transfer_priv *tpriv      = (struct transfer_priv *)usbi_get_transfer_priv(itransfer);
   struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
   struct libusb_device *dev        = NULL;
   struct device_priv *dpriv        = NULL;
   APIRET rc = NO_ERROR;
   unsigned int j = 0;
   unsigned int num_max_packets_per_execution = 0;
   unsigned int packet_len = 0;
   int length = 0;
   int iface = 0;
   int errorcode = LIBUSB_SUCCESS;
   SEMRECORD semRec = {0};

   if (!transfer->dev_handle)
   {
       return (LIBUSB_ERROR_INVALID_PARAM);
   }

   dev = transfer->dev_handle->dev;
   dpriv = (struct device_priv *)usbi_get_device_priv(dev);

   usbi_dbg("initial:num iso packets %u, buffer len %u",transfer->num_iso_packets,transfer->length);

   if (!transfer->num_iso_packets)
   {
      usbi_dbg("no iso packets defined");
      return (LIBUSB_ERROR_INVALID_PARAM);
   }

   if (transfer->num_iso_packets > MAX_NUM_ISO_PACKETS)
   {
      usbi_dbg("number of iso packets exceeds supported limit, num packets %u",transfer->num_iso_packets);
      return (LIBUSB_ERROR_INVALID_PARAM);
   }

   /*
    * preset all transfers to zero length and "success"
    * that will avoid libuvc based etc. applications in reporting
    * transfer errors where in reality we did not make any attempt to transfer at all
    */
   for (j=0,length=0;j<(unsigned int)transfer->num_iso_packets;j++,length += packet_len) {
      packet_len = transfer->iso_packet_desc[j].length;
      transfer->iso_packet_desc[j].actual_length = 0;
      transfer->iso_packet_desc[j].status        = LIBUSB_TRANSFER_COMPLETED;
   } /* endfor */
   if (transfer->length < length) {
      usbi_dbg("overall transfer length (%u) < sum packet lengths (%u)",transfer->length, length);
      return (LIBUSB_ERROR_INVALID_PARAM);
   }
   /* endif */

   iface = _interface_for_endpoint(dev,transfer->endpoint);
   if (iface < 0)
   {
      usbi_dbg("no interface to endpoint %#02x found",transfer->endpoint);
      return (LIBUSB_ERROR_INVALID_PARAM);
   } /* endif */

   if (!dpriv->altsetting[iface]) {
      usbi_dbg("cannot do isochronous transfer with altsetting = 0");
      return (LIBUSB_ERROR_INVALID_PARAM);
   }

   tpriv->hEventSem = NULLHANDLE;
   rc = DosCreateEventSem(NULL,&tpriv->hEventSem,DC_SEM_SHARED,FALSE);
   usbi_dbg("DosCreateEventSem, rc:%lu",rc);
   if (NO_ERROR != rc)
   {
       return(_apiret_to_libusb(rc));
   }


   tpriv->Processed                = 0;
   tpriv->ToProcess                = 0;
   tpriv->Response.usStatus        = 0;
   tpriv->Response.usDataLength    = 0;
   memset(tpriv->Response.usFrameSize,0,sizeof(tpriv->Response.usFrameSize));
   tpriv->status                   = LIBUSB_TRANSFER_COMPLETED; /* preset with a safe value, different from LIBUSB_TRANSFER_CANCELLED */
   tpriv->toCancel                 = FALSE;


   packet_len = transfer->iso_packet_desc[0].length;

   /*
    * IMPORTANT: for OS/2 we need to clip the overall length of a transfer request to MAX_ISO_TRANSFER_SIZE bytes
    * if we have more to transfer, we simply clip the list, every packet not served will return with zero size
    * and "success", see above
    */
   num_max_packets_per_execution = MAX_ISO_TRANSFER_SIZE / packet_len;
   /*
    * IMPORTANT: for every invocation of UsbStartIsoTransfer in conjunction with EHCI, the number of packets
    * has to be a multiple of 8 !!!
    * for UHCI/OHCI this does not matter but it's also not wrong
    */
   num_max_packets_per_execution = (num_max_packets_per_execution / 8U) * 8U;
   if (!num_max_packets_per_execution) num_max_packets_per_execution = 1;

   /* take device mutex: we need to manipulate per device control var "numIsoBuffsInUse" */
   usbi_mutex_lock(&dev->lock);

   if ((dpriv->numIsoBuffsInUse + 1) > NUM_ISO_BUFFS) {
      errorcode = LIBUSB_ERROR_OVERFLOW;
   }

   if (LIBUSB_SUCCESS == errorcode)
   {
      tpriv->ToProcess = (unsigned int)transfer->num_iso_packets < num_max_packets_per_execution ? (unsigned int)transfer->num_iso_packets : num_max_packets_per_execution;

      for (j=0,length=0;j< tpriv->ToProcess;j++,length += packet_len)
      {
         packet_len = transfer->iso_packet_desc[j].length;
         tpriv->Response.usFrameSize[j] = (USHORT)packet_len;
      }
      /* endfor */

      tpriv->Response.usStatus = 0;
      tpriv->Response.usDataLength = (USHORT)length;

      semRec.hsemCur = (HSEM)tpriv->hEventSem;
      semRec.ulUser  = (ULONG)itransfer;
      rc = DosAddMuxWaitSem(ghMux,&semRec);

      rc = UsbStartIsoTransfer(dpriv->fd,
                               transfer->endpoint,
                               dpriv->altsetting[iface],
                               tpriv->hEventSem,
                               (PUCHAR)&tpriv->Response,
                               transfer->buffer,
                               (USHORT)packet_len,
                               (USHORT)tpriv->ToProcess);
      usbi_dbg("UsbStartIsoTransfer with fd = %#.8lx",dpriv->fd);
      usbi_dbg("UsbStartIsoTransfer with iso ep = %#02x",transfer->endpoint);
      usbi_dbg("UsbStartIsoTransfer with usbcalls_timeout = %d",transfer->timeout);
      usbi_dbg("UsbStartIsoTransfer num iso packets %u, packet len %u, buffer len %u",tpriv->ToProcess,packet_len,length);
      usbi_dbg("UsbStartIsoTransfer rc = %lu",rc);

      errorcode = _apiret_to_libusb(rc);
   }

   if (NO_ERROR != rc)
   {
       DosDeleteMuxWaitSem(ghMux,(HSEM)tpriv->hEventSem);
       DosCloseEventSem(tpriv->hEventSem);
   }
   else
   {
      dpriv->numIsoBuffsInUse += 1;
   }

   usbi_dbg("Num Iso Buffers in use:%u, libusb error: %d",dpriv->numIsoBuffsInUse,errorcode);

   usbi_mutex_unlock(&dev->lock);

   return(errorcode);
}

