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

#define MAX_NUM_ISO_PACKETS 4096

#pragma pack(1)
typedef struct _GETDEVINFODATA_
{
   unsigned char      ctrlID;              /* (00) controller ID */
   unsigned char      deviceAddress;       /* (01) USB device address */
   unsigned char      bConfigurationValue; /* (02) USB device configuration value */
   unsigned char      bInterfaceNumber;    /* (03) 0 based index in interface array for this item */
   unsigned char      SpeedDevice;         /* (04) 0 for full speed device, 1 - low speed device  2 - high speed device */
   unsigned char      portNum;             /* (05) port number to which device is attached */
   unsigned short     parentHubIndex;      /* (06) index in hub table to parent hub, -1 for root hub device */
   unsigned long      rmDevHandle;         /* (08) Resource Manager device handle */
} GETDEVINFODATA, *PGETDEVINFO;

typedef struct _USBCALLS_MY_ISO_RSP_
{ unsigned short usStatus;
  unsigned short usDataLength;
  unsigned short usFrameSize[MAX_NUM_ISO_PACKETS];
} USBCALLS_MY_ISO_RSP, *PUSBCALLS_MY_ISO_RSP;
#pragma pack()

struct device_priv {
   int fd;                                  /* device file descriptor */
   ULONG rmDevHandle;                       /* the OS/2 Resource Manager device handle, a GUID */
   HEV   hCancelSem;                        /* event semaphore used to signal and of transfer cancelling */
   struct libusb_config_descriptor *curr_config_descriptor;
   uint8_t initial_altsetting[USB_MAXINTERFACES]; /* remembers the alternate setting that was set initially on an isochronous transfer, updated only for isochronous transfers */
   uint8_t altsetting[USB_MAXINTERFACES];   /* remembers what alternate setting was chosen for a given interface */
   uint8_t endpoint[USB_MAXINTERFACES];     /* remembers what endpoint was chosen for a given interface */
   unsigned char cdesc[4096];               /* active config descriptor */
};

struct transfer_priv
{
   uint8_t altsetting;                      /* last used altsetting in last iso transfer, used only for iso */
   uint8_t endpoint;                        /* last used endpoint in last iso transfer, used only for iso */
   unsigned long IsoOpened;                 /* indicator if UsbIsoOpen has been called (UsbIsoClose will close) */
   HEV   hTransferSem;                      /* event semaphore used to signal end of transfer, used only for iso */
   USBCALLS_MY_ISO_RSP IsoResponse;         /* structure to manage individual iso packets, used only for iso */
};

