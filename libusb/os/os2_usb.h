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

#define PACKET_SIZE_MASK            0x07FFU
#define PACKET_MUL_MASK             0x1800U
#define PACKET_MUL_SHIFT            11

#define MAX_TRANSFER_SIZE           32768

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

typedef struct _USBCALLS_MY_RSP_
{ unsigned short usStatus;
  unsigned short usDataLength;
  unsigned short usFrameSize[MAX_NUM_ISO_PACKETS];
} USBCALLS_MY_RSP, *PUSBCALLS_MY_RSP;
#pragma pack()

struct device_priv {
   unsigned long fd;                        /* device file descriptor */
   unsigned long rmDevHandle;               /* the OS/2 Resource Manager device handle, a GUID */
   struct libusb_config_descriptor *curr_config_descriptor; /* pointer to the parsed configuration */
   uint8_t altsetting[USB_MAXINTERFACES];   /* remembers what alternate setting was chosen for a given interface */
   uint8_t endpoint[USB_MAXINTERFACES];     /* remembers what endpoint was chosen for a given interface */
   unsigned char cdesc[4096];               /* active config descriptor */
};

struct entry
{
    struct usbi_transfer *itransfer;
    STAILQ_ENTRY(entry) entries;
};
STAILQ_HEAD(stailhead, entry);


struct transfer_priv
{
   int                 ToProcess;
   int                 Processed;
   int                 numMaxPacketsPerExecution; /* only used for isochronous transfers */
   enum libusb_transfer_status status;
   HEV                 hEventSem;           /* used to wait for termination event, used for all transfers */
   USBCALLS_MY_RSP     Response;            /* structure to manage individual transfers, extended by frame size list to support iso */
   struct entry        element;             /* structure to allow linking into the queue of transfers to manage */
};

