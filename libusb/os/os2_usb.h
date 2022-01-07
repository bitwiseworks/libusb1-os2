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

/* speed definitions from USBD.SYS / USBCALLS */
#define USB_SPEED_FULL                    0
#define USB_SPEED_LOW                     1
#define USB_SPEED_HIGH                    2
#define USB_SPEED_SUPER                   3

#define MAX_NUM_ISO_PACKETS              64
#define NUM_ISO_BUFFS                     8

#define PACKET_SIZE_MASK            0x07FFU
#define PACKET_MUL_MASK             0x1800U
#define PACKET_MUL_SHIFT            11

#define MAX_TRANSFER_SIZE           8192  /* this is the value that the ArcaOS version of USBCALLS supports as the maximum */
#define MAX_ISO_TRANSFER_SIZE       61440

#define MAX_TRANSFERS               128

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

struct device_priv
{
   unsigned int            numOpens;                        /* tracks the number of nested calls to os2_open */
   unsigned int            numIsoBuffsInUse;                /* only used for isochronous devices */
   unsigned long           fd;                              /* device file descriptor */
   struct libusb_config_descriptor *curr_config_descriptor; /* pointer to the parsed configuration */
   uint8_t                 altsetting[USB_MAXINTERFACES];   /* remembers what alternate setting was chosen for a given interface */
   uint8_t                 endpoint[USB_MAXINTERFACES];     /* remembers what endpoint was chosen for a given interface */
   unsigned char           cdesc[4096];                     /* active config descriptor */
};

struct transfer_mgmt
{
    HEV hEventSem;
    pthread_t thrd;
    struct usbi_transfer *itransfer;
    BOOL toTerminate;
    BOOL inProgress;
};

struct transfer_priv
{
   unsigned char       protect1[PAGE_SIZE];        /* USBRESMG quirk, we need to ensure that "Response" lives in its own page */
   USBCALLS_MY_RSP     Response;                   /* structure to manage individual transfers, extended by frame size list to support iso */
   unsigned char       protect2[PAGE_SIZE];        /* USBRESMG quirk, we need to ensure that "Response" lives in its own page */
   unsigned int        ToProcess;                  /* bytes for control/bulk/irq, packets for iso */
   unsigned int        Processed;                  /* bytes for control/bulk/irq, packets for iso */
   enum libusb_transfer_status status;
};
#pragma pack()

