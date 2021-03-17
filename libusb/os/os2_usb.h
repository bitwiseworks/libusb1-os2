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
  unsigned short usFrameSize[0];
} USBCALLS_MY_ISO_RSP, *PUSBCALLS_MY_ISO_RSP;
#pragma pack()

struct device_priv {
   int fd;                                /* device file descriptor */
   int altsetting[USB_MAXINTERFACES];     /* remembers what alternate setting was chosen for a given interface */
   int endpoint[USB_MAXINTERFACES];       /* remembers what endpoint was chosen for a given interface */
   unsigned char cdesc[4096];             /* active config descriptor */
};

struct transfer_priv {
   int currinterface;
   int curraltsetting;
};

