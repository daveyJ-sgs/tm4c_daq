//*****************************************************************************
//
// usb_serial_structs.c - USB CDC device descriptors and buffers for DAQ.
//
// Based on TivaWare usb_dev_serial example, customized for DAQ streaming.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"

//*****************************************************************************
// USB string descriptors
//*****************************************************************************
const uint8_t g_pui8LangDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

const uint8_t g_pui8ManufacturerString[] =
{
    (17 + 1) * 2,
    USB_DTYPE_STRING,
    'T', 0, 'e', 0, 'x', 0, 'a', 0, 's', 0, ' ', 0, 'I', 0, 'n', 0,
    's', 0, 't', 0, 'r', 0, 'u', 0, 'm', 0, 'e', 0, 'n', 0, 't', 0,
    's', 0,
};

const uint8_t g_pui8ProductString[] =
{
    2 + (12 * 2),
    USB_DTYPE_STRING,
    'T', 0, 'M', 0, '4', 0, 'C', 0, '1', 0, '2', 0, '3', 0, 'G', 0,
    ' ', 0, 'D', 0, 'A', 0, 'Q', 0
};

const uint8_t g_pui8SerialNumberString[] =
{
    2 + (8 * 2),
    USB_DTYPE_STRING,
    'D', 0, 'A', 0, 'Q', 0, '0', 0, '0', 0, '0', 0, '0', 0, '1', 0
};

const uint8_t g_pui8ControlInterfaceString[] =
{
    2 + (21 * 2),
    USB_DTYPE_STRING,
    'A', 0, 'C', 0, 'M', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 't', 0,
    'r', 0, 'o', 0, 'l', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0,
    'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0
};

const uint8_t g_pui8ConfigString[] =
{
    2 + (26 * 2),
    USB_DTYPE_STRING,
    'S', 0, 'e', 0, 'l', 0, 'f', 0, ' ', 0, 'P', 0, 'o', 0, 'w', 0,
    'e', 0, 'r', 0, 'e', 0, 'd', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0,
    'f', 0, 'i', 0, 'g', 0, 'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0,
    'o', 0, 'n', 0
};

const uint8_t * const g_ppui8StringDescriptors[] =
{
    g_pui8LangDescriptor,
    g_pui8ManufacturerString,
    g_pui8ProductString,
    g_pui8SerialNumberString,
    g_pui8ControlInterfaceString,
    g_pui8ConfigString
};

#define NUM_STRING_DESCRIPTORS (sizeof(g_ppui8StringDescriptors) /             \
                                sizeof(uint8_t *))

//*****************************************************************************
// CDC device instance
//*****************************************************************************
tUSBDCDCDevice g_sCDCDevice =
{
    USB_VID_TI_1CBE,
    USB_PID_SERIAL,
    0,
    USB_CONF_ATTR_SELF_PWR,
    ControlHandler,
    (void *)&g_sCDCDevice,
    USBBufferEventCallback,
    (void *)&g_sRxBuffer,
    TxHandler,
    (void *)0,
    g_ppui8StringDescriptors,
    NUM_STRING_DESCRIPTORS
};

//*****************************************************************************
// USB RX buffer (host -> device, commands)
//*****************************************************************************
uint8_t g_pui8USBRxBuffer[USB_RX_BUFFER_SIZE];
tUSBBuffer g_sRxBuffer =
{
    false,                          // Receive buffer
    RxHandler,                      // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data
    USBDCDCPacketRead,              // pfnTransfer
    USBDCDCRxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pui8USBRxBuffer,              // pui8Buffer
    USB_RX_BUFFER_SIZE,             // ui32BufferSize
};

//*****************************************************************************
// There is deliberately no TX tUSBBuffer.  main.c owns the transmit ring and
// feeds the CDC class whole packets; see the transmit path comment there for
// why usblib's buffer layer is not used on this side.
//*****************************************************************************
