//*****************************************************************************
//
// usb_serial_structs.h - USB CDC device data structures for TM4C123G DAQ.
//
//*****************************************************************************

#ifndef USB_SERIAL_STRUCTS_H_
#define USB_SERIAL_STRUCTS_H_

//*****************************************************************************
// USB buffer sizes (must be power of 2, >= 2x max USB packet size of 64)
//*****************************************************************************
#define USB_RX_BUFFER_SIZE  256     // Host -> Device (commands)

//*****************************************************************************
// Extern declarations
//*****************************************************************************
extern uint32_t RxHandler(void *pvCBData, uint32_t ui32Event,
                           uint32_t ui32MsgValue, void *pvMsgData);
extern uint32_t TxHandler(void *pvCBData, uint32_t ui32Event,
                           uint32_t ui32MsgValue, void *pvMsgData);
extern uint32_t ControlHandler(void *pvCBData, uint32_t ui32Event,
                                uint32_t ui32MsgValue, void *pvMsgData);

extern tUSBBuffer g_sRxBuffer;
extern tUSBDCDCDevice g_sCDCDevice;
extern uint8_t g_pui8USBRxBuffer[];

#endif // USB_SERIAL_STRUCTS_H_
