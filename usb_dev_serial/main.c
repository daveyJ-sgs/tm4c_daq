//*****************************************************************************
//
// main.c - TM4C123G DAQ with USB CDC streaming
//
// Hardware: EK-TM4C123GXL (Tiva C Series LaunchPad)
// MCU:      TM4C123GH6PM, ARM Cortex-M4F @ 80MHz
//
// USB CDC virtual COM port streams 12-bit ADC samples at configurable rate.
// Connect device USB port (J2, side micro-B) to PC for data.
// Debug USB port (J1, top micro-B) for programming.
//
// Pin assignments:
//   PD4/PD5 - USB D-/D+ (device port J2)
//   PE3     - ADC0 AIN0 (analog input)
//   PB6     - M0PWM0 (PWM test output, wire to PE3 for self-test)
//   PF1     - Red LED (heartbeat)
//   PF2     - Blue LED (USB connected)
//   PF3     - Green LED (ADC activity)
//   PF4     - SW1 button (cycle PWM frequency)
//   PF0     - SW2 button (cycle PWM duty)
//
// Binary protocol (3 bytes per 2 × 12-bit samples):
//   Byte 0:  sA[11:4]  — clamped to 0xFE max; 0xFF reserved for commands
//   Byte 1:  sA[3:0] | sB[11:8]   (nibble boundary)
//   Byte 2:  sB[7:0]
//   Decode:  sA = (b0 << 4) | (b1 >> 4)
//            sB = ((b1 & 0x0F) << 8) | b2
//   Note:    sA is clipped at 4079 (3.287 V) to keep 0xFF free for framing
//
// Command packets (3 bytes, triplet-aligned):
//   [0xFF] [0x20 | freq_index] [0x00]   PWM frequency changed
//   [0xFF] [0x40 | duty_index] [0x00]   PWM duty changed
//   [0xFF] [0x60 | rate_index] [0x00]   ADC sample rate changed
//   [0xFF] [0x80 | oflow[11:8]] [oflow[7:0]]  ADC overflow delta report
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/usb.h"
#include "inc/hw_adc.h"
#include "inc/hw_udma.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"

//*****************************************************************************
// Configuration
//*****************************************************************************
#define SYSTICKS_PER_SECOND     100
#define SYS_CLOCK_HZ            80000000

// ADC circular buffer (power of 2, sized for 500kHz DMA streaming)
#define ADC_BUFFER_SIZE         8192
#define ADC_BUFFER_MASK         (ADC_BUFFER_SIZE - 1)

//*****************************************************************************
// PWM frequency presets — logarithmic spread from 100 Hz to 500 kHz
//
// PWM counter is 16-bit (max period 65536), so we select the clock divider
// per-frequency to keep good duty cycle resolution at all ranges.
//
//   Freq      Divider   PWM Clock    Period   Min duty step
//   100 Hz    /64       1.25 MHz     12500    0.008%
//   1 kHz     /2        40 MHz       40000    0.0025%
//   10 kHz    /1        80 MHz       8000     0.0125%
//   20 kHz    /1        80 MHz       4000     0.025%
//   40 kHz    /1        80 MHz       2000     0.05%
//*****************************************************************************
#define NUM_FREQ_PRESETS    5

typedef struct {
    uint32_t ui32FreqHz;
    uint32_t ui32ClkDiv;        // SYSCTL_PWMDIV_x
    uint32_t ui32Period;        // PWM clock ticks per cycle
} tPWMFreqPreset;

static const tPWMFreqPreset g_psFreqPresets[NUM_FREQ_PRESETS] = {
    {    100, SYSCTL_PWMDIV_64, 12500 },
    {   1000, SYSCTL_PWMDIV_2,  40000 },
    {  10000, SYSCTL_PWMDIV_1,   8000 },
    {  20000, SYSCTL_PWMDIV_1,   4000 },
    {  40000, SYSCTL_PWMDIV_1,   2000 },
};

//*****************************************************************************
// ADC sample-rate presets
//*****************************************************************************
#define NUM_SAMPLE_RATE_PRESETS 5

static const uint32_t g_pui32SampleRateHz[NUM_SAMPLE_RATE_PRESETS] = {
    100000,
    200000,
    250000,
    333333,
    400000,
};

//*****************************************************************************
// PWM duty cycle presets (percent)
//*****************************************************************************
#define NUM_DUTY_PRESETS    5
static const uint8_t g_pui8DutyPercent[NUM_DUTY_PRESETS] = {10, 25, 50, 75, 90};

//*****************************************************************************
// Global state
//*****************************************************************************

// ADC sample circular buffer
static volatile uint16_t g_pui16ADCBuffer[ADC_BUFFER_SIZE];
static volatile uint32_t g_ui32ADCHead = 0;
static volatile uint32_t g_ui32ADCTail = 0;
static volatile uint32_t g_ui32ADCOverflow = 0;
static volatile uint32_t g_ui32SampleCount = 0;
static volatile uint32_t g_ui32OverflowReported = 0;

// System state
static volatile uint32_t g_ui32SysTickCount = 0;
volatile bool g_bUSBConfigured = false;

// PWM state
static volatile uint8_t g_ui8FreqIndex = 2;    // Start at 10 kHz
static volatile uint8_t g_ui8DutyIndex = 2;    // Start at 50%
static volatile uint8_t g_ui8SampleRateIndex = 1; // Start at 200 kS/s

// Button debounce (two buttons)
#define BTN_FREQ    0       // SW1 (PF4) - cycle frequency
#define BTN_DUTY    1       // SW2 (PF0) - cycle duty
#define DEBOUNCE_TICKS  8   // 80ms at 100Hz SysTick

static struct {
    uint8_t  ui8State;      // Debounced state (1=released, active low)
    uint8_t  ui8Count;      // Debounce counter
    volatile bool bPressed; // Set on press, cleared by main loop
} g_sButtons[2] = {
    {1, 0, false},          // SW1
    {1, 0, false},          // SW2
};

// PC command state (set by RxHandler, processed in main loop)
static volatile bool g_bCmdFreq = false;
static volatile bool g_bCmdDuty = false;
static volatile bool g_bCmdSampleRate = false;
static volatile uint8_t g_ui8CmdFreqIndex = 0;
static volatile uint8_t g_ui8CmdDutyIndex = 0;
static volatile uint8_t g_ui8CmdSampleRateIndex = 0;

// USB batch write buffer for high-throughput streaming
// Must be a multiple of 3 (3 bytes per 2-sample triplet)
#define USB_BATCH_BYTES     768

// Bytes reserved at the top of the USB TX buffer exclusively for command packets.
// The data encoding loop stops early to keep this space free, guaranteeing
// SendUSBCommand always finds room even when the pipeline is fully loaded.
#define USB_CMD_RESERVE     24  // 8 command triplets
static uint8_t g_pui8USBBatch[USB_BATCH_BYTES];

// uDMA control table — must be 1024-byte aligned in SRAM
static uint8_t g_pui8DMAControlTable[1024] __attribute__((aligned(1024)));

// DMA ping-pong buffers (32-bit: ADC FIFO register is 32-bit)
#define DMA_BUFFER_SIZE     256
static uint32_t g_pui32DMABufA[DMA_BUFFER_SIZE];
static uint32_t g_pui32DMABufB[DMA_BUFFER_SIZE];

//*****************************************************************************
// Error handler (DEBUG builds only)
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1)
    {
    }
}
#endif

//*****************************************************************************
// PWM helper: apply current frequency and duty settings
//*****************************************************************************
static void
PWMApplySettings(void)
{
    const tPWMFreqPreset *psFreq = &g_psFreqPresets[g_ui8FreqIndex];
    uint32_t ui32PulseWidth;

    //
    // Compute pulse width from period and duty percentage
    //
    ui32PulseWidth = psFreq->ui32Period * g_pui8DutyPercent[g_ui8DutyIndex]
                     / 100;
    if(ui32PulseWidth < 1) ui32PulseWidth = 1;
    if(ui32PulseWidth >= psFreq->ui32Period) ui32PulseWidth = psFreq->ui32Period - 1;

    //
    // Disable output, reconfigure, re-enable
    //
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
    MAP_PWMGenDisable(PWM0_BASE, PWM_GEN_0);

    MAP_SysCtlPWMClockSet(psFreq->ui32ClkDiv);
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, psFreq->ui32Period);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ui32PulseWidth);

    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}

//*****************************************************************************
// Timer helper: apply current ADC sample-rate preset
//*****************************************************************************
static void
TimerApplySampleRate(void)
{
    uint32_t ui32Load =
        (SYS_CLOCK_HZ / g_pui32SampleRateHz[g_ui8SampleRateIndex]) - 1;

    MAP_TimerDisable(TIMER0_BASE, TIMER_A);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Load);
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}

//*****************************************************************************
// USB CDC Callbacks
//*****************************************************************************
uint32_t
ControlHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
    switch(ui32Event)
    {
        case USB_EVENT_CONNECTED:
            g_bUSBConfigured = true;
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
            break;

        case USB_EVENT_DISCONNECTED:
            g_bUSBConfigured = false;
            MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
            break;

        case USBD_CDC_EVENT_GET_LINE_CODING:
        case USBD_CDC_EVENT_SET_LINE_CODING:
            break;

        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
            //
            // Host opened (or closed) the COM port.  When DTR is asserted
            // the host just opened the port: flush any in-progress TX data so
            // the new reader starts at a clean triplet boundary.
            //
            if(ui32MsgValue & 0x01)     // bit 0 = DTR
            {
                USBBufferFlush(&g_sTxBuffer);
            }
            break;

        case USBD_CDC_EVENT_SEND_BREAK:
        case USBD_CDC_EVENT_CLEAR_BREAK:
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        default:
            break;
    }

    return(0);
}

uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    return(0);
}

uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    switch(ui32Event)
    {
        case USB_EVENT_RX_AVAILABLE:
        {
            uint8_t ui8Char;
            static uint8_t ui8RxState = 0;

            while(USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ui8Char, 1))
            {
                //
                // Two-byte command: [0xFE] [0x20|freq_idx] or [0xFE] [0x40|duty_idx]
                //
                if(ui8RxState == 0)
                {
                    if(ui8Char == 0xFE)
                        ui8RxState = 1;
                }
                else
                {
                    ui8RxState = 0;
                    if((ui8Char & 0x60) == 0x20)
                    {
                        uint8_t idx = ui8Char & 0x0F;
                        if(idx < NUM_FREQ_PRESETS)
                        {
                            g_ui8CmdFreqIndex = idx;
                            g_bCmdFreq = true;
                        }
                    }
                    else if((ui8Char & 0x60) == 0x40)
                    {
                        uint8_t idx = ui8Char & 0x0F;
                        if(idx < NUM_DUTY_PRESETS)
                        {
                            g_ui8CmdDutyIndex = idx;
                            g_bCmdDuty = true;
                        }
                    }
                    else if((ui8Char & 0x60) == 0x60)
                    {
                        uint8_t idx = ui8Char & 0x0F;
                        if(idx < NUM_SAMPLE_RATE_PRESETS)
                        {
                            g_ui8CmdSampleRateIndex = idx;
                            g_bCmdSampleRate = true;
                        }
                    }
                }
            }
            break;
        }

        case USB_EVENT_DATA_REMAINING:
            return(0);

        case USB_EVENT_REQUEST_BUFFER:
            return(0);

        default:
            break;
    }

    return(0);
}

//*****************************************************************************
// Debounce helper — called from SysTick for each button
//*****************************************************************************
static inline void
DebounceButton(uint32_t ui32Idx, uint8_t ui8Raw)
{
    if(ui8Raw != g_sButtons[ui32Idx].ui8State)
    {
        g_sButtons[ui32Idx].ui8Count++;
        if(g_sButtons[ui32Idx].ui8Count >= DEBOUNCE_TICKS)
        {
            g_sButtons[ui32Idx].ui8State = ui8Raw;
            g_sButtons[ui32Idx].ui8Count = 0;
            if(ui8Raw == 0)                 // Just pressed (active low)
            {
                g_sButtons[ui32Idx].bPressed = true;
            }
        }
    }
    else
    {
        g_sButtons[ui32Idx].ui8Count = 0;
    }
}

//*****************************************************************************
// SysTick ISR — button debounce and heartbeat LED
//*****************************************************************************
void
SysTickIntHandler(void)
{
    g_ui32SysTickCount++;

    //
    // Poll both buttons (active low: read != 0 means released)
    //
    uint32_t ui32PortF = MAP_GPIOPinRead(GPIO_PORTF_BASE,
                                          GPIO_PIN_4 | GPIO_PIN_0);
    DebounceButton(BTN_FREQ, (ui32PortF & GPIO_PIN_4) ? 1 : 0);
    DebounceButton(BTN_DUTY, (ui32PortF & GPIO_PIN_0) ? 1 : 0);

    //
    // Heartbeat: toggle red LED every 500ms
    //
    if((g_ui32SysTickCount % 50) == 0)
    {
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,
            MAP_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) ^ GPIO_PIN_1);
    }
}

//*****************************************************************************
// Process a completed DMA buffer into the ADC ring buffer
//*****************************************************************************
static void
ProcessDMABuffer(uint32_t *pui32Buf, uint32_t ui32Count)
{
    uint32_t i;
    for(i = 0; i < ui32Count; i++)
    {
        uint32_t ui32Next = (g_ui32ADCHead + 1) & ADC_BUFFER_MASK;
        if(ui32Next != g_ui32ADCTail)
        {
            g_pui16ADCBuffer[g_ui32ADCHead] = (uint16_t)(pui32Buf[i] & 0xFFF);
            g_ui32ADCHead = ui32Next;
        }
        else
        {
            g_ui32ADCOverflow += (ui32Count - i);
            break;
        }
    }
    g_ui32SampleCount += ui32Count;
}

//*****************************************************************************
// ADC0 Sequence 3 ISR — uDMA ping-pong completion
//
// With DMA enabled, this fires per-buffer (128 samples), NOT per sample.
// At 500 kHz: ~3.9 kHz interrupt rate (vs 500 kHz without DMA).
//*****************************************************************************
void
ADC0Seq3Handler(void)
{
    MAP_ADCIntClear(ADC0_BASE, 3);

    //
    // Check primary buffer (A) — UDMA_MODE_STOP means transfer complete
    //
    if(MAP_uDMAChannelModeGet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT)
       == UDMA_MODE_STOP)
    {
        ProcessDMABuffer(g_pui32DMABufA, DMA_BUFFER_SIZE);

        MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
            UDMA_MODE_PINGPONG,
            (void *)(ADC0_BASE + ADC_O_SSFIFO3),
            g_pui32DMABufA, DMA_BUFFER_SIZE);
    }

    //
    // Check alternate buffer (B)
    //
    if(MAP_uDMAChannelModeGet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT)
       == UDMA_MODE_STOP)
    {
        ProcessDMABuffer(g_pui32DMABufB, DMA_BUFFER_SIZE);

        MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT,
            UDMA_MODE_PINGPONG,
            (void *)(ADC0_BASE + ADC_O_SSFIFO3),
            g_pui32DMABufB, DMA_BUFFER_SIZE);
    }

    //
    // Toggle green LED every ~131072 samples (~0.7s at 192kHz, gentle blink)
    //
    if((g_ui32SampleCount & 0x1FFFF) == 0)
    {
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3,
            MAP_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) ^ GPIO_PIN_3);
    }
}

//*****************************************************************************
// Peripheral initialization
//*****************************************************************************
static void
ConfigureSystemClock(void)
{
    //
    // 80 MHz: 16MHz xtal -> PLL 400MHz -> /2 -> /2.5 = 80MHz
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                       SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
}

static void
ConfigureGPIO(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}

    //
    // Unlock PF0 (NMI pin) so it can be used as SW2 button input
    //
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    //
    // RGB LED outputs: PF1 (Red), PF2 (Blue), PF3 (Green)
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
                              GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    //
    // SW1 (PF4) and SW2 (PF0): active low, internal pull-up
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0);
    MAP_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // USB pins: PD4 (D-), PD5 (D+)
    //
    MAP_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);
}

static void
ConfigureADC(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}

    //
    // PE3 = AIN0
    //
    MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    //
    // ADC0 Sequencer 3: single sample, Timer0A trigger, highest priority
    //
    MAP_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                                 ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    MAP_ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Enable uDMA for SS3 — DMA request replaces per-sample CPU interrupt.
    // CPU interrupt fires only on DMA buffer completion (every 128 samples).
    //
    ADCSequenceDMAEnable(ADC0_BASE, 3);

    MAP_ADCIntEnable(ADC0_BASE, 3);
    MAP_IntEnable(INT_ADC0SS3);
}

static void
ConfigureDMA(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA)) {}

    MAP_uDMAEnable();
    MAP_uDMAControlBaseSet(g_pui8DMAControlTable);

    //
    // Channel 17 = ADC0 SS3 (default assignment on TM4C123G).
    // Clear all attributes, then enable high priority.
    //
    MAP_uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC3,
        UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
        UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
    MAP_uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC3,
        UDMA_ATTR_HIGH_PRIORITY);

    //
    // Primary control: ADC0 SS3 FIFO -> Buffer A
    // 32-bit transfers, source fixed, dest increment, arb after each transfer
    //
    MAP_uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
        UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 | UDMA_ARB_1);
    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
        UDMA_MODE_PINGPONG,
        (void *)(ADC0_BASE + ADC_O_SSFIFO3),
        g_pui32DMABufA, DMA_BUFFER_SIZE);

    //
    // Alternate control: ADC0 SS3 FIFO -> Buffer B
    //
    MAP_uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT,
        UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 | UDMA_ARB_1);
    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT,
        UDMA_MODE_PINGPONG,
        (void *)(ADC0_BASE + ADC_O_SSFIFO3),
        g_pui32DMABufB, DMA_BUFFER_SIZE);

    //
    // Enable the channel — DMA will start on first ADC trigger
    //
    MAP_uDMAChannelEnable(UDMA_CHANNEL_ADC3);
}

static void
ConfigureTimer(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}

    //
    // Full-width periodic timer, hardware ADC trigger output
    //
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    TimerApplySampleRate();
}

static void
ConfigurePWM(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {}

    //
    // PB6 = M0PWM0
    //
    MAP_GPIOPinConfigure(GPIO_PB6_M0PWM0);
    MAP_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    //
    // PWM0 Gen0: count-down mode, no sync
    //
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //
    // Apply initial frequency and duty (10 kHz, 50%)
    //
    PWMApplySettings();
}

static void
ConfigureUSB(void)
{
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);
    USBStackModeSet(0, eUSBModeForceDevice, 0);
    USBDCDCInit(0, &g_sCDCDevice);
}

//*****************************************************************************
// Send a 3-byte command triplet over USB (space guaranteed by USB_CMD_RESERVE)
//*****************************************************************************
static void
SendUSBCommand(uint8_t ui8Cmd)
{
    if(g_bUSBConfigured && USBBufferSpaceAvailable(&g_sTxBuffer) >= 3)
    {
        uint8_t pui8Pkt[3] = {0xFF, ui8Cmd, 0x00};
        USBBufferWrite(&g_sTxBuffer, pui8Pkt, 3);
    }
}

//*****************************************************************************
// Report a firmware overflow delta using one or more 12-bit status triplets
//*****************************************************************************
static uint32_t
SendUSBOverflowDelta(uint32_t ui32Delta)
{
    uint32_t ui32Sent = 0;

    while(ui32Delta && g_bUSBConfigured &&
          USBBufferSpaceAvailable(&g_sTxBuffer) >= 3)
    {
        uint16_t ui16Chunk = (ui32Delta > 0x0FFF) ? 0x0FFF : (uint16_t)ui32Delta;
        uint8_t pui8Pkt[3] = {
            0xFF,
            (uint8_t)(0x80 | ((ui16Chunk >> 8) & 0x0F)),
            (uint8_t)(ui16Chunk & 0xFF)
        };
        USBBufferWrite(&g_sTxBuffer, pui8Pkt, 3);
        ui32Delta -= ui16Chunk;
        ui32Sent += ui16Chunk;
    }

    return(ui32Sent);
}

//*****************************************************************************
// Main
//*****************************************************************************
int
main(void)
{
    MAP_FPULazyStackingEnable();

    ConfigureSystemClock();
    ConfigureGPIO();
    ConfigurePWM();
    ConfigureADC();
    ConfigureDMA();
    ConfigureTimer();

    //
    // SysTick for button debounce and heartbeat (100 Hz)
    //
    MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();

    ConfigureUSB();

    //
    // Interrupt priorities: ADC/DMA completion must re-arm ping-pong promptly.
    // Lower number = higher priority. TM4C123 uses top 3 bits (0x00-0xE0).
    //
    MAP_IntPrioritySet(INT_ADC0SS3, 0x00);      // Highest: ADC sampling
    MAP_IntPrioritySet(INT_USB0, 0x40);          // Medium:  USB transport
    MAP_IntPrioritySet(FAULT_SYSTICK, 0x80);     // Lower:   buttons/heartbeat

    MAP_IntMasterEnable();

    //
    // Main loop
    //
    while(1)
    {
        //
        // Batch-drain ADC ring buffer into USB for max throughput.
        // Encode pairs of samples as 3-byte triplets (1.5 bytes/sample),
        // write up to USB_BATCH_BYTES per iteration in one call.
        //
        if(g_bUSBConfigured)
        {
            uint32_t ui32Tail = g_ui32ADCTail;
            uint32_t ui32Head;

            while(ui32Tail != (ui32Head = g_ui32ADCHead))
            {
                uint32_t ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);
                if(ui32Space <= USB_CMD_RESERVE + 2)
                    break;

                // Round limit down to a triplet boundary, keeping CMD_RESERVE free
                uint32_t ui32DataSpace = ui32Space - USB_CMD_RESERVE;
                uint32_t ui32Limit = (ui32DataSpace < USB_BATCH_BYTES)
                                     ? (ui32DataSpace / 3 * 3)
                                     : USB_BATCH_BYTES;
                uint32_t ui32Bytes = 0;

                // Encode pairs of ADC samples: 2 × 12-bit → 3 bytes
                while((ui32Bytes + 3 <= ui32Limit) &&
                      (ui32Tail != ui32Head) &&
                      (((ui32Tail + 1) & ADC_BUFFER_MASK) != ui32Head))
                {
                    uint16_t sA = g_pui16ADCBuffer[ui32Tail];
                    ui32Tail = (ui32Tail + 1) & ADC_BUFFER_MASK;
                    uint16_t sB = g_pui16ADCBuffer[ui32Tail];
                    ui32Tail = (ui32Tail + 1) & ADC_BUFFER_MASK;
                    // 0xFF is reserved for command triplets, so sA[11:4] must
                    // never reach 0xFF.  Clamp the whole sample rather than just
                    // its high byte: clamping the byte alone folds codes
                    // 4080-4095 back down by 16 counts (non-monotonic).  This
                    // saturates them at 4079 (3.287 V) instead.
                    if(sA > 4079) { sA = 4079; }
                    g_pui8USBBatch[ui32Bytes++] = (uint8_t)(sA >> 4);
                    g_pui8USBBatch[ui32Bytes++] = (uint8_t)((sA << 4) | (sB >> 8));
                    g_pui8USBBatch[ui32Bytes++] = (uint8_t)(sB);
                }

                if(ui32Bytes == 0)
                    break;

                USBBufferWrite(&g_sTxBuffer, g_pui8USBBatch, ui32Bytes);
                g_ui32ADCTail = ui32Tail;
            }

            if(g_ui32ADCOverflow != g_ui32OverflowReported)
            {
                uint32_t ui32PendingOverflow =
                    g_ui32ADCOverflow - g_ui32OverflowReported;
                g_ui32OverflowReported += SendUSBOverflowDelta(ui32PendingOverflow);
            }
        }
        else
        {
            //
            // Not connected: discard samples to prevent overflow
            //
            g_ui32ADCTail = g_ui32ADCHead;
            g_ui32OverflowReported = g_ui32ADCOverflow;
        }

        //
        // SW1: cycle PWM frequency
        //
        if(g_sButtons[BTN_FREQ].bPressed)
        {
            g_sButtons[BTN_FREQ].bPressed = false;
            g_ui8FreqIndex = (g_ui8FreqIndex + 1) % NUM_FREQ_PRESETS;
            PWMApplySettings();
            SendUSBCommand(0x20 | g_ui8FreqIndex);
        }

        //
        // SW2: cycle PWM duty
        //
        if(g_sButtons[BTN_DUTY].bPressed)
        {
            g_sButtons[BTN_DUTY].bPressed = false;
            g_ui8DutyIndex = (g_ui8DutyIndex + 1) % NUM_DUTY_PRESETS;
            PWMApplySettings();
            SendUSBCommand(0x40 | g_ui8DutyIndex);
        }

        //
        // PC commands: set frequency
        //
        if(g_bCmdFreq)
        {
            g_bCmdFreq = false;
            g_ui8FreqIndex = g_ui8CmdFreqIndex;
            PWMApplySettings();
            SendUSBCommand(0x20 | g_ui8FreqIndex);
        }

        //
        // PC commands: set duty cycle
        //
        if(g_bCmdDuty)
        {
            g_bCmdDuty = false;
            g_ui8DutyIndex = g_ui8CmdDutyIndex;
            PWMApplySettings();
            SendUSBCommand(0x40 | g_ui8DutyIndex);
        }

        //
        // PC commands: set ADC sample rate
        //
        if(g_bCmdSampleRate)
        {
            g_bCmdSampleRate = false;
            g_ui8SampleRateIndex = g_ui8CmdSampleRateIndex;
            TimerApplySampleRate();
            SendUSBCommand(0x60 | g_ui8SampleRateIndex);
        }
    }
}
