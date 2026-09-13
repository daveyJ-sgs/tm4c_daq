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
//   [0xFF] [0xA0 | recoveries[3:0]] [0x00]     Acquisition stall recovered
//   [0xFF] [0xC0 | sub] [data]                 Burst/trigger, see below
//
// Burst capture and trigger
// -------------------------
// Streaming and burst are mutually exclusive: both use g_pui16ADCBuffer, and
// with ~1.1 KB of SRAM free there is no room for a second capture buffer.
//
// Device -> host burst subcodes (0xC0 | sub):
//   0x0 STATE    0=stream 1=idle 2=armed 3=triggered 4=full 5=draining
//   0x1 LEN_HI   0x2 LEN_LO      capture length in samples
//   0x3 TRIG_HI  0x4 TRIG_LO     trigger offset within the frame
//   0x5 RATE     sample-rate preset index used for the capture
//   0x6 FLAGS    bit0 slope (0=rising), bit1 trigger was forced/auto
//   0x8 LVL_HI   0x9 LVL_LO      trigger level this capture actually used
//   0x7 BEGIN    frame payload starts at the very next triplet
//
// Host -> device, four bytes with a 16-bit big-endian argument:
//   [0xFE] [0xC0 | sub] [argHi] [argLo]
//   0x0 ACTION   0=disarm 1=arm single 2=arm continuous 3=force trigger
//   0x1 LEVEL    trigger level 0-4095
//   0x2 SLOPE    0=rising 1=falling
//   0x3 PRE_PCT  pre-trigger window as a percent of capture length
//   0x4 LENGTH   capture length in samples (64-8192, forced even)
//   0x5 AUTO_MS  auto-trigger timeout in ms, 0 = wait indefinitely
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
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

// Acquisition watchdog: SysTick ticks of no new samples before we assume the
// uDMA ping-pong has stalled and rebuild it.  At the slowest preset (100 kS/s)
// a buffer completes every 2.6 ms, so 10 ticks (100 ms) cannot false-trip.
#define WATCHDOG_TICKS          10

// Burst capture limits.  The shortest capture is bounded so a frame is always
// worth drawing, and the longest is the whole shared buffer.
#define BURST_MIN_SAMPLES       64
#define BURST_MAX_SAMPLES       ADC_BUFFER_SIZE

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

// Acquisition watchdog state (g_ui32WatchdogLast/Ticks are SysTick-private)
static volatile bool     g_bPipelineStalled = false;
static volatile uint32_t g_ui32StallRecoveries = 0;
static uint32_t          g_ui32WatchdogLast = 0;
static uint8_t           g_ui8WatchdogTicks = 0;

//
// Burst capture / trigger state.
//
// g_pui16ADCBuffer is shared with streaming, so only one of the two can be
// live at a time.  In burst mode the same memory is a rolling pre-trigger
// window: samples are written circularly while ARMED, and on the trigger edge
// we remember where the record starts and count down the post-trigger
// samples.  The record is therefore always contiguous modulo ADC_BUFFER_MASK
// and the trigger always lands at offset g_ui32ActivePre.
//
typedef enum
{
    BURST_OFF = 0,          // streaming; burst subsystem idle
    BURST_IDLE,             // burst engaged, not armed; samples discarded
    BURST_ARMED,            // filling the pre-trigger window, hunting the edge
    BURST_TRIGGERED,        // edge found, filling the post-trigger window
    BURST_FULL,             // capture complete, buffer frozen
    BURST_DRAINING          // sending the frame to the host
}
tBurstState;

static volatile tBurstState g_eBurstState    = BURST_OFF;
static tBurstState          g_eReportedState = BURST_OFF;

// Configuration.  Written by the RX ISR at any time; snapshotted on arm, so a
// setting sent mid-capture takes effect on the next one rather than corrupting
// the record in flight.
static volatile uint16_t g_ui16TrigLevel  = 2048;
static volatile uint16_t g_ui16CaptureLen = BURST_MAX_SAMPLES;
static volatile uint16_t g_ui16AutoMs     = 0;
static volatile uint8_t  g_ui8PrePercent  = 50;
static volatile bool     g_bTrigFalling   = false;

// Live capture state (ISR-owned once armed).  Level and slope are copied here
// at arm time rather than read live: the ISR compares every sample against
// them, so a host that retunes the level mid-capture would otherwise change
// the threshold underneath a record already in progress, and the frame header
// would describe a trigger that never happened.
static volatile uint16_t g_ui16ActiveLevel  = 0;
static volatile bool     g_bActiveFalling   = false;
static volatile bool     g_bBurstContinuous = false;
static volatile bool     g_bForceTrigger    = false;
static volatile bool     g_bTrigWasForced   = false;
static volatile uint32_t g_ui32ActiveLen    = 0;
static volatile uint32_t g_ui32ActivePre    = 0;
static volatile uint32_t g_ui32BurstWrite   = 0;
static volatile uint32_t g_ui32BurstFill    = 0;
static volatile uint32_t g_ui32BurstStart   = 0;
static volatile uint32_t g_ui32PostRemain   = 0;
static volatile uint16_t g_ui16PrevSample   = 0;
static volatile uint32_t g_ui32ArmTick      = 0;
static volatile uint32_t g_ui32CaptureCount = 0;

// Drain cursor (main-loop private)
static uint32_t          g_ui32DrainPos  = 0;
static uint32_t          g_ui32DrainLeft = 0;

// Burst command staging (RX ISR -> main loop)
static volatile bool     g_bCmdBurst      = false;
static volatile uint8_t  g_ui8BurstAction = 0;

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

// USB batch encode buffer.  Must be a multiple of 3 (3 bytes per triplet).
#define USB_BATCH_BYTES     768

static uint8_t g_pui8USBBatch[USB_BATCH_BYTES];

//*****************************************************************************
// Transmit path
//
// usblib's tUSBBuffer is NOT used for transmit.  USBBufferWrite copies into
// its ring one byte at a time via USBRingBufWriteOne, and every single byte
// goes through UpdateIndexAtomic, which globally disables and re-enables
// interrupts.  That is 768 function calls and 768 CPSID/CPSIE pairs for one
// 768-byte batch: measured at 1,564 us per call and 69.9% of the CPU, against
// 17.4% for the entire USB interrupt handler.  That call, not the USB bus and
// not the number of packets in flight, was what held the link to ~350 kB/s --
// which is also why enabling double-packet buffering made no difference.
//
// We keep our own byte ring instead and hand the CDC class whole 64-byte
// packets.  Filling it is a memcpy and draining it is a memcpy.
//
// Single producer (main loop writes g_ui32TxHead), single consumer (the USB
// interrupt advances g_ui32TxTail), so no lock is needed on the indices
// themselves -- both are naturally aligned 32-bit stores.  The main loop does
// mask the USB interrupt around its own call to TxPumpPacket, because that
// one function is genuinely reachable from both contexts.
//*****************************************************************************
#define TX_RING_SIZE        4096            // power of 2
#define TX_RING_MASK        (TX_RING_SIZE - 1)
#define USB_PACKET_BYTES    64              // Full-Speed bulk max packet

static uint8_t           g_pui8TxRing[TX_RING_SIZE];
static uint8_t           g_pui8TxPacket[USB_PACKET_BYTES];
static volatile uint32_t g_ui32TxHead = 0;
static volatile uint32_t g_ui32TxTail = 0;

//
// Set from the USB interrupt, honoured by the main loop.  See TxRingFlush.
//
static volatile bool     g_bTxFlushPending = false;

// Pending status triplets (command echoes, overflow reports, stall recoveries).
//
// Status must never be dropped: a vanished overflow report hides data loss, and
// a lost command echo leaves the GUI's indicators disagreeing with the firmware.
// Reserving space at the top of the TX buffer did not work -- under sustained
// saturation the stream still consumed it and status went out at zero packets
// per second.  Queue status instead and hand it the buffer BEFORE bulk data, so
// it is retried until it fits rather than discarded.  The cost is negligible:
// at 400 kS/s the overflow rate needs ~39 triplets/s, ~120 B/s out of ~358 kB/s.
#define STATUS_QUEUE_SIZE   16      // power of 2
#define STATUS_QUEUE_MASK   (STATUS_QUEUE_SIZE - 1)
static volatile uint8_t g_pui8StatusQueue[STATUS_QUEUE_SIZE][2];
static volatile uint8_t g_ui8StatusHead = 0;
static volatile uint8_t g_ui8StatusTail = 0;


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
// Transmit ring helpers
//*****************************************************************************
static uint32_t
TxRingUsed(void)
{
    return((g_ui32TxHead - g_ui32TxTail) & TX_RING_MASK);
}

static uint32_t
TxRingFree(void)
{
    //
    // One slot is always left empty so that head == tail means empty rather
    // than ambiguously full.
    //
    return(TX_RING_MASK - TxRingUsed());
}

static void
TxRingWrite(const uint8_t *pui8Data, uint32_t ui32Len)
{
    uint32_t ui32At;
    uint32_t ui32First;
    uint32_t ui32Room = TxRingFree();

    //
    // Callers size their writes from TxRingFree() and must not over-ask, but
    // clamp anyway: overrunning here walks head past tail, and because the
    // wire protocol has no sync marker the host would never recover.  Keep
    // the clamp on a triplet boundary so a truncated write cannot shift every
    // following sample by a byte.
    //
    if(ui32Len > ui32Room)
    {
        ui32Len = ui32Room - (ui32Room % 3);
        if(ui32Len == 0)
        {
            return;
        }
    }

    ui32At = g_ui32TxHead & TX_RING_MASK;
    ui32First = TX_RING_SIZE - ui32At;

    if(ui32First > ui32Len)
    {
        ui32First = ui32Len;
    }

    memcpy(&g_pui8TxRing[ui32At], pui8Data, ui32First);
    if(ui32Len > ui32First)
    {
        memcpy(&g_pui8TxRing[0], pui8Data + ui32First, ui32Len - ui32First);
    }

    g_ui32TxHead = (g_ui32TxHead + ui32Len) & TX_RING_MASK;
}

//*****************************************************************************
// Hand one packet to the CDC class if it is idle and we have bytes.
//
// Reachable from the USB interrupt (on TX complete) and from the main loop
// (to restart the chain once it has drained).  The main loop masks INT_USB0
// around its call; the interrupt path cannot be preempted by the main loop.
//*****************************************************************************
static void
TxPumpPacket(void)
{
    uint32_t ui32Used = TxRingUsed();
    uint32_t ui32Len;
    uint32_t ui32At;
    uint32_t ui32First;

    if(ui32Used == 0)
    {
        return;
    }

    //
    // Zero means the class is still sending the previous packet.
    //
    if(USBDCDCTxPacketAvailable((void *)&g_sCDCDevice) == 0)
    {
        return;
    }

    ui32Len = (ui32Used > USB_PACKET_BYTES) ? USB_PACKET_BYTES : ui32Used;
    ui32At = g_ui32TxTail & TX_RING_MASK;
    ui32First = TX_RING_SIZE - ui32At;
    if(ui32First > ui32Len)
    {
        ui32First = ui32Len;
    }

    memcpy(g_pui8TxPacket, &g_pui8TxRing[ui32At], ui32First);
    if(ui32Len > ui32First)
    {
        memcpy(g_pui8TxPacket + ui32First, &g_pui8TxRing[0],
               ui32Len - ui32First);
    }

    if(USBDCDCPacketWrite((void *)&g_sCDCDevice, g_pui8TxPacket, ui32Len,
                          true) == ui32Len)
    {
        g_ui32TxTail = (g_ui32TxTail + ui32Len) & TX_RING_MASK;
    }
}

//*****************************************************************************
// Request that the transmit ring be emptied.
//
// Called from USB interrupt context (connect, and DTR when the host opens the
// port).  It must NOT zero the indices here: the main loop is the only
// producer, and a flush landing between its memcpy and its update of
// g_ui32TxHead would put the stale head back, making used jump to whatever
// happened to be in the ring.  The device would then transmit kilobytes of
// stale bytes and the host, which has no sync marker, would lose triplet
// alignment permanently.  Defer it to the producer.
//*****************************************************************************
static void
TxRingFlush(void)
{
    g_bTxFlushPending = true;
}

//*****************************************************************************
// Honour a pending flush.  Main loop only, and never mid-write by
// construction.  Masking the USB interrupt keeps the consumer out.
//*****************************************************************************
static void
TxRingServiceFlush(void)
{
    if(!g_bTxFlushPending)
    {
        return;
    }

    g_bTxFlushPending = false;

    MAP_IntDisable(INT_USB0);
    g_ui32TxHead = 0;
    g_ui32TxTail = 0;
    MAP_IntEnable(INT_USB0);

    //
    // Anything queued belonged to the previous reader.
    //
    g_ui8StatusTail = g_ui8StatusHead;
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
            TxRingFlush();
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
                TxRingFlush();
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

//*****************************************************************************
// CDC transmit-complete callback (USB interrupt context)
//
// usbdcdc clears its TX state to idle before calling us, so the next packet
// can be staged immediately from here.
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    if(ui32Event == USB_EVENT_TX_COMPLETE)
    {
        TxPumpPacket();
    }

    return(0);
}

//*****************************************************************************
// Apply a burst/trigger configuration word from the host (RX ISR context)
//
// Only ACTION needs main-loop work -- the rest are plain stores that the arm
// path snapshots, so they are safe to take at any point in a capture.
//*****************************************************************************
static void
BurstCommand(uint8_t ui8Sub, uint16_t ui16Arg)
{
    switch(ui8Sub)
    {
        case 0x0:                           // ACTION
            g_ui8BurstAction = (uint8_t)ui16Arg;
            g_bCmdBurst = true;
            break;

        case 0x1:                           // LEVEL
            g_ui16TrigLevel = (ui16Arg > 4095) ? 4095 : ui16Arg;
            break;

        case 0x2:                           // SLOPE
            g_bTrigFalling = (ui16Arg != 0);
            break;

        case 0x3:                           // PRE_PCT
            g_ui8PrePercent = (ui16Arg > 95) ? 95 : (uint8_t)ui16Arg;
            break;

        case 0x4:                           // LENGTH
        {
            uint16_t ui16Len = ui16Arg;
            if(ui16Len > BURST_MAX_SAMPLES) { ui16Len = BURST_MAX_SAMPLES; }
            if(ui16Len < BURST_MIN_SAMPLES) { ui16Len = BURST_MIN_SAMPLES; }
            g_ui16CaptureLen = (uint16_t)(ui16Len & 0xFFFE);
            break;
        }

        case 0x5:                           // AUTO_MS
            g_ui16AutoMs = ui16Arg;
            break;

        default:
            break;
    }
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
            static uint8_t ui8RxCode  = 0;
            static uint8_t ui8RxArgHi = 0;

            while(USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ui8Char, 1))
            {
                //
                // [0xFE] [code|index]                two-byte preset commands
                // [0xFE] [0xC0|sub] [argHi] [argLo]  four-byte burst commands
                //
                // The code byte is masked with 0xE0, not 0x60: the burst codes
                // set bit 7 and the old mask would have aliased 0xC0 onto
                // 0x40, turning a trigger setting into a duty-cycle change.
                //
                if(ui8RxState == 0)
                {
                    if(ui8Char == 0xFE)
                        ui8RxState = 1;
                }
                else if(ui8RxState == 1)
                {
                    if((ui8Char & 0xE0) == 0xC0)
                    {
                        ui8RxCode  = ui8Char;
                        ui8RxState = 2;
                    }
                    else
                    {
                        uint8_t idx = ui8Char & 0x0F;
                        ui8RxState = 0;
                        if((ui8Char & 0xE0) == 0x20)
                        {
                            if(idx < NUM_FREQ_PRESETS)
                            {
                                g_ui8CmdFreqIndex = idx;
                                g_bCmdFreq = true;
                            }
                        }
                        else if((ui8Char & 0xE0) == 0x40)
                        {
                            if(idx < NUM_DUTY_PRESETS)
                            {
                                g_ui8CmdDutyIndex = idx;
                                g_bCmdDuty = true;
                            }
                        }
                        else if((ui8Char & 0xE0) == 0x60)
                        {
                            if(idx < NUM_SAMPLE_RATE_PRESETS)
                            {
                                g_ui8CmdSampleRateIndex = idx;
                                g_bCmdSampleRate = true;
                            }
                        }
                    }
                }
                else if(ui8RxState == 2)
                {
                    ui8RxArgHi = ui8Char;
                    ui8RxState = 3;
                }
                else
                {
                    ui8RxState = 0;
                    BurstCommand(ui8RxCode & 0x0F,
                                 ((uint16_t)ui8RxArgHi << 8) | ui8Char);
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
    // Acquisition watchdog.  If both ping-pong halves are ever left in
    // UDMA_MODE_STOP -- an ISR delayed past a full buffer, or a debugger halt --
    // nothing re-arms them and sampling stops silently with USB still up.
    // Detect here; repair from the main loop where the ADC ISR can be masked.
    //
    if(g_ui32SampleCount == g_ui32WatchdogLast)
    {
        if(++g_ui8WatchdogTicks >= WATCHDOG_TICKS)
        {
            g_ui8WatchdogTicks = 0;
            g_bPipelineStalled = true;
        }
    }
    else
    {
        g_ui32WatchdogLast = g_ui32SampleCount;
        g_ui8WatchdogTicks = 0;
    }

    //
    // Auto-trigger timeout.  Scope "Auto" mode: if no edge turns up within
    // g_ui16AutoMs the capture proceeds anyway so the display keeps updating.
    // The ISR only honours the force once the pre-trigger window is full, so a
    // timeout shorter than the pre-fill simply fires as soon as it can.
    //
    if((g_eBurstState == BURST_ARMED) && (g_ui16AutoMs != 0))
    {
        uint32_t ui32Elapsed = (g_ui32SysTickCount - g_ui32ArmTick) *
                               (1000 / SYSTICKS_PER_SECOND);
        if(ui32Elapsed >= g_ui16AutoMs)
        {
            g_bForceTrigger = true;
        }
    }

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
BurstProcessBuffer(uint32_t *pui32Buf, uint32_t ui32Count)
{
    uint32_t i;

    for(i = 0; i < ui32Count; i++)
    {
        uint16_t ui16S = (uint16_t)(pui32Buf[i] & 0xFFF);

        switch(g_eBurstState)
        {
            case BURST_ARMED:
                g_pui16ADCBuffer[g_ui32BurstWrite] = ui16S;
                g_ui32BurstWrite = (g_ui32BurstWrite + 1) & ADC_BUFFER_MASK;

                if(g_ui32BurstFill < g_ui32ActivePre)
                {
                    //
                    // Still filling the pre-trigger window.  Do not look for
                    // the edge yet: triggering now would hand back a record
                    // with less history than the host asked for.
                    //
                    g_ui32BurstFill++;
                }
                else
                {
                    bool bEdge = g_bActiveFalling
                        ? ((g_ui16PrevSample >= g_ui16ActiveLevel) &&
                           (ui16S < g_ui16ActiveLevel))
                        : ((g_ui16PrevSample < g_ui16ActiveLevel) &&
                           (ui16S >= g_ui16ActiveLevel));

                    if(bEdge || g_bForceTrigger)
                    {
                        uint32_t ui32TrigSlot =
                            (g_ui32BurstWrite - 1) & ADC_BUFFER_MASK;

                        g_bTrigWasForced = (!bEdge);
                        g_bForceTrigger  = false;
                        g_ui32BurstStart =
                            (ui32TrigSlot - g_ui32ActivePre) & ADC_BUFFER_MASK;
                        g_ui32PostRemain =
                            g_ui32ActiveLen - g_ui32ActivePre - 1;
                        g_eBurstState = BURST_TRIGGERED;
                    }
                }
                break;

            case BURST_TRIGGERED:
                g_pui16ADCBuffer[g_ui32BurstWrite] = ui16S;
                g_ui32BurstWrite = (g_ui32BurstWrite + 1) & ADC_BUFFER_MASK;
                if(--g_ui32PostRemain == 0)
                {
                    g_ui32CaptureCount++;
                    g_eBurstState = BURST_FULL;
                }
                break;

            default:
                //
                // IDLE, FULL or DRAINING: keep converting so the acquisition
                // watchdog still sees progress, but discard.
                //
                break;
        }

        g_ui16PrevSample = ui16S;
    }
}

static void
ProcessDMABuffer(uint32_t *pui32Buf, uint32_t ui32Count)
{
    uint32_t i;

    if(g_eBurstState != BURST_OFF)
    {
        BurstProcessBuffer(pui32Buf, ui32Count);
        g_ui32SampleCount += ui32Count;
        return;
    }

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
    MAP_ADCIntClearEx(ADC0_BASE, ADC_INT_DMA_SS3);

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

    //
    // Interrupt on DMA completion rather than sequence completion.  With uDMA
    // driving SS3 the sequence interrupt is the wrong source: clearing it does
    // not acknowledge the DMA-done condition, so the two can drift apart.
    //
    MAP_ADCIntClearEx(ADC0_BASE, ADC_INT_DMA_SS3 | ADC_INT_SS3);
    MAP_ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS3);
    MAP_IntEnable(INT_ADC0SS3);
}

//*****************************************************************************
// (Re)program both uDMA ping-pong halves and restart the channel.
//
// Used both for first-time setup and by the watchdog to recover a stalled
// pipeline.  Safe to call with the channel already running provided the ADC
// interrupt is masked by the caller.
//*****************************************************************************
static void
ADCPipelineRearm(void)
{
    //
    // Stop the channel and drain anything left in the SS3 FIFO, so the first
    // DMA transfer after the restart is a fresh conversion rather than a stale
    // one captured while the pipeline was wedged.
    //
    MAP_uDMAChannelDisable(UDMA_CHANNEL_ADC3);

    while(!(HWREG(ADC0_BASE + ADC_O_SSFSTAT3) & ADC_SSFSTAT3_EMPTY))
    {
        (void)HWREG(ADC0_BASE + ADC_O_SSFIFO3);
    }

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
    // Enable the channel — DMA will start on the next ADC trigger
    //
    MAP_ADCIntClearEx(ADC0_BASE, ADC_INT_DMA_SS3);
    MAP_uDMAChannelEnable(UDMA_CHANNEL_ADC3);
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
    // Program both ping-pong halves and start the channel
    //
    ADCPipelineRearm();
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
    USBBufferInit(&g_sRxBuffer);
    USBStackModeSet(0, eUSBModeForceDevice, 0);
    USBDCDCInit(0, &g_sCDCDevice);
}

//*****************************************************************************
// Queue one status triplet [0xFF][ui8B1][ui8B2] for transmission
//*****************************************************************************
static bool
QueueStatus(uint8_t ui8B1, uint8_t ui8B2)
{
    uint8_t ui8Next = (g_ui8StatusHead + 1) & STATUS_QUEUE_MASK;

    //
    // Nothing may be interleaved between the BEGIN triplet and the end of a
    // burst frame -- the host takes the payload as one contiguous run, and a
    // stray echo would shift every sample after it by one triplet.  The frame
    // header is queued while the state is still BURST_FULL, so refusing every
    // caller here is safe.
    //
    if(g_eBurstState == BURST_DRAINING)
    {
        return(false);
    }

    if(ui8Next == g_ui8StatusTail)
    {
        return(false);              // queue full; caller retries later
    }

    g_pui8StatusQueue[g_ui8StatusHead][0] = ui8B1;
    g_pui8StatusQueue[g_ui8StatusHead][1] = ui8B2;
    g_ui8StatusHead = ui8Next;

    return(true);
}

//*****************************************************************************
// Queue a command echo
//*****************************************************************************
static void
SendUSBCommand(uint8_t ui8Cmd)
{
    QueueStatus(ui8Cmd, 0x00);
}

//*****************************************************************************
// Turn the outstanding overflow delta into queued 12-bit status triplets
//*****************************************************************************
static void
QueueOverflowDelta(void)
{
    while(g_ui32ADCOverflow != g_ui32OverflowReported)
    {
        uint32_t ui32Delta = g_ui32ADCOverflow - g_ui32OverflowReported;
        uint16_t ui16Chunk = (ui32Delta > 0x0FFF) ? 0x0FFF : (uint16_t)ui32Delta;

        if(!QueueStatus((uint8_t)(0x80 | ((ui16Chunk >> 8) & 0x0F)),
                        (uint8_t)(ui16Chunk & 0xFF)))
        {
            break;                  // queue full; the rest reports next pass
        }

        g_ui32OverflowReported += ui16Chunk;
    }
}

//*****************************************************************************
// Push queued status triplets into the USB TX buffer, oldest first
//*****************************************************************************
static void
ServiceStatusQueue(void)
{
    while(g_ui8StatusTail != g_ui8StatusHead)
    {
        uint8_t pui8Pkt[3];

        //
        // Only write a whole triplet.  A partial write would shift every later
        // sample by a byte and the protocol has no way to resynchronise.
        //
        if(TxRingFree() < 3)
        {
            return;
        }

        pui8Pkt[0] = 0xFF;
        pui8Pkt[1] = g_pui8StatusQueue[g_ui8StatusTail][0];
        pui8Pkt[2] = g_pui8StatusQueue[g_ui8StatusTail][1];

        TxRingWrite(pui8Pkt, 3);
        g_ui8StatusTail = (g_ui8StatusTail + 1) & STATUS_QUEUE_MASK;
    }
}

//*****************************************************************************
// Free slots in the status queue
//*****************************************************************************
static uint8_t
StatusQueueFree(void)
{
    return((uint8_t)((STATUS_QUEUE_SIZE - 1) -
                     ((g_ui8StatusHead - g_ui8StatusTail) & STATUS_QUEUE_MASK)));
}

//*****************************************************************************
// Arm a capture.  Snapshots the configuration so a setting that arrives
// mid-capture cannot change the geometry of the record being taken.
//*****************************************************************************
static void
BurstArm(bool bContinuous)
{
    uint32_t ui32Len = g_ui16CaptureLen;
    uint32_t ui32Pre = (ui32Len * g_ui8PrePercent) / 100;

    //
    // At least one pre-trigger sample, because the edge test needs a previous
    // sample that belongs to this capture, and at most len-2 so there is
    // always a post-trigger sample left to count down.
    //
    if(ui32Pre < 1)             { ui32Pre = 1; }
    if(ui32Pre > (ui32Len - 2)) { ui32Pre = ui32Len - 2; }

    MAP_IntDisable(INT_ADC0SS3);

    g_ui32ActiveLen    = ui32Len;
    g_ui32ActivePre    = ui32Pre;
    g_ui16ActiveLevel  = g_ui16TrigLevel;
    g_bActiveFalling   = g_bTrigFalling;
    g_ui32BurstWrite   = 0;
    g_ui32BurstFill    = 0;
    g_ui32PostRemain   = 0;
    g_bForceTrigger    = false;
    g_bTrigWasForced   = false;
    g_bBurstContinuous = bContinuous;
    g_ui32ArmTick      = g_ui32SysTickCount;
    g_eBurstState      = BURST_ARMED;

    MAP_IntEnable(INT_ADC0SS3);
}

//*****************************************************************************
// Queue the frame header.  Must be called while still in BURST_FULL, before
// QueueStatus starts refusing.
//*****************************************************************************
static void
BurstQueueHeader(void)
{
    uint8_t ui8Flags = (uint8_t)((g_bActiveFalling ? 0x01 : 0x00) |
                                 (g_bTrigWasForced ? 0x02 : 0x00));

    QueueStatus(0xC1, (uint8_t)(g_ui32ActiveLen >> 8));
    QueueStatus(0xC2, (uint8_t)(g_ui32ActiveLen & 0xFF));
    QueueStatus(0xC3, (uint8_t)(g_ui32ActivePre >> 8));
    QueueStatus(0xC4, (uint8_t)(g_ui32ActivePre & 0xFF));
    QueueStatus(0xC5, g_ui8SampleRateIndex);
    QueueStatus(0xC6, ui8Flags);
    QueueStatus(0xC8, (uint8_t)(g_ui16ActiveLevel >> 8));
    QueueStatus(0xC9, (uint8_t)(g_ui16ActiveLevel & 0xFF));
    QueueStatus(0xC7, 0x00);                // BEGIN, always last
}

//*****************************************************************************
// Push one batch of the captured frame.  Returns true when the frame is done.
//
// Bounded like the streaming drain, and for the same reason: USBBufferWrite
// frees space inside the call, so a loop that only exits on "buffer full"
// never exits at all.
//*****************************************************************************
static bool
BurstDrainBatch(void)
{
    uint32_t ui32Space = TxRingFree();
    uint32_t ui32Limit;
    uint32_t ui32Bytes = 0;

    if(ui32Space < 3)
    {
        return(false);
    }

    ui32Limit = (ui32Space < USB_BATCH_BYTES) ? (ui32Space / 3 * 3)
                                              : USB_BATCH_BYTES;

    while((ui32Bytes + 3 <= ui32Limit) && (g_ui32DrainLeft >= 2))
    {
        uint16_t sA = g_pui16ADCBuffer[g_ui32DrainPos];
        g_ui32DrainPos = (g_ui32DrainPos + 1) & ADC_BUFFER_MASK;
        uint16_t sB = g_pui16ADCBuffer[g_ui32DrainPos];
        g_ui32DrainPos = (g_ui32DrainPos + 1) & ADC_BUFFER_MASK;
        g_ui32DrainLeft -= 2;

        if(sA > 4079) { sA = 4079; }
        g_pui8USBBatch[ui32Bytes++] = (uint8_t)(sA >> 4);
        g_pui8USBBatch[ui32Bytes++] = (uint8_t)((sA << 4) | (sB >> 8));
        g_pui8USBBatch[ui32Bytes++] = (uint8_t)(sB);
    }

    if(ui32Bytes)
    {
        TxRingWrite(g_pui8USBBatch, ui32Bytes);
    }

    return(g_ui32DrainLeft == 0);
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
        // Watchdog tripped: the ping-pong stalled and sampling has stopped.
        // Rebuild it with the ADC interrupt masked so the ISR cannot observe a
        // half-programmed channel.
        //
        if(g_bPipelineStalled)
        {
            g_bPipelineStalled = false;
            MAP_IntDisable(INT_ADC0SS3);
            ADCPipelineRearm();
            MAP_IntEnable(INT_ADC0SS3);
            g_ui32StallRecoveries++;
            SendUSBCommand(0xA0 | (uint8_t)(g_ui32StallRecoveries & 0x0F));
        }

        //
        // Honour a deferred transmit-ring flush before writing anything.
        //
        TxRingServiceFlush();

        //
        // Report burst state transitions from here rather than from the ISR:
        // the status queue is single-producer by design.  DRAINING is skipped
        // because the host infers it from the header it has just received.
        //
        if(g_bUSBConfigured && (g_eBurstState != g_eReportedState) &&
           (g_eBurstState != BURST_DRAINING))
        {
            if(QueueStatus(0xC0, (uint8_t)g_eBurstState))
            {
                g_eReportedState = g_eBurstState;
            }
        }

        //
        // Burst mode owns g_pui16ADCBuffer while it is engaged, so the
        // streaming path below is skipped entirely.
        //
        if(g_eBurstState != BURST_OFF)
        {
            if(g_bUSBConfigured)
            {
                if(g_eBurstState == BURST_FULL)
                {
                    //
                    // Hand the whole header to the queue in one go, then lock
                    // it by entering DRAINING.  Wait for room rather than
                    // emitting half a header.
                    //
                    ServiceStatusQueue();
                    if(StatusQueueFree() >= 9)
                    {
                        BurstQueueHeader();
                        g_ui32DrainPos   = g_ui32BurstStart;
                        g_ui32DrainLeft  = g_ui32ActiveLen;
                        g_eBurstState    = BURST_DRAINING;
                        g_eReportedState = BURST_DRAINING;
                    }
                }
                else if(g_eBurstState == BURST_DRAINING)
                {
                    //
                    // Header out in full before the first payload byte.
                    //
                    if(g_ui8StatusTail != g_ui8StatusHead)
                    {
                        ServiceStatusQueue();
                    }
                    else if(BurstDrainBatch())
                    {
                        if(g_bBurstContinuous)
                        {
                            BurstArm(true);
                        }
                        else
                        {
                            g_eBurstState = BURST_IDLE;
                        }
                    }
                }
                else
                {
                    ServiceStatusQueue();
                }
            }
            else
            {
                //
                // Host vanished mid-frame.  Drop the partial transfer: the
                // reader that reconnects has no way to resynchronise onto the
                // middle of a record.
                //
                g_ui8StatusTail = g_ui8StatusHead;
                if(g_eBurstState == BURST_DRAINING)
                {
                    g_eBurstState = BURST_IDLE;
                }
            }
        }
        //
        // Batch-drain ADC ring buffer into USB for max throughput.
        // Encode pairs of samples as 3-byte triplets (1.5 bytes/sample),
        // write up to USB_BATCH_BYTES per iteration in one call.
        //
        else if(g_bUSBConfigured)
        {
            uint32_t ui32Tail = g_ui32ADCTail;
            uint32_t ui32Head = g_ui32ADCHead;
            uint32_t ui32Space;

            //
            // Status first, then exactly ONE batch of samples.
            //
            // The drain loop must stay bounded.  Under saturation the ADC ring
            // never empties, and USBBufferWrite hands bytes straight to the
            // endpoint so TX space never runs out either -- so a loop that
            // exits only on "ring empty" or "buffer full" never exits at all.
            // It spins here writing data and never returns to the top of the
            // main loop, which is why status previously went out at exactly
            // zero packets per second no matter how much space was reserved
            // for it.  One batch per pass keeps status serviced; at 768 bytes
            // a pass the loop still outruns the USB link by orders of
            // magnitude.
            //
            QueueOverflowDelta();
            ServiceStatusQueue();

            //
            // Read the free space AFTER servicing status: those writes go
            // into this same ring, so a figure taken before them is stale by
            // up to a full status queue.
            //
            ui32Space = TxRingFree();

            //
            // Samples waiting in the ADC ring.
            //
            uint32_t ui32Avail = (ui32Head - ui32Tail) & ADC_BUFFER_MASK;

            //
            // Encode a whole batch when a whole batch is there.  The loop runs
            // far faster than the ADC produces, so without this it would wake
            // up to ~70 samples every pass and pay the full per-call cost to
            // move a hundred bytes -- measured at 58% of the CPU.
            //
            // The trickle case keeps a slow sample rate alive: if the
            // transmit ring is down to less than a packet, send whatever is
            // available rather than waiting for a batch that may be a long
            // time coming.
            //
            if((ui32Space >= 3) &&
               (((ui32Avail >= (USB_BATCH_BYTES / 3 * 2)) &&
                 (ui32Space >= USB_BATCH_BYTES)) ||
                ((TxRingUsed() < USB_PACKET_BYTES) && (ui32Avail >= 2))))
            {
                // Round the limit down to a whole number of triplets
                uint32_t ui32Limit = (ui32Space < USB_BATCH_BYTES)
                                     ? (ui32Space / 3 * 3)
                                     : USB_BATCH_BYTES;
                uint32_t ui32Bytes = 0;

                // Encode pairs of ADC samples: 2 x 12-bit -> 3 bytes
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

                if(ui32Bytes)
                {
                    TxRingWrite(g_pui8USBBatch, ui32Bytes);
                    g_ui32ADCTail = ui32Tail;
                }
            }
        }
        else
        {
            //
            // Not connected: discard samples to prevent overflow
            //
            g_ui32ADCTail = g_ui32ADCHead;
            g_ui32OverflowReported = g_ui32ADCOverflow;
            g_ui8StatusTail = g_ui8StatusHead;   // discard stale status
        }

        //
        // Keep the packet chain running.  usblib's buffer layer used to
        // restart transmission for us; without it, nothing else will once the
        // class goes idle with bytes still queued.  Masking the USB interrupt
        // here is the only place the transmit path takes a lock at all -- once
        // per main-loop pass, against once per byte before.
        //
        if(g_bUSBConfigured)
        {
            MAP_IntDisable(INT_USB0);
            TxPumpPacket();
            MAP_IntEnable(INT_USB0);
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
        // PC commands: burst / trigger action
        //
        //
        // A burst action taken mid-frame would abandon the payload partway
        // through.  The host counts the payload out by length and has no
        // resynchronisation point inside it, so it would swallow whatever
        // came next -- including the very state message announcing the
        // change -- and only recover once it had eaten a frame's worth of
        // unrelated bytes.  Hold the action until the frame is out; the
        // worst case is one frame of latency, ~34 ms at 8192 samples.
        //
        if(g_bCmdBurst && (g_eBurstState != BURST_DRAINING))
        {
            uint8_t ui8Action = g_ui8BurstAction;
            g_bCmdBurst = false;

            switch(ui8Action)
            {
                case 0:                         // disarm, resume streaming
                    MAP_IntDisable(INT_ADC0SS3);
                    g_eBurstState = BURST_OFF;
                    g_ui32ADCTail = g_ui32ADCHead;
                    MAP_IntEnable(INT_ADC0SS3);
                    g_ui32OverflowReported = g_ui32ADCOverflow;
                    break;

                case 1:                         // arm, single shot
                    BurstArm(false);
                    break;

                case 2:                         // arm, auto-rearm after drain
                    BurstArm(true);
                    break;

                case 3:                         // force the trigger now
                    g_bForceTrigger = true;
                    break;

                default:
                    break;
            }
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
