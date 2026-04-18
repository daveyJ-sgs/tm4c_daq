# MSP430 DAQ Project — Handoff Summary
## For continuation in Claude Code or new chat session

---

## Developer Profile
- Dave, experienced hardware/PCB engineer (analog background, high-speed digital, SI/PI)
- Age 51, home lab setup, expanding into firmware development
- Uses Code Composer Studio (CCS 20.4) on Windows 11
- Python 3.14 installed, familiar with Claude Code

---

## Completed Project: MSP430G2553 Real-Time DAQ System

### Hardware
- **Target MCU:** MSP430G2553 on MSP-EXP430G2 LaunchPad
- **FTDI cable:** TTL-232R-RPI (3.3V), connected to P1.1(RX) / P1.2(TX)
- **COM port:** COM4 @ 115200 baud
- **Jumper wire:** P2.4 → P1.4 (PWM output to ADC input)

### Firmware (C, CCS project: UART_GPIO_CLAUDE)
Four source files:

**hard_uart.h / hard_uart.c**
- USCI_A0 hardware UART, 115200 baud, 16MHz SMCLK
- Interrupt-driven TX (USCIAB0TX_VECTOR) and RX (USCIAB0RX_VECTOR)
- Circular buffers 32 bytes TX/RX
- Public API: uart_init(), uart_send_byte(), uart_send_string(),
  uart_receive_byte(), uart_data_available()

**adc_pwm.h / adc_pwm.c**
- PWM: TA1 CCR2 on P2.4, SMCLK/4=4MHz, CCR0=39999 → 100Hz
  Duty presets: 10%=3999, 25%=9999, 50%=19999, 75%=29999, 90%=35999
- ADC: A4 (P1.4), SMCLK/3=5.33MHz, single conversion, 10-bit
  ADC10SHT_0, INCH_4, ADC10DIV_2, ADC10SSEL_3, CONSEQ_0
- TA0 triggers ADC at 1051Hz (CCR0=15222) - async with PWM (GCD=1)
- ADC ISR sets adc_result + adc_data_ready flag (NO blocking calls)
- extern volatile uint16_t adc_result
- extern volatile uint8_t  adc_data_ready

**main.c**
- 16MHz DCO: BCSCTL1=CALBC1_16MHZ, DCOCTL=CALDCO_16MHZ
- LED1 red P1.0 (heartbeat), LED2 green P1.6 (ADC toggle in ISR)
- S2 button P1.3 (pull-up) cycles PWM duty 10/25/50/75/90%
- Debounce: up-counting counter inside adc_data_ready block (50ms)
- All logic gated on adc_data_ready → runs at exactly 1051Hz
- Binary UART protocol (2 bytes/sample, self-synchronizing):
    Byte 1: 0x80 | (result >> 7)   bit7=1 sync marker
    Byte 2: result & 0x7F           bit7=0
    Decode: value = ((byte1 & 0x7F) << 7) | byte2
  Special packet: 0xFF, duty_index → PWM duty change notification

### PC GUI (Python: msp430_daq.py)
- PySide6 + PyQtGraph, dark theme
- Live scrolling waveform, adjustable display window (50-5000 samples)
- Statistics (mean/min/max/stddev) computed from full 5000-sample buffer
  for stability — NOT from display window
- Mean voltage line overlay (orange dashed)
- PWM duty cycle indicator with 5 preset bars, updates on S2 press
- Session sample counter, elapsed time, measured sample rate
- Binary protocol parser with 0xFF duty-change packet handling

### Validated Results
| Duty | Expected | Measured |
|------|----------|---------|
| 10%  | 0.360V   | 0.361V  |
| 25%  | 0.900V   | 0.901V  |
| 50%  | 1.800V   | 1.764V  |
| 75%  | 2.700V   | 2.699V  |
All within 1-2mV. Sample rate: ~1034Hz actual vs 1051Hz target.

---

## Next Project: TM4C123G LaunchPad DAQ System

### Hardware
- **Board:** Tiva C Series TM4C123G LaunchPad (EK-TM4C123GXL)
- **MCU:** TM4C123GH6PM, ARM Cortex-M4F @ 80MHz
- **Flash:** 256KB, **RAM:** 32KB
- Same FTDI cable (COM4) for UART initially
- Suggest migrating to USB CDC later (native USB on TM4C)
- Working directory: C:\ti\Work\CORTEXM4F_DAQ

### Key Improvements Over MSP430 DAQ
- **12-bit ADC** (vs 10-bit) → 4x better resolution
- **Up to 1Msps ADC** → vastly higher sample rates possible
- **Hardware FPU** → real voltage conversion in firmware, not just raw counts
- **8 hardware UARTs** → no pin conflicts
- **Dedicated PWM module** → cleaner signal generation
- **UART at 921600 baud** feasible → ~92kHz sample throughput

### Suggested Architecture for TM4C123G DAQ
- Use TivaWare peripheral library (already in CCS)
- UART0 on PA0/PA1 (same as default, maps to USB debug port / virtual COM)
- ADC0 on PE3 (AIN0) - clean analog input pin
- PWM on PB6 or PF2 - dedicated PWM module output
- Timer0A as ADC trigger at configurable sample rate
- Target 10kHz initial sample rate (well within UART budget at 921600)
- Same binary streaming protocol (extend to 3 bytes for 12-bit data)
- Extend GUI with configurable sample rate, trigger, data logging to CSV

### Suggested 3-byte Protocol for 12-bit ADC
    Byte 1: 0x80 | (result >> 8)   bits[11:8], bit7=1 sync
    Byte 2: (result >> 4) & 0x7F   bits[7:4],  bit7=0
    Byte 3:  result & 0x0F         bits[3:0],  bit7=0
    Decode: value = ((b1&0x7F)<<8) | (b2<<4) | b3

### Development Environment
- CCS 20.4, TivaWare already included
- Same workflow: bare metal C, modular files (uart.h/c, adc_pwm.h/c, main.c)
- Claude Code for direct filesystem access - much faster than download/copy

---

## Key Lessons Learned (MSP430 Session)
1. Never call blocking functions from ISR context (GIE cleared on entry)
2. Use disassembly to verify cycle counts for timing-critical code
3. Variable shifts (1<<i) generate library calls on no-barrel-shifter MCUs
4. ADC and PWM sample rates must be asynchronous (GCD=1) to avoid aliasing
5. Debounce logic must run at same rate as its timebase counter
6. Stats window vs display window should be independent for stability
7. Hardware UART peripheral eliminates all soft UART timing fragility

