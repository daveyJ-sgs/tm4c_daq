> **Superseded.** See the CLAUDE.md at the repository root for current build,
> protocol and pin information. This file is a March 2026 snapshot describing an
> older 2-byte protocol and PWM preset set, kept for history only.

# TM4C123G DAQ — USB CDC Streaming

## Project Overview
12-bit ADC data acquisition system on EK-TM4C123GXL, streaming over USB Full-Speed (12 Mbps) using CDC virtual COM port.

## Source Files
- `main.c` — Application: ADC, PWM, USB streaming, button handling
- `usb_serial_structs.c/.h` — USB CDC descriptors and buffer configuration
- `startup_ccs.c` — Interrupt vector table (SysTick, ADC0Seq3, USB0)
- `tm4c123_daq_ccs.cmd` — Linker command file (256KB flash, 32KB SRAM)

## CCS Project Setup (CCS 20.4)

### Create Project
1. File > New > CCS Project
2. Target: TM4C123GH6PM
3. Connection: Stellaris In-Circuit Debug Interface
4. Project name: CORTEXM4F_DAQ
5. Empty project, select ti-cgt-armllvm compiler

### Add Source Files
Add all .c and .h files from this directory. Set `tm4c123_daq_ccs.cmd` as the linker command file.

### Compiler Settings (Project Properties > Build > ARM Compiler)
- **Include paths** — Add: `C:\ti\TivaWare_C_Series-2.2.0.295`
- **Predefined symbols** — Add: `PART_TM4C123GH6PM` and `TARGET_IS_TM4C123_RB1`

### Linker Settings (Project Properties > Build > ARM Linker)
- **Library search path** — Add:
  - `C:\ti\TivaWare_C_Series-2.2.0.295\driverlib\ccs\Debug`
  - `C:\ti\TivaWare_C_Series-2.2.0.295\usblib\ccs\Debug`
- **Libraries** — Add: `driverlib.lib` and `usblib.lib`

## Hardware Connections
- **Debug USB (J1, top)** — to PC for programming/debug
- **Device USB (J2, side)** — to PC for data (second USB cable!)
- **PB6 → PE3** — jumper wire for PWM self-test

## Controls
- **SW1 (left button)** — Cycle PWM frequency: 100 Hz → 1 kHz → 10 kHz → 100 kHz → 500 kHz
- **SW2 (right button)** — Cycle PWM duty: 10% → 25% → 50% → 75% → 90%
- Defaults: 10 kHz, 50% duty

## LED Indicators
- **Red (PF1)** — Heartbeat (blinks 1 Hz)
- **Blue (PF2)** — USB connected (solid on)
- **Green (PF3)** — ADC activity (toggles every 4096 samples)

## Binary Protocol
Data: `[0x80|(val>>5)] [val&0x1F]` — 12-bit ADC in 2 bytes, bit7 sync
Commands: `[0xFF] [0x20|freq_idx]` or `[0xFF] [0x40|duty_idx]`

## Windows Driver
Windows 10/11 should auto-detect the CDC device and load usbser.sys. If not, INF file at: `C:\ti\TivaWare_C_Series-2.2.0.295\windows_drivers\`
