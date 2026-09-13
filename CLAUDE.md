# TM4C123G DAQ — USB CDC streaming ADC

12-bit data acquisition on an EK-TM4C123GXL, streaming over USB Full-Speed CDC
to a PySide6 GUI.

> Authoritative as of 2026-09-12. `Archive/CLAUDE.md` and `Archive/save/*.md` are
> earlier snapshots kept for history and describe a protocol and pin set that no
> longer exist — do not build from them.

## Hardware

| | |
|---|---|
| Board | EK-TM4C123GXL (TM4C123GH6PM, Cortex-M4F @ 80 MHz, 256 KB flash, 32 KB SRAM) |
| **J1** (top micro-B) | ICDI debug + programming. Enumerates as *Stellaris Virtual Serial Port*, VID:PID `1CBE:00FD` |
| **J2** (side micro-B) | CDC data port — **this is the one the GUI opens**. Enumerates as *TM4C123G DAQ*, VID:PID `1CBE:0002` |
| PE3 | ADC0 AIN0 (analog in) |
| PB6 | M0PWM0 test output — **jumper PB6 → PE3** for the self-test |
| PF1 / PF2 / PF3 | Red heartbeat / Blue USB-connected / Green ADC activity |
| PF4 = SW1, PF0 = SW2 | Cycle PWM frequency / duty (PF0 needs its NMI lock cleared; the firmware does this) |

Both USB cables must be plugged in: J1 to flash, J2 to stream.

## Build and flash

Toolchain: CCS 20.5 (`C:\ti\ccs2050`), `ti-cgt-armllvm_4.0.4.LTS`, TivaWare
`C:\ti\TivaWare_C_Series-2.2.0.295`.

- Predefined symbols: `PART_TM4C123GH6PM`, `TARGET_IS_TM4C123_RB1`, `ccs`
  (`ccs` is required — `usblib.h`'s `PACKED` macro depends on it)
- Libraries: `usblib.lib` and `driverlib.lib` from TivaWare's `ccs/Debug`
- Linker stack size is **2048** and lives in three places that must agree:
  `.cproject` (`STACK_SIZE`, the authoritative one), `tm4c123_daq_ccs.cmd`
  (`__STACK_TOP`), and the CCS-generated `Debug/makefile`

```sh
# build
cd usb_dev_serial/Debug && /c/ti/ccs2050/ccs/utils/bin/gmake.exe all

# flash, reset and run
/c/ti/ccs2050/ccs/ccs_base/DebugServer/bin/DSLite.exe flash --reset 1 --run \
  --config=../target_config.ccxml usb_dev_serial.out
```

Two build warnings are expected and harmless: the `wchar_t` mismatch between
tiarmclang and the pre-built TivaWare libs, and the `tiobj2bin` post-build
failure (it only affects `.bin`; the `.out` flashes fine).

Current footprint: ~17.5 KB of 256 KB flash, 31,611 of 32,768 B SRAM
(**~1.1 KB free** — the ADC ring is 16 KB and the USB TX buffer 8 KB).

## Source layout

- `usb_dev_serial/main.c` — everything: ADC, uDMA, PWM, USB, protocol
- `usb_dev_serial/usb_serial_structs.c/.h` — CDC descriptors, TX 8192 / RX 256
- `usb_dev_serial/startup_ccs.c` — vector table (`.intvecs` via `__attribute__`)
- `usb_dev_serial/tm4c123_daq_ccs.cmd` — linker script
- `tm4c_daq.py` — PySide6 + PyQtGraph GUI, includes an automated test harness
- `usb_test.py` — raw byte dump for diagnosing the port

## Wire protocol

**Device → host, 3 bytes per 2 samples** (1.5 B/sample):

```
b0 = sA[11:4]      b1 = sA[3:0]|sB[11:8]      b2 = sB[7:0]
sA = (b0 << 4) | (b1 >> 4)      sB = ((b1 & 0x0F) << 8) | b2
```

`0xFF` is reserved as the status marker, so **sA saturates at 4079** (3.287 V).
Clamp the whole sample, never just its high byte — clamping the byte folds
4080–4095 down by 16 counts and makes the transfer non-monotonic.

**Status triplets** `[0xFF][code|arg][arg2]`:

| code | meaning |
|---|---|
| `0x20` | PWM frequency index changed |
| `0x40` | PWM duty index changed |
| `0x60` | ADC sample-rate index changed |
| `0x80` | overflow delta — 12-bit count of samples dropped |
| `0xA0` | acquisition stall recovered (count in low nibble) |

**Host → device**: `[0xFE][code|index]`, same codes.

There is **no sync marker**. A single lost byte misaligns every following
triplet permanently. Data triplets can never start with `0xFF`, and the GUI
reader uses that invariant to detect a slip and re-derive the offset
(`ALIGN_TOLERANCE` in `tm4c_daq.py`).

## Presets

- PWM frequency: 100 Hz, 1 kHz, 10 kHz, 20 kHz, 40 kHz
- PWM duty: 10, 25, 50, 75, 90 %
- ADC rate: 100 k, 200 k, 250 k, 333 k, 400 k S/s — **default 200 kS/s**

## Measured performance and limits

- **200 kS/s is the safe continuous operating point**: 100.0 % delivered, zero
  loss over 10 s soaks.
- The **ADC/uDMA path is not the bottleneck** — it hits 400,687 S/s on command.
- The **USB CDC link is**: ~358 kB/s ≈ 239 kS/s. Anything above that is dropped
  at the ring and reported via the `0x80` telemetry. Accounting balances to
  within 0.1 % (e.g. at 400 kS/s: 242,235 delivered + 157,564 reported drops).
- That ~358 kB/s is only **25–30 % of the USB Full-Speed bulk ceiling**
  (~1.216 MB/s). The limit is usblib: both `usbdcdc` and `usbdbulk` allow only
  one 64-byte packet in flight. Double-packet buffering was tried, verified set
  in hardware (`TXFIFOSZ = 0x13`), and made no difference — the class state
  machine never stages the second packet. Going faster needs a custom bulk class
  with endpoint uDMA and a WinUSB host.

## Gotchas

- **Attaching a debugger halts the CPU and strands the uDMA ping-pong** with
  both halves `STOP`ped. A SysTick watchdog now detects stalled acquisition
  (`g_ui32SampleCount` not advancing for 100 ms) and rebuilds the channel, so
  this self-heals in ~1 s and ICDI reads are survivable. Without it, the board
  streams nothing until reset.
- The ADC interrupt must be enabled and cleared with `ADCIntEnableEx` /
  `ADCIntClearEx` and `ADC_INT_DMA_SS3`. With `ADCSequenceDMAEnable` the
  sequence-completion interrupt is the wrong source.
- **Keep the USB drain loop bounded.** `USBBufferWrite` frees TX space inside
  the call and under saturation the ADC ring never empties, so a loop that exits
  only on "ring empty" or "buffer full" never exits — it strands the main loop
  and starves everything below it. One batch per pass.
- Once the firmware is pushed past the link ceiling it stops echoing commands
  until the load drops; commands still apply.

## Roadmap

1. ~~uDMA ping-pong for the ADC~~ — done
2. **Burst capture** — fill SRAM at full ADC speed, then transfer. The real
   unlock given the USB ceiling.
3. **Trigger system** — edge trigger, pre-trigger buffer, auto/normal/single.
   Shares a state machine with burst capture; build them together.
4. Scope GUI — timebase, cursors, FFT, measurements
5. Multi-channel, analog frontend
