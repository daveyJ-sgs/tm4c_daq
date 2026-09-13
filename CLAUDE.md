# TM4C123G DAQ — USB CDC streaming ADC

12-bit data acquisition on an EK-TM4C123GXL, streaming over USB Full-Speed CDC
to a PySide6 GUI.

> Authoritative as of 2026-09-12 (burst capture + trigger landed). `Archive/CLAUDE.md` and `Archive/save/*.md` are
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

Current footprint: ~20.5 KB of 256 KB flash, 27,593 of 32,768 B SRAM
(**~5.1 KB free** — the ADC ring is 16 KB and the transmit ring 4 KB).
Burst mode still reuses the streaming ring, from when only 1.1 KB was free;
dropping usblib's 8 KB TX buffer freed the rest.

## Source layout

- `usb_dev_serial/main.c` — everything: ADC, uDMA, PWM, USB, protocol
- `usb_dev_serial/usb_serial_structs.c/.h` — CDC descriptors, RX buffer 256.
  There is deliberately **no TX `tUSBBuffer`** — see the transmit-path gotcha
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
| `0xC0` | burst/trigger, sub-coded in the low nibble — see below |

**Host → device**: `[0xFE][code|index]` for the presets above, and a four-byte
form with a 16-bit big-endian argument for burst: `[0xFE][0xC0|sub][hi][lo]`.

The code byte is matched against `0xE0`, **not** `0x60`. The old mask ignored
bit 7, which aliases `0xC0` onto `0x40` — a trigger setting would have been
read as a duty-cycle change.

There is **no sync marker**. A single lost byte misaligns every following
triplet permanently. Data triplets can never start with `0xFF`, and the GUI
reader uses that invariant to detect a slip and re-derive the offset
(`ALIGN_TOLERANCE` in `tm4c_daq.py`).

## Burst capture and trigger

Streaming and burst are **mutually exclusive** — both use `g_pui16ADCBuffer`.
While burst is engaged the ADC ISR stops filling the ring and the streaming
drain is skipped entirely (measured: zero stray samples).

States, reported as `[0xFF][0xC0][state]`:
`0` stream · `1` idle · `2` armed · `3` triggered · `4` full · `5` draining.

Host commands, `[0xFE][0xC0|sub][hi][lo]`:

| sub | name | argument |
|---|---|---|
| `0x0` | ACTION | 0 disarm · 1 arm single · 2 arm continuous · 3 force trigger |
| `0x1` | LEVEL | trigger level, 0–4095 |
| `0x2` | SLOPE | 0 rising, 1 falling |
| `0x3` | PRE_PCT | pre-trigger window as a percent of capture length, 0–95 |
| `0x4` | LENGTH | capture length in samples, 64–8192, forced even |
| `0x5` | AUTO_MS | auto-trigger timeout in ms (scope "Auto"); 0 waits forever |

A capture is delivered as `STATE=full`, then header triplets `0xC1`/`0xC2`
(length), `0xC3`/`0xC4` (trigger offset), `0xC5` (rate index), `0xC6` (flags:
bit0 slope, bit1 trigger was forced), `0xC8`/`0xC9` (the level that actually
fired), then `0xC7` BEGIN, then exactly `length/2` ordinary data triplets with
nothing interleaved, then `STATE=idle` or `STATE=armed`.

**BEGIN is always the last header triplet** — do not assume ascending subcodes.

At 8192 samples the frame is 12,288 bytes, ~34 ms on the wire. Config subcodes
are snapshotted when the device arms, so a setting sent mid-capture applies to
the next one.

## Presets

- PWM frequency: 100 Hz, 1 kHz, 10 kHz, 20 kHz, 40 kHz
- PWM duty: 10, 25, 50, 75, 90 %
- ADC rate: 100 k, 200 k, 250 k, 333 k, 400 k S/s — **default 200 kS/s**

## Measured performance and limits

- **333 kS/s is the safe continuous operating point**: 501.5 kB/s measured at
  the byte level = 334,365 S/s delivered, zero reported drops. 200 kS/s
  measures 300.1 kB/s = 200,040 S/s, also zero loss.
- The **ADC/uDMA path is not the bottleneck** — it hits 400,687 S/s on command.
- The **USB CDC link is**, at ~516 kB/s ≈ 344 kS/s peak. Anything above that
  is dropped at the ring and reported via the `0x80` telemetry; accounting
  balances to within 0.1 %.
- **Burst capture is not subject to any of this.** All five presets return a
  complete 8192-sample frame with the trigger at exactly the requested offset,
  acquired rate measured from the data itself against the known PWM period:
  100 k, 200 k, 250 k and 400 k all land at +0.00 %, and 333 k at 333,320 S/s
  (-0.004 %).  The 333 k and 400 k presets cannot be streamed at all.

- That ~516 kB/s is **42 % of the USB Full-Speed bulk ceiling** (~1.216 MB/s,
  19 × 64 B per 1 ms frame), up from 29 % before the transmit path was
  rewritten.
- **What the remaining limit is.** With the copy cost gone, the transmit ring
  is now never empty (measured 0 %) and 94 % of send attempts find the class
  still busy with the previous packet — so the device is genuinely serialised
  on one packet in flight. *That* is what double-packet buffering addresses,
  and it is the next thing to try. It was tried once before and did nothing,
  which was correct at the time: the bottleneck then was CPU in the copy, not
  packets in flight, so a second FIFO slot could not have helped.

## Gotchas

- **Do not use `USBBufferWrite` on the transmit side.** `USBRingBufWrite`
  copies one byte at a time through `UpdateIndexAtomic`, which globally
  disables and re-enables interrupts *per byte* — 768 function calls and 768
  CPSID/CPSIE pairs for one 768-byte batch. Measured at **1,564 µs per call
  and 69.9 % of the CPU**, against 17.4 % for the entire USB interrupt
  handler. `main.c` keeps its own transmit ring and hands the CDC class whole
  64-byte packets instead; filling and draining are both `memcpy`.
- **Flushing that ring must be deferred to the main loop.** `ControlHandler`
  runs in USB interrupt context and the main loop is the ring's only producer.
  A flush landing between the producer's `memcpy` and its head update puts the
  stale head back, and the device then transmits kilobytes of stale bytes —
  which, with no sync marker in the protocol, the host never recovers from.
- **Re-read the free space after servicing the status queue**, since those
  writes go into the same ring. Sizing a sample batch from a figure taken
  before them overruns the ring by up to a full status queue.

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
- **Nothing may be interleaved into a burst frame.** `QueueStatus` refuses
  every caller while the state is DRAINING. One stray status triplet after
  BEGIN would shift every following sample by a byte, and the protocol has no
  way to signal it. The header is queued while still in FULL, before the lock.
- **Burst actions are deferred while DRAINING.** Arming or disarming mid-frame
  would abandon the payload partway through, and the host counts the payload
  out by length with no resynchronisation point inside it -- so it swallows
  whatever comes next, including the state message announcing the change, and
  only recovers after eating a frame's worth of unrelated bytes. The action is
  held until the frame is out; worst case one frame of latency, ~34 ms at 8192.
- **The ADC keeps converting during IDLE/FULL/DRAINING** and throws the samples
  away. Stopping it would freeze `g_ui32SampleCount`, trip the acquisition
  watchdog, and have it rebuild the pipeline in the middle of a transfer.
- **`sA` is clamped to 4079, `sB` is not** — only `b0` has to avoid `0xFF`. A
  rail-to-rail square wave therefore shows a 16-count sawtooth on alternate
  samples at the top rail. This is correct; do not "fix" it.
- Do not call `reset_input_buffer()` on the host. It cuts mid-triplet and there
  is no sync marker, so everything after it decodes as garbage. Discard whole
  reads and re-derive the offset from the `b0` invariant instead.

## Roadmap

1. ~~uDMA ping-pong for the ADC~~ — done
2. ~~Burst capture~~ — done
3. ~~Trigger system~~ — done (edge, pre-trigger, auto/normal/single)
4. **Double-packet buffering**, now that the device is actually serialised on
   one packet in flight rather than CPU-bound. Needs `usbdcdc.c` built from
   source so its binary busy flag becomes a count of packets outstanding.
5. **Burst sample rates above 400 kS/s.** Burst no longer has to respect the
   link ceiling, so the preset table is the only thing holding the rate down.
   The TM4C123 ADC is specified to 1 Msps and 400,687 S/s is simply the highest
   preset we have, not a measured limit. Add presets and verify the achieved
   rate the same way — deliberately as its own step, not folded into another
   change.
6. Scope GUI — timebase, cursors, FFT, measurements
7. Multi-channel, analog frontend
