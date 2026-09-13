# Double-packet buffering: scope, findings and plan

Scoping notes for roadmap item 4, written 2026-09-13 from reading the actual
transmit path in `main.c` and TivaWare's `usbdcdc.c`. Nothing here has been
built or tested yet -- this is the design study, not a result.

## Why this is the next thing

With the host reader in its own process the host is not a limit anywhere. The
device is genuinely serialised on one packet in flight: the transmit ring is
measured as **never empty (0%)** and **94% of send attempts find the class
still busy**. The data is ready; the endpoint is not allowed to take it.

In bus terms: USB Full-Speed offers 19 bulk slots of 64 B per 1 ms frame
(1.216 MB/s). We achieve ~516 kB/s, which is **about 8 of those 19 slots** --
one packet every ~124 us when the packet itself occupies only ~50 us of bus
time. The remaining ~74 us is turnaround: host ACK -> device interrupt ->
firmware reloads the FIFO.

Double buffering hides that turnaround, because packet N+1 is already loaded
while packet N is on the wire.

Note this was tried once before and did nothing. That was **correct at the
time**: the bottleneck then was CPU burning inside `USBBufferWrite`'s per-byte
interrupt masking, so a second FIFO slot had nowhere to help. Rewriting the
transmit path removed that. Same change, different system.

---

## Finding 1: the hardware supports it, driverlib does not expose it

The controller has the feature:

- `USB_TXFIFOSZ_DPB` (bit 0x10) -- `inc/hw_usb.h:641`
- `USB_O_TXDPKTBUFDIS` with a per-endpoint disable bit -- `inc/hw_usb.h:365`

But **`USBFIFOConfigSet` has no way to request it.** The `USB_FIFO_SZ_*` list in
`driverlib/usb.h` runs 8 through 2048 with no `_DB` variants. (An earlier
assumption that `USB_FIFO_SZ_64_DB` existed was wrong -- it does not.) Enabling
DPB therefore means direct register writes.

There is also a FIFO allocation problem. `usbdconfig.c:379` hands each endpoint
exactly one packet's worth of FIFO RAM by first-fit at enumeration. Double
buffering needs 128 B, not 64.

**The clean dodge:** re-point the bulk IN FIFO from `main.c` *after*
configuration completes -- choose a high offset in the 4 KB of FIFO RAM (there
is plenty spare) and set DPB there. This avoids forking `usbdconfig.c`.

Roughly 20 lines. Confirm reset values and the correct reconfiguration sequence
in the datasheet before trusting it.

## Finding 2: two viable software paths

`iCDCTxState` is a two-state enum gating the whole transmit path:

| what | where |
|---|---|
| set to `WaitData` on send | `usbdcdc.c:2748` |
| gate that rejects a second packet | `usbdcdc.c:2713` |
| reset to `Idle` on TX-complete | `usbdcdc.c:1277` |
| read by `USBDCDCTxPacketAvailable` | `usbdcdc.c:2925` |
| polled by our `TxPumpPacket` | `main.c:472` |

### Path A -- fork `usbdcdc.c`, make the state a count

The four touch points above are straightforward. The real complication is
**`ui16LastTxSize`**, which assumes exactly one packet in flight and drives both
the TX callback's byte count and the zero-length-packet decision. With two
outstanding it must become a small queue of sizes.

Build integration: add the file to `Debug/subdir_vars.mk`, `Debug/subdir_rules.mk`
and the `ORDERED_OBJS` list in `Debug/makefile` -- all three CCS-generated and
all three needing to stay consistent. Same trap as `STACK_SIZE` living in three
places. Our object defines the symbols so the library member is never pulled in.

- Pro: the class stays in charge; ZLP, flush and control paths keep working.
- Con: a permanent fork of a 3,156-line TI file, re-merged on every TivaWare
  update.

### Path B -- bypass the class on the data path only

`TxPumpPacket` already owns its own ring and hands over complete 64-byte
packets. `USBDCDCPacketWrite` adds almost nothing over `MAP_USBEndpointDataPut`
+ `MAP_USBEndpointDataSend`. Call those directly and track
`g_ui8PacketsInFlight` (0..2) ourselves.

**The snag:** our TX-complete callback arrives via usblib's CDC handler, which
only calls `pfnTxCallback` when `ui16LastTxSize` is non-zero -- and it would be
zero, because we never went through `USBDCDCPacketWrite`. So the interrupt
notification is lost and completion must be read from `USBEndpointStatus` in the
main loop instead. Less of a change than it sounds: `TxPumpPacket` is already
called from the main loop at `main.c:1642`.

- Pro: no TI source forked, no build-system surgery, reversible.
- Con: we half-own an endpoint the class still believes it owns. The DTR, flush
  and disconnect paths need checking for divergence between our count and the
  hardware state, especially after a bus reset.

## Finding 3: a possible free win -- check this first

usblib sends a **zero-length packet after every full 64-byte packet** if the TX
callback did not immediately queue more (`usbdcdc.c:1305-1318`).

In steady state our ring is measured as never empty, so it should not fire. But
if it *is* firing we are burning a whole transaction slot per packet, which
alone could explain a chunk of why we get 8 of 19 slots per frame. A counter in
`TxHandler` answers it in about ten minutes and might make the rest of this
work unnecessary.

---

## Estimate

| piece | effort | risk |
|---|---|---|
| ZLP counter (do first) | ~15 min | none |
| FIFO / DPB register work | ~1 hr incl. datasheet | low |
| Path B spike | 2-3 hrs | medium |
| Path A, if made permanent | +1 session, then fork upkeep | medium |
| Verification | ~1 hr, tooling already exists | low |

**One focused session to a measurable answer.** Complexity is moderate: the
code is small, but the failure modes are nasty. The protocol has no sync marker,
so any transmit-path bug appears as *permanent* host desync rather than a
glitch -- the same class of failure already documented for the flush race.

## Verification and the early stopping point

`txpace.js` + `mkaddrs.py` already read the firmware pacing counters over DSS.
Run `mkaddrs.py` between every build and every read, or the addresses go stale.

The gate: **"send attempts that found the class busy" should collapse from 94%
to near zero.** Then host-side throughput via `one_run.py` in fresh processes,
>= 15 s windows, repeated.

If busy% collapses and throughput does **not** move, the limit is host-side bulk
scheduling by the xHCI driver. Stop there -- that is a cheap, clean answer and
there is nothing further to win on the device.

## Host-side follow-on, if it works

At >500 kS/s the hard 16,384-byte Windows receive buffer holds under 20 ms
again, and the margin won by the process split partly evaporates. The child
process would need to read in larger chunks. Solvable, but it is a second piece
of work -- do not assume the host stays free.

## What the extra bandwidth is actually for

See [oversampling-and-adc-characterisation.md](oversampling-and-adc-characterisation.md).
Short version: half a bit per doubling of the link, and an easier analog
anti-alias filter. Worth doing, but do not oversell it.
