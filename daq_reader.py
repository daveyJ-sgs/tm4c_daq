"""Wire protocol decode and the serial reader process for the TM4C123G DAQ.

This module deliberately imports **no Qt**.  That is the entire point of it.

The host's USB CDC receive buffer is a hard 16,384 bytes: `set_buffer_size()`
succeeds and `usbser.sys` ignores it, at 64 KB and at 1 MB alike.  At 333 kS/s
(~500 kB/s) that buffer holds only ~32 ms of data, and a read cadence of ~31 ms
-- Windows quantises the 20 ms timeout up to two 15.6 ms scheduler ticks --
already occupies 95 % of it before anything goes wrong.

Meanwhile pyqtgraph's paint is Python and holds the GIL.  A thread that wanted
the GIL every 1 ms was measured waiting up to 25.6 ms while the plot redrew,
which is more than enough to overrun what little slack that buffer has.  With
the reader in the GUI's process, 5 of 6 fresh runs lost data at 333 kS/s; with
it in its own process, 4 of 4 were clean and peak buffer occupancy fell from a
pinned 16,384 to 768-8,192.

Decode cost is only 0.7 % of a core, so none of this is about CPU.  It is about
scheduling latency on the drain, and a separate interpreter with its own GIL is
what fixes it.  Keep this module free of anything that could hold the GIL: no
Qt, no plotting, no GUI imports.
"""
import collections
import queue
import sys
import threading
import time

import numpy as np
import serial

# ---------------------------------------------
# Hardware / protocol constants
# ---------------------------------------------
VCC          = 3.3              # TM4C123G ADC reference voltage (V)
ADC_BITS     = 12
ADC_MAX      = (1 << ADC_BITS) - 1   # 4095

# Triplet realignment.  Only the b0 slot is constrained (never 0xFF), so an
# elevated 0xFF rate there means the stream has slipped.  Command triplets do
# legitimately put 0xFF in b0 but stay well under 1% even during heavy overflow
# reporting, while a slipped stream measures 10-90%.
ALIGN_TOLERANCE   = 0.05
ALIGN_MIN_BYTES   = 300         # need a reasonable sample before judging

# Serial read timeout.  Windows rounds this up to whole 15.6 ms scheduler
# ticks, so 20 ms actually yields a ~31 ms cadence and ~15.5 kB per read.
# Do NOT "fix" that by shortening it: 5 ms and 2 ms were both measured worse,
# as was polling in_waiting instead of blocking.  Every one of those adds GIL
# round-trips, and a blocking read is the one strategy that does not need the
# GIL while it waits.
READ_TIMEOUT = 0.020

# ---- Burst capture / trigger (protocol v1) ----
# Host -> device is [0xFE][0xC0|sub][argHi][argLo]; device -> host is the
# ordinary status triplet [0xFF][0xC0|sub][data].
BURST_CMD           = 0xC0      # command/status code nibble-pair for burst
BURST_SUB_ACTION    = 0x0
BURST_SUB_LEVEL     = 0x1
BURST_SUB_SLOPE     = 0x2
BURST_SUB_PRE_PCT   = 0x3
BURST_SUB_LENGTH    = 0x4
BURST_SUB_AUTO_MS   = 0x5

BURST_ACTION_DISARM     = 0     # return to STREAM
BURST_ACTION_ARM_SINGLE = 1
BURST_ACTION_ARM_CONT   = 2     # device auto-rearms after each drain
BURST_ACTION_FORCE      = 3

# Device -> host status subcodes.  0x0 is the state enum; every other subcode
# except BEGIN is a frame header field.  BEGIN is emitted LAST regardless of its
# number, so "is a header" is `sub != BEGIN`, never `sub < BEGIN`.
BURST_ST_STATE   = 0x0
BURST_ST_LEN_HI  = 0x1
BURST_ST_LEN_LO  = 0x2
BURST_ST_TRIG_HI = 0x3
BURST_ST_TRIG_LO = 0x4
BURST_ST_RATE    = 0x5
BURST_ST_FLAGS   = 0x6
BURST_ST_BEGIN   = 0x7
BURST_ST_LVL_HI  = 0x8          # v2: the level this capture actually used
BURST_ST_LVL_LO  = 0x9

BURST_STATE_STREAM    = 0
BURST_STATE_IDLE      = 1
BURST_STATE_ARMED     = 2
BURST_STATE_TRIGGERED = 3
BURST_STATE_FULL      = 4
BURST_STATE_DRAINING  = 5

BURST_LENGTH_MIN   = 64         # device clamps to 64..8192 and forces even
BURST_LENGTH_MAX   = 8192

# Event kinds handed to the decoder's emit() callback.  The GUI maps these onto
# Qt signals; the reader process maps them onto pipe messages.
EV_SAMPLES     = "samples"
EV_DUTY        = "duty"
EV_FREQ        = "freq"
EV_RATE        = "rate"
EV_OVERFLOW    = "overflow"
EV_STALL       = "stall"
EV_RESYNC      = "resync"
EV_BURST_STATE = "burst_state"
EV_BURST_FRAME = "burst_frame"
EV_STATUS      = "status"
EV_STATS       = "stats"


# ---------------------------------------------
# Protocol decoder (no I/O, no Qt)
# ---------------------------------------------
class ProtocolDecoder:
    """Turns raw USB CDC bytes into samples and events.

    One implementation, used by both the in-process reader and the reader
    process, so the protocol only ever lives in one place.  `emit(kind, value)`
    receives every event; kinds are the EV_* constants above.
    """

    def __init__(self, emit):
        self._emit = emit
        self._leftover = b""
        self._resync_count = 0

        # Burst frame assembly.  The header subcodes land in _burst_hdr as they
        # arrive; BEGIN switches the decoder into collecting mode, where whole
        # triplet rows are appended to _burst_accum until _burst_needed samples
        # have been gathered.  Payload samples never reach the streaming queue.
        self._burst_hdr    = {}
        self._burst_accum  = []
        self._burst_needed = 0

    # -- public ------------------------------------
    def feed(self, raw):
        """Consume one read's worth of bytes."""
        if not raw:
            return
        buf = np.frombuffer(self._leftover + raw, dtype=np.uint8)

        # Triplet alignment guard.  The protocol has no sync marker, so a
        # single lost or duplicated byte would garble every sample from here on
        # with nothing to signal it.  Data triplets never start with 0xFF; if
        # the b0 slot says otherwise, re-derive the offset from the slot that
        # does satisfy the invariant.  Suppressed mid-frame: a burst payload is
        # pure data, so the b0 invariant cannot fire legitimately there, and a
        # false positive would shear the frame in half.
        if len(buf) >= ALIGN_MIN_BYTES and self._burst_needed <= 0:
            rates = [float((buf[o::3] == 0xFF).mean()) for o in range(3)]
            best = min(range(3), key=lambda o: rates[o])
            if (best != 0 and rates[0] > ALIGN_TOLERANCE
                    and rates[best] < rates[0] / 4):
                buf = buf[best:]
                self._resync_count += 1
                self._emit(EV_RESYNC, self._resync_count)

        n_triplets = len(buf) // 3
        self._leftover = bytes(buf[n_triplets * 3:])    # save 0-2 bytes
        if n_triplets:
            self._process_triplets(buf[:n_triplets * 3].reshape(n_triplets, 3))

    def reset(self):
        self._leftover = b""
        self._burst_hdr = {}
        self._burst_accum = []
        self._burst_needed = 0

    # -- internals ---------------------------------
    @staticmethod
    def _decode_triplets(t):
        """Vectorized triplet -> sample decode.  t is an Nx3 uint8 array."""
        b0 = t[:, 0].astype(np.uint16)
        b1 = t[:, 1].astype(np.uint16)
        b2 = t[:, 2].astype(np.uint16)
        samples = np.empty(len(t) * 2, dtype=np.uint16)
        samples[0::2] = (b0 << 4) | (b1 >> 4)
        samples[1::2] = ((b1 & 0x0F) << 8) | b2
        return samples

    def _begin_burst(self):
        """BEGIN seen -- snapshot the header and switch into collecting mode."""
        length = ((self._burst_hdr.get(BURST_ST_LEN_HI, 0) << 8)
                  | self._burst_hdr.get(BURST_ST_LEN_LO, 0))
        if (length < BURST_LENGTH_MIN or length > BURST_LENGTH_MAX
                or length % 2):
            # Header lost or corrupt -- drop the frame rather than swallow an
            # arbitrary slice of the stream as payload.
            self._burst_hdr = {}
            return False
        self._burst_needed = length
        self._burst_accum = []
        return True

    def _finish_burst(self):
        """Payload complete -- decode it and hand the frame up."""
        hdr = self._burst_hdr
        length = (hdr.get(BURST_ST_LEN_HI, 0) << 8) | hdr.get(BURST_ST_LEN_LO, 0)
        trig = (hdr.get(BURST_ST_TRIG_HI, 0) << 8) | hdr.get(BURST_ST_TRIG_LO, 0)
        flags = hdr.get(BURST_ST_FLAGS, 0)

        # The level the capture actually fired against (v2).  Absent from a v1
        # device -- report None rather than 0 so the GUI falls back to its own
        # pending value instead of drawing the threshold down at 0 V.
        if BURST_ST_LVL_HI in hdr or BURST_ST_LVL_LO in hdr:
            level = ((hdr.get(BURST_ST_LVL_HI, 0) << 8)
                     | hdr.get(BURST_ST_LVL_LO, 0))
        else:
            level = None

        if self._burst_accum:
            t = (self._burst_accum[0] if len(self._burst_accum) == 1
                 else np.concatenate(self._burst_accum))
            samples = self._decode_triplets(t)[:length]
        else:
            samples = np.array([], dtype=np.uint16)

        self._burst_hdr    = {}
        self._burst_accum  = []
        self._burst_needed = 0

        self._emit(EV_BURST_FRAME, {
            "samples":  samples,
            "trig":     min(trig, max(len(samples) - 1, 0)),
            "rate_idx": hdr.get(BURST_ST_RATE, 0),
            "slope":    flags & 0x01,
            "forced":   bool(flags & 0x02),
            "level":    level,
        })

    def _process_triplets(self, t):
        """Split a read into burst payload runs and ordinary stream triplets.

        Burst payload is order-sensitive: it is the contiguous run immediately
        after BEGIN.  Everything outside that run goes through the usual
        mask-based streaming decode, which does not care about ordering.
        """
        while len(t):
            if self._burst_needed > 0:
                want = (self._burst_needed + 1) // 2      # len is always even
                take = min(want, len(t))
                self._burst_accum.append(t[:take])
                self._burst_needed -= take * 2
                t = t[take:]
                if self._burst_needed <= 0:
                    self._finish_burst()
                continue

            # Not collecting -- look for the next BEGIN triplet.
            begins = np.flatnonzero((t[:, 0] == 0xFF)
                                    & (t[:, 1] == (BURST_CMD | BURST_ST_BEGIN)))
            if not begins.size:
                self._process_stream(t)
                return

            idx = int(begins[0])
            self._process_stream(t[:idx])       # header subcodes live in here
            t = t[idx + 1:]                     # BEGIN itself is consumed
            self._begin_burst()

    def _process_stream(self, t):
        """The streaming decode: order-independent, fully vectorized."""
        if not len(t):
            return

        b0 = t[:, 0]
        is_cmd = (b0 == 0xFF)

        # Command triplets -- iterate (rare, at most a few per button press)
        if is_cmd.any():
            for cmd_b0, cmd_b1, cmd_b2 in t[is_cmd]:
                cmd = int(cmd_b1)
                code = cmd & 0xF0
                arg = cmd & 0x0F
                if code == 0x40:
                    self._emit(EV_DUTY, arg)
                elif code == 0x20:
                    self._emit(EV_FREQ, arg)
                elif code == 0x60:
                    self._emit(EV_RATE, arg)
                elif code == 0x80:
                    self._emit(EV_OVERFLOW, (arg << 8) | int(cmd_b2))
                elif code == 0xA0:
                    self._emit(EV_STALL, arg)
                elif code == BURST_CMD:
                    data = int(cmd_b2)
                    if arg == BURST_ST_STATE:
                        self._emit(EV_BURST_STATE, data)
                    elif arg != BURST_ST_BEGIN:
                        # Frame header.  BEGIN is emitted last but is not the
                        # highest subcode, so this must not be a `<` test.
                        self._burst_hdr[arg] = data
                    # BEGIN never reaches here; _process_triplets consumes it.

            keep = ~is_cmd
            if not keep.any():
                return
            db0 = b0[keep].astype(np.uint16)
            db1 = t[:, 1][keep].astype(np.uint16)
            db2 = t[:, 2][keep].astype(np.uint16)
        else:
            # Overwhelmingly the common case: no command triplets at all, so
            # skip the boolean masking entirely.
            db0 = b0.astype(np.uint16)
            db1 = t[:, 1].astype(np.uint16)
            db2 = t[:, 2].astype(np.uint16)

        samples = np.empty(len(db0) * 2, dtype=np.uint16)
        samples[0::2] = (db0 << 4) | (db1 >> 4)
        samples[1::2] = ((db1 & 0x0F) << 8) | db2
        self._emit(EV_SAMPLES, samples)


# ---------------------------------------------
# Shared serial plumbing
# ---------------------------------------------
def open_port(port, baud=115200):
    """Open the CDC port and discard whatever arrived before we were ready."""
    ser = serial.Serial(port, baud, timeout=READ_TIMEOUT)
    if hasattr(ser, "set_buffer_size"):
        try:
            # Measured no-op: usbser.sys ignores SetupComm and the receive
            # buffer stays 16,384 bytes whatever is asked for.  Left in because
            # it is harmless and other drivers do honour it.
            ser.set_buffer_size(rx_size=1 << 20, tx_size=1 << 16)
        except (AttributeError, OSError, ValueError):
            pass

    # Startup drain: DTR flush has already told the firmware to reset its TX
    # buffer, but Windows may have buffered some pre-flush bytes already.  Wait
    # briefly for the flush to propagate, then discard whatever is in the
    # receive buffer so decoding starts at a clean triplet boundary.
    time.sleep(0.08)
    ser.read(65536)
    return ser


def burst_command(sub, arg):
    """Build a 4-byte burst command [0xFE][0xC0|sub][argHi][argLo]."""
    return bytes([0xFE, BURST_CMD | (sub & 0x0F),
                  (arg >> 8) & 0xFF, arg & 0xFF])


# ---------------------------------------------
# Reader process entry point
# ---------------------------------------------
# How many pending messages the child will hold for a parent that has fallen
# behind.  ~32 chunks/s at 333 kS/s, so this is about 8 seconds.  If it fills,
# the OLDEST are dropped and counted -- the child must never block on send,
# because blocking the sender stalls the serial drain and reintroduces exactly
# the bug this process split exists to fix.
SEND_QUEUE_DEPTH = 256

STATS_INTERVAL_S = 0.5


def reader_main(port, cmd_conn, data_conn, baud=115200):
    """Own the serial port: drain, decode, publish.  Runs as a child process.

    Nothing here may block on the parent.  The parent is a GUI and will stall
    for tens of milliseconds at a time; that must not reach the drain loop.
    """
    outq = queue.Queue(maxsize=SEND_QUEUE_DEPTH)
    dropped = [0]
    stop = threading.Event()

    def publish(kind, value):
        """Never blocks.  Drops the oldest message if the parent is behind."""
        try:
            outq.put_nowait((kind, value))
        except queue.Full:
            try:
                outq.get_nowait()
                dropped[0] += 1
                outq.put_nowait((kind, value))
            except (queue.Empty, queue.Full):
                dropped[0] += 1

    def sender():
        while True:
            item = outq.get()
            if item is None:
                break
            try:
                data_conn.send(item)
            except (BrokenPipeError, EOFError, OSError):
                stop.set()
                break

    tx = threading.Thread(target=sender, daemon=True)
    tx.start()

    try:
        ser = open_port(port, baud)
    except serial.SerialException as e:
        publish(EV_STATUS, "Error: %s" % e)
        outq.put(None)
        tx.join(timeout=1.0)
        return

    publish(EV_STATUS, "Connected: %s (USB CDC)" % port)

    decoder = ProtocolDecoder(publish)
    peak_q = 0
    n_samples = 0
    last_stats = time.perf_counter()

    try:
        while not stop.is_set():
            # Commands from the GUI.  Drained first so a rate change is not
            # held up behind a read.
            while cmd_conn.poll():
                try:
                    kind, payload = cmd_conn.recv()
                except (EOFError, OSError):
                    stop.set()
                    break
                if kind == "write":
                    ser.write(payload)
                elif kind == "stop":
                    stop.set()
            if stop.is_set():
                break

            try:
                q = ser.in_waiting
                if q > peak_q:
                    peak_q = q
            except (OSError, serial.SerialException):
                pass

            raw = ser.read(65536)
            if raw:
                n_samples += (len(raw) // 3) * 2
                decoder.feed(raw)

            now = time.perf_counter()
            if now - last_stats >= STATS_INTERVAL_S:
                last_stats = now
                # peak_q is the margin against the 16 KB OS buffer and is the
                # single most useful number for diagnosing a stalled drain, so
                # it is reported rather than left to a test harness.
                publish(EV_STATS, {"peak_queue": peak_q,
                                   "display_dropped": dropped[0],
                                   "raw_samples": n_samples})
                peak_q = 0
    finally:
        try:
            ser.close()
        except Exception:
            pass
        publish(EV_STATUS, "Disconnected")
        outq.put(None)
        tx.join(timeout=1.0)
        try:
            data_conn.close()
        except Exception:
            pass
