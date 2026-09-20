"""
TM4C123G DAQ GUI
Real-time 12-bit ADC data visualization over USB CDC
for EK-TM4C123GXL LaunchPad

Dependencies:
    pip install pyserial pyqtgraph PySide6

Binary protocol (3 bytes per 2 × 12-bit samples):
    Byte 0:  sA[11:4]  — clamped to 0xFE max; 0xFF is reserved for commands
    Byte 1:  sA[3:0] | sB[11:8]   (nibble boundary)
    Byte 2:  sB[7:0]
    Decode:  sA = (b0 << 4) | (b1 >> 4)
             sB = ((b1 & 0x0F) << 8) | b2
    Note:    sA clips at 4079 (3.287 V) — signals above that rail near VCC anyway

Command packets (3 bytes, triplet-aligned):
    [0xFF] [0x20 | freq_index] [0x00]   PWM frequency changed (SW1)
    [0xFF] [0x40 | duty_index] [0x00]   PWM duty changed (SW2)
    [0xFF] [0x60 | rate_index] [0x00]   ADC sample-rate changed
    [0xFF] [0x80 | oflow_hi]   [oflow_lo]  Firmware overflow delta
    [0xFF] [0xA0 | recoveries]  [0x00]      Acquisition stall recovered
    [0xFF] [0xC0 | sub]         [data]      Burst capture / trigger telemetry

Burst frames (code 0xC0) arrive as a header run followed by a contiguous,
uninterrupted payload:

    0xC0 state, 0xC1 len_hi, 0xC2 len_lo, 0xC3 trig_hi, 0xC4 trig_lo,
    0xC5 rate_idx, 0xC6 flags, 0xC8 lvl_hi, 0xC9 lvl_lo, 0xC7 BEGIN,
    then exactly len/2 data triplets.

BEGIN is always the LAST header triplet — the subcodes are not in ascending
order, so never treat "sub >= BEGIN" as "not a header".

The payload is ORDER-SENSITIVE, so unlike the streaming path it cannot be
recovered with a boolean mask — the reader slices from the row after BEGIN and
carries an accumulator across reads until len samples have been collected.

Note: only b0 must avoid 0xFF, so sA (even-index samples) clamps at 4079 while
sB (odd-index) reaches 4095.  A rail-to-rail square wave therefore shows a
16-count sawtooth on alternate samples at the top rail.  That is by design —
nothing here may assume the two halves of a triplet are symmetric.

Note: the stream carries no sync marker, so a lost or duplicated byte would
misalign every triplet after it and decode as garbage forever.  Data triplets
can never begin with 0xFF, and the reader uses that invariant to detect the
condition and re-derive the offset.
"""

import sys
import os
import collections
import multiprocessing as mp
import queue
import threading
import time

import serial
import serial.tools.list_ports
import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QGroupBox, QGridLayout, QScrollArea,
    QDoubleSpinBox,
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont, QColor

import daq_reader
# Protocol and decode live in daq_reader, which imports no Qt so it can run in
# the reader process.  Re-exported here because scripts and tests reach for
# these through this module.
from daq_reader import (
    VCC, ADC_BITS, ADC_MAX,
    ALIGN_TOLERANCE, ALIGN_MIN_BYTES, READ_TIMEOUT,
    BURST_CMD,
    BURST_SUB_ACTION, BURST_SUB_LEVEL, BURST_SUB_SLOPE,
    BURST_SUB_PRE_PCT, BURST_SUB_LENGTH, BURST_SUB_AUTO_MS,
    BURST_ACTION_DISARM, BURST_ACTION_ARM_SINGLE,
    BURST_ACTION_ARM_CONT, BURST_ACTION_FORCE,
    BURST_ST_STATE, BURST_ST_LEN_HI, BURST_ST_LEN_LO,
    BURST_ST_TRIG_HI, BURST_ST_TRIG_LO, BURST_ST_RATE,
    BURST_ST_FLAGS, BURST_ST_BEGIN, BURST_ST_LVL_HI, BURST_ST_LVL_LO,
    BURST_STATE_STREAM, BURST_STATE_IDLE, BURST_STATE_ARMED,
    BURST_STATE_TRIGGERED, BURST_STATE_FULL, BURST_STATE_DRAINING,
    BURST_LENGTH_MIN, BURST_LENGTH_MAX,
    ProtocolDecoder,
)

# ---------------------------------------------
# Constants
# ---------------------------------------------
SAMPLE_RATE_PRESETS = [
    (100_000, "100 kS/s"),
    (200_000, "200 kS/s"),
    (250_000, "250 kS/s"),
    (333_333, "333 kS/s"),
    (400_000, "400 kS/s"),
]
DEFAULT_SAMPLE_RATE_INDEX = 1   # Default to the current reliable operating point

# The highest rate that streams without loss, measured end to end through this
# application with the reader in its own process: 333 kS/s delivers ~332,500
# S/s with zero drops and zero resyncs, and peak OS-buffer occupancy stays at
# 5-50% instead of pinned at the 16 KB ceiling.
#
# The remaining limit above this is the FIRMWARE's USB link, not the host: at
# the 400 kS/s preset the device delivers ~352,000 S/s and reports ~50,900
# dropped, which sums to the ADC rate, while the host's receive buffer never
# goes above 25%.  That puts the link ceiling at ~352 kS/s (~528 kB/s), and
# lifting it is a firmware change (double-packet buffering), not a GUI one.
LINK_LIMIT_SPS = 340_000

BUFFER_SIZE  = 2500000          # Rolling buffer (~5s at 500kHz)
DISPLAY_WINDOW = 2000           # Default samples shown in plot
UPDATE_HZ    = 30               # GUI refresh rate
RATE_WINDOW_S = 2.0             # rolling window for the measured-rate readout
MAX_PLOT_POINTS = 4000          # Downsample display above this

DUTY_LABELS = {0: "10%", 1: "25%", 2: "50%", 3: "75%", 4: "90%"}
FREQ_LABELS = {0: "100 Hz", 1: "1 kHz", 2: "10 kHz", 3: "20 kHz", 4: "40 kHz"}

STYLE_ACTIVE   = ("background:#ffaa00; color:#000; border-radius:3px; "
                   "padding:2px; font-weight:bold;")
STYLE_INACTIVE = "background:#333; color:#666; border-radius:3px; padding:2px;"

STYLE_FREQ_ACTIVE   = ("background:#44aaff; color:#000; border-radius:3px; "
                        "padding:2px; font-weight:bold;")
STYLE_FREQ_INACTIVE = "background:#333; color:#666; border-radius:3px; padding:2px;"

# ---- Burst capture / trigger: GUI presentation ----
# The protocol constants themselves (BURST_CMD, BURST_SUB_*, BURST_ST_*,
# BURST_ACTION_*, the state enum) live in daq_reader and are imported above.
BURST_STATE_NAMES = {
    BURST_STATE_STREAM:    "Stream",
    BURST_STATE_IDLE:      "Idle",
    BURST_STATE_ARMED:     "Armed",
    BURST_STATE_TRIGGERED: "Triggered",
    BURST_STATE_FULL:      "Full",
    BURST_STATE_DRAINING:  "Draining",
}

def _burst_state_style(bg, fg="#000"):
    return (f"background:{bg}; color:{fg}; border-radius:3px; padding:3px; "
            f"font-weight:bold;")

BURST_STATE_STYLES = {
    BURST_STATE_STREAM:    _burst_state_style("#333", "#888"),
    BURST_STATE_IDLE:      _burst_state_style("#555", "#ccc"),
    BURST_STATE_ARMED:     _burst_state_style("#ffaa00"),
    BURST_STATE_TRIGGERED: _burst_state_style("#00ff88"),
    BURST_STATE_FULL:      _burst_state_style("#44aaff"),
    BURST_STATE_DRAINING:  _burst_state_style("#aa66ff"),
}

# (label, action, auto_ms) — Single arms once, Normal re-arms with no timeout,
# Auto re-arms and lets the device self-trigger when no edge shows up.
BURST_AUTO_MS  = 200
BURST_MODES = [
    ("Single", BURST_ACTION_ARM_SINGLE, 0),
    ("Normal", BURST_ACTION_ARM_CONT,   0),
    ("Auto",   BURST_ACTION_ARM_CONT,   BURST_AUTO_MS),
]
BURST_LENGTHS      = [1024, 2048, 4096, 8192]
BURST_PRE_PERCENTS = [0, 25, 50, 75]
BURST_SLOPES       = [("Rising", 0), ("Falling", 1)]
BURST_DEFAULT_LEVEL_V = VCC / 2

# ---- Live (display) trigger ----
# Stabilises the streaming view the way a scope does: every refresh finds the
# most recent level crossing in the ring and draws the window aligned to it,
# with the pre-trigger fraction to its left.  Level, slope and pre-trigger are
# the same controls the burst engine uses.  Host-side only; the firmware never
# knows.  Off is the old free-running scroll.
LIVE_TRIGGER_MODES = ["Off", "Auto", "Normal"]
LIVE_TRIGGER_DEFAULT = 1                # Auto, like a scope out of the box
LIVE_TRIGGER_HYST_COUNTS = 24           # ~19 mV: noise cannot re-arm the trigger
LIVE_TRIGGER_SEARCH_WINDOWS = 4         # look back this many windows for an edge


def find_last_edge(x, level, falling, hyst, first, last):
    """Index of the most recent hysteretic level crossing in x[first:last], or -1.

    Rising: the signal must first drop to (level - hyst) to arm, and the first
    sample at or above level then fires.  Falling is the mirror image.  Pure
    numpy, so even a 250 k-sample window costs a few milliseconds.
    """
    if last <= first + 1 or last > len(x):
        return -1
    seg = x[:last].astype(np.int32)
    if falling:
        seg = -seg
        level = -level
    state = np.zeros(len(seg), dtype=np.int8)
    state[seg >= level] = 1
    state[seg <= level - hyst] = -1
    idx = np.where(state != 0, np.arange(len(seg)), 0)
    np.maximum.accumulate(idx, out=idx)
    filled = state[idx]                     # forward-filled armed/fired state
    edges = np.flatnonzero((filled[1:] == 1) & (filled[:-1] == -1)) + 1
    edges = edges[edges >= first]
    return int(edges[-1]) if len(edges) else -1


# ---------------------------------------------
# Numpy-backed ring buffer for high-throughput
# ---------------------------------------------
class RingBuffer:
    """Fixed-capacity circular buffer backed by a numpy array."""

    def __init__(self, capacity, dtype=np.uint16):
        self.buf = np.zeros(capacity, dtype=dtype)
        self.capacity = capacity
        self.count = 0
        self.head = 0       # Next write position

    def extend(self, data):
        if not isinstance(data, np.ndarray):
            data = np.array(data, dtype=self.buf.dtype)
        n = len(data)
        if n == 0:
            return
        if n >= self.capacity:
            data = data[-self.capacity:]
            n = self.capacity
            self.buf[:] = data
            self.head = 0
            self.count = self.capacity
            return
        end = self.head + n
        if end <= self.capacity:
            self.buf[self.head:end] = data
        else:
            first = self.capacity - self.head
            self.buf[self.head:] = data[:first]
            self.buf[:n - first] = data[first:]
        self.head = end % self.capacity
        self.count = min(self.count + n, self.capacity)

    def last_n(self, n):
        """Return the last n elements as a contiguous numpy array."""
        n = min(n, self.count)
        if n == 0:
            return np.array([], dtype=self.buf.dtype)
        start = (self.head - n) % self.capacity
        if start + n <= self.capacity:
            return self.buf[start:start + n].copy()
        else:
            return np.concatenate([self.buf[start:], self.buf[:self.head]])

    def clear(self):
        self.count = 0
        self.head = 0

    def __len__(self):
        return self.count


# ---------------------------------------------
# Serial readers
# ---------------------------------------------
# Two readers with one interface.  ProcessReader is what the application uses;
# SerialReader is kept because the hardware test scripts drive it directly and
# because `--thread-reader` is a useful fallback when debugging the child.
#
# Why the process exists: the host's USB CDC receive buffer is a hard 16,384
# bytes (set_buffer_size is a measured no-op), which at 333 kS/s holds only
# ~32 ms of data.  pyqtgraph's paint is Python and holds the GIL; a thread
# wanting the GIL every 1 ms was measured waiting up to 25.6 ms.  In-process,
# 5 of 6 fresh runs lost data at 333 kS/s.  Out-of-process, 4 of 4 were clean.
# Decode itself costs 0.7 % of a core -- this was never about CPU.
class _ReaderBase(QObject):
    """Signals, the sample queue, and event dispatch shared by both readers."""

    duty_changed   = Signal(int)    # Duty index 0-4
    freq_changed   = Signal(int)    # Frequency index 0-4
    sample_rate_changed = Signal(int)  # Sample-rate preset index
    overflow_delta = Signal(int)    # Number of samples dropped in firmware
    stall_recovered = Signal(int)   # Firmware recovered a stalled DMA pipeline
    resynced       = Signal(int)    # Reader re-derived triplet alignment
    status_changed = Signal(str)    # Status string for UI
    burst_state    = Signal(int)    # Burst state enum 0-5
    burst_frame    = Signal(object)  # dict: samples/trig/rate_idx/slope/forced

    def __init__(self, port, baud=115200):
        super().__init__()
        self.port = port
        self.baud = baud            # Ignored by USB CDC, required by pyserial

        self._samples_lock = threading.Lock()
        self._sample_chunks = collections.deque()

        # Link health, published by the reader process every 0.5 s.  peak_queue
        # is occupancy of the 16,384-byte OS receive buffer since the last
        # report and is the single most useful number for spotting a stalled
        # drain before it turns into lost bytes.  display_dropped counts
        # messages the child threw away because the GUI fell behind -- distinct
        # from firmware overflow, which means the device lost samples.
        self.peak_queue = 0
        self.display_dropped = 0

    # -- consumed by the GUI -----------------------
    def drain_samples(self):
        """Return all decoded sample chunks received since the last GUI update."""
        with self._samples_lock:
            if not self._sample_chunks:
                return []
            chunks = list(self._sample_chunks)
            self._sample_chunks.clear()
            return chunks

    def send_burst(self, sub, arg):
        """Queue a 4-byte burst command [0xFE][0xC0|sub][argHi][argLo]."""
        self.send(daq_reader.burst_command(sub, arg))

    # -- event fan-out -----------------------------
    def _dispatch(self, kind, value):
        """Turn one decoder event into the matching Qt signal."""
        if kind == daq_reader.EV_SAMPLES:
            with self._samples_lock:
                self._sample_chunks.append(value)
        elif kind == daq_reader.EV_DUTY:
            self.duty_changed.emit(value)
        elif kind == daq_reader.EV_FREQ:
            self.freq_changed.emit(value)
        elif kind == daq_reader.EV_RATE:
            self.sample_rate_changed.emit(value)
        elif kind == daq_reader.EV_OVERFLOW:
            self.overflow_delta.emit(value)
        elif kind == daq_reader.EV_STALL:
            self.stall_recovered.emit(value)
        elif kind == daq_reader.EV_RESYNC:
            self.resynced.emit(value)
        elif kind == daq_reader.EV_BURST_STATE:
            self.burst_state.emit(value)
        elif kind == daq_reader.EV_BURST_FRAME:
            self.burst_frame.emit(value)
        elif kind == daq_reader.EV_STATUS:
            self.status_changed.emit(value)
        elif kind == daq_reader.EV_STATS:
            self.peak_queue = value.get("peak_queue", 0)
            self.display_dropped = value.get("display_dropped", 0)


class SerialReader(_ReaderBase):
    """In-process reader: a background thread owns the port.

    Retained for the hardware test scripts and as a debugging fallback.  It is
    NOT the application's default, because Qt's paint can stall this thread for
    longer than the 16 KB OS receive buffer can absorb.
    """

    def __init__(self, port, baud=115200):
        super().__init__(port, baud)
        self._stop        = threading.Event()
        self._thread      = threading.Thread(target=self._run, daemon=True)
        self._ser         = None
        self._write_queue = queue.SimpleQueue()  # GUI thread -> reader thread
        self._decoder     = ProtocolDecoder(self._dispatch)

    def start(self):
        self._stop.clear()
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def send(self, data):
        """Queue bytes for the reader thread to write (thread-safe)."""
        self._write_queue.put(data)

    def _run(self):
        try:
            ser = daq_reader.open_port(self.port, self.baud)
            self._ser = ser
            self.status_changed.emit(f"Connected: {self.port} (USB CDC)")
        except serial.SerialException as e:
            self.status_changed.emit(f"Error: {e}")
            return

        peak = 0
        try:
            while not self._stop.is_set():
                # Drain write queue first -- keeps all port I/O on this thread.
                while not self._write_queue.empty():
                    ser.write(self._write_queue.get_nowait())

                try:
                    q = ser.in_waiting
                    if q > peak:
                        peak = q
                        self.peak_queue = peak
                except (OSError, serial.SerialException):
                    pass

                self._decoder.feed(ser.read(65536))
        finally:
            self._ser = None
            ser.close()
            self.status_changed.emit("Disconnected")


class ProcessReader(_ReaderBase):
    """Out-of-process reader: a child process owns the port.

    The child has its own interpreter and its own GIL, so no amount of plotting
    in this process can delay the drain.  Only decoded samples and small events
    cross the pipe; the child never blocks on this side, dropping and counting
    instead, because a blocked sender would stall the drain.
    """

    def __init__(self, port, baud=115200):
        super().__init__(port, baud)
        self._proc = None
        self._cmd_tx = None
        self._data_rx = None
        self._rx_thread = None
        self._stop = threading.Event()

    def start(self):
        ctx = mp.get_context("spawn")
        cmd_rx, self._cmd_tx = ctx.Pipe(duplex=False)
        self._data_rx, data_tx = ctx.Pipe(duplex=False)

        self._stop.clear()
        self._proc = ctx.Process(
            target=daq_reader.reader_main,
            args=(self.port, cmd_rx, data_tx, self.baud),
            daemon=True,
        )
        self._proc.start()

        # The child holds its own duplicates; closing ours here means recv()
        # raises EOFError when the child exits instead of blocking forever.
        cmd_rx.close()
        data_tx.close()

        self._rx_thread = threading.Thread(target=self._receive, daemon=True)
        self._rx_thread.start()
        self.status_changed.emit(f"Starting reader for {self.port}...")

    def stop(self):
        self._stop.set()
        try:
            if self._cmd_tx is not None:
                self._cmd_tx.send(("stop", None))
        except (BrokenPipeError, EOFError, OSError):
            pass

        if self._proc is not None:
            self._proc.join(timeout=2.0)
            if self._proc.is_alive():
                self._proc.terminate()
                self._proc.join(timeout=1.0)
            self._proc = None

        for conn in (self._cmd_tx, self._data_rx):
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass
        self._cmd_tx = None
        self._data_rx = None

        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

    def send(self, data):
        try:
            if self._cmd_tx is not None:
                self._cmd_tx.send(("write", data))
        except (BrokenPipeError, EOFError, OSError):
            pass

    def _receive(self):
        """Block on the pipe and fan messages out as Qt signals.

        This thread can be starved by the GUI without consequence: the child is
        the one draining the serial port, and the pipe buffers far more
        generously than the 16 KB the OS gives the COM port.
        """
        conn = self._data_rx
        while not self._stop.is_set():
            try:
                kind, value = conn.recv()
            except (EOFError, OSError):
                break
            self._dispatch(kind, value)


# Which reader the GUI builds.  Overridden to SerialReader by --thread-reader.
READER_CLASS = ProcessReader

# ---------------------------------------------
# Main Window
# ---------------------------------------------
class DAQWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TM4C123G DAQ — USB CDC ADC Monitor")
        self.resize(1100, 700)

        self._reader       = None
        self._running      = False
        self._ring         = RingBuffer(BUFFER_SIZE)
        self._display_window = DISPLAY_WINDOW
        self._duty_index   = 2      # Default 50%
        self._freq_index   = 2      # Default 10 kHz
        self._sample_rate_index = DEFAULT_SAMPLE_RATE_INDEX
        self._sample_count = 0
        self._overflow_count = 0
        self._stall_count  = 0
        self._start_time   = 0.0
        self._rate_t0      = 0.0    # rolling-rate window start
        self._rate_n0      = 0
        self._stats        = {}
        self._test_harness = None

        # Burst / trigger.  _frame_mode gates the live plot: while a burst is
        # engaged the frozen frame owns the curve and the streaming timer must
        # not overwrite it.
        self._burst_state   = BURST_STATE_STREAM
        self._burst_engaged = False
        self._frame_mode    = False
        self._frame_count   = 0
        self._burst_level_counts = int(round(
            BURST_DEFAULT_LEVEL_V / VCC * ADC_MAX))
        # Level echoed by the frame on screen, or None while no frame has been
        # drawn for this engagement (then the pending spin-box value is shown).
        self._frame_level_counts = None

        # Live display trigger: streaming view only, see LIVE_TRIGGER_MODES.
        self._live_trigger_mode = LIVE_TRIGGER_DEFAULT
        self._live_look = False             # time axis and marker lines shown
        self._live_axis_key = None          # (window, pre_n, rate) last ranged

        self._build_ui()
        self._build_plot()

        self._timer = QTimer()
        self._timer.timeout.connect(self._update_plot)
        self._timer.start(1000 // UPDATE_HZ)

    # -- UI Layout ---------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(6)

        # -- Top bar: connection controls --------
        top = QHBoxLayout()

        top.addWidget(QLabel("Port:"))
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(120)
        self._refresh_ports()
        top.addWidget(self.cb_port)

        btn_refresh = QPushButton("↻")
        btn_refresh.setFixedWidth(32)
        btn_refresh.clicked.connect(self._refresh_ports)
        top.addWidget(btn_refresh)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setFixedWidth(100)
        self.btn_connect.clicked.connect(self._toggle_connection)
        top.addWidget(self.btn_connect)

        self.lbl_status = QLabel("Disconnected")
        self.lbl_status.setStyleSheet("color: #888;")
        top.addWidget(self.lbl_status)
        top.addStretch()

        self.lbl_rate = QLabel("Rate: — Hz")
        self.lbl_rate.setStyleSheet("color: #aaa;")
        top.addWidget(self.lbl_rate)

        top.addWidget(QLabel("  ADC Rate:"))
        self.cb_sample_rate = QComboBox()
        for idx, (rate, label) in enumerate(SAMPLE_RATE_PRESETS):
            if rate > LINK_LIMIT_SPS:
                # Acquires fine; the USB link cannot carry it continuously.
                self.cb_sample_rate.addItem(f"{label}  (exceeds link)", idx)
                self.cb_sample_rate.setItemData(
                    idx,
                    f"{label}: the ADC keeps up, but the USB CDC link tops out "
                    f"near {LINK_LIMIT_SPS:,} S/s. The firmware drops the excess "
                    f"and reports it in the Overflows counter. Use for burst "
                    f"capture, not continuous streaming.",
                    Qt.ToolTipRole)
            else:
                self.cb_sample_rate.addItem(label, idx)
        self.cb_sample_rate.setCurrentIndex(DEFAULT_SAMPLE_RATE_INDEX)
        self.cb_sample_rate.currentIndexChanged.connect(
            self._on_sample_rate_selection_changed
        )
        top.addWidget(self.cb_sample_rate)

        top.addWidget(QLabel("  Window:"))
        self.spin_window = QComboBox()
        for w in [100, 500, 1000, 2000, 5000, 10000, 50000, 100000, 250000]:
            self.spin_window.addItem(f"{w}", w)
        self.spin_window.setCurrentIndex(3)     # 2000 default
        self.spin_window.currentIndexChanged.connect(self._on_window_changed)
        top.addWidget(self.spin_window)

        root.addLayout(top)

        # -- Middle: plot + right panel ----------
        middle = QHBoxLayout()

        self.plot_container = QVBoxLayout()
        pw = QWidget()
        pw.setLayout(self.plot_container)
        middle.addWidget(pw, stretch=4)

        # Right panel
        right = QVBoxLayout()
        right.setSpacing(8)

        # Voltage meter
        grp_volt = QGroupBox("Voltage (Mean)")
        grp_volt.setMinimumWidth(200)
        v_layout = QGridLayout(grp_volt)

        self.lbl_current = QLabel("—")
        self.lbl_current.setAlignment(Qt.AlignCenter)
        self.lbl_current.setFixedHeight(50)
        self.lbl_current.setMinimumWidth(180)
        self.lbl_current.setStyleSheet(
            "color: #00ff88; font-size: 36px; font-weight: bold;")
        v_layout.addWidget(self.lbl_current, 0, 0, 1, 2, Qt.AlignCenter)

        v_layout.addWidget(QLabel("Min:"),    1, 0)
        self.lbl_min = QLabel("—")
        v_layout.addWidget(self.lbl_min,      1, 1)

        v_layout.addWidget(QLabel("Max:"),    2, 0)
        self.lbl_max = QLabel("—")
        v_layout.addWidget(self.lbl_max,      2, 1)

        v_layout.addWidget(QLabel("Mean:"),   3, 0)
        self.lbl_mean = QLabel("—")
        v_layout.addWidget(self.lbl_mean,     3, 1)

        v_layout.addWidget(QLabel("StdDev:"), 4, 0)
        self.lbl_std = QLabel("—")
        v_layout.addWidget(self.lbl_std,      4, 1)

        right.addWidget(grp_volt)

        # ADC raw counts
        grp_raw = QGroupBox("ADC Raw (12-bit)")
        r_layout = QGridLayout(grp_raw)

        r_layout.addWidget(QLabel("Current:"), 0, 0)
        self.lbl_raw_cur = QLabel("—")
        r_layout.addWidget(self.lbl_raw_cur,   0, 1)

        r_layout.addWidget(QLabel("Min:"), 1, 0)
        self.lbl_raw_min = QLabel("—")
        r_layout.addWidget(self.lbl_raw_min, 1, 1)

        r_layout.addWidget(QLabel("Max:"), 2, 0)
        self.lbl_raw_max = QLabel("—")
        r_layout.addWidget(self.lbl_raw_max, 2, 1)

        right.addWidget(grp_raw)

        # PWM frequency indicator
        grp_freq = QGroupBox("PWM Frequency (SW1)")
        f_layout = QVBoxLayout(grp_freq)

        self.lbl_freq = QLabel("10 kHz")
        self.lbl_freq.setAlignment(Qt.AlignCenter)
        self.lbl_freq.setFixedHeight(40)
        self.lbl_freq.setStyleSheet(
            "color: #44aaff; font-size: 28px; font-weight: bold;")
        f_layout.addWidget(self.lbl_freq)

        freq_bar = QHBoxLayout()
        self.freq_indicators = []
        for label in ["100", "1k", "10k", "20k", "40k"]:
            lbl = QLabel(label)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFixedWidth(40)
            lbl.setStyleSheet(STYLE_FREQ_INACTIVE)
            freq_bar.addWidget(lbl)
            self.freq_indicators.append(lbl)
        self.freq_indicators[2].setStyleSheet(STYLE_FREQ_ACTIVE)
        f_layout.addLayout(freq_bar)

        right.addWidget(grp_freq)

        # PWM duty cycle indicator
        grp_duty = QGroupBox("PWM Duty Cycle (SW2)")
        d_layout = QVBoxLayout(grp_duty)

        self.lbl_duty = QLabel("50%")
        self.lbl_duty.setAlignment(Qt.AlignCenter)
        self.lbl_duty.setFixedHeight(40)
        self.lbl_duty.setStyleSheet(
            "color: #ffaa00; font-size: 28px; font-weight: bold;")
        d_layout.addWidget(self.lbl_duty)

        duty_bar = QHBoxLayout()
        self.duty_indicators = []
        for pct in ["10%", "25%", "50%", "75%", "90%"]:
            lbl = QLabel(pct)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFixedWidth(36)
            lbl.setStyleSheet(STYLE_INACTIVE)
            duty_bar.addWidget(lbl)
            self.duty_indicators.append(lbl)
        self.duty_indicators[2].setStyleSheet(STYLE_ACTIVE)
        d_layout.addLayout(duty_bar)

        right.addWidget(grp_duty)

        # Burst capture / trigger
        grp_burst = QGroupBox("Burst / Trigger")
        bu_layout = QGridLayout(grp_burst)

        self.lbl_burst_state = QLabel(BURST_STATE_NAMES[BURST_STATE_STREAM])
        self.lbl_burst_state.setAlignment(Qt.AlignCenter)
        self.lbl_burst_state.setFixedHeight(26)
        self.lbl_burst_state.setStyleSheet(
            BURST_STATE_STYLES[BURST_STATE_STREAM])
        bu_layout.addWidget(self.lbl_burst_state, 0, 0, 1, 2)

        self.btn_burst_arm = QPushButton("Arm")
        self.btn_burst_arm.clicked.connect(self._on_burst_arm_clicked)
        bu_layout.addWidget(self.btn_burst_arm, 1, 0)

        self.btn_burst_force = QPushButton("Force")
        self.btn_burst_force.setToolTip(
            "Trigger the current capture immediately, edge or no edge")
        self.btn_burst_force.clicked.connect(self._on_burst_force_clicked)
        self.btn_burst_force.setEnabled(False)
        bu_layout.addWidget(self.btn_burst_force, 1, 1)

        bu_layout.addWidget(QLabel("Mode:"), 2, 0)
        self.cb_burst_mode = QComboBox()
        for i, (label, _action, _auto) in enumerate(BURST_MODES):
            self.cb_burst_mode.addItem(label, i)
        self.cb_burst_mode.setCurrentIndex(0)
        self.cb_burst_mode.setToolTip(
            "Single: one capture.  Normal: re-arm after each frame.  "
            "Auto: re-arm and self-trigger if no edge arrives.")
        self.cb_burst_mode.currentIndexChanged.connect(self._on_burst_mode_changed)
        bu_layout.addWidget(self.cb_burst_mode, 2, 1)

        bu_layout.addWidget(QLabel("Level:"), 3, 0)
        self.spin_burst_level = QDoubleSpinBox()
        self.spin_burst_level.setRange(0.0, VCC)
        self.spin_burst_level.setDecimals(3)
        self.spin_burst_level.setSingleStep(0.05)
        self.spin_burst_level.setSuffix(" V")
        self.spin_burst_level.setValue(BURST_DEFAULT_LEVEL_V)
        self.spin_burst_level.valueChanged.connect(self._on_burst_level_changed)
        bu_layout.addWidget(self.spin_burst_level, 3, 1)

        bu_layout.addWidget(QLabel("Slope:"), 4, 0)
        self.cb_burst_slope = QComboBox()
        for label, value in BURST_SLOPES:
            self.cb_burst_slope.addItem(label, value)
        self.cb_burst_slope.currentIndexChanged.connect(self._on_burst_slope_changed)
        bu_layout.addWidget(self.cb_burst_slope, 4, 1)

        bu_layout.addWidget(QLabel("Pre-trig:"), 5, 0)
        self.cb_burst_pre = QComboBox()
        for pct in BURST_PRE_PERCENTS:
            self.cb_burst_pre.addItem(f"{pct}%", pct)
        self.cb_burst_pre.setCurrentIndex(BURST_PRE_PERCENTS.index(50))
        self.cb_burst_pre.currentIndexChanged.connect(self._on_burst_pre_changed)
        bu_layout.addWidget(self.cb_burst_pre, 5, 1)

        bu_layout.addWidget(QLabel("Length:"), 6, 0)
        self.cb_burst_length = QComboBox()
        for n in BURST_LENGTHS:
            self.cb_burst_length.addItem(f"{n}", n)
        self.cb_burst_length.setCurrentIndex(len(BURST_LENGTHS) - 1)
        self.cb_burst_length.currentIndexChanged.connect(
            self._on_burst_length_changed)
        bu_layout.addWidget(self.cb_burst_length, 6, 1)

        bu_layout.addWidget(QLabel("Frames:"), 7, 0)
        self.lbl_burst_frames = QLabel("0")
        bu_layout.addWidget(self.lbl_burst_frames, 7, 1)

        bu_layout.addWidget(QLabel("Live:"), 8, 0)
        self.cb_live_trigger = QComboBox()
        for i, label in enumerate(LIVE_TRIGGER_MODES):
            self.cb_live_trigger.addItem(label, i)
        self.cb_live_trigger.setCurrentIndex(LIVE_TRIGGER_DEFAULT)
        self.cb_live_trigger.setToolTip(
            "Display trigger for the streaming view, using the level, slope "
            "and pre-trigger above.  Auto: align to the latest edge, free-run "
            "if there is none.  Normal: hold the last aligned picture until "
            "another edge arrives.  Off: free-running scroll.")
        self.cb_live_trigger.currentIndexChanged.connect(
            self._on_live_trigger_changed)
        bu_layout.addWidget(self.cb_live_trigger, 8, 1)

        right.addWidget(grp_burst)

        # Session info
        grp_info = QGroupBox("Session")
        i_layout = QGridLayout(grp_info)
        i_layout.addWidget(QLabel("Samples:"), 0, 0)
        self.lbl_samples = QLabel("0")
        i_layout.addWidget(self.lbl_samples, 0, 1)
        i_layout.addWidget(QLabel("Elapsed:"), 1, 0)
        self.lbl_elapsed = QLabel("0s")
        i_layout.addWidget(self.lbl_elapsed, 1, 1)
        i_layout.addWidget(QLabel("Overflows:"), 2, 0)
        self.lbl_overflow = QLabel("0")
        i_layout.addWidget(self.lbl_overflow, 2, 1)
        i_layout.addWidget(QLabel("DMA recoveries:"), 3, 0)
        self.lbl_stalls = QLabel("0")
        i_layout.addWidget(self.lbl_stalls, 3, 1)
        i_layout.addWidget(QLabel("Resyncs:"), 4, 0)
        self.lbl_resyncs = QLabel("0")
        i_layout.addWidget(self.lbl_resyncs, 4, 1)

        btn_clear = QPushButton("Clear Statistics")
        btn_clear.clicked.connect(self._clear_stats)
        i_layout.addWidget(btn_clear, 5, 0, 1, 2)

        self.btn_test = QPushButton("Run Test Suite")
        self.btn_test.setStyleSheet("background: #1a5c2a; font-weight: bold;")
        self.btn_test.clicked.connect(self._run_tests)
        i_layout.addWidget(self.btn_test, 4, 0, 1, 2)

        right.addWidget(grp_info)
        right.addStretch()

        right_widget = QWidget()
        right_widget.setLayout(right)

        scroll = QScrollArea()
        scroll.setWidget(right_widget)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(220)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet("QScrollArea { border: none; }")

        middle.addWidget(scroll, stretch=1)
        root.addLayout(middle, stretch=1)

        self.statusBar().showMessage(
            "Ready — connect device USB (J2) and select COM port")

    def _build_plot(self):
        pg.setConfigOption('background', '#1a1a1a')
        pg.setConfigOption('foreground', '#cccccc')

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('left',   'Voltage', units='V')
        self.plot_widget.setLabel('bottom', 'Sample',  units='')
        self.plot_widget.setTitle('ADC Input — AIN0 (PE3)', color='#cccccc')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setYRange(0, VCC)
        self.plot_widget.setMouseEnabled(x=True, y=False)
        self.plot_widget.enableAutoRange(axis='x')

        self.curve = self.plot_widget.plot(
            pen=pg.mkPen(color='#00ff88', width=1.5),
            name='ADC'
        )

        self.mean_line = pg.InfiniteLine(
            pos=VCC / 2, angle=0,
            pen=pg.mkPen(color='#ffaa00', width=1, style=Qt.DashLine),
            label='mean', labelOpts={'color': '#ffaa00'}
        )
        self.plot_widget.addItem(self.mean_line)

        # Burst markers — hidden until a frame is on screen.  The vertical line
        # sits at t=0 (the triggering sample), the horizontal one at the level.
        self.trig_line = pg.InfiniteLine(
            pos=0.0, angle=90,
            pen=pg.mkPen(color='#ff4444', width=1, style=Qt.DashLine),
            label='trig', labelOpts={'color': '#ff4444', 'position': 0.95}
        )
        self.trig_line.setVisible(False)
        self.plot_widget.addItem(self.trig_line)

        self.level_line = pg.InfiniteLine(
            pos=BURST_DEFAULT_LEVEL_V, angle=0,
            pen=pg.mkPen(color='#ff4444', width=1, style=Qt.DotLine),
            label='level', labelOpts={'color': '#ff4444', 'position': 0.05}
        )
        self.level_line.setVisible(False)
        self.plot_widget.addItem(self.level_line)

        self.plot_container.addWidget(self.plot_widget)

    # -- Connection --------------------------------
    def _refresh_ports(self):
        self.cb_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in sorted(ports):
            self.cb_port.addItem(f"{p.device}  ({p.description})", p.device)
        if not ports:
            self.cb_port.addItem("No ports found", "")

    def _toggle_connection(self):
        if not self._running:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        port = self.cb_port.currentData()
        if not port:
            self.statusBar().showMessage("Select a valid COM port")
            return

        self._reader = READER_CLASS(port)
        self._reader.duty_changed.connect(self._on_duty_changed)
        self._reader.freq_changed.connect(self._on_freq_changed)
        self._reader.sample_rate_changed.connect(self._on_sample_rate_changed)
        self._reader.overflow_delta.connect(self._on_overflow_delta)
        self._reader.stall_recovered.connect(self._on_stall_recovered)
        self._reader.resynced.connect(self._on_resynced)
        self._reader.status_changed.connect(self._on_status)
        self._reader.burst_state.connect(self._on_burst_state)
        self._reader.burst_frame.connect(self._on_burst_frame)
        self._reader.start()

        self._running      = True
        self._start_time   = time.time()
        self._sample_count = 0
        self._rate_t0      = self._start_time
        self._rate_n0      = 0
        self.btn_connect.setText("Disconnect")
        self.btn_connect.setStyleSheet("background: #8b0000;")
        self.cb_port.setEnabled(False)
        QTimer.singleShot(150, self._sync_device_settings)

    def _disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
        self._running = False
        self._on_burst_state(BURST_STATE_STREAM)
        self.btn_connect.setText("Connect")
        self.btn_connect.setStyleSheet("")
        self.cb_port.setEnabled(True)
        self.lbl_status.setText("Disconnected")
        self.lbl_status.setStyleSheet("color: #888;")

    # -- Signal handlers (from serial thread) ------
    def _on_duty_changed(self, index):
        if index >= NUM_DUTY_PRESETS:
            return
        self._duty_index = index
        label = DUTY_LABELS.get(index, "?")
        self.lbl_duty.setText(label)
        for i, ind in enumerate(self.duty_indicators):
            ind.setStyleSheet(STYLE_ACTIVE if i == index else STYLE_INACTIVE)
        self.statusBar().showMessage(
            f"PWM duty changed to {label}  (SW2 pressed)", 3000)

    def _on_freq_changed(self, index):
        if index >= NUM_FREQ_PRESETS:
            return
        self._freq_index = index
        label = FREQ_LABELS.get(index, "?")
        self.lbl_freq.setText(label)
        for i, ind in enumerate(self.freq_indicators):
            ind.setStyleSheet(
                STYLE_FREQ_ACTIVE if i == index else STYLE_FREQ_INACTIVE)
        self.statusBar().showMessage(
            f"PWM frequency changed to {label}  (SW1 pressed)", 3000)

    def _on_sample_rate_changed(self, index):
        if index >= len(SAMPLE_RATE_PRESETS):
            return
        self._sample_rate_index = index
        label = SAMPLE_RATE_PRESETS[index][1]
        self.cb_sample_rate.blockSignals(True)
        self.cb_sample_rate.setCurrentIndex(index)
        self.cb_sample_rate.blockSignals(False)
        self.statusBar().showMessage(f"ADC sample rate set to {label}", 3000)

    def _on_overflow_delta(self, delta):
        if delta <= 0:
            return
        self._overflow_count += delta
        self.lbl_overflow.setText(f"{self._overflow_count:,}")
        self.statusBar().showMessage(
            f"Firmware overflow reported: +{delta} samples", 3000)

    def _on_stall_recovered(self, count):
        # Firmware watchdog rebuilt a stalled uDMA ping-pong.  Rare; a steadily
        # climbing count means the acquisition path is being starved.
        self._stall_count += 1
        self.lbl_stalls.setText(f"{self._stall_count:,}")
        self.statusBar().showMessage(
            "Firmware recovered a stalled acquisition pipeline", 4000)

    def _on_resynced(self, count):
        # Reader re-derived triplet alignment after a byte slip.
        self.lbl_resyncs.setText(f"{count:,}")
        self.statusBar().showMessage(
            "Stream misaligned — triplet alignment recovered", 4000)

    def _on_status(self, msg):
        self.lbl_status.setText(msg)
        color = "#00cc44" if "Connected" in msg else \
                "#ff4444" if "Error" in msg else "#888"
        self.lbl_status.setStyleSheet(f"color: {color};")

    # -- Burst / trigger ---------------------------
    def _on_burst_state(self, state):
        self._burst_state = state
        name = BURST_STATE_NAMES.get(state, f"? ({state})")
        self.lbl_burst_state.setText(name)
        self.lbl_burst_state.setStyleSheet(
            BURST_STATE_STYLES.get(state, STYLE_INACTIVE))

        engaged = (state != BURST_STATE_STREAM)
        was_engaged = self._burst_engaged
        self._burst_engaged = engaged
        self.btn_burst_arm.setText("Stop" if engaged else "Arm")
        self.btn_burst_arm.setStyleSheet(
            "background: #8b0000;" if engaged else "")
        self.btn_burst_force.setEnabled(engaged)

        if engaged:
            self._enter_frame_mode()
        else:
            self._exit_frame_mode()
            if was_engaged:
                self.statusBar().showMessage("Burst disarmed — streaming", 3000)

    def _on_burst_frame(self, frame):
        samples = frame["samples"]
        if not len(samples):
            return

        self._frame_count += 1
        self.lbl_burst_frames.setText(f"{self._frame_count:,}")

        rate_idx = frame["rate_idx"]
        if rate_idx >= len(SAMPLE_RATE_PRESETS):
            rate_idx = self._sample_rate_index
        rate = SAMPLE_RATE_PRESETS[rate_idx][0]

        # X axis in milliseconds with t=0 on the triggering sample, so the
        # pre-trigger window sits at negative time like a real scope.
        trig = frame["trig"]
        t_ms = (np.arange(len(samples), dtype=np.float64) - trig) * (1000.0 / rate)
        volts = samples.astype(np.float32) * (VCC / ADC_MAX)

        # Draw the threshold from the level the device actually triggered on,
        # not from whatever the spin box holds now — the device snapshots it at
        # arm time, so a mid-capture retune must not move the line on a record
        # that already fired.
        level = frame.get("level")
        if level is None:
            level = self._burst_level_counts
        self._frame_level_counts = level

        self._enter_frame_mode()
        self.curve.setData(t_ms, volts)
        self.trig_line.setValue(0.0)
        self.level_line.setValue(level * (VCC / ADC_MAX))
        self.plot_widget.setXRange(t_ms[0], t_ms[-1], padding=0.02)

        # Reuse the streaming stats panel — the frame is just a frozen window.
        vmin, vmax = float(volts.min()), float(volts.max())
        vmean, vstd = float(volts.mean()), float(volts.std())
        self._stats = {'min': vmin, 'max': vmax, 'mean': vmean, 'std': vstd}
        self.lbl_current.setText(f"{vmean:.3f} V")
        self.lbl_min.setText(f"{vmin:.3f} V")
        self.lbl_max.setText(f"{vmax:.3f} V")
        self.lbl_mean.setText(f"{vmean:.3f} V")
        self.lbl_std.setText(f"{vstd:.4f} V")
        self.mean_line.setValue(vmean)
        self.lbl_raw_cur.setText(f"{int(samples[trig])}")
        self.lbl_raw_min.setText(f"{int(samples.min())}")
        self.lbl_raw_max.setText(f"{int(samples.max())}")

        slope = "falling" if frame["slope"] else "rising"
        how = "forced/auto" if frame["forced"] else slope
        self.statusBar().showMessage(
            f"Frame {self._frame_count}: {len(samples):,} samples @ "
            f"{SAMPLE_RATE_PRESETS[rate_idx][1]}, trigger at {trig:,} ({how}) "
            f"on {level * (VCC / ADC_MAX):.3f} V",
            5000)

    def _enter_frame_mode(self):
        """Hand the plot over to the frozen frame."""
        if self._frame_mode:
            return
        self._frame_mode = True
        self._frame_level_counts = None
        self._live_look = False
        self._live_axis_key = None
        self.plot_widget.setLabel('bottom', 'Time', units='ms')
        self.plot_widget.disableAutoRange(axis='x')
        self.trig_line.setVisible(True)
        # Nothing captured yet — show the level that is pending for the arm.
        self.level_line.setValue(self._burst_level_counts * (VCC / ADC_MAX))
        self.level_line.setVisible(True)
        self.curve.setData([])

    def _exit_frame_mode(self):
        """Give the plot back to the live scrolling trace."""
        if not self._frame_mode:
            return
        self._frame_mode = False
        self._frame_level_counts = None
        self._live_look = False
        self._live_axis_key = None
        self.plot_widget.setLabel('bottom', 'Sample', units='')
        self.plot_widget.enableAutoRange(axis='x')
        self.trig_line.setVisible(False)
        self.level_line.setVisible(False)
        self.curve.setData([])

    def _send_burst_config(self):
        """Push every config subcode; the device snapshots them when it arms."""
        if not self._reader:
            return
        _label, _action, auto_ms = BURST_MODES[self.cb_burst_mode.currentIndex()]
        self._reader.send_burst(BURST_SUB_LEVEL,   self._burst_level_counts)
        self._reader.send_burst(BURST_SUB_SLOPE,   self.cb_burst_slope.currentData())
        self._reader.send_burst(BURST_SUB_PRE_PCT, self.cb_burst_pre.currentData())
        self._reader.send_burst(BURST_SUB_LENGTH,  self.cb_burst_length.currentData())
        self._reader.send_burst(BURST_SUB_AUTO_MS, auto_ms)

    def _on_burst_arm_clicked(self):
        if not self._reader:
            self.statusBar().showMessage("Connect to device first!", 3000)
            return
        if self._burst_engaged:
            self._reader.send_burst(BURST_SUB_ACTION, BURST_ACTION_DISARM)
            self.statusBar().showMessage("Disarming burst…", 2000)
            return

        _label, action, _auto_ms = BURST_MODES[self.cb_burst_mode.currentIndex()]
        self._send_burst_config()
        self._reader.send_burst(BURST_SUB_ACTION, action)
        self.statusBar().showMessage(
            f"Arming burst ({self.cb_burst_mode.currentText()})", 2000)

    def _on_burst_force_clicked(self):
        if self._reader:
            self._reader.send_burst(BURST_SUB_ACTION, BURST_ACTION_FORCE)

    def _on_burst_mode_changed(self, _index):
        if self._reader:
            _label, _action, auto_ms = BURST_MODES[self.cb_burst_mode.currentIndex()]
            self._reader.send_burst(BURST_SUB_AUTO_MS, auto_ms)

    def _on_burst_level_changed(self, volts):
        counts = int(round(volts / VCC * ADC_MAX))
        self._burst_level_counts = max(0, min(ADC_MAX, counts))
        # Only track the spin box while no captured frame is on screen; once one
        # is, the line belongs to the level that frame fired against.
        if self._frame_mode and self._frame_level_counts is None:
            self.level_line.setValue(self._burst_level_counts * (VCC / ADC_MAX))
        if self._reader:
            self._reader.send_burst(BURST_SUB_LEVEL, self._burst_level_counts)

    def _on_burst_slope_changed(self, _index):
        if self._reader:
            self._reader.send_burst(BURST_SUB_SLOPE,
                                    self.cb_burst_slope.currentData())

    def _on_burst_pre_changed(self, _index):
        if self._reader:
            self._reader.send_burst(BURST_SUB_PRE_PCT,
                                    self.cb_burst_pre.currentData())

    def _on_burst_length_changed(self, _index):
        if self._reader:
            self._reader.send_burst(BURST_SUB_LENGTH,
                                    self.cb_burst_length.currentData())

    # -- Live display trigger ----------------------
    def _on_live_trigger_changed(self, index):
        self._live_trigger_mode = index
        if index == 0 and not self._frame_mode:
            self._set_live_look(False)
        self.statusBar().showMessage(
            f"Live trigger: {LIVE_TRIGGER_MODES[index]}", 3000)

    def _set_live_look(self, on):
        """Time axis with t=0 on the edge, plus the trigger and level lines."""
        self._live_look = on
        self._live_axis_key = None
        if on:
            self.plot_widget.setLabel('bottom', 'Time', units='ms')
            self.plot_widget.disableAutoRange(axis='x')
            self.trig_line.setValue(0.0)
            self.trig_line.setVisible(True)
            self.level_line.setVisible(True)
        else:
            self.plot_widget.setLabel('bottom', 'Sample', units='')
            self.plot_widget.enableAutoRange(axis='x')
            self.trig_line.setVisible(False)
            self.level_line.setVisible(False)

    def _live_triggered_window(self, n):
        """The newest n-sample window aligned to a level crossing, or (None, None).

        Returns (samples, pre_n) with the edge at index pre_n.  Searches the
        last few windows' worth of the ring so a low-frequency signal still
        finds an edge with a short window.
        """
        pre_n = n * self.cb_burst_pre.currentData() // 100
        post_n = n - pre_n
        search = self._ring.last_n(min(len(self._ring),
                                       LIVE_TRIGGER_SEARCH_WINDOWS * n))
        e = find_last_edge(search, self._burst_level_counts,
                           bool(self.cb_burst_slope.currentData()),
                           LIVE_TRIGGER_HYST_COUNTS,
                           pre_n, len(search) - post_n + 1)
        if e < 0:
            return None, None
        return search[e - pre_n:e + post_n], pre_n

    # -- Plot update (GUI timer) -------------------
    def _update_plot(self):
        if not self._reader:
            return

        if self._frame_mode:
            # The frozen frame owns the curve.  Anything still in the streaming
            # queue predates the mode switch — drop it rather than draw it.
            self._reader.drain_samples()
            return

        chunks = self._reader.drain_samples()

        if not chunks:
            return

        new_count = 0
        last_chunk = None
        for chunk in chunks:
            self._ring.extend(chunk)
            new_count += len(chunk)
            last_chunk = chunk

        self._sample_count += new_count

        n_disp = min(self._display_window, len(self._ring))
        mode = self._live_trigger_mode
        if mode == 0:
            if self._live_look:
                self._set_live_look(False)
            raw, t0 = self._ring.last_n(n_disp), None
        else:
            if not self._live_look:
                self._set_live_look(True)
            raw, t0 = self._live_triggered_window(n_disp)
            if raw is None:
                if mode == 2:
                    # Normal: hold the last aligned picture until an edge
                    # arrives; only the session counters move.
                    self._update_session_labels()
                    return
                # Auto: free-run, keeping the axis where the edge would be.
                raw = self._ring.last_n(n_disp)
                t0 = n_disp * self.cb_burst_pre.currentData() // 100

        self._draw_live(raw, t0, last_chunk)
        self._update_session_labels()

    def _draw_live(self, raw, t0, last_chunk):
        """Draw a streaming window.  t0 is the index shown at t=0 (the trigger
        sample) for the time axis, or None for the plain sample axis."""
        volts = raw.astype(np.float32) * (VCC / ADC_MAX)

        # Downsample using envelope outline: trace max forward, min backward.
        # This draws horizontal lines for flat portions (square wave plateaus)
        # and vertical lines for transitions — like a real oscilloscope trace.
        if len(volts) > MAX_PLOT_POINTS:
            n_chunks = MAX_PLOT_POINTS // 2
            chunk_size = len(volts) // n_chunks
            trimmed = volts[:n_chunks * chunk_size]
            chunks = trimmed.reshape(n_chunks, chunk_size)
            mins = chunks.min(axis=1)
            maxs = chunks.max(axis=1)
            centers = np.arange(n_chunks, dtype=np.float32) * chunk_size + chunk_size * 0.5
            x_plot = np.concatenate([centers, centers[::-1]])
            volts_plot = np.concatenate([maxs, mins[::-1]])
        else:
            x_plot = np.arange(len(volts), dtype=np.float32)
            volts_plot = volts

        if t0 is not None:
            rate = SAMPLE_RATE_PRESETS[self._sample_rate_index][0]
            ms_per = 1000.0 / rate
            x_plot = (x_plot - t0) * ms_per
            key = (len(raw), t0, rate)
            if key != self._live_axis_key:
                self._live_axis_key = key
                self.plot_widget.setXRange(-t0 * ms_per,
                                           (len(raw) - t0) * ms_per,
                                           padding=0.02)
            self.level_line.setValue(self._burst_level_counts * (VCC / ADC_MAX))
        self.curve.setData(x_plot, volts_plot)

        # Stats from display window (numpy vectorized — fast)
        vmin  = volts.min()
        vmax  = volts.max()
        vmean = volts.mean()
        vstd  = volts.std()
        self._stats = {'min': vmin, 'max': vmax, 'mean': vmean, 'std': vstd}

        self.lbl_current.setText(f"{vmean:.3f} V")
        self.lbl_min.setText(f"{vmin:.3f} V")
        self.lbl_max.setText(f"{vmax:.3f} V")
        self.lbl_mean.setText(f"{vmean:.3f} V")
        self.lbl_std.setText(f"{vstd:.4f} V")
        self.mean_line.setValue(vmean)

        # Raw ADC counts
        self.lbl_raw_cur.setText(f"{int(last_chunk[-1])}")
        self.lbl_raw_min.setText(f"{int(raw.min())}")
        self.lbl_raw_max.setText(f"{int(raw.max())}")

    def _update_session_labels(self):
        self.lbl_samples.setText(f"{self._sample_count:,}")
        self.lbl_overflow.setText(f"{self._overflow_count:,}")
        if self._start_time:
            now = time.time()
            self.lbl_elapsed.setText(f"{now - self._start_time:.0f}s")

            # Rolling window rather than a session average: a session average
            # never converges after a sample-rate change, and this readout is
            # what tells you whether the link is keeping up right now.
            window = now - self._rate_t0
            if window >= RATE_WINDOW_S:
                self.lbl_rate.setText(
                    f"Rate: {(self._sample_count - self._rate_n0) / window:,.0f} Hz")
                self._rate_t0 = now
                self._rate_n0 = self._sample_count

    # -- Utilities ---------------------------------
    def _clear_stats(self):
        self._ring.clear()
        self._sample_count = 0
        self._overflow_count = 0
        self._stall_count = 0
        self._start_time   = time.time()
        self._rate_t0      = self._start_time
        self._rate_n0      = 0
        self._frame_count  = 0
        self.curve.setData([])
        self.lbl_overflow.setText("0")
        self.lbl_stalls.setText("0")
        self.lbl_burst_frames.setText("0")
        if self._reader:
            self._reader.drain_samples()

    def _on_window_changed(self):
        self._display_window = self.spin_window.currentData()

    # -- Device commands (PC -> firmware) ---------
    def _send_sample_rate_command(self, index):
        if self._reader:
            self._reader.send(bytes([0xFE, 0x60 | index]))

    def _send_freq_command(self, index):
        if self._reader:
            self._reader.send(bytes([0xFE, 0x20 | index]))

    def _send_duty_command(self, index):
        if self._reader:
            self._reader.send(bytes([0xFE, 0x40 | index]))

    def _on_sample_rate_selection_changed(self, _index):
        if not self._running:
            self._sample_rate_index = self.cb_sample_rate.currentData()
            return
        self._send_sample_rate_command(self.cb_sample_rate.currentData())

    def _sync_device_settings(self):
        if not self._reader or not self._running:
            return
        self._send_sample_rate_command(self.cb_sample_rate.currentData())

    # -- Test harness ------------------------------
    def _run_tests(self):
        if not self._running:
            self.statusBar().showMessage("Connect to device first!")
            return
        self.btn_test.setEnabled(False)
        self._test_harness = TestHarness(self)
        self._test_harness.start()

    def closeEvent(self, event):
        # The refresh timer is started in __init__ and nothing else ever stops
        # it, so without this a closed window keeps calling _update_plot at
        # UPDATE_HZ for the life of the process.
        self._timer.stop()
        self._disconnect()
        event.accept()


NUM_DUTY_PRESETS = len(DUTY_LABELS)
NUM_FREQ_PRESETS = len(FREQ_LABELS)
DUTY_PERCENTS = [10, 25, 50, 75, 90]


# ---------------------------------------------
# Automated Test Harness
# ---------------------------------------------
class TestHarness(QObject):
    """Sweeps all freq × duty × window combos, captures screenshots + stats."""

    SETTLE_MS = 2500     # Time to let waveform stabilize
    TEST_WINDOWS = [500, 2000, 10000, 50000]

    def __init__(self, daq_window):
        super().__init__()
        self.daq = daq_window
        self.test_cases = []
        self.current = 0
        self.results = []
        self.output_dir = ""
        self.timer = QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self._on_timer)
        self.phase = 'apply'

    def start(self):
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_dir = os.path.join(script_dir, f'test_results_{timestamp}')
        os.makedirs(self.output_dir, exist_ok=True)

        self.test_cases = []
        for fi in range(NUM_FREQ_PRESETS):
            for di in range(NUM_DUTY_PRESETS):
                for w in self.TEST_WINDOWS:
                    self.test_cases.append({
                        'freq_idx': fi, 'duty_idx': di, 'window': w
                    })

        self.current = 0
        self.results = []
        self.phase = 'apply'
        self._on_timer()

    def _on_timer(self):
        if self.phase == 'apply':
            if self.current >= len(self.test_cases):
                self._finish()
                return

            tc = self.test_cases[self.current]
            n = len(self.test_cases)

            # Send commands to firmware and update indicators immediately
            # (don't wait for the firmware echo — it may arrive late or be missed)
            self.daq._send_freq_command(tc['freq_idx'])
            self.daq._send_duty_command(tc['duty_idx'])
            self.daq._on_freq_changed(tc['freq_idx'])
            self.daq._on_duty_changed(tc['duty_idx'])

            # Set window size
            idx = self.daq.spin_window.findData(tc['window'])
            if idx >= 0:
                self.daq.spin_window.setCurrentIndex(idx)

            # Clear for fresh data
            self.daq._clear_stats()

            self.daq.statusBar().showMessage(
                f"Test {self.current + 1}/{n}: "
                f"{FREQ_LABELS[tc['freq_idx']]} / {DUTY_LABELS[tc['duty_idx']]} / "
                f"Window {tc['window']}")

            self.phase = 'capture'
            self.timer.start(self.SETTLE_MS)

        elif self.phase == 'capture':
            tc = self.test_cases[self.current]

            # Capture screenshot
            freq_str = FREQ_LABELS[tc['freq_idx']].replace(' ', '')
            fname = f"test_{self.current:03d}_{freq_str}_{DUTY_LABELS[tc['duty_idx']]}_w{tc['window']}.png"
            fpath = os.path.join(self.output_dir, fname)
            self.daq.grab().save(fpath)

            # Collect numeric stats
            s = self.daq._stats
            duty_pct = DUTY_PERCENTS[tc['duty_idx']]
            expected_mean = VCC * duty_pct / 100.0

            issues = []
            if not s:
                issues.append("No data received")
            else:
                if abs(s['mean'] - expected_mean) > 0.3:
                    issues.append(
                        f"Mean {s['mean']:.3f}V vs expected {expected_mean:.3f}V")
                if s['min'] > 0.2:
                    issues.append(f"Min too high: {s['min']:.3f}V")
                if s['max'] < 3.0:
                    issues.append(f"Max too low: {s['max']:.3f}V")
                if self.daq._sample_count < tc['window'] // 2:
                    issues.append(
                        f"Low sample count: {self.daq._sample_count}")
                if self.daq._overflow_count:
                    issues.append(
                        f"Overflowed {self.daq._overflow_count} samples")

            self.results.append({
                'freq': FREQ_LABELS[tc['freq_idx']],
                'duty': DUTY_LABELS[tc['duty_idx']],
                'window': tc['window'],
                'screenshot': fname,
                'mean': s.get('mean'),
                'expected_mean': expected_mean,
                'min': s.get('min'),
                'max': s.get('max'),
                'std': s.get('std'),
                'samples': self.daq._sample_count,
                'overflows': self.daq._overflow_count,
                'issues': issues,
                'passed': len(issues) == 0,
            })

            self.current += 1
            self.phase = 'apply'
            self._on_timer()

    def _finish(self):
        passed = sum(1 for r in self.results if r['passed'])
        failed = len(self.results) - passed

        report_path = os.path.join(self.output_dir, 'report.txt')
        with open(report_path, 'w') as f:
            f.write("TM4C123G DAQ Test Report\n")
            f.write(f"{'=' * 74}\n")
            f.write(f"Date:   {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Tests:  {len(self.results)}  "
                    f"Passed: {passed}  Failed: {failed}\n\n")

            if failed > 0:
                f.write("FAILURES:\n")
                f.write(f"{'-' * 74}\n")
                for r in self.results:
                    if not r['passed']:
                        f.write(f"  {r['freq']:<8} {r['duty']:<5} "
                                f"Window {r['window']:<6}  ")
                        f.write(f"{', '.join(r['issues'])}\n")
                        f.write(f"    Screenshot: {r['screenshot']}\n")
                f.write("\n")

            f.write(f"{'Freq':<8} {'Duty':<5} {'Win':<7} "
                    f"{'Mean':>7} {'Expect':>7} {'Min':>7} {'Max':>7} "
                    f"{'Samples':>9} {'Oflow':>7} {'Status':<6}\n")
            f.write(f"{'-' * 74}\n")
            for r in self.results:
                def fv(v): return f"{v:.3f}" if v is not None else "  N/A"
                status = "PASS" if r['passed'] else "FAIL"
                f.write(f"{r['freq']:<8} {r['duty']:<5} {r['window']:<7} "
                        f"{fv(r['mean']):>7} {r['expected_mean']:>7.3f} "
                        f"{fv(r['min']):>7} {fv(r['max']):>7} "
                        f"{r['samples']:>9,} {r['overflows']:>7,} {status:<6}\n")

        self.daq.statusBar().showMessage(
            f"Tests complete: {passed}/{len(self.results)} passed. "
            f"Report: {self.output_dir}")
        self.daq.btn_test.setEnabled(True)

# ---------------------------------------------
# Entry point
# ---------------------------------------------
if __name__ == "__main__":
    # Required before any Process is created when this is ever frozen; harmless
    # otherwise.  The reader child re-imports daq_reader only, not this module.
    mp.freeze_support()

    if "--thread-reader" in sys.argv:
        # Fallback for debugging the child: runs the reader in this process,
        # which is measurably lossy above ~250 kS/s.
        READER_CLASS = SerialReader
        sys.argv.remove("--thread-reader")

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    from PySide6.QtGui import QPalette
    palette = QPalette()
    palette.setColor(QPalette.Window,          QColor(30, 30, 30))
    palette.setColor(QPalette.WindowText,      QColor(204, 204, 204))
    palette.setColor(QPalette.Base,            QColor(20, 20, 20))
    palette.setColor(QPalette.AlternateBase,   QColor(40, 40, 40))
    palette.setColor(QPalette.Text,            QColor(204, 204, 204))
    palette.setColor(QPalette.Button,          QColor(50, 50, 50))
    palette.setColor(QPalette.ButtonText,      QColor(204, 204, 204))
    palette.setColor(QPalette.Highlight,       QColor(0, 160, 100))
    palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)

    win = DAQWindow()
    win.show()
    sys.exit(app.exec())
