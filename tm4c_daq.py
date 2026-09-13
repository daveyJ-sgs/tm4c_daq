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

Note: the stream carries no sync marker, so a lost or duplicated byte would
misalign every triplet after it and decode as garbage forever.  Data triplets
can never begin with 0xFF, and the reader uses that invariant to detect the
condition and re-derive the offset.
"""

import sys
import os
import collections
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
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont, QColor

# ---------------------------------------------
# Constants
# ---------------------------------------------
VCC          = 3.3              # TM4C123G ADC reference voltage (V)
ADC_BITS     = 12               # ADC resolution
ADC_MAX      = (1 << ADC_BITS) - 1   # 4095
SAMPLE_RATE_PRESETS = [
    (100_000, "100 kS/s"),
    (200_000, "200 kS/s"),
    (250_000, "250 kS/s"),
    (333_333, "333 kS/s"),
    (400_000, "400 kS/s"),
]
DEFAULT_SAMPLE_RATE_INDEX = 1   # Default to the current reliable operating point

# Measured sustained throughput of the USB CDC link on this host: ~358 kB/s at
# 1.5 bytes/sample.  Rates above this acquire correctly but cannot be streamed
# continuously -- the firmware ring overflows and reports the drops.
LINK_LIMIT_SPS = 240_000

# Triplet realignment.  Only the b0 slot is constrained (never 0xFF), so an
# elevated 0xFF rate there means the stream has slipped.  Command triplets do
# legitimately put 0xFF in b0 but stay well under 1% even during heavy overflow
# reporting, while a slipped stream measures 10-90%.
ALIGN_TOLERANCE   = 0.05
ALIGN_MIN_BYTES   = 300         # need a reasonable sample before judging
BUFFER_SIZE  = 2500000          # Rolling buffer (~5s at 500kHz)
DISPLAY_WINDOW = 2000           # Default samples shown in plot
UPDATE_HZ    = 30               # GUI refresh rate
MAX_PLOT_POINTS = 4000          # Downsample display above this

DUTY_LABELS = {0: "10%", 1: "25%", 2: "50%", 3: "75%", 4: "90%"}
FREQ_LABELS = {0: "100 Hz", 1: "1 kHz", 2: "10 kHz", 3: "20 kHz", 4: "40 kHz"}

STYLE_ACTIVE   = ("background:#ffaa00; color:#000; border-radius:3px; "
                   "padding:2px; font-weight:bold;")
STYLE_INACTIVE = "background:#333; color:#666; border-radius:3px; padding:2px;"

STYLE_FREQ_ACTIVE   = ("background:#44aaff; color:#000; border-radius:3px; "
                        "padding:2px; font-weight:bold;")
STYLE_FREQ_INACTIVE = "background:#333; color:#666; border-radius:3px; padding:2px;"


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
# Serial reader thread
# ---------------------------------------------
class SerialReader(QObject):
    """Background thread: reads USB CDC, parses binary protocol, queues samples."""

    duty_changed   = Signal(int)    # Duty index 0-4
    freq_changed   = Signal(int)    # Frequency index 0-4
    sample_rate_changed = Signal(int)  # Sample-rate preset index
    overflow_delta = Signal(int)    # Number of samples dropped in firmware
    stall_recovered = Signal(int)   # Firmware recovered a stalled DMA pipeline
    resynced       = Signal(int)    # Reader re-derived triplet alignment
    status_changed = Signal(str)    # Status string for UI

    def __init__(self, port, baud=115200):
        super().__init__()
        self.port         = port
        self.baud         = baud          # Ignored by USB CDC, required by pyserial
        self._stop        = threading.Event()
        self._thread      = threading.Thread(target=self._run, daemon=True)
        self._ser         = None
        self._write_queue = queue.SimpleQueue()  # GUI thread → reader thread writes
        self._samples_lock = threading.Lock()
        self._sample_chunks = collections.deque()
        self._resync_count = 0

    def start(self):
        self._stop.clear()
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def send(self, data):
        """Queue bytes for the reader thread to write (thread-safe)."""
        self._write_queue.put(data)

    def drain_samples(self):
        """Return all decoded sample chunks received since the last GUI update."""
        with self._samples_lock:
            if not self._sample_chunks:
                return []
            chunks = list(self._sample_chunks)
            self._sample_chunks.clear()
            return chunks

    def _queue_samples(self, samples):
        with self._samples_lock:
            self._sample_chunks.append(samples)

    def _run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.001)
            if hasattr(ser, "set_buffer_size"):
                try:
                    ser.set_buffer_size(rx_size=1 << 20, tx_size=1 << 16)
                except (AttributeError, OSError, ValueError):
                    pass
            self._ser = ser
            self.status_changed.emit(f"Connected: {self.port} (USB CDC)")
        except serial.SerialException as e:
            self.status_changed.emit(f"Error: {e}")
            return

        # Startup drain: DTR flush has already told the firmware to reset its TX
        # buffer, but Windows may have buffered some pre-flush bytes already.
        # Wait briefly for the flush to propagate, then discard whatever is in
        # the Windows receive buffer so decoding starts at a clean triplet boundary.
        time.sleep(0.08)
        ser.read(65536)     # discard pre-flush bytes

        # Numpy vectorized triplet decoder.
        # Protocol: every 3 bytes is either a data triplet or command triplet.
        #   data:    b0 != 0xFF → sA=(b0<<4)|(b1>>4), sB=((b1&0xF)<<8)|b2
        #   command: b0 == 0xFF → [0xFF][0x20|fi or 0x40|di][0x00]
        # We maintain up to 2 leftover bytes between reads to keep triplet alignment.
        leftover = b''

        try:
            while not self._stop.is_set():
                # Drain write queue first — keeps all port I/O on this thread.
                while not self._write_queue.empty():
                    ser.write(self._write_queue.get_nowait())

                raw = ser.read(65536)
                if not raw:
                    continue

                buf = np.frombuffer(leftover + raw, dtype=np.uint8)

                # Triplet alignment guard.  The protocol has no sync marker, so
                # a single lost or duplicated byte would garble every sample
                # from here on with nothing to signal it.  Data triplets never
                # start with 0xFF; if the b0 slot says otherwise, re-derive the
                # offset from the slot that does satisfy the invariant.
                if len(buf) >= ALIGN_MIN_BYTES:
                    rates = [float((buf[o::3] == 0xFF).mean()) for o in range(3)]
                    best = min(range(3), key=lambda o: rates[o])
                    if best != 0 and rates[0] > ALIGN_TOLERANCE                             and rates[best] < rates[0] / 4:
                        buf = buf[best:]
                        self._resync_count += 1
                        self.resynced.emit(self._resync_count)

                n_triplets = len(buf) // 3
                leftover = bytes(buf[n_triplets * 3:])  # save 0-2 bytes

                if n_triplets == 0:
                    continue

                # Reshape to column vectors: each row is one triplet [b0, b1, b2]
                t = buf[:n_triplets * 3].reshape(n_triplets, 3)
                b0 = t[:, 0]
                b1 = t[:, 1].astype(np.uint16)
                b2 = t[:, 2].astype(np.uint16)

                is_cmd  = (b0 == 0xFF)
                is_data = ~is_cmd

                # Command triplets — iterate (rare, at most a few per button press)
                if is_cmd.any():
                    for cmd_b0, cmd_b1, cmd_b2 in t[is_cmd]:
                        cmd = int(cmd_b1)
                        code = cmd & 0xF0
                        arg = cmd & 0x0F
                        if code == 0x40:
                            self.duty_changed.emit(arg)
                        elif code == 0x20:
                            self.freq_changed.emit(arg)
                        elif code == 0x60:
                            self.sample_rate_changed.emit(arg)
                        elif code == 0x80:
                            delta = (arg << 8) | int(cmd_b2)
                            self.overflow_delta.emit(delta)
                        elif code == 0xA0:
                            self.stall_recovered.emit(arg)

                # Data triplets — fully vectorized, no Python loop
                if is_data.any():
                    db0 = b0[is_data].astype(np.uint16)
                    db1 = b1[is_data]
                    db2 = b2[is_data]
                    sA = (db0 << 4) | (db1 >> 4)
                    sB = ((db1 & 0x0F) << 8) | db2
                    samples = np.empty(len(sA) * 2, dtype=np.uint16)
                    samples[0::2] = sA
                    samples[1::2] = sB
                    self._queue_samples(samples)
        finally:
            self._ser = None
            ser.close()
            self.status_changed.emit("Disconnected")


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
        self._stats        = {}
        self._test_harness = None

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

        self._reader = SerialReader(port)
        self._reader.duty_changed.connect(self._on_duty_changed)
        self._reader.freq_changed.connect(self._on_freq_changed)
        self._reader.sample_rate_changed.connect(self._on_sample_rate_changed)
        self._reader.overflow_delta.connect(self._on_overflow_delta)
        self._reader.stall_recovered.connect(self._on_stall_recovered)
        self._reader.resynced.connect(self._on_resynced)
        self._reader.status_changed.connect(self._on_status)
        self._reader.start()

        self._running      = True
        self._start_time   = time.time()
        self._sample_count = 0
        self.btn_connect.setText("Disconnect")
        self.btn_connect.setStyleSheet("background: #8b0000;")
        self.cb_port.setEnabled(False)
        QTimer.singleShot(150, self._sync_device_settings)

    def _disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
        self._running = False
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

    # -- Plot update (GUI timer) -------------------
    def _update_plot(self):
        if not self._reader:
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

        # Get display window as numpy array (fast slice from ring buffer)
        n_disp = min(self._display_window, len(self._ring))
        raw = self._ring.last_n(n_disp)
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
            self.curve.setData(x_plot, volts_plot)
        else:
            self.curve.setData(volts)

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

        # Session info
        self.lbl_samples.setText(f"{self._sample_count:,}")
        self.lbl_overflow.setText(f"{self._overflow_count:,}")
        if self._start_time:
            elapsed = time.time() - self._start_time
            self.lbl_elapsed.setText(f"{elapsed:.0f}s")
            if elapsed > 0:
                self.lbl_rate.setText(
                    f"Rate: {self._sample_count / elapsed:,.0f} Hz")

    # -- Utilities ---------------------------------
    def _clear_stats(self):
        self._ring.clear()
        self._sample_count = 0
        self._overflow_count = 0
        self._stall_count = 0
        self._start_time   = time.time()
        self.curve.setData([])
        self.lbl_overflow.setText("0")
        self.lbl_stalls.setText("0")
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
