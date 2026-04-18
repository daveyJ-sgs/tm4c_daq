"""
MSP430 DAQ GUI
Real-time ADC data visualization for MSP430G2553 LaunchPad

Dependencies:
    pip install pyserial pyqtgraph PySide6

Protocol (2-byte binary, self-synchronizing):
    Byte 1: 0x80 | (result >> 7)   bit7 always 1 = sync marker
    Byte 2: result & 0x7F           bit7 always 0
    Decode: value = ((byte1 & 0x7F) << 7) | byte2

Special packet:
    0xFF, duty_index  = PWM duty change notification from S2 press
"""

import sys
import collections
import threading
import time

import serial
import serial.tools.list_ports
import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QGroupBox, QGridLayout, QFrame
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont, QColor

# ─────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────
VCC          = 3.6          # LaunchPad supply voltage (V)
ADC_BITS     = 10           # ADC resolution
ADC_MAX      = (1 << ADC_BITS) - 1   # 1023
SAMPLE_RATE  = 1000         # Hz
BUFFER_SIZE    = 5000        # Total rolling buffer
DISPLAY_WINDOW = 200         # Samples shown in plot
UPDATE_HZ    = 30           # GUI refresh rate

DUTY_LABELS  = {0: "10%", 1: "25%", 2: "50%", 3: "75%", 4: "90%"}

# ─────────────────────────────────────────────
# Serial reader thread
# ─────────────────────────────────────────────
class SerialReader(QObject):
    """Runs in a background thread, parses binary protocol, feeds sample queue."""

    sample_ready   = Signal(int)    # Decoded ADC value 0-1023
    duty_changed   = Signal(int)    # Duty index 0-4
    status_changed = Signal(str)    # Status string for UI

    def __init__(self, port, baud=115200):
        super().__init__()
        self.port    = port
        self.baud    = baud
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._stop.clear()
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.status_changed.emit(f"Connected: {self.port} @ {self.baud}")
        except serial.SerialException as e:
            self.status_changed.emit(f"Error: {e}")
            return

        sync = False
        byte1 = 0

        try:
            while not self._stop.is_set():
                raw = ser.read(64)   # Read up to 64 bytes at a time
                for b in raw:
                    # Special packet: 0xFF = duty change notification
                    if b == 0xFF:
                        sync = False
                        byte1 = 0xFF
                        continue
                    if byte1 == 0xFF:
                        self.duty_changed.emit(b)
                        byte1 = 0
                        continue

                    # Normal 2-byte ADC packet
                    if b & 0x80:
                        # Byte 1: sync marker (bit7=1)
                        byte1 = b
                        sync  = True
                    elif sync:
                        # Byte 2: data byte (bit7=0)
                        value = ((byte1 & 0x7F) << 7) | (b & 0x7F)
                        if value <= ADC_MAX:
                            self.sample_ready.emit(value)
                        sync  = False
                        byte1 = 0
        finally:
            ser.close()
            self.status_changed.emit("Disconnected")


# ─────────────────────────────────────────────
# Main Window
# ─────────────────────────────────────────────
class DAQWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MSP430 DAQ — Real-Time ADC Monitor")
        self.resize(1100, 700)

        self._reader     = None
        self._running    = False
        self._samples    = collections.deque(maxlen=BUFFER_SIZE)
        self._display_window = DISPLAY_WINDOW
        self._new_data   = []       # Accumulates between GUI updates
        self._lock       = threading.Lock()
        self._duty_index = 2        # Default 50%
        self._sample_count = 0
        self._start_time   = 0.0

        self._build_ui()
        self._build_plot()

        # GUI refresh timer
        self._timer = QTimer()
        self._timer.timeout.connect(self._update_plot)
        self._timer.start(1000 // UPDATE_HZ)

    # ── UI Layout ────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(6)

        # ── Top bar: connection controls ──────
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

        top.addWidget(QLabel("  Window:"))
        self.spin_window = QComboBox()
        for w in [50, 100, 200, 500, 1000, 2000, 5000]:
            self.spin_window.addItem(f"{w} samples", w)
        self.spin_window.setCurrentIndex(2)  # 200 default
        self.spin_window.currentIndexChanged.connect(self._on_window_changed)
        top.addWidget(self.spin_window)

        root.addLayout(top)

        # ── Middle: plot + right panel ─────────
        middle = QHBoxLayout()

        # Plot placeholder (filled in _build_plot)
        self.plot_container = QVBoxLayout()
        pw = QWidget()
        pw.setLayout(self.plot_container)
        middle.addWidget(pw, stretch=4)

        # Right panel: meters and stats
        right = QVBoxLayout()
        right.setSpacing(8)

        # Voltage meter
        grp_volt = QGroupBox("Voltage (Mean)")
        grp_volt.setMinimumWidth(200)
        v_layout = QGridLayout(grp_volt)

        self.lbl_current = self._big_label("—")
        self.lbl_current.setStyleSheet("color: #00ff88; font-size: 36px; font-weight: bold;")
        v_layout.addWidget(self.lbl_current, 0, 0, 1, 2, Qt.AlignCenter)

        v_layout.addWidget(QLabel("Current:"), 1, 0)
        self.lbl_min = QLabel("—")
        v_layout.addWidget(self.lbl_min, 1, 1)

        v_layout.addWidget(QLabel("Max:"),  2, 0)
        self.lbl_max = QLabel("—")
        v_layout.addWidget(self.lbl_max, 2, 1)

        v_layout.addWidget(QLabel("Mean:"), 3, 0)
        self.lbl_mean = QLabel("—")
        v_layout.addWidget(self.lbl_mean, 3, 1)

        v_layout.addWidget(QLabel("StdDev:"), 4, 0)
        self.lbl_std = QLabel("—")
        v_layout.addWidget(self.lbl_std, 4, 1)

        right.addWidget(grp_volt)

        # ADC raw counts
        grp_raw = QGroupBox("ADC Raw (10-bit)")
        r_layout = QGridLayout(grp_raw)

        r_layout.addWidget(QLabel("Current:"), 0, 0)
        self.lbl_raw_cur = QLabel("—")
        r_layout.addWidget(self.lbl_raw_cur, 0, 1)

        r_layout.addWidget(QLabel("Min:"), 1, 0)
        self.lbl_raw_min = QLabel("—")
        r_layout.addWidget(self.lbl_raw_min, 1, 1)

        r_layout.addWidget(QLabel("Max:"), 2, 0)
        self.lbl_raw_max = QLabel("—")
        r_layout.addWidget(self.lbl_raw_max, 2, 1)

        right.addWidget(grp_raw)

        # PWM duty cycle indicator
        grp_duty = QGroupBox("PWM Duty Cycle")
        d_layout = QVBoxLayout(grp_duty)

        self.lbl_duty = self._big_label("50%")
        self.lbl_duty.setStyleSheet("color: #ffaa00; font-size: 28px; font-weight: bold;")
        d_layout.addWidget(self.lbl_duty, alignment=Qt.AlignCenter)

        d_layout.addWidget(QLabel("Press S2 on LaunchPad\nto cycle duty cycle",
                                   alignment=Qt.AlignCenter))

        # Duty bar indicators
        duty_bar = QHBoxLayout()
        self.duty_indicators = []
        for pct in ["10%", "25%", "50%", "75%", "90%"]:
            lbl = QLabel(pct)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFixedWidth(36)
            lbl.setStyleSheet("background:#333; color:#666; border-radius:3px; padding:2px;")
            duty_bar.addWidget(lbl)
            self.duty_indicators.append(lbl)
        self.duty_indicators[2].setStyleSheet(
            "background:#ffaa00; color:#000; border-radius:3px; padding:2px; font-weight:bold;")
        d_layout.addLayout(duty_bar)

        right.addWidget(grp_duty)

        # Sample counter
        grp_info = QGroupBox("Session")
        i_layout = QGridLayout(grp_info)
        i_layout.addWidget(QLabel("Samples:"), 0, 0)
        self.lbl_samples = QLabel("0")
        i_layout.addWidget(self.lbl_samples, 0, 1)
        i_layout.addWidget(QLabel("Elapsed:"), 1, 0)
        self.lbl_elapsed = QLabel("0s")
        i_layout.addWidget(self.lbl_elapsed, 1, 1)

        btn_clear = QPushButton("Clear Statistics")
        btn_clear.clicked.connect(self._clear_stats)
        i_layout.addWidget(btn_clear, 2, 0, 1, 2)

        right.addWidget(grp_info)
        right.addStretch()

        middle.addLayout(right, stretch=1)
        root.addLayout(middle, stretch=1)

        # Status bar
        self.statusBar().showMessage("Ready — select COM port and click Connect")

    def _big_label(self, text):
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignCenter)
        return lbl

    def _build_plot(self):
        pg.setConfigOption('background', '#1a1a1a')
        pg.setConfigOption('foreground', '#cccccc')

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('left',   'Voltage', units='V')
        self.plot_widget.setLabel('bottom', 'Sample',  units='')
        self.plot_widget.setTitle('ADC Input — A4 (P1.4)', color='#cccccc')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setYRange(0, VCC)
        self.plot_widget.setMouseEnabled(x=True, y=False)
        self.plot_widget.enableAutoRange(axis='x')

        # Waveform curve
        self.curve = self.plot_widget.plot(
            pen=pg.mkPen(color='#00ff88', width=1.5),
            name='ADC'
        )

        # Horizontal mean line
        self.mean_line = pg.InfiniteLine(
            pos=VCC/2, angle=0,
            pen=pg.mkPen(color='#ffaa00', width=1, style=Qt.DashLine),
            label='mean', labelOpts={'color': '#ffaa00'}
        )
        self.plot_widget.addItem(self.mean_line)

        self.plot_container.addWidget(self.plot_widget)

    # ── Connection ────────────────────────────
    def _refresh_ports(self):
        self.cb_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in sorted(ports):
            self.cb_port.addItem(p.device)
        if not ports:
            self.cb_port.addItem("No ports found")

    def _toggle_connection(self):
        if not self._running:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        port = self.cb_port.currentText()
        if not port or "No ports" in port:
            self.statusBar().showMessage("Select a valid COM port")
            return

        self._reader = SerialReader(port)
        self._reader.sample_ready.connect(self._on_sample)
        self._reader.duty_changed.connect(self._on_duty_changed)
        self._reader.status_changed.connect(self._on_status)
        self._reader.start()

        self._running    = True
        self._start_time = time.time()
        self._sample_count = 0
        self.btn_connect.setText("Disconnect")
        self.btn_connect.setStyleSheet("background: #8b0000;")
        self.cb_port.setEnabled(False)

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

    # ── Signal handlers (from serial thread) ──
    def _on_sample(self, value):
        with self._lock:
            self._new_data.append(value)

    def _on_duty_changed(self, index):
        self._duty_index = index
        label = DUTY_LABELS.get(index, "?")
        self.lbl_duty.setText(label)
        # Update duty bar indicators
        for i, ind in enumerate(self.duty_indicators):
            if i == index:
                ind.setStyleSheet(
                    "background:#ffaa00; color:#000; border-radius:3px; "
                    "padding:2px; font-weight:bold;")
            else:
                ind.setStyleSheet(
                    "background:#333; color:#666; border-radius:3px; padding:2px;")
        self.statusBar().showMessage(
            f"PWM duty changed to {label}  (S2 pressed)", 3000)

    def _on_status(self, msg):
        self.lbl_status.setText(msg)
        color = "#00cc44" if "Connected" in msg else "#ff4444" \
                if "Error" in msg else "#888"
        self.lbl_status.setStyleSheet(f"color: {color};")

    # ── Plot update (GUI timer) ───────────────
    def _update_plot(self):
        with self._lock:
            new = list(self._new_data)
            self._new_data.clear()

        if not new:
            return

        self._sample_count += len(new)
        self._samples.extend(new)

        # Display window: only what's plotted
        disp = np.array(list(self._samples)[-self._display_window:], dtype=np.float32)
        volts_disp = disp * (VCC / ADC_MAX)

        # Stats window: full buffer for stable mean/min/max
        full = np.array(self._samples, dtype=np.float32)
        volts_full = full * (VCC / ADC_MAX)

        # Update waveform (display window only)
        self.curve.setData(volts_disp)

        # Statistics from full buffer - stable, not affected by window size
        vmin  = volts_full.min()
        vmax  = volts_full.max()
        vmean = volts_full.mean()
        vstd  = volts_full.std()

        self.lbl_current.setText(f"{vmean:.3f} V")
        self.lbl_min.setText(f"{vmin:.3f} V")
        self.lbl_max.setText(f"{vmax:.3f} V")
        self.lbl_mean.setText(f"{vmean:.3f} V")
        self.lbl_std.setText(f"{vstd:.4f} V")
        self.mean_line.setValue(vmean)

        # Raw ADC counts from full buffer
        self.lbl_raw_cur.setText(f"{new[-1]}")
        self.lbl_raw_min.setText(f"{int(full.min())}")
        self.lbl_raw_max.setText(f"{int(full.max())}")

        # Session info
        self.lbl_samples.setText(f"{self._sample_count:,}")
        if self._start_time:
            elapsed = time.time() - self._start_time
            self.lbl_elapsed.setText(f"{elapsed:.0f}s")
            if elapsed > 0:
                self.lbl_rate.setText(
                    f"Rate: {self._sample_count/elapsed:.0f} Hz")

    # ── Utilities ─────────────────────────────
    def _clear_stats(self):
        self._samples.clear()
        self._sample_count = 0
        self._start_time   = time.time()
        self.curve.setData([])

    def _on_window_changed(self):
        self._display_window = self.spin_window.currentData()

    def closeEvent(self, event):
        self._disconnect()
        event.accept()


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette
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
