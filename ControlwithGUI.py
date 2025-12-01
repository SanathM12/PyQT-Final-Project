import sys
import time
from collections import deque

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QLineEdit, QComboBox, QGroupBox, QFormLayout, QMessageBox, QCheckBox
)


# ----------------------------
# Worker thread: serial read + control
# ----------------------------
class SerialWorker(QThread):
    data_signal = pyqtSignal(float, int, float)  # temp, pwm, timestamp
    error_signal = pyqtSignal(str)
    mode_changed = pyqtSignal(int, float)  # new_mode, timestamp

    MODE_BANG = 0
    MODE_PID = 1
    MODE_EMERGENCY = 2

    def __init__(self, port, baud=9600, update_interval=0.5, dry_run=False):
        super().__init__()
        self.port = port
        self.baud = baud
        self.update_interval = update_interval
        self.dry_run = dry_run
        self._running = False
        self._ser = None

        # control params (defaults)
        self.mode = SerialWorker.MODE_BANG
        self.bb_low = 24.0
        self.bb_high = 26.0
        self.pid_target = 25.0
        self.Kp = 20.0
        self.Ki = 0.5
        self.Kd = 5.0
        self.emergency_temp = 40.0

        # PID internals
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

        # last pwm (for hysteresis maintaining / simulation)
        self._last_pwm = 0

        # --- Simulation State (Dry Run) ---
        self.sim_current_temp = 20.0  # Starting simulated temperature
        self.ambient_temp = 25.0
        self.cooling_factor = 3.0
        self.heating_inertia = 0.2

    def simulate_temperature(self, dt):
        """Simulates temperature change based on fan output."""
        fan_cooling_effect = (self._last_pwm / 255.0) * self.cooling_factor * dt
        heating_rate = (self.ambient_temp - self.sim_current_temp) * self.heating_inertia * dt

        new_temp = self.sim_current_temp + heating_rate - fan_cooling_effect

        new_temp = max(18.0, min(50.0, new_temp))

        self.sim_current_temp = new_temp
        return new_temp

    def run(self):
        # Open serial (only if not in dry run mode)
        if not self.dry_run:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=1)
                time.sleep(2)
            except Exception as e:
                self.error_signal.emit(f"Could not open serial port {self.port}: {e}")
                return

        self._running = True
        self._last_time = time.time()

        while self._running:
            temp = None

            # --- Read Data (Actual or Simulated) ---
            if not self.dry_run:
                try:
                    raw = self._ser.readline().decode('utf-8', errors='ignore').strip()
                    if raw:
                        parts = raw.split(",")
                        if len(parts) >= 1:
                            temp = float(parts[0])
                except Exception as e:
                    self.error_signal.emit(f"Serial read error: {e}")
                    temp = None
            else:
                # Simulation Read
                cur_time = time.time()
                dt = cur_time - self._last_time if self._last_time is not None else 0.0
                temp = self.simulate_temperature(dt)
                self._last_time = cur_time

            # Compute dt for control loop
            cur_time = time.time()
            dt = cur_time - self._last_time if self._last_time is not None else 0.0

            # Decide PWM based on mode
            pwm = self._last_pwm
            if temp is not None:
                if self.mode == SerialWorker.MODE_BANG:
                    if temp >= self.bb_high:
                        pwm = 255
                    elif temp <= self.bb_low:
                        pwm = 0
                elif self.mode == SerialWorker.MODE_PID:
                    error = temp - self.pid_target

                    self._integral += error * dt
                    self._integral = max(-50, min(50, self._integral))

                    derivative = (error - self._last_error) / dt if dt > 0 else 0.0
                    self._last_error = error

                    output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
                    pwm = int(max(0, min(255, output)))

                elif self.mode == SerialWorker.MODE_EMERGENCY:
                    if temp >= self.emergency_temp:
                        pwm = 255
                    else:
                        pwm = 0

            # Send pwm byte to Arduino
            if not self.dry_run:
                try:
                    if self._ser and self._ser.is_open:
                        self._ser.write(bytes([pwm]))
                except Exception as e:
                    self.error_signal.emit(f"Serial write error: {e}")

            # store last pwm
            self._last_pwm = pwm

            # Emit data for GUI plotting
            timestamp = time.time()
            if temp is not None:
                self.data_signal.emit(temp, pwm, timestamp)

            self._last_time = timestamp

            time.sleep(self.update_interval)

        # Cleanup
        if not self.dry_run:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(bytes([0]))
                finally:
                    self._ser.close()

    def stop(self):
        self._running = False
        self.wait()

    def set_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            self._integral = 0.0
            self._last_error = 0.0
            self.mode_changed.emit(mode, time.time())

    def set_bb_thresholds(self, low, high):
        self.bb_low = low
        self.bb_high = high

    def set_pid_params(self, target, Kp, Ki, Kd):
        self.pid_target = target
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self._integral = 0.0
        self._last_error = 0.0

    def set_emergency_threshold(self, temp):
        self.emergency_temp = temp


# ----------------------------
# Main GUI
# ----------------------------
class FanControllerGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Fan Controller (PyQt + PID/Bang-Bang/Emergency)")
        self.resize(1000, 700)

        # Data buffer
        self.max_points = 600
        self.times = deque(maxlen=self.max_points)
        self.temps = deque(maxlen=self.max_points)
        self.pwms = deque(maxlen=self.max_points)
        self.mode_changes = []

        # Reference time for plotting
        self.t0 = None

        self._create_ui()
        self.worker = None
        self.plot_timer = QTimer()
        self.plot_timer.setInterval(500)
        self.plot_timer.timeout.connect(self._update_plot)

    def _create_ui(self):
        layout = QVBoxLayout(self)

        top_row = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_ports()
        self.refresh_button = QPushButton("Refresh Ports")
        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.setEnabled(False)

        self.dry_run_checkbox = QCheckBox("Simulation Mode (Dry Run)")
        self.dry_run_checkbox.setChecked(True)

        top_row.addWidget(QLabel("Serial Port:"))
        top_row.addWidget(self.port_combo)
        top_row.addWidget(self.refresh_button)
        top_row.addWidget(self.dry_run_checkbox)
        top_row.addWidget(self.connect_button)
        top_row.addWidget(self.disconnect_button)
        layout.addLayout(top_row)

        mode_box = QGroupBox("Control Mode")
        mode_layout = QHBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Bang-Bang", SerialWorker.MODE_BANG)
        self.mode_combo.addItem("PID", SerialWorker.MODE_PID)
        self.mode_combo.addItem("Emergency", SerialWorker.MODE_EMERGENCY)
        mode_layout.addWidget(QLabel("Mode:"))
        mode_layout.addWidget(self.mode_combo)
        mode_box.setLayout(mode_layout)
        layout.addWidget(mode_box)

        params_layout = QHBoxLayout()

        bb_group = QGroupBox("Bang-Bang")
        bb_form = QFormLayout()
        self.bb_low_input = QLineEdit("24.0")
        self.bb_high_input = QLineEdit("26.0")
        bb_form.addRow("Low (OFF below):", self.bb_low_input)
        bb_form.addRow("High (ON above):", self.bb_high_input)
        bb_group.setLayout(bb_form)
        params_layout.addWidget(bb_group)

        pid_group = QGroupBox("PID")
        pid_form = QFormLayout()
        self.pid_target_input = QLineEdit("25.0")
        self.kp_input = QLineEdit("20.0")
        self.ki_input = QLineEdit("0.5")
        self.kd_input = QLineEdit("5.0")
        pid_form.addRow("Target Temp (°C):", self.pid_target_input)
        pid_form.addRow("Kp:", self.kp_input)
        pid_form.addRow("Ki:", self.ki_input)
        pid_form.addRow("Kd:", self.kd_input)
        pid_group.setLayout(pid_form)
        params_layout.addWidget(pid_group)

        em_group = QGroupBox("Emergency")
        em_form = QFormLayout()
        self.em_temp_input = QLineEdit("40.0")
        em_form.addRow("Emergency Temp (°C):", self.em_temp_input)
        em_group.setLayout(em_form)
        params_layout.addWidget(em_group)

        layout.addLayout(params_layout)

        buttons = QHBoxLayout()
        self.apply_button = QPushButton("Apply Params")
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        buttons.addWidget(self.apply_button)
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.stop_button)
        layout.addLayout(buttons)

        self.fig, self.ax_temp = plt.subplots(1, 1, figsize=(8, 6))
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        self.ax_temp.set_ylabel("Temperature °C")
        self.ax_temp.set_xlabel("Time (s)")
        self.fig.tight_layout()

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.connect_serial)
        self.disconnect_button.clicked.connect(self.disconnect_serial)
        self.apply_button.clicked.connect(self.apply_params)
        self.start_button.clicked.connect(self.start_worker)
        self.stop_button.clicked.connect(self.stop_worker)
        self.mode_combo.currentIndexChanged.connect(self._on_mode_change_ui)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)

    def connect_serial(self):
        dry_run = self.dry_run_checkbox.isChecked()
        port = self.port_combo.currentData() if self.port_combo.count() > 0 else "SIMULATION"

        if not dry_run and self.port_combo.count() == 0:
            QMessageBox.warning(self, "No port", "No serial ports found. Plug in Arduino or enable Simulation Mode.")
            return

        self.worker = SerialWorker(port, dry_run=dry_run)

        self.worker.data_signal.connect(self._on_new_data)
        self.worker.error_signal.connect(self._on_error)
        self.worker.mode_changed.connect(self._on_mode_changed_from_worker)
        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)

        status_msg = "Simulation Worker prepared." if dry_run else f"Serial port {port} prepared."
        QMessageBox.information(self, "Worker prepared", f"{status_msg} Hit Start to begin.")

    def disconnect_serial(self):
        if self.worker and self.worker.isRunning():
            QMessageBox.warning(self, "Running", "Stop worker before disconnecting.")
            return
        self.worker = None
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        QMessageBox.information(self, "Disconnected", "Serial detached.")

    def apply_params(self):
        if not self.worker:
            QMessageBox.warning(self, "No serial", "Connect serial (Prepare) before applying params.")
            return
        try:
            bb_low = float(self.bb_low_input.text())
            bb_high = float(self.bb_high_input.text())
            pid_target = float(self.pid_target_input.text())
            Kp = float(self.kp_input.text())
            Ki = float(self.ki_input.text())
            Kd = float(self.kd_input.text())
            em_temp = float(self.em_temp_input.text())
        except ValueError:
            QMessageBox.warning(self, "Bad input", "Please enter numeric values for all parameters.")
            return

        self.worker.set_bb_thresholds(bb_low, bb_high)
        self.worker.set_pid_params(pid_target, Kp, Ki, Kd)
        self.worker.set_emergency_threshold(em_temp)

        mode = self.mode_combo.currentData()
        self.worker.set_mode(mode)

        QMessageBox.information(self, "Parameters applied", "Parameters sent to controller (Python-side).")

    def start_worker(self):
        if not self.worker:
            QMessageBox.warning(self, "No serial", "Connect serial (Prepare) first.")
            return
        if self.worker.isRunning():
            QMessageBox.warning(self, "Already running", "Worker is already running.")
            return

        self.times.clear()
        self.temps.clear()
        self.pwms.clear()
        self.mode_changes.clear()
        self.t0 = None

        self.apply_params()

        self.worker.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.plot_timer.start()
        QMessageBox.information(self, "Started", "Control loop started.")

    def stop_worker(self):
        if self.worker and self.worker.isRunning():
            self.worker.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.plot_timer.stop()
        QMessageBox.information(self, "Stopped", "Control loop stopped.")

    def _on_new_data(self, temp, pwm, timestamp):
        if self.t0 is None:
            self.t0 = timestamp
            rel_time = 0.0
        else:
            rel_time = timestamp - self.t0

        self.times.append(rel_time)
        self.temps.append(temp)
        self.pwms.append(pwm)

    def _on_error(self, msg):
        QMessageBox.critical(self, "Serial Error", msg)

    def _on_mode_changed_from_worker(self, mode, timestamp):
        if self.t0 is None:
            self.t0 = time.time()
            t_ref = self.t0
        else:
            t_ref = self.t0

        self.mode_changes.append((mode, timestamp - t_ref))
        self._update_plot()

    def _on_mode_change_ui(self, idx):
        mode = self.mode_combo.itemData(idx)
        if self.worker:
            self.worker.set_mode(mode)

            if self.t0 is None:
                self.t0 = time.time()
                t_ref = self.t0
            else:
                t_ref = self.t0

            self.mode_changes.append((mode, time.time() - t_ref))
            self._update_plot()

    def _update_plot(self):
        if len(self.times) == 0:
            return

        self.ax_temp.cla()

        self.ax_pwm_twin = self.ax_temp.twinx()

        temp_line, = self.ax_temp.plot(self.times, self.temps, label='Temperature', color='blue')
        self.ax_temp.set_ylabel("Temperature °C", color='blue')
        self.ax_temp.tick_params(axis='y', labelcolor='blue')
        self.ax_temp.set_xlabel("Time (s)")

        t_min = self.times[0]
        t_max = self.times[-1]
        self.ax_temp.set_xlim(t_min, t_max + 1)

        pwm_line, = self.ax_pwm_twin.plot(self.times, self.pwms, drawstyle='steps-post', label='PWM Output',
                                          color='red', alpha=0.6)
        self.ax_pwm_twin.set_ylabel("PWM (0-255)", color='red')
        self.ax_pwm_twin.tick_params(axis='y', labelcolor='red')
        self.ax_pwm_twin.set_ylim(0, 255)

        # Plot Setpoints/Thresholds
        if self.worker and self.worker.mode == SerialWorker.MODE_PID:
            self.ax_temp.axhline(self.worker.pid_target, color='r', linestyle=':', label='Target Temp')

        if self.worker and self.worker.mode == SerialWorker.MODE_BANG:
            self.ax_temp.axhline(self.worker.bb_high, color='orange', linestyle='--', label='BB High')
            self.ax_temp.axhline(self.worker.bb_low, color='green', linestyle='--', label='BB Low')

        if self.worker and self.worker.emergency_temp is not None:
            self.ax_temp.axhline(self.worker.emergency_temp, color='purple', linestyle='-', linewidth=2,
                                 label='Emergency Temp')

        lines, labels = self.ax_temp.get_legend_handles_labels()
        lines2, labels2 = self.ax_pwm_twin.get_legend_handles_labels()
        self.ax_temp.legend(lines + lines2, labels + labels2, loc='upper right', fontsize=9)

        max_temp = max(self.temps) if self.temps else 30
        for (mode, t_rel) in self.mode_changes:
            label = {0: "Bang-Bang", 1: "PID", 2: "Emergency"}.get(mode, str(mode))
            self.ax_temp.axvline(x=t_rel, color='k', linestyle='--', linewidth=1, alpha=0.5)
            self.ax_temp.text(t_rel, max_temp * 0.98, label, rotation=90, va='top', ha='right', fontsize=8,
                              backgroundcolor='w', alpha=0.8)

        self.fig.tight_layout()
        self.canvas.draw_idle()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = FanControllerGUI()
    w.show()
    sys.exit(app.exec_())