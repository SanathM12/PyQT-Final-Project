import sys
import serial
import time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QLabel, QHBoxLayout
)
from PyQt5.QtCore import QTimer

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


PORT = "COM6"
BAUD = 9600
AUTO_ON_THRESHOLD = 30.0
AUTO_OFF_THRESHOLD = 25.0
POLL_INTERVAL_MS = 1000


class TempControlGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Temperature + Fan Control")

        # Serial
        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)

        # State
        self.fan_manual = False
        self.auto_mode = True
        self.temp_history = []

        # --- UI ---
        layout = QVBoxLayout()

        # Temperature Label
        self.temp_label = QLabel("Temp: -- °C")
        layout.addWidget(self.temp_label)

        # Fan Button
        self.fan_button = QPushButton("Manual: Turn Fan ON")
        self.fan_button.clicked.connect(self.toggle_fan_manual)
        layout.addWidget(self.fan_button)

        # Auto Toggle
        self.auto_button = QPushButton("Switch to MANUAL Mode")
        self.auto_button.clicked.connect(self.toggle_auto_mode)
        layout.addWidget(self.auto_button)

        # Plot Button
        self.plot_button = QPushButton("Show Temperature Plot")
        self.plot_button.clicked.connect(self.open_plot)
        layout.addWidget(self.plot_button)

        self.setLayout(layout)

        # Timer to read Arduino temps
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_temperature)
        self.timer.start(POLL_INTERVAL_MS)

    def read_temp(self):
        """Reads temperature from Arduino, returns float or None."""
        try:
            line = self.ser.readline().decode().strip()
            return float(line)
        except:
            return None

    def update_temperature(self):
        """Read data, update display, store history, auto toggle fan."""
        t = self.read_temp()
        if t is None:
            return

        # Update label
        self.temp_label.setText(f"Temp: {t:.2f} °C")

        # Store history
        self.temp_history.append(t)

        # Auto-mode logic
        if self.auto_mode:
            if t >= AUTO_ON_THRESHOLD:
                self.set_fan(True)
            elif t <= AUTO_OFF_THRESHOLD:
                self.set_fan(False)

    def set_fan(self, state: bool):
        """Send serial command to change fan state."""
        if state:
            self.ser.write(b'ON\n')
            if not self.auto_mode:
                self.fan_button.setText("Manual: Turn Fan OFF")
        else:
            self.ser.write(b'OFF\n')
            if not self.auto_mode:
                self.fan_button.setText("Manual: Turn Fan ON")

        self.fan_manual = state

    def toggle_fan_manual(self):
        """Manual user button press."""
        if self.auto_mode:
            return  # ignore manual button in auto mode

        new_state = not self.fan_manual
        self.set_fan(new_state)

    def toggle_auto_mode(self):
        """Switch between auto and manual."""
        self.auto_mode = not self.auto_mode

        if self.auto_mode:
            self.auto_button.setText("Switch to MANUAL Mode")
            self.fan_button.setEnabled(False)
        else:
            self.auto_button.setText("Switch to AUTO Mode")
            self.fan_button.setEnabled(True)

    def open_plot(self):
        """Show temp history plot."""
        plt.figure()
        plt.plot(self.temp_history)
        plt.title("Temperature Trend")
        plt.xlabel("Time (samples)")
        plt.ylabel("Temperature (°C)")
        plt.show()

    def closeEvent(self, event):
        """Ensure safe shutdown."""
        self.ser.write(b'OFF\n')
        self.ser.close()
        event.accept()
