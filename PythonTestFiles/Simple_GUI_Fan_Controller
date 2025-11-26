import sys
import serial
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton

# Change COM port to your Arduino's port
PORT = "COM6"
BAUD = 9600

class MotorTest(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Motor Test")

        # Open Serial
        self.ser = serial.Serial(PORT, BAUD, timeout=1)

        # Motor state
        self.motor_on = False

        # Layout
        layout = QVBoxLayout()

        # Button
        self.button = QPushButton("Turn Motor ON")
        self.button.clicked.connect(self.toggle_motor)

        layout.addWidget(self.button)
        self.setLayout(layout)

    def toggle_motor(self):
        if not self.motor_on:
            self.ser.write(bytes([255]))   # Turn motor ON
            self.button.setText("Turn Motor OFF")
            self.motor_on = True
        else:
            self.ser.write(bytes([0]))     # Turn motor OFF
            self.button.setText("Turn Motor ON")
            self.motor_on = False

    def closeEvent(self, event):
        # Turn motor off when closing
        self.ser.write(bytes([0]))
        self.ser.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorTest()
    window.show()
    sys.exit(app.exec_())

