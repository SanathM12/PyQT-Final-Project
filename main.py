if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer

    app = QApplication(sys.argv)
    window = MotorTest()
    window.show()

    # Timer to read temperature from Arduino continuously
    window.read_timer = QTimer()
    window.read_timer.timeout.connect(window.read_temperature)
    window.read_timer.start(1000)

    # Timer to update MatPlotLib graph
    window.plot_timer = QTimer()
    window.plot_timer.timeout.connect(window.update_plot)
    window.plot_timer.start(1000)

    # Timer to perform automated fan control logic
    window.control_timer = QTimer()
    window.control_timer.timeout.connect(window.auto_fan_control)
    window.control_timer.start(1000)

    sys.exit(app.exec_())