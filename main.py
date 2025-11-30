if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TempControlGUI()
    window.show()
    sys.exit(app.exec_())