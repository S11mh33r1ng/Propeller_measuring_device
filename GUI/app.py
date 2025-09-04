#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication
from ui.main_window import MainWindow
import app_globals

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    app_globals.window = window  # expose the main window to other modules
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
