from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from data.shared_data import SharedData

class SerialReader(QObject):
    serial_readout = pyqtSignal(str)
    calValueReceived = pyqtSignal(str)

    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.shared_data = SharedData()
        self.update_timer = QTimer()
        self.latest_data = 0
        self.running = True
        self.buf = bytearray()

    def run(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while self.running and self.controller and self.controller.isOpen():
            i = max(1, min(2048, self.controller.in_waiting))
            data = self.controller.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                self.latest_data = r.decode('utf_8').strip()
                self.serial_readout.emit(str(self.latest_data))
            else:
                self.buf.extend(data)

    def stop(self):
        self.running = False
