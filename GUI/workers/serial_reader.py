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
    # Make sure the serial port has a small timeout so read(1) doesn’t block forever
        try:
            # pyserial: both 'timeout' and 'in_waiting' are available on Serial
            if hasattr(self.controller, "timeout") and (self.controller.timeout is None or self.controller.timeout > 0.1):
                self.controller.timeout = 0.05
        except Exception:
            pass

        self.running = True

        while self.running and self.controller and self.controller.isOpen():
            # Read whatever is available; if nothing, read(1) will wait up to timeout
            n = int(getattr(self.controller, "in_waiting", 0) or 0)
            data = self.controller.read(n if n > 0 else 1)
            if not data:
                continue

            # Accumulate into buffer
            self.buf.extend(data)

            # Emit all complete lines currently in the buffer
            while True:
                i = self.buf.find(b"\n")
                if i < 0:
                    break  # no complete line yet

                # slice line (strip trailing CR if present), remove it from buffer
                raw_line = self.buf[:i].rstrip(b"\r")
                del self.buf[:i+1]

                # decode + emit
                try:
                    text = raw_line.decode("utf-8", errors="replace").strip()
                except Exception:
                    text = repr(raw_line)

                self.latest_data = text
                self.serial_readout.emit(text)

    def stop(self):
        self.running = False
