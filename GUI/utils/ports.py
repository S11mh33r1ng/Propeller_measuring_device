import serial.tools.list_ports

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports if 'ACM' in p.device or 'USB' in p.device]
