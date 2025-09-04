import sys
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QDoubleSpinBox, QSpinBox, QPushButton
import serial
import app_globals

class RPM_controller_2(QWidget):
    sendData = pyqtSignal(str)
    
    def __init__(self, shared_data):
        super().__init__()
        
        self.shared_data = shared_data
        
        layout5 = QVBoxLayout()
        self.setLayout(layout5)
        
        self.setWindowTitle("RPM kontrolleri sättimine")
        
        self.label80 = QLabel("Miinimum PWM väärtus")
        layout5.addWidget(self.label80)
        
        self.min_pwm = QSpinBox()
        self.min_pwm.setMinimum(800)
        self.min_pwm.setMaximum(1400)
        self.min_pwm.setSingleStep(10)
        self.min_pwm.setValue(self.shared_data.min_pwm)
        layout5.addWidget(self.min_pwm)
        
        self.min_pwm_button = QPushButton("Kinnita miinimum PWMi väärtus", self)
        self.min_pwm_button.clicked.connect(self.send_min_pwm)
        layout5.addWidget(self.min_pwm_button)
        
        self.label81 = QLabel("Maksimum PWM väärtus")
        layout5.addWidget(self.label81)
        
        self.max_pwm = QSpinBox()
        self.max_pwm.setMinimum(1200)
        self.max_pwm.setMaximum(2000)
        self.max_pwm.setSingleStep(10)
        self.max_pwm.setValue(self.shared_data.max_pwm)
        layout5.addWidget(self.max_pwm)
        
        self.max_pwm_button = QPushButton("Kinnita maksimum PWMi väärtus", self)
        self.max_pwm_button.clicked.connect(self.send_max_pwm)
        layout5.addWidget(self.max_pwm_button)

        # Connect signals
        self.min_pwm.valueChanged.connect(self.enable_min_pwm_button)
        self.max_pwm.valueChanged.connect(self.enable_max_pwm_button)
        
    def enable_min_pwm_button(self):
        self.min_pwm_button.setStyleSheet("background-color: orange; color: black;")
        
    def enable_max_pwm_button(self):
        self.max_pwm_button.setStyleSheet("background-color: orange; color: black;")
        
    def send_min_pwm(self):
        try:
            self.shared_data.min_pwm = self.min_pwm.value()
            min_pwm_data = 'min|%d' % (self.shared_data.min_pwm)
            self.sendData.emit(min_pwm_data)
            self.min_pwm_button.setStyleSheet("background-color: None; color: None;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            
    def send_max_pwm(self):
        try:
            self.shared_data.max_pwm = self.max_pwm.value()
            max_pwm_data = 'max|%d' % (self.shared_data.max_pwm)
            self.sendData.emit(max_pwm_data)
            self.max_pwm_button.setStyleSheet("background-color: None; color: None;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
