import sys
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QDoubleSpinBox, QSpinBox, QPushButton, QCheckBox
import serial
import app_globals

class AoA_AoSS(QWidget):
    sendData = pyqtSignal(str)
    
    def __init__(self, shared_data):
        super().__init__()
        
        self.shared_data = shared_data
        
        layout3 = QVBoxLayout()
        self.setLayout(layout3)
        
        self.setWindowTitle("AoA ja AoSS telje parameetrite sättimine")
        
        self.label50 = QLabel("AoA telje trimm (kraadides)")
        layout3.addWidget(self.label50)
        
        self.aoa_trim = QSpinBox()
        self.aoa_trim.setMinimum(-120)
        self.aoa_trim.setMaximum(120)
        self.aoa_trim.setSingleStep(1)
        self.aoa_trim.setValue(self.shared_data.aoa_trim)
        layout3.addWidget(self.aoa_trim)
        
        self.aoa_trim_button = QPushButton("Kinnita AoA trimm", self)
        self.aoa_trim_button.clicked.connect(self.send_aoa_trim)
        layout3.addWidget(self.aoa_trim_button)
        
        self.label51 = QLabel("Maksimaalne AoA telje käik 0-punkti suhtes (kraadides)")
        layout3.addWidget(self.label51)
        
        self.aoa_limit = QSpinBox()
        self.aoa_limit.setMinimum(1)
        self.aoa_limit.setMaximum(80)
        self.aoa_limit.setSingleStep(1)
        self.aoa_limit.setValue(self.shared_data.aoa_limit)
        layout3.addWidget(self.aoa_limit)
        
        self.aoa_limit_button = QPushButton("Kinnita AoA piirväärtus", self)
        self.aoa_limit_button.clicked.connect(self.send_aoa_limit)
        layout3.addWidget(self.aoa_limit_button)
        
        self.label52 = QLabel("AoSS telje trimm (kraadides)")
        layout3.addWidget(self.label52)
        
        self.aoss_trim = QSpinBox()
        self.aoss_trim.setMinimum(-120)
        self.aoss_trim.setMaximum(120)
        self.aoss_trim.setSingleStep(1)
        self.aoss_trim.setValue(self.shared_data.aoss_trim)
        layout3.addWidget(self.aoss_trim)
        
        self.aoss_trim_button = QPushButton("Kinnita AoSS trimm", self)
        self.aoss_trim_button.clicked.connect(self.send_aoss_trim)
        layout3.addWidget(self.aoss_trim_button)
        
        self.label53 = QLabel("Maksimaalne AoSS telje käik 0-punkti suhtes (kraadides)")
        layout3.addWidget(self.label53)
        
        self.aoss_max_limit = QSpinBox()
        self.aoss_max_limit.setMinimum(1)
        self.aoss_max_limit.setMaximum(200)
        self.aoss_max_limit.setSingleStep(1)
        self.aoss_max_limit.setValue(self.shared_data.aoss_max_limit)
        layout3.addWidget(self.aoss_max_limit)
        
        self.aoss_min_limit = QSpinBox()
        self.aoss_min_limit.setMinimum(1)
        self.aoss_min_limit.setMaximum(200)
        self.aoss_min_limit.setSingleStep(1)
        self.aoss_min_limit.setValue(self.shared_data.aoss_min_limit)
        layout3.addWidget(self.aoss_min_limit)
        
        self.aoss_limit_button = QPushButton("Kinnita AoSS piirväärtus", self)
        self.aoss_limit_button.clicked.connect(self.send_aoss_limit)
        layout3.addWidget(self.aoss_limit_button)
        
        self.aoss_enabled_button = QCheckBox("AoSS telg aktiivne", self)
        self.aoss_enabled_button.setChecked(True)
        layout3.addWidget(self.aoss_enabled_button)

        # Connect signals
        self.aoa_trim.valueChanged.connect(self.enable_aoa_trim_button)
        self.aoa_limit.valueChanged.connect(self.enable_aoa_limit_button)
        self.aoss_trim.valueChanged.connect(self.enable_aoss_trim_button)
        self.aoss_max_limit.valueChanged.connect(self.enable_aoss_limit_button)
        self.aoss_min_limit.valueChanged.connect(self.enable_aoss_limit_button)
        self.aoss_enabled_button.stateChanged.connect(self.send_aoss_enable_flag)
        
    def enable_aoa_trim_button(self):
        self.aoa_trim_button.setStyleSheet("background-color: orange; color: black;")
        
    def enable_aoa_limit_button(self):
        self.aoa_limit_button.setStyleSheet("background-color: orange; color: black;")
        
    def enable_aoss_trim_button(self):
        self.aoss_trim_button.setStyleSheet("background-color: orange; color: black;")
        
    def enable_aoss_limit_button(self):
        self.aoss_limit_button.setStyleSheet("background-color: orange; color: black;")
        
    def send_aoa_trim(self):
        try:
            self.shared_data.aoa_trim = self.aoa_trim.value()
            aoa_trim_data = 'trimAoA|%d' % (self.shared_data.aoa_trim)
            self.sendData.emit(aoa_trim_data)
            #self.aoa_trim_button.setEnabled(False)
            self.aoa_trim_button.setStyleSheet("background-color: None; color: None;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            
    def send_aoa_limit(self):
        try:
            self.shared_data.aoa_limit = self.aoa_limit.value()
            aoa_limit_data = 'AoAlim|%d' % (self.shared_data.aoa_limit)
            self.sendData.emit(aoa_limit_data)
            #self.aoa_limit_button.setEnabled(False)
            self.aoa_limit_button.setStyleSheet("background-color: None; color: None;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
    
    def send_aoss_trim(self):
        try:
            self.shared_data.aoss_trim = self.aoss_trim.value()
            aoss_trim_data = 'trimAoSS|%d' % (self.shared_data.aoss_trim)
            self.sendData.emit(aoss_trim_data)
            #self.aoss_trim_button.setEnabled(False)
            self.aoss_trim_button.setStyleSheet("background-color: None; color: None;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            
    def send_aoss_limit(self):
        try:
            self.shared_data.aoss_max_limit = self.aoss_max_limit.value()
            self.shared_data.aoss_min_limit = self.aoss_min_limit.value()
            aoss_max_limit_data = 'AoSSLimMax|%d' % (self.shared_data.aoss_max_limit)
            self.sendData.emit(aoss_max_limit_data)
            aoss_min_limit_data = 'AoSSLimMin|%d' % (self.shared_data.aoss_min_limit)
            self.sendData.emit(aoss_min_limit_data)
            #self.aoss_limit_button.setEnabled(False)
            self.aoss_limit_button.setStyleSheet("background-color: None; color: None;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            
    def send_aoss_enable_flag(self, state):
        try:
            if state == 2:
                self.sendData.emit("enableAoSS")
            else:
                self.sendData.emit("disableAoSS")
        except:
            print("AoSS set failed")