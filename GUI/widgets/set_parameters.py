import sys
from PyQt5.QtCore import pyqtSignal, QTimer, pyqtSlot
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QDoubleSpinBox, QSpinBox, QPushButton, QCheckBox
import serial
import app_globals

class SetParameters(QWidget):
    sendData = pyqtSignal(str)

    def __init__(self, shared_data):
        super().__init__()
        self.shared_data = shared_data
        layout1 = QVBoxLayout(); self.setLayout(layout1)
        self.setWindowTitle("Parameetrite sättimine")
        
        self.label1 = QLabel("Õhutihedus (kg/m3)")
        layout1.addWidget(self.label1)
        
        self.rho = QDoubleSpinBox()
        self.rho.setMinimum(0)
        self.rho.setDecimals(3)
        self.rho.setSingleStep(0.001)
        self.rho.setValue(self.shared_data.rho)
        layout1.addWidget(self.rho)
        
        self.label2 = QLabel("Kinemaatiline viskoossus (m2/s)")
        layout1.addWidget(self.label2)
        
        self.kin_visc = QDoubleSpinBox()
        self.kin_visc.setMinimum(0)
        self.kin_visc.setDecimals(2)
        self.kin_visc.setSingleStep(0.01)
        self.kin_visc.setValue(self.shared_data.kin_visc)
        layout1.addWidget(self.kin_visc)
        
        self.tandem = QCheckBox("Tandemrootor", self)
        self.tandem.clicked.connect(self.set_tandem_flag)
        layout1.addWidget(self.tandem)
        
        self.label_probe_offset = QLabel("Pitot' nihe tsentri suhtes (mm)")
        layout1.addWidget(self.label_probe_offset)

        self.probe_offset = QDoubleSpinBox()
        self.probe_offset.setMinimum(0.0)   # or safe range for your rig
        self.probe_offset.setMaximum(50.0)
        self.probe_offset.setSingleStep(0.1)
        self.probe_offset.setDecimals(2)
        self.probe_offset.setValue(getattr(self.shared_data, "probe_offset", 0.0))
        self.probe_offset.setEnabled(False)
        layout1.addWidget(self.probe_offset)

        # when user clicks Apply / Save:
        #self.shared_data.probe_offset_mm = float(self.probeOffsetSpin.value())

        
        self.label3 = QLabel("1. posti pöördemomendi õla pikkus (mm)")
        layout1.addWidget(self.label3)
        
        self.first_trq_arm_length = QDoubleSpinBox()
        self.first_trq_arm_length.setMinimum(0.00)
        self.first_trq_arm_length.setSingleStep(0.01)
        self.first_trq_arm_length.setValue(self.shared_data.first_trq_arm_length)
        layout1.addWidget(self.first_trq_arm_length)
        
        self.label4 = QLabel("1. posti pöördemomendi koormusanduri kal. faktor")
        layout1.addWidget(self.label4)
        
        self.first_trq_lc_factor = QDoubleSpinBox()
        self.first_trq_lc_factor.setMinimum(-sys.float_info.max)
        self.first_trq_lc_factor.setMaximum(sys.float_info.max)
        self.first_trq_lc_factor.setSingleStep(0.01)
        self.first_trq_lc_factor.setValue(self.shared_data.first_trq_cal_val)
        layout1.addWidget(self.first_trq_lc_factor)
        
        self.label5 = QLabel("1. posti tõmbeanduri kordaja (pöörlemistelg/survepunkt)")
        layout1.addWidget(self.label5)
        
        self.first_thr_arm_length = QDoubleSpinBox()
        self.first_thr_arm_length.setMinimum(-sys.float_info.max)
        self.first_thr_arm_length.setMaximum(sys.float_info.max)
        self.first_thr_arm_length.setSingleStep(0.01)
        self.first_thr_arm_length.setValue(self.shared_data.first_thr_arm_length)
        layout1.addWidget(self.first_thr_arm_length)
        
        self.label6 = QLabel("1. posti tõmbe koormusanduri kal.faktor")
        layout1.addWidget(self.label6)
        
        self.first_thr_lc_factor = QDoubleSpinBox()
        self.first_thr_lc_factor.setMinimum(-sys.float_info.max)
        self.first_thr_lc_factor.setMaximum(sys.float_info.max)
        self.first_thr_lc_factor.setSingleStep(0.01)
        self.first_thr_lc_factor.setValue(self.shared_data.first_thr_cal_val)
        layout1.addWidget(self.first_thr_lc_factor)
        
        self.label7 = QLabel("2. posti pöördemomendi õla pikkus (mm)")
        layout1.addWidget(self.label7)
        
        self.second_trq_arm_length = QDoubleSpinBox()
        self.second_trq_arm_length.setMinimum(0.00)
        self.second_trq_arm_length.setSingleStep(0.01)
        self.second_trq_arm_length.setValue(self.shared_data.second_trq_arm_length)
        layout1.addWidget(self.second_trq_arm_length)
        
        self.label8 = QLabel("2. posti pöördemomendi koormusanduri kal. faktor")
        layout1.addWidget(self.label8)
        
        self.second_trq_lc_factor = QDoubleSpinBox()
        self.second_trq_lc_factor.setMinimum(-sys.float_info.max)
        self.second_trq_lc_factor.setMaximum(sys.float_info.max)
        self.second_trq_lc_factor.setSingleStep(0.01)
        self.second_trq_lc_factor.setValue(self.shared_data.second_trq_cal_val)
        layout1.addWidget(self.second_trq_lc_factor)
        
        self.label9 = QLabel("2. posti tõmbeanduri kordaja (pöörlemistelg/survepunkt)")
        layout1.addWidget(self.label9)
        
        self.second_thr_arm_length = QDoubleSpinBox()
        self.second_thr_arm_length.setMinimum(-sys.float_info.max)
        self.second_thr_arm_length.setMaximum(sys.float_info.max)
        self.second_thr_arm_length.setSingleStep(0.01)
        self.second_thr_arm_length.setValue(self.shared_data.second_thr_arm_length)
        layout1.addWidget(self.second_thr_arm_length)
        
        self.label10 = QLabel("2. posti tõmbe koormusanduri kal.faktor")
        layout1.addWidget(self.label10)
        
        self.second_thr_lc_factor = QDoubleSpinBox()
        self.second_thr_lc_factor.setMinimum(-sys.float_info.max)
        self.second_thr_lc_factor.setMaximum(sys.float_info.max)
        self.second_thr_lc_factor.setSingleStep(0.01)
        self.second_thr_lc_factor.setValue(self.shared_data.second_thr_cal_val)
        layout1.addWidget(self.second_thr_lc_factor)
        
        self.confirm_button = QPushButton("Kinnita algparameetrid", self)
        self.confirm_button.clicked.connect(self.confirm_changes)
        layout1.addWidget(self.confirm_button)

        # Connect signals
        self.rho.valueChanged.connect(self.enable_confirm_button)
        self.kin_visc.valueChanged.connect(self.enable_confirm_button)
        self.first_trq_arm_length.valueChanged.connect(self.enable_confirm_button)
        self.first_trq_lc_factor.valueChanged.connect(self.enable_confirm_button)
        self.first_thr_arm_length.valueChanged.connect(self.enable_confirm_button)
        self.first_thr_lc_factor.valueChanged.connect(self.enable_confirm_button)
        self.second_trq_arm_length.valueChanged.connect(self.enable_confirm_button)
        self.second_trq_lc_factor.valueChanged.connect(self.enable_confirm_button)
        self.second_thr_arm_length.valueChanged.connect(self.enable_confirm_button)
        self.second_thr_lc_factor.valueChanged.connect(self.enable_confirm_button)
        self.probe_offset.valueChanged.connect(self.enable_confirm_button)

    def set_tandem_flag(self):
        if self.tandem.isChecked():
            app_globals.window.tandem_setup = True
            self.shared_data.no_of_props = 2
            app_globals.window.second_throttle.setEnabled(True)
            app_globals.window.label7.setEnabled(True)
            app_globals.window.homing.setEnabled(True)
            app_globals.window.file.setEnabled(False)
            self.probe_offset.setEnabled(True)
        else:
            app_globals.window.tandem_setup = False
            self.shared_data.no_of_props = 1
            app_globals.window.second_throttle.setEnabled(False)
            app_globals.window.label7.setEnabled(False)
            self.probe_offset.setEnabled(False)
        
    def enable_confirm_button(self):
        self.confirm_button.setEnabled(True)
        self.confirm_button.setStyleSheet("background-color: orange; color: black;")
        
    def confirm_changes(self):
        try:
            self.shared_data.rho = self.rho.value()
            self.shared_data.kin_visc = self.kin_visc.value()
            self.shared_data.probe_offset = self.probe_offset.value()
            self.shared_data.first_trq_arm_length = self.first_trq_arm_length.value()
            self.shared_data.first_thr_arm_length = self.first_thr_arm_length.value()
            self.shared_data.second_trq_arm_length = self.second_trq_arm_length.value()
            self.shared_data.second_thr_arm_length = self.second_thr_arm_length.value()
            init_data = 'init|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f|%.2f|%.0f' % (self.shared_data.first_trq_arm_length,
                                                                          self.first_trq_lc_factor.value(), self.shared_data.first_thr_arm_length,
                                                                          self.first_thr_lc_factor.value(), self.shared_data.second_trq_arm_length,
                                                                          self.second_trq_lc_factor.value(), self.shared_data.second_thr_arm_length,
                                                                          self.second_thr_lc_factor.value(), self.shared_data.no_of_props)
            self.sendData.emit(init_data)
            self.confirm_button.setEnabled(False)
            self.confirm_button.setStyleSheet("background-color: None; color: None;")
            app_globals.window.params.setStyleSheet("background-color: orange; color: black;")
            app_globals.window.params.setText("Oota...")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            
    @pyqtSlot()
    def on_init_ready(self):
        self.close()
