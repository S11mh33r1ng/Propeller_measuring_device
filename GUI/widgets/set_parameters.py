import sys
from PyQt5.QtCore import pyqtSignal, QTimer
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
        self.first_trq_lc_factor.setValue(880.65)
        layout1.addWidget(self.first_trq_lc_factor)
        
        self.label5 = QLabel("1. posti tõmbeanduri kordaja (pöörlemistelg/survepunkt)")
        layout1.addWidget(self.label5)
        
        self.first_thr_arm_length = QDoubleSpinBox()
        self.first_thr_arm_length.setMinimum(-sys.float_info.max)
        self.first_thr_arm_length.setMaximum(sys.float_info.max)
        self.first_thr_arm_length.setSingleStep(0.01)
        self.first_thr_arm_length.setValue(4.10) #-872.54; -291.75
        layout1.addWidget(self.first_thr_arm_length)
        
        self.label6 = QLabel("1. posti tõmbe koormusanduri kal.faktor")
        layout1.addWidget(self.label6)
        
        self.first_thr_lc_factor = QDoubleSpinBox()
        self.first_thr_lc_factor.setMinimum(-sys.float_info.max)
        self.first_thr_lc_factor.setMaximum(sys.float_info.max)
        self.first_thr_lc_factor.setSingleStep(0.01)
        self.first_thr_lc_factor.setValue(144.79) #878.00
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
        self.second_trq_lc_factor.setValue(880.65)
        layout1.addWidget(self.second_trq_lc_factor)
        
        self.label9 = QLabel("2. posti tõmbeanduri kordaja (pöörlemistelg/survepunkt)")
        layout1.addWidget(self.label9)
        
        self.second_thr_arm_length = QDoubleSpinBox()
        self.second_thr_arm_length.setMinimum(-sys.float_info.max)
        self.second_thr_arm_length.setMaximum(sys.float_info.max)
        self.second_thr_arm_length.setSingleStep(0.01)
        self.second_thr_arm_length.setValue(4.10) #-872.54; -291.75
        layout1.addWidget(self.second_thr_arm_length)
        
        self.label10 = QLabel("2. posti tõmbe koormusanduri kal.faktor")
        layout1.addWidget(self.label10)
        
        self.second_thr_lc_factor = QDoubleSpinBox()
        self.second_thr_lc_factor.setMinimum(-sys.float_info.max)
        self.second_thr_lc_factor.setMaximum(sys.float_info.max)
        self.second_thr_lc_factor.setSingleStep(0.01)
        self.second_thr_lc_factor.setValue(144.79) #878.00
        layout1.addWidget(self.second_thr_lc_factor)
        
        self.confirm_button = QPushButton("Kinnita algparameetrid", self)
        self.confirm_button.clicked.connect(self.confirm_changes)
        layout1.addWidget(self.confirm_button)
        
#         self.label36 = QLabel("X-teljes mootori tsentri koordinaat (max käik) (mm)")
#         layout1.addWidget(self.label36)
#         
#         self.x_center = QDoubleSpinBox()
#         self.x_center.setMinimum(1)
#         self.x_center.setMaximum(330)
#         self.x_center.setSingleStep(0.1)
#         self.x_center.setValue(self.shared_data.x_center)
#         layout1.addWidget(self.x_center)
#         
#         self.label37 = QLabel("Y-telje maksimaalne käik (mm)" )
#         layout1.addWidget(self.label37)
#         
#         self.y_max = QSpinBox()
#         self.y_max.setMinimum(1)
#         self.y_max.setMaximum(107)
#         self.y_max.setSingleStep(1)
#         self.y_max.setValue(self.shared_data.y_max)
#         layout1.addWidget(self.y_max)
#         
#         self.label38 = QLabel("X-telje maksimaalne kiirus (sammu/s)" )
#         layout1.addWidget(self.label38)
#         
#         self.x_max_speed = QSpinBox()
#         self.x_max_speed.setMinimum(100)
#         self.x_max_speed.setMaximum(2000)
#         self.x_max_speed.setSingleStep(100)
#         self.x_max_speed.setValue(self.shared_data.x_max_speed)
#         layout1.addWidget(self.x_max_speed)
#         
#         self.label39 = QLabel("Y-telje maksimaalne kiirus (sammu/s)" )
#         layout1.addWidget(self.label39)
#         
#         self.y_max_speed = QSpinBox()
#         self.y_max_speed.setMinimum(100)
#         self.y_max_speed.setMaximum(2000)
#         self.y_max_speed.setSingleStep(100)
#         self.y_max_speed.setValue(self.shared_data.y_max_speed)
#         layout1.addWidget(self.y_max_speed)
#         
#         self.label40 = QLabel("X-telje maksimaalne kiirendus (sammu/s/s)" )
#         layout1.addWidget(self.label40)
#         
#         self.x_max_accel = QSpinBox()
#         self.x_max_accel.setMinimum(50)
#         self.x_max_accel.setMaximum(1500)
#         self.x_max_accel.setSingleStep(100)
#         self.x_max_accel.setValue(self.shared_data.x_max_accel)
#         layout1.addWidget(self.x_max_accel)
#         
#         self.label41 = QLabel("Y-telje maksimaalne kiirendus (sammu/s/s)" )
#         layout1.addWidget(self.label41)
#         
#         self.y_max_accel = QSpinBox()
#         self.y_max_accel.setMinimum(50)
#         self.y_max_accel.setMaximum(1500)
#         self.y_max_accel.setSingleStep(100)
#         self.y_max_accel.setValue(self.shared_data.y_max_accel)
#         layout1.addWidget(self.y_max_accel)
#         
#         self.confirm_axis_button = QPushButton("Kinnita telgede parameetrid", self)
#         self.confirm_axis_button.clicked.connect(self.confirm_axis_changes)
#         self.confirm_axis_button.setEnabled(False)  # Initially disabled
#         layout1.addWidget(self.confirm_axis_button)

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
        
#         self.x_center.valueChanged.connect(self.enable_confirm_axis_button)
#         self.y_max.valueChanged.connect(self.enable_confirm_axis_button)
#         self.x_max_speed.valueChanged.connect(self.enable_confirm_axis_button)
#         self.y_max_speed.valueChanged.connect(self.enable_confirm_axis_button)
#         self.x_max_accel.valueChanged.connect(self.enable_confirm_axis_button)
#         self.y_max_accel.valueChanged.connect(self.enable_confirm_axis_button)
        
#     def enable_confirm_axis_button(self):
#         self.confirm_axis_button.setEnabled(True)
#         self.confirm_axis_button.setStyleSheet("background-color: orange; color: black;")

    def set_tandem_flag(self):
        if self.tandem.isChecked():
            self.tandem_setup = True
            self.shared_data.no_of_props = 2
            #self.second_throttle.setEnabled(True)
            #self.label7.setEnabled(True)
        else:
            self.tandem_setup = False
            self.shared_data.no_of_props = 1
            #self.second_throttle.setEnabled(False)
            #self.label7.setEnabled(False)
        
    def enable_confirm_button(self):
        self.confirm_button.setEnabled(True)
        self.confirm_button.setStyleSheet("background-color: orange; color: black;")
        
    def confirm_changes(self):
        try:
            self.shared_data.rho = self.rho.value()
            self.shared_data.kin_visc = self.kin_visc.value()
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
            app_globals.window.xy_axes.setEnabled(True)
            QTimer.singleShot(1000, self.close)
            app_globals.window.params.setStyleSheet("background-color: green; color: white;")
            
            #self.confirm_axis_button.setEnabled(True)
#             app_globals.window.aoa_aoss_action.setEnabled(True)
#             app_globals.window.calibrate_loadcells_action.setEnabled(True)
#             app_globals.window.rpm_controller_action.setEnabled(True)
#             app_globals.window.map_action.setEnabled(True)
#             app_globals.window.clear_plot_action.setEnabled(True)
#             app_globals.window.save_plot_action.setEnabled(True)
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
    
#     def confirm_axis_changes(self):
#         try:
#             self.shared_data.x_center = self.x_center.value()
#             self.shared_data.y_max = self.y_max.value()
#             self.shared_data.x_max_speed = self.x_max_speed.value()
#             self.shared_data.y_max_speed = self.y_max_speed.value()
#             self.shared_data.x_max_accel = self.x_max_accel.value()
#             self.shared_data.y_max_accel = self.y_max_accel.value()
#             limit_data = 'l|%d|%d|%d|%d|%d|%d' % ((self.shared_data.x_center * self.shared_data.ratio), (self.shared_data.y_max * self.shared_data.ratio), self.shared_data.x_max_speed, self.shared_data.y_max_speed, self.shared_data.x_max_accel, self.shared_data.y_max_accel)
#             self.sendData.emit(limit_data)
#             self.confirm_axis_button.setEnabled(False)
#             self.confirm_axis_button.setStyleSheet("background-color: None; color: None;")
#             app_globals.window.file.setEnabled(True)
#             QTimer.singleShot(1000, self.close)
#             app_globals.window.params.setStyleSheet("background-color: green; color: white;")
#         except serial.SerialException as e:
#             print(f"Error sending data: {e}")
