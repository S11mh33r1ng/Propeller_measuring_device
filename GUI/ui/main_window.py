import time, os, csv, math, statistics, datetime
import serial
from pathlib import Path
from PyQt5.QtCore import Qt, QThread, QTimer
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QLabel, QGridLayout, QToolBar, QAction, QComboBox,
    QPushButton, QDoubleSpinBox, QSpinBox, QProgressBar, QFileDialog, QCheckBox
)

from plot.canvas import Canvas
from data.shared_data import SharedData
from utils.ports import list_serial_ports
from workers.serial_reader import SerialReader
from workers.measuring_worker import MeasuringWorker
from widgets.set_parameters import SetParameters
from widgets.set_xy_axes import SetXYAxes
from widgets.map_trajectory import MapTrajectory
from widgets.aoa_aoss import AoA_AoSS
from widgets.rpm_controller_1 import RPM_controller_1
from widgets.rpm_controller_2 import RPM_controller_2
from widgets.lc_calibration_1 import LC_calibration_1
from widgets.lc_calibration_2 import LC_calibration_2
from tools.calc_center_of_thrust import Calculate_center_of_thrust

from config import (
    max_number_of_samples_default
)
import app_globals

class MainWindow(QMainWindow):
    def __init__(self, parent=None, **kwargs):
        super().__init__(parent, **kwargs)
        self.shared_data = SharedData()
        self.setupUI()
        self.controller = None
        self.serialReader = None
        self.measuringWorker = None
        self.serialReaderThread = QThread()
        self.measuringThread = QThread()
        #self.xy_axes_conf = Axes_configuration(self.shared_data)
        self.lc_calibration_1 = LC_calibration_1(self.shared_data)
        self.lc_calibration_2 = LC_calibration_2(self.shared_data)
        self.rpm_setup_1 = RPM_controller_1(self.shared_data)
        self.rpm_setup_2 = RPM_controller_2(self.shared_data)
        self.calculate_CT = Calculate_center_of_thrust()
        self.today_dt = None
        self.path = None
        self.csvfile = None

    def setupUI(self):
        self.measuring_stopped = True
        self.e_stop = False
        self.first_rpm = 0
        self.second_rpm = 0
        self.cnv = Canvas()
        self.counter = 0
        self.custom_trajectory = False
        self.first_rpm_label = QLabel()
        self.second_rpm_label = QLabel()
        self.first_thr_label = QLabel()
        self.second_thr_label = QLabel()
        self.first_trq_label = QLabel()
        self.second_trq_label = QLabel()
        self.first_thr_weight_label = QLabel()
        self.second_thr_weight_label = QLabel()
        self.first_trq_weight_label = QLabel()
        self.second_trq_weight_label = QLabel()
        self.motor_test = False
        self.homing_done = False
        self.tare_done = False
        self.fileSelected = False
        self.cal_value = 0
        self.first_thrust_test_value = 0.0
        self.first_thr_weight_test_value = 0.0
        self.first_torque_test_value = 0.0
        self.first_trq_weight_test_value = 0.0
        self.x_normalized_mm = 0
        self.x_current_steps = 0
        self.y_current_steps = 0
        self.first_thr_current = 0
        self.second_thr_current = 0
        self.first_trq_current = 0
        self.second_trq_current = 0
        self.first_rpm_current = 0
        self.second_rpm_current = 0
        self.airspeed = 0
        self.aoa_sensor = 0
        self.aoa_abs = 0
        self.aoss_sensor = 0
        self.aoss_abs = 0
        self.omega = 0
        self.first_omega = 0
        self.second_omega = 0
        self.meas_data_running = False
        self.in_position = False
        self.jog_done = False
        self.centering_done = False
        self.radius_mm = None
        self.x_goal = None
        self.tandem_setup = False

        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.showMaximized()
        layout = QGridLayout(self.centralWidget)
        self.toolbar = QToolBar("Lisavalikud")
        self.addToolBar(self.toolbar)
        
        #self.axes_conf_action = QAction("Telgede konfigureerimine", self)
        #self.axes_conf_action.triggered.connect(self.axes_conf_params)
        #self.toolbar.addAction(self.axes_conf_action)

        self.aoa_aoss_action = QAction("AoA ja AoSS trimmimine", self)
        self.aoa_aoss_action.triggered.connect(self.aoa_aoss_params)
        self.toolbar.addAction(self.aoa_aoss_action)

        self.calibrate_first_loadcells_action = QAction("1.posti koormusandurite kalibreerimine", self)
        self.calibrate_first_loadcells_action.triggered.connect(self.calibrate_first_loadcells)
        self.toolbar.addAction(self.calibrate_first_loadcells_action)

        self.calibrate_second_loadcells_action = QAction("2.posti koormusandurite kalibreerimine", self)
        self.calibrate_second_loadcells_action.triggered.connect(self.calibrate_second_loadcells)
        self.toolbar.addAction(self.calibrate_second_loadcells_action)

        self.rpm_first_controller_action = QAction("1. posti RPM kontrolleri valikud", self)
        self.rpm_first_controller_action.triggered.connect(self.setup_first_rpm)
        self.toolbar.addAction(self.rpm_first_controller_action)
        
        self.rpm_second_controller_action = QAction("2. posti RPM kontrolleri valikud", self)
        self.rpm_second_controller_action.triggered.connect(self.setup_second_rpm)
        self.toolbar.addAction(self.rpm_second_controller_action)

        self.map_action = QAction("Trajektoori valikud", self)
        self.map_action.triggered.connect(self.map_)
        self.toolbar.addAction(self.map_action)

        self.clear_plot_action = QAction("Tühjenda graafik", self)
        self.clear_plot_action.triggered.connect(self.cnv.clear_plots)

        self.save_plot_action = QAction("Salvesta graafik", self)
        self.save_plot_action.triggered.connect(self.save_plot)

        self.center_of_thrust_action = QAction("Arvuta tõmbekese", self)
        self.center_of_thrust_action.triggered.connect(self.calculate_CT_show)
        self.toolbar.addAction(self.center_of_thrust_action)

        # Disable initially (as in your original)
        self.aoa_aoss_action.setEnabled(False)
        self.calibrate_first_loadcells_action.setEnabled(False)
        self.calibrate_second_loadcells_action.setEnabled(False)
        self.rpm_first_controller_action.setEnabled(False)
        self.rpm_second_controller_action.setEnabled(False)
        self.map_action.setEnabled(False)
        self.clear_plot_action.setEnabled(False)
        self.save_plot_action.setEnabled(False)

        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setRowStretch(0, 0)
        layout.setRowStretch(1, 0)
        layout.setRowStretch(2, 0)
        layout.setRowStretch(3, 1)

        self.setWindowTitle("Stend")
        layout.addWidget(self.cnv, 0, 1, -1, 1)

        self.label20 = QLabel("Vali ühendusport:", self)
        layout.addWidget(self.label20, 0, 0, 1, 1)
        
        self.comboBox = QComboBox(self)
        self.update_ports()
        layout.addWidget(self.comboBox, 1, 0, 1, 1)
        
        self.connect = QPushButton("Ühenda", self)
        self.connect.clicked.connect(self.on_connect_clicked)
        layout.addWidget(self.connect, 2, 0, 1, 1)
        
        self.params = QPushButton("Säti andurid", self)
        self.params.setStyleSheet("background-color: None; color: None;")
        self.params.setEnabled(False)
        self.params.clicked.connect(self.set_params)
        layout.addWidget(self.params, 3, 0, 1, 1)
        
        self.xy_axes = QPushButton("Säti teljed", self)
        self.xy_axes.setStyleSheet("background-color: None; color: None;")
        self.xy_axes.setEnabled(False)
        self.xy_axes.clicked.connect(self.set_xy_axes)
        layout.addWidget(self.xy_axes, 4, 0, 1, 1)
        
        self.file = QPushButton("Vali propelleri konfiguratsiooni fail")
        self.file.clicked.connect(self.showDialog)
        self.file.setStyleSheet("background-color: None; color: None;")
        self.file.setEnabled(False)
        layout.addWidget(self.file, 5, 0, 1, 1)
        
        self.homing = QPushButton("Telgede referents")
        self.homing.clicked.connect(self.home)
        self.homing.setStyleSheet("background-color: None; color: None;")
        self.homing.setEnabled(False)
        layout.addWidget(self.homing, 6, 0, 1, 1)
        
        #self.homed = QPushButton("Override")
        #self.homed.clicked.connect(self.homingDone)
        #self.homed.setStyleSheet("background-color: None; color: None;")
        #self.homed.setEnabled(False)
        #homingButtonsLayout.addWidget(self.homed)
        
        #homingButtonsWidget = QWidget()
        #homingButtonsWidget.setLayout(homingButtonsLayout)
        
        #layout.addWidget(homingButtonsWidget, 5, 0, 1, 1)
        
        self.label1 = QLabel("Sisesta propelleri läbimõõt tollides")
        layout.addWidget(self.label1, 7, 0, 1, 1)
        
        self.prop = QDoubleSpinBox()
        self.prop.setMinimum(5.0)
        self.prop.setMaximum(23.0)
        self.prop.setSingleStep(0.1)
        self.prop.setValue(20.2)
        layout.addWidget(self.prop, 8, 0, 1, 1)
        
        self.label80 = QLabel("Sisesta mõõtmiskauguse/raadiuse suhe")
        layout.addWidget(self.label80, 9, 0, 1, 1)
        
        self.dr_ratio = QDoubleSpinBox()
        self.dr_ratio.setMinimum(0.0)
        self.dr_ratio.setMaximum(2.0)
        self.dr_ratio.setSingleStep(0.4)
        self.dr_ratio.setValue(0.0)
        layout.addWidget(self.dr_ratio, 10, 0, 1, 1)
        
        self.centering = QPushButton("Pitot' tsentrisse")
        self.centering.setEnabled(False)
        self.centering.clicked.connect(self.center)
        self.centering.setStyleSheet("background-color: None; color: None;")
        layout.addWidget(self.centering, 11, 0, 1, 1)
        
        #self.label3 = QLabel("Pitot' mootorite kiirus")
        #layout.addWidget(self.label3, 9, 0, 1, 1)
        
        self.jog_speed = QSpinBox()
        self.jog_speed.setMinimum(500)
        self.jog_speed.setMaximum(2000)
        self.jog_speed.setSingleStep(100)
        self.jog_speed.setValue(self.shared_data.x_max_speed)
        #layout.addWidget(self.jog_speed, 10, 0, 1, 1)
        
        self.label2 = QLabel("Y-telje positsioon mm" )
        layout.addWidget(self.label2, 12, 0, 1, 1)
        
        self.Y_pos = QSpinBox()
        self.Y_pos.setMinimum(0)
        self.Y_pos.setMaximum(105)
        layout.addWidget(self.Y_pos, 13, 0, 1, 1)
        
        self.Y_pos.valueChanged.connect(self.enable_Y_move_button)
        
        self.Y_move = QPushButton("Liiguta Y-telge")
        self.Y_move.clicked.connect(self.moveY)
        self.Y_move.setEnabled(False)
        layout.addWidget(self.Y_move, 14, 0, 1, 1)
        
        self.label4 = QLabel("1. posti propelleri kiirus (%)")
        layout.addWidget(self.label4, 15, 0, 1, 1)
        
        self.first_throttle = QDoubleSpinBox()
        self.first_throttle.setMinimum(10.0)
        self.first_throttle.setMaximum(100.0)
        self.first_throttle.setValue(10.0)
        self.first_throttle.setSingleStep(0.1)
        layout.addWidget(self.first_throttle, 16, 0, 1, 1)
        
        self.label7 = QLabel("2. posti propelleri kiirus (%)")
        self.label7.setEnabled(False)
        layout.addWidget(self.label7, 17, 0, 1, 1)
        
        self.second_throttle = QDoubleSpinBox()
        self.second_throttle.setEnabled(False)
        self.second_throttle.setMinimum(10.0)
        self.second_throttle.setMaximum(100.0)
        self.second_throttle.setValue(10.0)
        self.second_throttle.setSingleStep(0.1)
        layout.addWidget(self.second_throttle, 18, 0, 1, 1)
        
        self.testMotorButton = QPushButton("Testi mootorit", self)
        self.testMotorButton.setEnabled(False)
        self.testMotorButton.setCheckable(True)
        self.testMotorButton.clicked.connect(self.toggle_motor)
        layout.addWidget(self.testMotorButton, 19, 0, 1, 1)
        
        self.label5 = QLabel("Mõõdistamise kiirus")
        layout.addWidget(self.label5, 20, 0, 1, 1)
        
        self.measure_speed = QSpinBox()
        self.measure_speed.setMinimum(200)
        self.measure_speed.setMaximum(500)
        self.measure_speed.setSingleStep(100)
        self.measure_speed.setValue(200)
        layout.addWidget(self.measure_speed, 21, 0, 1, 1)
        
        self.label11 = QLabel("Kordusmõõtmiste arv")
        layout.addWidget(self.label11, 22, 0, 1, 1)
        
        self.sweep_count = QSpinBox()
        self.sweep_count.setMinimum(1)
        self.sweep_count.setMaximum(max_number_of_samples_default)
        self.sweep_count.setSingleStep(1)
        self.sweep_count.setValue(1)
        layout.addWidget(self.sweep_count, 23, 0, 1, 1)
        
        self.measure = QPushButton("Alusta mõõtmisega")
        self.measure.setEnabled(False)
        self.measure.clicked.connect(self.start_measuring)
        layout.addWidget(self.measure, 24, 0, 1, 1)
        
        self.label6 = QLabel("Mõõdistamise kulg")
        layout.addWidget(self.label6, 25, 0, 1, 1)
        
        self.progress = QProgressBar()
        self.progress.setMinimum(0)
        self.progress.setMaximum(100)
        self.progress.setValue(0)
        layout.addWidget(self.progress, 26, 0, 1, 1)
        
        self.test_progress = QProgressBar()
        self.test_progress.setMinimum(0)
        self.test_progress.setMaximum(self.sweep_count.value())
        self.test_progress.setValue(0)
        layout.addWidget(self.test_progress, 27, 0, 1, 1)
        
        self.back = QPushButton("Pitot' tagasi algasendisse")
        self.back.setEnabled(False)
        self.back.clicked.connect(self.come_back)
        layout.addWidget(self.back, 28, 0, 1, 1)
        
        self.danger = QLabel("Emergency!")
        self.danger.setAlignment(Qt.AlignCenter)
        self.danger.setStyleSheet("background-color: None")
        layout.addWidget(self.danger, 29, 0, 1, 1)
        
        self.label14 = QLabel("Induced power (W):")
        layout.addWidget(self.label14, 0, 3)
        
        self.label15 = QLabel()
        self.label15.setText(" ")
        self.label15.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.label15, 1, 3)
        
        self.label16 = QLabel("Total power (W):")
        layout.addWidget(self.label16, 2, 3)
        
        self.label17 = QLabel()
        self.label17.setText(" ")
        self.label17.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.label17, 3, 3)
        
        self.label18 = QLabel("Propeller efficiency (%):")
        layout.addWidget(self.label18, 4, 3)
        
        self.label19 = QLabel()
        self.label19.setText(" ")
        self.label19.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.label19, 5, 3)
        
        self.label12 = QLabel("Av induced speed (m/s):")
        layout.addWidget(self.label12, 6, 3)
        
        self.label13 = QLabel()
        self.label13.setText(" ")
        self.label13.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.label13, 7, 3)
        
        self.label64 = QLabel("Airspeed ratio:")
        layout.addWidget(self.label64, 8, 3)
        
        self.label65 = QLabel()
        self.label65.setText(" ")
        self.label65.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.label65, 9, 3)
        
        self.label66 = QLabel("Airmass speed (kg/s):")
        layout.addWidget(self.label66, 10, 3)
        
        self.label67 = QLabel()
        self.label67.setText(" ")
        self.label67.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.label67, 11, 3)
        
        self.label68 = QLabel("Av max speed (m/s):")
        layout.addWidget(self.label68, 12, 3)
        
        self.label69 = QLabel()
        self.label69.setText(" ")
        self.label69.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.label69, 13, 3)
        
        self.labelRPM = QLabel("1. posti mõõdetud pöörded (RPM)", self)
        layout.addWidget(self.labelRPM, 14, 3)
        
        self.first_rpm_label.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.first_rpm_label, 15, 3)
        
        self.labelThr  = QLabel("1. posti jõuõlaga arvutatud tõmme (N)", self)
        layout.addWidget(self.labelThr, 16, 3)
        
        self.first_thr_label.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.first_thr_label, 17, 3)
        
        self.labelThrWeight  = QLabel("1. posti mõõdetud tõmbe kaal (g)", self)
        layout.addWidget(self.labelThrWeight, 18, 3)
        
        self.first_thr_weight_label.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.first_thr_weight_label, 19, 3)
        
        self.labelTrq  = QLabel("1. posti mõõdetud moment (Nm)", self)
        layout.addWidget(self.labelTrq, 20, 3)
        
        self.first_trq_label.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.first_trq_label, 21, 3)
        
        self.labelTrqWeight  = QLabel("1. posti mõõdetud momendi kaal (g)", self)
        layout.addWidget(self.labelTrqWeight, 22, 3)
        
        self.first_trq_weight_label.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.first_trq_weight_label, 23, 3)
        
        self.labelRPM = QLabel("2. posti mõõdetud pöörded (RPM)", self)
        layout.addWidget(self.labelRPM, 24, 3)
        
        self.second_rpm_label.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.second_rpm_label, 25, 3)
        
        self.labelThr  = QLabel("2. posti jõuõlaga arvutatud tõmme (N)", self)
        layout.addWidget(self.labelThr, 26, 3)
        
        self.second_thr_label.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.second_thr_label, 27, 3)
        
        self.labelThrWeight  = QLabel("2. posti mõõdetud tõmbe kaal (g)", self)
        layout.addWidget(self.labelThrWeight, 28, 3)
        
        self.second_thr_weight_label.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.second_thr_weight_label, 29, 3)
        
        self.labelTrq  = QLabel("2. posti mõõdetud moment (Nm)", self)
        layout.addWidget(self.labelTrq, 30, 3)
        
        self.second_trq_label.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.second_trq_label, 31, 3)
        
        self.labelTrqWeight  = QLabel("2. posti mõõdetud momendi kaal (g)", self)
        layout.addWidget(self.labelTrqWeight, 32, 3)
        
        self.second_trq_weight_label.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.second_trq_weight_label, 33, 3)
        
        self.update_ports()
        self.initTimer()
        self.reset_button()
        self.last_first_throttle_value = 0
        self.last_second_throttle_value = 0

        self.timer_motor = QTimer(self)
        self.timer_motor.timeout.connect(self.read_values)

        self.delay = QTimer(self)
        self.delay.timeout.connect(self.on_delay_timeout)

    def on_delay_timeout(self):
        pass

    def set_delay_timer(self, interval):
        self.timer.stop()  # Stop the current timer (if running)
        self.timer.setInterval(interval)  # Set the new interval
        self.timer.start()  # Start the timer with the new interval
        
    def sendData(self, data):
        if self.controller:
            try:
                encoded_data = data.encode('utf-8')
                self.controller.write(encoded_data)
                print(data)
            except serial.SerialException as e:
                print(f"Error sending data: {e}")
                
    def homingDone(self):
        self.homing_done = True
        self.measure.setEnabled(False)
        self.centering.setEnabled(True)
        self.testMotorButton.setEnabled(True)
        self.back.setEnabled(True)
        self.progress.setValue(0)
        self.test_progress.setValue(0)
        self.homing.setStyleSheet("background-color: green; color: white;")
        #self.homed.setStyleSheet("background-color: green; color: white;")
        self.meas_data_running = False
        
    def handleData(self, data):
        #print(data)
        if (data == 'Emergency!'):
            self.e_stop = True
            self.meas_data_running = False
            self.update_emergency()
        if (data == 'Emergency cleared'):
            self.e_stop = False
            self.meas_data_running = False
            self.update_emergency()
        if self.motor_test == False:
            self.update_first_rpm_label('0')
            #self.update_second_rpm_label('0')
            self.update_first_thr_label('0')
            #self.update_second_thr_label('0')
            self.update_first_trq_label('0')
            #self.update_second_trq_label('0')
            self.update_first_thr_weight_label('0')
            #self.update_second_thr_weight_label('0')
            self.update_first_trq_weight_label('0')
            #self.update_second_trq_weight_label('0')
            self.meas_data_running = False
        if (data == 'homing done'):
            self.homingDone()
        if (data == 'limit switch'):
            self.homing_done = True
            self.measure.setEnabled(False)
            self.centering.setEnabled(False)
            self.testMotorButton.setEnabled(True)
            self.progress.setValue(0)
            self.test_progress.setValue(0)
            self.homing.setStyleSheet("background-color: orange; color: None;")
        if (data == 'tare done'):
            self.tare_done = True
            self.meas_data_running = False
        if (data == 'centering done'):
            self.centering.setStyleSheet("background-color: green; color: white;")
            self.meas_data_running = False
            time.sleep(1)
            self.Y_move.setEnabled(True)
        if (data == 'jog done'):
            self.jog_done = True
        if (data.startswith('CalVal:')):
            self.cal_value = data[7:].strip()
            if (self.cal_value != 0):
                self.lc_calibration.update_cal_factor_label()
        if (data.startswith('LC test:')):
            print(data)
            data = data[8:].strip()
            parts = data.split()
            if len(parts) == 10:
                try:
                    self.first_thrust_test_value = float(int(parts[0])/1000.00)
                    self.first_thr_weight_test_value = float(parts[1])
                    self.first_torque_test_value = float(int(parts[2])/1000.00)
                    self.first_trq_weight_test_value = float(parts[3])
                    self.first_rpm_value = float(parts[4])
                    self.second_thrust_test_value = float(int(parts[5])/1000.00)
                    self.second_thr_weight_test_value = float(parts[6])
                    self.second_torque_test_value = float(int(parts[7])/1000.00)
                    self.second_trq_weight_test_value = float(parts[8])
                    self.second_rpm_value = float(parts[9])
                    if (self.motor_test == True):
                        self.update_first_thr_label(f"{self.first_thrust_test_value}")
                        self.update_first_trq_label(f"{self.first_torque_test_value}")
                        self.update_first_thr_weight_label(f"{self.first_thr_weight_test_value}")
                        self.update_first_trq_weight_label(f"{self.first_trq_weight_test_value}")
                        self.update_first_rpm_label(f"{self.first_rpm_value}")
                        self.update_second_thr_label(f"{self.second_thrust_test_value}")
                        self.update_second_trq_label(f"{self.second_torque_test_value}")
                        self.update_second_thr_weight_label(f"{self.second_thr_weight_test_value}")
                        self.update_second_trq_weight_label(f"{self.second_trq_weight_test_value}")
                        self.update_second_rpm_label(f"{self.second_rpm_value}")
                    if self.motor_test == False:
                        self.update_first_thr_label('0')
                        self.update_first_trq_label('0')
                        self.update_first_thr_weight_label('0')
                        self.update_first_trq_weight_label('0')
                        self.update_first_rpm_label('0')
                        self.update_second_thr_label('0')
                        self.update_second_trq_label('0')
                        self.update_second_thr_weight_label('0')
                        self.update_second_trq_weight_label('0')
                        self.update_second_rpm_label('0')
                        try:
                            self.lc_calibration_1.update_data()
                        except:
                            pass
                except ValueError:
                    print("Error parsing numeric data:", parts)
        if (data.startswith('RPM_test:')):
            data = data[9:].strip()
            self.update_first_rpm_label(f"{data}")
            if self.tandem_setup:
                self.update_second_rpm_label(f"{data}")
            self.meas_data_running = False
        if (data.startswith('Measurements:')):
            data = data[13:].strip()
            self.meas_data_running = True
            #print(data)
            parts = data.split()
            if len(parts) == 10:
                try:
                    self.x_current_steps = float(parts[0])
                    self.y_current_steps = float(parts[1])
                    self.first_thr_current = float(int(parts[2])/1000.00)
                    self.first_trq_current = float(int(parts[3])/1000.00)
                    self.first_rpm_current = float(parts[4])
                    self.airspeed = float(parts[5])
                    self.aoa_sensor = float(parts[6])
                    self.aoa_abs = float(parts[7])
                    self.aoss_sensor = float(parts[8])
                    self.aoss_abs = float(parts[9])
                    self.second_thr_current = float(int(parts[10])/1000.00)
                    self.second_trq_current = float(int(parts[11])/1000.00)
                    self.second_rpm_current = float(parts[12])
                    self.omega = format((((2.0 * math.pi)/60.0) * self.first_rpm_current),'.2f')
                    self.first_omega = self.omega
                    self.second_omega = format((((2.0 * math.pi)/60.0) * self.second_rpm_current),'.2f')
                except ValueError:
                    print("Error parsing data:", parts)
        
    def toggle_motor(self):
        if self.testMotorButton.isChecked():
            self.testMotorButton.setText("Seiska mootor")
            self.sendData('ON')
            time.sleep(2)
            self.sendData('BeaconON')
            time.sleep(2)
            self.test_motor()
        else:
            self.testMotorButton.setText("Testi mootorit")
            self.stop_motor()
            time.sleep(2)
            self.sendData('OFF')
            time.sleep(2)
            self.sendData('BeaconOFF')
            time.sleep(2)
            self.sendData('OFF')

    def test_motor(self):
        self.update_throttle_test()
        self.timer_motor.start(1000)

    def stop_motor(self):
        self.sendData('stop')
        self.timer_motor.stop()
        self.sendData('stop')
        self.motor_test = False
        time.sleep(2)

    def update_throttle_test(self):
        if self.tandem_setup:
            first_throttle = int(1000 + (self.first_throttle.value() * 10))
            second_throttle = int(1000 + (self.second_throttle.value() * 10))
            start_first = f'test1|{first_throttle}'
            self.sendData(start_first)
            start_second = f'test2|{second_throttle}'
            self.motor_test = True
            #print(start_first)
            #print(start_second)
        else:
            first_throttle = int(1000 + (self.first_throttle.value() * 10))
            start_first = f'test1|{first_throttle}'
            self.sendData(start_first)
            self.motor_test = True
            #print(start_first)
        
    def read_values(self):
        if self.tandem_setup:
            current_first_throttle = self.first_throttle.value()
            current_second_throttle = self.second_throttle.value()
            if current_first_throttle != self.last_first_throttle_value:
                self.update_throttle_test()
                self.last_first_throttle_value = current_first_throttle
            if current_second_throttle != self.last_second_throttle_value:
                self.update_throttle_test()
                self.last_second_throttle_value = current_second_throttle
        else:    
            current_first_throttle = self.first_throttle.value()
            if current_first_throttle != self.last_first_throttle_value:
                self.update_throttle_test()
                self.last_first_throttle_value = current_first_throttle

    def initSerialReader(self):
        if self.controller and not self.serialReaderThread.isRunning():
            if self.serialReader:
                self.serialReaderThread.quit()
                self.serialReaderThread.wait()

            self.serialReader = SerialReader(self.controller)
            self.serialReader.moveToThread(self.serialReaderThread)
            #self.serialReader.calValueReceived.connect(self.lc_calibration_1.update_cal_factor_label)
            self.serialReader.serial_readout.connect(self.handleData)
            self.serialReaderThread.started.connect(self.serialReader.run)
            self.serialReaderThread.start()
            
            self.measuringWorker = MeasuringWorker(self.shared_data)
            self.measuringWorker.moveToThread(self.measuringThread)
            self.measuringThread.started.connect(self.measuringWorker.run)
            self.measuringWorker.requestStop.connect(self.measuringWorker.stop_measuring)

    def update_first_rpm_label(self, rpm):
        if self.motor_test == True:
            self.first_rpm_label.setText(rpm)
        else:
            self.first_rpm_label.setText('0')
            
    def update_second_rpm_label(self, rpm):
        if self.motor_test == True:
            self.second_rpm_label.setText(rpm)
        else:
            self.second_rpm_label.setText('0')
            
    def update_first_thr_label(self, thr):
        if self.motor_test == True:
            self.first_thr_label.setText(thr)
        else:
            self.first_thr_label.setText('0')
            
    def update_first_trq_label(self, trq):
        if self.motor_test == True:
            self.first_trq_label.setText(trq)
        else:
            self.first_trq_label.setText('0')
            
    def update_first_thr_weight_label(self, thr_weight):
        if self.motor_test == True:
            self.first_thr_weight_label.setText(thr_weight)
        else:
            self.first_thr_weight_label.setText('0')
            
    def update_first_trq_weight_label(self, trq_weight):
        if self.motor_test == True:
            self.first_trq_weight_label.setText(trq_weight)
        else:
            self.first_trq_weight_label.setText('0')
        
    def save_plot(self):
        fileName, _ = QFileDialog.getSaveFileName(self, "Save Plot", "",
                      "PNG Files (*.png);;JPEG Files (*.jpg);;All Files (*)", options=QFileDialog.Options())
        if fileName:
            self.cnv.save_only_second_plot(fileName)
        
    def update_plot_ax1(self, x, rpms, airspeed, absAoa, absAoss, trq_curr, thr_curr):
        try:
        # Convert input values to float and ensure they are valid numbers
            x = float(x)
            scaled_rpms = float(rpms)/10.00
            airspeed = float(airspeed)
            absAoa = float(absAoa)
            absAoss = float(absAoss)
            trq = float(trq_curr)
            thr = float(thr_curr)

            # Example labels and styles for different plots
            labels_styles = [
                ("Omega x 10 (rad/s)", 'r-'),
                ("Airspeed (m/s)", 'b-'),
                ("AoA (deg)", 'g-'),
                ("AoSS (deg)", 'm-'),
                ("Torque (Nm)", 'c-'),
                ("Thrust (N)", 'k-')
            ]
            
            # Plot the data, adding it to the canvas
            self.cnv.add_data_ax1([x], [scaled_rpms], labels_styles[0][0], labels_styles[0][1])
            self.cnv.add_data_ax1([x], [airspeed], labels_styles[1][0], labels_styles[1][1])
            self.cnv.add_data_ax1([x], [absAoa], labels_styles[2][0], labels_styles[2][1])
            self.cnv.add_data_ax1([x], [absAoss], labels_styles[3][0], labels_styles[3][1])
            self.cnv.add_data_ax1([x], [trq], labels_styles[4][0], labels_styles[4][1])
            self.cnv.add_data_ax1([x], [thr], labels_styles[5][0], labels_styles[5][1])
            
            #print(f"Data added to plot: x={x}, rpms={rpms}, airspeed={airspeed}")
        except Exception as e:
            print(f"Error updating plot: {e}")
            
    def update_plot_ax2(self, x, v_tan, v_rad, v_axial):
        try:
            x = float(x)
            v_tan = float(v_tan)
            v_rad = float(v_rad)
            v_axial = float(v_axial)

            # Example labels and styles for different plots
            labels_styles = [
                ("V_tan (m/s)", 'r-'),
                ("V_rad (m/s)", 'b-'),
                ("V_axial (m/s)", 'g-')
                #("Propeller tip (mm)", 'm-')
            ]
            
            # Plot the data, adding it to the canvas
            self.cnv.add_data_ax2([x], [v_tan], labels_styles[0][0], labels_styles[0][1])
            self.cnv.add_data_ax2([x], [v_rad], labels_styles[1][0], labels_styles[1][1])
            self.cnv.add_data_ax2([x], [v_axial], labels_styles[2][0], labels_styles[2][1])
            #self.cnv.add_data_ax2([radius], [0], labels_styles[3][0], labels_styles[3][1])
            
            #print(f"Data added to plot: x={x}, rpms={rpms}, airspeed={airspeed}")
        except Exception as e:
            print(f"Error updating plot: {e}")
        
    def initTimer(self):
        self.timer_ports = QTimer()
        self.timer_ports.setInterval(500)
        self.timer_ports.timeout.connect(self.update_ports)
        self.timer_ports.start()
        
    def update_ports(self):
        current_ports = list_serial_ports()
        current_selection = self.comboBox.currentText()
        self.comboBox.clear()
        if current_ports:
            self.comboBox.addItems(current_ports)
            if current_selection in current_ports:
                self.comboBox.setCurrentText(current_selection)
        else:
            self.comboBox.addItem("Ei leia ACM/USB porte")
            self.reset_button()
            
    def on_connect_clicked(self):
        selected_port = self.comboBox.currentText()
        if selected_port:
            try:
                self.controller = serial.Serial(selected_port, 115200, timeout = 1)
                self.initSerialReader()
                self.lc_calibration_1.initialize(self.shared_data)
                self.lc_calibration_2.initialize(self.shared_data)
                self.connect.setStyleSheet("background-color: green; color: white;")
                self.connect.setText("Ühendatud")
                self.serialReaderThread.start()
                self.params.setEnabled(True)
                
            except serial.SerialException as e:
                print(f"Could not open serial port {selected_port}: {e}")
                self.connect.setStyleSheet("background-color: red; color: white;")
                self.connect.setText("Ühenda")
                self.testMotorButton.setEnabled(False)
                
    def reset_button(self):
        try:
            self.connect.setStyleSheet("background-color: None; color: None;")
            self.connect.setText("Ühenda")
            self.testMotorButton.setEnabled(False)
            self.file.setStyleSheet("background-color: None; color: None;")
            self.params.setStyleSheet("background-color: None; color: None;")
            self.homing.setStyleSheet("background-color: None; color: None;")
            self.centering.setStyleSheet("background-color: None; color: None;")
        except:
            pass
        
    def showDialog(self):
        home_dir = str(Path.home())
        self.file.setStyleSheet("background-color: None; color: None;")
        self.fname = QFileDialog.getOpenFileName(self, 'Otsi propelleri konfiguratsiooni fail', home_dir, "csv(*.csv)")
        if self.fname:
            self.fileSelected = True
            self.homing.setEnabled(True)
            self.file.setStyleSheet("background-color: green; color: white;")
        
    def update_emergency(self):
        if self.e_stop == True:
            self.sendData('stop')
            self.motor_test = False
            self.testMotorButton.setChecked(False)
            self.testMotorButton.setEnabled(False)
            self.measuring_stopped = True
            self.danger.setStyleSheet("background-color: red; color: None;")
            self.measure.setEnabled(False)
            self.centering.setEnabled(False)
            self.back.setEnabled(False)
            self.homing.setStyleSheet("background-color: None; color: None;")
        else:
            self.e_stop = False
            self.danger.setStyleSheet("background-color: None; color: None;")
            
    def set_params(self):
        if not hasattr(self, 'Parameetrid'):
            self.param_window = SetParameters(self.shared_data)
            self.param_window.sendData.connect(self.sendData)
        self.param_window.show()
            
    def map_(self):
        if not hasattr(self, 'Trajektoor'):
            self.map_window = MapTrajectory(self.shared_data)
            self.map_window.sendData.connect(self.sendData)
        self.map_window.show()
                
    def calibrate_first_loadcells(self):
        if not hasattr(self, '1. posti andurite kalibreerimine'):
            self.lc_calib_1_window = LC_calibration_1(self.shared_data)
            self.lc_calib_1_window.sendData.connect(self.sendData)
        self.lc_calib_1_window.show()
        
    def calibrate_second_loadcells(self):
        if not hasattr(self, '2. posti andurite kalibreerimine'):
            self.lc_calib_2_window = LC_calibration_2(self.shared_data)
            self.lc_calib_2_window.sendData.connect(self.sendData)
        self.lc_calib_2_window.show()
    
    def set_xy_axes(self):
        if not hasattr(self, 'XY telgede konfigureerimine'):
            self.axes_conf_window = SetXYAxes(self.shared_data)
            self.axes_conf_window.sendData.connect(self.sendData)
        self.axes_conf_window.show()
    
    def aoa_aoss_params(self):
        if not hasattr(self, 'AoA ja AoSS teljed'):
            self.aoa_aoss_window = AoA_AoSS(self.shared_data)
            self.aoa_aoss_window.sendData.connect(self.sendData)
        self.aoa_aoss_window.show()
        
    def setup_first_rpm(self):
        if not hasattr(self, '1. posti RPM'):
            self.first_rpm_setup_window = RPM_controller_1(self.shared_data)
            self.first_rpm_setup_window.sendData.connect(self.sendData)
        self.first_rpm_setup_window.show()
        
    def setup_second_rpm(self):
        if not hasattr(self, '2. posti RPM'):
            self.second_rpm_setup_window = RPM_controller_2(self.shared_data)
            self.second_rpm_setup_window.sendData.connect(self.sendData)
        self.second_rpm_setup_window.show()
        
    def correct_log(self):
        pass
    
    def calculate_CT(self):
        if not hasattr(self, 'Tümbekeskme arvutus'):
            self.CT_setup_window = Calculate_center_of_thrust()
        self.CT_setup_window.show()
        
    def home(self):
        self.homing_done = False
        self.homing.setStyleSheet("background-color: None; color:None;")
        #self.homed.setEnabled(True)
        self.sendData('home')
        
    def center(self):
        self.back.setEnabled(True)
        self.Y_move.setEnabled(False)
        self.sendData('center')
        
    def come_back(self):
        self.meas_data_running = False
        if self.measuringThread.isRunning():
            self.measuringThread.quit()
            self.measuringThread.wait()
        self.sendData('stop')
        self.centering.setStyleSheet("background-color: None; color: None;")
        time.sleep(2)
        self.measure.setEnabled(False)
        self.testMotorButton.setEnabled(True)
        self.progress.setValue(0)
        self.test_progress.setValue(0)
        self.counter = 0
        jog_home = 'j|%d|%d|%d|%d' % (0, 0, self.jog_speed.value(), self.jog_speed.value())
        self.sendData(jog_home)
        time.sleep(2)
        self.sendData('BeaconOFF')
        
    def enable_Y_move_button(self):
        self.Y_move.setStyleSheet("background-color: orange; color: black;")
        
    def moveY(self):
        moveY = 'j|%d|%d|%d|%d' %((self.shared_data.x_center * self.shared_data.ratio), (self.Y_pos.value() * self.shared_data.ratio), self.jog_speed.value(), self.jog_speed.value())
        self.sendData(moveY)
        self.Y_move.setStyleSheet("background-color: None; color: None;")
        self.measure.setEnabled(True)
        
    def tare(self):
        self.tare_done = False
        self.sendData('tare')
        
    def mm_to_steps(self, coordinates_mm):
        self.coordinates_steps = int(int(coordinates_mm) * float(self.shared_data.ratio))
        #print(self.coordinates_steps)
        return self.coordinates_steps
    
    def steps_to_mm(self, coordinates_steps):
        self.coordinates_mm = int(int(coordinates_steps) / float(self.shared_data.ratio))
        #print(self.coordinates_mm)
        return self.coordinates_mm
        
    def start_measuring(self):
        self.measuring_stopped = False
        self.testMotorButton.setEnabled(False)
        if not self.measuringThread.isRunning():
            self.measuringThread.start()
            
    def process_data(self):
        plot_filename = f"log{self.today_dt}.png"
        mean_csvfile = "log" + self.today_dt + "_mean.csv"
        mean_header = ['#_of_samples ' 'Prop_diam(inch) ' 'X_position(mm) ' 'Y_position(mm) ' 'Torque(Nm) ' 'Thrust(N) ' 
                       'Airspeed(m/s) ' 'AoA(deg) ' 'AoSS(deg) ' 'V_tan(m/s) ' 'V_rad(m/s) ' 'V_axial(m/s) ' 'Chord_angle(deg) ' 'Chord_angle_eff(deg) '
                       'Chord_length(mm) ' 'Chord_length_eff(mm) ' 'Helix_angle_eff(deg) ' 'Alpha_angle(deg) ' 'V_total(m/s) ' 'V_lift(m/s) ' 'V_drag(m/s) '
                       'CL ' 'CD ' 'Reynolds_number ' 'V_a+r(m/s) ' 'D/R_ratio ']       
        
        omega_values = []
        try:
            with open(os.path.join(self.path, self.csvfile), newline='') as csvfile:
                row_read = csv.reader(csvfile, delimiter=' ')
                for row in row_read:
                    line = ' '.join(row)
                    sample = list(line.split(" "))
                    try:
                        omega_value = float(sample[5])
                        omega_values.append(omega_value)
                    except (IndexError, ValueError):
                        continue
        except FileNotFoundError as e:
            print(f"Error reading log file: {e}")
            return

        if omega_values:
            try:
                omega_mode = statistics.mode(omega_values)
            except statistics.StatisticsError as e:
                print(f"Could not compute mode: {e}. Using mean instead.")
                omega_mode = statistics.mean(omega_values)
        else:
            print("No omega values found in the log file.")
            omega_mode = 0

        
        with open(os.path.join(self.path,mean_csvfile), 'a') as h:
            k = csv.writer(h)
            k.writerow(mean_header)
        x_max = 3 * math.floor((float(self.radius_mm) + (1 - (self.shared_data.safety_over_prop/100)))/3)
        x_mp = 0
        while x_mp <= x_max:
            try:
                with open(os.path.join(self.path,self.csvfile), newline='') as csvfile:
                    row_read = csv.reader(csvfile, delimiter=' ')
                    j = 0
                    mean_counter = 0
                    testnumber = 0
                    dict_prop = {}
                    dict_x = {}
                    dict_y = {}
                    dict_trq = {}
                    dict_thr = {}
                    dict_omega = {}
                    dict_arspd = {}
                    dict_aoa = {}
                    dict_aoss = {}
                    dict_v_tan = {}
                    dict_v_rad = {}
                    dict_v_axial = {}
                    for row in row_read:
                        line = ' '.join(row)
                        j = j + 1
                        sample = list(line.split(" "))
                        if sample[1] == str(x_mp):
                            dict_prop[mean_counter] = float(sample[0])
                            dict_x[mean_counter] = int(sample[1])
                            dict_y[mean_counter] = int(sample[2])
                            dict_trq[mean_counter] = float(sample[3])
                            dict_thr[mean_counter] = float(sample[4])
                            dict_omega[mean_counter] = float(sample[5])
                            dict_arspd[mean_counter] = float(sample[6])
                            dict_aoa[mean_counter] = float(sample[7])
                            dict_aoss[mean_counter] = float(sample[8])
                            dict_v_tan[mean_counter] = float(sample[9])
                            dict_v_rad[mean_counter] = float(sample[10])
                            dict_v_axial[mean_counter] = float(sample[11])
                            mean_counter = mean_counter + 1
                            
                    testnumber = int(len(list(dict_x.values())))
                    prop = statistics.mean(list(dict_prop.values()))
                    x_pos = statistics.mean(list(dict_x.values()))
                    y_pos = statistics.mean(list(dict_y.values()))
                    trq_mean = format(statistics.mean(list(dict_trq.values())),'.2f')
                    thr_mean = format(statistics.mean(list(dict_thr.values())),'.2f')
                    arspd_mean = format(statistics.mean(list(dict_arspd.values())),'.2f')
                    aoa_mean = format(statistics.mean(list(dict_aoa.values())),'.2f')
                    aoss_mean = format(statistics.mean(list(dict_aoss.values())),'.2f')
                    v_tan_mean = format(statistics.mean(list(dict_v_tan.values())),'.2f')
                    v_rad_mean = format(statistics.mean(list(dict_v_rad.values())),'.2f')
                    v_axial_mean = format(statistics.mean(list(dict_v_axial.values())),'.2f')
                    
                    mean_list = str(str(testnumber)+" "+str(prop)+" "+str(x_pos)+" "+str(y_pos)+" "+str(trq_mean)+" "+str(thr_mean)+" "+str(arspd_mean)+" "+str(aoa_mean)+" "+str(aoss_mean)+" "+str(v_tan_mean)+" "+str(v_rad_mean)+" "+str(v_axial_mean))
                    
                    var = format((float(x_pos)/1000)*float(v_axial_mean),'.2f')
                    var_list.append(float(var))
                    trq_list.append(float(trq_mean))
                    thr_list.append(float(thr_mean))
            except:
                pass
                
            with open(self.fname[0], newline = '') as propfile:
                read_data = csv.reader(propfile, delimiter=' ')
                dict_blade_angle = {}  
                for r in read_data:
                    line2 = ' '.join(r)
                    sample2 = list(r)
                    if sample2[0] == str(x_mp):
                        dict_blade_angle[mean_counter] = sample2[1]
                        angle_list = sample2[0]+" "+sample2[1]+" "+sample2[2]
            mean_data = list(mean_list.split(" "))
            angle_data = list(angle_list.split(" "))
            if mean_data[2] == angle_data[0]:
                chord_angle_raw = angle_data[1]
                chord_length_raw = angle_data[2]
                
                denominator1 = (float(omega_mode)*float(x_pos/1000)-float(v_tan_mean))
                if denominator1 == 0:
                    chord_angle = 0.0
                    chord_length = 0.0
                else:
                    denominator2 = math.cos(math.atan(float(v_rad_mean)/denominator1))
                    vs1 = math.tan(math.radians(float(chord_angle_raw)))
                    vs2 = vs1*denominator2
                    chord_angle = math.degrees(math.atan(vs2))
                    chord_length = float(chord_length_raw)/math.cos(math.atan(float(v_rad_mean)/denominator1))
            else:
                chord_angle_raw = 0.0
                chord_angle = 0.0
                chord_length_raw = 0.0
                chord_length = 0.0
            mean_data.append(str(chord_angle_raw))
            mean_data.append(str(format(chord_angle,'.2f')))
            mean_data.append(str(chord_length_raw))
            mean_data.append(str(format(chord_length,'.2f')))
            total_speed = math.sqrt(math.pow(((float(omega_mode)*float(x_pos/1000))-float(v_tan_mean)),2)+math.pow(float(v_axial_mean),2)+math.pow(float(v_rad_mean),2))
            try:
                helix_angle = math.degrees(math.asin(float(v_axial_mean)/float(total_speed)))
            except:
                helix_angle = 0
            mean_data.append(str(format(helix_angle,'.2f')))
            alpha_angle = format(float(chord_angle) - helix_angle,'.2f')
            mean_data.append(str(alpha_angle))
            mean_data.append(str(format(total_speed,'.2f'))) 
            v_lift = (float(v_axial_mean) * math.cos(math.radians(float(helix_angle))) + (float(v_tan_mean) * math.sin(math.radians(float(helix_angle)))))
            mean_data.append(str(format(v_lift,'.2f')))
            v_drag = (float(v_tan_mean) * math.cos(math.radians(float(helix_angle))) - (float(v_axial_mean) * math.sin(math.radians(float(helix_angle)))))
            mean_data.append(str(format(v_drag,'.2f')))
            try:
                coeff_lift = (2 * float(v_lift))/float(total_speed)
            except:
                coeff_lift = 0
            mean_data.append(str(format(coeff_lift,'.3f')))
            try:
                coeff_drag = (2 * float(v_drag))/float(total_speed)
            except:
                coeff_drag = 0
            mean_data.append(str(format(coeff_drag,'.3f')))
            Re = (float(chord_length)/1000 * float(total_speed))/(float(self.shared_data.kin_visc) * math.pow(10,-5))
            mean_data.append(str(format(Re,'.0f')))
            v_2d = math.sqrt(math.pow(float(v_axial_mean),2) + math.pow(float(v_rad_mean),2))
            mean_data.append(str(format(v_2d,'.2f')))
            mean_data.append(str(format(self.dr_ratio.value(),'.1f')))
#             diff_mass_rate = math.pi*self.shared_data.rho*float(v_axial_mean)*(self.radius_mm/1000)
#             mean_data.append(str(format(diff_mass_rate,'.2f')))

            with open(os.path.join(self.path,mean_csvfile), 'a') as f:
                w = csv.writer(f)
                w.writerow([' '.join(mean_data)])
            
            self.update_plot_ax2(x_pos, v_tan_mean, v_rad_mean, v_axial_mean)
            x_mp = x_mp + 3
            
        vi = (2*(self.shared_data.x_delta/1000)*sum(var_list))/math.pow(float(self.radius_mm)/1000,2)
        self.label13.setText(str(format(vi,'.2f')))
        T = statistics.mean(thr_list)
        Pi = float(vi)*float(T)
        self.label15.setText(str(format(Pi,'.2f')))
        try:
            vv = float(T)/(self.shared_data.rho * math.pi * math.pow(float(self.radius_mm)/1000,2) * math.pow(float(vi),2))
        except:
            vv = 0
        self.label65.setText(str(format(vv,'.2f')))
        M = statistics.mean(trq_list)
        P = float(M)*float(omega_mode)
        self.label17.setText(str(format(P,'.2f')))
        vm = self.shared_data.rho * math.pi * math.pow(float(self.radius_mm)/1000,2) * float(vi)
        self.label67.setText(str(format(vm,'.2f')))
        try:
            v_max_mean = float(T)/float(vm)
        except:
            v_max_mean = 0
        self.label69.setText(str(format(v_max_mean,'.2f')))
        try:
            Ct = float(T)/(self.shared_data.rho * math.pow(float(omega_mode),2) * math.pow(((float(self.radius_mm)*2)/1000),4))
        except:
            Ct = 0
        try:
            Cp = float(M)/(self.shared_data.rho * math.pow(float(omega_mode),2) * math.pow(((float(self.radius_mm)*2)/1000),5))
        except:
            Cp = 0
        try:
            nu = (Pi/P)*100
            self.label19.setText(str(format(nu,'.2f')))
        except:
            nu = -1
            self.label19.setText(str(nu))
        
        with open(os.path.join(self.path,mean_csvfile), 'a') as f:
            w = csv.writer(f, delimiter=' ')
            w.writerow(['Omega',format(float(omega_mode),'.2f'),'rad/s'])
            w.writerow(['Induced_power',format(Pi,'.2f'),'W'])
            w.writerow(['Power',format(P,'.2f'),'W'])
            w.writerow(['Efficiency',format(nu,'.2f'),'%'])
            w.writerow(['Average_induced_speed',format(vi,'.2f'),'m/s'])
            w.writerow(['Airspeed_ratio',format(vv,'.2f')])
            w.writerow(['V_mass',format(vm,'.2f'),'kg/s'])
            w.writerow(['V_max_mean',format(v_max_mean,'.2f'),'m/s'])
            w.writerow(['Ct',format(Ct,'.7f')])
            w.writerow(['Cp',format(Cp,'.7f')])
            w.writerow(['Air_density',self.shared_data.rho,'kg/m3'])
            w.writerow(['Air_kinematic_viscosity',self.shared_data.kin_visc,'x10-5 m2/s'])
        
        var_list.clear()
        trq_list.clear()
        thr_list.clear()
        self.counter = 0
        self.cnv.draw_ax2()
        
        self.cnv.save_only_second_plot(os.path.join(self.path, plot_filename))

    def calculate_CT_show(self):
        self.calculate_CT.show()
