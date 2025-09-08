import time, os, csv, math, statistics, datetime
import serial
from pathlib import Path
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal, pyqtSlot, QMetaObject
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QLabel, QGridLayout, QToolBar, QAction, QComboBox,
    QPushButton, QDoubleSpinBox, QSpinBox, QProgressBar, QFileDialog, QCheckBox
)

from plot.canvas import Canvas
from data.shared_data import SharedData
from data import data_processing as _process_data
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
    calFactorUpdated = pyqtSignal(float)
    tareDone = pyqtSignal()
    initReady = pyqtSignal()
    measurementsFrame = pyqtSignal(int, int, object)
    
    def __init__(self, parent=None, **kwargs):
        super().__init__(parent, **kwargs)
        self.shared_data = SharedData()
        self.setupUI()
        app_globals.window = self
        self.controller = None
        self.serialReader = None
        self.measuringWorker = None
        self.serialReaderThread = QThread()
        self.measuringThread = QThread()
        self.lc_calibration_1 = LC_calibration_1(self.shared_data)
        self.lc_calibration_2 = LC_calibration_2(self.shared_data)
        self.calFactorUpdated.connect(self.lc_calibration_1.on_cal_factor)
        self.calFactorUpdated.connect(self.lc_calibration_2.on_cal_factor)
        self.tareDone.connect(self.lc_calibration_1.on_tare_done, Qt.QueuedConnection)
        self.tareDone.connect(self.lc_calibration_2.on_tare_done, Qt.QueuedConnection)
        self.rpm_setup_1 = RPM_controller_1(self.shared_data)
        self.rpm_setup_2 = RPM_controller_2(self.shared_data)
        self.calculate_CT = Calculate_center_of_thrust()
        self.today_dt = None
        self.path = None
        self.csvfile = None
        self.cal_value = 0.0

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
        self.first_thrust_test_value = 0.0          # corrected thrust (N) or mN? (see mapping below)
        self.first_thr_weight_test_value = 0.0      # raw thrust in grams
        self.first_torque_test_value = 0.0          # corrected torque (N·mm) or mN·m
        self.first_trq_weight_test_value = 0.0      # raw torque in grams
        self.first_rpm = 0.0
        self.second_thrust_test_value = 0.0
        self.second_thr_weight_test_value = 0.0
        self.second_torque_test_value = 0.0
        self.second_trq_weight_test_value = 0.0
        self.second_rpm = 0.0
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
        self._series_running = False
        self._await_center_for_next_sweep = False
        self._series_running = False
        self._post_sweep_phase = "idle"   # idle|move_y0|centering|move_y_back
        self._post_sweep_delay_ms = 3000  # ~3 s between commands
        self._postprocess_after_home = False
        self._going_home = False
        self._x_center_steps_actual = None
        self._returning_home = False
        self._home_retry = False
        self._centering_via_jog = False
        self._user_abort = False

        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.showMaximized()
        layout = QGridLayout(self.centralWidget)
        self.toolbar = QToolBar("Lisavalikud")
        self.addToolBar(self.toolbar)

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
        self.dr_ratio.setSingleStep(0.2)
        self.dr_ratio.setValue(0.0)
        layout.addWidget(self.dr_ratio, 10, 0, 1, 1)
        
        self.centering = QPushButton("Pitot' tsentrisse")
        self.centering.setEnabled(False)
        self.centering.clicked.connect(self.center)
        self.centering.setStyleSheet("background-color: None; color: None;")
        layout.addWidget(self.centering, 11, 0, 1, 1)
        
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
        self.Y_move.setStyleSheet("background-color: None; color: None;")
        layout.addWidget(self.Y_move, 14, 0, 1, 1)
        
        self.label_reminder1 = QLabel("Lülita toiteplokk sisse")
        layout.addWidget(self.label_reminder1, 15, 0, 1, 1)
        
        self.label_reminder2 = QLabel("Vali korrektne pinge ja vool")
        layout.addWidget(self.label_reminder2, 16, 0, 1, 1)
        
        self.label4 = QLabel("1. posti propelleri kiirus (%)")
        layout.addWidget(self.label4, 17, 0, 1, 1)
        
        self.first_throttle = QDoubleSpinBox()
        self.first_throttle.setMinimum(10.0)
        self.first_throttle.setMaximum(100.0)
        self.first_throttle.setValue(10.0)
        self.first_throttle.setSingleStep(0.1)
        layout.addWidget(self.first_throttle, 18, 0, 1, 1)
        
        self.label7 = QLabel("2. posti propelleri kiirus (%)")
        self.label7.setEnabled(False)
        layout.addWidget(self.label7, 19, 0, 1, 1)
        
        self.second_throttle = QDoubleSpinBox()
        self.second_throttle.setEnabled(False)
        self.second_throttle.setMinimum(10.0)
        self.second_throttle.setMaximum(100.0)
        self.second_throttle.setValue(10.0)
        self.second_throttle.setSingleStep(0.1)
        layout.addWidget(self.second_throttle, 20, 0, 1, 1)
        
        self.testMotorButton = QPushButton("Testi mootorit", self)
        self.testMotorButton.setEnabled(False)
        self.testMotorButton.setCheckable(True)
        self.testMotorButton.clicked.connect(self.toggle_motor)
        layout.addWidget(self.testMotorButton, 21, 0, 1, 1)
        
        self.label5 = QLabel("Mõõdistamise kiirus")
        layout.addWidget(self.label5, 22, 0, 1, 1)
        
        self.measure_speed = QSpinBox()
        self.measure_speed.setMinimum(200)
        self.measure_speed.setMaximum(500)
        self.measure_speed.setSingleStep(100)
        self.measure_speed.setValue(200)
        layout.addWidget(self.measure_speed, 23, 0, 1, 1)
        
        self.label11 = QLabel("Kordusmõõtmiste arv")
        layout.addWidget(self.label11, 24, 0, 1, 1)
        
        self.sweep_count = QSpinBox()
        self.sweep_count.setMinimum(1)
        self.sweep_count.setMaximum(max_number_of_samples_default)
        self.sweep_count.setSingleStep(1)
        self.sweep_count.setValue(1)
        layout.addWidget(self.sweep_count, 25, 0, 1, 1)
        
        self.trajectory_mode_label = QLabel("", self)
        layout.addWidget(self.trajectory_mode_label, 26, 0, 1, 1)  # sits right above the start button
        self.update_trajectory_label()
        
        self.measure = QPushButton("Alusta mõõtmisega")
        self.measure.setEnabled(False)
        self.measure.clicked.connect(self.start_measuring)
        layout.addWidget(self.measure, 27, 0, 1, 1)
        
        self.label6 = QLabel("Mõõdistamise kulg")
        layout.addWidget(self.label6, 28, 0, 1, 1)
        
        self.progress = QProgressBar()
        self.progress.setMinimum(0)
        self.progress.setMaximum(100)
        self.progress.setValue(0)
        #layout.addWidget(self.progress, 26, 0, 1, 1)
        
        self.test_progress = QProgressBar()
        self.test_progress.setMinimum(0)
        self.test_progress.setMaximum(self.sweep_count.value())
        self.test_progress.setValue(0)
        layout.addWidget(self.test_progress, 29, 0, 1, 1)
        
        self.back = QPushButton("Pitot' tagasi algasendisse")
        self.back.setEnabled(False)
        self.back.clicked.connect(self.come_back)
        layout.addWidget(self.back, 30, 0, 1, 1)
        
        self.danger = QLabel("Emergency!")
        self.danger.setAlignment(Qt.AlignCenter)
        self.danger.setStyleSheet("background-color: None")
        layout.addWidget(self.danger, 31, 0, 1, 1)
        
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

        self.timer_first_motor = QTimer(self)
        self.timer_first_motor.timeout.connect(self.read_first_thr_values)
        
        self.timer_second_motor = QTimer(self)
        self.timer_second_motor.timeout.connect(self.read_second_thr_values)

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
        self.homing.setText("Telgede referents ✓")
        self.meas_data_running = False
    
    @pyqtSlot(str)    
    def handleData(self, data):
        for raw in (data or "").splitlines():
            s = raw.strip()
            if not s:
                continue
            slow = s.lower()
            print(slow)
            if 'ready!' in slow:
                self.params.setStyleSheet("background-color: green; color: white;")
                self.params.setText("Säti andurid ✓")  # or whatever label you like
                self.xy_axes.setEnabled(True)
                self.initReady.emit()
            if 'emergency!' in slow:
                self.e_stop = True
                self.meas_data_running = False
                self.update_emergency()
                return
            if 'emergency cleared' in slow:
                self.e_stop = False
                self.meas_data_running = False
                self.update_emergency()
                return
            if 'homing done' in slow:
                self.homingDone()
                return
            if 'limit switch' in slow:
                self.homing_done = True
                self.measure.setEnabled(False)
                self.centering.setEnabled(False)
                self.testMotorButton.setEnabled(True)
                self.test_progress.setValue(0)
                self.homing.setStyleSheet("background-color: orange; color: None;")
                return
            if 'tare done' in slow:
                self.meas_data_running = False
                self.tareDone.emit()
                self.tare_done = True
                return
            if 'centering done' in slow:
                self.centering.setStyleSheet("background-color: green; color: white;")
                self.centering.setText("Pitot' tsentrisse ✓")
                self.meas_data_running = False
                self.Y_move.setEnabled(True)
                if self._post_sweep_phase == "centering":
                    QTimer.singleShot(self._post_sweep_delay_ms, self._post_sweep_moveYBack)
                return
            if 'jog done' in slow:
                self.jog_done = True
                if getattr(self, "_centering_via_jog", False):
                    try:
                        # Mirror your existing 'centering done' visuals/text
                        self.centering.setStyleSheet("background-color: green; color: white;")
                        self.centering.setText("Pitot' tsentrisse ✓")
                        self.meas_data_running = False
                        self.Y_move.setEnabled(True)
                        self._centering_via_jog = False
                    except Exception:
                        pass
                    #finally:
                    #    self._centering_via_jog = False

                # Continue post-sweep chain if applicable
                if getattr(self, "_post_sweep_phase", "idle") == "centering":
                    QTimer.singleShot(self._post_sweep_delay_ms, self._post_sweep_moveYBack)
                    return
                # Post-sweep chaining
                if self._post_sweep_phase == "move_y0":
                    # 3) center after delay
                    QTimer.singleShot(self._post_sweep_delay_ms, self._post_sweep_center)
                    return

                if self._post_sweep_phase == "move_y_back":
                    # We have completed Y -> series Y₀
                    self._post_sweep_phase = "idle"
                    if self.current_sweep < self.total_sweeps:
                        # More sweeps to go -> start the next sweep after a small delay
                        QTimer.singleShot(self._post_sweep_delay_ms, self.run_next_sweep)
                    else:
                        # FINAL: sweep count reached AND Y is back -> now go home
                        QTimer.singleShot(self._post_sweep_delay_ms, self.come_back)
                if getattr(self, "_going_home", False):
                    self._going_home = False
                    if getattr(self, "_postprocess_after_home", False):
                        self._postprocess_after_home = False
                        try:
                            self.process_data()
                        except Exception as e:
                            print("post-processing error:", e)
                # If we are returning home via jog, finish up now
                if getattr(self, "_returning_home", False):
                    self._returning_home = False
                    # Beacon off AFTER movement completes
                    QTimer.singleShot(1000, lambda: self.sendData('BeaconOFF'))
                return
            if 'over axis limit' in slow:
                # Always stop motion first
                self.sendData('stop')

                # Small helper: nudge Y target inward if we were aiming for the boundary
                def _shrink_y_target():
                    try:
                        ratio = float(self.shared_data.ratio)
                        # back off by 2 mm from max
                        max_safe_y = int(self.Y_pos.maximum() * ratio) - int(2 * ratio)
                        self._series_y0_steps = max(0, min(int(self._series_y0_steps), max_safe_y))
                    except Exception:
                        pass

                phase = getattr(self, "_post_sweep_phase", "idle")

                if phase in ("stopping", "move_y0"):
                    # We were sending Y->0; just retry that step after delay
                    QTimer.singleShot(self._post_sweep_delay_ms, self._post_sweep_moveY0)
                    return

                if phase == "centering":
                    # We were centering; try center again after delay
                    QTimer.singleShot(self._post_sweep_delay_ms, self._post_sweep_center)
                    return

                if phase == "move_y_back":
                    # We were restoring Y; clamp & back off a bit, then retry
                    _shrink_y_target()
                    QTimer.singleShot(self._post_sweep_delay_ms, self._post_sweep_moveYBack)
                    return

                # If the message arrives outside post-sweep phases (e.g., during measuring),
                # do a gentle recovery: center → Y back → continue, all with delays.
                def _soft_recover():
                    self._post_sweep_phase = "centering"
                    self._post_sweep_center()
                    QTimer.singleShot(self._post_sweep_delay_ms, self._post_sweep_moveYBack)
                QTimer.singleShot(self._post_sweep_delay_ms, _soft_recover)
                
                self.sendData('stop')

                if getattr(self, "_returning_home", False):
                    def _retry_home():
                        x_home, y_home = self._home_steps()  # already clamped to HW limits
                        feed_xy, feed_y = self._safe_feeds()
                        self.sendData(f'j|{x_home}|{y_home}|{feed_xy}|{feed_y}')
                    # back off once; if it still fails, we just won't loop forever
                    if not self._home_retry:
                        self._home_retry = True
                        QTimer.singleShot(self._post_sweep_delay_ms if hasattr(self, "_post_sweep_delay_ms") else 1500,
                                          _retry_home)
                    else:
                        print("Home jog failed twice; staying stopped for safety.")
                        self._returning_home = False
                return
            if slow.startswith('calval:'):
                try:
                    val = float(data.split(":", 1)[1].strip())
                except ValueError:
                    return
                self.cal_value = val
                self.calFactorUpdated.emit(val)  # <- broadcast to any open calibration windows
                return
            elif slow.startswith('lc test:'):
                parts = slow.split()[2:]  # after "LC" and "test:"
                if len(parts) != 10:
                    print("Error parsing numeric data:", parts)
                else:
                    try:
                        vals = [float(x) for x in parts]
                        (thr1_corr, thr1_raw_g, trq1_corr, trq1_raw_g, rpm1,
                         thr2_corr, thr2_raw_g, trq2_corr, trq2_raw_g, rpm2) = vals

                        w = app_globals.window
                        # prop #1
                        w.first_thrust_test_value     = thr1_corr
                        w.first_thr_weight_test_value = thr1_raw_g
                        w.first_torque_test_value     = trq1_corr
                        w.first_trq_weight_test_value = trq1_raw_g
                        w.first_rpm                   = rpm1

                        # prop #2
                        w.second_thrust_test_value     = thr2_corr
                        w.second_thr_weight_test_value = thr2_raw_g
                        w.second_torque_test_value     = trq2_corr
                        w.second_trq_weight_test_value = trq2_raw_g
                        w.second_rpm                   = rpm2
                        
                        # Convert ONLY for display (mN→N, N·mm→N·m)
                        self.update_first_thr_label(f"{thr1_corr/1000.0:.2f}")
                        self.update_first_thr_weight_label(f"{thr1_raw_g:.1f}")
                        self.update_first_trq_label(f"{trq1_corr/1000.0:.3f}")
                        self.update_first_trq_weight_label(f"{trq1_raw_g:.1f}")
                        self.update_first_rpm_label(f"{rpm1:.0f}")

                        self.update_second_thr_label(f"{thr2_corr/1000.0:.2f}")
                        self.update_second_thr_weight_label(f"{thr2_raw_g:.1f}")
                        self.update_second_trq_label(f"{trq2_corr/1000.0:.3f}")
                        self.update_second_trq_weight_label(f"{trq2_raw_g:.1f}")
                        self.update_second_rpm_label(f"{rpm2:.0f}")
                    except Exception as e:
                        print("Error parsing numeric data:", parts, e)
                    return
            elif slow.startswith('measurements:'):
                # MCU sends: mN, Nmm; we convert to N, Nm.
                # Expected order (13 floats):
                # X Y firstThrust_mN firstTorque_Nmm firstRPM airspeed aoa_raw aoa_abs aoss_raw aoss_abs secondThrust_mN secondTorque_Nmm secondRPM
                payload = slow.split('measurements:', 1)[1].strip()
                parts = payload.split()
                if len(parts) != 13:
                    print("Measurements expected 13 fields, got", len(parts), parts)
                    return

                try:
                    (x_f, y_f,
                     first_thrust_mN, first_torque_Nmm, first_rpm,
                     airspeed, aoa_raw, aoa_abs, aoss_raw, aoss_abs,
                     second_thrust_mN, second_torque_Nmm, second_rpm) = map(float, parts)
                except ValueError:
                    print("Error parsing Measurements numeric data:", parts)
                    return

                # Positions are integers in steps
                x_steps = int(round(x_f))
                y_steps = int(round(y_f))

                # Convert units + apply arm ratio:
                first_thrust_N  = (first_thrust_mN  / 1000.0)         # mN -> N, then divide by ratio
                first_torque_Nm = (first_torque_Nmm / 1000.0)                 # Nmm -> Nm

                second_thrust_N  = (second_thrust_mN  / 1000.0)
                second_torque_Nm = (second_torque_Nmm / 1000.0)

                # Update window fields (now in SI)
                self.x_pos = x_steps
                self.y_pos = y_steps

                self.first_thrust_value  = first_thrust_N
                self.first_torque_value  = first_torque_Nm
                self.first_rpm           = first_rpm

                self.airspeed       = airspeed
                self.aoa_raw_value  = aoa_raw
                self.aoa_abs_value  = aoa_abs
                self.aoss_raw_value = aoss_raw
                self.aoss_abs_value = aoss_abs

                self.second_thrust_value = second_thrust_N
                self.second_torque_value = second_torque_Nm
                self.second_rpm          = second_rpm

                self.meas_data_running = True

                # Emit a clean, 13-field SI frame for workers/plots:
                vals = [
                    float(x_steps), float(y_steps),
                    first_thrust_N, first_torque_Nm, first_rpm,
                    airspeed, aoa_raw, aoa_abs, aoss_raw, aoss_abs,
                    second_thrust_N, second_torque_Nm, second_rpm
                ]
                # Make sure you've declared in MainWindow: measurementsFrame = pyqtSignal(int, int, object)
                self.measurementsFrame.emit(x_steps, y_steps, vals)

        if (self.motor_test == False) and (not getattr(self, "_series_running", False)) and (not self.measuringThread.isRunning()):
            self.update_first_rpm_label('0')
            self.update_second_rpm_label('0')
            self.update_first_thr_label('0')
            self.update_second_thr_label('0')
            self.update_first_trq_label('0')
            self.update_second_trq_label('0')
            self.update_first_thr_weight_label('0')
            self.update_second_thr_weight_label('0')
            self.update_first_trq_weight_label('0')
            self.update_second_trq_weight_label('0')
            #self.meas_data_running = False
        
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
        self.timer_first_motor.start(1000)
        if self.tandem_setup:
            self.timer_second_motor.start(1000)

    def stop_motor(self):
        self.sendData('stop')
        self.timer_first_motor.stop()
        if self.tandem_setup:
            self.timer_second_motor.stop()
        self.sendData('stop')
        self.motor_test = False
        #time.sleep(2)

    def update_throttle_test(self):
        first_throttle = int(1000 + (self.first_throttle.value() * 10))
        if self.tandem_setup:
            second_throttle = int(1000 + (self.second_throttle.value() * 10))
        else:
            second_throttle = 1000
        start_motor = f'startMotor|{first_throttle}|{second_throttle}'
        self.sendData(start_motor)
        self.motor_test = True
        
    def read_first_thr_values(self):
        current_first_throttle = self.first_throttle.value()
        if current_first_throttle != self.last_first_throttle_value:
            self.update_throttle_test()
            self.last_first_throttle_value = current_first_throttle

    def read_second_thr_values(self):
        current_second_throttle = self.second_throttle.value()
        if current_second_throttle != self.last_second_throttle_value:
            self.update_throttle_test()
            self.last_second_throttle_value = current_second_throttle
            
    def initSerialReader(self):
        if self.controller and not self.serialReaderThread.isRunning():
            if self.serialReader:
                self.serialReaderThread.quit()
                self.serialReaderThread.wait()

            self.serialReader = SerialReader(self.controller)
            self.serialReader.moveToThread(self.serialReaderThread)
            self.serialReader.serial_readout.connect(self.handleData)
            self.serialReaderThread.started.connect(self.serialReader.run)
            self.serialReaderThread.start()

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
            
    def update_second_thr_label(self, thr):
        if self.motor_test == True:
            self.second_thr_label.setText(thr)
        else:
            self.second_thr_label.setText('0')
            
    def update_first_trq_label(self, trq):
        if self.motor_test == True:
            self.first_trq_label.setText(trq)
        else:
            self.first_trq_label.setText('0')
            
    def update_second_trq_label(self, trq):
        if self.motor_test == True:
            self.second_trq_label.setText(trq)
        else:
            self.second_trq_label.setText('0')
            
    def update_first_thr_weight_label(self, thr_weight):
        if self.motor_test == True:
            self.first_thr_weight_label.setText(thr_weight)
        else:
            self.first_thr_weight_label.setText('0')
            
    def update_second_thr_weight_label(self, thr_weight):
        if self.motor_test == True:
            self.second_thr_weight_label.setText(thr_weight)
        else:
            self.second_thr_weight_label.setText('0')
            
    def update_first_trq_weight_label(self, trq_weight):
        if self.motor_test == True:
            self.first_trq_weight_label.setText(trq_weight)
        else:
            self.first_trq_weight_label.setText('0')
            
    def update_second_trq_weight_label(self, trq_weight):
        if self.motor_test == True:
            self.second_trq_weight_label.setText(trq_weight)
        else:
            self.second_trq_weight_label.setText('0')
        
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
                ("Omega1 x 10 (rad/s)", 'r-'),
                ("Airspeed (m/s)", 'b-'),
                ("AoA (deg)", 'g-'),
                ("AoSS (deg)", 'm-'),
                ("Torque1 (Nm)", 'c-'),
                ("Thrust1 (N)", 'k-')
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
                self.connect.setText("Ühendatud ✓")
                self.params.setEnabled(True)
                self.sendData('BeaconOFF')
                
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
        # Construct path to ~/Desktop/propellerid
        prop_folder = Path.home() / "Desktop" / "propellerid"
        prop_folder.mkdir(parents=True, exist_ok=True)  # ensure it exists

        self.file.setStyleSheet("background-color: None; color: None;")
        self.fname = QFileDialog.getOpenFileName(
            self,
            'Otsi propelleri konfiguratsiooni fail',
            str(prop_folder),          # start directory
            "CSV Files (*.csv)"        # file type filter
            )

        if self.fname[0]:  # check filename part
            self.fileSelected = True
            self.homing.setEnabled(True)
            self.file.setStyleSheet("background-color: green; color: white;")
            self.file.setText("Vali propelleri konfiguratsiooni fail ✓")

    def update_emergency(self):
        if self.e_stop:
            # Hard stop + tear down any motor-test activity
            try:
                self.stop_motor()  # stops timers + sends 'stop'
            except Exception:
                self.sendData('stop')

            # Ensure ESCs and beacon are OFF so MCU stops LC test spam
            QTimer.singleShot(0,   lambda: self.sendData('OFF'))
            QTimer.singleShot(200, lambda: self.sendData('BeaconOFF'))
            QTimer.singleShot(400, lambda: self.sendData('OFF'))

            self.motor_test = False
            self.testMotorButton.setChecked(False)
            self.testMotorButton.setText("Testi mootorit")   # reset label right away
            self.testMotorButton.setEnabled(False)

            self.measuring_stopped = True
            self.danger.setStyleSheet("background-color: red; color: None;")
            self.measure.setEnabled(False)
            self.centering.setEnabled(False)
            self.centering.setText("Pitot' tsentrisse")
            self.back.setEnabled(False)
            self.homing.setStyleSheet("background-color: None; color: None;")
            self.homing.setText("Telgede referents")
        else:
            # Emergency cleared: restore visuals and allow motor test again (when safe)
            self.danger.setStyleSheet("background-color: None; color: None;")
            # Re-enable the motor test button if we have homed (same as normal flow)
            self.testMotorButton.setEnabled(self.homing_done)
            self.sendData('OFF')
            self.testMotorButton.setText("Testi mootorit")
            
    def set_params(self):
        if not hasattr(self, 'Parameetrid'):
            #self.param_window = SetParameters(self.shared_data)
            self.param_window = SetParameters(self.shared_data)
            self.param_window.sendData.connect(self.sendData)
            self.initReady.connect(self.param_window.on_init_ready)
        self.param_window.show()
        
    def map_(self):
        if not hasattr(self, 'Trajektoor'):
            self.map_window = MapTrajectory(self.shared_data)
            self.map_window.sendData.connect(self.sendData)
            try:
                self.map_window.modeChanged.connect(lambda active: self.update_trajectory_label())
            except Exception:
                pass
        self.map_window.show()
                
    def calibrate_first_loadcells(self):
        if not hasattr(self, '1. posti andurite kalibreerimine'):
            self.lc_calib_1_window = LC_calibration_1(self.shared_data)
            self.lc_calib_1_window.sendData.connect(self.sendData)
            self.calFactorUpdated.connect(self.lc_calib_1_window.on_cal_factor)  # <- fix
        self.lc_calib_1_window.show()
        if self.cal_value:
            self.lc_calib_1_window.on_cal_factor(self.cal_value)
        
    def calibrate_second_loadcells(self):
        if not hasattr(self, '2. posti andurite kalibreerimine'):
            self.lc_calib_2_window = LC_calibration_2(self.shared_data)
            self.lc_calib_2_window.sendData.connect(self.sendData)
            self.calFactorUpdated.connect(self.lc_calib_2_window.on_cal_factor)
        self.lc_calib_2_window.show()
        if self.cal_value:
            self.lc_calib_2_window.on_cal_factor(self.cal_value)
    
    def set_xy_axes(self):
        if not hasattr(self, 'XY telgede konfigureerimine'):
            self.axes_conf_window = SetXYAxes(self.shared_data)
            self.axes_conf_window.sendData.connect(self.sendData)
            self.axes_conf_window.centerStepsChanged.connect(self.on_center_steps_changed)
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
        if not hasattr(self, 'Tõmbekeskme arvutus'):
            self.CT_setup_window = Calculate_center_of_thrust()
        self.CT_setup_window.show()
        
    def home(self):
        self.homing_done = False
        self.homing.setStyleSheet("background-color: None; color:None;")
        self.sendData('home')
        
    def center(self):
        """UI 'Center' now means: jog to (center – |offset|, Y=0)."""
        self.back.setEnabled(True)
        self.Y_move.setEnabled(False)

        # Mark this jog as 'centering', so handleData can flip the same UI flags
        self._centering_via_jog = True

        # X target with probe-offset toward zero
        x_center = self._center_steps()
        x_target = x_center + self._offset_towards_zero_steps()

        # Y MUST be 0 — do not read telemetry/UI; clamp with 0 mm margin to reach 0 exactly
        x_min, x_max, y_min, y_max = self._axis_limits_steps(x_margin_mm=0.0, y_margin_mm=0.0)
        x_cmd = max(x_min, min(int(x_target), x_max))
        y_cmd = max(y_min, min(0, y_max))  # → 0

        feed_xy, feed_y = self._safe_feeds()
        self.sendData(f'j|{x_cmd}|{y_cmd}|{feed_xy}|{feed_y}')
        
#     def center(self):
#         self.back.setEnabled(True)
#         self.Y_move.setEnabled(False)
#         self.sendData('center')
    
    def on_center_steps_changed(self, steps: int):
        """Keep shared_data.x_center (mm) snapped to the integer step grid."""
        r = float(self.shared_data.ratio)
        self._x_center_steps_from_ui = int(steps)
        self.shared_data.x_center = self._x_center_steps_from_ui / r
        print(f"[center] from SetXYAxes: {self._x_center_steps_from_ui} steps "
              f"({self.shared_data.x_center:.6f} mm)")
    
    def come_back(self):
        # Abort any series chaining asap
        self._user_abort = True
        self._series_running = False
        self._post_sweep_phase = "idle"

        # Mark return-home path
        self._returning_home = True
        self._home_retry = False

        # 1) Gracefully stop the worker (flush CSV, close file)
        try:
            if getattr(self, "measuringWorker", None) is not None:
                QMetaObject.invokeMethod(self.measuringWorker, "cancel", Qt.QueuedConnection)
        except Exception as e:
            print("cancel invoke error:", e)

        # 2) Stop motion and tear down thread
        self.meas_data_running = False
        self.sendData('stop')
        try:
            if self.measuringThread.isRunning():
                self.measuringThread.quit()
                self.measuringThread.wait()
        except Exception as e:
            print("thread stop error:", e)

        # 3) Reset UI bits (NO self.progress here)
        try:
            self.centering.setStyleSheet("background-color: None; color: None;")
            self.centering.setText("Pitot' tsentrisse")
            self.Y_move.setStyleSheet("background-color: None; color: None;")
            self.Y_move.setText("Liiguta Y-telge")
            self.measure.setText("Alusta mõõtmist")
            self.measure.setStyleSheet("background-color: none; color: none;")
            self.measure.setEnabled(False)
            self.testMotorButton.setEnabled(True)
            if hasattr(self, "test_progress") and self.test_progress:
                self.test_progress.setValue(0)
            self.counter = 0
        except Exception:
            pass

        # 4) After a safer pause, send a single jog to HW home (Xmin, Ymin)
        def _do_home_jog():
            try:
                x_home, y_home = self._home_steps()
            except Exception:
                x_min, x_max, y_min, y_max = self._axis_limits_steps(x_margin_mm=0.0, y_margin_mm=0.0)
                x_home, y_home = x_min, y_min
            feed_xy, feed_y = self._safe_feeds()
            cmd = f'j|{x_home}|{y_home}|{feed_xy}|{feed_y}'
            print("[come_back] HOME jog:", cmd)
            self.sendData(cmd)

        gap = getattr(self, "_command_gap_ms", 1800)  # give the MCU time to settle post-'stop'
        QTimer.singleShot(gap, _do_home_jog)


#     def come_back(self):
#         # Stop any multi-sweep chaining immediately
#         self._user_abort = True
#         self._series_running = False
#         self._post_sweep_phase = "idle"
# 
#         # Mark we're returning home via jog
#         self._returning_home = True
#         self._home_retry = False
# 
#         # 1) Gracefully stop the worker (flush CSV, close file)
#         try:
#             if getattr(self, "measuringWorker", None) is not None:
#                 QMetaObject.invokeMethod(self.measuringWorker, "cancel", Qt.QueuedConnection)
#         except Exception as e:
#             print("cancel invoke error:", e)
# 
#         # 2) Stop motion and tear down thread
#         self.meas_data_running = False
#         self.sendData('stop')
#         try:
#             if self.measuringThread.isRunning():
#                 self.measuringThread.quit()
#                 self.measuringThread.wait()
#         except Exception as e:
#             print("thread stop error:", e)
# 
#         # 3) Reset UI bits (but DO NOT BeaconOFF yet)
#         self.centering.setStyleSheet("background-color: None; color: None;")
#         self.centering.setText("Pitot' tsentrisse")
#         self.Y_move.setStyleSheet("background-color: None; color: None;")
#         self.Y_move.setText("Liiguta Y-telge")
#         self.measure.setText("Alusta mõõtmist")
#         self.measure.setStyleSheet("background-color: none; color: none;")
#         self.measure.setEnabled(False)
#         self.testMotorButton.setEnabled(True)
#         self.test_progress.setValue(0)
#         self.counter = 0

#         # 4) After a short pause, send a single jog to HW home (Xmin, Ymin)
#         def _do_home_jog():
#             x_home, y_home = self._home_steps()
#             feed_xy, feed_y = self._safe_feeds()
#             self.sendData(f'j|{x_home}|{y_home}|{feed_xy}|{feed_y}')
#         QTimer.singleShot(1000, _do_home_jog)
        
    def enable_Y_move_button(self):
        self.Y_move.setStyleSheet("background-color: orange; color: black;")
        
    def moveY(self):
        """
        Move Y to the requested coordinate while keeping X parked at (center – |offset|).
        """
        # Compute X target with the shaft-safe offset toward zero
        x_center = self._center_steps()
        x_target = x_center + self._offset_towards_zero_steps()

        # Desired Y in steps from the spinbox
        ratio = float(self.shared_data.ratio) or 1.0
        y_target = int(round(self.Y_pos.value() * ratio))

        # Clamp and send jog
        x_cmd, y_cmd = self._clamp_xy_steps(x_target, y_target)
        feed_xy, feed_y = self._safe_feeds()
        self.sendData(f'j|{x_cmd}|{y_cmd}|{feed_xy}|{feed_y}')

        # UI feedback (unchanged)
        self.Y_move.setStyleSheet("background-color: green; color: white;")
        self.Y_move.setText("Liiguta Y-telge ✓")
        self.measure.setEnabled(True)
        
#     def moveY(self):
#         moveY = 'j|%d|%d|%d|%d' %((self.shared_data.x_center * self.shared_data.ratio), (self.Y_pos.value() * self.shared_data.ratio), self.jog_speed.value(), self.jog_speed.value())
#         self.sendData(moveY)
#         self.Y_move.setStyleSheet("background-color: green; color: white;")
#         self.Y_move.setText("Liiguta Y-telge ✓")
#         self.measure.setEnabled(True)
        
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
    
    @pyqtSlot(list)
    def on_live_data(self, row):
        """
        MeasuringWorker.liveData payload (see worker headers):
          1-prop (len=12):
            [Prop_in, X, Y, Trq1, Thr1, Omega1, Air, AoA, AoSS, V_tan, V_rad, V_ax]
          2-prop (len=15):
            [Prop_in, X, Y, Trq1, Thr1, Omega1, Air, AoA, AoSS, V_tan, V_rad, V_ax, Trq2, Thr2, Omega2]
        We update Plot 1 live. Plot 2 remains managed by post-processing, as before.
        """
        if not row or len(row) < 12:
            return
        try:
            X      = float(row[1])
            Trq1   = float(row[3])
            Thr1   = float(row[4])
            Omega1 = float(row[5])   # rad/s from worker
            Air    = float(row[6])
            AoA    = float(row[7])
            AoSS   = float(row[8])

            # Your update_plot_ax1 divides the input by 10 and labels "Omega x 10".
            # Pass Omega * 10 so the plotted value equals Omega (rad/s) on-screen.
            self.update_plot_ax1(X, Omega1, Air, AoA, AoSS, Trq1, Thr1)  # Plot 1

            # If tandem, add prop #2 torque/thrust overlays on Plot 1
            if len(row) >= 15:
                Trq2   = float(row[12])
                Thr2   = float(row[13])
                Omega2 = float(row[14])
                # Draw as separate series so both are visible
                try:
                    self.cnv.add_data_ax1([X], [Trq2], "Torque2 (Nm)", 'c--')
                    self.cnv.add_data_ax1([X], [Thr2], "Thrust2 (N)",  'k--')
                    self.cnv.add_data_ax1([X], [Omega2 / 10.0], "Omega2 x 10 (rad/s)", 'r--')
                except Exception:
                    pass
                # Update labels for prop #2
                self.update_second_trq_label(f"{Trq2:.2f}")
                self.update_second_thr_label(f"{Thr2:.2f}")
#                 try:
#                     rpm2 = (Omega2 * 60.0) / (2.0 * math.pi)
                self.update_second_rpm_label(f"{Omega2:.0f}")
#                 except Exception:
#                     pass
        except Exception:
            pass
    
    def start_measuring(self):
        self.measure.setText("Mõõtmine käib...")
        self.measure.setStyleSheet("background-color: orange; color: black;")
        self.measure.setEnabled(False)

        try:
            self.cnv.clear_plots()
        except Exception:
            pass
        # --- series state from UI ---
        self.current_sweep = 0
        self.total_sweeps = int(self.sweep_count.value())
        
        ratio = float(self.shared_data.ratio)
        self._series_y0_steps = int(round(self.Y_pos.value() * ratio))

        self._series_running = True
        self.measuring_stopped = False
        self.testMotorButton.setEnabled(False)

        # --- one folder + one CSV for the whole series ---
        from pathlib import Path
        import datetime
        self.today_dt = datetime.datetime.today().strftime('%d-%m-%Y-%H:%M:%S')
        out_dir = Path.home() / "Desktop" / "logid" / self.today_dt
        out_dir.mkdir(parents=True, exist_ok=True)
        self.path = str(out_dir)
        self.csvfile = f"log{self.today_dt}.csv"
        self.series_csv_path = str(out_dir / self.csvfile)

        # progress across sweeps
        self.test_progress.setMaximum(self.total_sweeps)
        self.test_progress.setValue(0)
        
        ratio = float(self.shared_data.ratio)
        base_y_steps = int(round(self.Y_pos.value() * ratio))

        if getattr(self, 'custom_trajectory', False) and hasattr(self, 'list_of_x_targets') and self.list_of_x_targets:
            ys_off = [int(y) for y in getattr(self, 'list_of_y_targets', [0]*len(self.list_of_x_targets))]
            # If your waypoint Y values are OFFSETS (recommended):
            self._series_y0_steps = base_y_steps + (ys_off[0] if ys_off else 0)
            # If your waypoint Y values are ABSOLUTE steps instead, use this instead:
            # self._series_y0_steps = int(ys_off[0]) if ys_off else base_y_steps
        else:
            self._series_y0_steps = base_y_steps

        # kick off first sweep
        self.run_next_sweep()
        
    def run_next_sweep(self):
        if self.current_sweep >= self.total_sweeps:
            return

        self.current_sweep += 1
        print(f"Starting sweep {self.current_sweep}/{self.total_sweeps}")
        
        try:
            self.cnv.clear_plot1()
        except Exception:
            pass

        points = []
        ratio = float(self.shared_data.ratio)
        x_center_steps = self._center_steps()
        self.shared_data.x_center = x_center_steps / ratio

        self.radius_mm = (self.prop.value() * 25.4) * (1.0 + self.shared_data.safety_over_prop / 100.0) / 2.0

        if getattr(self, 'custom_trajectory', False) and hasattr(self, 'list_of_x_targets') and self.list_of_x_targets:
            xs_steps = [int(x) for x in self.list_of_x_targets]
            ys_off   = [int(y) for y in getattr(self, 'list_of_y_targets', [0]*len(xs_steps))]

            # If your file stores Y OFFSETS (relative to series start Y):
            for x_s, dy in zip(xs_steps, ys_off):
                x_cmd, y_cmd = self._clamp_xy_steps(int(x_s), int(self._series_y0_steps + dy))
                points.append((x_cmd, y_cmd))

            # If your file stores ABSOLUTE Y values instead, switch to:
            # for x_s, y_abs in zip(xs_steps, ys_off):
            #     x_cmd, y_cmd = self._clamp_xy_steps(int(x_s), int(y_abs))
            #     points.append((x_cmd, y_cmd))
        else:
            x_center_steps = self._center_steps()
            self.shared_data.x_center = x_center_steps / ratio   # keeps worker baseline identical to jogs

            # Now use that step for your geometry
            radius_mm   = (self.prop.value() * 25.4) * (1.0 + self.shared_data.safety_over_prop / 100.0) / 2.0
            self.radius_mm = radius_mm
            x_goal_steps = max(0, x_center_steps - int(round(radius_mm * ratio)))
            y0_steps     = int(self._series_y0_steps)

            # Always clamp final waypoints to HW limits
            x_cmd, y_cmd = self._clamp_xy_steps(x_goal_steps, y0_steps)
            points = [(x_cmd, y_cmd)]

        # --- Worker for this sweep (append to SAME CSV) ---
        from PyQt5.QtCore import Qt
        from workers.measuring_worker import MeasuringWorker

        first_pwm  = int(1000 + (self.first_throttle.value() * 10))
        second_pwm = int(1000 + (self.second_throttle.value() * 10)) if self.tandem_setup else 1000

        self.measuringWorker = MeasuringWorker(
            points=points,
            csv_path=self.series_csv_path,   # single file for all sweeps
            samples_per_point=1,
            settle_timeout_s=12.0,
            tandem_setup=self.tandem_setup,
            motor_pwm1=first_pwm,
            motor_pwm2=second_pwm,
            steps_per_mm=float(self.shared_data.ratio),
            parent=self
        )
        self.measuringWorker.moveToThread(self.measuringThread)

        # wire signals
        self.measuringWorker.sendData.connect(self.sendData)  # serial out path
        self.measurementsFrame.connect(self.measuringWorker.on_measurements, type=Qt.QueuedConnection)
        self.tareDone.connect(self.measuringWorker.on_tare_done, type=Qt.QueuedConnection)
        self.measuringWorker.finished.connect(self.on_measuring_finished, Qt.QueuedConnection)
        self.measuringWorker.progress.connect(self.on_worker_progress, Qt.QueuedConnection)        
        try:
            self.measuringWorker.liveData.connect(self.on_live_data, type=Qt.QueuedConnection)
        except Exception:
            pass

        # If tare already done earlier (e.g. calibration), allow immediate proceed
        if getattr(self, "tare_done", False):
            QTimer.singleShot(0, self.measuringWorker.on_tare_done)

        if self.measuringThread.isRunning():
            self.measuringThread.quit()
            self.measuringThread.wait()
        try:
            self.measuringThread.started.disconnect()
        except Exception:
            pass
        self.measuringThread.started.connect(self.measuringWorker.start, Qt.QueuedConnection)
        self.measuringThread.start()
        
    @pyqtSlot(str)
    def on_measuring_finished(self, csv_path):
        # progress across sweeps
        try:
            self.test_progress.setValue(self.current_sweep)
        except Exception:
            pass

        # FINAL SWEEP? go home
        if self.current_sweep >= self.total_sweeps:
            self._post_sweep_phase = "idle"
            self._postprocess_after_home = True
            self._going_home = True
            self.sendData('stop')
            self.measure.setText("Alusta mõõtmist")
            self.measure.setStyleSheet("background-color: none; color: none;")
            self.measure.setEnabled(False)
            QTimer.singleShot(self._post_sweep_delay_ms, self.come_back)
            return

        # --- INTERMEDIATE SWEEP chain ---
        self._post_sweep_phase = "stopping"
        self.sendData('stop')
        QTimer.singleShot(self._post_sweep_delay_ms, self._post_sweep_moveY0)

    @pyqtSlot(int, int)
    def on_worker_progress(self, idx: int, total: int):
        return

    def process_data(self):
        return _process_data.process_data(self)
    
    def _post_sweep_center(self):
        if not getattr(self, "_series_running", False):
            return
        self._post_sweep_phase = "centering"
        self._centering_via_jog = True  # mirror 'centering done' UI on jog

        x_center = self._center_steps()
        x_target = x_center + self._offset_towards_zero_steps()  # center – |offset|
        y_target = int(self._series_y0_steps)  # will be used in next step; keep Y at 0 here if you prefer

        x_cmd, y_cmd = self._clamp_xy_steps(x_target, 0)  # <-- ensure Y stays at 0 in this step
        feed_xy, feed_y = self._safe_feeds()
        self.sendData(f'j|{x_cmd}|{y_cmd}|{feed_xy}|{feed_y}')

#     def _post_sweep_center(self):
#         if not getattr(self, "_series_running", False):
#             return
#         self._post_sweep_phase = "centering"
#         self.sendData('center')

    def _post_sweep_moveY0(self):
        if not getattr(self, "_series_running", False):
            return
        x_now = int(getattr(self, "x_pos", 0))  # last measured steps
        y_zero = 0
        x_cmd, y_cmd = self._clamp_xy_steps(x_now, y_zero)
        feed_xy, feed_y = self._safe_feeds()
        self._post_sweep_phase = "move_y0"
        self.sendData(f'j|{x_cmd}|{y_cmd}|{feed_xy}|{feed_y}')
    
    def _post_sweep_moveYBack(self):
        if not getattr(self, "_series_running", False):
            return
        try:
            x_center_steps = self._center_steps()
            x_target_steps = x_center_steps + self._offset_towards_zero_steps()  # keep offset!
            y_target_steps = int(self._series_y0_steps)

            x_cmd, y_cmd = self._clamp_xy_steps(x_target_steps, y_target_steps)
            feed_xy, feed_y = self._safe_feeds()
            self._post_sweep_phase = "move_y_back"
            self.sendData(f'j|{x_cmd}|{y_cmd}|{feed_xy}|{feed_y}')
        except Exception as e:
            print("moveYBack error:", e)
    
#     def _post_sweep_moveYBack(self):
#         if not getattr(self, "_series_running", False):
#             return
#         try:
#             x_center_steps = self._center_steps()          # <<< use the exact clamped center
#             y_target_steps = int(self._series_y0_steps)
#             x_cmd, y_cmd = self._clamp_xy_steps(x_center_steps, y_target_steps)
#             feed_xy, feed_y = self._safe_feeds()
#             self._post_sweep_phase = "move_y_back"
#             self.sendData(f'j|{x_cmd}|{y_cmd}|{feed_xy}|{feed_y}')
#         except Exception as e:
#             print("moveYBack error:", e)
                
    def _safe_feeds(self):
        feed_xy = int(self.jog_speed.value())
        feed_y = max(1, int(feed_xy / 3))  # Y slower = safer, like MeasuringWorker does
        return feed_xy, feed_y

    def _clamp_xy_steps(self, x_steps: int, y_steps: int):
        ratio = float(self.shared_data.ratio)
        # Best available limits: X center must be valid, Y is 0..Y_max_from_UI
        # If you have real travel limits in shared_data, clamp to those instead.
        y_steps_min = 0
        y_steps_max = int(self.Y_pos.maximum() * ratio)  # UI max is 105 mm -> steps
        # X: if you have known limits, clamp here. Otherwise leave as-is.
        # Example (uncomment if you have them):
        x_min = int(self.shared_data.x_min_mm * ratio)
        x_max = int(self.shared_data.x_max_mm * ratio)
        x_steps = max(x_min, min(x_steps, x_max))

        y_steps = max(y_steps_min, min(y_steps, y_steps_max))
        return x_steps, y_steps
    
    def _axis_limits_steps(self, x_margin_mm: float = 0.0, y_margin_mm: float = 0.5):
        r = float(self.shared_data.ratio)
        # Read limits (in mm) from shared_data; adjust names if yours differ
        x_min_mm = float(getattr(self.shared_data, "x_min_mm", 0.0))
        x_max_mm = float(getattr(self.shared_data, "x_max_mm", getattr(self.shared_data, "x_center", 0.0)))
        y_min_mm = float(getattr(self.shared_data, "y_min_mm", 0.0))
        y_max_mm = float(getattr(self.shared_data, "y_max_mm", float(self.Y_pos.maximum())))

        # No X margin (per your preference); small Y margin is okay
        xm, ym = float(x_margin_mm), float(y_margin_mm)

        x_min_steps = int(round((x_min_mm + xm) * r))
        x_max_steps = int(round((x_max_mm - xm) * r))
        y_min_steps = int(round((y_min_mm + ym) * r))
        y_max_steps = int(round((y_max_mm - ym) * r))
        return x_min_steps, x_max_steps, y_min_steps, y_max_steps

    def _clamp_xy_steps(self, x_steps: int, y_steps: int):
        x_min, x_max, y_min, y_max = self._axis_limits_steps(x_margin_mm=0.0, y_margin_mm=0.5)
        x_cl = max(x_min, min(int(x_steps), x_max))
        y_cl = max(y_min, min(int(y_steps), y_max))
        return x_cl, y_cl

    def _safe_feeds(self):
        feed_xy = int(self.jog_speed.value())
        feed_y = max(1, int(feed_xy / 3))  # match worker's gentler Y feed
        return feed_xy, feed_y
    
    def _home_steps(self):
        # Home = (Xmin, Ymin) in steps, clamped exactly to HW limits
        x_min, x_max, y_min, y_max = self._axis_limits_steps(x_margin_mm=0.0, y_margin_mm=0.0)
        return x_min, y_min

    def _center_steps(self) -> int:
        """Exact center in steps; prefer the value provided by SetXYAxes."""
        r = float(self.shared_data.ratio)
        s = (int(self._x_center_steps_from_ui)
             if getattr(self, "_x_center_steps_from_ui", None) is not None
             else int(round(float(self.shared_data.x_center) * r)))
        # clamp to HW limits if you have them (no X margin)
        x_min, x_max, _, _ = self._axis_limits_steps(x_margin_mm=0.0, y_margin_mm=0.5)
        return max(x_min, min(s, x_max))
    
    def _offset_steps(self) -> int:
        r = float(self.shared_data.ratio)
        try:
            off_mm = float(getattr(self.shared_data, "probe_offset", 0.0) or 0.0)
        except Exception:
            off_mm = 0.0
        return int(round(off_mm * r))
    
    def _offset_towards_zero_steps(self) -> int:
        """
        Convert shared_data.probe_offset (mm) to steps, always toward X=0.
        We ignore the sign the user entered and move to (center - |offset|).
        """
        try:
            r = float(getattr(self.shared_data, "ratio", 0.0) or 0.0)
        except Exception:
            r = 0.0
        try:
            off_mm = float(getattr(self.shared_data, "probe_offset", 0.0) or 0.0)
        except Exception:
            off_mm = 0.0
        return -int(round(abs(off_mm) * r))
    
    def update_trajectory_label(self):
        if getattr(self, "custom_trajectory", False):
            self.trajectory_mode_label.setText("Mõõtmisrežiim: Trajektoor failist")
            self.trajectory_mode_label.setStyleSheet("color: gray;")
        else:
            self.trajectory_mode_label.setText("Mõõtmisrežiim: Sirge trajektoor")
            self.trajectory_mode_label.setStyleSheet("color: gray;")
#     @pyqtSlot(str)
#     def on_measurement_finished(self, csv_path: str):
#         self.come_back()
#             
#     def process_data(self):
#         return data_processing(self)
        
#     def start_measuring(self):
#         self.measuring_stopped = False
#         self.testMotorButton.setEnabled(False)
#         if not self.measuringThread.isRunning():
#             self.measuringThread.start()
            
#     def process_data(self):
#         plot_filename = f"log{self.today_dt}.png"
#         mean_csvfile = "log" + self.today_dt + "_mean.csv"
#         mean_header = ['#_of_samples ' 'Prop_diam(inch) ' 'X_position(mm) ' 'Y_position(mm) ' 'Torque(Nm) ' 'Thrust(N) ' 
#                        'Airspeed(m/s) ' 'AoA(deg) ' 'AoSS(deg) ' 'V_tan(m/s) ' 'V_rad(m/s) ' 'V_axial(m/s) ' 'Chord_angle(deg) ' 'Chord_angle_eff(deg) '
#                        'Chord_length(mm) ' 'Chord_length_eff(mm) ' 'Helix_angle_eff(deg) ' 'Alpha_angle(deg) ' 'V_total(m/s) ' 'V_lift(m/s) ' 'V_drag(m/s) '
#                        'CL ' 'CD ' 'Reynolds_number ' 'V_a+r(m/s) ' 'D/R_ratio ']       
#         
#         omega_values = []
#         try:
#             with open(os.path.join(self.path, self.csvfile), newline='') as csvfile:
#                 row_read = csv.reader(csvfile, delimiter=' ')
#                 for row in row_read:
#                     line = ' '.join(row)
#                     sample = list(line.split(" "))
#                     try:
#                         omega_value = float(sample[5])
#                         omega_values.append(omega_value)
#                     except (IndexError, ValueError):
#                         continue
#         except FileNotFoundError as e:
#             print(f"Error reading log file: {e}")
#             return
# 
#         if omega_values:
#             try:
#                 omega_mode = statistics.mode(omega_values)
#             except statistics.StatisticsError as e:
#                 print(f"Could not compute mode: {e}. Using mean instead.")
#                 omega_mode = statistics.mean(omega_values)
#         else:
#             print("No omega values found in the log file.")
#             omega_mode = 0
# 
#         
#         with open(os.path.join(self.path,mean_csvfile), 'a') as h:
#             k = csv.writer(h)
#             k.writerow(mean_header)
#         x_max = 3 * math.floor((float(self.radius_mm) + (1 - (self.shared_data.safety_over_prop/100)))/3)
#         x_mp = 0
#         while x_mp <= x_max:
#             try:
#                 with open(os.path.join(self.path,self.csvfile), newline='') as csvfile:
#                     row_read = csv.reader(csvfile, delimiter=' ')
#                     j = 0
#                     mean_counter = 0
#                     testnumber = 0
#                     dict_prop = {}
#                     dict_x = {}
#                     dict_y = {}
#                     dict_trq = {}
#                     dict_thr = {}
#                     dict_omega = {}
#                     dict_arspd = {}
#                     dict_aoa = {}
#                     dict_aoss = {}
#                     dict_v_tan = {}
#                     dict_v_rad = {}
#                     dict_v_axial = {}
#                     for row in row_read:
#                         line = ' '.join(row)
#                         j = j + 1
#                         sample = list(line.split(" "))
#                         if sample[1] == str(x_mp):
#                             dict_prop[mean_counter] = float(sample[0])
#                             dict_x[mean_counter] = int(sample[1])
#                             dict_y[mean_counter] = int(sample[2])
#                             dict_trq[mean_counter] = float(sample[3])
#                             dict_thr[mean_counter] = float(sample[4])
#                             dict_omega[mean_counter] = float(sample[5])
#                             dict_arspd[mean_counter] = float(sample[6])
#                             dict_aoa[mean_counter] = float(sample[7])
#                             dict_aoss[mean_counter] = float(sample[8])
#                             dict_v_tan[mean_counter] = float(sample[9])
#                             dict_v_rad[mean_counter] = float(sample[10])
#                             dict_v_axial[mean_counter] = float(sample[11])
#                             mean_counter = mean_counter + 1
#                             
#                     testnumber = int(len(list(dict_x.values())))
#                     prop = statistics.mean(list(dict_prop.values()))
#                     x_pos = statistics.mean(list(dict_x.values()))
#                     y_pos = statistics.mean(list(dict_y.values()))
#                     trq_mean = format(statistics.mean(list(dict_trq.values())),'.2f')
#                     thr_mean = format(statistics.mean(list(dict_thr.values())),'.2f')
#                     arspd_mean = format(statistics.mean(list(dict_arspd.values())),'.2f')
#                     aoa_mean = format(statistics.mean(list(dict_aoa.values())),'.2f')
#                     aoss_mean = format(statistics.mean(list(dict_aoss.values())),'.2f')
#                     v_tan_mean = format(statistics.mean(list(dict_v_tan.values())),'.2f')
#                     v_rad_mean = format(statistics.mean(list(dict_v_rad.values())),'.2f')
#                     v_axial_mean = format(statistics.mean(list(dict_v_axial.values())),'.2f')
#                     
#                     mean_list = str(str(testnumber)+" "+str(prop)+" "+str(x_pos)+" "+str(y_pos)+" "+str(trq_mean)+" "+str(thr_mean)+" "+str(arspd_mean)+" "+str(aoa_mean)+" "+str(aoss_mean)+" "+str(v_tan_mean)+" "+str(v_rad_mean)+" "+str(v_axial_mean))
#                     
#                     var = format((float(x_pos)/1000)*float(v_axial_mean),'.2f')
#                     var_list.append(float(var))
#                     trq_list.append(float(trq_mean))
#                     thr_list.append(float(thr_mean))
#             except:
#                 pass
#                 
#             with open(self.fname[0], newline = '') as propfile:
#                 read_data = csv.reader(propfile, delimiter=' ')
#                 dict_blade_angle = {}  
#                 for r in read_data:
#                     line2 = ' '.join(r)
#                     sample2 = list(r)
#                     if sample2[0] == str(x_mp):
#                         dict_blade_angle[mean_counter] = sample2[1]
#                         angle_list = sample2[0]+" "+sample2[1]+" "+sample2[2]
#             mean_data = list(mean_list.split(" "))
#             angle_data = list(angle_list.split(" "))
#             if mean_data[2] == angle_data[0]:
#                 chord_angle_raw = angle_data[1]
#                 chord_length_raw = angle_data[2]
#                 
#                 denominator1 = (float(omega_mode)*float(x_pos/1000)-float(v_tan_mean))
#                 if denominator1 == 0:
#                     chord_angle = 0.0
#                     chord_length = 0.0
#                 else:
#                     denominator2 = math.cos(math.atan(float(v_rad_mean)/denominator1))
#                     vs1 = math.tan(math.radians(float(chord_angle_raw)))
#                     vs2 = vs1*denominator2
#                     chord_angle = math.degrees(math.atan(vs2))
#                     chord_length = float(chord_length_raw)/math.cos(math.atan(float(v_rad_mean)/denominator1))
#             else:
#                 chord_angle_raw = 0.0
#                 chord_angle = 0.0
#                 chord_length_raw = 0.0
#                 chord_length = 0.0
#             mean_data.append(str(chord_angle_raw))
#             mean_data.append(str(format(chord_angle,'.2f')))
#             mean_data.append(str(chord_length_raw))
#             mean_data.append(str(format(chord_length,'.2f')))
#             total_speed = math.sqrt(math.pow(((float(omega_mode)*float(x_pos/1000))-float(v_tan_mean)),2)+math.pow(float(v_axial_mean),2)+math.pow(float(v_rad_mean),2))
#             try:
#                 helix_angle = math.degrees(math.asin(float(v_axial_mean)/float(total_speed)))
#             except:
#                 helix_angle = 0
#             mean_data.append(str(format(helix_angle,'.2f')))
#             alpha_angle = format(float(chord_angle) - helix_angle,'.2f')
#             mean_data.append(str(alpha_angle))
#             mean_data.append(str(format(total_speed,'.2f'))) 
#             v_lift = (float(v_axial_mean) * math.cos(math.radians(float(helix_angle))) + (float(v_tan_mean) * math.sin(math.radians(float(helix_angle)))))
#             mean_data.append(str(format(v_lift,'.2f')))
#             v_drag = (float(v_tan_mean) * math.cos(math.radians(float(helix_angle))) - (float(v_axial_mean) * math.sin(math.radians(float(helix_angle)))))
#             mean_data.append(str(format(v_drag,'.2f')))
#             try:
#                 coeff_lift = (2 * float(v_lift))/float(total_speed)
#             except:
#                 coeff_lift = 0
#             mean_data.append(str(format(coeff_lift,'.3f')))
#             try:
#                 coeff_drag = (2 * float(v_drag))/float(total_speed)
#             except:
#                 coeff_drag = 0
#             mean_data.append(str(format(coeff_drag,'.3f')))
#             Re = (float(chord_length)/1000 * float(total_speed))/(float(self.shared_data.kin_visc) * math.pow(10,-5))
#             mean_data.append(str(format(Re,'.0f')))
#             v_2d = math.sqrt(math.pow(float(v_axial_mean),2) + math.pow(float(v_rad_mean),2))
#             mean_data.append(str(format(v_2d,'.2f')))
#             mean_data.append(str(format(self.dr_ratio.value(),'.1f')))
# #             diff_mass_rate = math.pi*self.shared_data.rho*float(v_axial_mean)*(self.radius_mm/1000)
# #             mean_data.append(str(format(diff_mass_rate,'.2f')))
# 
#             with open(os.path.join(self.path,mean_csvfile), 'a') as f:
#                 w = csv.writer(f)
#                 w.writerow([' '.join(mean_data)])
#             
#             self.update_plot_ax2(x_pos, v_tan_mean, v_rad_mean, v_axial_mean)
#             x_mp = x_mp + 3
#             
#         vi = (2*(self.shared_data.x_delta/1000)*sum(var_list))/math.pow(float(self.radius_mm)/1000,2)
#         self.label13.setText(str(format(vi,'.2f')))
#         T = statistics.mean(thr_list)
#         Pi = float(vi)*float(T)
#         self.label15.setText(str(format(Pi,'.2f')))
#         try:
#             vv = float(T)/(self.shared_data.rho * math.pi * math.pow(float(self.radius_mm)/1000,2) * math.pow(float(vi),2))
#         except:
#             vv = 0
#         self.label65.setText(str(format(vv,'.2f')))
#         M = statistics.mean(trq_list)
#         P = float(M)*float(omega_mode)
#         self.label17.setText(str(format(P,'.2f')))
#         vm = self.shared_data.rho * math.pi * math.pow(float(self.radius_mm)/1000,2) * float(vi)
#         self.label67.setText(str(format(vm,'.2f')))
#         try:
#             v_max_mean = float(T)/float(vm)
#         except:
#             v_max_mean = 0
#         self.label69.setText(str(format(v_max_mean,'.2f')))
#         try:
#             Ct = float(T)/(self.shared_data.rho * math.pow(float(omega_mode),2) * math.pow(((float(self.radius_mm)*2)/1000),4))
#         except:
#             Ct = 0
#         try:
#             Cp = float(M)/(self.shared_data.rho * math.pow(float(omega_mode),2) * math.pow(((float(self.radius_mm)*2)/1000),5))
#         except:
#             Cp = 0
#         try:
#             nu = (Pi/P)*100
#             self.label19.setText(str(format(nu,'.2f')))
#         except:
#             nu = -1
#             self.label19.setText(str(nu))
#         
#         with open(os.path.join(self.path,mean_csvfile), 'a') as f:
#             w = csv.writer(f, delimiter=' ')
#             w.writerow(['Omega',format(float(omega_mode),'.2f'),'rad/s'])
#             w.writerow(['Induced_power',format(Pi,'.2f'),'W'])
#             w.writerow(['Power',format(P,'.2f'),'W'])
#             w.writerow(['Efficiency',format(nu,'.2f'),'%'])
#             w.writerow(['Average_induced_speed',format(vi,'.2f'),'m/s'])
#             w.writerow(['Airspeed_ratio',format(vv,'.2f')])
#             w.writerow(['V_mass',format(vm,'.2f'),'kg/s'])
#             w.writerow(['V_max_mean',format(v_max_mean,'.2f'),'m/s'])
#             w.writerow(['Ct',format(Ct,'.7f')])
#             w.writerow(['Cp',format(Cp,'.7f')])
#             w.writerow(['Air_density',self.shared_data.rho,'kg/m3'])
#             w.writerow(['Air_kinematic_viscosity',self.shared_data.kin_visc,'x10-5 m2/s'])
#         
#         var_list.clear()
#         trq_list.clear()
#         thr_list.clear()
#         self.counter = 0
#         self.cnv.draw_ax2()
#         
#         self.cnv.save_only_second_plot(os.path.join(self.path, plot_filename))

    def calculate_CT_show(self):
        self.calculate_CT.show()
