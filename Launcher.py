#!/usr/bin/env python3
import sys
import time
import datetime
import serial
import serial.tools.list_ports
import os
import csv
import math
import statistics
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from queue import *
from matplotlib import pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from pathlib import Path
import numpy as np

#default values used on startup
rho_default = 1.225 #kg/cm3 standard air density
kin_visc_default = 1.48
ratio_default = 24.9955 #stepper drive ratio one revolution/distance travelled in mm
x_center_default = 324 #in mm, center position of the probe from home
y_max_default = 107 #in mm, max travel from home
safety_over_prop_default = 2 # in % from diameter of how much extra travel after propeller tip to reduce the risk of collision of probe to prop
max_number_of_samples_default = 10 #increase it if needed to do more samples
x_delta_default = 3 #measurement resolution in mm (half of the diameter of the Pitot' tube)
arm_length_default = 72.0 #torque arm length in mm
x_max_speed_default = 1800
y_max_speed_default = 1800
x_max_accel_default = 1000
y_max_accel_default = 800
aoa_trim_default = 79
aoa_limit_default = 50
aoss_trim_default = 91
aoss_max_limit_default = 44
aoss_min_limit_default = 94
g_const = 9.8066520482
min_pwm_default = 1000
max_pwm_default = 2000

var_list = []
trq_list = []
thr_list = []
omega_list = []
list_of_x_targets = []
list_of_y_targets = []
list_of_y_abs = []

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    filtered_ports = [port.device for port in ports if 'ACM' in port.device or 'USB' in port.device]
    return filtered_ports

class Canvas(FigureCanvasQTAgg):

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        super().__init__(self.fig)
        self.ax.grid(True)
        self.plot_lines = {}

    def add_data(self, x, y, label, style):
        if label in self.plot_lines:
            # Update existing line data
            line = self.plot_lines[label]
            line.set_xdata(np.append(line.get_xdata(), x))
            line.set_ydata(np.append(line.get_ydata(), y))
        else:
            # Create a new line
            self.plot_lines[label], = self.ax.plot(x, y, style, label=label)
            self.ax.legend()

        # Redraw the canvas
        self.ax.relim()
        self.ax.autoscale_view(True, True, True)
        self.draw_idle()
        #print(f"Added data to plot: {label}, x={x}, y={y}")

    def clear_plot(self):
        self.ax.clear()
        self.ax.grid(True)
        self.plot_lines.clear()
        self.ax.legend()
        self.draw_idle()
        print("Plot cleared")

    def save_plot(self, filename):
        self.fig.savefig(filename)
        print(f"Plot saved as {filename}")

class SetParameters(QWidget):
    sendData = pyqtSignal(str)
    
    def __init__(self, shared_data):
        super().__init__()
        
        self.shared_data = shared_data
        self.trq_lc = 'L'
        
        layout1 = QVBoxLayout()
        self.setLayout(layout1)
        
        self.setWindowTitle("Parameetrite sättimine")
        
        self.label30 = QLabel("Stepperite ülekandearvu (sammu/mm)")
        layout1.addWidget(self.label30)
        
        self.ratio = QDoubleSpinBox()
        self.ratio.setMinimum(0)
        self.ratio.setDecimals(4)
        self.ratio.setSingleStep(0.0001)
        self.ratio.setValue(self.shared_data.ratio)
        layout1.addWidget(self.ratio)
        
        self.label31 = QLabel("Õhutihedus (kg/m3)")
        layout1.addWidget(self.label31)
        
        self.rho = QDoubleSpinBox()
        self.rho.setMinimum(0)
        self.rho.setDecimals(3)
        self.rho.setSingleStep(0.001)
        self.rho.setValue(self.shared_data.rho)
        layout1.addWidget(self.rho)
        
        self.label32 = QLabel("Kinemaatiline viskoossus (m2/s)")
        layout1.addWidget(self.label32)
        
        self.kin_visc = QDoubleSpinBox()
        self.kin_visc.setMinimum(0)
        self.kin_visc.setDecimals(2)
        self.kin_visc.setSingleStep(0.01)
        self.kin_visc.setValue(self.shared_data.kin_visc)
        layout1.addWidget(self.kin_visc)
        
        self.label33 = QLabel("Turvaala suurus (% propelleri diameetrist)")
        layout1.addWidget(self.label33)
        
        self.safety_over_prop = QSpinBox()
        self.safety_over_prop.setMinimum(1)
        self.safety_over_prop.setMaximum(10)
        self.safety_over_prop.setSingleStep(1)
        self.safety_over_prop.setValue(self.shared_data.safety_over_prop)
        layout1.addWidget(self.safety_over_prop)
        
        self.label34 = QLabel("Mõõtmise resolutsioon (mm)")
        layout1.addWidget(self.label34)
        
        self.x_delta = QSpinBox()
        self.x_delta.setMinimum(1)
        self.x_delta.setMaximum(20)
        self.x_delta.setSingleStep(1)
        self.x_delta.setValue(self.shared_data.x_delta)
        layout1.addWidget(self.x_delta)
        
        self.label35 = QLabel("Maksimaalne mõõtmiste arv")
        layout1.addWidget(self.label35)
        
        self.max_number_of_samples = QSpinBox()
        self.max_number_of_samples.setMinimum(1)
        self.max_number_of_samples.setMaximum(30)
        self.max_number_of_samples.setSingleStep(1)
        self.max_number_of_samples.setValue(self.shared_data.max_number_of_samples)
        layout1.addWidget(self.max_number_of_samples)
        
        self.label42 = QLabel("Pöördemomendi õla pikkus (mm)")
        layout1.addWidget(self.label42)
        
        self.arm_length = QDoubleSpinBox()
        self.arm_length.setMinimum(0.00)
        self.arm_length.setSingleStep(0.01)
        self.arm_length.setValue(self.shared_data.arm_length)
        layout1.addWidget(self.arm_length)
        
        self.label42 = QLabel("Vasaku koormusanduri koefitsient")
        layout1.addWidget(self.label42)
        
        self.left_lc = QDoubleSpinBox()
        self.left_lc.setMinimum(-sys.float_info.max)
        self.left_lc.setMaximum(sys.float_info.max)
        self.left_lc.setSingleStep(0.01)
        self.left_lc.setValue(-897.59)
        layout1.addWidget(self.left_lc)
        
        self.label43 = QLabel("Parema koormusanduri koefitsient")
        layout1.addWidget(self.label43)
        
        self.right_lc = QDoubleSpinBox()
        self.right_lc.setMinimum(-sys.float_info.max)
        self.right_lc.setMaximum(sys.float_info.max)
        self.right_lc.setSingleStep(0.01)
        self.right_lc.setValue(-291.75) #936.54
        layout1.addWidget(self.right_lc)
        
        # Radio buttons for three positions
        self.radio1 = QRadioButton("Vasak andur kasutuses")
        self.radio2 = QRadioButton("Parem andur kasutuses")
        self.radio3 = QRadioButton("Ei mõõda pöördemomenti")
        self.radio1.setChecked(True)  # Default position

        # Button group for exclusivity
        self.buttonGroup = QButtonGroup(self)
        self.buttonGroup.addButton(self.radio1, 1)
        self.buttonGroup.addButton(self.radio2, 2)
        self.buttonGroup.addButton(self.radio3, 3)

        layout1.addWidget(self.radio1)
        layout1.addWidget(self.radio2)
        layout1.addWidget(self.radio3)

        self.buttonGroup.buttonClicked[int].connect(self.updateLabel)
        
        self.label44 = QLabel("Tõmbe koormusanduri koefitsient")
        layout1.addWidget(self.label44)
        
        self.thr_lc = QDoubleSpinBox()
        self.thr_lc.setMinimum(-sys.float_info.max)
        self.thr_lc.setMaximum(sys.float_info.max)
        self.thr_lc.setSingleStep(0.01)
        self.thr_lc.setValue(301.27) #878.00
        layout1.addWidget(self.thr_lc)
        
        self.confirm_button = QPushButton("Kinnita algparameetrid", self)
        self.confirm_button.clicked.connect(self.confirm_changes)
        #self.confirm_button.setEnabled(False)  # Initially disabled
        layout1.addWidget(self.confirm_button)
        
        self.label36 = QLabel("X-teljes mootori tsentri koordinaat (max käik) (mm)")
        layout1.addWidget(self.label36)
        
        self.x_center = QDoubleSpinBox()
        self.x_center.setMinimum(1)
        self.x_center.setMaximum(330)
        self.x_center.setSingleStep(0.1)
        self.x_center.setValue(self.shared_data.x_center)
        layout1.addWidget(self.x_center)
        
        self.label37 = QLabel("Y-telje maksimaalne käik (mm)" )
        layout1.addWidget(self.label37)
        
        self.y_max = QSpinBox()
        self.y_max.setMinimum(1)
        self.y_max.setMaximum(107)
        self.y_max.setSingleStep(1)
        self.y_max.setValue(self.shared_data.y_max)
        layout1.addWidget(self.y_max)
        
        self.label38 = QLabel("X-telje maksimaalne kiirus (sammu/s)" )
        layout1.addWidget(self.label38)
        
        self.x_max_speed = QSpinBox()
        self.x_max_speed.setMinimum(100)
        self.x_max_speed.setMaximum(2000)
        self.x_max_speed.setSingleStep(100)
        self.x_max_speed.setValue(self.shared_data.x_max_speed)
        layout1.addWidget(self.x_max_speed)
        
        self.label39 = QLabel("Y-telje maksimaalne kiirus (sammu/s)" )
        layout1.addWidget(self.label39)
        
        self.y_max_speed = QSpinBox()
        self.y_max_speed.setMinimum(100)
        self.y_max_speed.setMaximum(2000)
        self.y_max_speed.setSingleStep(100)
        self.y_max_speed.setValue(self.shared_data.y_max_speed)
        layout1.addWidget(self.y_max_speed)
        
        self.label40 = QLabel("X-telje maksimaalne kiirendus (sammu/s/s)" )
        layout1.addWidget(self.label40)
        
        self.x_max_accel = QSpinBox()
        self.x_max_accel.setMinimum(50)
        self.x_max_accel.setMaximum(1500)
        self.x_max_accel.setSingleStep(100)
        self.x_max_accel.setValue(self.shared_data.x_max_accel)
        layout1.addWidget(self.x_max_accel)
        
        self.label41 = QLabel("Y-telje maksimaalne kiirendus (sammu/s/s)" )
        layout1.addWidget(self.label41)
        
        self.y_max_accel = QSpinBox()
        self.y_max_accel.setMinimum(50)
        self.y_max_accel.setMaximum(1500)
        self.y_max_accel.setSingleStep(100)
        self.y_max_accel.setValue(self.shared_data.y_max_accel)
        layout1.addWidget(self.y_max_accel)
        
        self.confirm_axis_button = QPushButton("Kinnita telgede parameetrid", self)
        self.confirm_axis_button.clicked.connect(self.confirm_axis_changes)
        self.confirm_axis_button.setEnabled(False)  # Initially disabled
        layout1.addWidget(self.confirm_axis_button)

        # Connect signals
        self.ratio.valueChanged.connect(self.enable_confirm_button)
        self.rho.valueChanged.connect(self.enable_confirm_button)
        self.kin_visc.valueChanged.connect(self.enable_confirm_button)
        self.safety_over_prop.valueChanged.connect(self.enable_confirm_button)
        self.x_delta.valueChanged.connect(self.enable_confirm_button)
        self.max_number_of_samples.valueChanged.connect(self.enable_confirm_button)
        self.arm_length.valueChanged.connect(self.enable_confirm_button)
        self.left_lc.valueChanged.connect(self.enable_confirm_button)
        self.right_lc.valueChanged.connect(self.enable_confirm_button)
        self.thr_lc.valueChanged.connect(self.enable_confirm_button)
        self.radio1.toggled.connect(self.enable_confirm_button)
        self.radio2.toggled.connect(self.enable_confirm_button)
        self.radio3.toggled.connect(self.enable_confirm_button)
        
        self.x_center.valueChanged.connect(self.enable_confirm_axis_button)
        self.y_max.valueChanged.connect(self.enable_confirm_axis_button)
        self.x_max_speed.valueChanged.connect(self.enable_confirm_axis_button)
        self.y_max_speed.valueChanged.connect(self.enable_confirm_axis_button)
        self.x_max_accel.valueChanged.connect(self.enable_confirm_axis_button)
        self.y_max_accel.valueChanged.connect(self.enable_confirm_axis_button)
        
    def enable_confirm_axis_button(self):
        self.confirm_axis_button.setEnabled(True)
        self.confirm_axis_button.setStyleSheet("background-color: orange; color: black;")
        
    def enable_confirm_button(self):
        self.confirm_button.setEnabled(True)
        self.confirm_button.setStyleSheet("background-color: orange; color: black;")
    
    def updateLabel(self, id):
        if (id == 1):
            self.trq_lc = 'L'
        if (id == 2):
            self.trq_lc = 'R'
        if (id == 3):
            self.trq_lc = '0'
        
    def confirm_changes(self):
        try:
            self.shared_data.ratio = self.ratio.value()
            self.shared_data.rho = self.rho.value()
            self.shared_data.kin_visc = self.kin_visc.value()
            self.shared_data.safety_over_prop = self.safety_over_prop.value()
            self.shared_data.x_delta = self.x_delta.value()
            self.shared_data.max_number_of_samples = self.max_number_of_samples.value()
            self.shared_data.arm_length = self.arm_length.value()
            init_data = 'init|%.2f|%.2f|%.2f|%.2f|%s' % (self.shared_data.arm_length, self.left_lc.value(), self.right_lc.value(), self.thr_lc.value(), self.trq_lc)
            self.sendData.emit(init_data)
            self.confirm_button.setEnabled(False)
            self.confirm_button.setStyleSheet("background-color: None; color: None;")
            self.confirm_axis_button.setEnabled(True)
            window.aoa_aoss_action.setEnabled(True)
            window.calibrate_loadcells_action.setEnabled(True)
            window.rpm_controller_action.setEnabled(True)
            window.map_action.setEnabled(True)
            window.clear_plot_action.setEnabled(True)
            window.save_plot_action.setEnabled(True)
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
    
    def confirm_axis_changes(self):
        try:
            self.shared_data.x_center = self.x_center.value()
            self.shared_data.y_max = self.y_max.value()
            self.shared_data.x_max_speed = self.x_max_speed.value()
            self.shared_data.y_max_speed = self.y_max_speed.value()
            self.shared_data.x_max_accel = self.x_max_accel.value()
            self.shared_data.y_max_accel = self.y_max_accel.value()
            limit_data = 'l|%d|%d|%d|%d|%d|%d' % ((self.shared_data.x_center * self.shared_data.ratio), (self.shared_data.y_max * self.shared_data.ratio), self.shared_data.x_max_speed, self.shared_data.y_max_speed, self.shared_data.x_max_accel, self.shared_data.y_max_accel)
            self.sendData.emit(limit_data)
            self.confirm_axis_button.setEnabled(False)
            self.confirm_axis_button.setStyleSheet("background-color: None; color: None;")
            window.file.setEnabled(True)
            QTimer.singleShot(1000, self.close)
            window.params.setStyleSheet("background-color: green; color: white;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
        
class MapTrajectory(QWidget):
    sendData = pyqtSignal(str)
    
    def __init__(self, shared_data):
        super().__init__()
        
        self.shared_data = shared_data
        
        layout2 = QVBoxLayout()
        
        self.setWindowTitle("Trajektoori kaardistamine")
        
        self.label3 = QLabel("1. Liiguta Pitot' Y-telg korrektsele kaugusele")
        layout2.addWidget(self.label3)
        
        self.label4 = QLabel("2. Salvesta algse punkti koordinaadid")
        layout2.addWidget(self.label4)
        
        self.label5 = QLabel("3. Liiguta Pitot' järgmisesse mõõtepunkti")
        layout2.addWidget(self.label5)
        
        self.label6 = QLabel("4. Salvesta koordinaadid")
        layout2.addWidget(self.label6)
        
        self.label7 = QLabel("5. Kui trajektoor on kaardistatud, siis sulge aken")
        layout2.addWidget(self.label7)
        
        self.label8 = QLabel(" " )
        layout2.addWidget(self.label8)
        
        self.centering = QPushButton("Pitot' tsentrisse")
        self.centering.setEnabled(True)
        self.centering.clicked.connect(window.center)
        self.centering.setStyleSheet("background-color: None; color: None;")
        layout2.addWidget(self.centering)
        
        self.label10 = QLabel("X-telje positsioon mm")
        layout2.addWidget(self.label10)
        
        self.X_pos_map = QDoubleSpinBox()
        self.X_pos_map.setMinimum(0)
        self.X_pos_map.setMaximum(self.shared_data.x_center)
        self.X_pos_map.setSingleStep(3)
        layout2.addWidget(self.X_pos_map)
        
        self.label9 = QLabel("Y-telje positsioon mm")
        layout2.addWidget(self.label9)
        
        self.Y_pos_map = QSpinBox()
        self.Y_pos_map.setMinimum(0)
        self.Y_pos_map.setMaximum(self.shared_data.y_max)
        layout2.addWidget(self.Y_pos_map)
        
        self.sensor_move_map = QPushButton("Liiguta Pitot' sensorit")
        self.sensor_move_map.clicked.connect(self.move_sensor_map)
        layout2.addWidget(self.sensor_move_map)
        
        self.save = QPushButton("Salvesta koordinaadid mällu")
        self.save.clicked.connect(self.save_xy)
        layout2.addWidget(self.save)
        
        self.space = QLabel(" " )
        layout2.addWidget(self.space)
        
        self.wp_file = QPushButton("Salvesta trajektoori fail")
        self.wp_file.clicked.connect(self.save_wp_file)
        layout2.addWidget(self.wp_file)
        
        self.wp_load = QPushButton("Otsi olemasolev trajektoori fail")
        self.wp_load.clicked.connect(self.load_wp_file)
        layout2.addWidget(self.wp_load)
        
        self.delete = QPushButton("Kustuta trajektoor mälust")
        self.delete.clicked.connect(self.delete_wp)
        layout2.addWidget(self.delete)
        
        self.x_target = int(round((self.shared_data.x_center - self.X_pos_map.value()) * self.shared_data.ratio, 0))
        self.current_Y = int(round(self.Y_pos_map.value() * self.shared_data.ratio, 0))
        
        self.setLayout(layout2)

        self.r = 0
        
        self.X_pos_map.valueChanged.connect(self.enable_move_sensor_button)
        self.Y_pos_map.valueChanged.connect(self.enable_move_sensor_button)
        
    def enable_move_sensor_button(self):
        self.sensor_move_map.setStyleSheet("background-color: orange; color: black;")
        
    def delete_wp(self):
        list_of_x_targets.clear()
        list_of_y_targets.clear()
        list_of_y_abs.clear()
        self.r = 0
        window.custom_trajectory = False
        
    def save_xy(self):
        self.save.setStyleSheet("background-color: None; color: None;")
        window.custom_trajectory = True
        first_y = 0
        if self.x_target not in list_of_x_targets:
            list_of_x_targets.append(self.x_target)
            list_of_y_abs.append(self.current_Y)
            first_y = list_of_y_abs[0]
            list_of_y_targets.append(list_of_y_abs[self.r] - first_y)
            self.r = self.r + 1
            
    def move_sensor_map(self):
        self.sensor_move_map.setStyleSheet("background-color: None; color: None;")
        self.x_target = int((self.shared_data.x_center - self.X_pos_map.value()) * self.shared_data.ratio)
        self.current_Y = int(self.Y_pos_map.value() * self.shared_data.ratio)
        self.sendData.emit('j|%d|%d|%d|%d' % (self.x_target, self.current_Y, window.jog_speed.value(),window.jog_speed.value()))
        self.save.setStyleSheet("background-color: orange; color: black;")
        
    def save_wp_file(self):
        l = 0
        dialog = QFileDialog()
        dialog.setDefaultSuffix(".csv")
        traj_file = dialog.getSaveFileName(self, 'Salvesta trajektoori fail')
        if traj_file[0]:
            file_path = traj_file[0]
            if not file_path.lower().endswith('.csv'):
                file_path += '.csv'
            while l < len(list_of_x_targets):
                coords = str(list_of_x_targets[l]) + ' ' + str(list_of_y_targets[l])
                l = l + 1
                with open(file_path, 'a') as f:
                    w = csv.writer(f)
                    w.writerow([coords])
                    
    def load_wp_file(self):
        home_dir = str(Path.home())
        dialog =QFileDialog()
        wp_file = dialog.getOpenFileName(self, 'Otsi trajektoori fail', home_dir, "csv(*.csv)")
        try:
            if wp_file:
                list_of_x_targets.clear()
                list_of_y_targets.clear()
                list_of_y_abs.clear()
                window.custom_trajectory = True
                with open(wp_file[0], newline = '') as wpfile:
                    read_wps = csv.reader(wpfile, delimiter = ' ')
                    for wp in read_wps:
                       #wp_line = ' '.join(wp)
                        sample_wp = list(wp)
                        list_of_x_targets.append(int(sample_wp[0]))
                        list_of_y_targets.append(int(sample_wp[1]))
                        print(list_of_x_targets, list_of_y_targets)
        except:
            window.custom_trajectory = False
            pass
            
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

        # Connect signals
        self.aoa_trim.valueChanged.connect(self.enable_aoa_trim_button)
        self.aoa_limit.valueChanged.connect(self.enable_aoa_limit_button)
        self.aoss_trim.valueChanged.connect(self.enable_aoss_trim_button)
        self.aoss_max_limit.valueChanged.connect(self.enable_aoss_limit_button)
        self.aoss_min_limit.valueChanged.connect(self.enable_aoss_limit_button)
        
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
            
class RPM_controller(QWidget):
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
            
class LC_calibration(QWidget):
    sendData = pyqtSignal(str)
    updateCalFactor = pyqtSignal(str)
    
    def __init__(self, shared_data):
        super().__init__()
        self.cal_factor = QLabel()
        self.wait_for_cal_factor = False
        self.serialreader = None  # Initialized later
        self.shared_data = shared_data
        self.cal_val_received = False
        self.lc_test = False
        self.thrust_test_value = 0
        self.weight_test_value = 0
        self.torque_test_value = 0
        
        layout4 = QVBoxLayout()
        self.setLayout(layout4)
        
        self.setWindowTitle("Koormusandurite kalibreerimine")
        
        self.label54 = QLabel("1. Kalibreeritav koormusandur ei tohi olla koormatud!")
        layout4.addWidget(self.label54)
        
        self.label55 = QLabel("2. Aseta kalibreeritav koormusandur selliselt, et koormusandurile saaks rakendada koormust")
        layout4.addWidget(self.label55)
        
        self.label56 = QLabel("3. Vali kalibreeritav koormusandur")
        layout4.addWidget(self.label56)
        
        self.cal_left_button = QPushButton("Vasaku koormusanduri kalibreerimine", self)
        self.cal_left_button.clicked.connect(self.send_cal_left)
        layout4.addWidget(self.cal_left_button)
        
        self.cal_right_button = QPushButton("Parema koormusanduri kalibreerimine", self)
        self.cal_right_button.clicked.connect(self.send_cal_right)
        layout4.addWidget(self.cal_right_button)
        
        self.cal_thrust_button = QPushButton("Tõmbe koormusanduri kalibreerimine", self)
        self.cal_thrust_button.clicked.connect(self.send_cal_thrust)
        layout4.addWidget(self.cal_thrust_button)
        
        self.label57 = QLabel("4. Oota ca 2 sekundit ja aseta tuntud mass koormusandurile")
        layout4.addWidget(self.label57)
        
        self.label58 = QLabel("5. Sisesta mass (grammides)")
        layout4.addWidget(self.label58)
        
        self.known_mass = QDoubleSpinBox()
        self.known_mass.setMinimum(0.01)
        self.known_mass.setMaximum(sys.float_info.max)
        self.known_mass.setSingleStep(0.01)
        layout4.addWidget(self.known_mass)
        
        self.known_mass_button = QPushButton("Kinnita mass (grammides)", self)
        self.known_mass_button.clicked.connect(self.send_known_mass)
        layout4.addWidget(self.known_mass_button)
        
        self.label59 = QLabel("6. Märgi kalibratsioonifaktor koormusanduri peale")
        layout4.addWidget(self.label59)
        
        self.label60 = QLabel("Kalibratsioonifaktor:")
        layout4.addWidget(self.label60)
        
        layout4.addWidget(self.cal_factor)
        
        self.cal_factor.setText('NIL')
        
        self.label64 = QLabel("7. Salvesta kalibratsioonifaktor")
        layout4.addWidget(self.label64)
        
        self.label65 = QLabel("Vasaku koormusanduri kal. faktor")
        layout4.addWidget(self.label65)
        
        self.left_cal_val = QDoubleSpinBox()
        self.left_cal_val.setMinimum(-999999.00)
        self.left_cal_val.setMaximum(999999.00)
        self.left_cal_val.setSingleStep(0.01)
        layout4.addWidget(self.left_cal_val)
        
        self.left_cal_val_button = QPushButton("Salvesta vasaku koormusanduri kal. faktor", self)
        self.left_cal_val_button.clicked.connect(self.send_left_cal_val)
        layout4.addWidget(self.left_cal_val_button)
        
        self.label66 = QLabel("Parema koormusanduri kal. faktor")
        layout4.addWidget(self.label66)
        
        self.right_cal_val = QDoubleSpinBox()
        self.right_cal_val.setMinimum(-999999.00)
        self.right_cal_val.setMaximum(999999.00)
        self.right_cal_val.setSingleStep(0.01)
        layout4.addWidget(self.right_cal_val)
        
        self.right_cal_val_button = QPushButton("Salvesta parema koormusanduri kal. faktor", self)
        self.right_cal_val_button.clicked.connect(self.send_right_cal_val)
        layout4.addWidget(self.right_cal_val_button)
        
        self.label67 = QLabel("Tõmbe koormusanduri kal. faktor")
        layout4.addWidget(self.label67)
        
        self.thrust_cal_val = QDoubleSpinBox()
        self.thrust_cal_val.setMinimum(-999999.00)
        self.thrust_cal_val.setMaximum(999999.00)
        self.thrust_cal_val.setSingleStep(0.01)
        layout4.addWidget(self.thrust_cal_val)
        
        self.thrust_cal_val_button = QPushButton("Salvesta tõmbe koormusanduri kal. faktor", self)
        self.thrust_cal_val_button.clicked.connect(self.send_thrust_cal_val)
        layout4.addWidget(self.thrust_cal_val_button)
        
        self.label68 = QLabel("Pöördemomendi jõuõla pikkus")
        layout4.addWidget(self.label68)
        
        self.arm_length = QDoubleSpinBox()
        self.arm_length.setMinimum(0.00)
        self.arm_length.setSingleStep(0.01)
        self.arm_length.setValue(self.shared_data.arm_length)
        layout4.addWidget(self.arm_length)
        
        self.arm_length_button = QPushButton("Muuda jõuõla pikkust", self)
        self.arm_length_button.clicked.connect(self.send_arm_length)
        layout4.addWidget(self.arm_length_button)
        
        self.lc_test_button = QPushButton("Testi koormusandureid", self)
        self.lc_test_button.setCheckable(True)
        self.lc_test_button.clicked.connect(self.toggle_test)
        layout4.addWidget(self.lc_test_button)
        
        self.tare_button = QPushButton("Nulli koormusandurid", self)
        self.tare_button.clicked.connect(self.tare)
        layout4.addWidget(self.tare_button)
        
        # Radio buttons for three positions
        self.radio1 = QRadioButton("Vasak andur")
        self.radio2 = QRadioButton("Parem andur")
        self.radio3 = QRadioButton("Ei mõõda pöördemomenti")
        self.radio1.setChecked(True)  # Default position

        # Button group for exclusivity
        self.buttonGroup = QButtonGroup(self)
        self.buttonGroup.addButton(self.radio1, 1)
        self.buttonGroup.addButton(self.radio2, 2)
        self.buttonGroup.addButton(self.radio3, 3)

        layout4.addWidget(self.radio1)
        layout4.addWidget(self.radio2)
        layout4.addWidget(self.radio3)
        
        self.buttonGroup.buttonClicked[int].connect(self.change_trq_lc)
        
        self.label61 = QLabel("Tõmbe koormusanduri lugem (N):")
        layout4.addWidget(self.label61)
        
        self.thrust_lc_reading = QLabel()
        layout4.addWidget(self.thrust_lc_reading)
        
        self.labelThr_grams = QLabel("Tõmbe koormusanduri lugem (g):")
        layout4.addWidget(self.labelThr_grams)
        
        self.thrust_lc_reading_grams = QLabel()
        layout4.addWidget(self.thrust_lc_reading_grams)
        
        self.label62 = QLabel("Pöördemomendi koormusanduri lugem (g):")
        layout4.addWidget(self.label62)
        
        self.weight_lc_reading = QLabel()
        layout4.addWidget(self.weight_lc_reading)
        
        self.label63 = QLabel("Pöördemomendi koormusanduri lugem (Nm):")
        layout4.addWidget(self.label63)
        
        self.torque_lc_reading = QLabel()
        layout4.addWidget(self.torque_lc_reading)
        
        self.timer_test = QTimer(self)
        self.timer_test.timeout.connect(self.update_data)
        
        self.timer_cal = QTimer(self)
        self.timer_cal.timeout.connect(self.update_cal_factor_label)
        
    def initialize(self, shared_data):
        self.shared_data = shared_data
        self.serialreader = SerialReader(shared_data)
        
    def send_cal_left(self):
        try:
            cal_left_data = 'calLeft'
            self.sendData.emit(cal_left_data)
            self.cal_left_button.setEnabled(False)
            self.cal_right_button.setEnabled(False)
            self.cal_thrust_button.setEnabled(False)
            window.cal_val_received = False
            self.cal_left_button.setStyleSheet("background-color: yellow; color: black;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            
    def send_cal_right(self):
        try:
            cal_right_data = 'calRight'
            self.sendData.emit(cal_right_data)
            self.cal_right_button.setEnabled(False)
            self.cal_left_button.setEnabled(False)
            self.cal_thrust_button.setEnabled(False)
            window.cal_val_received = False
            self.cal_right_button.setStyleSheet("background-color: yellow; color: black;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
    
    def send_cal_thrust(self):
        try:
            cal_thrust_data = 'calThrust'
            self.sendData.emit(cal_thrust_data)
            self.cal_thrust_button.setEnabled(False)
            self.cal_left_button.setEnabled(False)
            self.cal_right_button.setEnabled(False)
            window.cal_val_received = False
            self.cal_thrust_button.setStyleSheet("background-color: yellow; color: black;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            
    def send_known_mass(self):
        try:
            self.timer_cal.start(500)
            known_mass_data = 'calMass|%.2f' %(self.known_mass.value())
            self.sendData.emit(known_mass_data)
            self.known_mass_button.setStyleSheet("background-color: yellow; color: black;")
            self.known_mass_button.setEnabled(False)
            self.wait_for_cal_factor = True
            self.cal_val_received = False
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            
    def update_cal_factor_label(self):
        cal = window.cal_value
        print(cal)
        self.cal_factor.setText(f"{cal}")
        if (cal != 0):
            self.cal_left_button.setEnabled(True)
            self.cal_right_button.setEnabled(True)
            self.cal_thrust_button.setEnabled(True)
            self.known_mass_button.setEnabled(True)
            self.cal_left_button.setStyleSheet("background-color: None; color: None;")
            self.cal_right_button.setStyleSheet("background-color: None; color: None;")
            self.cal_thrust_button.setStyleSheet("background-color: None; color: None;")
            self.known_mass_button.setStyleSheet("background-color: None; color: None;")
            self.cal_val_received = False
            self.wait_for_cal_factor = False
            self.timer_cal.stop()
        
    def toggle_test(self):
        if self.lc_test_button.isChecked():
            self.lc_test_button.setText("Lõpeta test")
            self.test_on()
        else:
            self.lc_test_button.setText("Testi koormusandureid")
            self.test_off()

    def test_on(self):
        window.sendData('ON')
        self.timer_test.start(500)
        self.lc_test = True

    def test_off(self):
        window.sendData('OFF')
        self.timer_test.stop()
        self.lc_test = False

    def update_data(self):
        thrust = float(window.thrust_test_value)
        thrust_grams = format((float(window.thrust_test_value) *1000.00) / g_const, '.2f')
        weight = float(window.weight_test_value)
        torque = float(window.torque_test_value)
        self.thrust_lc_reading.setText(f"{thrust}")
        self.thrust_lc_reading_grams.setText(f"{thrust_grams}")
        self.weight_lc_reading.setText(f"{weight}")
        self.torque_lc_reading.setText(f"{torque}")
        
    def tare(self):
        self.sendData.emit('tare')
        
    def change_trq_lc(self, id):
        if (id == 1):
            msg = 'measureL'
            window.sendData(msg)
        if (id == 2):
            msg = 'measureR'
            window.sendData(msg)
        if (id == 3):
            msg = 'measure0'
            window.sendData(msg)
            
    def send_left_cal_val(self):
        left_cal_val_data = 'setTrqLCalVal|%.2f' %(self.left_cal_val.value())
        self.sendData.emit(left_cal_val_data)
    
    def send_right_cal_val(self):
        right_cal_val_data = 'setTrqRCalVal|%.2f' %(self.right_cal_val.value())
        self.sendData.emit(right_cal_val_data)
        
    def send_thrust_cal_val(self):
        thrust_cal_val_data = 'setThrCalVal|%.2f' %(self.thrust_cal_val.value())
        self.sendData.emit(thrust_cal_val_data)
        
    def send_arm_length(self):
        arm_length_data = 'setArmLength|%.2f' %(self.arm_length.value())

class SharedData():
    def __init__(self):
        self._arm_length = arm_length_default
        self._ratio = ratio_default
        self._rho = rho_default
        self._kin_visc = kin_visc_default
        self._safety_over_prop = safety_over_prop_default
        self._x_delta = x_delta_default
        self._max_number_of_samples = max_number_of_samples_default
        self._x_center = x_center_default
        self._y_max = y_max_default
        self._x_max_speed = x_max_speed_default
        self._y_max_speed = y_max_speed_default
        self._x_max_accel = x_max_accel_default
        self._y_max_accel = y_max_accel_default
        self._aoa_trim = aoa_trim_default
        self._aoa_limit = aoa_limit_default
        self._aoss_trim = aoss_trim_default
        self._aoss_max_limit = aoss_max_limit_default
        self._aoss_min_limit = aoss_min_limit_default
        self._min_pwm = min_pwm_default
        self._max_pwm = max_pwm_default
        
    @property
    def arm_length(self):
        return self._arm_length

    @arm_length.setter
    def arm_length(self, value):
        self._arm_length = value
    
    @property
    def ratio(self):
        return self._ratio

    @ratio.setter
    def ratio(self, value):
        self._ratio = value
    
    @property
    def rho(self):
        return self._rho

    @rho.setter
    def rho(self, value):
        self._rho = value
    
    @property
    def kin_visc(self):
        return self._kin_visc

    @kin_visc.setter
    def kin_visc(self, value):
        self._kin_visc = value
        
    @property
    def safety_over_prop(self):
        return self._safety_over_prop

    @safety_over_prop.setter
    def safety_over_prop(self, value):
        self._safety_over_prop = value
        
    @property
    def x_delta(self):
        return self._x_delta

    @x_delta.setter
    def x_delta(self, value):
        self._x_delta = value
        
    @property
    def max_number_of_samples(self):
        return self._max_number_of_samples

    @max_number_of_samples.setter
    def max_number_of_samples(self, value):
        self._max_number_of_samples = value
    
    @property
    def x_center(self):
        return self._x_center

    @x_center.setter
    def x_center(self, value):
        self._x_center = value

    @property
    def y_max(self):
        return self._y_max

    @y_max.setter
    def y_max(self, value):
        self._y_max = value
        
    @property
    def x_max_speed(self):
        return self._x_max_speed

    @x_max_speed.setter
    def x_max_speed(self, value):
        self._x_max_speed = value
    
    @property
    def y_max_speed(self):
        return self._y_max_speed

    @y_max_speed.setter
    def y_max_speed(self, value):
        self._y_max_speed = value
        
    @property
    def x_max_accel(self):
        return self._x_max_accel

    @x_max_accel.setter
    def x_max_accel(self, value):
        self._x_max_accel = value
        
    @property
    def y_max_accel(self):
        return self._y_max_accel

    @y_max_accel.setter
    def y_max_accel(self, value):
        self._y_max_accel = value
        
    @property
    def aoa_trim(self):
        return self._aoa_trim

    @aoa_trim.setter
    def aoa_trim(self, value):
        self._aoa_trim = value
        
    @property
    def aoa_limit(self):
        return self._aoa_limit

    @aoa_limit.setter
    def aoa_limit(self, value):
        self._aoa_limit = value
        
    @property
    def aoss_trim(self):
        return self._aoss_trim

    @aoss_trim.setter
    def aoss_trim(self, value):
        self._aoss_trim = value
        
    @property
    def aoss_max_limit(self):
        return self._aoss_max_limit

    @aoss_max_limit.setter
    def aoss_max_limit(self, value):
        self._aoss_max_limit = value
        
    @property
    def aoss_min_limit(self):
        return self._aoss_min_limit

    @aoss_min_limit.setter
    def aoss_min_limit(self, value):
        self._aoss_min_limit = value
        
    @property
    def cal_value(self):
        return self._cal_value

    @cal_value.setter
    def cal_value(self, value):
        self._cal_value = value
        
    @property
    def min_pwm(self):
        return self._min_pwm

    @min_pwm.setter
    def min_pwm(self, value):
        self._min_pwm = value
        
    @property
    def max_pwm(self):
        return self._max_pwm

    @max_pwm.setter
    def max_pwm(self, value):
        self._max_pwm = value

class SerialReader(QObject):
    serial_readout = pyqtSignal(str)
    calValueReceived = pyqtSignal(str)

    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.shared_data = SharedData()
        self.lc_cal = LC_calibration(self.shared_data)
        self.update_timer = QTimer()
        self.latest_data = 0
        self.running = True  # Flag to control the running of the loop
        self.buf = bytearray()

    def run(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            self.r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return self.r
        while self.running and self.controller and self.controller.isOpen(): 
            i = max(1, min(2048, self.controller.in_waiting))
            data = self.controller.read(i)
            i = data.find(b"\n")
            if i >= 0:
                self.r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                self.latest_data = self.r.decode('utf_8').strip()
                self.serial_readout.emit(str(self.latest_data))
            else:
                self.buf.extend(data)
    
    def stop(self):
        self.running = False
        
class MeasuringWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int, str)
    input_data = pyqtSignal(str)
    requestStart = pyqtSignal()
    requestStop = pyqtSignal()
    stopTimer = pyqtSignal()

    def __init__(self):
        super(MeasuringWorker, self).__init__()
        self.shared_data = SharedData()
        self._running = False
        self.stopTimer.connect(self.stop_timer)
        self.measuring_stopped = True
        self.e_stop = False
        self.rpm = 0
        self.cnv = Canvas()
        self.counter = 0
        self.custom_trajectory = False
        self.omega_avg = 0
        self.arspd_avg = 0
        self.aoa_avg = 0
        self.aoss_avg = 0
        self.v_tan = 0
        self.v_rad = 0
        self.v_axial = 0
        self.meas_timer = QTimer(self)
        self.meas_timer.timeout.connect(self.measure)
        self.x_goal = 0
        self.x_prev = 0
        self.first_line_done = False
        self.header_added = False
        self.x_normalized_mm = 0
        self.goal_reached = False
        self.b = 0
        self.time_delay = 0
        self.send_once = False
        self.cycle_time = 1
        self.current_x_target = 0
        self.current_y_target = 0
        self.data = ''
   
    def start_measuring(self):
        window.tare_done = False
        self.wait = 0
        if window.counter == 0:
            window.today_dt = datetime.datetime.today().strftime('%d-%m-%Y-%H:%M:%S')
            self.parent_dir = "/home/siim/Desktop/logid/"
            window.path = os.path.join(self.parent_dir, window.today_dt)
            os.mkdir(window.path)
            window.csvfile = "log" + window.today_dt + ".csv"
            self.header = ['Prop_diam(inch) ' 'X_position(mm) ' 'Y_position(mm) ' 'Torque(Nm) ' 'Thrust(N) ' 'Omega(rad/s) ' 'Airspeed(m/s) ' 'AoA(deg) ' 'AoSS(deg) ' 'V_tan(m/s) ' 'V_rad(m/s) ' 'V_axial(m/s)']
            self.header_added = False
        window.cnv.clear_plot()
        self.aoss_a = []
        self.aoa_a = []
        self.arspd_a = []
        self.omega_a = []
        window.label13.clear()
        window.label15.clear()
        window.label17.clear()
        window.label19.clear()
        window.label65.clear()
        window.label67.clear()
        window.label69.clear()
        window.measure.setEnabled(False)
        window.Y_move.setEnabled(False)
        window.measuring_stopped = False
        window.test_progress.setMaximum(window.sweep_count.value())
        self.first_line_done = False
        window.radius_mm = format(((window.prop.value()*25.4)/2),'.1f')
        self.radius_steps = format((float(window.radius_mm) * float(self.shared_data.ratio)),'.0f')
        window.x_goal = int((window.mm_to_steps(self.shared_data.x_center) - int(self.radius_steps)) * (1 - self.shared_data.safety_over_prop/100))
        window.progress.setValue(1)
        window.sendData('BeaconON')
        time.sleep(3)
        window.sendData('tare')
        while (window.tare_done == False):
            print("andurite nullimine")
        window.progress.setValue(5)
        window.sendData('streamStart')
        time.sleep(2)
        throttle = int(1000 + (window.throttle.value() * 10))
        start = f'start|{throttle}'
        window.sendData(start)
        time.sleep(10)
        window.progress.setValue(10)
        self.x_normalized_mm = 0
        self.b = 0
        self.current_x_target = 0
        self.current_y_target = 0
        self.send_once = True
        window.sendData('m|%d|%d|%d|%d' %(((self.shared_data.x_center - 3) * self.shared_data.ratio), (window.Y_pos.value() * self.shared_data.ratio), window.measure_speed.value(), (window.measure_speed.value()/3)))
        time.sleep(2)
        window.sendData('m|%d|%d|%d|%d' %(((self.shared_data.x_center) * self.shared_data.ratio), (window.Y_pos.value() * self.shared_data.ratio), window.measure_speed.value(), (window.measure_speed.value()/3)))
        time.sleep(3)
        self.meas_timer.start(self.cycle_time)  # Measure every 100ms
        
    def start_measuring_after_first(self):
        window.tare_done = False
        window.cnv.clear_plot()
        self.aoss_a = []
        self.aoa_a = []
        self.arspd_a = []
        self.omega_a = []
        window.label13.clear()
        window.label15.clear()
        window.label17.clear()
        window.label19.clear()
        window.label65.clear()
        window.label67.clear()
        window.label69.clear()
        window.measure.setEnabled(False)
        window.Y_move.setEnabled(False)
        window.measuring_stopped = False
        window.test_progress.setMaximum(window.sweep_count.value())
        self.first_line_done = False
        window.radius_mm = format(((window.prop.value()*25.4)/2),'.1f')
        self.radius_steps = format((float(window.radius_mm) * float(self.shared_data.ratio)),'.0f')
        window.x_goal = int((window.mm_to_steps(self.shared_data.x_center) - int(self.radius_steps)) * (1 - self.shared_data.safety_over_prop/100))
        time.sleep(10)
        if window.counter == 0:
            window.progress.setValue(1)
            window.sendData('tare')
            while (window.tare_done == False):
                print("ootan")
        window.progress.setValue(5)
        window.sendData('streamStart')
        time.sleep(2)
        throttle = int(1000 + (window.throttle.value() * 10))
        start = f'start|{throttle}'
        window.sendData(start)
        time.sleep(10)
        window.progress.setValue(10)
        self.x_normalized_mm = 0
        self.b = 0
        self.x_prev = 0
        self.send_once = True
        self.goal_reached == False
        self.current_x_target = 0
        self.current_y_target = 0
        self.data = ""
        print(self.data)
        self.measure()
            
    def measure(self):
        if not self._running:
            self.meas_timer.stop()
            print("lõpetan")
            return
        if window.custom_trajectory == False and self.send_once == True:
            list_of_x_targets.append(window.x_goal)
            list_of_y_targets.append(0)
            window.sendData('m|%d|%d|%d|%d' %(list_of_x_targets[0], (list_of_y_targets[0] + (window.Y_pos.value() * self.shared_data.ratio)), window.measure_speed.value(), (window.measure_speed.value()/3)))
            self.send_once = False
        if window.custom_trajectory == True and self.b == 0 and self.send_once == True:
            try:
                window.sendData('m|%d|%d|%d|%d' % (list_of_x_targets[self.b], ((window.Y_pos.value() * self.shared_data.ratio) + list_of_y_targets[self.b]), window.measure_speed.value(), (window.measure_speed.value() / 3)))     
                self.current_x_target = list_of_x_targets[self.b]
                self.current_y_target = int(((window.Y_pos.value() * self.shared_data.ratio) + list_of_y_targets[self.b]))
                self.send_once = False
            except:
                print("0 write failed")
        if window.custom_trajectory == True and 0 < self.b <= (len(list_of_x_targets) - 1) and self.send_once == True:
            try:
                window.sendData('m|%d|%d|%d|%d' % (list_of_x_targets[self.b], ((window.Y_pos.value() * self.shared_data.ratio) + list_of_y_targets[self.b]), window.measure_speed.value(), (window.measure_speed.value() / 3)))     
                self.current_x_target = list_of_x_targets[self.b]
                self.current_y_target = int(((window.Y_pos.value() * self.shared_data.ratio) + list_of_y_targets[self.b]))
                self.send_once = False
            except:
                print("write failed")
        if (self.x_normalized_mm == 0 and self.first_line_done == False):
            aoss_zero = format(window.aoss_abs,'.2f')
            aoa_zero = format(window.aoa_abs,'.2f')
            omega_zero = window.omega
            arspd_zero = format(window.airspeed,'.2f')
            v_tan_zero = format(float(self.arspd_avg) * math.sin(math.radians(float(self.aoa_avg))),'.2f')
            v_rad_zero = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.sin(math.radians(float(self.aoss_avg))),'.2f')
            v_axial_zero = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.cos(math.radians(float(self.aoss_avg))),'.2f')
            data_zero = str(str(format(window.prop.value(),'.1f'))+" "+str(self.x_normalized_mm)+" "+str(window.Y_pos.value())+" "+str(window.trq_current)+" "+str(window.thr_current)+" "+str(omega_zero)+" "+str(arspd_zero)+" "+str(aoa_zero)+" "+str(aoss_zero)+" "+str(v_tan_zero)+" "+str(v_rad_zero)+" "+str(v_axial_zero))
            with open(os.path.join(window.path,window.csvfile), 'a') as f:
                w = csv.writer(f)
                if not self.header_added:
                    w.writerow(self.header)
                    self.header_added = True
                w.writerow([data_zero])
            self.first_line_done = True
            self.x_prev = self.x_normalized_mm
        if (self.x_normalized_mm - self.x_prev < self.shared_data.x_delta and self.first_line_done == True):
            self.aoss_a.append(float(window.aoss_abs))
            self.aoa_a.append(float(window.aoa_abs))
            self.arspd_a.append(float(window.airspeed))
            self.omega_a.append(float(window.omega))
        if (self.x_normalized_mm - self.x_prev >= self.shared_data.x_delta and self.first_line_done == True):
            self.aoss_avg = format(statistics.mean(self.aoss_a), '.2f')
            self.aoa_avg = format(statistics.mean(self.aoa_a),'.2f')
            self.arspd_avg = format(statistics.mean(self.arspd_a),'.2f')
            if float(self.arspd_avg) > 100.00:
                self.arspd_avg = 0.00
            self.omega_avg = format(statistics.mean(self.omega_a),'.2f')
            self.v_tan = format(float(self.arspd_avg) * math.sin(math.radians(float(self.aoa_avg))),'.2f')
            self.v_rad = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.sin(math.radians(float(self.aoss_avg))),'.2f') 
            self.v_axial = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.cos(math.radians(float(self.aoss_avg))),'.2f') 
            self.data = str(str(format(window.prop.value(),'.1f'))+" "+str(self.x_normalized_mm)+" "+str(window.steps_to_mm(window.y_current_steps))+" "+str(window.trq_current)+" "+str(window.thr_current)+" "+str(self.omega_avg)+" "+str(self.arspd_avg)+" "+str(self.aoa_avg)+" "+str(self.aoss_avg)+" "+str(self.v_tan)+" "+str(self.v_rad)+" "+str(self.v_axial))
            #print(self.data)
            with open(os.path.join(window.path,window.csvfile), 'a') as f:
                w = csv.writer(f)
                if not self.header_added:
                    w.writerow(self.header)
                    self.header_added = True
                w.writerow([self.data])
            self.x_prev = self.x_normalized_mm
            window.update_plot(self.x_normalized_mm, self.omega_avg, self.arspd_avg, self.aoa_avg, self.aoss_avg, window.trq_current, window.thr_current)
            self.aoss_a.clear()
            self.aoa_a.clear()
            self.arspd_a.clear()
            self.omega_a.clear()
        fg = self.shared_data.x_center - (int(window.steps_to_mm(window.x_goal)))
        if window.meas_data_running:
            self.x_normalized_mm = self.shared_data.x_center - (int(window.steps_to_mm(window.x_current_steps)))
            x_progress = round((self.x_normalized_mm/fg)*90,0)
            window.progress.setValue(10 + int(x_progress))
            #print(window.x_current_steps)
            if (window.x_current_steps <= window.x_goal and self.goal_reached == False):
                #print(window.x_goal)
                self.goal_reached = True
                window.counter = window.counter + 1
                window.test_progress.setValue(window.counter)
                window.meas_data_running = False
                #print("eesmärk täidetud")
                self.check_progress(window.counter, window.x_current_steps)
#             if self.goal_reached:
#                 window.counter = window.counter + 1
#                 window.test_progress.setValue(window.counter)
#                 self.goal_reached = False
#                 window.meas_data_running = False
#                 self.check_progress(window.counter, window.x_current_steps)
            if (window.x_current_steps == self.current_x_target and window.y_current_steps == self.current_y_target and window.custom_trajectory == True and self.goal_reached == False):
                self.b = self.b + 1
                if self.b < len(list_of_x_targets):
                    self.send_once = True
                if self.b >= len(list_of_x_targets):
                    self.send_once = False
                #print(self.b)
                pass
            
        else:
            self.x_normalized_mm = 0
        
    def check_progress(self, cycles_done, x_curr):
        #print(cycles_done)
        if cycles_done == window.sweep_count.value():
            window.measuring_stopped = True
            window.come_back()
            window.process_data()
        else:
            self.goal_reached = False
            time.sleep(1)
            window.sendData('stop')
            time.sleep(2)
            window.sendData('j|%d|0|%d|%d' %(x_curr, window.jog_speed.value(), window.jog_speed.value()))
            time.sleep(3)
            window.sendData('center')
            time.sleep(5)
            window.sendData('j|%d|%d|%d|%d' %((self.shared_data.x_center * self.shared_data.ratio), (window.Y_pos.value() * self.shared_data.ratio), window.jog_speed.value(), window.jog_speed.value()))
            self.start_measuring_after_first()

    def stop_measuring(self):
        self._running = False
        self.stopTimer.emit()

    @pyqtSlot()
    def stop_timer(self):
        self.meas_timer.stop()
        self._running = False
        print("Measurement stopped")
    
    @pyqtSlot()
    def run(self):
        self._running = True
        self.start_measuring()

# Subclass QMainWindow to customize your application's main window
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
        self.lc_calibration = LC_calibration(self.shared_data)
        self.rpm_setup = RPM_controller(self.shared_data)
        self.today_dt = None
        self.path = None
        self.csvfile = None
        
    def setupUI(self):
        self.measuring_stopped = True
        self.e_stop = False
        self.rpm = 0
        self.cnv = Canvas()
        self.counter = 0
        self.custom_trajectory = False
        self.rpm_label = QLabel()
        self.thr_label = QLabel()
        self.trq_label = QLabel()
        self.weight_label = QLabel()
        self.motor_test = False
        self.homing_done = False
        self.tare_done = False
        self.fileSelected = False
        self.cal_value = 0
        self.thrust_test_value = 0.0
        self.weight_test_value = 0.0
        self.torque_test_value = 0.0
        self.x_normalized_mm = 0
        self.x_current_steps = 0
        self.y_current_steps = 0
        self.thr_current = 0
        self.trq_current = 0
        self.rpm_current = 0
        self.airspeed = 0
        self.aoa_sensor = 0
        self.aoa_abs = 0
        self.aoss_sensor = 0
        self.aoss_abs = 0
        self.omega = 0
        self.meas_data_running = False
        self.in_position = False
        self.jog_done = False
        self.centering_done = False
        self.radius_mm = None
        self.x_goal = None
        
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.showMaximized()
        
        layout = QGridLayout(self.centralWidget)
        
        self.toolbar = QToolBar("Lisavalikud")
        self.addToolBar(self.toolbar)
        
        self.aoa_aoss_action = QAction("AoA ja AoSS trimmimine", self)
        self.aoa_aoss_action.triggered.connect(self.aoa_aoss_params)
        self.toolbar.addAction(self.aoa_aoss_action)

        self.calibrate_loadcells_action = QAction("Koormusandurite kalibreerimine", self)
        self.calibrate_loadcells_action.triggered.connect(self.calibrate_loadcells)
        self.toolbar.addAction(self.calibrate_loadcells_action)
        
        self.rpm_controller_action = QAction("RPM kontrolleri valikud", self)
        self.rpm_controller_action.triggered.connect(self.setup_rpm)
        self.toolbar.addAction(self.rpm_controller_action)

        self.map_action = QAction("Trajektoori valikud", self)
        self.map_action.triggered.connect(self.map_)
        self.toolbar.addAction(self.map_action)

        self.clear_plot_action = QAction("Tühjenda graafik", self)
        self.clear_plot_action.triggered.connect(self.cnv.clear_plot)
        self.toolbar.addAction(self.clear_plot_action)

        self.save_plot_action = QAction("Salvesta graafik", self)
        self.save_plot_action.triggered.connect(self.save_plot)
        self.toolbar.addAction(self.save_plot_action)

        # Disable the toolbar buttons
        self.aoa_aoss_action.setEnabled(False)
        self.calibrate_loadcells_action.setEnabled(False)
        self.rpm_controller_action.setEnabled(False)
        self.map_action.setEnabled(False)
        self.clear_plot_action.setEnabled(False)
        self.save_plot_action.setEnabled(False)
        
         # Set column stretch factors
        layout.setColumnStretch(0, 0)  # First column does not stretch
        layout.setColumnStretch(1, 1)  # Second column takes all remaining space

        # Set row stretch factors if necessary
        layout.setRowStretch(0, 0)
        layout.setRowStretch(1, 0)
        layout.setRowStretch(2, 0)
        layout.setRowStretch(3, 1)  # Allows the canvas row to expand more

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
        
        self.params = QPushButton("Säti parameetrid", self)
        self.params.setStyleSheet("background-color: None; color: None;")
        self.params.setEnabled(False)
        self.params.clicked.connect(self.set_params)
        layout.addWidget(self.params, 3, 0, 1, 1)
        
        self.file = QPushButton("Vali propelleri konfiguratsiooni fail")
        self.file.clicked.connect(self.showDialog)
        self.file.setStyleSheet("background-color: None; color: None;")
        self.file.setEnabled(False)
        layout.addWidget(self.file, 4, 0, 1, 1)
        
        self.label9 = QLabel("Sisesta Pitot' AoA eelnurk (absoluutsuurus)")
        #layout.addWidget(self.label9)
        
        self.aoa_pre = QDoubleSpinBox()
        self.aoa_pre.setMinimum(0.00)
        self.aoa_pre.setMaximum(45.00)
        self.aoa_pre.setValue(0)
        #layout.addWidget(self.aoa_pre)
        
        self.label10 = QLabel("Sisesta Pitot' AoSS eelnurk (absoluutsuurus)")
        #layout.addWidget(self.label10)
        
        self.aoss_pre = QDoubleSpinBox()
        self.aoss_pre.setMinimum(0.00)
        self.aoss_pre.setMaximum(45.00)
        self.aoss_pre.setValue(0)
        #layout.addWidget(self.aoss_pre)
        
        self.homing = QPushButton("Telgede referents")
        self.homing.clicked.connect(self.home)
        self.homing.setStyleSheet("background-color: None; color: None;")
        self.homing.setEnabled(False)
        layout.addWidget(self.homing, 5, 0, 1, 1)
        
        self.label1 = QLabel("Sisesta propelleri läbimõõt tollides")
        layout.addWidget(self.label1, 7, 0, 1, 1)
        
        self.prop = QDoubleSpinBox()
        self.prop.setMinimum(5.00)
        self.prop.setMaximum(23.00)
        self.prop.setSingleStep(0.1)
        self.prop.setValue(20.2)
        layout.addWidget(self.prop, 8, 0, 1, 1)
        
        self.centering = QPushButton("Pitot' tsentrisse")
        self.centering.setEnabled(False)
        self.centering.clicked.connect(self.center)
        self.centering.setStyleSheet("background-color: None; color: None;")
        layout.addWidget(self.centering, 10, 0, 1, 1)
        
        #self.label3 = QLabel("Pitot' mootorite kiirus")
        #layout.addWidget(self.label3, 9, 0, 1, 1)
        
        self.jog_speed = QSpinBox()
        self.jog_speed.setMinimum(500)
        self.jog_speed.setMaximum(2000)
        self.jog_speed.setSingleStep(100)
        self.jog_speed.setValue(self.shared_data.x_max_speed)
        #layout.addWidget(self.jog_speed, 10, 0, 1, 1)
        
        self.label2 = QLabel("Y-telje positsioon mm" )
        layout.addWidget(self.label2, 11, 0, 1, 1)
        
        self.Y_pos = QSpinBox()
        self.Y_pos.setMinimum(0)
        self.Y_pos.setMaximum(105)
        layout.addWidget(self.Y_pos, 12, 0, 1, 1)
        
        self.Y_pos.valueChanged.connect(self.enable_Y_move_button)
        
        self.Y_move = QPushButton("Liiguta Y-telge")
        self.Y_move.clicked.connect(self.moveY)
        self.Y_move.setEnabled(False)
        layout.addWidget(self.Y_move, 13, 0, 1, 1)
        
        self.label4 = QLabel("Propelleri kiirus (%)")
        layout.addWidget(self.label4, 14, 0, 1, 1)
        
        self.throttle = QDoubleSpinBox()
        self.throttle.setMinimum(10.0)
        self.throttle.setMaximum(100.0)
        self.throttle.setValue(10.0)
        self.throttle.setSingleStep(0.1)
        layout.addWidget(self.throttle, 15, 0, 1, 1)
        
        self.testMotorButton = QPushButton("Testi mootorit", self)
        self.testMotorButton.setEnabled(False)
        self.testMotorButton.setCheckable(True)
        self.testMotorButton.clicked.connect(self.toggle_motor)
        layout.addWidget(self.testMotorButton, 16, 0, 1, 1)
        
        self.labelRPM = QLabel("Mõõdetud pöörded (RPM)", self)
        layout.addWidget(self.labelRPM, 17, 0, 1, 1)
        
        layout.addWidget(self.rpm_label, 18, 0, 1, 1)
        
        self.labelThr  = QLabel("Mõõdetud tõmme (N)", self)
        layout.addWidget(self.labelThr, 19, 0, 1, 1)
        
        layout.addWidget(self.thr_label, 20, 0, 1, 1)
        
        self.labelTrq  = QLabel("Mõõdetud moment (Nm)", self)
        layout.addWidget(self.labelTrq, 21, 0, 1, 1)
        
        layout.addWidget(self.trq_label, 22, 0, 1, 1)
        
        self.labelWeight  = QLabel("Mõõdetud momendi kaal (g)", self)
        layout.addWidget(self.labelWeight, 23, 0, 1, 1)
        
        layout.addWidget(self.weight_label, 24, 0, 1, 1)
        
        self.label5 = QLabel("Mõõdistamise kiirus")
        layout.addWidget(self.label5, 25, 0, 1, 1)
        
        self.measure_speed = QSpinBox()
        self.measure_speed.setMinimum(200)
        self.measure_speed.setMaximum(500)
        self.measure_speed.setSingleStep(100)
        self.measure_speed.setValue(300)
        layout.addWidget(self.measure_speed, 26, 0, 1, 1)
        
        self.label11 = QLabel("Kordusmõõtmiste arv")
        layout.addWidget(self.label11, 27, 0, 1, 1)
        
        self.sweep_count = QSpinBox()
        self.sweep_count.setMinimum(1)
        self.sweep_count.setMaximum(max_number_of_samples_default)
        self.sweep_count.setSingleStep(1)
        self.sweep_count.setValue(1)
        layout.addWidget(self.sweep_count, 28, 0, 1, 1)
        
        self.measure = QPushButton("Alusta mõõtmisega")
        self.measure.setEnabled(False)
        self.measure.clicked.connect(self.start_measuring)
        layout.addWidget(self.measure, 29, 0, 1, 1)
        
        self.label6 = QLabel("Mõõdistamise kulg")
        layout.addWidget(self.label6, 30, 0, 1, 1)
        
        self.progress = QProgressBar()
        self.progress.setMinimum(0)
        self.progress.setMaximum(100)
        self.progress.setValue(0)
        layout.addWidget(self.progress, 31, 0, 1, 1)
        
        self.test_progress = QProgressBar()
        self.test_progress.setMinimum(0)
        self.test_progress.setMaximum(self.sweep_count.value())
        self.test_progress.setValue(0)
        layout.addWidget(self.test_progress, 32, 0, 1, 1)
        
        self.back = QPushButton("Pitot' tagasi algasendisse")
        self.back.setEnabled(False)
        self.back.clicked.connect(self.come_back)
        layout.addWidget(self.back, 33, 0, 1, 1)
        
        self.danger = QLabel("Emergency!")
        self.danger.setAlignment(Qt.AlignCenter)
        self.danger.setStyleSheet("background-color: None")
        layout.addWidget(self.danger, 34, 0, 1, 1)
        
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
        
        self.update_ports()
        self.initTimer()
        self.reset_button()
        self.last_throttle_value = self.throttle.value()
        
        self.timer_motor = QTimer()
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
                
    def handleData(self, data):
        #print(data)
        if (data == 'Emergency!'):
            self.e_stop = True
            self.meas_data_running = False
            self.update_emergency()
        if (data == 'OK'):
            self.e_stop = False
            self.meas_data_running = False
            self.update_emergency()
        if self.motor_test == False:
            self.update_rpm_label('0')
            self.update_thr_label('0')
            self.update_trq_label('0')
            self.update_weight_label('0')
            self.meas_data_running = False
        if (data == 'homing done'):
            self.homing_done = True
            self.measure.setEnabled(False)
            self.centering.setEnabled(True)
            self.testMotorButton.setEnabled(True)
            self.back.setEnabled(True)
            self.progress.setValue(0)
            self.test_progress.setValue(0)
            self.homing.setStyleSheet("background-color: green; color: white;")
            self.meas_data_running = False
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
            if len(parts) == 3:
                try:
                    self.thrust_test_value = float(int(parts[0])/1000.00)
                    self.weight_test_value = float(parts[1])
                    self.torque_test_value = float(int(parts[2])/1000.00)
                    #self.lc_cal.update_data()
                    if (self.motor_test == True):
                        self.update_thr_label(f"{self.thrust_test_value}")
                        self.update_trq_label(f"{self.torque_test_value}")
                        self.update_weight_label(f"{self.weight_test_value}")
                    if self.motor_test == False:
                        self.update_thr_label('0')
                        self.update_trq_label('0')
                        self.update_weight_label('0')
                        try:
                            self.lc_calibration.update_data()
                        except:
                            pass
                except ValueError:
                    print("Error parsing numeric data:", parts)
        if (data.startswith('RPM_test:')):
            data = data[9:].strip()
            self.update_rpm_label(f"{data}")
            self.meas_data_running = False
        if (data.startswith('Measurements:')):
            data = data[13:].strip()
            self.meas_data_running = True
            parts = data.split()
            if len(parts) == 10:
                try:
                    self.x_current_steps = int(parts[0])
                    self.y_current_steps = int(parts[1])
                    self.thr_current = float(int(parts[2])/1000.00)
                    self.trq_current = float(int(parts[3])/1000.00)
                    self.rpm_current = int(parts[4])
                    self.airspeed = float(parts[5])
                    self.aoa_sensor = float(parts[6])
                    self.aoa_abs = float(parts[7])
                    self.aoss_sensor = float(parts[8])
                    self.aoss_abs = float(parts[9])
                    self.omega = format((((2.0 * math.pi)/60.0) * self.rpm_current),'.2f')
                except ValueError:
                    print("Error parsing data:", parts)
        
    def toggle_motor(self):
        if self.testMotorButton.isChecked():
            self.testMotorButton.setText("Seiska mootor")
            window.sendData('ON')
            time.sleep(2)
            window.sendData('BeaconON')
            time.sleep(2)
            self.test_motor()
        else:
            self.testMotorButton.setText("Testi mootorit")
            self.stop_motor()
            time.sleep(2)
            window.sendData('OFF')
            time.sleep(2)
            window.sendData('BeaconOFF')
            time.sleep(2)
            window.sendData('OFF')

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
        throttle = int(1000 + (self.throttle.value() * 10))
        start = f'test|{throttle}'
        self.sendData(start)
        self.motor_test = True
        
    def read_values(self):
        current_throttle = self.throttle.value()
        if current_throttle != self.last_throttle_value:
            self.update_throttle_test()
            self.last_throttle_value = current_throttle

    def initSerialReader(self):
        if self.controller and not self.serialReaderThread.isRunning():
            if self.serialReader:
                self.serialReaderThread.quit()
                self.serialReaderThread.wait()

            self.serialReader = SerialReader(self.controller)
            self.serialReader.moveToThread(self.serialReaderThread)
            self.serialReader.calValueReceived.connect(self.lc_calibration.update_cal_factor_label)
            self.serialReader.serial_readout.connect(self.handleData)
            self.serialReaderThread.started.connect(self.serialReader.run)
            self.serialReaderThread.start()
            
            self.measuringWorker = MeasuringWorker()
            self.measuringWorker.moveToThread(self.measuringThread)
            self.measuringThread.started.connect(self.measuringWorker.run)
            self.measuringWorker.requestStop.connect(self.measuringWorker.stop_measuring)

    def update_rpm_label(self, rpm):
        if self.motor_test == True:
            self.rpm_label.setText(rpm)
        else:
            self.rpm_label.setText('0')
            
    def update_thr_label(self, thr):
        if self.motor_test == True:
            self.thr_label.setText(thr)
        else:
            self.thr_label.setText('0')
            
    def update_trq_label(self, trq):
        if self.motor_test == True:
            self.trq_label.setText(trq)
        else:
            self.trq_label.setText('0')
            
    def update_weight_label(self, weight):
        if self.motor_test == True:
            self.weight_label.setText(weight)
        else:
            self.weight_label.setText('0')
        
    def save_plot(self):
        fileName, _ = QFileDialog.getSaveFileName(self, "Save Plot", "",
                      "PNG Files (*.png);;JPEG Files (*.jpg);;All Files (*)", options=QFileDialog.Options())
        if fileName:
            self.cnv.save_plot(fileName)
        
    def update_plot(self, x, rpms, airspeed, absAoa, absAoss, trq_curr, thr_curr):
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
            self.cnv.add_data([x], [scaled_rpms], labels_styles[0][0], labels_styles[0][1])
            self.cnv.add_data([x], [airspeed], labels_styles[1][0], labels_styles[1][1])
            self.cnv.add_data([x], [absAoa], labels_styles[2][0], labels_styles[2][1])
            self.cnv.add_data([x], [absAoss], labels_styles[3][0], labels_styles[3][1])
            self.cnv.add_data([x], [trq], labels_styles[4][0], labels_styles[4][1])
            self.cnv.add_data([x], [thr], labels_styles[5][0], labels_styles[5][1])
            
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
                self.lc_calibration.initialize(self.shared_data)
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
        if not hasattr(self, 'Param_window'):
            self.param_window = SetParameters(self.shared_data)
            self.param_window.sendData.connect(self.sendData)
        self.param_window.show()
            
    def map_(self):
        if not hasattr(self, 'Mapping_window'):
            self.map_window = MapTrajectory(self.shared_data)
            self.map_window.sendData.connect(self.sendData)
        self.map_window.show()
                
    def calibrate_loadcells(self):
        if not hasattr(self, 'LC_calib_window'):
            self.lc_calib_window = LC_calibration(self.shared_data)
            self.lc_calib_window.sendData.connect(self.sendData)
        self.lc_calib_window.show()
    
    def aoa_aoss_params(self):
        if not hasattr(self, 'AoA_AoSS_window'):
            self.aoa_aoss_window = AoA_AoSS(self.shared_data)
            self.aoa_aoss_window.sendData.connect(self.sendData)
        self.aoa_aoss_window.show()
        
    def setup_rpm(self):
        if not hasattr(self, 'RPM setup'):
            self.rpm_setup_window = RPM_controller(self.shared_data)
            self.rpm_setup_window.sendData.connect(self.sendData)
        self.rpm_setup_window.show()
        
    def home(self):
        self.homing_done = False
        self.homing.setStyleSheet("background-color: None; color:None;")
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
        window.sendData('BeaconOFF')
        
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
        mean_csvfile = "log" + self.today_dt + "_mean.csv"
        mean_header = ['#_of_samples ' 'Prop_diam(inch) ' 'X_position(mm) ' 'Y_position(mm) ' 'Torque(Nm) ' 'Thrust(N) ' 'Omega(rad/s) '
                       'Airspeed(m/s) ' 'AoA(deg) ' 'AoSS(deg) ' 'V_tan(m/s) ' 'V_rad(m/s) ' 'V_axial(m/s) ' 'Chord_angle(deg) '
                       'Chord_length(mm) ' 'Helix_angle(deg) ' 'Alpha_angle(deg) ' 'V_total(m/s) ' 'V_lift(m/s) ' 'V_drag(m/s) '
                       'CL ' 'CD ' 'Reynolds_number']       
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
                    omega_mean = format(statistics.mean(list(dict_omega.values())),'.2f')
                    arspd_mean = format(statistics.mean(list(dict_arspd.values())),'.2f')
                    aoa_mean = format(statistics.mean(list(dict_aoa.values())),'.2f')
                    aoss_mean = format(statistics.mean(list(dict_aoss.values())),'.2f')
                    v_tan_mean = format(statistics.mean(list(dict_v_tan.values())),'.2f')
                    v_rad_mean = format(statistics.mean(list(dict_v_rad.values())),'.2f')
                    v_axial_mean = format(statistics.mean(list(dict_v_axial.values())),'.2f')
                    
                    mean_list = str(str(testnumber)+" "+str(prop)+" "+str(x_pos)+" "+str(y_pos)+" "+str(trq_mean)+" "+str(thr_mean)+" "+str(omega_mean)+" "+str(arspd_mean)+" "+str(aoa_mean)+" "+str(aoss_mean)+" "+str(v_tan_mean)+" "+str(v_rad_mean)+" "+str(v_axial_mean))
                    
                    var = format((float(x_pos)/1000)*float(v_axial_mean),'.2f')
                    var_list.append(float(var))
                    trq_list.append(float(trq_mean))
                    thr_list.append(float(thr_mean))
                    omega_list.append(float(omega_mean))
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
            #chord_length = '0.0'
            if mean_data[2] == angle_data[0]:
                chord_angle = angle_data[1]
                chord_length = angle_data[2]
                mean_data.append(chord_angle)
                mean_data.append(chord_length)
            else:
                chord_angle = '0.0'
                chord_length = '0.0'
                mean_data.append(chord_angle)
                mean_data.append(chord_length)
            try:
                helix_angle = math.degrees(math.atan(float(v_axial_mean)/((float(omega_mean)*float(x_pos/1000))-float(v_tan_mean))))
            except:
                helix_angle = 0
            mean_data.append(str(format(helix_angle,'.2f')))
            alpha_angle = format(float(chord_angle) - helix_angle,'.2f')
            mean_data.append(str(alpha_angle))
            total_speed = math.sqrt(math.pow(((float(omega_mean)*float(x_pos/1000)) - float(v_tan_mean)),2) + math.pow(float(v_axial_mean),2) + math.pow(float(v_rad_mean),2))
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

            with open(os.path.join(self.path,mean_csvfile), 'a') as f:
                w = csv.writer(f)
                w.writerow([' '.join(mean_data)])
            
            x_mp = x_mp + 3
            
        vi = (2*(self.shared_data.x_delta/1000)*sum(var_list))/((float(self.radius_mm)/1000)*(float(self.radius_mm)/1000))
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
        o = statistics.mean(omega_list)
        P = float(M)*float(o)
        self.label17.setText(str(format(P,'.2f')))
        vm = self.shared_data.rho * math.pi * math.pow(float(self.radius_mm)/1000,2) * float(vi)
        self.label67.setText(str(format(vm,'.2f')))
        try:
            v_max_mean = float(T)/float(vm)
        except:
            v_max_mean = 0
        self.label69.setText(str(format(v_max_mean,'.2f')))
        try:
            Ct = float(T)/(self.shared_data.rho * math.pow(float(o),2) * math.pow(((float(self.radius_mm)*2)/1000),4))
        except:
            Ct = 0
        try:
            Cp = float(M)/(self.shared_data.rho * math.pow(float(o),2) * math.pow(((float(self.radius_mm)*2)/1000),5))
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
            w.writerow(['Induced_power ',format(Pi,'.2f'),' W'])
            w.writerow(['Power ',format(P,'.2f'),' W'])
            w.writerow(['Efficiency ',format(nu,'.2f'),' %'])
            w.writerow(['Average_induced_speed ',format(vi,'.2f'),' m/s'])
            w.writerow(['Airspeed_ratio ',format(vv,'.2f')])
            w.writerow(['V_mass ',format(vm,'.2f'),' kg/s'])
            w.writerow(['V_max_mean ',format(v_max_mean,'.2f'),' m/s'])
            w.writerow(['Ct ',format(Ct,'.5f')])
            w.writerow(['Cp ',format(Cp,'.5f')])
            w.writerow(['Air_density ',self.shared_data.rho,' kg/m3'])
            w.writerow(['Air_kinematic_viscosity ',self.shared_data.kin_visc,' x10-5 m2/s'])
        
        var_list.clear()
        trq_list.clear()
        thr_list.clear()
        omega_list.clear()
        self.counter = 0
            
app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec_()