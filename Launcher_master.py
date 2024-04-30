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
from PyQt5.QtGui import QIcon
from queue import *
from matplotlib import pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
from pathlib import Path

#default values used on startup
rho_default = 1.225 #kg/cm3 standard air density
kin_visc_default = 1.48
ratio_default = 24.9955 #stepper drive ratio one revolution/distance travelled in mm
x_center_default = 315 #in mm, center position of the probe from home
y_max_default = 105 #in mm, max travel from home
safety_over_prop_default = 4 # in % from diameter of how much extra travel after propeller tip to reduce the risk of collision of probe to prop
max_number_of_samples_default = 10 #increase it if needed to do more samples
x_delta_default = 3 #measurement resolution in mm (half of the diameter of the Pitot' tube)
arm_length_default = 72.0 #torque arm length in mm
x_max_speed_default = 2000
y_max_speed_default = 2000
x_max_accel_default = 1000
y_max_accel_default = 800
aoa_trim_default = 85
aoa_limit_default = 50
aoss_trim_default = 90
aoss_limit_default = 50

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
    def __init__(self, parent=None, **kwargs):
        self.fig, self.ax = plt.subplots()
        super().__init__(self.fig)
        #self.ax.set_title("")
        self.ax.grid(True)
        # Dictionary to hold plot lines references
        self.plot_lines = {}

    def add_data(self, x, y, label, style='-'):
        if label in self.plot_lines:
            # Update existing line
            self.plot_lines[label].set_xdata(x)
            self.plot_lines[label].set_ydata(y)
        else:
            # Create a new line
            self.plot_lines[label], = self.ax.plot(x, y, style, label=label)
        self.ax.relim()  # Recalculate limits
        self.ax.autoscale_view(True, True, True)  # Autoscale
        self.draw_idle()  # Redraw the canvas

    def clear_plot(self):
        self.ax.clear()
        self.ax.grid(True)
        self.plot_lines.clear()  # Clear all references to old lines
        self.ax.legend()  # Reset the legend
        self.draw_idle()
        
    def save_plot(self, filename):
        self.fig.savefig(filename)  # Saves the figure to the specified file
        
# class Canvas(FigureCanvasQTAgg):
#     """
#     This "window" is a QWidget. If it has no parent, it
#     will appear as a free-floating window as we want.
#     """
#     def __init__(self):
#         super(Canvas, self).__init__(Figure())       
#         plt.ion()
#         self.theplot = plt.subplot(111)
#         self.x_axis = []
#         self.y1_axis = []
#         self.y2_axis = []
#         self.y3_axis = []
#         self.y4_axis = []  
#         
#     def add_to_list(self, x, y1, y2, y3, radius):
#         self.x_axis.append(int(x))
#         self.y1_axis.append(float(y1))
#         self.y2_axis.append(float(y2))
#         self.y3_axis.append(float(y3))
#         self.radius_axis = float(radius)
#         
#     def plot(self):   
#         self.theplot.plot(self.x_axis, self.y1_axis, marker=".")
#         self.theplot.plot(self.x_axis, self.y2_axis, marker="x")
#         self.theplot.plot(self.x_axis, self.y3_axis, marker="|")
#         self.theplot.plot(self.radius_axis, 0, marker="8")
#         self.theplot.xaxis.set_minor_locator(MultipleLocator(25))
#         self.theplot.yaxis.set_minor_locator(MultipleLocator(2))
#         plt.grid(color = 'g', linestyle = '--', linewidth = 0.2)
#         plt.grid(which = 'minor', linestyle = '-', linewidth = 0.1)
#         plt.legend(['V_tan (m/s)', 'V_rad (m/s)', 'V_axial (m/s)', 'Propeller tip'])
#         plt.show()
#         
#     def clear_plot(self):
#         self.theplot.clear()
#         self.x_axis = []
#         self.y1_axis = []
#         self.y2_axis = []
#         self.y3_axis = []
#         self.y4_axis = []

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
        self.right_lc.setValue(936.54)
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
        self.thr_lc.setValue(878.00)
        layout1.addWidget(self.thr_lc)
        
        self.confirm_button = QPushButton("Kinnita algparameetrid", self)
        self.confirm_button.clicked.connect(self.confirm_changes)
        #self.confirm_button.setEnabled(False)  # Initially disabled
        layout1.addWidget(self.confirm_button)
        
        self.label36 = QLabel("X-teljes mootori tsentri koordinaat (max käik) (mm)")
        layout1.addWidget(self.label36)
        
        self.x_center = QSpinBox()
        self.x_center.setMinimum(1)
        self.x_center.setMaximum(325)
        self.x_center.setSingleStep(1)
        self.x_center.setValue(self.shared_data.x_center)
        layout1.addWidget(self.x_center)
        
        self.label37 = QLabel("Y-telje maksimaalne käik (mm)" )
        layout1.addWidget(self.label37)
        
        self.y_max = QSpinBox()
        self.y_max.setMinimum(1)
        self.y_max.setMaximum(105)
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
        
        self.label10 = QLabel("X-telje positsioon mm")
        layout2.addWidget(self.label10)
        
        self.X_pos_map = QSpinBox()
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
        
        self.delete = QPushButton("Kustuta trajektoor mälust")
        self.delete.clicked.connect(self.delete_wp)
        layout2.addWidget(self.delete)
        
        self.wp_file = QPushButton("Salvesta trajektoori fail")
        self.wp_file.clicked.connect(self.save_wp_file)
        layout2.addWidget(self.wp_file)
        
        self.wp_load = QPushButton("Otsi olemasolev trajektoori fail")
        self.wp_load.clicked.connect(self.load_wp_file)
        layout2.addWidget(self.wp_load)
        
        #self.x_target = self.shared_data.x_center - self.X_pos_map.value()
        #self.current_Y = self.Y_pos_map.value()
        
        self.setLayout(layout2)
        
        #self.custom_trajectory = True
        self.r = 0
        
    def delete_wp(self):
        list_of_x_targets.clear()
        list_of_y_targets.clear()
        list_of_y_abs.clear()
        self.r = 0
        window.custom_trajectory = False
        
    def save_xy(self):
        window.custom_trajectory = True
        #print(window.custom_trajectory)
        first_y = 0
        if self.x_target not in list_of_x_targets:
            list_of_x_targets.append(self.x_target)
            list_of_y_abs.append(self.current_Y)
            first_y = list_of_y_abs[0]
            #y_relative = [r - first_y for r in list_of_y_abs]
            list_of_y_targets.append(list_of_y_abs[self.r] - first_y)
            self.r = self.r + 1
        #print(list_of_x_targets)
        #print(list_of_y_targets)
            
    def move_sensor_map(self):
        self.x_target = (self.shared_data.x_center - self.X_pos_map.value()) * self.shared_data.ratio
        self.current_Y = self.Y_pos_map.value() * self.shared_data.ratio
        map_data = 'j|%d|%d|%d|%d' % (self.x_target, self.current_Y, window.jog_speed.value(),window.jog_speed.value())
        self.sendData.emit(map_data)
        
    def save_wp_file(self):
        l = 0
        dialog = QFileDialog()
        #dialog.setDefaultSuffix("csv")
        traj_file = dialog.getSaveFileName(self, 'Salvesta trajektoori fail')
        #print(traj_file[0])
        if traj_file:
            while l < len(list_of_x_targets):
                coords = str(list_of_x_targets[l]) + ' ' + str(list_of_y_targets[l])
                l = l + 1
                with open(traj_file[0], 'a') as f:
                    w = csv.writer(f)
                    w.writerow([coords])
                    
    def load_wp_file(self):
        home_dir = str(Path.home())
        dialog =QFileDialog()
        wp_file = dialog.getOpenFileName(self, 'Otsi trajektoori faili', home_dir, "csv(*.csv)")
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
        
        self.aoss_limit = QSpinBox()
        self.aoss_limit.setMinimum(1)
        self.aoss_limit.setMaximum(80)
        self.aoss_limit.setSingleStep(1)
        self.aoss_limit.setValue(self.shared_data.aoss_limit)
        layout3.addWidget(self.aoss_limit)
        
        self.aoss_limit_button = QPushButton("Kinnita AoSS piirväärtus", self)
        self.aoss_limit_button.clicked.connect(self.send_aoss_limit)
        layout3.addWidget(self.aoss_limit_button)

        # Connect signals
        self.aoa_trim.valueChanged.connect(self.enable_aoa_trim_button)
        self.aoa_limit.valueChanged.connect(self.enable_aoa_limit_button)
        self.aoss_trim.valueChanged.connect(self.enable_aoss_trim_button)
        self.aoss_limit.valueChanged.connect(self.enable_aoss_limit_button)
        
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
            self.shared_data.aoss_limit = self.aoss_limit.value()
            aoss_limit_data = 'AoSSlim|%d' % (self.shared_data.aoss_limit)
            self.sendData.emit(aoss_limit_data)
            #self.aoss_limit_button.setEnabled(False)
            self.aoss_limit_button.setStyleSheet("background-color: None; color: None;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
        
class LC_calibration(QWidget):
    sendData = pyqtSignal(str)
    
    def __init__(self, shared_data):
        super().__init__()
        
        self.shared_data = shared_data
        self.cal_factor = QLabel()
        self.wait_for_cal_factor = False
        
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
        
        self.cal_factor.setText('000')
        
    def send_cal_left(self):
        try:
            cal_left_data = 'calLeft'
            self.sendData.emit(cal_left_data)
            self.cal_left_button.setEnabled(False)
            self.cal_left_button.setStyleSheet("background-color: yellow; color: black;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            
    def send_cal_right(self):
        try:
            cal_right_data = 'calRight'
            self.sendData.emit(cal_right_data)
            self.cal_right_button.setEnabled(False)
            self.cal_right_button.setStyleSheet("background-color: yellow; color: black;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
    
    def send_cal_thrust(self):
        try:
            cal_thrust_data = 'calThrust'
            self.sendData.emit(cal_thrust_data)
            self.cal_thrust_button.setEnabled(False)
            self.cal_thrust_button.setStyleSheet("background-color: yellow; color: black;")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            
    def send_known_mass(self):
        try:
            known_mass_data = 'calMass|%.2f' %(self.known_mass.value())
            self.sendData.emit(known_mass_data)
            self.known_mass_button.setStyleSheet("background-color: yellow; color: black;")
            self.wait_for_cal_factor = True
        except serial.SerialException as e:
            print(f"Error sending data: {e}") 

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
        self._aoss_limit = aoss_limit_default
        
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
    def aoss_limit(self):
        return self._aoss_limit

    @aoss_limit.setter
    def aoss_limit(self, value):
        self._aoss_limit = value

class SerialReader(QObject):
    serial_readout = pyqtSignal(int)

    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.update_timer = QTimer()
        self.latest_data = 0
        self.update_timer.timeout.connect(self.emit_data)
        self.update_timer.start(10)  # Emit updates every second
        self.running = True  # Flag to control the running of the loop

    def run(self):
        while self.running and self.controller and self.controller.isOpen():
            try:
                data = self.controller.readline().decode().strip()
                if data:
                    print("Data received:", data)  # Debug print
                    self.latest_data = data
                    self.sort_data(self.latest_data)
            except Exception as e:
                print(f"Error reading from serial: {e}")
                
    def sort_data(self, data):
        if (data == 'Emergency!'):
            window.e_stop = True
            window.update_emergency()
        if (data == 'OK'):
            window.e_stop = False
            window.update_emergency()
        if (window.motor_test == True):
            window.update_rpm_label(str(data))
        if window.motor_test == False:
            window.update_rpm_label('0')
        if (data == 'homing done'):
            window.homing_done = True
            window.homing.setStyleSheet("background-color: green; color: white;")
            window.measure.setEnabled(False)
            window.centering.setEnabled(True)
            window.testMotorButton.setEnabled(True)
            window.progress.setValue(0)
            window.test_progress.setValue(0)
        if (data == 'centering done'):
            window.centering.setStyleSheet("background-color: green; color: white;")
        if (data == 'CalVal:'):
            print("siin")

    def emit_data(self):
        if self.running:
            self.serial_readout.emit(self.latest_data)
    
    def stop(self):
        self.running = False

# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    def __init__(self, parent=None, **kwargs):
        super().__init__(parent, **kwargs)
        self.shared_data = SharedData()
        self.setupUI()
        self.controller = None
        self.serialReader = None
        self.serialReaderThread = QThread()
        
    def setupUI(self):

        self.measuring_stopped = True
        self.e_stop = False
        self.rpm = 0
        self.cnv = Canvas()
        self.counter = 0
        self.custom_trajectory = False
        self.w = None
        self.y = None
        self.update_plot()
        self.connection_ok = False
        self.rpm_label = QLabel()
        self.motor_test = False
        self.homing_done = False
        self.stop = 'stop'
        self.fileSelected = False
        
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        
        layout = QGridLayout(self.centralWidget)
        
        self.toolbar = QToolBar("Lisavalikud")
        self.addToolBar(self.toolbar)
        self.toolbar.addAction("AoA ja AoSS trimmimine", self.aoa_aoss_params)
        self.toolbar.addAction("Koormusandurite kalibreerimine", self.calibrate_loadcells)
        self.toolbar.addAction("Trajektoori valikud", self.map_)
        #self.toolbar.addAction("Update Plot", self.update_plot)
        self.toolbar.addAction("Tühjenda graafik", self.cnv.clear_plot)
        self.toolbar.addAction("Salvesta graafik", self.save_plot)
        
        #layout = QVBoxLayout()
        #layout = QGridLayout()
        #widget = QWidget()
        #widget.setLayout(layout)

        # Set the central widget of the Window. Widget will expand
        # to take up all the space in the window by default.
        #self.setCentralWidget(self.centralWidget)
        
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
        
        self.label7 = QLabel("1.Eemalda Pitot' kate kohe pärast stendi sisselülitamist!" )
        #layout.addWidget(self.label7, 0, 0, 1, 1)
        
        self.label8 = QLabel("2.Ühenda aku enne mõõdistamise algust!" )
        #layout.addWidget(self.label8, 1, 0, 1, 1)
        
        self.label8 = QLabel("------------------" )
        #layout.addWidget(self.label8)
        
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
        
        self.Y_move = QPushButton("Liiguta Y-telge")
        self.Y_move.clicked.connect(self.moveY)
        self.Y_move.setEnabled(False)
        layout.addWidget(self.Y_move, 13, 0, 1, 1)
        
        self.label4 = QLabel("Propelleri kiirus (%)")
        layout.addWidget(self.label4, 14, 0, 1, 1)
        
        self.throttle = QDoubleSpinBox()
        self.throttle.setMinimum(10.0)
        self.throttle.setMaximum(100.0)
        self.throttle.setValue(20.0)
        self.throttle.setSingleStep(0.1)
        layout.addWidget(self.throttle, 15, 0, 1, 1)
        
        self.testMotorButton = QPushButton("Testi mootorit", self)
        self.testMotorButton.setEnabled(False)
        self.testMotorButton.setCheckable(True)
        self.testMotorButton.clicked.connect(self.toggle_motor)
        layout.addWidget(self.testMotorButton, 16, 0, 1, 1)
        
        self.labelRPM = QLabel("Mõõdetud pöörded", self)  # Initialize the RPM label
        layout.addWidget(self.labelRPM, 17, 0, 1, 1)
        
        layout.addWidget(self.rpm_label, 18, 0, 1, 1)
        
        self.label5 = QLabel("Mõõdistamise kiirus")
        layout.addWidget(self.label5, 19, 0, 1, 1)
        
        self.measure_speed = QSpinBox()
        self.measure_speed.setMinimum(200)
        self.measure_speed.setMaximum(500)
        self.measure_speed.setSingleStep(100)
        self.measure_speed.setValue(300)
        layout.addWidget(self.measure_speed, 20, 0, 1, 1)
        
        self.label11 = QLabel("Kordusmõõtmiste arv")
        layout.addWidget(self.label11, 21, 0, 1, 1)
        
        self.sweep_count = QSpinBox()
        self.sweep_count.setMinimum(1)
        self.sweep_count.setMaximum(max_number_of_samples_default)
        self.sweep_count.setSingleStep(1)
        self.sweep_count.setValue(1)
        layout.addWidget(self.sweep_count, 22, 0, 1, 1)
        
        self.measure = QPushButton("Alusta mõõtmisega")
        self.measure.setEnabled(False)
        self.measure.clicked.connect(self.start_measuring)
        layout.addWidget(self.measure, 23, 0, 1, 1)
        
        self.label6 = QLabel("Mõõdistamise kulg")
        layout.addWidget(self.label6, 24, 0, 1, 1)
        
        self.progress = QProgressBar()
        self.progress.setMinimum(0)
        self.progress.setMaximum(100)
        self.progress.setValue(0)
        layout.addWidget(self.progress, 25, 0, 1, 1)
        
        self.test_progress = QProgressBar()
        self.test_progress.setMinimum(0)
        self.test_progress.setMaximum(self.sweep_count.value())
        self.test_progress.setValue(0)
        layout.addWidget(self.test_progress, 26, 0, 1, 1)
        
        self.back = QPushButton("Pitot' tagasi algasendisse")
        self.back.setEnabled(False)
        self.back.clicked.connect(self.come_back)
        layout.addWidget(self.back, 27, 0, 1, 1)
        
        #self.label3 = QLabel("Propelleri pöörded")
        #layout.addWidget(self.label3)
        
        #self.rpm = QLabel()
        #layout.addWidget(self.rpm)
                
        #self.label5 = QLabel("Vajuta, kui propelleri suund on vastupäeva")
        #layout.addWidget(self.label5)
        
        #self.ccw = QRadioButton()
        #layout.addWidget(self.ccw)
        
        self.danger = QLabel("Emergency!")
        self.danger.setAlignment(Qt.AlignCenter)
        self.danger.setStyleSheet("background-color: None")
        layout.addWidget(self.danger, 28, 0, 1, 1)

        self.label12 = QLabel("Average inductive speed:")
        #layout.addWidget(self.label12)
        
        self.label13 = QLabel()
        self.label13.setText(" ")
        self.label13.setStyleSheet("border: 1px solid black;")
        #layout.addWidget(self.label13)
        
        self.label14 = QLabel("Induced power:")
        layout.addWidget(self.label14, 0, 3)
        
        self.label15 = QLabel()
        self.label15.setText(" ")
        self.label15.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.label15, 1, 3)
        
        self.label16 = QLabel("Total power:")
        layout.addWidget(self.label16, 2, 3)
        
        self.label17 = QLabel()
        self.label17.setText(" ")
        self.label17.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.label17, 3, 3)
        
        self.label18 = QLabel("Propeller efficiency:")
        layout.addWidget(self.label18, 4, 3)
        
        self.label19 = QLabel()
        self.label19.setText(" ")
        self.label19.setStyleSheet("border: 1px solid black;")
        layout.addWidget(self.label19, 5, 3)
        
        self.update_ports()
        self.initTimer()
        self.reset_button()
        self.last_throttle_value = self.throttle.value()
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_values)
        
    def sendData(self, data):
        if self.controller:
            try:
                encoded_data = data.encode('utf-8')
                self.controller.write(encoded_data)
                print(data)
            except serial.SerialException as e:
                print(f"Error sending data: {e}")
        
    def toggle_motor(self):
        if self.testMotorButton.isChecked():
            self.testMotorButton.setText("Seiska mootor")
            self.test_motor()
        else:
            self.testMotorButton.setText("Testi mootorit")
            self.stop_motor()

    def test_motor(self):
        self.update_throttle_test()
        self.timer.start(1000)

    def stop_motor(self):
        self.sendData('stop')
        self.timer.stop()
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

            self.serialReader.serial_readout.connect(self.update_rpm_label)
            self.serialReaderThread.started.connect(self.serialReader.run)

            self.serialReaderThread.start()

    def update_rpm_label(self, rpm):
        if self.motor_test == True:
            self.rpm_label.setText(rpm)
        else:
            self.rpm_label.setText('0')
        
    def save_plot(self):
        fileName, _ = QFileDialog.getSaveFileName(self, "Save Plot", "",
                      "PNG Files (*.png);;JPEG Files (*.jpg);;All Files (*)", options=QFileDialog.Options())
        if fileName:
            self.cnv.save_plot(fileName)
        
    def update_plot(self):
        # Example data
        x = list(range(10))
        y1 = [i ** 2 for i in x]
        y2 = [i ** 1.5 for i in x]
        self.cnv.add_data(x, y1, label="x squared", style='r-')
        self.cnv.add_data(x, y2, label="x to the 1.5", style='b--')
        
    def initTimer(self):
        self.timer = QTimer(self)
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.update_ports)
        self.timer.start()
        
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
                self.connect.setStyleSheet("background-color: green; color: white;")
                self.connect.setText("Ühendatud")
                self.serialReaderThread.start()
                
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
        self.fname = QFileDialog.getOpenFileName(self, 'Otsi faili', home_dir, "csv(*.csv)")
        if self.fname:
            self.fileSelected = True
            self.homing.setEnabled(True)
            self.file.setStyleSheet("background-color: green; color: white;")
        
    def update_emergency(self):
        if self.e_stop == True:
            self.testMotorButton.setChecked(False)
            self.toggle_motor()
            self.testMotorButton.setEnabled(False)
            self.measuring_stopped = True
            self.danger.setStyleSheet("background-color: red")
            self.measure.setEnabled(False)
            self.centering.setEnabled(False)
            self.back.setEnabled(False)
            self.homing.setStyleSheet("background-color: None; color: None;")
        else:
            self.e_stop = False
            self.danger.setStyleSheet("background-color: None")
            
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
        
    def home(self):
        self.homing_done = False
        self.homing.setStyleSheet("background-color: None")
        self.sendData('h')
        if (self.homing_done == True):
            self.homing.setStyleSheet("background-color: green; color: white;")
            self.measure.setEnabled(False)
            self.centering.setEnabled(True)
            self.testMotorButton.setEnabled(True)
            self.progress.setValue(0)
            self.test_progress.setValue(0)
            self.sendData('tare')
        
    def center(self):
        self.back.setEnabled(True)
        self.Y_move.setEnabled(True)
        self.measure.setEnabled(True)
        self.sendData('c')
        
    def come_back(self):
        self.sendData('stop')
        self.centering.setStyleSheet("background-color: None; color: None;")
        time.sleep(2)
        self.measure.setEnabled(False)
        self.progress.setValue(0)
        self.test_progress.setValue(0)
        self.counter = 0
        jog_home = 'j|%d|%d|%d|%d' % (0, 0, self.jog_speed.value(), self.jog_speed.value())
        self.sendData(jog_home)
        
    def moveY(self):
        moveY = 'j|%d|%d|%d|%d' %((self.shared_data.x_center * self.shared_data.ratio), (self.Y_pos.value() * self.shared_data.ratio), self.jog_speed.value(), self.jog_speed.value())
        self.sendData(moveY)
        
    def start_measuring(self):
        self.sendData('streamStart')
        if self.counter == 0:
            self.today_dt = datetime.datetime.today().strftime('%d-%m-%Y-%H:%M:%S')
            self.parent_dir = "/home/siim/Desktop/logid/"
            self.path = os.path.join(self.parent_dir, self.today_dt)
            os.mkdir(self.path)
            self.csvfile = "log" + self.today_dt + ".csv"
            self.header = ['Prop_diam(inch) ' 'X_position(mm) ' 'Y_position(mm) ' 'Torque(Nm) ' 'Thrust(N) ' 'Omega(rad/s) ' 'Airspeed(m/s) ' 'AoA(deg) ' 'AoSS(deg) ' 'V_tan(m/s) ' 'V_rad(m/s) ' 'V_axial(m/s)']
        self.cnv.clear_plot()
        aoss_a = []
        aoa_a = []
        arspd_a = []
        omega_a = []
        x_prev = 0
        omega = 0
        self.label13.clear()
        self.label15.clear()
        self.label17.clear()
        self.label19.clear()
        self.measure.setEnabled(False)
        self.Y_move.setEnabled(False)
        self.measuring_stopped = False
        self.test_progress.setMaximum(self.sweep_count.value())
        header_added = False
        self.radius = format(((self.prop.value()*25.4)/2),'.1f')
        x_goal = x_center - float(self.radius) - safety_over_prop
        self.sendData('tare')
        self.progress.setValue(1)
        time.sleep(3)
        self.progress.setValue(5)
        throttle = int(1000 + (self.throttle.value() * 10))
        start = f'test|{throttle}'
        self.sendData(start)
        time.sleep(12)
        self.progress.setValue(10)
        if self.custom_trajectory == False and self.counter == 0:
            list_of_x_targets.append(x_goal)
            list_of_y_targets.append(0)
        b = 0
        self.controller.write(str.encode('m|%d|%d|%d|%d' %(list_of_x_targets[b], list_of_y_targets[b] + self.Y_pos.value(), self.measure_speed.value(), self.measure_speed.value()/3)))
        while 1:
            if (self.measuring_stopped == False):
                    raw_sensors = self.read_sensors()
                    if "Measurements" in raw_sensors:
                        #print('mõõdan')
                        handled_s = raw_sensors.split(" ")
                        x = handled_s[1]
                        y = handled_s[2]
                        trq = handled_s[3]
                        thr = handled_s[4]
                        rpm_time = handled_s[5]
                        if 1500 <= int(rpm_time) <= 65000:
                            rpms = (60000000/int(rpm_time))
                            omega = format((((2.0 * math.pi)/60.0) * rpms),'.2f')
                        tr = int (x_center - x_goal)
                        x_mm = (x_center - int(format((float(x)/ratio),'.0f')))
                        x_progress = round((x_mm/tr)*99,0)
                        self.progress.setValue(10 + int(x_progress))
                        x_current = int((float(x)/ratio))
                        y_mm = int((float(y)/ratio))
                        #raw_pitot = p
                        #handled_p = raw_pitot.split(",")
                        arspd = 0 #handled_p[2]
                        aoa = 0 #handled_p[4]
                        aoss = 0 #handled_p[5]
                        if (x_mm == 0):
                            aoss_zero = format((float(aoss) + (float(0 - self.aoss_pre.value()))),'.2f')
                            aoa_zero = format((float(aoa) + float(self.aoa_pre.value())),'.2f')
                            omega_zero = format(float(omega),'.2f')
                            arspd_zero = format(float(arspd),'.2f')
                            v_tan = format(0,'.2f')
                            v_rad = format(0,'.2f')
                            v_axial = format(0,'.2f')
                            data_zero = str(str(format(self.prop.value(),'.1f'))+" "+str(x_mm)+" "+str(y_mm)+" "+trq+" "+thr+" "+str(omega_zero)+" "+arspd_zero+" "+aoa_zero+" "+aoss_zero+" "+str(v_tan)+" "+str(v_rad)+" "+str(v_axial))
                            with open(os.path.join(self.path,self.csvfile), 'a') as f:
                                w = csv.writer(f)
                                if not header_added:
                                    w.writerow(self.header)
                                    header_added = True
                                w.writerow([data_zero])
                        if (x_mm - x_prev < x_delta):
                            aoss_a.append((float(aoss) + float(0 - self.aoss_pre.value())))
                            aoa_a.append((float(aoa) + float(self.aoa_pre.value())))
                            arspd_a.append(float(arspd))
                            omega_a.append(float(omega))
                        if (x_mm - x_prev == x_delta):
                            aoss_avg = format(statistics.mean(aoss_a), '.2f')
                            aoa_avg = format(statistics.mean(aoa_a),'.2f')
                            arspd_avg = format(statistics.mean(arspd_a),'.2f')
                            omega_avg = format(statistics.mean(omega_a),'.2f')
                            v_tan = format(float(arspd_avg) * math.sin(math.radians(float(aoa_avg))),'.2f')
                            v_rad = format(float(arspd_avg) * math.cos(math.radians(float(aoa_avg))) * math.sin(math.radians(float(aoss_avg))),'.2f') 
                            v_axial = format(float(arspd_avg) * math.cos(math.radians(float(aoa_avg))) * math.cos(math.radians(float(aoss_avg))),'.2f') 
                            data = str(str(format(self.prop.value(),'.1f'))+" "+str(x_mm)+" "+str(y_mm)+" "+trq+" "+thr+" "+str(omega_avg)+" "+arspd_avg+" "+aoa_avg+" "+aoss_avg+" "+str(v_tan)+" "+str(v_rad)+" "+str(v_axial))
                            print(data)
                            with open(os.path.join(self.path,self.csvfile), 'a') as f:
                                w = csv.writer(f)
                                if not header_added:
                                    w.writerow(self.header)
                                    header_added = True
                                w.writerow([data])
                            x_prev = x_mm
                            aoss_a.clear()
                            aoa_a.clear()
                            arspd_a.clear()
                            omega_a.clear()
                    if (raw_sensors == "Measuring done"):
                        last_x = list_of_x_targets[b]
                        last_y = list_of_y_targets[b]
                        #print(b, len(list_of_x_targets))
                        if b < len(list_of_x_targets):
                            controller.write(str.encode('m|%d|%d|%d|%d' %(list_of_x_targets[b], list_of_y_targets[b] + self.Y_pos.value(), self.measure_speed.value(), self.measure_speed.value()/3)))
                            #print(list_of_x_targets[b], list_of_y_targets[b])
                            b = b + 1
                        if b == len(list_of_x_targets):
                            controller.write(str.encode('j|%d|%d|%d|%d' %(last_x - safety_over_prop, last_y + self.Y_pos.value(), self.measure_speed.value(), self.measure_speed.value()/3)))
                            #time.sleep(.1)
                            while 1:
                                arrived = self.read_sensors()
                                #print(arrived)
                                #print('lõpp')
                                if ("Measuring done" in arrived):
                                    #print('siin')
                                    #self.stop_measuring()
                                    self.test_progress.setValue(self.counter + 1)
                                    self.counter = self.counter + 1
                                    self.stop_measuring(self.counter, last_x - safety_over_prop)
                                    break
                            break
                    if (raw_sensors == "Emergency!"):
                        controller.write(str.encode('stop'))
                        self.e_stop = True
                        self.update_emergency()
                        break
        
    def stop_measuring(self, done, x_curr):
        if done == self.sweep_count.value():
            self.measuring_stopped = True
            self.Y_move.setEnabled(False)
            self.measure.setEnabled(False)
            self.controller.write(str.encode('stop'))
            time.sleep(2)
            self.come_back()
            self.process_data()
        else:
            self.Y_move.setEnabled(False)
            self.measure.setEnabled(False)
            time.sleep(2)
            self.controller.write(str.encode('stop'))
            time.sleep(1)
            self.controller.write(str.encode('j|%d|0|%d|%d' %(x_curr,self.jog_speed.value(),self.jog_speed.value())))
            while 1:
                location = self.read_sensors()
                if ("In position" in location):
                    break
            #time.sleep(5)
            self.controller.write(str.encode('c'))
            #stepper.write(str.encode('j %d 0 %d %d' %(x_center,self.jog_speed.value(),self.jog_speed.value())))
            while 1:
                location = self.read_sensors()
                print(location)
                if ("Pitot centered" in location):
                    break
            #time.sleep(10)
            self.controller.write(str.encode('j|%d|%d|%d|%d' %(x_center, self.Y_pos.value(),self.jog_speed.value(),self.jog_speed.value())))
            while 1:
                location = self.read_sensors()
                if ("In position" in location):
                    break
            #time.sleep(10)
            #list_of_x_targets.clear()
            self.start_measuring()
            
    def process_data(self):
        mean_csvfile = "log" + self.today_dt + "_mean.csv"
        mean_header = ['#_of_samples ' 'Prop_diam(inch) ' 'X_position(mm) ' 'Y_position(mm) ' 'Torque(Nm) ' 'Thrust(N) ' 'Omega(rad/s) '
                       'Airspeed(m/s) ' 'AoA(deg) ' 'AoSS(deg) ' 'V_tan(m/s) ' 'V_rad(m/s) ' 'V_axial(m/s) ' 'Chord_angle(deg) '
                       'Chord_length(mm) ' 'Helix_angle(deg) ' 'Alpha_angle(deg) ' 'V_total(m/s) ' 'V_lift(m/s) ' 'V_drag(m/s) '
                       'CL ' 'CD ' 'Reynolds_number']       
        with open(os.path.join(self.path,mean_csvfile), 'a') as h:
            k = csv.writer(h)
            k.writerow(mean_header)
        x_max = 3 * math.floor((float(self.radius) + safety_over_prop)/3)
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
            Re = (float(chord_length)/1000 * float(total_speed))/(float(kin_visc) * math.pow(10,-5))
            mean_data.append(str(format(Re,'.0f')))

            with open(os.path.join(self.path,mean_csvfile), 'a') as f:
                w = csv.writer(f)
                w.writerow([' '.join(mean_data)])
            
            self.cnv.add_to_list(x_pos, v_tan_mean, v_rad_mean, v_axial_mean, float(window.radius))
            
            x_mp = x_mp + 3
            
        vi = (2*(x_delta/1000)*sum(var_list))/((float(self.radius)/1000)*(float(self.radius)/1000))
        self.label13.setText(str(format(vi,'.2f')))
        T = statistics.mean(thr_list)
        Pi = float(vi)*float(T)
        self.label15.setText(str(format(Pi,'.2f')))
        try:
            vv = float(T)/(rho * math.pi * math.pow(float(self.radius)/1000,2) * math.pow(float(vi),2))  #(2*math.pi*(x_delta/1000)*sum(var_list))
        except:
            vv = 0
        M = statistics.mean(trq_list)
        o = statistics.mean(omega_list)
        P = float(M)*float(o)
        self.label17.setText(str(format(P,'.2f')))
        vm = rho * math.pi * math.pow(float(self.radius)/1000,2) * float(vi)
        try:
            v_max_mean = float(T)/float(vm)
        except:
            v_max_mean = 0
        try:
            Ct = float(T)/(rho * math.pow(float(o),2) * math.pow(((float(self.radius)*2)/1000),4))
        except:
            Ct = 0
        try:
            Cp = float(M)/(rho * math.pow(float(o),2) * math.pow(((float(self.radius)*2)/1000),5))
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
                w.writerow(['V_mass ',format(vm,'.2f'),' m/s'])
                w.writerow(['V_max_mean ',format(v_max_mean,'.2f'),' m/s'])
                w.writerow(['Ct ',format(Ct,'.5f')])
                w.writerow(['Cp ',format(Cp,'.5f')])
                w.writerow(['Air_density ',rho,' kg/m3'])
                w.writerow(['Air_kinematic_viscosity ',kin_visc,' x10-5 m2/s'])
        
        self.cnv.plot()
        
        var_list.clear()
        trq_list.clear()
        thr_list.clear()
        omega_list.clear()
    def closeEvent(self, event):
        if self.serialReader:
            self.serialReader.stop()

        # Stop the thread safely
        if self.serialReaderThread.isRunning():
            self.serialReaderThread.quit()  # Signals the thread to quit
            self.serialReaderThread.wait()  # Waits for the thread to actually quit

        # Close the serial port
        if self.controller and self.controller.isOpen():
            self.controller.close()

        # Proceed with the normal closure of the window
        super().closeEvent(event)
            
app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec_()