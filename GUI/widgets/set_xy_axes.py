import sys
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QDoubleSpinBox, QSpinBox, QPushButton, QRadioButton, QGroupBox, QHBoxLayout
import serial
import app_globals

class SetXYAxes(QWidget):
    sendData = pyqtSignal(str)
    centerStepsChanged = pyqtSignal(int)

    def __init__(self, shared_data):
        super().__init__()
        self.shared_data = shared_data
        layout1 = QVBoxLayout(); self.setLayout(layout1)
        self.setWindowTitle("Telgede sättimine")
        
        rot_group = QGroupBox("Pöörlemissuund")
        rot_layout = QHBoxLayout(rot_group)
        self.rot_cw  = QRadioButton("CW (päripäeva)")
        self.rot_ccw = QRadioButton("CCW (vastupäeva)")
        rot_layout.addWidget(self.rot_cw)
        rot_layout.addWidget(self.rot_ccw)
        layout1.addWidget(rot_group)

        # default from shared_data if present
        dir0 = getattr(self.shared_data, "rotation_dir", 0)
        if dir0 == 1: self.rot_cw.setChecked(True)
        elif dir0 == -1: self.rot_ccw.setChecked(True)

        # ensure confirm stays disabled until a choice is made
        def _on_rotation_changed():
            self.enable_confirm_axis_button()
        self.rot_cw.toggled.connect(_on_rotation_changed)
        self.rot_ccw.toggled.connect(_on_rotation_changed)
        
        self.label33 = QLabel("Turvaala suurus (% propelleri raadiusest)")
        layout1.addWidget(self.label33)
        
        self.safety_o_prop = QSpinBox()
        self.safety_o_prop.setMinimum(0)
        self.safety_o_prop.setMaximum(15)
        self.safety_o_prop.setSingleStep(1)
        self.safety_o_prop.setValue(self.shared_data.safety_over_prop)
        layout1.addWidget(self.safety_o_prop)
        
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
        
        self.label30 = QLabel("Stepperite ülekandearvu (sammu/mm)")
        layout1.addWidget(self.label30)
        
        self.ratio = QDoubleSpinBox()
        self.ratio.setMinimum(0)
        self.ratio.setDecimals(4)
        self.ratio.setSingleStep(0.0001)
        self.ratio.setValue(self.shared_data.ratio)
        layout1.addWidget(self.ratio)
        
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
        self.confirm_axis_button.setEnabled(False)
        layout1.addWidget(self.confirm_axis_button)
        
        self.safety_o_prop.valueChanged.connect(self.enable_confirm_axis_button)
        self.x_delta.valueChanged.connect(self.enable_confirm_axis_button)
        self.max_number_of_samples.valueChanged.connect(self.enable_confirm_axis_button)
        self.ratio.valueChanged.connect(self.enable_confirm_axis_button)
        self.x_center.valueChanged.connect(self.enable_confirm_axis_button)
        self.y_max.valueChanged.connect(self.enable_confirm_axis_button)
        self.x_max_speed.valueChanged.connect(self.enable_confirm_axis_button)
        self.y_max_speed.valueChanged.connect(self.enable_confirm_axis_button)
        self.x_max_accel.valueChanged.connect(self.enable_confirm_axis_button)
        self.y_max_accel.valueChanged.connect(self.enable_confirm_axis_button)
        
    def enable_confirm_axis_button(self):
        self.confirm_axis_button.setEnabled(True)
        self.confirm_axis_button.setStyleSheet("background-color: orange; color: black;")
    
    def confirm_axis_changes(self):
        try:
            if not (self.rot_cw.isChecked() or self.rot_ccw.isChecked()):
                # require explicit choice
                self.confirm_axis_button.setEnabled(False)
                #self.confirm_axis_button.setStyleSheet("background-color: ; color: white;")
                return

            self.shared_data.rotation_dir = 1 if self.rot_cw.isChecked() else -1
        
            self.shared_data.safety_over_prop = self.safety_o_prop.value()
            self.shared_data.x_delta = self.x_delta.value()
            self.shared_data.max_number_of_samples = self.max_number_of_samples.value()
            self.shared_data.ratio = self.ratio.value()
            self.shared_data.x_center = self.x_center.value()
            self.shared_data.y_max = self.y_max.value()
            self.shared_data.x_max_speed = self.x_max_speed.value()
            self.shared_data.y_max_speed = self.y_max_speed.value()
            self.shared_data.x_max_accel = self.x_max_accel.value()
            self.shared_data.y_max_accel = self.y_max_accel.value()
            
            limit_data = 'l|%d|%d|%d|%d|%d|%d' % (
                (self.shared_data.x_center * self.shared_data.ratio), (self.shared_data.y_max * self.shared_data.ratio),
                self.shared_data.x_max_speed, self.shared_data.y_max_speed, self.shared_data.x_max_accel,
                self.shared_data.y_max_accel
                )
            self.sendData.emit(limit_data)
            
            x_center_steps = int(self.shared_data.x_center * self.shared_data.ratio)
            self.centerStepsChanged.emit(x_center_steps)
            
            self.confirm_axis_button.setEnabled(True)
            self.confirm_axis_button.setStyleSheet("background-color: None; color: None;")
            if app_globals.window.tandem_setup:
                app_globals.window.file.setEnabled(False)
            else:
                app_globals.window.file.setEnabled(True)
            QTimer.singleShot(1000, self.close)
            app_globals.window.aoa_aoss_action.setEnabled(True)
            app_globals.window.calibrate_first_loadcells_action.setEnabled(True)
            if app_globals.window.tandem_setup == True:
                app_globals.window.calibrate_second_loadcells_action.setEnabled(True)
                app_globals.window.rpm_second_controller_action.setEnabled(True)
            app_globals.window.rpm_first_controller_action.setEnabled(True)
            app_globals.window.map_action.setEnabled(True)
            app_globals.window.clear_plot_action.setEnabled(True)
            app_globals.window.save_plot_action.setEnabled(True)
            QTimer.singleShot(1000, self.close)
            app_globals.window.xy_axes.setStyleSheet("background-color: green; color: white;")
            app_globals.window.xy_axes.setText("Säti teljed ✓")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
