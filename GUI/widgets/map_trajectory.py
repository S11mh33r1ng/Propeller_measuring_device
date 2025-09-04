import sys
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QDoubleSpinBox, QSpinBox, QPushButton
import serial
import app_globals

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
        self.centering.clicked.connect(app_globals.window.center)
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
        app_globals.window.custom_trajectory = False
        
    def save_xy(self):
        self.save.setStyleSheet("background-color: None; color: None;")
        app_globals.window.custom_trajectory = True
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
        self.sendData.emit('j|%d|%d|%d|%d' % (self.x_target, self.current_Y, app_globals.window.jog_speed.value(),app_globals.window.jog_speed.value()))
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
                app_globals.window.custom_trajectory = True
                with open(wp_file[0], newline = '') as wpfile:
                    read_wps = csv.reader(wpfile, delimiter = ' ')
                    for wp in read_wps:
                       #wp_line = ' '.join(wp)
                        sample_wp = list(wp)
                        list_of_x_targets.append(int(sample_wp[0]))
                        list_of_y_targets.append(int(sample_wp[1]))
                        print(list_of_x_targets, list_of_y_targets)
        except:
            app_globals.window.custom_trajectory = False
            pass