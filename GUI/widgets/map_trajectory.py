import sys
import csv
import os
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QDoubleSpinBox, QSpinBox, QPushButton, QFileDialog, QMessageBox
import serial
from pathlib import Path
import app_globals

list_of_x_targets: list[int] = []
list_of_y_targets: list[int] = []   # relative steps
list_of_y_abs:     list[int] = []

class MapTrajectory(QWidget):
    sendData = pyqtSignal(str)
    modeChanged = pyqtSignal(bool)
    
    def __init__(self, shared_data):
        super().__init__()
        
        self.shared_data = shared_data
        
        self._wp_dirty = False 
        
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
        
        
    def _set_wp_status(self, state: str):
            """Set visual status for the 'Salvesta trajektoori fail' button."""
            if state == "saved":
                self.wp_file.setStyleSheet("background-color: green; color: white;")
                self._wp_dirty = False
            elif state == "dirty":
                self.wp_file.setStyleSheet("background-color: orange; color: black;")
                self._wp_dirty = True
            else:
                # neutral
                self.wp_file.setStyleSheet("background-color: None; color: None;")
                self._wp_dirty = False

            # save handle for use in other methods
            #self._set_wp_status = _set_wp_status

            # initial neutral state
            #self._set_wp_status("neutral")
        
    def enable_move_sensor_button(self):
        self.sensor_move_map.setStyleSheet("background-color: orange; color: black;")

    def delete_wp(self):
        """Clear trajectory from memory and notify MainWindow + user."""
        list_of_x_targets.clear()
        list_of_y_targets.clear()
        list_of_y_abs.clear()
        self.r = 0
        # Mirror into MainWindow so start_measuring() falls back to default
        try:
            app_globals.window.custom_trajectory = False
            app_globals.window.list_of_x_targets = list(list_of_x_targets)
            app_globals.window.list_of_y_targets = list(list_of_y_targets)
        except Exception:
            pass
        try:
            QMessageBox.information(self, "Trajektoor eemaldatud",
                                    "Trajektoor kustutati mälust. Järgmine mõõtmine kasutab vaikimisi trajektoori.")
            self._set_wp_status("neutral")
        except Exception:
            pass
        finally:
            self.modeChanged.emit(False)

        
    def save_xy(self):
        self.save.setStyleSheet("background-color: None; color: None;")
        app_globals.window.custom_trajectory = True
        app_globals.window.list_of_x_targets = list(list_of_x_targets)
        app_globals.window.list_of_y_targets = list(list_of_y_targets)
        first_y = 0
        if self.x_target not in list_of_x_targets:
            list_of_x_targets.append(self.x_target)
            list_of_y_abs.append(self.current_Y)
            first_y = list_of_y_abs[0]
            list_of_y_targets.append(list_of_y_abs[self.r] - first_y)
            self.r = self.r + 1
            self._set_wp_status("dirty")
            self.modeChanged.emit(True)
            
    def move_sensor_map(self):
        self.sensor_move_map.setStyleSheet("background-color: None; color: None;")
        self.x_target = int((self.shared_data.x_center - self.X_pos_map.value()) * self.shared_data.ratio)
        self.current_Y = int(self.Y_pos_map.value() * self.shared_data.ratio)
        self.sendData.emit('j|%d|%d|%d|%d' % (self.x_target, self.current_Y, app_globals.window.jog_speed.value(),app_globals.window.jog_speed.value()))
        self.save.setStyleSheet("background-color: orange; color: black;")
        self._set_wp_status("dirty")
    
#     def save_wp_file(self):
#         if (fp := QFileDialog().getSaveFileName(self, 'Salvesta trajektoori fail')[0]):
#             with open(fp if fp.endswith('.csv') else fp + '.csv', 'w', newline='') as f:
#                 w = csv.writer(f, delimiter=' ')
#                 for x, yrel in zip(list_of_x_targets, list_of_y_targets):
#                     w.writerow([x, yrel])

    # --- Helpers for robust parsing ---
    def _steps_per_mm(self) -> float:
        try:
            return float(getattr(self.shared_data, "ratio", 1.0) or 1.0)
        except Exception:
            try:
                return float(getattr(getattr(app_globals.window, "shared_data", object()), "ratio", 1.0) or 1.0)
            except Exception:
                return 1.0

    def _parse_num_to_steps(self, token: str) -> int:
        """
        Accept integers/decimals, optional unit suffix 'mm' or 'steps'.
        If unit missing: decimal -> mm, integer -> steps.
        Examples: '123', '123.5', '2,2', '40 mm', '1200 steps'
        """
        import re as _re
        t = token.strip()
        t_norm = t.replace(',', '.')
        m = _re.fullmatch(r'([+-]?\d+(?:\.\d+)?)(?:\s*(mm|steps?))?$', t_norm, flags=_re.I)
        if not m:
            raise ValueError(f"Vigane number: “{token}”")
        val = float(m.group(1))
        unit = (m.group(2) or '').lower()
        spmm = self._steps_per_mm()
        if unit.startswith('mm') or ('.' in t_norm):
            return int(round(val * spmm))
        return int(round(val))

    def load_wp_file(self):
        default_dir = self._get_default_wp_dir()
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Laadi trajektoori fail",
            default_dir,
            "csv(*.csv)"
        )
        if not path:
            QMessageBox.information(self, "Tühistatud", "Faili ei laetud.")
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                r = csv.reader(f, delimiter=' ')
                list_of_x_targets.clear()
                list_of_y_targets.clear()
                for row in r:
                    list_of_x_targets.append(int(row[0]))
                    list_of_y_targets.append(int(row[1]))
            QMessageBox.information(self, "Laetud", f"Fail: {path}\nPunkte: {len(list_of_x_targets)}")
            #self._set_wp_status("saved")
            app_globals.window.custom_trajectory = len(list_of_x_targets) > 0
            self.modeChanged.emit(app_globals.window.custom_trajectory) 
        except Exception as e:
            QMessageBox.critical(self, "Viga laadimisel", str(e))

    def _get_default_wp_dir(self):
        """Return the Desktop/trajektoorid folder, create if missing."""
        base = Path.home() / "Desktop" / "trajektoorid"
        os.makedirs(base, exist_ok=True)
        return str(base)

    def save_wp_file(self):
        default_dir = self._get_default_wp_dir()
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Salvesta trajektoori fail",
            str(Path(default_dir) / "trajektoor.csv"),
            "csv(*.csv)"
        )
        if not path:
            QMessageBox.information(self, "Tühistatud", "Faili ei salvestatud.")
            return
        if not path.endswith(".csv"):
            path += ".csv"
        try:
            with open(path, "w", newline='', encoding="utf-8") as f:
                w = csv.writer(f, delimiter=' ')
                for x, yrel in zip(list_of_x_targets, list_of_y_targets):
                    w.writerow([int(x), int(yrel)])
            QMessageBox.information(self, "Salvestatud", f"Fail: {path}\nPunkte: {len(list_of_x_targets)}")
            self._set_wp_status("saved")
        except Exception as e:
            QMessageBox.critical(self, "Viga salvestamisel", str(e))
