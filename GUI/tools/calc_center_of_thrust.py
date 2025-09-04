from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QFileDialog
from PyQt5.QtCore import QTimer
import csv, math, numpy as np
from scipy.integrate import quad
from pathlib import Path

class Calculate_center_of_thrust(QWidget):
    def __init__(self):
        super().__init__()
        
        layout6 = QVBoxLayout()
        self.setLayout(layout6)
        
        self.setWindowTitle("Õhuvoo monitoorimise lisaarvutused")
        
        # Variables to track file selection
        self.zero_file_selected = False
        self.point_eight_file_selected = False
        self.zero_log_file_name = None
        self.point_eight_log_file_name = None

        self.zero_log_file_button = QPushButton("Vali 0R logi fail")
        self.zero_log_file_button.clicked.connect(self.show_0R_Dialog)
        self.zero_log_file_button.setStyleSheet("background-color: None; color: None;")
        layout6.addWidget(self.zero_log_file_button)

        self.point_eight_log_file_button = QPushButton("Vali 0.8R logifail")
        self.point_eight_log_file_button.clicked.connect(self.show_8R_Dialog)
        self.point_eight_log_file_button.setStyleSheet("background-color: None; color: None;")
        layout6.addWidget(self.point_eight_log_file_button)
        
        self.label200 = QLabel("Aken sulgub automaatselt kui arvutus on edukalt tehtud", self)
        layout6.addWidget(self.label200)

        self.confirm_button = QPushButton("Arvuta", self)
        self.confirm_button.clicked.connect(lambda: self.process_logs(self.zero_log_file_name, self.point_eight_log_file_name))
        self.confirm_button.setStyleSheet("background-color: None; color: None;")
        self.confirm_button.setEnabled(False)  # Initially disabled
        layout6.addWidget(self.confirm_button)

    def show_0R_Dialog(self):
        home_dir = str(Path.home())
        self.zero_log_file_button.setStyleSheet("background-color: None; color: None;")
        self.zero_log_file_name, _ = QFileDialog.getOpenFileName(self, 'Otsi 0R logi fail', home_dir, "csv(*.csv)")
        if self.zero_log_file_name:
            self.zero_file_selected = True  # Set flag when the file is selected
            self.zero_log_file_button.setStyleSheet("background-color: green; color: white;")
            self.check_files_selected()  # Check if both files are selected

    def show_8R_Dialog(self):
        home_dir = str(Path.home())
        self.point_eight_log_file_button.setStyleSheet("background-color: None; color: None;")
        self.point_eight_log_file_name, _ = QFileDialog.getOpenFileName(self, 'Otsi 0.8R logi fail', home_dir, "csv(*.csv)")
        if self.point_eight_log_file_name:
            self.point_eight_file_selected = True  # Set flag when the file is selected
            self.point_eight_log_file_button.setStyleSheet("background-color: green; color: white;")
            self.check_files_selected()  # Check if both files are selected

    # Enable confirm button if both files are selected
    def check_files_selected(self):
        if self.zero_file_selected and self.point_eight_file_selected:
            self.confirm_button.setEnabled(True)
        else:
            self.confirm_button.setEnabled(False)
               
    def process_logs(self, zero_log_file, point_eight_log_file):
        va_zero_list = []
        x_pos_list = []
        thrust_list = []
        delta_va_list = []
        global rho
        
    #     # Check if rho has been defined; if not, find it in the log file
    #     if rho is None:
    #         found_rho = find_rho_in_log(zero_log_file)
    #         if not found_rho:
    #             return  # Stop execution if air density was not found
        
        with open(zero_log_file, 'r') as f:
            lines = f.readlines()
            for line in lines[::-1]:  # Reverse the file and look for air density
                if 'Air_density' in line:
                    # Extract the value (before 'kg/m3')
                    try:
                        rho = float(line.split()[1])  # Extract the second item as the air density value
                        print(f"Air density (rho) found: {rho} kg/m^3")
                    except ValueError:
                        print("Error extracting air density.")
                    break

        if rho is None:
            print("Error: Air density value not found in the log file.")
            return

        
        # Read the first log file (zero_log_file)
        with open(zero_log_file, newline='') as csv1file:
            row_read = csv.reader(csv1file, delimiter=' ')
            next(row_read)  # Skip header if necessary
            rows_zero_log = [row for row in row_read]  # Collect all rows from the first log
        
        # Read the second log file (point_eight_log_file)
        with open(point_eight_log_file, newline='') as csv2file:
            row_read = csv.reader(csv2file, delimiter=' ')
            next(row_read)  # Skip header if necessary
            rows_point_eight_log = [row for row in row_read]  # Collect all rows from the second log
        
        # Ensure both logs have the same length for comparison
        min_length = min(len(rows_zero_log), len(rows_point_eight_log))
        rows_zero_log = rows_zero_log[:min_length]
        rows_point_eight_log = rows_point_eight_log[:min_length]

        # Process each row and compare the v_axial values
        for i in range(min_length):
            row_zero = rows_zero_log[i]
            row_point_eight = rows_point_eight_log[i]

            try:
                # Assuming column 11 is v_axial in both log files
                v_axial_zero = float(row_zero[11])  # v_axial from zero_log_file
                v_axial_point_eight = float(row_point_eight[11])  # v_axial from point_eight_log_file
                x_pos = float(row_zero[2]) / 1000  # X position in meters
                radius = ((float(row_zero[1]) * 25.4) / 2) / 1000 #Get prop diameter and convert it to metric and radius
                thrust_zero = float(row_zero[5])

                # Calculate delta_va
                if v_axial_point_eight > v_axial_zero:
                    delta_va = round(v_axial_point_eight - v_axial_zero, 2)
                else:
                    delta_va = 0.00

                # Append delta_va to the row
                #row_zero.append(delta_va)

                # Calculate diff_mass_rate for each row
                diff_mass_rate = round(math.pi * rho * v_axial_zero * x_pos, 2)
                row_zero.append(diff_mass_rate)
                
                # Calculate diff_thrust for each row
                diff_thrust = round(math.pi * rho * v_axial_zero * (v_axial_zero + delta_va) * x_pos, 2)
                row_zero.append(diff_thrust)

                # Store diff_mass_rate and delta_va
                delta_va_list.append(delta_va)
                va_zero_list.append(v_axial_zero)
                x_pos_list.append(x_pos)
                thrust_list.append(thrust_zero)
                
                max_radius = radius
                
                def thrust_moment_integrand(x):
                    # Safeguard for out-of-bound x
                    if x < x_pos_list[0] or x > x_pos_list[-1]:
                        return 0
                    
                    # Find the closest index
                    index = np.searchsorted(x_pos_list, x) - 1
                    if index < 0 or index >= len(x_pos_list):
                        return 0
                    
                    # Linear interpolation between points to avoid discontinuity  o
                    va_zero = va_zero_list[index]
                    delta_va = delta_va_list[index]
                    # Calculate thrust contribution at radial position x
                    thrust_contribution = math.pi * rho * va_zero * (va_zero + delta_va) * x
                    
                    # Return thrust moment contribution = thrust_contribution * x
                    return thrust_contribution * x
                
                result, error = quad(thrust_moment_integrand, x_pos_list[0], max_radius, limit=1500)
                
                thrust = round(sum(thrust_list)/len(thrust_list),2)
                
                r_CT = 2*result/thrust
                
                center_of_thrust = (r_CT/max_radius)*100
                

            except (IndexError, ValueError):
                # Skip rows with invalid data
                row_zero.append(0.00)  # For delta_va
                row_zero.append(0.00)  # For diff_mass_rate

        # Write the modified rows back to a new CSV file or overwrite the original
        output_file = zero_log_file
        with open(output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=' ')
            # Write a header including the new columns
            writer.writerow(['#_of_samples', 'Prop_diam(inch)', 'X_position(mm)', 'Y_position(mm)', 'Torque(Nm)', 'Thrust(N)', 
                             'Airspeed(m/s)', 'AoA(deg)', 'AoSS(deg)', 'V_tan(m/s)', 'V_rad(m/s)', 'V_axial(m/s)', 
                             'Chord_angle(deg)', 'Chord_angle_eff(deg)', 'Chord_length(mm)', 'Chord_length_eff(mm)', 
                             'Helix_angle_eff(deg)', 'Alpha_angle(deg)', 'V_total(m/s)', 'V_lift(m/s)', 'V_drag(m/s)', 
                             'CL', 'CD', 'Reynolds_number', 'V_a+r(m/s)', 'D/R_ratio', 'Diff_mass_rate(kg/s)', 'Diff_thrust(N/m)'])  # Add delta_va and diff_mass_rate headers
            writer.writerows(rows_zero_log)
            writer.writerow(['Thrust_moment',round(result,2), 'Nm'])
            writer.writerow(['Center_of_thrust_radius',round(r_CT,2), 'm'])
            writer.writerow(['Center_of_thrust',round(center_of_thrust,2), '%'])
            
        print(f"Modified CSV written to '{output_file}'")
        self.confirm_button.setStyleSheet("background-color: green; color: white;")
        self.confirm_button.setText("Arvutatud")
        QTimer.singleShot(1000, self.close)