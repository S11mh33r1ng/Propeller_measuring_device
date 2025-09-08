from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QFileDialog
from PyQt5.QtCore import QTimer
import csv, math, numpy as np, os
from scipy.integrate import quad
from pathlib import Path

# ------------------------------------------------------------
# Local helpers (mirrors data_processing’s file reader style)
# ------------------------------------------------------------
def _read_rows_space_delimited(path):
    """
    Yield token lists from a file that may be comma- or space-delimited.
    Skips header or any row where the first token is non-numeric.
    """
    with open(path, newline='') as f:
        for raw in f:
            s = raw.strip()
            if not s:
                continue
            tokens = [t.strip() for t in s.split(',')] if ',' in s else s.split()
            if not tokens:
                continue
            try:
                float(tokens[0])
            except ValueError:
                # allow summary lines to be captured elsewhere; skip here
                continue
            yield tokens

def _try_read_rho_from_summary(path):
    """
    Try to find 'Air_density' in legacy/older-style summaries.
    Newer data_processing outputs may not include it; return None if absent.
    """
    try:
        with open(path, 'r') as f:
            for line in reversed(f.readlines()):
                if 'Air_density' in line:
                    parts = line.replace(',', ' ').split()
                    # e.g., ['Air_density', '1.225', 'kg/m3']
                    for i, p in enumerate(parts):
                        if p.lower().startswith('air_density'):
                            # the number should be the next token
                            pass
                    # fallback simple parse: first float on the line
                    for tok in parts:
                        try:
                            return float(tok)
                        except ValueError:
                            continue
                    break
    except Exception:
        pass
    return None

class Calculate_center_of_thrust(QWidget):
    def __init__(self):
        super().__init__()
        layout6 = QVBoxLayout()
        self.setLayout(layout6)
        self.setWindowTitle("Õhuvoo monitoorimise lisaarvutused")

        # UI state
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
        self.confirm_button.setEnabled(False)
        layout6.addWidget(self.confirm_button)

    def show_0R_Dialog(self):
        home_dir = str(Path.home())
        self.zero_log_file_button.setStyleSheet("background-color: None; color: None;")
        self.zero_log_file_name, _ = QFileDialog.getOpenFileName(self, 'Otsi 0R logi fail', home_dir, "csv(*.csv)")
        if self.zero_log_file_name:
            self.zero_file_selected = True
            self.zero_log_file_button.setStyleSheet("background-color: green; color: white;")
            self.check_files_selected()

    def show_8R_Dialog(self):
        home_dir = str(Path.home())
        self.point_eight_log_file_button.setStyleSheet("background-color: None; color: None;")
        self.point_eight_log_file_name, _ = QFileDialog.getOpenFileName(self, 'Otsi 0.8R logi fail', home_dir, "csv(*.csv)")
        if self.point_eight_log_file_name:
            self.point_eight_file_selected = True
            self.point_eight_log_file_button.setStyleSheet("background-color: green; color: white;")
            self.check_files_selected()

    def check_files_selected(self):
        self.confirm_button.setEnabled(self.zero_file_selected and self.point_eight_file_selected)

    # ------------------------------------------------------------
    # Core computation + CSV writing (aligned with data_processing)
    # ------------------------------------------------------------
    def process_logs(self, zero_log_file, point_eight_log_file):
        if not zero_log_file or not point_eight_log_file:
            return

        # 1) Air density rho
        rho = _try_read_rho_from_summary(zero_log_file)
        if rho is None:
            # Newer data_processing no longer writes Air_density; use a sensible default.
            rho = 1.225  # kg/m^3 @ sea level
            print("Air density not found in summary; using default rho = 1.225 kg/m^3.")

        # 2) Load numeric rows from both files (comma- or space-delimited)
        rows_zero_raw = list(_read_rows_space_delimited(zero_log_file))
        rows_point8_raw = list(_read_rows_space_delimited(point_eight_log_file))
        min_len = min(len(rows_zero_raw), len(rows_point8_raw))
        rows_zero_raw = rows_zero_raw[:min_len]
        rows_point8_raw = rows_point8_raw[:min_len]

        # 3) Compute per-row metrics and build output rows
        data_rows = []
        va_zero_list, delta_va_list, x_pos_list, thrust_list = [], [], [], []
        radius_m = None

        for rz, rp in zip(rows_zero_raw, rows_point8_raw):
            try:
                # Column map follows data_processing single-prop CSV (no omega column in output):contentReference[oaicite:4]{index=4}
                # 0=prop_inch, 1=x_mm, 2=y_mm, 3=torque, 4=thrust, 5=omega (raw logs), 6=airspeed, 7=aoa, 8=aoss,
                # 9=v_tan, 10=v_rad, 11=v_axial, then geometry/kinematics in mean CSV.
                v_axial_zero = float(rz[11])
                v_axial_p8   = float(rp[11])
                x_pos_m      = float(rz[2]) / 1000.0  # index 2 is y_mm in raw logs; BUT in mean CSV header it's x_mm at index 3.
                                                     # We rely on raw-log layout used by data_processing readers:contentReference[oaicite:5]{index=5}.
                # If the inputs are mean CSVs (comma variant), indices still align for the first 12 fields.
                prop_inch    = float(rz[0])
                thrust_zero  = float(rz[4])
                delta_va     = max(v_axial_p8 - v_axial_zero, 0.0)

                # radius from prop diameter in inches
                radius_m = (prop_inch * 25.4 / 1000.0) / 2.0

                diff_mass_rate = math.pi * rho * v_axial_zero * x_pos_m
                diff_thrust    = math.pi * rho * v_axial_zero * (v_axial_zero + delta_va) * x_pos_m

                # Track arrays (sorted later for integration)
                va_zero_list.append(v_axial_zero)
                delta_va_list.append(delta_va)
                x_pos_list.append(x_pos_m)
                thrust_list.append(thrust_zero)

                # Append two new columns to the original row
                data_rows.append(rz + [f"{diff_mass_rate:.2f}", f"{diff_thrust:.2f}"])
            except Exception:
                # Skip malformed rows silently (consistent with data_processing’s forgiving IO):contentReference[oaicite:6]{index=6}
                continue

        if not data_rows or radius_m is None:
            print("No valid rows to process; aborting.")
            return

        # 4) Sort by radius position for stable integration
        order = np.argsort(x_pos_list)
        x_sorted = [x_pos_list[i] for i in order]
        va_sorted = [va_zero_list[i] for i in order]
        dva_sorted = [delta_va_list[i] for i in order]

        def thrust_moment_integrand(x):
            # piecewise-constant (left) sampling with searchsorted
            idx = np.searchsorted(x_sorted, x) - 1
            if idx < 0 or idx >= len(x_sorted):
                return 0.0
            va0 = va_sorted[idx]
            dva = dva_sorted[idx]
            thrust_contrib = math.pi * rho * va0 * (va0 + dva) * x
            return thrust_contrib * x  # moment = force * radius

        lower = x_sorted[0]
        upper = radius_m
        if lower >= upper:
            lower = 0.0  # fallback

        thrust_moment, _ = quad(thrust_moment_integrand, lower, upper, limit=1500)
        thrust_mean = sum(thrust_list) / max(len(thrust_list), 1)
        r_CT = 2.0 * thrust_moment / thrust_mean if thrust_mean else 0.0
        center_of_thrust_pct = (r_CT / radius_m) * 100.0 if radius_m else 0.0

        # 5) Write structured CSV (comma-separated) with a summary block appended (like data_processing):contentReference[oaicite:7]{index=7}
        base, ext = os.path.splitext(zero_log_file)
        out_path = base + "_ct.csv"

#         header = [
#             "#_of_samples", "Prop_diam(inch)", "X_position(mm)", "Y_position(mm)",
#             "Torque(Nm)", "Thrust(N)", "Airspeed(m/s)", "AoA(deg)", "AoSS(deg)",
#             "V_tan(m/s)", "V_rad(m/s)", "V_axial(m/s)",
#             "Chord_angle(deg)", "Chord_angle_eff(deg)", "Chord_length(mm)", "Chord_length_eff(mm)",
#             "Helix_angle_eff(deg)", "Alpha_angle(deg)", "V_total(m/s)", "V_lift(m/s)", "V_drag(m/s)",
#             "CL", "CD", "Re", "V_a+r(m/s)", "D/R_ratio",
#             "Diff_mass_rate(kg/s)", "Diff_thrust(N/m)"
#         ]
        
        # Combined header (NO omega1_rad_s)
        header = [
            "samples", "prop_inch", "dr_ratio", "x_mm", "y_mm",
            "torque1_Nm", "thrust1_N",
            "airspeed_mps", "aoa_deg", "aoss_deg",
            "v_tan_mps", "v_rad_mps", "v_axial_mps",
            "chord_angle_deg", "chord_angle_eff_deg",
            "chord_length_mm", "chord_length_eff_mm",
            "helix_angle_eff_deg", "alpha_angle_deg",
            "v_total_mps", "v_lift_mps", "v_drag_mps",
            "CL", "CD", "Re", "v_a+r_mps", "d/r_ratio",
            "diff_mass_rate_kg/s", "diff_thrust_N/m"
        ]

        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)  # comma-separated to match data_processing outputs:contentReference[oaicite:8]{index=8}
            w.writerow(header)
            w.writerows(data_rows)
            w.writerow([])  # blank line before summary
            w.writerow(["Thrust_moment", f"{thrust_moment:.2f}", "Nm"])
            w.writerow(["Center_of_thrust_radius", f"{r_CT:.4f}", "m"])
            w.writerow(["Center_of_thrust", f"{center_of_thrust_pct:.2f}", "%"])
            w.writerow(["Air_density", f"{rho:.3f}", "kg/m3"])  # helpful for traceability even if newer logs omit it

        print(f"Modified CSV written to '{out_path}'")
        self.confirm_button.setStyleSheet("background-color: green; color: white;")
        self.confirm_button.setText("Arvutatud")
        QTimer.singleShot(1000, self.close)

# from data_processing import _read_rows_space_delimited  # reuse existing helper
# import csv, math, numpy as np
# from scipy.integrate import quad
# 
# def process_logs(self, zero_log_file, point_eight_log_file):
#     # --- Step 1: find rho (from summary rows at end) ---
#     rho = None
#     for tokens in _read_rows_space_delimited(zero_log_file):
#         if "Air_density" in tokens:
#             try:
#                 rho = float(tokens[1])
#             except Exception:
#                 pass
#             break
#     if rho is None:
#         print("Error: Air density not found in log file")
#         return
# 
#     # --- Step 2: load rows from both files ---
#     rows_zero = list(_read_rows_space_delimited(zero_log_file))
#     rows_point8 = list(_read_rows_space_delimited(point_eight_log_file))
#     min_len = min(len(rows_zero), len(rows_point8))
#     rows_zero, rows_point8 = rows_zero[:min_len], rows_point8[:min_len]
# 
#     # --- Step 3: compute per-row metrics ---
#     data_rows = []
#     va_zero_list, delta_va_list, x_pos_list, thrust_list = [], [], [], []
#     radius = None
# 
#     for rz, rp in zip(rows_zero, rows_point8):
#         try:
#             v_axial_zero = float(rz[11])
#             v_axial_p8   = float(rp[11])
#             x_pos        = float(rz[2]) / 1000.0
#             radius       = ((float(rz[1]) * 25.4) / 2) / 1000.0
#             thrust_zero  = float(rz[5])
# 
#             delta_va = max(v_axial_p8 - v_axial_zero, 0.0)
#             diff_mass_rate = math.pi * rho * v_axial_zero * x_pos
#             diff_thrust    = math.pi * rho * v_axial_zero * (v_axial_zero + delta_va) * x_pos
# 
#             va_zero_list.append(v_axial_zero)
#             delta_va_list.append(delta_va)
#             x_pos_list.append(x_pos)
#             thrust_list.append(thrust_zero)
# 
#             data_rows.append(rz + [f"{diff_mass_rate:.2f}", f"{diff_thrust:.2f}"])
#         except Exception:
#             continue
# 
#     # --- Step 4: integrate thrust moment ---
#     def thrust_moment_integrand(x):
#         idx = np.searchsorted(x_pos_list, x) - 1
#         if idx < 0 or idx >= len(x_pos_list): return 0
#         va0, dva = va_zero_list[idx], delta_va_list[idx]
#         thrust_contrib = math.pi * rho * va0 * (va0 + dva) * x
#         return thrust_contrib * x
# 
#     result, _ = quad(thrust_moment_integrand, x_pos_list[0], radius)
#     thrust_mean = sum(thrust_list) / len(thrust_list)
#     r_CT = 2 * result / thrust_mean
#     center_of_thrust = (r_CT / radius) * 100.0
# 
#     # --- Step 5: write structured CSV (like data_processing) ---
#     out_path = zero_log_file.replace(".csv", "_ct.csv")
#     header = ["#samples","Prop_diam(inch)","X_pos(mm)","Y_pos(mm)",
#               "Torque(Nm)","Thrust(N)","Airspeed(m/s)","AoA(deg)","AoSS(deg)",
#               "V_tan(m/s)","V_rad(m/s)","V_axial(m/s)",
#               "Chord_angle","Chord_angle_eff","Chord_len","Chord_len_eff",
#               "Helix_angle","Alpha_angle","V_total","V_lift","V_drag","CL","CD",
#               "Re","V_a+r","D/R","Diff_mass_rate(kg/s)","Diff_thrust(N/m)"]
# 
#     with open(out_path, "w", newline="") as f:
#         w = csv.writer(f)
#         w.writerow(header)
#         w.writerows(data_rows)
#         w.writerow([])
#         w.writerow(["Thrust_moment", f"{result:.2f}", "Nm"])
#         w.writerow(["Center_of_thrust_radius", f"{r_CT:.2f}", "m"])
#         w.writerow(["Center_of_thrust", f"{center_of_thrust:.2f}", "%"])
# 
#     return out_path

# from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QFileDialog
# from PyQt5.QtCore import QTimer
# import csv, math, numpy as np
# from scipy.integrate import quad
# from pathlib import Path
# 
# class Calculate_center_of_thrust(QWidget):
#     def __init__(self):
#         super().__init__()
#         
#         layout6 = QVBoxLayout()
#         self.setLayout(layout6)
#         
#         self.setWindowTitle("Õhuvoo monitoorimise lisaarvutused")
#         
#         # Variables to track file selection
#         self.zero_file_selected = False
#         self.point_eight_file_selected = False
#         self.zero_log_file_name = None
#         self.point_eight_log_file_name = None
# 
#         self.zero_log_file_button = QPushButton("Vali 0R logi fail")
#         self.zero_log_file_button.clicked.connect(self.show_0R_Dialog)
#         self.zero_log_file_button.setStyleSheet("background-color: None; color: None;")
#         layout6.addWidget(self.zero_log_file_button)
# 
#         self.point_eight_log_file_button = QPushButton("Vali 0.8R logifail")
#         self.point_eight_log_file_button.clicked.connect(self.show_8R_Dialog)
#         self.point_eight_log_file_button.setStyleSheet("background-color: None; color: None;")
#         layout6.addWidget(self.point_eight_log_file_button)
#         
#         self.label200 = QLabel("Aken sulgub automaatselt kui arvutus on edukalt tehtud", self)
#         layout6.addWidget(self.label200)
# 
#         self.confirm_button = QPushButton("Arvuta", self)
#         self.confirm_button.clicked.connect(lambda: self.process_logs(self.zero_log_file_name, self.point_eight_log_file_name))
#         self.confirm_button.setStyleSheet("background-color: None; color: None;")
#         self.confirm_button.setEnabled(False)  # Initially disabled
#         layout6.addWidget(self.confirm_button)
# 
#     def show_0R_Dialog(self):
#         home_dir = str(Path.home())
#         self.zero_log_file_button.setStyleSheet("background-color: None; color: None;")
#         self.zero_log_file_name, _ = QFileDialog.getOpenFileName(self, 'Otsi 0R logi fail', home_dir, "csv(*.csv)")
#         if self.zero_log_file_name:
#             self.zero_file_selected = True  # Set flag when the file is selected
#             self.zero_log_file_button.setStyleSheet("background-color: green; color: white;")
#             self.check_files_selected()  # Check if both files are selected
# 
#     def show_8R_Dialog(self):
#         home_dir = str(Path.home())
#         self.point_eight_log_file_button.setStyleSheet("background-color: None; color: None;")
#         self.point_eight_log_file_name, _ = QFileDialog.getOpenFileName(self, 'Otsi 0.8R logi fail', home_dir, "csv(*.csv)")
#         if self.point_eight_log_file_name:
#             self.point_eight_file_selected = True  # Set flag when the file is selected
#             self.point_eight_log_file_button.setStyleSheet("background-color: green; color: white;")
#             self.check_files_selected()  # Check if both files are selected
# 
#     # Enable confirm button if both files are selected
#     def check_files_selected(self):
#         if self.zero_file_selected and self.point_eight_file_selected:
#             self.confirm_button.setEnabled(True)
#         else:
#             self.confirm_button.setEnabled(False)
#                
#     def process_logs(self, zero_log_file, point_eight_log_file):
#         va_zero_list = []
#         x_pos_list = []
#         thrust_list = []
#         delta_va_list = []
#         global rho
#         
#     #     # Check if rho has been defined; if not, find it in the log file
#     #     if rho is None:
#     #         found_rho = find_rho_in_log(zero_log_file)
#     #         if not found_rho:
#     #             return  # Stop execution if air density was not found
#         
#         with open(zero_log_file, 'r') as f:
#             lines = f.readlines()
#             for line in lines[::-1]:  # Reverse the file and look for air density
#                 if 'Air_density' in line:
#                     # Extract the value (before 'kg/m3')
#                     try:
#                         rho = float(line.split()[1])  # Extract the second item as the air density value
#                         print(f"Air density (rho) found: {rho} kg/m^3")
#                     except ValueError:
#                         print("Error extracting air density.")
#                     break
# 
#         if rho is None:
#             print("Error: Air density value not found in the log file.")
#             return
# 
#         
#         # Read the first log file (zero_log_file)
#         with open(zero_log_file, newline='') as csv1file:
#             row_read = csv.reader(csv1file, delimiter=' ')
#             next(row_read)  # Skip header if necessary
#             rows_zero_log = [row for row in row_read]  # Collect all rows from the first log
#         
#         # Read the second log file (point_eight_log_file)
#         with open(point_eight_log_file, newline='') as csv2file:
#             row_read = csv.reader(csv2file, delimiter=' ')
#             next(row_read)  # Skip header if necessary
#             rows_point_eight_log = [row for row in row_read]  # Collect all rows from the second log
#         
#         # Ensure both logs have the same length for comparison
#         min_length = min(len(rows_zero_log), len(rows_point_eight_log))
#         rows_zero_log = rows_zero_log[:min_length]
#         rows_point_eight_log = rows_point_eight_log[:min_length]
# 
#         # Process each row and compare the v_axial values
#         for i in range(min_length):
#             row_zero = rows_zero_log[i]
#             row_point_eight = rows_point_eight_log[i]
# 
#             try:
#                 # Assuming column 11 is v_axial in both log files
#                 v_axial_zero = float(row_zero[11])  # v_axial from zero_log_file
#                 v_axial_point_eight = float(row_point_eight[11])  # v_axial from point_eight_log_file
#                 x_pos = float(row_zero[2]) / 1000  # X position in meters
#                 radius = ((float(row_zero[1]) * 25.4) / 2) / 1000 #Get prop diameter and convert it to metric and radius
#                 thrust_zero = float(row_zero[5])
# 
#                 # Calculate delta_va
#                 if v_axial_point_eight > v_axial_zero:
#                     delta_va = round(v_axial_point_eight - v_axial_zero, 2)
#                 else:
#                     delta_va = 0.00
# 
#                 # Append delta_va to the row
#                 #row_zero.append(delta_va)
# 
#                 # Calculate diff_mass_rate for each row
#                 diff_mass_rate = round(math.pi * rho * v_axial_zero * x_pos, 2)
#                 row_zero.append(diff_mass_rate)
#                 
#                 # Calculate diff_thrust for each row
#                 diff_thrust = round(math.pi * rho * v_axial_zero * (v_axial_zero + delta_va) * x_pos, 2)
#                 row_zero.append(diff_thrust)
# 
#                 # Store diff_mass_rate and delta_va
#                 delta_va_list.append(delta_va)
#                 va_zero_list.append(v_axial_zero)
#                 x_pos_list.append(x_pos)
#                 thrust_list.append(thrust_zero)
#                 
#                 max_radius = radius
#                 
#                 def thrust_moment_integrand(x):
#                     # Safeguard for out-of-bound x
#                     if x < x_pos_list[0] or x > x_pos_list[-1]:
#                         return 0
#                     
#                     # Find the closest index
#                     index = np.searchsorted(x_pos_list, x) - 1
#                     if index < 0 or index >= len(x_pos_list):
#                         return 0
#                     
#                     # Linear interpolation between points to avoid discontinuity  o
#                     va_zero = va_zero_list[index]
#                     delta_va = delta_va_list[index]
#                     # Calculate thrust contribution at radial position x
#                     thrust_contribution = math.pi * rho * va_zero * (va_zero + delta_va) * x
#                     
#                     # Return thrust moment contribution = thrust_contribution * x
#                     return thrust_contribution * x
#                 
#                 result, error = quad(thrust_moment_integrand, x_pos_list[0], max_radius, limit=1500)
#                 
#                 thrust = round(sum(thrust_list)/len(thrust_list),2)
#                 
#                 r_CT = 2*result/thrust
#                 
#                 center_of_thrust = (r_CT/max_radius)*100
#                 
# 
#             except (IndexError, ValueError):
#                 # Skip rows with invalid data
#                 row_zero.append(0.00)  # For delta_va
#                 row_zero.append(0.00)  # For diff_mass_rate
# 
#         # Write the modified rows back to a new CSV file or overwrite the original
#         output_file = zero_log_file
#         with open(output_file, 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile, delimiter=' ')
#             # Write a header including the new columns
#             writer.writerow(['#_of_samples', 'Prop_diam(inch)', 'X_position(mm)', 'Y_position(mm)', 'Torque(Nm)', 'Thrust(N)', 
#                              'Airspeed(m/s)', 'AoA(deg)', 'AoSS(deg)', 'V_tan(m/s)', 'V_rad(m/s)', 'V_axial(m/s)', 
#                              'Chord_angle(deg)', 'Chord_angle_eff(deg)', 'Chord_length(mm)', 'Chord_length_eff(mm)', 
#                              'Helix_angle_eff(deg)', 'Alpha_angle(deg)', 'V_total(m/s)', 'V_lift(m/s)', 'V_drag(m/s)', 
#                              'CL', 'CD', 'Reynolds_number', 'V_a+r(m/s)', 'D/R_ratio', 'Diff_mass_rate(kg/s)', 'Diff_thrust(N/m)'])  # Add delta_va and diff_mass_rate headers
#             writer.writerows(rows_zero_log)
#             writer.writerow(['Thrust_moment',round(result,2), 'Nm'])
#             writer.writerow(['Center_of_thrust_radius',round(r_CT,2), 'm'])
#             writer.writerow(['Center_of_thrust',round(center_of_thrust,2), '%'])
#             
#         print(f"Modified CSV written to '{output_file}'")
#         self.confirm_button.setStyleSheet("background-color: green; color: white;")
#         self.confirm_button.setText("Arvutatud")
#         QTimer.singleShot(1000, self.close)