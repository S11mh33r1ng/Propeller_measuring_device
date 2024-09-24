import os
import csv
import math
import statistics
import argparse
from pathlib import Path

# Global lists for storing intermediate results
var_list = []
trq_list = []
thr_list = []

rho = 1.225 #kg/cm3 standard air density
kin_visc = 1.48
x_delta = 3

# Function to handle processing of the log and propeller files
def process_data(log_file, prop_file, output_dir, radius_mm, safety_over_prop, kin_visc, rho, dr_ratio_value):
    try:
        mean_csvfile = os.path.join(output_dir, os.path.basename(log_file).replace('.csv', '_mean.csv'))
        mean_header = ['#_of_samples', 'Prop_diam(inch)', 'X_position(mm)', 'Y_position(mm)', 'Torque(Nm)', 'Thrust(N)', 
                       'Airspeed(m/s)', 'AoA(deg)', 'AoSS(deg)', 'V_tan(m/s)', 'V_rad(m/s)', 'V_axial(m/s)', 
                       'Chord_angle(deg)', 'Chord_angle_eff(deg)', 'Chord_length(mm)', 'Chord_length_eff(mm)', 
                       'Helix_angle_eff(deg)', 'Alpha_angle(deg)', 'V_total(m/s)', 'V_lift(m/s)', 'V_drag(m/s)', 
                       'CL', 'CD', 'Reynolds_number', 'V_a+r(m/s)', 'D/R_ratio']
        
        # Read omega values from the log file
        omega_values = []
        with open(log_file, newline='') as csvfile:
            row_read = csv.reader(csvfile, delimiter=' ')
            for row in row_read:
                sample = row
                try:
                    omega_value = float(sample[5])
                    omega_values.append(omega_value)
                except (IndexError, ValueError):
                    continue

        # Calculate omega mode or mean if mode is not available
        if omega_values:
            try:
                omega_mode = statistics.mode(omega_values)
            except statistics.StatisticsError:
                omega_mode = statistics.mean(omega_values)
        else:
            print("No omega values found in the log file.")
            omega_mode = 0

        with open(mean_csvfile, 'a') as h:
            k = csv.writer(h)
            k.writerow(mean_header)
        
        x_max = 3 * math.floor((float(radius_mm) + (1 - (safety_over_prop/100)))/3)
        x_mp = 0
        while x_mp <= x_max:
            try:
                with open(log_file, newline='') as csvfile:
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
                    #omega_mean = format(statistics.mode(list(dict_omega.values())),'.2f')
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
                
            with open(prop_file, newline = '') as propfile:
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
            Re = (float(chord_length)/1000 * float(total_speed))/(float(kin_visc) * math.pow(10,-5))
            mean_data.append(str(format(Re,'.0f')))
            v_2d = math.sqrt(math.pow(float(v_axial_mean),2) + math.pow(float(v_rad_mean),2))
            mean_data.append(str(format(v_2d,'.2f')))
            mean_data.append(str(format(dr_ratio_value,'.1f')))

            with open(mean_csvfile, 'a') as f:
                w = csv.writer(f)
                w.writerow([' '.join(mean_data)])
            
            x_mp = x_mp + 3
            
        vi = (2*(x_delta/1000)*sum(var_list))/math.pow(float(radius_mm)/1000,2)
        T = statistics.mean(thr_list)
        Pi = float(vi)*float(T)
        try:
            vv = float(T)/(rho * math.pi * math.pow(float(radius_mm)/1000,2) * math.pow(float(vi),2))
        except:
            vv = 0
        M = statistics.mean(trq_list)
        P = float(M)*float(omega_mode)
        vm = rho * math.pi * math.pow(float(radius_mm)/1000,2) * float(vi)
        try:
            v_max_mean = float(T)/float(vm)
        except:
            v_max_mean = 0
        try:
            Ct = float(T)/(rho * math.pow(float(omega_mode),2) * math.pow(((float(radius_mm)*2)/1000),4))
        except:
            Ct = 0
        try:
            Cp = float(M)/(rho * math.pow(float(omega_mode),2) * math.pow(((float(radius_mm)*2)/1000),5))
        except:
            Cp = 0
        try:
            nu = (Pi/P)*100
        except:
            nu = -1
        
        with open(mean_csvfile, 'a') as f:
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
            w.writerow(['Air_density',rho,'kg/m3'])
            w.writerow(['Air_kinematic_viscosity',kin_visc,'x10-5 m2/s'])

        print(f"Mean data file created at: {mean_csvfile}")
        
    except FileNotFoundError as e:
        print(f"Error: {e}")

# Command-line argument handling
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process log and propeller configuration files.")
    parser.add_argument('log_file', type=str, help='Path to the raw log file')
    parser.add_argument('prop_file', type=str, help='Path to the propeller configuration file')
    parser.add_argument('--output_dir', type=str, default=str(Path.home()), help='Directory to save the output mean CSV file')
    parser.add_argument('--radius_mm', type=float, required=True, help='Propeller radius in mm')
    parser.add_argument('--safety_over_prop', type=float, required=True, help='Safety over prop percentage')
    #parser.add_argument('--kin_visc', type=float, required=True, help='Kinematic viscosity value')
    #parser.add_argument('--rho', type=float, required=True, help='Air density value (kg/m^3)')
    parser.add_argument('--dr_ratio', type=float, required=True, help='D/R ratio value')

    args = parser.parse_args()

    # Call the processing function with the provided arguments
    process_data(args.log_file, args.prop_file, args.output_dir, args.radius_mm, 
                 args.safety_over_prop, kin_visc, rho, args.dr_ratio)
