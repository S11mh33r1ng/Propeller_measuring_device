import os
import csv
import math
import statistics


# ============================================================
# Low-level file helpers
# ============================================================

def _read_rows_space_delimited(path):
    """Yield tokens from a file that may be comma- or space-delimited. Skips header rows."""
    with open(path, newline='') as f:
        for raw in f:
            s = raw.strip()
            if not s:
                continue
            tokens = [t.strip() for t in s.split(',')] if ',' in s else s.split()
            if not tokens:
                continue
            # Skip header or any non-numeric first token
            try:
                float(tokens[0])
            except ValueError:
                continue
            yield tokens


def _write_one_cell_row(path, row_str, mode='a', delimiter=','):
    """
    Write a single CSV row consisting of one cell that contains a space-joined string.
    This mirrors the original behavior where lines were written as [' '.join(mean_data)].
    """
    with open(path, mode, newline='') as f:
        w = csv.writer(f, delimiter=delimiter)
        w.writerow([row_str])


# ============================================================
# Tandem path (2 propellers): averaging by coordinates only
# ============================================================

def _read_omega_values_tandem(log_path):
    """Collect omega values from both props (if present) and return a combined list."""
    vals = []
    try:
        for tokens in _read_rows_space_delimited(log_path):
            if len(tokens) > 5:
                try: vals.append(float(tokens[5]))   # omega1
                except ValueError: pass
            if len(tokens) > 14:
                try: vals.append(float(tokens[14]))  # omega2
                except ValueError: pass
    except FileNotFoundError as e:
        print(f"Error reading log file: {e}")
    return vals

def _read_omegas_tandem_separate(log_path):
    """Return (omega1_list, omega2_list) from raw log."""
    o1, o2 = [], []
    try:
        for tokens in _read_rows_space_delimited(log_path):
            if len(tokens) > 5:
                try: o1.append(float(tokens[5]))
                except ValueError: pass
            if len(tokens) > 14:
                try: o2.append(float(tokens[14]))
                except ValueError: pass
    except FileNotFoundError as e:
        print(f"Error reading log file: {e}")
    return o1, o2

def _tandem_average_file(window):
    """
    Tandem (2 props):
      - Average by (x,y).
      - Write a proper comma CSV without omega columns.
      - Append a single Omega metric (mode across omega1/omega2) at the end.
      - No plotting or extra calculations.
    """
    plot_filename = f"log{window.today_dt}.png"
    log_path = os.path.join(window.path, window.csvfile)
    out_name = f"log{window.today_dt}_mean.csv"
    out_path = os.path.join(window.path, out_name)

    header = [
        "samples", "prop_inch", "dr_ratio", "x_mm", "y_mm",
        "torque1_Nm", "thrust1_N",
        "airspeed_mps", "aoa_deg", "aoss_deg",
        "v_tan_mps", "v_rad_mps", "v_axial_mps",
        "torque2_Nm", "thrust2_N",
    ]

    # Group rows by (X, Y)
    groups = {}
    for tokens in _read_rows_space_delimited(log_path):
        if len(tokens) < 12:
            continue
        try:
            x = int(float(tokens[1])); y = int(float(tokens[2]))
        except ValueError:
            continue
        groups.setdefault((x, y), []).append(tokens)

    # D/R from UI
    try:
        dr_ratio_val = float(window.dr_ratio.value())
    except Exception:
        dr_ratio_val = 0.0

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)

        for (x, y) in sorted(groups.keys()):
            rows = groups[(x, y)]
            n = len(rows)

            def col_mean(idx):
                vals = []
                for r in rows:
                    if len(r) <= idx:
                        continue
                    try:
                        vals.append(float(r[idx]))
                    except ValueError:
                        pass
                return statistics.mean(vals) if vals else 0.0

            # Column map (same as before, but we won't write omegas):
            # 0=prop_inch, 1=X, 2=Y, 3=Torque1, 4=Thrust1, 5=Omega1,
            # 6=Airspeed, 7=AoA, 8=AoSS, 9=V_tan, 10=V_rad, 11=V_axial,
            # 12=Torque2, 13=Thrust2, 14=Omega2
            prop_avg  = col_mean(0)
            trq1      = col_mean(3)
            thr1      = col_mean(4)
            air       = col_mean(6)
            aoa       = col_mean(7)
            aoss      = col_mean(8)
            vtan      = col_mean(9)
            vrad      = col_mean(10)
            vax       = col_mean(11)
            trq2      = col_mean(12)
            thr2      = col_mean(13)

            w.writerow([
                n, round(prop_avg, 3), round(dr_ratio_val, 1), x, y,
                round(trq1, 2), round(thr1, 2),
                round(air, 2), round(aoa, 2), round(aoss, 2),
                round(vtan, 2), round(vrad, 2), round(vax, 2),
                round(trq2, 2), round(thr2, 2),
            ])
            try:
                window.update_plot_ax2(int(x), float(vtan), float(vrad), float(vax))
            except Exception:
                pass

    omega1_list, omega2_list = _read_omegas_tandem_separate(log_path)
    omega1_m = _omega_mode(omega1_list)
    omega2_m = _omega_mode(omega2_list)

    with open(out_path, "a", newline="") as f:
        w = csv.writer(f)
        w.writerow([])
        #w.writerow(["metric", "value", "unit"])
        w.writerow(["Omega1", f"{float(omega1_m):.2f}", "rad/s"])
        w.writerow(["Omega2", f"{float(omega2_m):.2f}", "rad/s"])
        
    try:
        window.counter = 0
        window.cnv.draw_ax2()
        window.cnv.save_only_second_plot(os.path.join(window.path, plot_filename))
    except Exception as e:
        print(f"Plot 2 export failed in tandem: {e}")

    return out_path



# ============================================================
# Single-prop path (legacy behavior): helpers
# (same functionality as your current process_data, but modular)
# ============================================================

def _read_omega_values(log_path):
    vals = []
    try:
        for tokens in _read_rows_space_delimited(log_path):
            if len(tokens) > 5:
                try:
                    vals.append(float(tokens[5]))
                except ValueError:
                    pass
    except FileNotFoundError as e:
        print(f"Error reading log file: {e}")
    return vals


def _omega_mode(values):
    if not values:
        print("No omega values found in the log file.")
        return 0.0
    try:
        return statistics.mode(values)
    except statistics.StatisticsError as e:
        print(f"Could not compute mode: {e}. Using mean instead.")
        return statistics.mean(values)


def _aggregate_at_x(log_path, x_mp):
    """
    Collect rows with X == x_mp and compute means
    (matches your original per-x loop behavior).
    Returns dict or None if no matches.
    """
    mean_counter = 0
    d = {
        'prop': [], 'x': [], 'y': [],
        'trq': [], 'thr': [], 'omega': [],
        'arspd': [], 'aoa': [], 'aoss': [],
        'vtan': [], 'vrad': [], 'vax': []
    }

    for tokens in _read_rows_space_delimited(log_path):
        if len(tokens) < 12:
            continue
        try:
            x_val = int(float(tokens[1]))
        except ValueError:
            continue
        if x_val != x_mp:
            continue
        try:
            d['prop'].append(float(tokens[0]))
            d['x'].append(int(float(tokens[1])))
            d['y'].append(int(float(tokens[2])))
            d['trq'].append(float(tokens[3]))
            d['thr'].append(float(tokens[4]))
            d['omega'].append(float(tokens[5]))
            d['arspd'].append(float(tokens[6]))
            d['aoa'].append(float(tokens[7]))
            d['aoss'].append(float(tokens[8]))
            d['vtan'].append(float(tokens[9]))
            d['vrad'].append(float(tokens[10]))
            d['vax'].append(float(tokens[11]))
            mean_counter += 1
        except ValueError:
            continue

    if mean_counter == 0:
        return None

    m = statistics.mean
    return {
        'testnumber': mean_counter,
        'prop': m(d['prop']),
        'x_pos': m(d['x']),
        'y_pos': m(d['y']),
        'trq_mean': m(d['trq']),
        'thr_mean': m(d['thr']),
        'arspd_mean': m(d['arspd']),
        'aoa_mean': m(d['aoa']),
        'aoss_mean': m(d['aoss']),
        'v_tan_mean': m(d['vtan']),
        'v_rad_mean': m(d['vrad']),
        'v_axial_mean': m(d['vax']),
    }


def _read_blade_geometry(prop_cfg_path, x_mp):
    """Return (chord_angle_raw, chord_length_raw) as strings (or '0.0','0.0')."""
    try:
        with open(prop_cfg_path, newline='') as propfile:
            r = csv.reader(propfile, delimiter=' ')
            for row in r:
                if len(row) >= 3 and row[0] == str(x_mp):
                    return row[1], row[2]
    except Exception as e:
        print(f"_read_blade_geometry error: {e}")
    return "0.0", "0.0"


def _compute_chord_effective(omega_mode, x_pos, v_tan_mean, v_rad_mean, chord_angle_raw, chord_length_raw):
    try:
        denom1 = float(omega_mode) * (float(x_pos) / 1000.0) - float(v_tan_mean)
        if denom1 == 0:
            return 0.0, 0.0
        denom2 = math.cos(math.atan(float(v_rad_mean) / denom1))
        vs1 = math.tan(math.radians(float(chord_angle_raw)))
        chord_angle_eff = math.degrees(math.atan(vs1 * denom2))
        chord_length_eff = float(chord_length_raw) / math.cos(math.atan(float(v_rad_mean) / denom1))
        return chord_angle_eff, chord_length_eff
    except Exception:
        return 0.0, 0.0


def _compute_section_kinematics(omega_mode, x_pos, v_tan_mean, v_rad_mean, v_axial_mean, chord_angle_eff):
    total_speed = math.sqrt(
        (float(omega_mode) * (float(x_pos) / 1000.0) - float(v_tan_mean)) ** 2
        + float(v_axial_mean) ** 2 + float(v_rad_mean) ** 2
    )
    try:
        helix_angle = math.degrees(math.asin(float(v_axial_mean) / total_speed))
    except Exception:
        helix_angle = 0.0

    alpha_angle = float(chord_angle_eff) - helix_angle
    v_lift = float(v_axial_mean) * math.cos(math.radians(helix_angle)) + float(v_tan_mean) * math.sin(math.radians(helix_angle))
    v_drag = float(v_tan_mean) * math.cos(math.radians(helix_angle)) - float(v_axial_mean) * math.sin(math.radians(helix_angle))

    try:
        cl = (2.0 * v_lift) / total_speed
    except Exception:
        cl = 0.0
    try:
        cd = (2.0 * v_drag) / total_speed
    except Exception:
        cd = 0.0

    return {
        'total_speed': f"{total_speed:.2f}",
        'helix_angle': f"{helix_angle:.2f}",
        'alpha_angle': f"{alpha_angle:.2f}",
        'v_lift': f"{v_lift:.2f}",
        'v_drag': f"{v_drag:.2f}",
        'cl': f"{cl:.3f}",
        'cd': f"{cd:.3f}",
    }


def _reynolds(chord_length_eff, total_speed, kin_visc_times1e5):
    try:
        Re = (float(chord_length_eff) / 1000.0 * float(total_speed)) / (float(kin_visc_times1e5) * 1e-5)
    except Exception:
        Re = 0.0
    return f"{Re:.0f}"


def _finalize_metrics(window, omega_mode, var_list, trq_list, thr_list):
    radius_m = float(window.radius_mm) / 1000.0
    vi = (2 * (window.shared_data.x_delta / 1000.0) * sum(var_list)) / (radius_m ** 2)
    window.label13.setText(f"{vi:.2f}")

    T = statistics.mean(thr_list)
    Pi = vi * T
    window.label15.setText(f"{Pi:.2f}")

    try:
        vv = T / (window.shared_data.rho * math.pi * (radius_m ** 2) * (vi ** 2))
    except Exception:
        vv = 0.0
    window.label65.setText(f"{vv:.2f}")

    M = statistics.mean(trq_list)
    P = M * float(omega_mode)
    window.label17.setText(f"{P:.2f}")

    vm = window.shared_data.rho * math.pi * (radius_m ** 2) * vi
    window.label67.setText(f"{vm:.2f}")

    try:
        v_max_mean = T / vm
    except Exception:
        v_max_mean = 0.0
    window.label69.setText(f"{v_max_mean:.2f}")

    try:
        Ct = T / (window.shared_data.rho * (float(omega_mode) ** 2) * ((2 * radius_m) ** 4))
    except Exception:
        Ct = 0.0
    try:
        Cp = M / (window.shared_data.rho * (float(omega_mode) ** 2) * ((2 * radius_m) ** 5))
    except Exception:
        Cp = 0.0
    try:
        nu = (Pi / P) * 100.0
        window.label19.setText(f"{nu:.2f}")
    except Exception:
        nu = -1.0
        window.label19.setText(str(nu))

    return {'vi': vi, 'Pi': Pi, 'P': P, 'nu': nu, 'vv': vv, 'vm': vm, 'v_max_mean': v_max_mean, 'Ct': Ct, 'Cp': Cp}


def _single_prop_full(window):
    """
    Single propeller:
      - Write ONE comma-separated CSV: log{timestamp}_mean.csv
      - Columns = tandem-style basic + extended (NO per-row omega column)
      - Keep all computations, labels, plots.
      - Append summary rows at the end (Omega, Pi, P, etc.).
    """
    plot_filename = f"log{window.today_dt}.png"
    log_path = os.path.join(window.path, window.csvfile)
    out_path = os.path.join(window.path, f"log{window.today_dt}_mean.csv")

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
        "CL", "CD", "Re", "v_a+r_mps",
    ]

    # Omega (mode) from raw log (column 5) — used in summary only
    omega_values = _read_omega_values(log_path)    # existing helper reads col 5 :contentReference[oaicite:1]{index=1}
    omega_m = _omega_mode(omega_values)

    # D/R from UI
    try:
        dr_ratio_val = float(window.dr_ratio.value())
    except Exception:
        dr_ratio_val = 0.0

    # Sweep x in 3 mm steps (unchanged)
    x_max = 3 * math.floor(
        (float(window.radius_mm) + (1 - (window.shared_data.safety_over_prop / 100))) / 3
    )

    var_list, trq_list, thr_list = [], [], []

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)

        x_mp = 0
        window.cnv.clear_plots()
        while x_mp <= x_max:
            stats = _aggregate_at_x(log_path, x_mp)
            if stats:
                tn    = int(stats['testnumber'])
                prop  = float(stats['prop'])
                x_pos = float(stats['x_pos'])
                y_pos = float(stats['y_pos'])
                trq   = float(stats['trq_mean'])
                thr   = float(stats['thr_mean'])
                air   = float(stats['arspd_mean'])
                aoa   = float(stats['aoa_mean'])
                aoss  = float(stats['aoss_mean'])
                vtan  = float(stats['v_tan_mean'])
                vrad  = float(stats['v_rad_mean'])
                vax   = float(stats['v_axial_mean'])

                # accumulate for final metrics
                var_list.append((x_pos / 1000.0) * vax)
                trq_list.append(trq)
                thr_list.append(thr)

                # geometry & kinematics (unchanged)
                chord_raw, chord_len_raw = _read_blade_geometry(window.fname[0], x_mp)
                chord_eff, chord_len_eff = _compute_chord_effective(
                    omega_m, x_pos, vtan, vrad, chord_raw, chord_len_raw
                )
                kin = _compute_section_kinematics(
                    omega_m, x_pos, vtan, vrad, vax, chord_eff
                )
                Re_str = _reynolds(chord_len_eff, float(kin['total_speed']), window.shared_data.kin_visc)
                v_2d = math.sqrt(vax**2 + vrad**2)

                # ONE CSV row (NO omega column)
                w.writerow([
                    tn, round(prop, 3), round(dr_ratio_val, 1), int(x_pos), int(y_pos),
                    round(trq, 2), round(thr, 2),
                    round(air, 2), round(aoa, 2), round(aoss, 2),
                    round(vtan, 2), round(vrad, 2), round(vax, 2),
                    str(chord_raw), f"{chord_eff:.2f}",
                    str(chord_len_raw), f"{chord_len_eff:.2f}",
                    kin['helix_angle'], kin['alpha_angle'],
                    kin['total_speed'], kin['v_lift'], kin['v_drag'],
                    kin['cl'], kin['cd'], Re_str, f"{v_2d:.2f}",
                ])

                # original plot update
                window.update_plot_ax2(int(x_pos), f"{vtan:.2f}", f"{vrad:.2f}", f"{vax:.2f}")

            x_mp += 3

    # Summary metrics (Omega + others) appended at the end
    res = _finalize_metrics(window, omega_m, var_list, trq_list, thr_list)
    with open(out_path, "a", newline="") as f:
        w = csv.writer(f)
        w.writerow([])
        #w.writerow(["metric", "value", "unit"])
        w.writerow(["Omega",                 f"{float(omega_m):.2f}",     "rad/s"])
        w.writerow(["Induced_power",         f"{res['Pi']:.2f}",          "W"])
        w.writerow(["Power",                 f"{res['P']:.2f}",           "W"])
        w.writerow(["Efficiency",            f"{res['nu']:.2f}",          "%"])
        w.writerow(["Average_induced_speed", f"{res['vi']:.2f}",          "m/s"])
        w.writerow(["Airspeed_ratio",        f"{res['vv']:.2f}",          ""])
        w.writerow(["V_mass",                f"{res['vm']:.2f}",          "kg/s"])
        w.writerow(["V_max_mean",            f"{res['v_max_mean']:.2f}",  "m/s"])
        w.writerow(["Ct",                    f"{res['Ct']:.7f}",          ""])
        w.writerow(["Cp",                    f"{res['Cp']:.7f}",          ""])

    window.counter = 0
    window.cnv.draw_ax2()
    window.cnv.save_only_second_plot(os.path.join(window.path, plot_filename))

    return out_path




# def _single_prop_full(window):
#     """Legacy behavior (1 prop): same functionality as before."""
#     plot_filename = f"log{window.today_dt}.png"
#     mean_csvfile = f"log{window.today_dt}_mean.csv"
#     mean_header = (
#         "#_of_samples Prop_diam(inch) X_position(mm) Y_position(mm) Torque(Nm) Thrust(N) "
#         "Airspeed(m/s) AoA(deg) AoSS(deg) V_tan(m/s) V_rad(m/s) V_axial(m/s) "
#         "Chord_angle(deg) Chord_angle_eff(deg) Chord_length(mm) Chord_length_eff(mm) "
#         "Helix_angle_eff(deg) Alpha_angle(deg) V_total(m/s) V_lift(m/s) V_drag(m/s) "
#         "CL CD Reynolds_number V_a+r(m/s) D/R_ratio"
#     )
# 
#     log_path = os.path.join(window.path, window.csvfile)
#     omega_values = _read_omega_values(log_path)
#     omega_m = _omega_mode(omega_values)
# 
#     # write header (as one cell, mirroring original)
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile), mean_header, mode='w')
# 
#     x_max = 3 * math.floor((float(window.radius_mm) + (1 - (window.shared_data.safety_over_prop / 100))) / 3)
#     var_list, trq_list, thr_list = [], [], []
# 
#     x_mp = 0
#     while x_mp <= x_max:
#         stats = _aggregate_at_x(log_path, x_mp)
#         if stats:
#             tn   = int(stats['testnumber'])
#             prop = stats['prop']; x_pos = stats['x_pos']; y_pos = stats['y_pos']
#             trq_mean = float(stats['trq_mean']); thr_mean = float(stats['thr_mean'])
#             arspd = float(stats['arspd_mean']); aoa = float(stats['aoa_mean']); aoss = float(stats['aoss_mean'])
#             vtan = float(stats['v_tan_mean']); vrad = float(stats['v_rad_mean']); vax = float(stats['v_axial_mean'])
# 
#             var_list.append((float(x_pos) / 1000.0) * vax)
#             trq_list.append(trq_mean)
#             thr_list.append(thr_mean)
# 
#             # geometry
#             chord_raw, chord_len_raw = _read_blade_geometry(window.fname[0], x_mp)
#             chord_eff, chord_len_eff = _compute_chord_effective(omega_m, x_pos, vtan, vrad, chord_raw, chord_len_raw)
#             kin = _compute_section_kinematics(omega_m, x_pos, vtan, vrad, vax, chord_eff)
#             Re_str = _reynolds(chord_len_eff, float(kin['total_speed']), window.shared_data.kin_visc)
#             v_2d = math.sqrt(vax ** 2 + vrad ** 2)
# 
#             row = (
#                 f"{tn} {prop} {x_pos} {y_pos} "
#                 f"{trq_mean:.2f} {thr_mean:.2f} {arspd:.2f} {aoa:.2f} {aoss:.2f} "
#                 f"{vtan:.2f} {vrad:.2f} {vax:.2f} "
#                 f"{chord_raw} {chord_eff:.2f} {chord_len_raw} {chord_len_eff:.2f} "
#                 f"{kin['helix_angle']} {kin['alpha_angle']} {kin['total_speed']} "
#                 f"{kin['v_lift']} {kin['v_drag']} {kin['cl']} {kin['cd']} "
#                 f"{Re_str} {v_2d:.2f} {window.dr_ratio.value():.1f}"
#             )
#             _write_one_cell_row(os.path.join(window.path, mean_csvfile), row, mode='a')
# 
#             # original plotting call
#             window.update_plot_ax2(x_pos, f"{vtan:.2f}", f"{vrad:.2f}", f"{vax:.2f}")
# 
#         x_mp += 3
# 
#     res = _finalize_metrics(window, omega_m, var_list, trq_list, thr_list)
# 
#     # summary block (space-delimited)
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"Omega {float(omega_m):.2f} rad/s", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"Induced_power {res['Pi']:.2f} W", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"Power {res['P']:.2f} W", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"Efficiency {res['nu']:.2f} %", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"Average_induced_speed {res['vi']:.2f} m/s", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"Airspeed_ratio {res['vv']:.2f}", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"V_mass {res['vm']:.2f} kg/s", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"V_max_mean {res['v_max_mean']:.2f} m/s", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"Ct {res['Ct']:.7f}", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"Cp {res['Cp']:.7f}", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"Air_density {window.shared_data.rho} kg/m3", mode='a', delimiter=' ')
#     _write_one_cell_row(os.path.join(window.path, mean_csvfile),
#                         f"Air_kinematic_viscosity {window.shared_data.kin_visc} x10-5 m2/s", mode='a', delimiter=' ')
# 
#     # finalize plot like original
#     window.counter = 0
#     window.cnv.draw_ax2()
#     window.cnv.save_only_second_plot(os.path.join(window.path, plot_filename))
# 
#     return os.path.join(window.path, mean_csvfile)


# ============================================================
# Public API
# ============================================================

def process_data(window):
    """
    Switch behavior based on window.tandem_setup:
      - False (1 prop): run legacy full pipeline (plots + labels + summaries).
      - True  (2 props): output averaged-by-(X,Y) mean file only.
    Returns path to the produced mean file.
    """
    if getattr(window, 'tandem_setup', False):
        return _tandem_average_file(window)
    else:
        return _single_prop_full(window)









# import os
# import csv
# import math
# import statistics
# 
# 
# def process_data(window):
#     plot_filename = f"log{window.today_dt}.png"
#     mean_csvfile = "log" + window.today_dt + "_mean.csv"
#     mean_header = [
#         '#_of_samples Prop_diam(inch) X_position(mm) Y_position(mm) '
#         'Torque(Nm) Thrust(N) Airspeed(m/s) AoA(deg) AoSS(deg) '
#         'V_tan(m/s) V_rad(m/s) V_axial(m/s) Chord_angle(deg) '
#         'Chord_angle_eff(deg) Chord_length(mm) Chord_length_eff(mm) '
#         'Helix_angle_eff(deg) Alpha_angle(deg) V_total(m/s) '
#         'V_lift(m/s) V_drag(m/s) CL CD Reynolds_number V_a+r(m/s) '
#         'D/R_ratio'
#     ]
# 
#     omega_values = []
#     try:
#         with open(os.path.join(window.path, window.csvfile), newline='') as csvfile:
#             row_read = csv.reader(csvfile, delimiter=' ')
#             for row in row_read:
#                 line = ' '.join(row)
#                 sample = list(line.split(" "))
#                 try:
#                     omega_value = float(sample[5])
#                     omega_values.append(omega_value)
#                 except (IndexError, ValueError):
#                     continue
#     except FileNotFoundError as e:
#         print(f"Error reading log file: {e}")
#         return
# 
#     if omega_values:
#         try:
#             omega_mode = statistics.mode(omega_values)
#         except statistics.StatisticsError as e:
#             print(f"Could not compute mode: {e}. Using mean instead.")
#             omega_mode = statistics.mean(omega_values)
#     else:
#         print("No omega values found in the log file.")
#         omega_mode = 0
# 
#     with open(os.path.join(window.path, mean_csvfile), 'a') as h:
#         k = csv.writer(h)
#         k.writerow(mean_header)
# 
#     x_max = 3 * math.floor(
#         (float(window.radius_mm) + (1 - (window.shared_data.safety_over_prop / 100))) / 3
#     )
#     x_mp = 0
#     var_list, trq_list, thr_list = [], [], []
# 
#     while x_mp <= x_max:
#         try:
#             with open(os.path.join(window.path, window.csvfile), newline='') as csvfile:
#                 row_read = csv.reader(csvfile, delimiter=' ')
#                 j = 0
#                 mean_counter = 0
#                 testnumber = 0
#                 dict_prop, dict_x, dict_y = {}, {}, {}
#                 dict_trq, dict_thr, dict_omega = {}, {}, {}
#                 dict_arspd, dict_aoa, dict_aoss = {}, {}, {}
#                 dict_v_tan, dict_v_rad, dict_v_axial = {}, {}, {}
# 
#                 for row in row_read:
#                     line = ' '.join(row)
#                     j = j + 1
#                     sample = list(line.split(" "))
#                     if sample[1] == str(x_mp):
#                         dict_prop[mean_counter] = float(sample[0])
#                         dict_x[mean_counter] = int(sample[1])
#                         dict_y[mean_counter] = int(sample[2])
#                         dict_trq[mean_counter] = float(sample[3])
#                         dict_thr[mean_counter] = float(sample[4])
#                         dict_omega[mean_counter] = float(sample[5])
#                         dict_arspd[mean_counter] = float(sample[6])
#                         dict_aoa[mean_counter] = float(sample[7])
#                         dict_aoss[mean_counter] = float(sample[8])
#                         dict_v_tan[mean_counter] = float(sample[9])
#                         dict_v_rad[mean_counter] = float(sample[10])
#                         dict_v_axial[mean_counter] = float(sample[11])
#                         mean_counter = mean_counter + 1
# 
#                 testnumber = int(len(list(dict_x.values())))
#                 prop = statistics.mean(list(dict_prop.values()))
#                 x_pos = statistics.mean(list(dict_x.values()))
#                 y_pos = statistics.mean(list(dict_y.values()))
#                 trq_mean = format(statistics.mean(list(dict_trq.values())), '.2f')
#                 thr_mean = format(statistics.mean(list(dict_thr.values())), '.2f')
#                 arspd_mean = format(statistics.mean(list(dict_arspd.values())), '.2f')
#                 aoa_mean = format(statistics.mean(list(dict_aoa.values())), '.2f')
#                 aoss_mean = format(statistics.mean(list(dict_aoss.values())), '.2f')
#                 v_tan_mean = format(statistics.mean(list(dict_v_tan.values())), '.2f')
#                 v_rad_mean = format(statistics.mean(list(dict_v_rad.values())), '.2f')
#                 v_axial_mean = format(statistics.mean(list(dict_v_axial.values())), '.2f')
# 
#                 mean_list = str(
#                     str(testnumber) + " " + str(prop) + " " + str(x_pos) + " " + str(y_pos) + " " +
#                     str(trq_mean) + " " + str(thr_mean) + " " + str(arspd_mean) + " " + str(aoa_mean) + " " +
#                     str(aoss_mean) + " " + str(v_tan_mean) + " " + str(v_rad_mean) + " " + str(v_axial_mean)
#                 )
# 
#                 var = format((float(x_pos) / 1000) * float(v_axial_mean), '.2f')
#                 var_list.append(float(var))
#                 trq_list.append(float(trq_mean))
#                 thr_list.append(float(thr_mean))
#         except:
#             pass
# 
#         with open(window.fname[0], newline='') as propfile:
#             read_data = csv.reader(propfile, delimiter=' ')
#             dict_blade_angle = {}
#             for r in read_data:
#                 line2 = ' '.join(r)
#                 sample2 = list(r)
#                 if sample2[0] == str(x_mp):
#                     dict_blade_angle[mean_counter] = sample2[1]
#                     angle_list = sample2[0] + " " + sample2[1] + " " + sample2[2]
# 
#         mean_data = list(mean_list.split(" "))
#         angle_data = list(angle_list.split(" "))
#         if mean_data[2] == angle_data[0]:
#             chord_angle_raw = angle_data[1]
#             chord_length_raw = angle_data[2]
# 
#             denominator1 = (float(omega_mode) * float(x_pos / 1000) - float(v_tan_mean))
#             if denominator1 == 0:
#                 chord_angle = 0.0
#                 chord_length = 0.0
#             else:
#                 denominator2 = math.cos(math.atan(float(v_rad_mean) / denominator1))
#                 vs1 = math.tan(math.radians(float(chord_angle_raw)))
#                 vs2 = vs1 * denominator2
#                 chord_angle = math.degrees(math.atan(vs2))
#                 chord_length = float(chord_length_raw) / math.cos(
#                     math.atan(float(v_rad_mean) / denominator1)
#                 )
#         else:
#             chord_angle_raw = 0.0
#             chord_angle = 0.0
#             chord_length_raw = 0.0
#             chord_length = 0.0
# 
#         mean_data.append(str(chord_angle_raw))
#         mean_data.append(str(format(chord_angle, '.2f')))
#         mean_data.append(str(chord_length_raw))
#         mean_data.append(str(format(chord_length, '.2f')))
# 
#         total_speed = math.sqrt(
#             math.pow(((float(omega_mode) * float(x_pos / 1000)) - float(v_tan_mean)), 2)
#             + math.pow(float(v_axial_mean), 2)
#             + math.pow(float(v_rad_mean), 2)
#         )
#         try:
#             helix_angle = math.degrees(math.asin(float(v_axial_mean) / float(total_speed)))
#         except:
#             helix_angle = 0
# 
#         mean_data.append(str(format(helix_angle, '.2f')))
#         alpha_angle = format(float(chord_angle) - helix_angle, '.2f')
#         mean_data.append(str(alpha_angle))
#         mean_data.append(str(format(total_speed, '.2f')))
#         v_lift = (
#             float(v_axial_mean) * math.cos(math.radians(float(helix_angle)))
#             + (float(v_tan_mean) * math.sin(math.radians(float(helix_angle))))
#         )
#         mean_data.append(str(format(v_lift, '.2f')))
#         v_drag = (
#             float(v_tan_mean) * math.cos(math.radians(float(helix_angle)))
#             - (float(v_axial_mean) * math.sin(math.radians(float(helix_angle))))
#         )
#         mean_data.append(str(format(v_drag, '.2f')))
# 
#         try:
#             coeff_lift = (2 * float(v_lift)) / float(total_speed)
#         except:
#             coeff_lift = 0
#         mean_data.append(str(format(coeff_lift, '.3f')))
# 
#         try:
#             coeff_drag = (2 * float(v_drag)) / float(total_speed)
#         except:
#             coeff_drag = 0
#         mean_data.append(str(format(coeff_drag, '.3f')))
# 
#         Re = (float(chord_length) / 1000 * float(total_speed)) / (
#             float(window.shared_data.kin_visc) * math.pow(10, -5)
#         )
#         mean_data.append(str(format(Re, '.0f')))
#         v_2d = math.sqrt(math.pow(float(v_axial_mean), 2) + math.pow(float(v_rad_mean), 2))
#         mean_data.append(str(format(v_2d, '.2f')))
#         mean_data.append(str(format(window.dr_ratio.value(), '.1f')))
# 
#         with open(os.path.join(window.path, mean_csvfile), 'a') as f:
#             w = csv.writer(f)
#             w.writerow([' '.join(mean_data)])
# 
#         window.update_plot_ax2(x_pos, v_tan_mean, v_rad_mean, v_axial_mean)
#         x_mp = x_mp + 3
# 
#     vi = (2 * (window.shared_data.x_delta / 1000) * sum(var_list)) / math.pow(
#         float(window.radius_mm) / 1000, 2
#     )
#     window.label13.setText(str(format(vi, '.2f')))
#     T = statistics.mean(thr_list)
#     Pi = float(vi) * float(T)
#     window.label15.setText(str(format(Pi, '.2f')))
#     try:
#         vv = float(T) / (
#             window.shared_data.rho
#             * math.pi
#             * math.pow(float(window.radius_mm) / 1000, 2)
#             * math.pow(float(vi), 2)
#         )
#     except:
#         vv = 0
#     window.label65.setText(str(format(vv, '.2f')))
#     M = statistics.mean(trq_list)
#     P = float(M) * float(omega_mode)
#     window.label17.setText(str(format(P, '.2f')))
#     vm = window.shared_data.rho * math.pi * math.pow(float(window.radius_mm) / 1000, 2) * float(vi)
#     window.label67.setText(str(format(vm, '.2f')))
#     try:
#         v_max_mean = float(T) / float(vm)
#     except:
#         v_max_mean = 0
#     window.label69.setText(str(format(v_max_mean, '.2f')))
#     try:
#         Ct = float(T) / (
#             window.shared_data.rho
#             * math.pow(float(omega_mode), 2)
#             * math.pow(((float(window.radius_mm) * 2) / 1000), 4)
#         )
#     except:
#         Ct = 0
#     try:
#         Cp = float(M) / (
#             window.shared_data.rho
#             * math.pow(float(omega_mode), 2)
#             * math.pow(((float(window.radius_mm) * 2) / 1000), 5)
#         )
#     except:
#         Cp = 0
#     try:
#         nu = (Pi / P) * 100
#         window.label19.setText(str(format(nu, '.2f')))
#     except:
#         nu = -1
#         window.label19.setText(str(nu))
# 
#     with open(os.path.join(window.path, mean_csvfile), 'a') as f:
#         w = csv.writer(f, delimiter=' ')
#         w.writerow(['Omega', format(float(omega_mode), '.2f'), 'rad/s'])
#         w.writerow(['Induced_power', format(Pi, '.2f'), 'W'])
#         w.writerow(['Power', format(P, '.2f'), 'W'])
#         w.writerow(['Efficiency', format(nu, '.2f'), '%'])
#         w.writerow(['Average_induced_speed', format(vi, '.2f'), 'm/s'])
#         w.writerow(['Airspeed_ratio', format(vv, '.2f')])
#         w.writerow(['V_mass', format(vm, '.2f'), 'kg/s'])
#         w.writerow(['V_max_mean', format(v_max_mean, '.2f'), 'm/s'])
#         w.writerow(['Ct', format(Ct, '.7f')])
#         w.writerow(['Cp', format(Cp, '.7f')])
#         w.writerow(['Air_density', window.shared_data.rho, 'kg/m3'])
#         w.writerow(['Air_kinematic_viscosity', window.shared_data.kin_visc, 'x10-5 m2/s'])
# 
#     var_list.clear()
#     trq_list.clear()
#     thr_list.clear()
#     window.counter = 0
#     window.cnv.draw_ax2()
# 
#     window.cnv.save_only_second_plot(os.path.join(window.path, plot_filename))






# import os, csv, math, statistics
# 
# def process_data(window):
#     plot_filename = f"log{window.today_dt}.png"
#     mean_csvfile = "log" + window.today_dt + "_mean.csv"
#     mean_header = [
#         '#_of_samples Prop_diam(inch) X_position(mm) Y_position(mm) '
#         'Torque(Nm) Thrust(N) Airspeed(m/s) AoA(deg) AoSS(deg) '
#         'V_tan(m/s) V_rad(m/s) V_axial(m/s) Chord_angle(deg) '
#         'Chord_angle_eff(deg) Chord_length(mm) Chord_length_eff(mm) '
#         'Helix_angle_eff(deg) Alpha_angle(deg) V_total(m/s) '
#         'V_lift(m/s) V_drag(m/s) CL CD Reynolds_number V_a+r(m/s) '
#         'D/R_ratio'
#     ]
#     
#     omega_values = []
#     try:
#         with open(os.path.join(self.path, self.csvfile), newline='') as csvfile:
#             row_read = csv.reader(csvfile, delimiter=' ')
#             for row in row_read:
#                 line = ' '.join(row)
#                 sample = list(line.split(" "))
#                 try:
#                     omega_value = float(sample[5])
#                     omega_values.append(omega_value)
#                 except (IndexError, ValueError):
#                     continue
#     except FileNotFoundError as e:
#         print(f"Error reading log file: {e}")
#         return
# 
#     if omega_values:
#         try:
#             omega_mode = statistics.mode(omega_values)
#         except statistics.StatisticsError as e:
#             print(f"Could not compute mode: {e}. Using mean instead.")
#             omega_mode = statistics.mean(omega_values)
#     else:
#         print("No omega values found in the log file.")
#         omega_mode = 0
# 
#     
#     with open(os.path.join(self.path,mean_csvfile), 'a') as h:
#         k = csv.writer(h)
#         k.writerow(mean_header)
#     x_max = 3 * math.floor((float(self.radius_mm) + (1 - (self.shared_data.safety_over_prop/100)))/3)
#     x_mp = 0
#     while x_mp <= x_max:
#         try:
#             with open(os.path.join(self.path,self.csvfile), newline='') as csvfile:
#                 row_read = csv.reader(csvfile, delimiter=' ')
#                 j = 0
#                 mean_counter = 0
#                 testnumber = 0
#                 dict_prop = {}
#                 dict_x = {}
#                 dict_y = {}
#                 dict_trq = {}
#                 dict_thr = {}
#                 dict_omega = {}
#                 dict_arspd = {}
#                 dict_aoa = {}
#                 dict_aoss = {}
#                 dict_v_tan = {}
#                 dict_v_rad = {}
#                 dict_v_axial = {}
#                 for row in row_read:
#                     line = ' '.join(row)
#                     j = j + 1
#                     sample = list(line.split(" "))
#                     if sample[1] == str(x_mp):
#                         dict_prop[mean_counter] = float(sample[0])
#                         dict_x[mean_counter] = int(sample[1])
#                         dict_y[mean_counter] = int(sample[2])
#                         dict_trq[mean_counter] = float(sample[3])
#                         dict_thr[mean_counter] = float(sample[4])
#                         dict_omega[mean_counter] = float(sample[5])
#                         dict_arspd[mean_counter] = float(sample[6])
#                         dict_aoa[mean_counter] = float(sample[7])
#                         dict_aoss[mean_counter] = float(sample[8])
#                         dict_v_tan[mean_counter] = float(sample[9])
#                         dict_v_rad[mean_counter] = float(sample[10])
#                         dict_v_axial[mean_counter] = float(sample[11])
#                         mean_counter = mean_counter + 1
#                         
#                 testnumber = int(len(list(dict_x.values())))
#                 prop = statistics.mean(list(dict_prop.values()))
#                 x_pos = statistics.mean(list(dict_x.values()))
#                 y_pos = statistics.mean(list(dict_y.values()))
#                 trq_mean = format(statistics.mean(list(dict_trq.values())),'.2f')
#                 thr_mean = format(statistics.mean(list(dict_thr.values())),'.2f')
#                 arspd_mean = format(statistics.mean(list(dict_arspd.values())),'.2f')
#                 aoa_mean = format(statistics.mean(list(dict_aoa.values())),'.2f')
#                 aoss_mean = format(statistics.mean(list(dict_aoss.values())),'.2f')
#                 v_tan_mean = format(statistics.mean(list(dict_v_tan.values())),'.2f')
#                 v_rad_mean = format(statistics.mean(list(dict_v_rad.values())),'.2f')
#                 v_axial_mean = format(statistics.mean(list(dict_v_axial.values())),'.2f')
#                 
#                 mean_list = str(str(testnumber)+" "+str(prop)+" "+str(x_pos)+" "+str(y_pos)+" "+str(trq_mean)+" "+str(thr_mean)+" "+str(arspd_mean)+" "+str(aoa_mean)+" "+str(aoss_mean)+" "+str(v_tan_mean)+" "+str(v_rad_mean)+" "+str(v_axial_mean))
#                 
#                 var = format((float(x_pos)/1000)*float(v_axial_mean),'.2f')
#                 var_list.append(float(var))
#                 trq_list.append(float(trq_mean))
#                 thr_list.append(float(thr_mean))
#         except:
#             pass
#             
#         with open(self.fname[0], newline = '') as propfile:
#             read_data = csv.reader(propfile, delimiter=' ')
#             dict_blade_angle = {}  
#             for r in read_data:
#                 line2 = ' '.join(r)
#                 sample2 = list(r)
#                 if sample2[0] == str(x_mp):
#                     dict_blade_angle[mean_counter] = sample2[1]
#                     angle_list = sample2[0]+" "+sample2[1]+" "+sample2[2]
#         mean_data = list(mean_list.split(" "))
#         angle_data = list(angle_list.split(" "))
#         if mean_data[2] == angle_data[0]:
#             chord_angle_raw = angle_data[1]
#             chord_length_raw = angle_data[2]
#             
#             denominator1 = (float(omega_mode)*float(x_pos/1000)-float(v_tan_mean))
#             if denominator1 == 0:
#                 chord_angle = 0.0
#                 chord_length = 0.0
#             else:
#                 denominator2 = math.cos(math.atan(float(v_rad_mean)/denominator1))
#                 vs1 = math.tan(math.radians(float(chord_angle_raw)))
#                 vs2 = vs1*denominator2
#                 chord_angle = math.degrees(math.atan(vs2))
#                 chord_length = float(chord_length_raw)/math.cos(math.atan(float(v_rad_mean)/denominator1))
#         else:
#             chord_angle_raw = 0.0
#             chord_angle = 0.0
#             chord_length_raw = 0.0
#             chord_length = 0.0
#         mean_data.append(str(chord_angle_raw))
#         mean_data.append(str(format(chord_angle,'.2f')))
#         mean_data.append(str(chord_length_raw))
#         mean_data.append(str(format(chord_length,'.2f')))
#         total_speed = math.sqrt(math.pow(((float(omega_mode)*float(x_pos/1000))-float(v_tan_mean)),2)+math.pow(float(v_axial_mean),2)+math.pow(float(v_rad_mean),2))
#         try:
#             helix_angle = math.degrees(math.asin(float(v_axial_mean)/float(total_speed)))
#         except:
#             helix_angle = 0
#         mean_data.append(str(format(helix_angle,'.2f')))
#         alpha_angle = format(float(chord_angle) - helix_angle,'.2f')
#         mean_data.append(str(alpha_angle))
#         mean_data.append(str(format(total_speed,'.2f'))) 
#         v_lift = (float(v_axial_mean) * math.cos(math.radians(float(helix_angle))) + (float(v_tan_mean) * math.sin(math.radians(float(helix_angle)))))
#         mean_data.append(str(format(v_lift,'.2f')))
#         v_drag = (float(v_tan_mean) * math.cos(math.radians(float(helix_angle))) - (float(v_axial_mean) * math.sin(math.radians(float(helix_angle)))))
#         mean_data.append(str(format(v_drag,'.2f')))
#         try:
#             coeff_lift = (2 * float(v_lift))/float(total_speed)
#         except:
#             coeff_lift = 0
#         mean_data.append(str(format(coeff_lift,'.3f')))
#         try:
#             coeff_drag = (2 * float(v_drag))/float(total_speed)
#         except:
#             coeff_drag = 0
#         mean_data.append(str(format(coeff_drag,'.3f')))
#         Re = (float(chord_length)/1000 * float(total_speed))/(float(self.shared_data.kin_visc) * math.pow(10,-5))
#         mean_data.append(str(format(Re,'.0f')))
#         v_2d = math.sqrt(math.pow(float(v_axial_mean),2) + math.pow(float(v_rad_mean),2))
#         mean_data.append(str(format(v_2d,'.2f')))
#         mean_data.append(str(format(self.dr_ratio.value(),'.1f')))
# #             diff_mass_rate = math.pi*self.shared_data.rho*float(v_axial_mean)*(self.radius_mm/1000)
# #             mean_data.append(str(format(diff_mass_rate,'.2f')))
# 
#         with open(os.path.join(self.path,mean_csvfile), 'a') as f:
#             w = csv.writer(f)
#             w.writerow([' '.join(mean_data)])
#         
#         self.update_plot_ax2(x_pos, v_tan_mean, v_rad_mean, v_axial_mean)
#         x_mp = x_mp + 3
#         
#     vi = (2*(self.shared_data.x_delta/1000)*sum(var_list))/math.pow(float(self.radius_mm)/1000,2)
#     self.label13.setText(str(format(vi,'.2f')))
#     T = statistics.mean(thr_list)
#     Pi = float(vi)*float(T)
#     self.label15.setText(str(format(Pi,'.2f')))
#     try:
#         vv = float(T)/(self.shared_data.rho * math.pi * math.pow(float(self.radius_mm)/1000,2) * math.pow(float(vi),2))
#     except:
#         vv = 0
#     self.label65.setText(str(format(vv,'.2f')))
#     M = statistics.mean(trq_list)
#     P = float(M)*float(omega_mode)
#     self.label17.setText(str(format(P,'.2f')))
#     vm = self.shared_data.rho * math.pi * math.pow(float(self.radius_mm)/1000,2) * float(vi)
#     self.label67.setText(str(format(vm,'.2f')))
#     try:
#         v_max_mean = float(T)/float(vm)
#     except:
#         v_max_mean = 0
#     self.label69.setText(str(format(v_max_mean,'.2f')))
#     try:
#         Ct = float(T)/(self.shared_data.rho * math.pow(float(omega_mode),2) * math.pow(((float(self.radius_mm)*2)/1000),4))
#     except:
#         Ct = 0
#     try:
#         Cp = float(M)/(self.shared_data.rho * math.pow(float(omega_mode),2) * math.pow(((float(self.radius_mm)*2)/1000),5))
#     except:
#         Cp = 0
#     try:
#         nu = (Pi/P)*100
#         self.label19.setText(str(format(nu,'.2f')))
#     except:
#         nu = -1
#         self.label19.setText(str(nu))
#     
#     with open(os.path.join(self.path,mean_csvfile), 'a') as f:
#         w = csv.writer(f, delimiter=' ')
#         w.writerow(['Omega',format(float(omega_mode),'.2f'),'rad/s'])
#         w.writerow(['Induced_power',format(Pi,'.2f'),'W'])
#         w.writerow(['Power',format(P,'.2f'),'W'])
#         w.writerow(['Efficiency',format(nu,'.2f'),'%'])
#         w.writerow(['Average_induced_speed',format(vi,'.2f'),'m/s'])
#         w.writerow(['Airspeed_ratio',format(vv,'.2f')])
#         w.writerow(['V_mass',format(vm,'.2f'),'kg/s'])
#         w.writerow(['V_max_mean',format(v_max_mean,'.2f'),'m/s'])
#         w.writerow(['Ct',format(Ct,'.7f')])
#         w.writerow(['Cp',format(Cp,'.7f')])
#         w.writerow(['Air_density',self.shared_data.rho,'kg/m3'])
#         w.writerow(['Air_kinematic_viscosity',self.shared_data.kin_visc,'x10-5 m2/s'])
#     
#     var_list.clear()
#     trq_list.clear()
#     thr_list.clear()
#     self.counter = 0
#     self.cnv.draw_ax2()
#     
#     self.cnv.save_only_second_plot(os.path.join(self.path, plot_filename))
# 