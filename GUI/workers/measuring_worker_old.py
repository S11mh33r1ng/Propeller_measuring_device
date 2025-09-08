# workers/measuring_worker.py
from __future__ import annotations

import csv
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QTimer

MEAS_PREFIX = "Measurements:"  # exact prefix printed by the MCU


@dataclass
class MeasurePoint:
    x_steps: int
    y_steps: int


class MeasuringWorker(QObject):
    """
    Drives the stepper via 'm|X|Y|' commands, and logs MCU 'Measurements:' frames.
    - points are given in *steps* (your trajectory files already use steps)
    - main window should parse the incoming line and emit:
        measurementsFrame.emit(x_steps, y_steps, vals13)
      where vals13 is a list of 13 floats in this order:
        [ X, Y, thr1, trq1, rpm1, airspeed, aoa_raw, aoa_abs, aoss_raw, aoss_abs, thr2, trq2, rpm2 ]
    - we average N samples per point (samples_per_point) before logging and advancing
    """

    # Outbound: connect this to MainWindow's serial write slot
    sendData = pyqtSignal(str)

    # Status & lifecycle
    progress = pyqtSignal(int, int)            # (current_index, total)
    pointStarted = pyqtSignal(int, int, int)   # (x_steps, y_steps, index)
    pointDone = pyqtSignal(int)                # index
    finished = pyqtSignal(str)                 # csv path
    error = pyqtSignal(str)

    def __init__(self,
                 points: Iterable[Tuple[int, int]],
                 csv_path: str,
                 samples_per_point: int = 1,
                 settle_timeout_s: float = 15.0,
                 overall_timeout_s: Optional[float] = None,
                 tandem_setup: bool = False,
                 motor_pwm1: Optional[int] = None,
                 motor_pwm2: Optional[int] = None,
                 arrival_tolerance_steps: int = 2,
                 steps_per_mm: Optional[float] = None,
                 parent: Optional[QObject] = None):
        super().__init__(parent)
        self._points: List[MeasurePoint] = [MeasurePoint(int(x), int(y)) for x, y in points]
        self._csv_path = csv_path
        self._samples_per_point = max(1, int(samples_per_point))
        self._settle_timeout_s = max(0.1, float(settle_timeout_s))
        self._overall_timeout_s = overall_timeout_s
        self._is_tandem = bool(tandem_setup)
        self._motor_pwm1 = motor_pwm1
        self._motor_pwm2 = motor_pwm2
        self._arrival_tol = max(0, int(arrival_tolerance_steps))
        p = self.parent()
        try:
            ratio = float(getattr(p, "shared_data").ratio)
        except Exception:
            ratio = 1.0
        self._bin_delta_mm    = float(getattr(getattr(p, "shared_data", object()), "x_delta", 3.0))
        self._bin_delta_steps = max(1, int(round(self._bin_delta_mm * ratio)))
        self._goal_x          = self._points[-1].x_steps  # optional: for final proximity check

        # fresh-run reset to get X_mm/Y_mm starting from 0
        self._y0_steps = None
        self._x_start_steps = None
        self._bins_logged = 0
        self._bin_samples.clear()
        self._logged_zero = False
        
        self._waiting_tare = False
        self._pending_motor_start = False
        
        self._csv_file = None
        self._csv_writer: Optional[csv.writer] = None
        
        self._bin_mode = False
        self._bin_delta_mm = 3.0       # default; overridden from MainWindow.shared_data.x_delta if present
        self._bin_delta_steps = 0
        self._x_start_steps = None
        self._bins_logged = 0
        self._bin_samples: List[List[float]] = []
        self._goal_x = None
        
        # 1-prop header (10 fields, no second propeller columns)
        self.CSV_HEADER_1P = [
            "X_mm", "Y_mm",
            "thr1_N", "trq1_Nm", "rpm1",
            "airspeed", "aoa_raw", "aoa_abs",
            "aoss_raw", "aoss_abs",
        ]

        self.CSV_HEADER_2P = [
            "X_mm", "Y_mm",
            "thr1_N", "trq1_Nm", "rpm1",
            "airspeed", "aoa_raw", "aoa_abs",
            "aoss_raw", "aoss_abs",
            "thr2_N", "trq2_Nm", "rpm2",
        ]
        self._csv_header = self.CSV_HEADER_2P if self._is_tandem else self.CSV_HEADER_1P

    # ---------- Public API ----------

    @pyqtSlot()
    def start(self):
        """Begin the measurement sequence."""
        self._y0_steps = None
        self._cur_idx = -1
        self._cur_target = None
        self._cur_samples = []
        self._bin_samples.clear()
        self._x_start_steps = None
        self._bins_logged = 0
        
        if self._running:
            return
        if not self._points:
            self.error.emit("No points to measure.")
            return

        self._running = True
        self._t_start_overall = time.monotonic()
        
        log_path = Path(self._csv_path)
        if log_path.exists():
            ts = datetime.now().strftime("%H%M%S")
            log_path = log_path.with_name(f"{log_path.stem}_{ts}{log_path.suffix}")
            self._csv_path = str(log_path)
        
        try:
            Path(self._csv_path).parent.mkdir(parents=True, exist_ok=True)  # safety
            self._csv_file = open(self._csv_path, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(self._csv_header)
            self._csv_file.flush()
        except Exception as e:
            self._running = False
            self.error.emit(f"Failed to open log file:\n{e}")
            return

        self.sendData.emit("BeaconON")
        time.sleep(3)
        self._waiting_tare = True
        self._pending_motor_start = True
        #time.sleep(1)
        self.sendData.emit("tare")
        
        self._bin_mode = (len(self._points) == 1)
        if self._bin_mode:
            p = self.parent()
            try:
                ratio = float(getattr(p.shared_data, "ratio", 1.0))
            except Exception:
                ratio = 1.0
            self._bin_delta_mm = float(getattr(getattr(p, "shared_data", object()), "x_delta", 3.0))
            self._bin_delta_steps = max(1, int(round(self._bin_delta_mm * ratio)))
            self._goal_x = self._points[0].x_steps

        self._cur_idx = -1
        #self._advance_to_next_point()
        
    @pyqtSlot()
    def on_tare_done(self):
        if not self._waiting_tare:
            return
        self._waiting_tare = False
        time.sleep(3)
        # Start motors after tare
        print(self._motor_pwm1)
        if self._pending_motor_start:
            if self._motor_pwm1 is not None and self._motor_pwm2 is not None:
                self.sendData.emit(f"startMotor|{self._motor_pwm1}|{self._motor_pwm2}")
            else:
                # Fallback: safe idle (single-motor rigs)
                self.sendData.emit("startMotor|1000|1000")
        # small delay to let ESCs spin up
        QTimer.singleShot(10_000, self._kickoff_points)
        
    @pyqtSlot()
    def _kickoff_points(self):
        # this is what you currently do at the end of start():
        #self._cur_idx = -1
        self._advance_to_next_point()

    @pyqtSlot()
    def cancel(self):
        """Abort the sequence."""
        self._finish("Canceled by user.")

    # MainWindow should connect its parsed frame signal to this slot:
    #   self.measurementsFrame.connect(worker.on_measurements)
    # Signature: (int x_meas, int y_meas, object vals13)
    @pyqtSlot(int, int, object)
    def on_measurements(self, x_meas: int, y_meas: int, vals_obj):
        try:
            vals = list(vals_obj)
        except Exception:
            return
        expected = 13 if self._is_tandem else 10
        if len(vals) < expected:
            vals += [0.0] * (expected - len(vals))
        elif len(vals) > expected:
            vals = vals[:expected]
                
        if not self._running:
            return

        if self._bin_mode:
            if self._x_start_steps is None:
                self._x_start_steps = int(self._x_center_steps)
                self._y0_steps      = y_meas
                self._bins_logged   = 0
                self._bin_samples.clear()
                self._logged_zero   = False

            # collect current frame into the bin
            self._bin_samples.append(vals)

            # emit 0-mm baseline row once so X_mm starts at 0 in the CSV
            if not self._logged_zero:
                baseline = list(vals)
                baseline[0] = float(x_meas); baseline[1] = float(y_meas)
                self._write_row(baseline)
                self._logged_zero = True

            # flush a row every Δx (in steps measured from center)
            traveled_steps = abs(int(x_meas) - int(self._x_start_steps))
            next_edge = (self._bins_logged + 1) * self._bin_delta_steps
            if traveled_steps >= next_edge:
                averaged = self._average_samples(self._bin_samples) or list(vals)
                averaged[0] = float(x_meas); averaged[1] = float(y_meas)
                self._write_row(averaged)
                self._bins_logged += 1
                self._bin_samples.clear()

            # finish when we reach the goal (within tolerance)
            if self._goal_x is not None and abs(int(x_meas) - int(self._goal_x)) <= self._arrival_tol:
                if self._bin_samples:
                    averaged = self._average_samples(self._bin_samples) or list(vals)
                    averaged[0] = float(x_meas); averaged[1] = float(y_meas)
                    self._write_row(averaged)
                self._finish()
            return
        
        if self._cur_target is None:
            return

        dx = abs(x_meas - self._cur_target.x_steps)
        dy = abs(y_meas - self._cur_target.y_steps)
        if dx <= self._arrival_tol and dy <= self._arrival_tol:
            self._cur_samples.append(vals)
            if self._y0_steps is None:
                self._y0_steps = y_meas
            if (time.monotonic() - self._t_start_point) > self._settle_timeout_s or \
               len(self._cur_samples) >= self._samples_per_point:
                averaged = self._average_samples(self._cur_samples) or list(vals)
                averaged[0] = float(x_meas); averaged[1] = float(y_meas)
                self._write_row(averaged)
                self.pointDone.emit(self._cur_idx)
                self._advance_to_next_point()

#         # --- original per-point mode (kept for multi-point trajectories) ---
#         if self._cur_target is None:
#             return
#         if x_meas == self._cur_target.x_steps and y_meas == self._cur_target.y_steps:
#             self._cur_samples.append(vals)
#             if self._y0_steps is None:
#                 self._y0_steps = y_meas
#             if (time.monotonic() - self._t_start_point) > self._settle_timeout_s:
#                 if self._cur_samples:
#                     averaged = self._average_samples(self._cur_samples) or list(vals)
#                     averaged[0] = float(x_meas)
#                     averaged[1] = float(y_meas)
#                     self._write_row(averaged)
#                 self.pointDone.emit(self._cur_idx)
#                 self._advance_to_next_point()
#                 return
#             if len(self._cur_samples) >= self._samples_per_point:
#                 averaged = self._average_samples(self._cur_samples) or list(vals)
#                 averaged[0] = float(x_meas)
#                 averaged[1] = float(y_meas)
#                 self._write_row(averaged)
#                 self.pointDone.emit(self._cur_idx)
#                 self._advance_to_next_point()
    # ---------- internals ----------

    def _average_samples(self, samples):
        """Return the element-wise average; None if samples is empty."""
        if not samples:
            return None
        n = len(samples)
        acc = [0.0] * len(samples[0])
        for row in samples:
            if len(row) != len(acc):
                continue  # skip malformed rows defensively
            for i, v in enumerate(row):
                acc[i] += float(v)
        return [a / n for a in acc]

    def _send_move(self, pt: MeasurePoint):
        """MCU expects m|X|Y|feed_xy|feed_y"""
        feed_xy = 200
        try:
            mw = self.parent()
            feed_xy = int(getattr(mw, "measure_speed").value())
        except Exception:
            pass
        feed_y = max(1, int(feed_xy / 3))
        self.sendData.emit(f"m|{pt.x_steps}|{pt.y_steps}|{feed_xy}|{feed_y}")

    def _advance_to_next_point(self):
        """Move to next point or finish."""
        # Overall timeout check
        if self._overall_timeout_s is not None:
            if time.monotonic() - self._t_start_overall > self._overall_timeout_s:
                self._finish("Overall timeout.")
                return

        self._cur_idx += 1
        total = len(self._points)
        if self._cur_idx >= total:
            self._finish()
            return

        self.progress.emit(self._cur_idx, total)
        self._cur_target = self._points[self._cur_idx]
        self._cur_samples = []
        self._t_start_point = time.monotonic()
        self.pointStarted.emit(self._cur_target.x_steps, self._cur_target.y_steps, self._cur_idx)

        # Kick off motion
        self._send_move(self._cur_target)

        # Harmless nudge to keep serial alive in some stacks
        self.sendData.emit("")

    def _write_row(self, *args):
        """
        Accepts:
          - _write_row(vals13)
          - _write_row(x_steps, y_steps, vals13)  # legacy call sites OK
        """
        if not self._csv_writer:
            return
        try:
            # normalize args to vals13 list with X/Y injected
            if len(args) == 1:
                vals13 = list(args[0])
            elif len(args) == 3:
                x_steps, y_steps, vals = args
                vals13 = list(vals)
                vals13[0] = float(x_steps)
                vals13[1] = float(y_steps)
            else:
                return

            spmm = self._steps_per_mm if self._steps_per_mm else 1.0
            xc   = self._x_center_steps if hasattr(self, "_x_center_steps") else 0
            y0   = self._y0_steps if self._y0_steps is not None else int(round(vals13[1]))

            x_steps_i = int(round(float(vals13[0])))
            y_steps_i = int(round(float(vals13[1])))

            x_mm = abs(x_steps_i - xc) / spmm
            if x_mm < 0.5:
                x_mm = 0.0
            y_mm = (y_steps_i - y0) / spmm

            X = int(round(x_mm))
            Y = int(round(y_mm))
            def r(x, nd=3): return round(float(x), nd)

            thr1, trq1, rpm1 = r(vals13[2], 3), r(vals13[3], 3), int(round(vals13[4]))
            air, aoa_r, aoa_a = r(vals13[5], 2), r(vals13[6], 2), r(vals13[7], 2)
            aoss_r, aoss_a    = r(vals13[8], 2), r(vals13[9], 2)

            if self._is_tandem:
                thr2, trq2, rpm2 = r(vals13[10], 3), r(vals13[11], 3), int(round(vals13[12]))
                row = [X, Y, thr1, trq1, rpm1, air, aoa_r, aoa_a, aoss_r, aoss_a, thr2, trq2, rpm2]
            else:
                row = [X, Y, thr1, trq1, rpm1, air, aoa_r, aoa_a, aoss_r, aoss_a]

            self._csv_writer.writerow(row)
            self._csv_file.flush()
        except Exception as e:
            self.error.emit(f"Failed to write CSV row:\n{e}")

    def _finish(self, reason_ok: Optional[str] = None):
        """Close CSV and announce completion or an error message."""
        if self._csv_file:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except Exception:
                pass
        self._csv_file = None
        self._csv_writer = None

        was_running = self._running
        self._running = False
        self._cur_target = None
        self._cur_samples = []

        if was_running:
            if reason_ok is None:
                self.finished.emit(self._csv_path)
            else:
                self.error.emit(reason_ok)
                self.finished.emit(self._csv_path)



# import app_globals
# from PyQt5.QtCore import QObject, QTimer, pyqtSignal, pyqtSlot
# import time, datetime, os, csv, math, statistics
# 
# TAKE_EVERY_MS = 100      # measure() tick
# TARE_TIMEOUT_S = 8       # don't wait forever
# 
# class MeasuringWorker(QObject):
#     finished     = pyqtSignal()
#     progress     = pyqtSignal(int, str)
#     input_data   = pyqtSignal(str)
#     requestStart = pyqtSignal()
#     requestStop  = pyqtSignal()
#     stopTimer    = pyqtSignal()
# 
#     def __init__(self, shared_data):
#         super().__init__()
#         self.shared_data = shared_data
#         self._running = False
#         self.measuring_stopped = True
# 
#         # state
#         self.first_line_done = False
#         self.header_added = False
#         self.goal_reached = False
#         self.send_once = False
# 
#         # rolling averages between x-delta buckets
#         self.aoss_a = []
#         self.aoa_a  = []
#         self.arspd_a = []
#         self.omega_a = []
# 
#         # current bucket reference
#         self.x_prev = 0
#         self.x_normalized_mm = 0
# 
#         # trajectory
#         self.list_of_x_targets = []
#         self.list_of_y_targets = []
#         self.b = 0  # index into custom lists
# 
#         # log path bits
#         self.parent_dir = "/home/siim/Desktop/logid/"
#         self.header = ['Prop_diam(inch) ' 'X_position(mm) ' 'Y_position(mm) '
#                        'Torque(Nm) ' 'Thrust(N) ' 'Omega(rad/s) '
#                        'Airspeed(m/s) ' 'AoA(deg) ' 'AoSS(deg) '
#                        'V_tan(m/s) ' 'V_rad(m/s) ' 'V_axial(m/s)']
# 
#         # timer
#         self.meas_timer = QTimer(self)
#         self.meas_timer.timeout.connect(self.measure)
#         self.stopTimer.connect(self.stop_timer)
# 
#     # -------------------------
#     # lifecycle
#     # -------------------------
#     @pyqtSlot()
#     def run(self):
#         self._running = True
#         self.start_measuring()
# 
#     def stop_measuring(self):
#         self._running = False
#         self.stopTimer.emit()
# 
#     @pyqtSlot()
#     def stop_timer(self):
#         self.meas_timer.stop()
#         self._running = False
#         print("Measurement stopped")
# 
#     # -------------------------
#     # helpers
#     # -------------------------
#     def _ensure_log_dir(self):
#         w = app_globals.window
#         if w.counter == 0:
#             w.today_dt = datetime.datetime.today().strftime('%d-%m-%Y-%H:%M:%S')
#             w.path = os.path.join(self.parent_dir, w.today_dt)
#             os.makedirs(w.path, exist_ok=True)
#             w.csvfile = f"log{w.today_dt}.csv"
#             self.header_added = False
# 
#     def _write_row(self, pieces):
#         """Maintain legacy CSV format: each row is one comma-separated string."""
#         w = app_globals.window
#         line = ','.join(map(str, pieces))
#         with open(os.path.join(w.path, w.csvfile), 'a', newline='') as f:
#             writer = csv.writer(f)
#             if not self.header_added:
#                 writer.writerow(self.header)
#                 self.header_added = True
#             writer.writerow([line])
# 
#     def _wait_for_tare(self, timeout_s=TARE_TIMEOUT_S):
#         """Wait for UI to flip tare_done, with timeout to avoid hangs."""
#         w = app_globals.window
#         start = time.time()
#         while not w.tare_done and (time.time() - start) < timeout_s:
#             time.sleep(0.05)
#         return w.tare_done
# 
#     def _build_targets(self):
#         """Prepare X/Y step targets for the MCU."""
#         w = app_globals.window
#         self.list_of_x_targets = []
#         self.list_of_y_targets = []
# 
#         # radius & x_goal
#         w.radius_mm = format(
#             float((w.prop.value() * 25.4 * (1 + self.shared_data.safety_over_prop / 100)) / 2), '.1f'
#         )
#         radius_steps = int(float(w.radius_mm) * float(self.shared_data.ratio))
#         w.x_goal = int(self.shared_data.x_center * self.shared_data.ratio) - radius_steps
# 
#         if getattr(w, 'custom_trajectory', False):
#             self.list_of_x_targets = list(getattr(w, 'list_of_x_targets', []))
#             self.list_of_y_targets = list(getattr(w, 'list_of_y_targets', []))
#         else:
#             self.list_of_x_targets.append(w.x_goal)
#             self.list_of_y_targets.append(0)
# 
#     def _start_stream_and_motors(self):
#         w = app_globals.window
#         # stream
#         #w.sendData('streamStart')
#         #time.sleep(2)
# 
#         # ESCs -> PWM values
#         first_throttle  = int(1000 + (w.first_throttle.value() * 10))
#         second_throttle = int(1000 + (w.second_throttle.value() * 10)) if w.tandem_setup else 1000
#         w.sendData(f"startMotor|{first_throttle}|{second_throttle}")
#         time.sleep(10)
# 
#     def _send_move(self, x_steps, y_steps, feed_xy, feed_y):
#         app_globals.window.sendData(f"m|{x_steps}|{y_steps}|{feed_xy}|{feed_y}")
# 
#     @staticmethod
#     def _omega_from_rpm(rpm):
#         try:
#             return float(rpm) * 2.0 * math.pi / 60.0
#         except Exception:
#             return 0.0
# 
#     @staticmethod
#     def _vel_components(airspeed, aoa_deg, aoss_deg):
#         """Compute V_tan / V_rad / V_axial from scalar airspeed & angles (deg)."""
#         try:
#             aoa = math.radians(float(aoa_deg))
#             aoss = math.radians(float(aoss_deg))
#             v = float(airspeed)
#             v_tan   = v * math.sin(aoa)
#             v_rad   = v * math.cos(aoa) * math.sin(aoss)
#             v_axial = v * math.cos(aoa) * math.cos(aoss)
#             return (f"{v_tan:.2f}", f"{v_rad:.2f}", f"{v_axial:.2f}")
#         except Exception:
#             return ("0.00", "0.00", "0.00")
# 
#     # -------------------------
#     # main workflow
#     # -------------------------
#     def start_measuring(self):
#         w = app_globals.window
# 
#         # Reset UI bits
#         w.cnv.clear_plots()
#         for lbl in (w.label13, w.label15, w.label17, w.label19, w.label65, w.label67, w.label69):
#             lbl.clear()
#         w.measure.setEnabled(False)
#         w.Y_move.setEnabled(False)
#         w.measuring_stopped = False
#         w.test_progress.setMaximum(w.sweep_count.value())
# 
#         self.first_line_done = False
#         self.goal_reached = False
#         self.send_once = True
#         self.b = 0
#         self.x_prev = 0
#         self.x_normalized_mm = 0
# 
#         # path + header
#         self._ensure_log_dir()
# 
#         # build trajectory
#         self._build_targets()
# 
#         # beacon + tare
#         w.sendData('BeaconON')
#         time.sleep(3)
#         w.tare_done = False
#         w.sendData('tare')
#         self._wait_for_tare()
# 
#         # stream + motors
#         w.progress.setValue(5)
#         self._start_stream_and_motors()
#         w.progress.setValue(10)
# 
#         # small centering jog (as in your original)
#         try:
#             self._send_move(int((self.shared_data.x_center - 3) * self.shared_data.ratio),
#                             int(w.Y_pos.value() * self.shared_data.ratio),
#                             w.measure_speed.value(), int(w.measure_speed.value()/3))
#             time.sleep(2)
#             self._send_move(int(self.shared_data.x_center * self.shared_data.ratio),
#                             int(w.Y_pos.value() * self.shared_data.ratio),
#                             w.measure_speed.value(), int(w.measure_speed.value()/3))
#             time.sleep(3)
#         except Exception:
#             pass
# 
#         # kick the measuring loop
#         self.meas_timer.start(TAKE_EVERY_MS)
# 
#     def measure(self):
#         if not self._running:
#             self.meas_timer.stop()
#             return
# 
#         w = app_globals.window
# 
#         # First move command (either default or first of custom trajectory)
#         if self.send_once:
#             try:
#                 if not getattr(w, 'custom_trajectory', False):
#                     self._send_move(self.list_of_x_targets[0],
#                                     int(w.Y_pos.value() * self.shared_data.ratio),
#                                     w.measure_speed.value(), int(w.measure_speed.value()/3))
#                     self.current_x_target = self.list_of_x_targets[0]
#                     self.current_y_target = int(w.Y_pos.value() * self.shared_data.ratio)
#                 else:
#                     self._send_move(self.list_of_x_targets[self.b],
#                                     int(w.Y_pos.value() * self.shared_data.ratio) + self.list_of_y_targets[self.b],
#                                     w.measure_speed.value(), int(w.measure_speed.value()/3))
#                     self.current_x_target = self.list_of_x_targets[self.b]
#                     self.current_y_target = int(w.Y_pos.value() * self.shared_data.ratio) + self.list_of_y_targets[self.b]
#                 self.send_once = False
#             except Exception:
#                 # wait next tick
#                 pass
# 
#         # Pull the newest parsed values from MainWindow (set in handleData)
#         x_steps = getattr(w, 'x_pos', 0)
#         y_steps = getattr(w, 'y_pos', 0)
# 
#         first_thrust_N  = getattr(w, 'first_thrust_value', 0.0)
#         first_torque_Nm = getattr(w, 'first_torque_value', 0.0)
#         rpm1            = getattr(w, 'first_rpm', 0.0)
#         airspeed        = getattr(w, 'airspeed', 0.0)
#         aoa_abs         = getattr(w, 'aoa_abs_value', 0.0)
#         aoss_abs        = getattr(w, 'aoss_abs_value', 0.0)
# 
#         # derived
#         omega = self._omega_from_rpm(rpm1)
# 
#         # x normalization for plotting & bucketing
#         try:
#             x_mm_now = w.steps_to_mm(x_steps)
#             self.x_normalized_mm = float(self.shared_data.x_center) - float(x_mm_now)
#         except Exception:
#             self.x_normalized_mm = 0.0
# 
#         # First “zero” line written once at x == 0 bucket
#         if (int(self.x_normalized_mm) == 0 and not self.first_line_done):
#             v_tan0, v_rad0, v_axial0 = self._vel_components(airspeed, aoa_abs, aoss_abs)
#             row = [f"{w.prop.value():.1f}",
#                    f"{self.x_normalized_mm:.0f}",
#                    f"{w.Y_pos.value()}",
#                    f"{first_torque_Nm}",
#                    f"{first_thrust_N}",
#                    f"{omega}",
#                    f"{airspeed:.2f}",
#                    f"{aoa_abs:.2f}",
#                    f"{aoss_abs:.2f}",
#                    v_tan0, v_rad0, v_axial0]
#             self._write_row(row)
#             self.first_line_done = True
#             self.x_prev = self.x_normalized_mm
# 
#         # accumulate within x-delta window
#         if self.first_line_done and (int(self.x_normalized_mm) - int(self.x_prev) < int(self.shared_data.x_delta)):
#             try:
#                 self.aoss_a.append(float(aoss_abs))
#                 self.aoa_a.append(float(aoa_abs))
#                 self.arspd_a.append(float(airspeed))
#                 self.omega_a.append(float(omega))
#             except Exception:
#                 pass
# 
#         # once we’ve advanced by >= x_delta mm: average, log, plot, advance bucket
#         if self.first_line_done and (int(self.x_normalized_mm) - int(self.x_prev) >= int(self.shared_data.x_delta)):
#             try:
#                 aoss_avg = statistics.mean(self.aoss_a) if self.aoss_a else 0.0
#                 aoa_avg  = statistics.mean(self.aoa_a)  if self.aoa_a  else 0.0
#                 arspd_avg= statistics.mean(self.arspd_a)if self.arspd_a else 0.0
#                 if arspd_avg > 100.0:  # your guard
#                     arspd_avg = 0.0
#                 omega_avg= statistics.mean(self.omega_a) if self.omega_a else 0.0
# 
#                 v_tan, v_rad, v_axial = self._vel_components(arspd_avg, aoa_avg, aoss_avg)
# 
#                 row = [f"{w.prop.value():.1f}",
#                        f"{self.x_normalized_mm:.0f}",
#                        f"{w.steps_to_mm(y_steps)}",
#                        f"{first_torque_Nm}",
#                        f"{first_thrust_N}",
#                        f"{omega_avg:.2f}",
#                        f"{arspd_avg:.2f}",
#                        f"{aoa_avg:.2f}",
#                        f"{aoss_avg:.2f}",
#                        v_tan, v_rad, v_axial]
#                 self._write_row(row)
# 
#                 # plot
#                 w.update_plot_ax1(self.x_normalized_mm, omega_avg, arspd_avg, aoa_avg, aoss_avg,
#                                   first_torque_Nm, first_thrust_N)
# 
#             finally:
#                 # reset bucket
#                 self.aoss_a.clear(); self.aoa_a.clear(); self.arspd_a.clear(); self.omega_a.clear()
#                 self.x_prev = self.x_normalized_mm
# 
#         # progress bar
#         try:
#             fg = self.shared_data.x_center - int(w.steps_to_mm(w.x_goal))
#             x_progress = round((int(self.x_normalized_mm) / int(fg)) * 90, 0)
#             w.progress.setValue(10 + int(x_progress))
#         except Exception:
#             pass
# 
#         # finish / next point logic
#         if getattr(w, 'meas_data_running', False):
#             # reached end of single sweep?
#             if (x_steps <= w.x_goal) and not self.goal_reached:
#                 self.goal_reached = True
#                 w.counter += 1
#                 w.test_progress.setValue(w.counter)
#                 w.meas_data_running = False
#                 self._check_progress(w.counter, x_steps)
# 
#             # custom trajectory waypoint reached?
#             if (getattr(w, 'custom_trajectory', False)
#                 and x_steps == getattr(self, 'current_x_target', 0)
#                 and y_steps == getattr(self, 'current_y_target', 0)
#                 and not self.goal_reached):
#                 self.b += 1
#                 self.send_once = (self.b < len(self.list_of_x_targets))
#         else:
#             self.x_normalized_mm = 0.0
# 
#     def _check_progress(self, cycles_done, x_curr_steps):
#         w = app_globals.window
#         if cycles_done == w.sweep_count.value():
#             w.measuring_stopped = True
#             w.come_back()
#             self.stop_timer()
#             w.process_data()
#             return
# 
#         # prepare next cycle
#         self.goal_reached = False
#         time.sleep(1)
#         w.sendData('stop')
#         time.sleep(2)
#         # jog back to where we were
#         w.sendData(f'j|{x_curr_steps}|0|{w.jog_speed.value()}|{w.jog_speed.value()}')
#         time.sleep(3)
#         # center
#         w.sendData('center')
#         time.sleep(5)
#         # move to start line at current Y
#         w.sendData(f'j|{int(self.shared_data.x_center * self.shared_data.ratio)}|{int(w.Y_pos.value() * self.shared_data.ratio)}|{w.jog_speed.value()}|{w.jog_speed.value()}')
#         # start next run immediately inside same worker
#         self._running = True
#         self.start_measuring()
# 
# 
# 
# # import app_globals
# # from plot.canvas import Canvas
# # from PyQt5.QtCore import QObject, QTimer, pyqtSignal, pyqtSlot
# # import time, datetime, os, csv, math, statistics
# # 
# # class MeasuringWorker(QObject):
# #     finished = pyqtSignal()
# #     progress = pyqtSignal(int, str)
# #     input_data = pyqtSignal(str)
# #     requestStart = pyqtSignal()
# #     requestStop = pyqtSignal()
# #     stopTimer = pyqtSignal()
# # 
# #     def __init__(self, shared_data):
# #         super(MeasuringWorker, self).__init__()
# #         self.shared_data = shared_data
# #         self._running = False
# #         self.stopTimer.connect(self.stop_timer)
# #         self.measuring_stopped = True
# #         self.e_stop = False
# #         self.rpm = 0
# #         self.cnv = Canvas()
# #         self.counter = 0
# #         self.custom_trajectory = False
# #         self.omega_avg = 0
# #         self.arspd_avg = 0
# #         self.aoa_avg = 0
# #         self.aoss_avg = 0
# #         self.v_tan = 0
# #         self.v_rad = 0
# #         self.v_axial = 0
# #         self.meas_timer = QTimer(self)
# #         self.meas_timer.timeout.connect(self.measure)
# #         self.x_goal = 0
# #         self.x_prev = 0
# #         self.first_line_done = False
# #         self.header_added = False
# #         self.x_normalized_mm = 0
# #         self.goal_reached = False
# #         self.b = 0
# #         self.time_delay = 0
# #         self.send_once = False
# #         self.cycle_time = 1
# #         self.current_x_target = 0
# #         self.current_y_target = 0
# #         self.data = ''
# #         self.list_of_x_targets = []
# #         self.list_of_y_targets = []
# #    
# #     def start_measuring(self):
# #         self.goal_reached = False
# #         app_globals.window.tare_done = False
# #         self.wait = 0
# #         if app_globals.window.counter == 0:
# #             app_globals.window.today_dt = datetime.datetime.today().strftime('%d-%m-%Y-%H:%M:%S')
# #             self.parent_dir = "/home/siim/Desktop/logid/"
# #             app_globals.window.path = os.path.join(self.parent_dir, app_globals.window.today_dt)
# #             os.mkdir(app_globals.window.path)
# #             app_globals.window.csvfile = "log" + app_globals.window.today_dt + ".csv"
# #             self.header = ['Prop_diam(inch) ' 'X_position(mm) ' 'Y_position(mm) ' 'Torque(Nm) ' 'Thrust(N) ' 'Omega(rad/s) ' 'Airspeed(m/s) ' 'AoA(deg) ' 'AoSS(deg) ' 'V_tan(m/s) ' 'V_rad(m/s) ' 'V_axial(m/s)']
# #             self.header_added = False
# #         app_globals.window.cnv.clear_plots()
# #         self.aoss_a = []
# #         self.aoa_a = []
# #         self.arspd_a = []
# #         self.omega_a = []
# #         app_globals.window.label13.clear()
# #         app_globals.window.label15.clear()
# #         app_globals.window.label17.clear()
# #         app_globals.window.label19.clear()
# #         app_globals.window.label65.clear()
# #         app_globals.window.label67.clear()
# #         app_globals.window.label69.clear()
# #         app_globals.window.measure.setEnabled(False)
# #         app_globals.window.Y_move.setEnabled(False)
# #         app_globals.window.measuring_stopped = False
# #         app_globals.window.test_progress.setMaximum(app_globals.window.sweep_count.value())
# #         self.first_line_done = False
# #         app_globals.window.radius_mm = format(float((app_globals.window.prop.value()*25.4*(1 + self.shared_data.safety_over_prop/100))/2),'.1f')
# #         self.radius_steps = format((float(app_globals.window.radius_mm) * float(self.shared_data.ratio)),'.0f')
# #         app_globals.window.x_goal = int(app_globals.window.mm_to_steps(self.shared_data.x_center) - int(self.radius_steps))
# #         app_globals.window.progress.setValue(1)
# #         # reset lists every run
# #         self.list_of_x_targets = []
# #         self.list_of_y_targets = []
# # 
# #         if app_globals.window.custom_trajectory:
# #             # copy from UI if present; fall back to empty lists
# #             try:
# #                 self.list_of_x_targets = list(app_globals.window.list_of_x_targets)
# #                 self.list_of_y_targets = list(app_globals.window.list_of_y_targets)
# #             except AttributeError:
# #                 print("Custom trajectory lists not found on window; using empty lists")
# #         else:
# #             # default (single point) path
# #             self.list_of_x_targets.append(app_globals.window.x_goal)
# #             self.list_of_y_targets.append(0)
# #         app_globals.window.sendData('BeaconON')
# #         time.sleep(3)
# #         app_globals.window.sendData('tare')
# #         while (app_globals.window.tare_done == False):
# #             print("andurite nullimine")
# #         app_globals.window.progress.setValue(5)
# #         app_globals.window.sendData('streamStart')
# #         time.sleep(2)
# #         print(app_globals.window.radius_mm)
# #         first_throttle = int(1000 + (app_globals.window.first_throttle.value() * 10))
# #         if app_globals.window.tandem_setup:
# #             second_throttle = int(1000 + (app_globals.window.second_throttle.value() * 10))
# #         else:
# #             second_throttle = 1000
# #         start = f'startMotor|{first_throttle}|{second_throttle}'
# #         app_globals.window.sendData(start)
# #         time.sleep(10)
# #         app_globals.window.progress.setValue(10)
# #         self.x_normalized_mm = 0
# #         self.b = 0
# #         self.current_x_target = 0
# #         self.current_y_target = 0
# #         self.send_once = True
# #         app_globals.window.sendData('m|%d|%d|%d|%d' %(((self.shared_data.x_center - 3) * self.shared_data.ratio), (app_globals.window.Y_pos.value() * self.shared_data.ratio), app_globals.window.measure_speed.value(), (app_globals.window.measure_speed.value()/3)))
# #         time.sleep(2)
# #         app_globals.window.sendData('m|%d|%d|%d|%d' %(((self.shared_data.x_center) * self.shared_data.ratio), (app_globals.window.Y_pos.value() * self.shared_data.ratio), app_globals.window.measure_speed.value(), (app_globals.window.measure_speed.value()/3)))
# #         time.sleep(3)
# #         self.meas_timer.start(self.cycle_time)  # Measure every 100ms
# #         
# #     def start_measuring_after_first(self):
# #         app_globals.window.tare_done = False
# #         app_globals.window.cnv.clear_plots()
# #         self.aoss_a = []
# #         self.aoa_a = []
# #         self.arspd_a = []
# #         self.omega_a = []
# #         app_globals.window.label13.clear()
# #         app_globals.window.label15.clear()
# #         app_globals.window.label17.clear()
# #         app_globals.window.label19.clear()
# #         app_globals.window.label65.clear()
# #         app_globals.window.label67.clear()
# #         app_globals.window.label69.clear()
# #         app_globals.window.measure.setEnabled(False)
# #         app_globals.window.Y_move.setEnabled(False)
# #         app_globals.window.measuring_stopped = False
# #         app_globals.window.test_progress.setMaximum(app_globals.window.sweep_count.value())
# #         self.first_line_done = False
# #         app_globals.window.radius_mm = format(float((app_globals.window.prop.value()*25.4*(1 + self.shared_data.safety_over_prop/100))/2),'.1f')
# #         self.radius_steps = format((float(app_globals.window.radius_mm) * float(self.shared_data.ratio)),'.0f')
# #         app_globals.window.x_goal = int(app_globals.window.mm_to_steps(self.shared_data.x_center) - int(self.radius_steps))
# #         # reset lists every run
# #         self.list_of_x_targets = []
# #         self.list_of_y_targets = []
# # 
# #         if app_globals.window.custom_trajectory:
# #             # copy from UI if present; fall back to empty lists
# #             try:
# #                 self.list_of_x_targets = list(app_globals.window.list_of_x_targets)
# #                 self.list_of_y_targets = list(app_globals.window.list_of_y_targets)
# #             except AttributeError:
# #                 print("Custom trajectory lists not found on window; using empty lists")
# #         else:
# #             # default (single point) path
# #             self.list_of_x_targets.append(app_globals.window.x_goal)
# #             self.list_of_y_targets.append(0)
# #         time.sleep(10)
# #         if app_globals.window.counter == 0:
# #             app_globals.window.progress.setValue(1)
# #             app_globals.window.sendData('tare')
# #             while (app_globals.window.tare_done == False):
# #                 print("ootan")
# #         app_globals.window.progress.setValue(5)
# #         app_globals.window.sendData('streamStart')
# #         time.sleep(2)
# #         first_throttle = int(1000 + (app_globals.window.first_throttle.value() * 10))
# #         if app_globals.window.tandem_setup:
# #             second_throttle = int(1000 + (app_globals.window.second_throttle.value() * 10))
# #         else:
# #             second_throttle = 1000
# #         start = f'startMotor|{first_throttle}|{second_throttle}'
# #         app_globals.window.sendData(start)
# #         time.sleep(10)
# #         app_globals.window.progress.setValue(10)
# #         self.x_normalized_mm = 0
# #         self.b = 0
# #         self.x_prev = 0
# #         self.send_once = True
# #         self.goal_reached = False
# #         self.current_x_target = 0
# #         self.current_y_target = 0
# #         self.data = ""
# #         print(self.data)
# #         self.measure()
# #             
# #     def measure(self):
# #         if not self._running:
# #             self.meas_timer.stop()
# #             #print("lõpetan")
# #             return
# #         if app_globals.window.custom_trajectory == False and self.send_once == True:
# #             self.list_of_x_targets.append(app_globals.window.x_goal)
# #             self.list_of_y_targets.append(0)
# #             app_globals.window.sendData('m|%d|%d|%d|%d' %(self.list_of_x_targets[0], (self.list_of_y_targets[0] + (app_globals.window.Y_pos.value() * self.shared_data.ratio)), app_globals.window.measure_speed.value(), (app_globals.window.measure_speed.value()/3)))
# #             self.send_once = False
# #         if app_globals.window.custom_trajectory == True and self.b == 0 and self.send_once == True:
# #             try:
# #                 app_globals.window.sendData('m|%d|%d|%d|%d' % (list_of_x_targets[self.b], ((app_globals.window.Y_pos.value() * self.shared_data.ratio) + list_of_y_targets[self.b]), app_globals.window.measure_speed.value(), (app_globals.window.measure_speed.value() / 3)))     
# #                 self.current_x_target = list_of_x_targets[self.b]
# #                 self.current_y_target = int(((app_globals.window.Y_pos.value() * self.shared_data.ratio) + self.list_of_y_targets[self.b]))
# #                 self.send_once = False
# #             except:
# #                 print("0 write failed")
# #         if app_globals.window.custom_trajectory == True and 0 < self.b <= (len(self.list_of_x_targets) - 1) and self.send_once == True:
# #             try:
# #                 app_globals.window.sendData('m|%d|%d|%d|%d' % (self.list_of_x_targets[self.b], ((app_globals.window.Y_pos.value() * self.shared_data.ratio) + self.list_of_y_targets[self.b]), app_globals.window.measure_speed.value(), (app_globals.window.measure_speed.value() / 3)))     
# #                 self.current_x_target = self.list_of_x_targets[self.b]
# #                 self.current_y_target = int(((app_globals.window.Y_pos.value() * self.shared_data.ratio) + self.list_of_y_targets[self.b]))
# #                 self.send_once = False
# #             except:
# #                 print("write failed")
# #         #print(self.x_normalized_mm)
# #         if (int(self.x_normalized_mm) == 0 and self.first_line_done == False):
# #             aoss_zero = format(app_globals.window.aoss_abs,'.2f')
# #             aoa_zero = format(app_globals.window.aoa_abs,'.2f')
# #             omega_zero = app_globals.window.omega
# #             arspd_zero = format(app_globals.window.airspeed,'.2f')
# #             v_tan_zero = format(float(self.arspd_avg) * math.sin(math.radians(float(self.aoa_avg))),'.2f')
# #             v_rad_zero = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.sin(math.radians(float(self.aoss_avg))),'.2f')
# #             v_axial_zero = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.cos(math.radians(float(self.aoss_avg))),'.2f')
# #             data_zero = str(str(format(app_globals.window.prop.value(),'.1f'))+" "+str(self.x_normalized_mm)+" "+str(app_globals.window.Y_pos.value())+" "+str(app_globals.window.trq_current)+" "+str(app_globals.window.thr_current)+" "+str(omega_zero)+" "+str(arspd_zero)+" "+str(aoa_zero)+" "+str(aoss_zero)+" "+str(v_tan_zero)+" "+str(v_rad_zero)+" "+str(v_axial_zero))
# #             with open(os.path.join(app_globals.window.path,app_globals.window.csvfile), 'a') as f:
# #                 w = csv.writer(f)
# #                 if not self.header_added:
# #                     w.writerow(self.header)
# #                     self.header_added = True
# #                 w.writerow([data_zero])
# #             self.first_line_done = True
# #             self.x_prev = self.x_normalized_mm
# #         if (int(self.x_normalized_mm) - int(self.x_prev) < int(self.shared_data.x_delta) and self.first_line_done == True):
# #             self.aoss_a.append(float(app_globals.window.aoss_abs))
# #             self.aoa_a.append(float(app_globals.window.aoa_abs))
# #             self.arspd_a.append(float(app_globals.window.airspeed))
# #             self.omega_a.append(float(app_globals.window.omega))
# #         if (int(self.x_normalized_mm) - int(self.x_prev) >= int(self.shared_data.x_delta) and self.first_line_done == True):
# #             self.aoss_avg = format(statistics.mean(self.aoss_a), '.2f')
# #             self.aoa_avg = format(statistics.mean(self.aoa_a),'.2f')
# #             self.arspd_avg = format(statistics.mean(self.arspd_a),'.2f')
# #             if float(self.arspd_avg) > 100.00:
# #                 self.arspd_avg = 0.00
# #             self.omega_avg = format(statistics.mean(self.omega_a),'.2f')
# #             self.v_tan = format(float(self.arspd_avg) * math.sin(math.radians(float(self.aoa_avg))),'.2f')
# #             self.v_rad = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.sin(math.radians(float(self.aoss_avg))),'.2f') 
# #             self.v_axial = format(float(self.arspd_avg) * math.cos(math.radians(float(self.aoa_avg))) * math.cos(math.radians(float(self.aoss_avg))),'.2f') 
# #             self.data = str(str(format(app_globals.window.prop.value(),'.1f'))+" "+str(self.x_normalized_mm)+" "+str(app_globals.window.steps_to_mm(app_globals.window.y_current_steps))+" "+str(app_globals.window.trq_current)+" "+str(app_globals.window.thr_current)+" "+str(self.omega_avg)+" "+str(self.arspd_avg)+" "+str(self.aoa_avg)+" "+str(self.aoss_avg)+" "+str(self.v_tan)+" "+str(self.v_rad)+" "+str(self.v_axial))
# #             #print(self.data)
# #             with open(os.path.join(app_globals.window.path,app_globals.window.csvfile), 'a') as f:
# #                 w = csv.writer(f)
# #                 if not self.header_added:
# #                     w.writerow(self.header)
# #                     self.header_added = True
# #                 w.writerow([self.data])
# #             self.x_prev = self.x_normalized_mm
# #             app_globals.window.update_plot_ax1(self.x_normalized_mm, self.omega_avg, self.arspd_avg, self.aoa_avg, self.aoss_avg, app_globals.window.trq_current, app_globals.window.thr_current)
# #             self.aoss_a.clear()
# #             self.aoa_a.clear()
# #             self.arspd_a.clear()
# #             self.omega_a.clear()
# #         fg = self.shared_data.x_center - (int(app_globals.window.steps_to_mm(app_globals.window.x_goal)))
# #         if app_globals.window.meas_data_running:
# #             self.x_normalized_mm = format(float(self.shared_data.x_center) - (int(app_globals.window.steps_to_mm(app_globals.window.x_current_steps))),'.0f')
# #             #print(self.x_normalized_mm)
# #             #print(type(self.x_normalized_mm))
# #             x_progress = round((int(self.x_normalized_mm)/int(fg))*90,0)
# #             app_globals.window.progress.setValue(10 + int(x_progress))
# #             #print(app_globals.window.x_current_steps)
# #             if (app_globals.window.x_current_steps <= app_globals.window.x_goal and self.goal_reached == False):
# #                 #print(app_globals.window.x_goal)
# #                 self.goal_reached = True
# #                 app_globals.window.counter = app_globals.window.counter + 1
# #                 app_globals.window.test_progress.setValue(app_globals.window.counter)
# #                 app_globals.window.meas_data_running = False
# #                 #print("eesmärk täidetud")
# #                 self.check_progress(app_globals.window.counter, app_globals.window.x_current_steps)
# # #             if self.goal_reached:
# # #                 app_globals.window.counter = app_globals.window.counter + 1
# # #                 app_globals.window.test_progress.setValue(app_globals.window.counter)
# # #                 self.goal_reached = False
# # #                 app_globals.window.meas_data_running = False
# # #                 self.check_progress(app_globals.window.counter, app_globals.window.x_current_steps)
# #             if (app_globals.window.x_current_steps == self.current_x_target and app_globals.window.y_current_steps == self.current_y_target and app_globals.window.custom_trajectory == True and self.goal_reached == False):
# #                 self.b = self.b + 1
# #                 if self.b < len(self.list_of_x_targets):
# #                     self.send_once = True
# #                 if self.b >= len(self.list_of_x_targets):
# #                     self.send_once = False
# #                 #print(self.b)
# #                 pass
# #             
# #         else:
# #             self.x_normalized_mm = 0
# #         
# #     def check_progress(self, cycles_done, x_curr):
# #         #print(cycles_done)
# #         if cycles_done == app_globals.window.sweep_count.value():
# #             app_globals.window.measuring_stopped = True
# #             app_globals.window.come_back()
# #             self.stop_timer()
# #             app_globals.window.process_data()
# #         else:
# #             self.goal_reached = False
# #             time.sleep(1)
# #             app_globals.window.sendData('stop')
# #             time.sleep(2)
# #             app_globals.window.sendData('j|%d|0|%d|%d' %(x_curr, app_globals.window.jog_speed.value(), app_globals.window.jog_speed.value()))
# #             time.sleep(3)
# #             app_globals.window.sendData('center')
# #             time.sleep(5)
# #             app_globals.window.sendData('j|%d|%d|%d|%d' %((self.shared_data.x_center * self.shared_data.ratio), (app_globals.window.Y_pos.value() * self.shared_data.ratio), app_globals.window.jog_speed.value(), app_globals.window.jog_speed.value()))
# #             self.start_measuring_after_first()
# # 
# #     def stop_measuring(self):
# #         self._running = False
# #         self.stopTimer.emit()
# # 
# #     @pyqtSlot()
# #     def stop_timer(self):
# #         self.meas_timer.stop()
# #         self._running = False
# #         print("Measurement stopped")
# #     
# #     @pyqtSlot()
# #     def run(self):
# #         self._running = True
# #         self.start_measuring()