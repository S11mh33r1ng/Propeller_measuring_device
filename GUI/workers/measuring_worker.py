
# workers/measuring_worker.py
from __future__ import annotations

import csv
import time
import math
from dataclasses import dataclass
from datetime import datetime
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
    - MainWindow should parse the incoming line and emit:
        measurementsFrame.emit(x_steps, y_steps, vals13)
      where vals13 is a list of 10 or 13 floats in this order:
        [ X, Y, thr1, trq1, rpm1, airspeed, aoa_raw, aoa_abs, aoss_raw, aoss_abs, (thr2, trq2, rpm2)? ]
    - We average N samples per point (samples_per_point) before advancing motion.
    - CSV writing is *decoupled* from waypoints: it logs baseline at 0 mm then every Δx mm, regardless of trajectory.
    """

    # Outbound: connect this to MainWindow's serial write slot
    sendData = pyqtSignal(str)
    
    # NEW: stream fully-processed (averaged/normalized) CSV rows to the UI
    # Shape matches your active CSV header:
    #  - 1 prop: [Prop_in, X, Y, Trq1, Thr1, Omega1, Air, AoA, AoSS, V_tan, V_rad, V_ax]
    #  - 2 prop: [Prop_in, X, Y, Trq1, Thr1, Omega1, Air, AoA, AoSS, V_tan, V_rad, V_ax, Trq2, Thr2, Omega2]
    liveData = pyqtSignal(list)

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

        # steps/mm & X-center (in steps) to compute radial X_mm; prefer ctor arg, fall back to parent.shared_data
        self._steps_per_mm = None
        self._x_center_steps = 0
        try:
            p = self.parent()
            ratio = float(getattr(getattr(p, "shared_data", object()), "ratio", 1.0) or 1.0)
            x_center_mm = float(getattr(getattr(p, "shared_data", object()), "x_center", 0.0) or 0.0)
        except Exception:
            ratio = 1.0
            x_center_mm = 0.0
        self._steps_per_mm = float(steps_per_mm) if steps_per_mm is not None else ratio
        self._x_center_steps = int(round(x_center_mm * self._steps_per_mm))

        # Δx binning is *always enabled* (even with trajectory files)
        try:
            x_delta_mm = float(getattr(getattr(self.parent(), "shared_data", object()), "x_delta", 3.0))
        except Exception:
            x_delta_mm = 3.0
        self._bin_delta_mm = x_delta_mm
        self._bin_delta_steps = max(1, int(round(self._bin_delta_mm * self._steps_per_mm)))
        self._goal_x = self._points[-1].x_steps if self._points else None

        # run state
        self._running = False
        self._t_start_overall = 0.0
        self._t_start_point = 0.0
        self._cur_idx = -1
        self._cur_target: Optional[MeasurePoint] = None
        self._cur_samples: List[List[float]] = []
        self._bin_samples: List[List[float]] = []
        self._x_start_steps: Optional[int] = None
        self._y0_steps: Optional[int] = None
        self._bins_logged = 0
        self._logged_zero = False
        self._last_x = None
        self._last_y = None

        self._waiting_tare = False
        self._pending_motor_start = False
        
        self._last_rpm1: float = 0.0
        self._last_rpm2: float = 0.0
        self._rpm_gate_active = False
        self._rpm_min = 500.0          # adjust to your setup
        self._rpm_stable_delta = 50.0  # max change between frames
        self._rpm_stable_needed = 5    # frames needed within delta
        self._rpm_stable_count = 0
        self._rpm_last_seen = None
        
        try:
            # Allow override from UI SharedData if you want later (optional)
            pre_mm = float(getattr(getattr(self.parent(), "shared_data", object()), "pre_settle_mm", 5.0))
        except Exception:
            pre_mm = 5.0
        self._pre_settle_mm = max(0.0, min(10.0, pre_mm))  # clamp to 0–10 mm
        self._pre_settle_active = False   # true only during the dither
        self._pre_state = 0               # 0:not started, 1:going +X, 2:returning to center
        self._pre_targets = None          # (pt_plus, pt_center) once we know Y
        self._pre_started = False

        # CSV
        self._csv_file = None
        self._csv_writer: Optional[csv.writer] = None

        # CSV headers (mm)
        self.CSV_HEADER_1P = [
            "Prop_diam(inch)", "X_position(mm)", "Y_position(mm)",
            "Torque(Nm)", "Thrust(N)", "Omega(rad/s)",
            "Airspeed(m/s)", "AoA(deg)", "AoSS(deg)",
            "V_tan(m/s)", "V_rad(m/s)", "V_axial(m/s)",
        ]
        self.CSV_HEADER_2P = [
            "Prop_diam(inch)", "X_position(mm)", "Y_position(mm)",
            "Torque1(Nm)", "Thrust1(N)", "Omega1(rad/s)",
            "Airspeed(m/s)", "AoA(deg)", "AoSS(deg)",
            "V_tan(m/s)", "V_rad(m/s)", "V_axial(m/s)",
            "Torque2(Nm)", "Thrust2(N)", "Omega2(rad/s)",
        ]
        self._csv_header = self.CSV_HEADER_2P if self._is_tandem else self.CSV_HEADER_1P

    # ---------- Public API ----------
    
    @pyqtSlot()
    def start(self):
        """Begin the measurement sequence (append to existing CSV if present)."""
        if self._running:
            return
        if not self._points:
            self.error.emit("No points to measure.")
            return

        # per-sweep resets
        self._y0_steps = None
        self._cur_idx = -1
        self._cur_target = None
        self._cur_samples = []
        self._bin_samples.clear()
        self._x_start_steps = None
        self._bins_logged = 0
        self._logged_zero = False
        self._last_x = None
        self._last_y = None

        # --- OPEN CSV (append if exists; write header only once) ---
        log_path = Path(self._csv_path)
        try:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            mode = "a" if log_path.exists() else "w"
            self._csv_file = open(self._csv_path, mode, newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            if mode == "w":
                self._csv_writer.writerow(self._csv_header)
            else:
                # blank separator keeps numeric parsers happy
                self._csv_writer.writerow([])
            self._csv_file.flush()
        except Exception as e:
            self._running = False
            self.error.emit(f"Failed to open log file:\n{e}")
            return

        # Beacon + tare-before-motors sequence
        self._running = True
        self._sweep_started = False
        self._pre_settle_active = self._pre_settle_mm > 0.0
        self._pre_state = 0
        self._pre_targets = None
        self._pre_started = False
        self._t_start_overall = time.monotonic()
        self.sendData.emit("BeaconON")
        QTimer.singleShot(3000, self._send_tare)

    def _send_tare(self):
        if not self._running:
            return
        self._waiting_tare = True
        self._pending_motor_start = True
        self.sendData.emit("tare")

    @pyqtSlot()
    def on_tare_done(self):
        if not self._waiting_tare:
            return
        self._waiting_tare = False
        # Start motors after tare; small delay to let ESCs spin up
        if self._pending_motor_start:
            if self._motor_pwm1 is not None and self._motor_pwm2 is not None:
                self.sendData.emit(f"startMotor|{self._motor_pwm1}|{self._motor_pwm2}")
            else:
                self.sendData.emit("startMotor|1000|1000")
        self._pending_motor_start = False

        # 🟢 Delay a few seconds before sending pre-settle move
        if self._pre_settle_active and not self._pre_started:
            def _delayed_pre_settle():
                try:
                    dither_steps = int(round(self._pre_settle_mm * self._steps_per_mm))
                    y0 = self._points[0].y_steps if self._points else 0
                    plus_pt   = MeasurePoint(self._x_center_steps + dither_steps, y0)
                    minus_pt  = MeasurePoint(self._x_center_steps - dither_steps, y0)
                    center_pt = MeasurePoint(self._x_center_steps, y0)
                    self._pre_targets = (minus_pt, center_pt)
                    print("[MW] Pre-settle wakeup move issued (toward home)")
                    self._send_move(minus_pt)
                    self._pre_state = 1
                    self._pre_started = True
                except Exception as e:
                    print(f"[MW] Pre-settle init failed: {e}")

            # ⏱  delay 3 seconds after motor start
            QTimer.singleShot(3000, _delayed_pre_settle)

        # schedule the sweep kickoff (will wait if pre-settle still active)
        QTimer.singleShot(10_000, self._kickoff_points)



    @pyqtSlot()
    def _kickoff_points(self):
        # Don't send the first waypoint while the pre-settle dither is active.
        if getattr(self, "_pre_settle_active", False):
            QTimer.singleShot(100, self._kickoff_points)
            return
        # Start the sweep only if it hasn't already begun
        self._start_sweep_once()

    def _start_sweep_once(self):
        """Begin the real trajectory exactly once."""
        if getattr(self, "_sweep_started", False):
            return
        self._sweep_started = True
        # Make sure we start at index 0
        self._cur_idx = -1
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
        if not self._running:
            return

        # remember last positions for tail flush
        self._last_x = x_meas
        self._last_y = y_meas

        # Normalize frame length (10 vs 13 fields)
        try:
            vals = list(vals_obj)
        except Exception:
            return
        expected = 13 if self._is_tandem else 10
        if len(vals) < expected:
            vals += [0.0] * (expected - len(vals))
        elif len(vals) > expected:
            vals = vals[:expected]
            
        try:
            self._last_rpm1 = float(vals[4])
            self._last_rpm2 = float(vals[12]) if self._is_tandem and len(vals) >= 13 else 0.0
        except Exception:
            pass
        
        try:
            cur_rpm1 = float(vals[4])
        except Exception:
            cur_rpm1 = 0.0
            
        if self._pre_settle_active:
            # We rely on measured positions to build the dither targets on the first frame
            if not self._pre_started:
                try:
                    x_meas_int = int(x_meas)
                    y_meas_int = int(y_meas)
                except Exception:
                    return  # wait for a valid frame

                dither_steps = int(round(self._pre_settle_mm * self._steps_per_mm))
                plus_pt   = MeasurePoint(self._x_center_steps - dither_steps, y_meas_int)
                center_pt = MeasurePoint(self._x_center_steps,                 y_meas_int)
                self._pre_targets = (plus_pt, center_pt)
                self._pre_started = True
                self._pre_state = 1
                # Go +X using the same m|X|Y|feed_xy|feed_y you already use
                self._send_move(plus_pt)
                return  # do not collect/log during pre-settle

            # We’ve started; track arrival and sequence the two legs
            plus_pt, center_pt = self._pre_targets
            tol = self._arrival_tol

            if self._pre_state == 1:
                dx = abs(int(x_meas) - plus_pt.x_steps)
                dy = abs(int(y_meas) - plus_pt.y_steps)
                if dx <= tol and dy <= tol:
                    self._pre_state = 2
                    self._send_move(center_pt)
                return  # still settling; don't log

            if self._pre_state == 2:
                dx = abs(int(x_meas) - center_pt.x_steps)
                dy = abs(int(y_meas) - center_pt.y_steps)
                if dx <= tol and dy <= tol:
                    # Done settling → reset binning state and begin normal run
                    self._pre_settle_active = False
                    self._x_start_steps = None
                    self._bins_logged = 0
                    self._logged_zero = False
                    self._bin_samples.clear()
                    # Start the sweep exactly once
                    self._start_sweep_once()
                return  # don't fall through during this frame

        # Update last RPMs as before
        self._last_rpm1 = cur_rpm1
        self._last_rpm2 = float(vals[12]) if self._is_tandem and len(vals) >= 13 else 0.0

        # --- RPM stability gate ---
        if self._rpm_gate_active:
            if cur_rpm1 >= self._rpm_min:
                if self._rpm_last_seen is None:
                    self._rpm_last_seen = cur_rpm1
                    self._rpm_stable_count = 1
                else:
                    if abs(cur_rpm1 - self._rpm_last_seen) <= self._rpm_stable_delta:
                        self._rpm_stable_count += 1
                    else:
                        self._rpm_stable_count = 1  # reset window if jumpy
                    self._rpm_last_seen = cur_rpm1

                if self._rpm_stable_count >= self._rpm_stable_needed:
                    # RPM is stable → allow logging from next frame onward
                    self._rpm_gate_active = False
                    # drop any bin contents collected pre-stability (defensive)
                    self._bin_samples.clear()
                    # also defer the "0-mm baseline" until stability is achieved
                    self._logged_zero = False
            # While gate is active: do not collect bin samples or write baseline
            return

        # -------- Δx binning: always active --------
        if self._pre_settle_active:
            return 
        if self._x_start_steps is None:
            self._x_start_steps = int(self._x_center_steps)
            self._y0_steps = y_meas
            self._bins_logged = 0
            self._bin_samples.clear()
            self._logged_zero = False

        # collect current frame into the bin
        self._bin_samples.append(vals)

        # emit 0‑mm baseline once so X_mm starts at 0 in the CSV
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

        # -------- Waypoint advance only (no CSV writes here) --------
        if self._cur_target is not None:
            dx = abs(x_meas - self._cur_target.x_steps)
            dy = abs(y_meas - self._cur_target.y_steps)
            if dx <= self._arrival_tol and dy <= self._arrival_tol:
                self._cur_samples.append(vals)
                if (time.monotonic() - self._t_start_point) > self._settle_timeout_s or len(self._cur_samples) >= self._samples_per_point:
                    self.pointDone.emit(self._cur_idx)
                    self._advance_to_next_point()

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
            xc   = self._x_center_steps
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

            # first prop
            thr1, trq1 = r(vals13[2], 2), r(vals13[3], 2)
            airspeed, aoa_r, aoa_a = r(vals13[5], 2), r(vals13[6], 2), r(vals13[7], 2)
            aoss_r, aoss_a    = r(vals13[8], 2), r(vals13[9], 2)

            # --- pull rotation_dir (+1 CW / -1 CCW) and optional mount_sign from UI shared data ---
            mw = self.parent()
            d = 1
            m = 1
            try:
                d = int(getattr(getattr(mw, "shared_data", object()), "rotation_dir", 1) or 1)
                m = int(getattr(getattr(mw, "shared_data", object()), "mount_sign", 1) or 1)
            except Exception:
                pass

            # --- build the angles used for PHYSICS (signed) ---
            # same servo magnitude in opposite direction when rotation flips:
            aoa_servo_abs   = d * abs(float(aoa_a))
            aoa_abs  = m * (float(aoa_r) + aoa_servo_abs)
            
            # Omegas from RAW sensor RPMs (no averaging)
            omega1 = round((float(self._last_rpm1) * 6.283185307179586) / 60.0, 2)  # 2π/60
            omega2 = 0.0

            if self._is_tandem:
                thr2, trq2 = r(vals13[10], 2), r(vals13[11], 2)
                omega2 = round((float(self._last_rpm2) * 6.283185307179586) / 60.0, 2)
            else:
                thr2 = trq2 = 0.0
                
            # pull prop diameter (inch) from UI
            prop_in = 0.0
            try:
                mw = self.parent()
                # MainWindow has self.prop (QDoubleSpinBox) for diameter in inches
                prop_in = float(getattr(mw, "prop").value())
            except Exception:
                pass

            # --- flow components from *averaged* samples ---
            # Use AoA/AoSS (degrees) and Airspeed (m/s) that arrived in vals13
            # These are bin-averaged when called from the bin flush path.
            aoa_rad  = math.radians(float(aoa_abs))
            aoss_rad = math.radians(float(aoss_a))
            air_f    = float(airspeed)
            
            v_tan = round(air_f * math.sin(aoa_rad), 2)
            v_rad = round(air_f * math.cos(aoa_rad) * math.sin(aoss_rad), 2)
            v_ax  = round(air_f * math.cos(aoa_rad) * math.cos(aoss_rad), 2)

            if self._is_tandem:
                row = [
                    prop_in, X, Y,
                    trq1, thr1, omega1,
                    round(air_f, 2), round(aoa_abs, 2), round(aoss_a, 2),
                    v_tan, v_rad, v_ax,
                    r(trq2, 3), r(thr2, 3), omega2
                ]
            else:
                row = [
                    prop_in, X, Y,
                    trq1, thr1, omega1,
                    round(air_f, 2), round(aoa_abs, 2), round(aoss_a, 2),
                    v_tan, v_rad, v_ax
                ]

            self._csv_writer.writerow(row)
            self._csv_file.flush()
            # NEW: publish live row to UI listeners
            try:
                self.liveData.emit(list(row))
            except Exception:
                pass
        except Exception as e:
            self.error.emit(f"Failed to write CSV row:\n{e}")

    def _finish(self, reason_ok: Optional[str] = None):
        """Flush tail, close CSV and announce completion or an error message."""
        # tail flush of Δx bin
        try:
            if self._csv_writer and self._bin_samples:
                tail = self._average_samples(self._bin_samples)
                if tail:
                    if self._last_x is not None and self._last_y is not None:
                        tail[0] = float(self._last_x)
                        tail[1] = float(self._last_y)
                    self._write_row(tail)
        except Exception:
            pass

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
        self._bin_samples.clear()

        if was_running:
            if reason_ok is None:
                self.finished.emit(self._csv_path)
            else:
                self.error.emit(reason_ok)
                self.finished.emit(self._csv_path)
