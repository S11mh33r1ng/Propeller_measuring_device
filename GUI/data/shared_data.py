from config import (
    first_trq_arm_length_default, first_thr_arm_length_default, second_trq_arm_length_default, second_thr_arm_length_default, ratio_default, rho_default,
    kin_visc_default, safety_over_prop_default, x_delta_default,
    max_number_of_samples_default, x_center_default, y_max_default,
    x_max_speed_default, y_max_speed_default, x_max_accel_default, y_max_accel_default,
    aoa_trim_default, aoa_limit_default, aoss_trim_default, aoss_max_limit_default,
    aoss_min_limit_default, min_pwm_default, max_pwm_default, no_of_props_default, probe_offset_default, first_trq_cal_val_default, first_thr_cal_val_default,
    second_trq_cal_val_default, second_thr_cal_val_default, pwm_ramp_ms_default, aoss_enabled_default
)

class SharedData:
    def __init__(self):
        self._first_trq_arm_length = first_trq_arm_length_default
        self._first_thr_arm_length = first_thr_arm_length_default
        self._second_trq_arm_length = second_trq_arm_length_default
        self._second_thr_arm_length = second_thr_arm_length_default
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
        self._aoss_max_limit = aoss_max_limit_default
        self._aoss_min_limit = aoss_min_limit_default
        self._min_pwm = min_pwm_default
        self._max_pwm = max_pwm_default
        self._cal_value = 0
        self._no_of_props = no_of_props_default
        self._probe_offset = probe_offset_default
        self._first_trq_cal_val = first_trq_cal_val_default
        self._first_thr_cal_val = first_thr_cal_val_default
        self._second_trq_cal_val = second_trq_cal_val_default
        self._second_thr_cal_val = second_thr_cal_val_default
        self._pwm_ramp_ms = pwm_ramp_ms_default
        self._aoss_enabled = aoss_enabled_default
        

    # All your @property getters/setters:
    first_trq_arm_length = property(lambda s: s._first_trq_arm_length, lambda s, v: setattr(s, "_first_trq_arm_length", v))
    first_thr_arm_length = property(lambda s: s._first_thr_arm_length, lambda s, v: setattr(s, "_first_thr_arm_length", v))
    second_trq_arm_length = property(lambda s: s._second_trq_arm_length, lambda s, v: setattr(s, "_second_trq_arm_length", v))
    second_thr_arm_length = property(lambda s: s._second_thr_arm_length, lambda s, v: setattr(s, "_second_thr_arm_length", v))
    ratio          = property(lambda s: s._ratio,          lambda s, v: setattr(s, "_ratio", v))
    rho            = property(lambda s: s._rho,            lambda s, v: setattr(s, "_rho", v))
    kin_visc       = property(lambda s: s._kin_visc,       lambda s, v: setattr(s, "_kin_visc", v))
    safety_over_prop = property(lambda s: s._safety_over_prop, lambda s, v: setattr(s, "_safety_over_prop", v))
    x_delta        = property(lambda s: s._x_delta,        lambda s, v: setattr(s, "_x_delta", v))
    max_number_of_samples = property(lambda s: s._max_number_of_samples, lambda s, v: setattr(s, "_max_number_of_samples", v))
    x_center       = property(lambda s: s._x_center,       lambda s, v: setattr(s, "_x_center", v))
    y_max          = property(lambda s: s._y_max,          lambda s, v: setattr(s, "_y_max", v))
    x_max_speed    = property(lambda s: s._x_max_speed,    lambda s, v: setattr(s, "_x_max_speed", v))
    y_max_speed    = property(lambda s: s._y_max_speed,    lambda s, v: setattr(s, "_y_max_speed", v))
    x_max_accel    = property(lambda s: s._x_max_accel,    lambda s, v: setattr(s, "_x_max_accel", v))
    y_max_accel    = property(lambda s: s._y_max_accel,    lambda s, v: setattr(s, "_y_max_accel", v))
    aoa_trim       = property(lambda s: s._aoa_trim,       lambda s, v: setattr(s, "_aoa_trim", v))
    aoa_limit      = property(lambda s: s._aoa_limit,      lambda s, v: setattr(s, "_aoa_limit", v))
    aoss_trim      = property(lambda s: s._aoss_trim,      lambda s, v: setattr(s, "_aoss_trim", v))
    aoss_max_limit = property(lambda s: s._aoss_max_limit, lambda s, v: setattr(s, "_aoss_max_limit", v))
    aoss_min_limit = property(lambda s: s._aoss_min_limit, lambda s, v: setattr(s, "_aoss_min_limit", v))
    min_pwm        = property(lambda s: s._min_pwm,        lambda s, v: setattr(s, "_min_pwm", v))
    max_pwm        = property(lambda s: s._max_pwm,        lambda s, v: setattr(s, "_max_pwm", v))
    cal_value      = property(lambda s: s._cal_value,      lambda s, v: setattr(s, "_cal_value", v))
    no_of_props      = property(lambda s: s._no_of_props,	lambda s, v: setattr(s, "_no_of_props", v))
    probe_offset = property(lambda s: s._probe_offset,		lambda s, v: setattr(s, "_probe_offset", v))
    first_trq_cal_val = property(lambda s: s._first_trq_cal_val,		lambda s, v: setattr(s, "_first_trq_cal_val", v))
    first_thr_cal_val = property(lambda s: s._first_thr_cal_val,		lambda s, v: setattr(s, "_first_thr_cal_val", v))
    second_trq_cal_val = property(lambda s: s._second_trq_cal_val,		lambda s, v: setattr(s, "_second_trq_cal_val", v))
    second_thr_cal_val = property(lambda s: s._second_thr_cal_val,		lambda s, v: setattr(s, "_second_thr_cal_val", v))
    pwm_ramp_ms = property(lambda s: s._pwm_ramp_ms,		lambda s, v: setattr(s, "_pwm_ramp_ms", v))
    aoss_enabled   = property(lambda s: s._aoss_enabled,   lambda s, v: setattr(s, "_aoss_enabled", bool(v)))