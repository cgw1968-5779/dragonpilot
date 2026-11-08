"""
dragonpilot: acceleration profile per Toyota drive mode (carState.driveMode: 0 eco, 1 normal, 2 sport).

Tables copied from sunnypilot tnpb3 accel_controller (Corolla Altis Hybrid). The eco line is the owner's
own GitHub edit of 2026-11-02 (f4cea6f) - keep it.

Only the UPPER (throttle) limit of the planner's accel clip changes; the brake side (ACCEL_MIN), the MPC and
the stopping logic are untouched. A mode switch is smoothed by the planner's existing 0.05 per-step clip
rate limit. With no drive mode (-1: other brands, SecOC Toyotas, signal missing) the stock limit is kept.
"""
import numpy as np

MAX_ACCEL_BP = [0., 3., 12., 24., 36.]  # m/s
MAX_ACCEL_PROFILES = {
  0: [1.75, 1.50, 0.50, 0.30, 0.20],  # eco
  1: [1.95, 1.80, 0.70, 0.50, 0.30],  # normal
  2: [2.00, 2.00, 1.00, 0.70, 0.40],  # sport
}


def drive_mode_max_accel(v_ego: float, drive_mode: int, stock_max_accel: float) -> float:
  profile = MAX_ACCEL_PROFILES.get(int(drive_mode))
  if profile is None:
    return stock_max_accel
  return float(np.interp(max(0.0, v_ego), MAX_ACCEL_BP, profile))
