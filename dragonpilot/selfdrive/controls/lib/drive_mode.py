"""
dragonpilot: acceleration profile per Toyota drive mode (carState.driveMode: 0 eco, 1 normal, 2 sport).

Tables = the owner's own dppb tuning (GitHub web edits 2026-11-08 fc084b1f), copied to dpcr at the owner's request.
Originally from sunnypilot tncr1 accel_controller (Corolla Cross Hybrid).

Only the UPPER (throttle) limit of the planner's accel clip changes; the brake side (ACCEL_MIN), the MPC and
the stopping logic are untouched. A mode switch is smoothed by the planner's existing 0.05 per-step clip
rate limit. With no drive mode (-1: other brands, SecOC Toyotas, signal missing) the stock limit is kept.
"""
import numpy as np

MAX_ACCEL_BP = [0., 3., 12., 24., 36.]  # m/s
MAX_ACCEL_PROFILES = {
  0: [1.75, 1.50, 0.55, 0.28, 0.18],  # eco
  1: [1.95, 1.80, 0.75, 0.45, 0.25],  # normal
  2: [2.00, 2.00, 1.00, 0.70, 0.34],  # sport
}


def drive_mode_max_accel(v_ego: float, drive_mode: int, stock_max_accel: float) -> float:
  profile = MAX_ACCEL_PROFILES.get(int(drive_mode))
  if profile is None:
    return stock_max_accel
  return float(np.interp(max(0.0, v_ego), MAX_ACCEL_BP, profile))
