"""
dragonpilot (dpcr, Corolla Cross): soft acceleration start right after SET/RES above 30 km/h.

Owner 2026-11-05 night (route 000000e4 22:52:51, SET at 63 km/h felt like a push forward): "every time SET-/RES+ is
pressed above 30 km/h, the first 0.1 s of acceleration only 0.05, 0.2 s 0.2, 0.3 s 0.4, then join the planner".
At 22:52:51 the command jumped 0 -> +0.22 on the first engaged frame (the planner's acceleration computed before the set
speed existed) and the car went -0.26 -> +0.50 m/s^2 in 0.4 s.

First added in LongControl (f1a73c17), removed with dp's other LongControl layers when tncr1's longitudinal was copied
(c9280f5f); owner 2026-11-06: put it back. An engagement (not the release of a gas override) above MIN_V that asks for
acceleration is capped by T / A, then climbs on at J until the planner's request is reached. Never more than the
planner; an engagement that starts with a brake (or below MIN_V, or stopping) is left alone.
"""
import numpy as np

from openpilot.common.realtime import DT_CTRL

MIN_V = 30.0                 # km/h
T = [0.0, 0.1, 0.2, 0.3]     # s after engaging
A = [0.0, 0.05, 0.2, 0.4]    # m/s^2 cap
J = 2.0                      # m/s^3 after T[-1] (the 0.2 -> 0.4 slope)


class EngageGasOnset:
  def __init__(self, dt=DT_CTRL):
    self.dt = dt
    self.t: float | None = None  # s since engaging, while the cap is active

  def update(self, output_accel, CS, off: bool, pid: bool):
    """off: LongControl is off this frame; pid: LongControl is in its pid state"""
    if off:
      # releasing a gas override is not an engagement
      self.t = None if getattr(CS, "gasPressed", False) else 0.0
      return output_accel
    if self.t is None:
      return output_accel
    t = self.t
    self.t += self.dt
    if CS.vEgo * 3.6 <= MIN_V or not pid or (t == 0.0 and output_accel <= 0.0):
      self.t = None
      return output_accel
    cap = float(np.interp(t, T, A)) if t <= T[-1] else A[-1] + J * (t - T[-1])
    if output_accel > cap:
      return cap
    self.t = None  # the planner's request is reached (or it asks for less): done
    return output_accel
