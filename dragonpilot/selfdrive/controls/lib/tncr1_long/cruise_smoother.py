# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.realtime import DT_CTRL

# Steady-cruise smoother (tncr, Corolla Cross; owner 2026-10-16: "at cruise the throttle must have almost no ripple,
# press it to the minimum, even none; not a hysteresis"). With no lead at 60-100 km/h the planner's accel target
# itself ripples by about +-0.02 m/s^2 and changes sign every ~0.5 s (route 0000007d/7e/80: plan std 0.015-0.025,
# 16-24 sign flips per 10 s); the hybrid turns each sign change into a drive <-> regen hand-over the driver feels. The
# pitch-compensation dead band (5ee48a51bc) already removed the carcontroller's own share. This stage low-passes the
# target with TAU while the car is simply holding speed: long control active, no lead, above V_MIN, the target within
# +-A_MAX and not jumping. Anything else - a lead, a target beyond A_MAX, a jump of more than JUMP in one step (set-speed
# change, curve slow-down), stopping, low speed - passes the target straight through and re-seeds the filter, so real
# changes are not delayed; only the small steady corrections arrive 1-2 s later, as the owner accepted.
ENABLED = True
V_MIN = 40.0 / 3.6   # m/s
A_MAX = 0.30         # m/s^2: |target| above this is not steady cruise
JUMP = 0.12          # m/s^2: a step larger than this is a real change -> pass through, re-seed
TAU = 1.5            # s


class CruiseSmoother:
  def __init__(self, dt: float = DT_CTRL):
    self.dt = dt
    self.x: float | None = None
    self.active = False

  def reset(self) -> None:
    self.x = None
    self.active = False

  def update(self, a_target: float, v_ego: float, lead_present: bool, long_active: bool, stopping: bool) -> float:
    steady = (ENABLED and long_active and not lead_present and not stopping and v_ego >= V_MIN and abs(a_target) <= A_MAX
              and (self.x is None or abs(a_target - self.x) <= JUMP))
    if not steady:
      self.x = a_target if long_active else None
      self.active = False
      return a_target
    if self.x is None:
      self.x = a_target
    self.x += self.dt / (TAU + self.dt) * (a_target - self.x)
    self.active = True
    return float(self.x)
