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
import numpy as np

from openpilot.common.realtime import DT_CTRL

# Creep-stall guard. The Toyota hybrid makes automatic-gearbox creep with its motor, so near the end of a stop a brake
# force below ~800-1000 N only cancels it. Route 00000159 (true GPS time 09:05:55, 2.4 km/h, stopped lead 1.6 m ahead):
# the request sat at -0.55 but after a fast release (-1.10 -> -0.56 in 0.6 s) the brake force fell 1160 -> 840-920 N
# (BRAKE 0xA6) and the car did not slow at all for 1.3 s (aEgo ~0), the driver braked at 1.0 m. The same -0.57 gave
# 1160-1200 N and -0.44 at 09:12:02. The PCM brought the force back only at ~200 N/s.
# This does not change the end line; it adds back what the car fails to deliver: rolling between V_LO and V_HI with a
# braking request (<= REQ_MAX) while the car decelerates less than STALL_A for STALL_T, extra brake builds at
# BUILD_RATE (no bite) up to EXTRA_MAX, is kept while the decel is still weak, and fades at FADE_RATE once the car
# decelerates at RECOVER_A again (or the request is light, or it stands). Owner: never a heavy end of the stop - so the
# extra is scaled out from 2 to 1 km/h (END_BP) and the car always stops on the plain end line. Only actuators.accel is
# shaped (same rule as lead_brake_boost).
V_LO = 1.0 / 3.6       # m/s
V_HI = 5.0 / 3.6       # m/s
REQ_MAX = -0.4         # m/s^2
STALL_A = -0.15        # m/s^2: aEgo above this = not slowing
STALL_T = 0.3          # s
RECOVER_A = -0.3       # m/s^2: the extra only fades once the car decelerates at least this much again (or stands)
BUILD_RATE = 1.0       # m/s^3
FADE_RATE = 1.0        # m/s^3
EXTRA_MAX = 0.4        # m/s^2
END_BP = [1.0 / 3.6, 2.0 / 3.6]  # m/s: scaled out from 2 to 1 km/h
END_W = [0.0, 1.0]


class CreepStallGuard:
  def __init__(self):
    self.stall_t = 0.0
    self.extra = 0.0

  def update(self, accel: float, v_ego: float, a_ego: float, standstill: bool, long_active: bool,
             accel_min: float) -> float:
    if not long_active:
      self.stall_t, self.extra = 0.0, 0.0
      return accel

    rolling = not standstill and V_LO <= v_ego <= V_HI and accel <= REQ_MAX
    self.stall_t = self.stall_t + DT_CTRL if (rolling and a_ego > STALL_A) else 0.0
    if self.stall_t >= STALL_T:
      self.extra = min(self.extra + BUILD_RATE * DT_CTRL, EXTRA_MAX)
    elif not rolling or a_ego <= RECOVER_A:
      self.extra = max(self.extra - FADE_RATE * DT_CTRL, 0.0)

    if self.extra > 0.0 and accel < 0.0:
      return max(accel - self.extra * float(np.interp(v_ego, END_BP, END_W)), accel_min)
    return accel
