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

# tncr (Corolla Cross), owner 2026-10-20: "tncr needs on average about 0.2 more brake force between 30 and 10 km/h".
# A flat extra ADD (m/s^2) on braking requests in that speed band, shaped so it never makes a step:
# - speed weight: 0 at 35 km/h -> full from 30 down to 10 km/h -> 0 again at 6 km/h, so the 5-0 km/h end (the fixed
#   ceiling / end line in end_force_limit.py, -0.34 at 0 km/h) is untouched and the add is already gone before it;
# - request weight: nothing on a light touch (request lighter than REQ_LO), full once the request is REQ_HI or firmer,
#   linear in between - the first light touch of a brake stays as it was (no bite at brake onset);
# - the add builds at most RATE_IN m/s^3 and comes off at most RATE_OUT m/s^3 (faster, so a hard stop has shed it
#   by ~6 km/h even at -2.5 m/s^2).
# It only shapes what is sent (like the other controlsd hooks): the stopping controller / LongControl keep the plain
# request. Off while the stop-gap roll guard or the gap-close crawl drives the request, and never below accel_min.
# Runs before end_force_limit, which stays the last stage.
ENABLED = True
ADD = 0.1                                    # m/s^2. Owner, same day after 0.2: "a bit too heavy, halve it"
V_BP = [6.0 / 3.6, 10.0 / 3.6, 30.0 / 3.6, 35.0 / 3.6]  # m/s
V_W = [0.0, 1.0, 1.0, 0.0]
REQ_LO = -0.3                                # m/s^2: lighter than this, no add
REQ_HI = -0.6                                # m/s^2: full add from here
RATE_IN = 0.4                                # m/s^3: the add reaches 0.2 in 0.5 s at the fastest
RATE_OUT = 1.0                               # m/s^3: and is gone in 0.2 s at the fastest


class MidSpeedBrakeAdd:
  def __init__(self):
    self.add = 0.0

  def update(self, accel: float, v_ego: float, standstill: bool, long_active: bool, accel_min: float,
             hold_off: bool = False) -> float:
    if not ENABLED or not long_active or standstill:
      self.add = 0.0
      return accel
    target = 0.0
    if not hold_off and accel < REQ_LO:
      w_v = float(np.interp(v_ego, V_BP, V_W))
      w_a = float(np.clip((REQ_LO - accel) / (REQ_LO - REQ_HI), 0.0, 1.0))
      target = ADD * w_v * w_a
    self.add = float(np.clip(target, self.add - RATE_OUT * DT_CTRL, self.add + RATE_IN * DT_CTRL))
    if self.add > 0.0 and accel < 0.0:
      return max(accel - self.add, min(accel, accel_min))
    return accel
