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

# Engaging close behind a stopped lead at low speed. Route 0000015a (true GPS time 18:51:24): SET- at 7.2 km/h with the
# stopped lead ~4.2 m ahead. The plan asked -1.23 within 0.6 s (stopping ~2 m out needs only ~-0.9); the engage soft
# start (0.5 m/s^3 for 0.5 s, for the 08:54 SET- far from the lead) held the command light for 0.6 s while the car
# rolled ~1.2 m at -0.2, then the command built -0.39 -> -1.31 (the long PID integral and the low-speed hand-over
# feed-forward added 0.22 beyond the request), BRAKE_FORCE 2480 N, the car hit -1.9 m/s^2 at 3.5-2 km/h and stopped
# 1.5 m out. Owner: "the last moment too strong".
# So when, in the engage window, below V_MAX behind a (nearly) stopped lead the decel needed to stop STOP_GAP short of
# it is at least B_CLOSE (= the lead is close), this engagement is a close stop:
#   - the brake builds linearly from the first frame (J_BP / J_DOWN: 1.0 m/s^3, no light plateau),
#   - the request is capped at what the stop needs (b_need * NEED_MARGIN + NEED_ADD, recomputed every frame from the
#     radar distance, so it firms up by itself if the car is not slowing enough) - lighter than an over-eager plan,
#   - the long PID integral is frozen and the low-speed hand-over feed-forward is skipped (the car over-delivered here).
# A far SET- (08:54: 9.6 km/h, lead 10 m -> b_need 0.22) keeps the -0.25 soft start. It lasts until the car stands,
# the lead moves, the request turns positive or ACC disengages.
V_MAX = 15.0 / 3.6     # m/s
LEAD_V_MAX = 1.0       # m/s
STOP_GAP = 2.0         # m (radar dRel): where the close stop aims
B_CLOSE = 0.5          # m/s^2
D_MIN = 0.3            # m
NEED_MARGIN = 1.15
NEED_ADD = 0.1         # m/s^2
J_BP = [0.0, 1.0, 1.5]       # s since engaging
J_DOWN = [1.0, 1.0, 1.5]     # m/s^3
# tncr (Corolla Cross): the cap may not go lighter than the Cross's own end of a stop. Route 0000006b (2026-10-15, true
# time 09:06:05, SET- at 8.6 km/h, lead 8 m): b_need aims 2.0 m short, so near the end the cap fell to -0.23 at 2 km/h and
# -0.12 at 0.7 km/h (the request was -0.55..-0.85); the Cross's creep torque held the car at 0.7-0.9 km/h for ~1 s and
# it then stopped with a nod (-0.5 m/s^2, owner: "the end of the stop still jolts a little"). So the cap is at least
# CAP_FLOOR: -0.50 down to 2 km/h (the lightest request that still slows the Cross there), then a line to the owner's
# -0.35 at 0 km/h - the same end the plain stop has.
CAP_FLOOR_V = [0.0, 2.0 / 3.6]   # m/s
CAP_FLOOR_A = [-0.35, -0.50]     # m/s^2


def b_need(v_ego: float, d_rel: float) -> float:
  return v_ego ** 2 / (2.0 * max(d_rel - STOP_GAP, D_MIN))


class EngageCloseStop:
  def __init__(self):
    self.active = False

  def reset(self) -> None:
    self.active = False

  def update(self, lead, v_ego: float, standstill: bool, accel_request: float, t_engaged: float | None) -> None:
    lead_ok = bool(lead.status) and lead.vLead < LEAD_V_MAX
    if self.active:
      if standstill or not lead_ok or accel_request >= 0.0:
        self.active = False
    elif t_engaged is not None and lead_ok and v_ego <= V_MAX and not standstill and accel_request < 0.0:
      self.active = b_need(v_ego, lead.dRel) >= B_CLOSE

  def cap(self, accel_request: float, lead, v_ego: float) -> float:
    if not self.active:
      return accel_request
    cap = -(b_need(v_ego, lead.dRel) * NEED_MARGIN + NEED_ADD)
    return max(accel_request, min(cap, float(np.interp(v_ego, CAP_FLOOR_V, CAP_FLOOR_A))))

  def j_down(self, t_engaged: float) -> float:
    return float(np.interp(t_engaged, J_BP, J_DOWN))
