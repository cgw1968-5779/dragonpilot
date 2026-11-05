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
from dragonpilot.selfdrive.controls.lib.tncr1_long.stop_gap_close import decel_request, decel_from_request

# Stop-gap hold. Owner's braking principle (2026-10-14): light touch, linear build, hold once the decel is enough, release
# linearly only once the stop is sure to end beyond ~4 m, and if releasing would bring us too close, HOLD the current
# force at once. Route 00000159 (true GPS time 09:05:47-57, stopped lead first seen at 47 m from 42 km/h): the plan eased
# from its -2.3 peak while the stop was already short - at 12 km/h with the lead 7.3 m ahead the request was -1.49 and the
# car gave -1.21, i.e. a stop ~2.8 m out (d - v^2/2b); it kept easing (-1.14 at 5.7 km/h / 3.2 m) and the driver had to
# brake at 1.0 m. Owner: below 30 km/h, while the predicted stop gap is under 3.5 m the request must not get lighter;
# but the 2-0 km/h end must still ease off as before (no heavy end of stop again).
# So: behind a (nearly) stopped lead, below V_MAX, the predicted gap uses the weaker of the request and
# the delivered decel, and while it is under GAP_MIN the request may get firmer but not lighter. Once the gap is safe
# again the held value releases toward the request at RELEASE_RATE (linear, no step). From 7 down to 2 km/h (END_BP)
# it blends back into the plain request, so the 2-0 km/h end line is untouched. It waits until the braking has lasted
# BRAKING_T: replayed on 159, the 08:54:34 SET- engage (aEgo still ~0 while the request was -0.9) would otherwise have
# held the very first touch firmer, against principle step 1. The 7 -> 2 km/h blend keeps the release into the end line
# near 0.5 m/s^3 (a 5 -> 2 km/h blend released at up to 1.4 m/s^3 on 159 09:05:54). Only actuators.accel is shaped;
# the stopping controller's prev_accel keeps the plain request (same rule as lead_brake_boost).
GAP_MIN = 3.5            # m (radar dRel)
V_MAX = 30.0 / 3.6       # m/s
LEAD_V_MAX = 1.0         # m/s: lead treated as stopped (its own remaining travel is ignored -> conservative)
B_MIN = 0.3              # m/s^2: floor of the decel used in the prediction
RELEASE_RATE = 1.0       # m/s^3
BRAKING_REQ = -0.3       # m/s^2: braking ...
BRAKING_T = 1.0          # s: ... for this long before it may hold (never at the light first touch / right after engaging)
END_BP = [2.0 / 3.6, 7.0 / 3.6]  # m/s: blended back into the request from 7 to 2 km/h
END_W = [0.0, 1.0]
# tncr, owner 2026-10-15 (supreme principle): when the lead brakes hard, allow the stop down to 2 m so the brake is not
# too fierce: after an urgent lead brake the hold only acts below GAP_MIN_URGENT, until this stop ends. Urgent = the
# same event the MPC's emergency trim reacts to (long_mpc_ext EMERG_*: the lead brakes at <= URGENT_A while our request
# is firmer than URGENT_REQ), or the lead_brake_boost test (lead <= -2.0, closing >= URGENT_CLOSE, TTC <= URGENT_TTC).
# Holding to 3.5 m there would undo the MPC's 2.5 m trim and make the end of the stop firmer.
GAP_MIN_URGENT = 2.0     # m
URGENT_A = -1.5          # m/s^2
URGENT_REQ = -2.0        # m/s^2
URGENT_TTC = 3.5         # s
URGENT_CLOSE = 10.0 / 3.6  # m/s

# tncr roll guard - never roll into a stopped lead. Route 0000006b (2026-10-15, true time 09:21:01-09:21:06): the Cross
# stood 3.57 m behind a lead that never moved; radar noise (+0.3 m/s) made the plan go, the car launched, and the plan's
# -0.33..-0.39 (creep-follow -0.24) could not beat the creep torque: 3.3 km/h, 3.5 -> 1.4 m until the owner braked.
# Owner: keep the launch as quick as it is (no waiting for the lead), but the car must never move on into a lead that
# has not moved; once stopped, brake harder. So, below ROLL_V_MAX behind a lead that is not moving (vLead < ROLL_LEAD_V):
# if, one ROLL_LOOK_T ahead at the present speed and stopping at a comfortable ROLL_B, the car would end closer than
# ROLL_TRIGGER, the request is made at least what stops it ROLL_GAP short of the lead (Cross request mapping, built at
# ROLL_RATE). After such a catch the car is held at a standstill (ROLL_HOLD, reached at ROLL_HOLD_RATE) until the lead
# really moves (vLead >= ROLL_LEAD_V or the distance grows ROLL_GO_D) - only after a catch, so a normal launch is as
# quick as before.
ROLL_V_MAX = 10.0 / 3.6  # m/s
ROLL_LEAD_V = 0.5        # m/s
ROLL_LOOK_T = 1.0        # s
ROLL_B = 0.5             # m/s^2
ROLL_TRIGGER = 2.5       # m
ROLL_GAP = 2.5           # m
ROLL_SLACK = 0.3         # m: acts only if the present request would end closer than ROLL_GAP - ROLL_SLACK
ROLL_B_CAUGHT = 0.3      # m/s^2: once caught, at least this decel (Cross mapping, > the creep torque) until it stands
ROLL_D_MIN = 0.2         # m
ROLL_RATE = 1.5          # m/s^3
ROLL_HOLD = -1.2         # m/s^2
ROLL_HOLD_RATE = 1.0     # m/s^3
ROLL_GO_D = 0.4          # m
ROLL_RELEASE_RATE = 1.5  # m/s^3
# Owner 2026-10-15 evening (route 00000073 17:16:08, lead braked hard, bookmark 1): the guard fired at 6.6 km/h with
# the stopped lead 3.6 m ahead, demanded a 2.5 m stop and drove the request from -1.5 to -2.69 inside 0.5 s (4120 N
# at 2.7 km/h; the car stood with -1.67 still on it - the jolt). The urgent hold and the MPC trim allow 2.0 m in such
# a stop, so the guard conflicts with them at 2.5 m. While the stop is urgent the guard uses the urgent gap and
# trigger; and (controlsd) its output is no longer exempt from the end-force ceiling while the car moves.
ROLL_GAP_URGENT = 2.0    # m
ROLL_TRIGGER_URGENT = 2.0  # m


class StopGapHold:
  def __init__(self):
    self.held = None  # the held (never lighter) request while active, None when passing through
    self.holding = False
    self.braking_t = 0.0
    self.urgent = False
    self.roll_out = None    # roll guard output while it acts (None = not acting)
    self.roll_caught = False
    self.roll_d0 = 0.0

  def _roll_guard(self, accel: float, v_ego: float, standstill: bool, lead) -> float:
    present = lead is not None and bool(lead.present)
    lead_still = present and lead.vLead < ROLL_LEAD_V
    roll_gap, roll_trigger = (ROLL_GAP_URGENT, ROLL_TRIGGER_URGENT) if self.urgent else (ROLL_GAP, ROLL_TRIGGER)
    if standstill:
      if self.roll_caught and lead_still and lead.dRel < self.roll_d0 + ROLL_GO_D:
        prev = accel if self.roll_out is None else self.roll_out
        self.roll_out = min(accel, max(ROLL_HOLD, prev - ROLL_HOLD_RATE * DT_CTRL))
        return self.roll_out
      self.roll_caught = False
    elif lead_still and v_ego <= ROLL_V_MAX and (self.roll_caught or
                                                lead.dRel - v_ego * ROLL_LOOK_T - v_ego ** 2 / (2.0 * ROLL_B) < roll_trigger):
      b = v_ego ** 2 / (2.0 * max(lead.dRel - roll_gap, ROLL_D_MIN))
      if self.roll_caught:
        b = max(b, ROLL_B_CAUGHT)
      need = decel_request(b, v_ego)
      b_req = decel_from_request(accel, v_ego)
      ends_short = b_req <= 0.0 or lead.dRel - v_ego ** 2 / (2.0 * b_req) < roll_gap - ROLL_SLACK
      if need < accel and (self.roll_caught or ends_short):
        prev = min(accel, 0.0) if self.roll_out is None else self.roll_out
        self.roll_out = max(need, prev - ROLL_RATE * DT_CTRL)
        self.roll_caught, self.roll_d0 = True, lead.dRel
        return self.roll_out
    self.roll_caught = False  # handed back while rolling: the next catch starts fresh
    if self.roll_out is not None:  # hand back: linear release to the request
      self.roll_out = min(accel, self.roll_out + ROLL_RELEASE_RATE * DT_CTRL) if accel > self.roll_out else None
      if self.roll_out is not None:
        return self.roll_out
    return accel

  def update(self, accel: float, v_ego: float, a_ego: float, standstill: bool, lead, long_active: bool) -> float:
    if not long_active:
      self.held, self.holding, self.braking_t, self.urgent = None, False, 0.0, False
      self.roll_out, self.roll_caught = None, False
      return accel
    accel = self._roll_guard(accel, v_ego, standstill, lead)
    if standstill:
      self.held, self.holding, self.braking_t, self.urgent = None, False, 0.0, False
      return accel
    if bool(lead.present):
      closing = v_ego - lead.vLead
      if lead.aLeadK <= URGENT_A and accel <= URGENT_REQ:
        self.urgent = True
      elif lead.aLeadK <= -2.0 and closing >= URGENT_CLOSE and lead.dRel / max(closing, 0.3) <= URGENT_TTC:
        self.urgent = True

    self.braking_t = self.braking_t + DT_CTRL if accel < BRAKING_REQ else 0.0
    self.holding = False
    if bool(lead.present) and lead.vLead < LEAD_V_MAX and v_ego <= V_MAX and accel < 0.0 and self.braking_t >= BRAKING_T:
      b = max(min(-accel, -a_ego), B_MIN)
      self.holding = lead.dRel - v_ego ** 2 / (2.0 * b) < (GAP_MIN_URGENT if self.urgent else GAP_MIN)

    if self.held is None:
      if not self.holding:
        return accel
      self.held = accel
    if self.holding:
      self.held = min(accel, self.held)                         # firmer allowed, lighter not
    else:
      self.held = min(accel, self.held + RELEASE_RATE * DT_CTRL)  # linear release back to the request
      if self.held >= accel:
        self.held = None
        return accel

    w = float(np.interp(v_ego, END_BP, END_W))
    return accel + (self.held - accel) * w
