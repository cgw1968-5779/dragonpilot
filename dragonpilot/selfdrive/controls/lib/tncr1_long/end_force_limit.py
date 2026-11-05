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
import math

import numpy as np

from openpilot.common.realtime import DT_CTRL

# End-force limit - the owner's end-of-stop rule (2026-10-14 evening, "remember it forever"): the request at the 0 km/h
# stop is always -0.30 (or -0.28), never firmer; a safe, effective stop does all its force modulation ABOVE 2 km/h;
# "if the MPC asks for more than this, always override it". Evidence: route 0000015a 18:51:26 (engaged at 7 km/h behind
# a stopped lead 4 m ahead) stopped at -0.46 with the car at -1.5..-1.9 m/s^2 - "the last moment too strong"; route
# 0000015c 19:35:40 stopped at -0.34.
# So this is the LAST stage of the request (after the stop-gap hold/close, creep-stall guard and boost): below V_LINE the
# request may not be firmer than a line in speed from what it was at V_LINE (a_line) down to END_ACCEL at 0 km/h.
# Lighter is always passed. No exceptions. Once the wheels stop, the request continues from where the line left it and
# may only firm up at HOLD_RATE into whatever holds the car (the standstill hold), so the stop itself has no step.
# Safety valve: if below V_LINE the car is not slowing (aEgo above STALL_A for STALL_T while still rolling above
# STALL_V - the hybrid's creep torque can beat a light request), the line is let go and re-anchored at the current speed
# and request once it slows again: a stall is not "the stop", and it still ends on -0.30 at 0 km/h.
V_LINE = 2.0 / 3.6     # m/s
END_ACCEL = -0.34      # m/s^2 at 0 km/h. tncr (Corolla Cross): -0.35 -> -0.34 (owner 2026-10-15 evening, "0.01 lighter");
                       # -0.35 was the Cross stop template (stopped at
                       # 800-840 N, no nod); the Altis rule above is -0.30 and stays so on tnpb3
HOLD_RATE = 0.5        # m/s^3: after the stop, toward the standstill hold
# tncr (Corolla Cross), owner 2026-10-15 evening after route 00000073 17:16:08 (a stop that ended with 4120 N at
# 2.7 km/h and -1.67 m/s^2 still on the car as the wheels stopped - the jolt): 'the car must, without exception, stop
# on a force that cannot jolt; the end must ease smoothly down to -0.34 at 0 km/h; 5 km/h may still need a firm brake'.
# So besides the anchored line below V_LINE there is a FIXED ceiling line from CEIL_V down to END_ACCEL at 0 km/h that
# nothing may exceed while the car moves (the roll guard included - controlsd no longer bypasses this stage): -2.0 at
# 5 km/h, -1.34 at 3, -1.0 at 2, -0.67 at 1, -0.34 at 0. An ordinary stop (END curve -0.65 at 5 km/h) never touches
# it; only the hold / guard / boost extras are capped. From 5 km/h at -2.0 (~2800 N) the pressure has to come off at
# ~2000 N/s to follow the line; the Cross released 4120 -> 1400 N in 0.4 s on 17:16:07 (~6800 N/s), so it can.
# Owner, same evening, on 17:16:07.8-08.0 (sent -0.71 at 1.0 km/h, the car stood with -1.67): "the cap for the last
# 1-0 km/h must be below -0.6, best stay at -0.34; at 07.8 the sent -0.71 should already have been back below -0.4; the
# gap was still 2.2 m, so a lighter force was allowed". So the ceiling is a table, not a straight line: -2.0 at 5 km/h,
# -1.2 at 3, -0.60 at 2, -0.38 at 1, -0.34 at 0 (1.4 km/h -0.47, 0.5 km/h -0.36). Note the -1.67 on 17:16 was not a
# request: the request was already -0.75 -> -0.37 over the last 1.4 km/h; it was the pressure left from the 4120 N
# spike at 2.7 km/h (the guard, now capped at 3 km/h to -1.2 ~ 1700 N) that the PCM could not release in 0.3 s.
# tncr 2026-10-22, owner after ed30444a37 (sim of a stop: -0.39 at 3, -0.45 at 2, -0.38 at 1 km/h): "3 km/h -0.37, 2 km/h
# -0.37, 1 km/h -0.36, 0 km/h -0.34 - put it in, I want to see the effect". The ceiling below 3 km/h is now that line
# (was -1.2 / -0.60 / -0.38 / -0.34); 3 -> 5 km/h still opens to -2.0. A ceiling only: a lighter request stays lighter
# (item 0 - nothing raises the force before the stop).
CEIL_BP = [0.0, 1.0 / 3.6, 2.0 / 3.6, 3.0 / 3.6, 5.0 / 3.6]  # m/s
CEIL_A = [-0.34, -0.36, -0.37, -0.37, -2.0]                 # m/s^2 (CEIL_A[0] must equal END_ACCEL)
CEIL_V = CEIL_BP[-1]
CEIL_TOP = CEIL_A[-1]
STALL_V = 0.5 / 3.6    # m/s
STALL_A = -0.1         # m/s^2
STALL_T = 0.3          # s



# Downhill end braking (tncr, Corolla Cross, owner 2026-10-25, approved as an exception to item 0: "below 30 km/h, stopping on a
# downhill, add a little force step by step with the slope angle; start with 0.01 per degree"). The request AND every
# ceiling above (the end value -0.34 at 0 km/h included) move firmer by the same small amount, so the shape of the stop is
# unchanged and only its level follows the grade; flat roads and uphills are untouched. Pitch: calibrated pose (rad,
# + = nose up); standing on the flat both cars read +0.2..+1.6 deg (Altis routes 173-175 standstill p50 0.2-1.1, Cross
# route a3 stops p50 1.1; auto_brake_hold.FLAT_PITCH_DEG 1.0 from Altis routes ec/ee/f2), so downhill degrees count from
# +1.0 with a 0.5 deg dead band for the road crown and the nose dip under braking. 1 deg of grade is ~0.17 m/s^2 of
# gravity; 0.01 per degree is a first small step.
SLOPE_GAIN = 0.01             # m/s^2 per degree of downhill
SLOPE_FLAT_PITCH_DEG = 1.0    # deg
SLOPE_DEADBAND_DEG = 0.5      # deg
SLOPE_MAX = 0.10              # m/s^2 (reached at ~10.5 deg downhill)
SLOPE_V_BP = [25.0 / 3.6, 30.0 / 3.6]  # m/s: full below 25 km/h, none from 30 (no step at 30)
SLOPE_V_W = [1.0, 0.0]
SLOPE_A_BP = [-0.10, 0.0]     # m/s^2: only on a braking request (fades in from 0 to -0.1, cruise near 0 untouched)
SLOPE_A_W = [1.0, 0.0]


# Dynamic end (tncr, owner 2026-10-28, route 000000bb 09:17:46): the fixed ceiling above cut the request -2.26 -> -0.38
# in 0.4 s between 5.4 and 2.4 km/h while the stopped lead was only 2.5 m ahead (the plan still asked -1.3 .. -1.06); the
# brake let go, the hybrid's creep beat -0.37 (aEgo +0.17) and rolled the car from 2.1 to 1.3 m until the owner braked.
# Owner: the mid phase must blend smoothly into the end, never let go suddenly; only DYNAMIC handling, and ordinary soft
# stops must not change. So the ceiling (and the anchored line) may hold firmer than the table only by what the CURRENT
# state needs: (1) END_NEED - the constant decel that still stops END_GAP_MIN short of a stopped lead from the present
# speed and gap (it shrinks to nothing as the car slows with room to spare, so the end still tapers to END_ACCEL; it
# grows if the car creeps toward the lead); (2) whatever the ceiling / line cuts comes off as a ramp, never a step: at
# END_RELEASE_RATE, or faster when needed so that it is back on the end value just as the wheels stop (the remaining
# excess over the time to stop at the present decel) - the owner's own template (route bb 09:19:40-44, 3 bookmarks)
# releases the same way: -2.2 at 12 km/h, -1.5 at 9, -1.1 at 6, -1.0 at 3, -0.8 at 1.5, ~850 N at the stop, a steady
# 0.4-1.5 m/s^3 with no dip. Firming for (1) is at most END_FIRM_RATE. An ordinary stop arrives at ~-0.6..-0.7 and the
# table takes it to -0.37 over ~0.5 s anyway (09:21:53: 0.74 m/s^3): nothing changes there (C4 sim regression over 23
# stops: max difference 0.00-0.02 on the ordinary ones).
END_GAP_MIN = 2.0          # m: the closest a stop may end (owner: 2 m allowed when it is urgent)
END_LEAD_V_STOPPED = 1.0   # m/s: the lead counts as stopped below this
END_NEED_D_MIN = 0.3       # m: the gap left for the need never taken below this (no division blow-up)
END_NEED_MAX = 1.5         # m/s^2
END_RELEASE_RATE = 2.0     # m/s^3: how fast the held brake may come off
END_FIRM_RATE = 1.0        # m/s^3: how fast it may build (a creep toward the lead)
END_DECEL_MIN = 0.3        # m/s^2: the decel assumed for the time to stop when the car is barely slowing


def end_need(v_ego: float, lead_d: float | None, lead_v: float | None) -> float:
  """the brake (m/s^2, >= 0) the present state needs to stop END_GAP_MIN short of a stopped lead; 0 without one"""
  if lead_d is None or lead_v is None or not math.isfinite(lead_d) or lead_v > END_LEAD_V_STOPPED or v_ego <= 0.0:
    return 0.0
  return min(v_ego ** 2 / (2.0 * max(lead_d - END_GAP_MIN, END_NEED_D_MIN)), END_NEED_MAX)


def downhill_extra(pitch: float | None, v_ego: float, accel: float) -> float:
  """extra braking (m/s^2, >= 0) for the grade; 0 on the flat, uphill, above 30 km/h or without a braking request"""
  if pitch is None or not math.isfinite(pitch) or accel >= 0.0:
    return 0.0
  deg = SLOPE_FLAT_PITCH_DEG - math.degrees(pitch) - SLOPE_DEADBAND_DEG
  if deg <= 0.0:
    return 0.0
  w = float(np.interp(v_ego, SLOPE_V_BP, SLOPE_V_W)) * float(np.interp(accel, SLOPE_A_BP, SLOPE_A_W))
  return min(SLOPE_GAIN * deg, SLOPE_MAX) * w

class EndForceLimit:
  def __init__(self):
    self.reset()

  def reset(self):
    self.a_line = None   # request at the line's anchor (crossing V_LINE, or the end of a stall)
    self.v_line = V_LINE # speed at the anchor
    self.out = None      # last limited output (None = not limiting)
    self.stall_t = 0.0
    self.sent = None     # last output while moving (for END_RELEASE_RATE / END_FIRM_RATE)
    self.holding = False # the dynamic end is holding firmer than the ceiling / line

  def update(self, accel: float, v_ego: float, standstill: bool, long_active: bool, a_ego: float = -1.0,
             pitch: float | None = None, lead_d: float | None = None, lead_v: float | None = None) -> float:
    """pitch: calibrated pose, rad, + = nose up (downhill end braking, see downhill_extra). lead_d / lead_v: radar lead
    (None without one) for the dynamic end (END_*)"""
    if not long_active or accel >= 0.0:
      self.reset()
      return accel
    if standstill:
      self.sent, self.holding = None, False
      return self._update(accel, v_ego, standstill, a_ego, pitch)
    want = accel - downhill_extra(pitch, v_ego, accel)
    out = self._update(accel, v_ego, standstill, a_ego, pitch)
    need = end_need(v_ego, lead_d, lead_v)
    if out > want:  # the ceiling / line cut the request: never drop it in a step, and hold what the state needs
      held = min(out, -need)
      if self.sent is not None:
        # release just fast enough to be back on the end value as the wheels stop (time to stop at the present decel)
        t_stop = max(v_ego / max(-a_ego, END_DECEL_MIN), DT_CTRL)
        rate = max(END_RELEASE_RATE, (END_ACCEL - self.sent) / t_stop)
        held = float(np.clip(held, self.sent - END_FIRM_RATE * DT_CTRL, self.sent + rate * DT_CTRL))
      held = max(want, held)
      self.holding = held < out - 1e-3
      if self.holding:
        out = self.out = held  # the standstill hold then continues from what was really sent
    else:
      self.holding = False
    self.sent = out
    return out

  def _update(self, accel: float, v_ego: float, standstill: bool, a_ego: float, pitch: float | None) -> float:
    # downhill (owner-approved exception to item 0): the request and every end value below sit `extra` firmer
    extra = downhill_extra(pitch, v_ego, accel)
    accel -= extra
    end_accel = END_ACCEL - extra

    if standstill:
      if self.out is None:
        return accel
      self.out = max(accel, self.out - HOLD_RATE * DT_CTRL)
      if self.out <= accel + 1e-6:
        self.out = None
        return accel
      return self.out

    ceil_in = accel
    if v_ego <= CEIL_V:  # the fixed ceiling line (never firmer, whatever asked for it)
      ceil = float(np.interp(max(v_ego, 0.0), CEIL_BP, CEIL_A)) - extra
      if accel < ceil:
        accel = ceil
    if v_ego >= V_LINE:
      self.a_line, self.v_line, self.out, self.stall_t = accel, V_LINE, None, 0.0
      self.out = accel if accel > ceil_in else None
      return accel

    self.stall_t = self.stall_t + DT_CTRL if (v_ego > STALL_V and a_ego > STALL_A) else 0.0
    if self.a_line is None or self.stall_t >= STALL_T:  # braking started below the line, or a stall: anchor here
      self.a_line, self.v_line = accel, max(v_ego, STALL_V)
      if self.stall_t >= STALL_T:
        self.out = None
        return accel
    a_top = min(self.a_line, end_accel)
    floor = end_accel + (a_top - end_accel) * min(max(v_ego, 0.0) / self.v_line, 1.0)
    if accel < floor:
      self.out = floor
      return floor
    self.out = accel if accel > ceil_in else None
    return accel
