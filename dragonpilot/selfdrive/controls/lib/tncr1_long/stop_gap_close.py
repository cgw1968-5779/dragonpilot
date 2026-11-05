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

# Stop-gap close. Owner 2026-10-14 evening (route 0000015c, true GPS time 19:35:40): the stop behind a stopped lead
# ended 5.5 m out (radar dRel; the plan itself aimed at ~5.4 m), and most stops that evening ended 4.3-6.4 m out.
# Owner: "I do not want the car to stop and then creep forward a second time. The system should predict that it will
# stop beyond 3.5 m, not let the car stop completely, and keep creeping" up to 3-3.5 m; "keep the speed steady, do not
# over-release so it goes fast-slow-fast"; and (owner's end-force rule) all force modulation happens above 2 km/h, below
# it only a linear taper to -0.30 at 0 km/h (end_force_limit.py, the last stage, does that for every stop).
# That is step 5 of the owner's braking principle (released too little, would stop too far -> release more).
# So: below V_MAX, behind a stopped lead, once the predicted stop gap is beyond GAP_TRIGGER even with the WEAKER of
# the request and the delivered decel (capped at B_PRED_MAX: the end of a stop is lighter than its middle), the request
# becomes the constant decel that ends at the target point, recomputed every frame from the filtered speed and radar
# distance: b = v^2 / 2(d - GAP_TARGET - TAPER_ROLL). It is self-correcting without any integrator - if the car slows
# too much b falls and it rolls on (never stops short, never a second launch), if it rolls too fast b rises - so the
# speed only falls, steadily, to the target. The request for b includes what the hybrid's creep torque eats at low
# speed (CREEP_*), corrected by a proportional decel feedback. The change into it and every change after is rate limited (FIRM_RATE / RELEASE_RATE, linear, no
# bite). It hands back to the plain request (at RETURN_RATE) when the lead moves, the radar lead drops or jumps, the plan
# stops braking, or the stop can no longer be made lightly (b > B_MAX and firmer than the plan). Only actuators.accel is
# shaped (same rule as stop_gap_hold).
# tncr (Corolla Cross) 2026-10-15 (briefly OFF in baeb992b7c). Route 00000069 00:40:20.7 (stopped lead, plan a steady -1.05 at 10 km/h, 8.9 m):
# it predicted 4.1 m and eased the request -1.06 -> -0.58 in 0.4 s; the Cross brake fell 1120 -> 280 N, decel
# -1.1 -> -0.3, then it handed back and the stop finished firmer (2280 N, -1.37 at 5 km/h), ending 2.9 m. Owner: "around
# 10-5 km/h the brake feels slightly released - I want a linear deceleration there". Route 0000006a 00:48:24 (11 km/h,
# plan a steady -0.46..-0.54): the request pumped -0.28 <-> -0.46 every ~0.5 s and the brake 120 <-> 600 N - the decel
# feedback K_A against the Cross brake lag (the car followed each swing: -0.28 -> ~120 N, -0.46 -> ~600 N).
# Owner 2026-10-15 chose the fixed version (option B) on tncr: (1) the decel feedback may only firm up, and only when the
# stop would end too close; when it would end too far the present force is HELD (owner: hold it longer, neither release
# nor keep adding brake) and only eased - slowly - while holding would still end clearly too far; (2) it eases at most
# RELEASE_RATE 0.3 m/s^3 (was 1.0), a slow straight line; (3) above V_FLOOR it never asks for less than B_FLOOR of decel unless the plan itself is lighter
# (no coasting). Below V_FLOOR the end-force line (end_force_limit) takes it to -0.35.
# The Altis (tnpb3) keeps the original f908530 law.
ENABLED = True
B_FLOOR = 0.3              # m/s^2: above V_FLOOR never ask for less decel than this (mapped like b, so it includes the
                           # creep torque below 6 km/h), unless the plan itself is lighter. Proposed as a -0.5 request floor, but
                           # routes 69/6a at 3-20 km/h: req -0.2/-0.3/-0.4 still gave aEgo -0.30/-0.44/-0.51 (regen, 80-480 N) -
                           # no dead zone there - and -0.5 would have blocked closing the gap at all on 6a
V_FLOOR = 2.0 / 3.6        # m/s
V_MAX = 18.0 / 3.6         # m/s: tncr 12 -> 30 -> 18 km/h. 30 (owner 10-15: ease above 20 km/h) backfired on route 0000006b
                           # (2026-10-15 true 09:09:10 / 09:15:17, from 50 km/h, owner 1 bookmark each): taking over at 30 km/h with
                           # 37 m left, the constant-decel prediction saw the stop 13 m short and eased -1.36 -> -0.95 below a plan
                           # that would have eased by itself; the car slid (-0.6..-0.75), then firmed to -1.5 at 7-2 km/h (aEgo
                           # -1.6..-1.7) and stopped 2.8-2.9 m out. The 3-bookmark stops that day all began it at <= 17.5 km/h.
LEAD_V_MAX = 0.5           # m/s: lead treated as stopped
GAP_TRIGGER = 3.5          # m (radar dRel): predicted stop beyond this -> close the gap (tncr 4.0 -> 3.5, owner 10-15:
                           # ideal 3-3.5 m, beyond 3.5 ease slowly / creep-adjust)
GAP_TARGET = 3.1           # m: aim of the final stop (owner: 3-3.5 m; the sim ends ~0.2 m beyond it)
TAPER_ROLL = 0.75          # m: the 2 -> 0 km/h taper to -0.30 (end_force_limit) rolls a little further: aim short of it
                           # (0.25 -> 0.75 2026-10-22 with the owner's -0.37 end line below 3 km/h: sim min gap 2.11 -> 2.62 m)
B_MIN = 0.3                # m/s^2: floor of the decel used in the prediction
B_PRED_MAX = 0.9           # m/s^2: and its cap (route 00000159 09:14:41: -1.14 at 12 km/h and 9.3 m predicted 4.4 m,
                           # the stop ended 2.8 m)
HARD_ACCEL = -2.0          # m/s^2: never during a hard brake (route 00000159 08:58:18, -3.5 at 6.7 km/h)
BRAKING_T = 0.5            # s: braking (request < 0) for this long before it may act (not right after engaging)
B_MAX = 0.8                # m/s^2: firmest decel it asks for by itself (at <= 12 km/h; see B_HI_V)
# tncr 10-15: to ease ABOVE 20 km/h (owner) the prediction cap and the firmest own decel rise with speed - route
# 00000069 at 20 km/h / 21.5 m had the plan at -1.31 (the plan alone would end ~8 m out) and needed ~0.96: with 0.9 / 0.8
# it never acted there and only eased at 10 km/h
B_HI_V = [12.0 / 3.6, 20.0 / 3.6]  # m/s
B_PRED_MAX_V = [0.9, 1.5]  # m/s^2: prediction cap by speed
B_MAX_V = [0.8, 1.5]       # m/s^2: firmest own decel by speed
ROOM_MIN = 0.1             # m
K_A = 1.0                  # (m/s^2)/(m/s^2): decel feedback - tncr: FIRMER ONLY (two-sided it pumped against the Cross
                           # brake lag; one-sided it still makes up for creep torque the mapping underestimates)
ROOM_END = 0.4             # m: this close to the target point ...
V_END = 0.7 / 3.6          # m/s: ... or this slow, the request is at least END_ACCEL - the owner's end of a stop
END_ACCEL = -0.34          # m/s^2 (tncr, Corolla Cross end of stop, -0.35 -> -0.34 owner 2026-10-15 evening; Altis -0.30)
D_FILTER_T = 0.3           # s: radar distance filter
V_FILTER_T = 0.3           # s: speed filter
# request for a decel at low speed: the hybrid's creep torque eats the first part of the request (route 00000159:
# BRAKE_FORCE ~ 480 + 1200 * |req| N against ~750 N of creep -> decel ~ 1.2 * |req| - 0.27 below ~3 km/h; -0.55 gave
# -0.44, -0.30 ~0); above ~6 km/h the PCM gives what it is asked
CREEP_V = [3.0 / 3.6, 6.0 / 3.6]  # m/s
CREEP_OFF = [0.27, 0.0]           # m/s^2
CREEP_GAIN = [1.2, 1.0]
# tncr 2026-10-17 (route 00000084, true time 21:15:04, owner: "at 10-6 km/h a very slight brake release is still felt"):
# the Cross gives LESS than asked at 9-13 km/h (delivery map 10-15: median +0.21..+0.23 steady, +0.30 while the
# command is easing; ~+0.05 at 6-9 and 13-20 km/h). The ease from -1.03 to -0.65 at 17 -> 12 km/h became a decel drop
# -0.85 -> -0.39, then the gap feedback firmed back to -0.86 and -0.89 at 5-2 km/h (ease - firm - ease - firm). The
# mapping now asks that much more in the band, so the decel it plans is the decel the car gives. Closed-loop sim of the
# stop: decel 13 -> 5 km/h -0.89..-0.42 -> -0.68..-0.59, no re-firm, gap 3.24 -> 3.25 m (second stop 3.26 -> 3.38 m).
# Zero below 5 km/h: the end of the stop (end_force_limit) is untouched. Only this stage plans with it (gap_request);
# stop_gap_hold's roll guard keeps the plain mapping.
DELIVERY_V = [5.0 / 3.6, 9.0 / 3.6, 11.0 / 3.6, 13.0 / 3.6, 20.0 / 3.6]  # m/s
DELIVERY_HOLE = [0.0, 0.15, 0.22, 0.15, 0.0]                              # m/s^2 the Cross gives less than asked
HOLD_BAND = 0.15           # m: tncr - the present force is held while the stop ends within GAP_TARGET + TAPER_ROLL and
                           # this much beyond it (3.35-3.5 m; owner 10-15: beyond 3.5 m ease slowly / creep-adjust)
RELEASE_RATE = 0.3         # m/s^3: lighter (owner: linear release; tncr 1.0 -> 0.3, no release feel at 10-5 km/h). Owner 10-15
                           # "very slow easing is fine, but do it above 20 km/h, not at the end": done by starting at
                           # V_MAX 30 km/h; a slower rate below 10 km/h (0.15/0.2) only ended late triggers farther (sim)
FIRM_RATE = 1.0            # m/s^3: firmer (no bite)
RETURN_RATE = 1.5          # m/s^3: back to a LIGHTER plain request after a hand-back
# back to a FIRMER plain request: faster the firmer the plan (tncr 10-15, owner asked for a sudden-brake check). Sim: B
# eased from 28 km/h, then a car cuts in 8 m ahead at 8 km/h and brakes -6: returning at 1.5 m/s^3 ended 1.88 m from it
# with a -3.34 peak, plan alone 2.71 m / -2.21; 5 m/s^3 gave 2.67 m / -2.72. A mild re-firm (plan ~-1) stays gentle.
RETURN_FIRM_A = [-2.5, -1.0]   # m/s^2: plan request
RETURN_FIRM_RATE = [5.0, 2.0]  # m/s^3
# ... but never a bite, even in an emergency (owner 10-15, supreme principle: "even when the lead brakes hard, the first
# brake must be light, then build linearly as needed"): the firming starts at RETURN_FIRM_START and the rate itself grows
# linearly to the value above over RETURN_FIRM_RAMP_T
RETURN_FIRM_START = 1.0        # m/s^3
RETURN_FIRM_RAMP_T = 0.3       # s
# tncr 2026-10-22, route 00000096 09:19 (engaged at 8.9 km/h, scooter 17.8 m ahead, 1.4 deg uphill; offline replay of the
# controlsd chain matches the logged command): this stage turned the plan's -0.10 into -0.35 (b = 0.22 for 3.35 m, plus
# the 9-13 km/h delivery hole 0.15 and the firm-only decel feedback), while below 7 km/h the car gave -0.60..-0.74 for
# -0.25 - three times the decel it needed - and stopped at 9.7 m. So: never firmer than the plain request while the car
# ALREADY decelerates at least b (filtered aEgo); it may firm beyond the plan again only once the decel falls
# ENOUGH_HYST below b. A state gate on the measured decel, not a proportional loop (two-sided feedback pumped on the
# Cross). The end of the stop (END_ACCEL -0.34 below V_END / ROOM_END) and end_force_limit are unchanged.
ENOUGH_HYST = 0.03         # m/s^2 (sim: 0.1 held the plan too long on a flat stop, then -0.76 at the end)
# tncr 2026-10-27, route 000000ba 21:27:01 (owner: "a slight unevenness in the stop"): at 4.8 -> 3.6 km/h the car gave
# only -0.34 for the plan's -0.48 (the hybrid's regen -> creep hand-off) and the decel feedback firmed the request
# -0.49 -> -0.92 while the plan eased to -0.40; at 3.2 km/h the end line took it back to -0.37 - a small bite and release
# in the last 2-3 m, against the owner's supreme principle (no layer may raise the force before the stop). Same at
# 21:28:53 (-0.39 -> -0.67). Below NO_FIRM_V it may only hold or ease: never firmer than both the plan and its own
# present output (the owner's -0.34 end line below is unchanged).
NO_FIRM_V = 6.0 / 3.6      # m/s
JUMP_D = 1.0               # m: a dRel step this large in one frame = a different / flickering lead
LEAD_LOST_T = 0.3          # s


def decel_request(b: float, v_ego: float) -> float:
  return -(b + float(np.interp(v_ego, CREEP_V, CREEP_OFF))) / float(np.interp(v_ego, CREEP_V, CREEP_GAIN))


def decel_from_request(req: float, v_ego: float) -> float:
  return -req * float(np.interp(v_ego, CREEP_V, CREEP_GAIN)) - float(np.interp(v_ego, CREEP_V, CREEP_OFF))


def delivery_hole(v_ego: float) -> float:
  return float(np.interp(v_ego, DELIVERY_V, DELIVERY_HOLE))


# the mapping this stage plans with (the shared ones above stay as they are for stop_gap_hold's roll guard)
def gap_request(b: float, v_ego: float) -> float:
  return decel_request(b, v_ego) - delivery_hole(v_ego)


def gap_decel_from_request(req: float, v_ego: float) -> float:
  return decel_from_request(req + delivery_hole(v_ego), v_ego)


class StopGapClose:
  def __init__(self):
    self.reset()

  def reset(self):
    self.active = False
    self.out = None          # shaped output while active / returning
    self.b_max = B_MAX
    self.return_t = 0.0      # time spent firming back toward the plan
    self.braking_t = 0.0
    self.lost_t = 0.0
    self.last_d = None
    self.v_f = None
    self.d_f = None
    self.a_f = None
    self.enough = False

  def _hand_back(self):
    self.active, self.v_f, self.d_f, self.a_f = False, None, None, None
    self.return_t = 0.0
    self.enough = False

  def update(self, accel: float, v_ego: float, a_ego: float, standstill: bool, lead, long_active: bool) -> float:
    if not ENABLED or not long_active or standstill:
      self.reset()
      return accel

    self.braking_t = self.braking_t + DT_CTRL if accel < 0.0 else 0.0
    present = bool(lead.present)
    self.lost_t = 0.0 if present else self.lost_t + DT_CTRL
    d = lead.dRel if present else None
    jumped = d is not None and self.last_d is not None and abs(d - self.last_d) > JUMP_D
    if d is not None:
      self.last_d = d

    if self.active:
      if self.lost_t > LEAD_LOST_T or jumped or (present and lead.vLead >= LEAD_V_MAX) or accel >= 0.0:
        self._hand_back()
    elif present and lead.vLead < LEAD_V_MAX and not jumped and v_ego <= V_MAX and self.braking_t >= BRAKING_T and \
         accel > HARD_ACCEL:
      b = float(np.clip(min(-accel, -a_ego), B_MIN, np.interp(v_ego, B_HI_V, B_PRED_MAX_V)))
      if d - v_ego ** 2 / (2.0 * b) > GAP_TRIGGER:
        self.active = True
        self.out = accel
        # the firmest own decel is fixed at the speed it took over: dropping it with the speed would hand a stop it eased
        # at 25 km/h back to a firmer plan at 11 km/h (open-loop replay of route 00000069)
        self.b_max = float(np.interp(v_ego, B_HI_V, B_MAX_V))

    if self.active:
      self.v_f = v_ego if self.v_f is None else self.v_f + (v_ego - self.v_f) * DT_CTRL / V_FILTER_T
      self.d_f = self.last_d if self.d_f is None else self.d_f + (self.last_d - self.d_f) * DT_CTRL / D_FILTER_T
      b = self.v_f ** 2 / (2.0 * max(self.d_f - GAP_TARGET - TAPER_ROLL, ROOM_MIN))
      self.a_f = a_ego if self.a_f is None else self.a_f + (a_ego - self.a_f) * DT_CTRL / V_FILTER_T
      # tncr, owner 2026-10-15: "when the gap is too large, do not release and do not keep adding brake - HOLD the
      # current force longer; only adding brake can itself end the stop too far". Three zones by where the car would
      # stop with the decel the present command asks for (b_out, the inverse of decel_request - not the measured decel,
      # which lags and would make easing feed on itself): b is the decel that ends at GAP_TARGET (+ the taper roll),
      # b_far the one that ends HOLD_BAND beyond that.
      b_out = gap_decel_from_request(self.out, v_ego)
      b_far = self.v_f ** 2 / (2.0 * max(self.d_f - GAP_TARGET - TAPER_ROLL - HOLD_BAND, ROOM_MIN))
      # firm toward b; decelerating less than b firms more (proportional decel feedback, firmer only - two-sided it
      # pumped against the Cross brake lag)
      firm = gap_request(b, v_ego) + K_A * min(-b - self.a_f, 0.0)
      if b_out < b:
        target = firm                               # would end too close: firm up
      elif b_out <= b_far:
        target = min(self.out, firm)                # ends within 3.35-3.8 m: hold the present force (firmer only if the
                                                    # car is not decelerating as asked)
      else:
        target = max(gap_request(b_far, v_ego), self.out - RELEASE_RATE * DT_CTRL)  # clearly too far even holding:
                                                    # ease slowly, only down to the edge of the hold band
      b_max = self.b_max
      if b > b_max and accel < target:  # can no longer be done lightly and the plan brakes firmer: it takes over
        self._hand_back()
      else:
        target = min(max(target, gap_request(b_max, v_ego)), 0.0)
        if -self.a_f >= b:
          self.enough = True
        elif -self.a_f < b - ENOUGH_HYST:
          self.enough = False
        if self.enough:  # the car already slows enough: no firmer than the plan
          target = max(target, min(accel, 0.0))
        if v_ego < NO_FIRM_V:  # the last metres: hold or ease only
          target = max(target, min(accel, self.out))
        if self.d_f - GAP_TARGET < ROOM_END or self.v_f < V_END:  # the stop itself: at least the owner's end -0.35
          target = min(target, END_ACCEL)
        if v_ego > V_FLOOR:  # tncr: never a coast above 2 km/h
          target = min(target, max(accel, gap_request(B_FLOOR, v_ego)))
        self.out = float(np.clip(target, self.out - FIRM_RATE * DT_CTRL, self.out + RELEASE_RATE * DT_CTRL))
        return self.out

    if self.out is not None:            # returning to the plain request after a hand-back
      self.return_t = self.return_t + DT_CTRL if accel < self.out else 0.0
      firm_rate = float(np.interp(accel, RETURN_FIRM_A, RETURN_FIRM_RATE))
      firm_rate = RETURN_FIRM_START + (firm_rate - RETURN_FIRM_START) * min(self.return_t / RETURN_FIRM_RAMP_T, 1.0)
      self.out = float(np.clip(accel, self.out - firm_rate * DT_CTRL, self.out + RETURN_RATE * DT_CTRL))
      if abs(self.out - accel) < 1e-3:
        self.out = None
        return accel
      return self.out
    return accel
