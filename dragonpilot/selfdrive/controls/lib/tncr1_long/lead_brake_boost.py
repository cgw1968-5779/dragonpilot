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

# Lead hard-brake boost. Owner 2026-10-13 night, route 00000157 (device clock 23:29:36-47, Aggressive, 55 km/h): the lead
# braked at -3.6 m/s^2 to a stop; we peaked at -3.6, but below ~15 km/h the plan already eased (-1.47 at 9.9 km/h with
# the stopped lead 6.5 m ahead, -1.17 at 6.7 km/h / 4.0 m, -0.75 at 3.8 km/h / 2.4 m) and the car delivered less
# (-1.01 / -0.99 / -0.86); at 1.8 km/h it crept on the hybrid's creep torque (-0.28 -> 0) from 1.5 to 1.0 m and the
# driver had to stop it at 0.8 m: "the hard stop lacked force at the very end, it almost hit the car in front".
# Owner's rule: "do not change the 2-0 km/h end force; what needs more is 30 to 3 km/h; multiply by 1.2 to try, too heavy
# is uncomfortable". First version (94d92b0631): once the lead braked at <= -2 m/s^2, every braking request between 3 and
# 30 km/h x1.2 until the car stops or the lead drives off. On 00000157 the car travelled 15.35 m from 30 to 3 km/h;
# 1.2x the decel over the same profile is ~2.6 m shorter -> a stop ~3.4 m behind the lead instead of 0.8 m.
# Owner, same night, refined it: "x1.2 only when we are closing in fast, worked out at which hard-brake level; the harder
# the lead brakes, the higher the factor", and "simulate whether the braking stays linear and smooth".
# 19 lead-brake episodes (aLeadK < -1) on 9 Altis routes: ordinary lead stops at -2.0..-2.5 m/s^2 had TTC 5.5-5.7 s and
# need nothing; the urgent ones were 157 (-3.61, closing 20.7 km/h, TTC 2.2 s), 157 (-2.38, 11.7 km/h, 3.0 s) and 133
# 15:04 (-2.24, 15.3 km/h, 2.1 s). So it latches only when the lead brakes at <= TRIGGER_A AND we close at >= CLOSE_MIN
# with a time to collision <= TTC_MAX, and the factor follows the hardest lead decel of the event (GAIN_A/GAIN_V, 1.10 at
# -2.0 up to 1.30 at -4.0).
# Second version (cab3875796) - smoothness (C3X rlog of 157 at 100 Hz, open loop with the real requests, delivered modelled as a 0.35 s lag): a factor
# that is already full at 30 km/h made the request heavier again after the peak (-3.49 at 30 -> -4.17 at 27 km/h, a second
# bite); full from 20 km/h down and fading in from 30 keeps the stop easing monotonically: 30/24/18/12/6/3/2 km/h
# -3.49/-3.50/-3.19/-2.22/-1.41/-0.78/-0.56 (was -3.49/-3.09/-2.53/-1.77/-1.12/-0.69/-0.56), decel-vs-speed line R^2 0.980
# (0.969), delivered jerk p95 1.08 (1.04) m/s^3, no re-bite; estimated stop ~3.3 m behind the lead (0.8 m). The factor
# fades out from 4 to 2 km/h and ramps at GAIN_RATE; the 2-0 km/h end and the Toyota brake-onset shaping are unchanged.
# Owner, same night, after seeing the delivery data: "no more than that; compensation must not be too much, past analyses
# always overshot and made a mess. When the lead brakes hard add 1.2x for now, and only to the mid build-up towards the
# braking peak - not to the first light touch, and not to the 2-0 km/h end." So the factor is a flat GAIN, it waits until
# the braking has left its light first phase (request firmer than ONSET_REQ for at least ONSET_T since it started), then
# ramps in at GAIN_RATE and is held through the peak and the easing that follows (dropping it right after the peak would
# be a release step), fading out from 4 to 2 km/h.
TRIGGER_A = -2.0      # m/s^2: lead accel (filtered) that can start it
TTC_MAX = 3.5         # s: ... only while the time to collision is this short ...
CLOSE_MIN = 10.0 / 3.6  # m/s: ... and we close in at least this fast
RELEASE_A = 0.3       # m/s^2: the lead accelerating faster than this while not slower than us releases it
GAIN = 1.2
GAIN_RATE = 0.5       # 1/s
ONSET_REQ = -1.0      # m/s^2: the light first phase is over once the request is firmer than this ...
ONSET_T = 0.5         # s: ... and braking (request < BRAKING_REQ) has lasted this long
BRAKING_REQ = -0.3    # m/s^2
END_BP = [2.0 / 3.6, 4.0 / 3.6]  # m/s: faded out from 4 to 2 km/h
END_W = [0.0, 1.0]
# tncr, owner 2026-10-15 (supreme principle): "when the lead brakes hard, do not insist on stopping 3 m or more back -
# allow down to 2 m, so the brake is not too fierce". The x1.2 was sized to end ~3.3 m behind the lead (route 00000157
# ended 0.8 m without it). Now it builds only while the stop is predicted closer than BOOST_GAP (where the lead itself
# stops, minus our stopping distance at the weaker of the request and the delivered decel) and fades again once the
# prediction is back beyond BOOST_GAP_OFF; the ramp (GAIN_RATE) and the 4 -> 2 km/h fade are unchanged.
BOOST_GAP = 2.0       # m
BOOST_GAP_OFF = 2.5   # m
LEAD_STOP_A_MIN = 0.5  # m/s^2: floor of the lead decel used for where the lead stops
B_MIN = 0.3           # m/s^2: floor of our decel in the prediction


class LeadBrakeBoost:
  def __init__(self):
    self.latched = False
    self.braking_t = 0.0
    self.mid = False
    self.gain = 1.0
    self.short = False  # the stop is predicted closer than BOOST_GAP

  def update(self, accel: float, v_ego: float, standstill: bool, lead, long_active: bool, accel_min: float,
             a_ego: float | None = None, hold_off: bool = False) -> float:
    """hold_off (tncr 2026-10-15): the input already comes from the stop-gap roll guard - pass it through unchanged
    (the factor must not multiply the guard's own stop request; route 00000073 17:16:07: -2.15 x 1.2 = -2.58)"""
    present = bool(lead.present)
    closing = v_ego - lead.vLead
    ttc = lead.dRel / closing if closing > 0.3 else 99.0
    if not long_active or standstill or not present:
      self.latched = False
    elif lead.aLeadK <= TRIGGER_A and ttc <= TTC_MAX and closing >= CLOSE_MIN:
      self.latched = True
    elif lead.aLeadK > RELEASE_A and lead.vLead >= v_ego:
      self.latched = False

    # braking phase: the boost waits until the light first phase is over, then stays for the rest of this brake
    if accel < BRAKING_REQ:
      self.braking_t += DT_CTRL
      if self.braking_t >= ONSET_T and accel <= ONSET_REQ:
        self.mid = True
    elif accel >= 0.0:
      self.braking_t, self.mid = 0.0, False

    if present and a_ego is not None:
      lead_stop = lead.vLead ** 2 / (2.0 * max(-lead.aLeadK, LEAD_STOP_A_MIN)) if lead.vLead > 0.0 else 0.0
      b = max(min(-accel, -a_ego), B_MIN)
      gap = lead.dRel + lead_stop - v_ego ** 2 / (2.0 * b)
      self.short = gap < BOOST_GAP or (self.short and gap < BOOST_GAP_OFF)
    else:
      self.short = a_ego is None  # without a_ego (old callers / tests) keep the old behaviour
    boost = self.latched and self.mid and self.short
    target = 1.0 + (GAIN - 1.0) * float(np.interp(v_ego, END_BP, END_W)) if boost else 1.0
    step = GAIN_RATE * DT_CTRL
    self.gain = float(np.clip(target, self.gain - step, self.gain + step))
    if accel < 0.0 and self.gain > 1.0 and not hold_off:
      return max(accel * self.gain, accel_min)
    return accel
