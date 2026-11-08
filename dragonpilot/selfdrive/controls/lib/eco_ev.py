"""
dragonpilot (dpcr, Corolla Cross Hybrid): stay on the battery as much as possible in ECO, ported from the owner's
sunnypilot branch tncr1 (owner 2026-11-03: "add tncr1's eco-mode corrections that use EV mode as much as possible").

Both act only on the planner's EXISTING throttle ceiling (accel_clip[1], the drive mode accel profile); the brake side,
the MPC, the stopping logic and NORMAL / SPORT are untouched.

1. Engine-off eco line (tncr1 accel_controller ECO_ENGINE_OFF_*, from tncr18 9408976266 / 7c7540f433): while the
   hybrid's engine is OFF, eco keeps the wheel power m*a*v + road load under the ~11-14 kW at which the Corolla Cross
   starts its engine (30 Cross rlogs, not SOC dependent): 60 km/h 0.25, 70 km/h 0.16, 80 km/h 0.12 m/s^2. Once the
   engine runs there is nothing to save, so the normal eco line (drive_mode.py) takes over until it stops again.
2. EV glide entry (ev_glide.py, tncr1 c5f60391f1): cruising steadily at the set speed in eco with the engine running,
   flat, straight, no lead near, battery >= 35 %: a light automatic lift (ceiling down to -0.30 at 0.6 m/s^3, held until
   the engine reports off, at most 2 s, released at 0.5 m/s^3) - the owner's own "release the accelerator for half a
   second" cue that lets the hybrid stop its engine; line 1 then brings the speed back on the battery.
3. EV launch (owner 2026-11-07: "the eco launch is a bit strong - lighter, launch on EV unless the battery is low, but
   once above 20-30 km/h and the lead must be followed, keeping up comes first"). 35 engaged eco/normal launches
   (C4 11-06/11-07 qlogs, dpcr_1107/eco): the car delivered ~2.1 m/s^2 at 10 km/h (x1.25 the plan below ~12 km/h)
   and the engine started in 32 of 33 engine-off launches at 10-16 km/h, ~10-11 kW (lowest 7.8 kW). Line 1 now holds
   the launch to ~7 kW of wheel power: delivered <= 1.5 m/s^2 to 10 km/h, then (7 kW - road load) / (m v):
   plan 1.20 / 0.93 / 0.73 / 0.56 / 0.44 / 0.35 at 0-10 / 15 / 20 / 25 / 30 / 35 km/h (0 -> 20 km/h ~4.6 s, was ~3.4).
   11-08: replaced by the owner's own EV launch (route f8 00:28, see ECO_ENGINE_OFF_BP).
   It gives way to the normal eco line
     - when the battery is low: SOC 37 -> 33 % (as ev_glide: >= 35; unknown SOC: kept);
     - while a lead pulls away quickly (vLead > vEgo + 1.0 m/s, released at vEgo; 11-08, was +-0.5) between 20 and 30
       km/h and above; the weight moves at KEEP_UP_RATE so the catch-up builds smoothly.
   The engine-running eco line, normal and sport are untouched.
4. Engine-off cruise line by grade (owner 2026-11-08: "below 100 km/h, battery above the allowed level, eco on flat or
   downhill roads should use EV as much as possible"; dppb 0922510c method on C4 Cross data, 167 routes / 35.8 h,
   dpcr_1108). Engaged with the engine off, the chance per second that the Cross restarts its engine, by request and
   speed, on a FLAT road (grade within +-0.7 deg): 50-60 km/h +0.09..0.12 0 %, +0.12..0.16 6 %; 60-70 +0.03..0.06 1 %,
   +0.06..0.09 5 %, +0.09..0.12 10 %; 70-80 +0.06..0.09 2 %, +0.09..0.12 20 %; 80-90 0..+0.03 5 %, +0.03..0.06 14 %,
   +0.06..0.09 26 %; 90-100 <= 0 3 %, 0..+0.03 8 %. Line 1 (0.25 / 0.16 / 0.12 / 0.08 at 60 / 70 / 80 / 86 km/h) sat
   at 14-48 % there. On a gentle DOWNHILL (-2..-0.7 deg) the same requests restart it only 0-6 % (gravity carries part
   of the wheel power), uphill (> +0.7 deg) 13-31 % already at +0.03..0.09. So above 35 km/h, with the engine off:
     flat line ECO_EV_FLAT_* 0.40 / 0.20 / 0.14 / 0.11 / 0.08 / 0.06 / 0.05 / 0.03 / 0.01 / 0
       at 35 / 45 / 50 / 55 / 60 / 70 / 76 / 82 / 88 / 95 km/h (~3 %/s or less, flat),
     downhill: + g * sin|grade| (the wheel power the slope gives), never above line 1,
     uphill: back to line 1 over a grade of +0.3 -> +0.7 deg (uphill untouched),
     unknown pitch: line 1.
   grade = carControl pitch - FLAT_PITCH (+0.61 deg on a GPS-flat road, + = uphill), filtered with GRADE_TAU so the
   ceiling does not follow IMU noise. It is only ever at or below line 1, i.e. today's ceiling or lower.
   Because it is low, on a free road (no lead within max(60 m, 3.5 s), as ev_glide) and above 35 -> 40 km/h it gives
   way to the normal eco line while the car is well under the set speed (3 -> 8 km/h, dppb's rule), so regaining the
   set speed is not a slow crawl; behind a lead the pull-away rule above stays. When the engine stops above 35 km/h
   the engine-off line comes in over EV_RATE (1 s) instead of at once; an engine start hands back at once (as before).
"""
import math

import numpy as np

from openpilot.common.constants import CV
from dragonpilot.selfdrive.controls.lib.drive_mode import drive_mode_max_accel
from dragonpilot.selfdrive.controls.lib.ev_glide import EvGlide

ECO = 0  # carState.driveMode
# Launch part (0-35 km/h) = the owner's own EV launch (owner 2026-11-08: "00:28 I launched by hand with EV force -
# work out what it should be and put it in as the eco launch with no lead"). Route 000000f8 00:28:11 (eco, SOC 44 %,
# gas pedal, engine off until 34 km/h at ~8.4 kW), delivered aEgo median per band: 0-5 km/h 1.39, 5-10 1.16, 10-15 0.91,
# 15-20 0.73, 20-25 0.63, 25-30 0.51, 30-35 0.44. The plan here is that / the car's gain (x1.25 the plan below ~10
# km/h, 1.0 from 18 km/h; 35 engaged launches 11-06/11-07): 1.12 / 1.00 / 0.80 / 0.76 / 0.68 / 0.56 / 0.47 / 0.40.
ECO_ENGINE_OFF_BP = [0., 5 / 3.6, 10 / 3.6, 15 / 3.6, 20 / 3.6, 25 / 3.6, 30 / 3.6, 35 / 3.6, 16.67, 19.44, 22.22, 24., 36.]  # m/s
ECO_ENGINE_OFF_MAX_ACCEL = [1.12, 1.00, 0.80, 0.76, 0.68, 0.56, 0.47, 0.40, 0.25, 0.16, 0.12, 0.08, 0.06]
EV_SOC_BP = [33., 37.]                      # % hybrid battery: low (ev_glide also needs >= 35) -> the normal eco line
KEEP_UP_V_BP = [20 / 3.6, 30 / 3.6]         # m/s: a lead pulling away takes the normal eco line from 20 -> 30 km/h
KEEP_UP_ON, KEEP_UP_OFF = 1.0, 0.0          # m/s: vLead - vEgo hysteresis for "the lead pulls away quickly" (11-08: was 0.5 / -0.5)
KEEP_UP_RATE = 0.5                          # 1/s: the keep-up weight moves at most this fast (smooth catch-up, 2 s)
_K = 1 / 3.6
ECO_EV_FLAT_BP = [35 * _K, 45 * _K, 50 * _K, 55 * _K, 60 * _K, 70 * _K, 76 * _K, 82 * _K, 88 * _K, 95 * _K]  # m/s (2026-11-08)
ECO_EV_FLAT_MAX_ACCEL = [0.40, 0.20, 0.14, 0.11, 0.08, 0.06, 0.05, 0.03, 0.01, 0.0]
CRUISE_V = 35 * _K                          # m/s: the cruise line applies above this (at 35 km/h it equals line 1)
FLAT_PITCH = math.radians(0.61)             # rad: what a GPS-flat road reads (+ = uphill)
UP_GRADE_BP = [math.radians(0.3), math.radians(0.7)]  # rad: uphill -> back to line 1
GRADE_TAU = 2.0                             # s: grade filter
G = 9.81
DEFICIT_BP = [3 * _K, 8 * _K]               # m/s: set speed - v; well under the set speed on a free road -> normal eco line
DEFICIT_V_BP = [35 * _K, 40 * _K]           # m/s: only in the cruise part
FREE_D, FREE_T = 60.0, 3.5                  # m, s: no lead nearer than max(these) = free road
EV_RATE = 1.0                               # 1/s: the engine-off weight comes in at most this fast above CRUISE_V


def engine_off_line(v: float, grade: float) -> float:
  """eco ceiling while the engine is off: line 1 below 35 km/h, the grade-aware cruise line above (never above line 1)"""
  line1 = float(np.interp(max(0.0, v), ECO_ENGINE_OFF_BP, ECO_ENGINE_OFF_MAX_ACCEL))
  if v <= CRUISE_V or not math.isfinite(grade):
    return line1
  a = min(line1, float(np.interp(v, ECO_EV_FLAT_BP, ECO_EV_FLAT_MAX_ACCEL)) + G * math.sin(max(-grade, 0.0)))
  return a + float(np.interp(grade, UP_GRADE_BP, [0.0, 1.0])) * (line1 - a)


class _Lead:
  """radarState.leadOne as ev_glide expects it (dp: status instead of present)"""
  def __init__(self, lead):
    self.present, self.dRel, self.vLead = bool(lead.status), lead.dRel, lead.vLead


class EcoEv:
  def __init__(self, dt: float):
    self.dt = dt
    self.ev_glide = EvGlide(dt)
    self.glide_cap: float | None = None
    self.keep_up = False
    self.keep_w = 0.0  # smoothed keep-up weight
    self.ev_w = 0.0    # engine-off weight (comes in at EV_RATE above CRUISE_V)
    self.grade = None  # filtered grade, rad

  def max_accel(self, sm, long_control_off: bool, stock_max_accel: float) -> float:
    cs, cc = sm['carState'], sm['carControl']
    eco = int(cs.driveMode) == ECO
    a_max = drive_mode_max_accel(cs.vEgo, cs.driveMode, stock_max_accel)
    lead = sm['radarState'].leadOne
    dv = lead.vLead - cs.vEgo if lead.status else -math.inf
    self.keep_up = dv > KEEP_UP_ON or (self.keep_up and dv > KEEP_UP_OFF)
    pitch = cc.orientationNED[1] if len(cc.orientationNED) == 3 else math.inf  # unknown pitch: no glide
    if math.isfinite(pitch):
      g = pitch - FLAT_PITCH
      self.grade = g if self.grade is None else self.grade + self.dt / (GRADE_TAU + self.dt) * (g - self.grade)
    else:
      self.grade = None
    off = eco and cs.engineOff
    if not off:
      self.ev_w = 0.0
    elif cs.vEgo <= CRUISE_V:
      self.ev_w = 1.0
    else:
      self.ev_w = min(1.0, self.ev_w + EV_RATE * self.dt)
    if off:
      a_ev = engine_off_line(cs.vEgo, self.grade if self.grade is not None else math.nan)
      w = float(np.interp(cs.hvSoc, EV_SOC_BP, [0.0, 1.0])) if cs.hvSoc >= 0.0 else 1.0
      k_target = float(np.interp(cs.vEgo, KEEP_UP_V_BP, [0.0, 1.0])) if self.keep_up else 0.0
      free_road = not lead.status or lead.dRel > max(FREE_D, FREE_T * cs.vEgo)
      if free_road:
        k_target = max(k_target, float(np.interp(cs.vEgo, DEFICIT_V_BP, [0.0, 1.0])) *
                       float(np.interp(cs.vCruise * CV.KPH_TO_MS - cs.vEgo, DEFICIT_BP, [0.0, 1.0])))
      self.keep_w = float(np.clip(k_target, self.keep_w - KEEP_UP_RATE * self.dt, self.keep_w + KEEP_UP_RATE * self.dt))
      w *= (1.0 - self.keep_w) * self.ev_w
      a_max = a_max + w * (a_ev - a_max)
    self.glide_cap = self.ev_glide.update(eco, not long_control_off, cs.gasPressed or cs.brakePressed, cs.vEgo, cs.aEgo,
                                          cs.vCruise * CV.KPH_TO_MS, cs.engineOff, 0.0 if cs.engineOff else 1000.0,
                                          cs.hvSoc, pitch, cs.steeringAngleDeg, _Lead(sm['radarState'].leadOne))
    return a_max if self.glide_cap is None else min(a_max, self.glide_cap)
