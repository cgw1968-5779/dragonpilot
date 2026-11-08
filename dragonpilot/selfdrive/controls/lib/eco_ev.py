"""
dragonpilot (dppb, Corolla Altis Hybrid): eco on the battery as much as possible, as dpcr - EV launch (owner 2026-11-08:
"eco launch like dpcr"; "an EV launch does not have to reach any particular speed - just start on electric as much as
possible"), and at mid / high speed an engine-off cruise line + the EV glide entry (owner 2026-11-08: "does dppb also
use EV at mid-high speed like dpcr?" -> "do it").

Acts only on the planner's EXISTING throttle ceiling (accel_clip[1], the drive mode accel profile, drive_mode.py);
the brake side, the MPC, the stopping logic, NORMAL / SPORT and eco with the engine running are untouched. The
ceiling is only ever LOWERED.

Altis data (C3X 11-01..11-08, 179 launches with the engine off at rest, hy/*.pkl, 0x1C4 / 0x380):
- the engine starts at a median 6.4 kW of wheel power (p10 3.1, p90 8.8), median 10.9 km/h in engaged launches
  (delivered ~1.9 m/s^2) and 12.7 km/h in manual ones (~1.3); launches under 1.6 m/s^2 started it at a median ~15
  km/h, those over 2.0 at ~10.7. None stayed on the battery to 20 km/h. SOC 40-51 %.
- the eco ceiling today is 1.75 / 1.50 at 0 / 11 km/h and the Altis delivers ~x1.3 the plan below ~12 km/h (2.0-2.3
  m/s^2), ~x1.0 from 20 km/h.
While the engine is off in eco the ceiling follows ECO_ENGINE_OFF_*: delivered ~1.4 m/s^2 to 5 km/h, 1.3 at 10 km/h,
then wheel power held near 6-7 kW (1.24 / 1.06 at 12 / 15 km/h), divided by the car's gain. Over 15 -> 25 km/h it
hands back to the normal eco line (only the start is on the battery, owner: no speed has to be reached), so the
delivered accel falls smoothly 1.4 -> 1.3 -> 1.06 -> 1.0 -> 1.06 -> 0.9 at 5 / 10 / 15 / 20 / 25 / 30 km/h (today
~2.0 / 2.0 / 1.6 / 1.2 / 1.06 / 0.9). Simulated with the car's gain and the 6.4 kW median engine start: the battery
carries the launch to ~15-16 km/h instead of ~8-11, 0 -> 20 km/h 5.0 s instead of 3.9, 0 -> 30 km/h unchanged within
~1.5 s. When the engine starts the weight goes back to the normal eco line at EV_RATE (1 s), so the extra pull after
an engine start builds in instead of stepping.
Engine-off cruise line (dpcr line 1, Altis values). Altis, 143 routes 09-27..11-08, engaged with the engine off: the
chance per second that the engine restarts, by the request: -0.05..+0.05 1-5 % (steady cruise needs ~0 on the Altis,
it holds speed on the battery), +0.05..+0.15 4-28 % (5 % at 50-60, 14 % at 60-70, 28 % at 70-80 km/h), +0.15..+0.30
18-39 %, > +0.30 ~100 %. 181 of the 321 engine-off spells at 50-90 km/h ended with a positive request (median 6.6-8.9
kW); the rest (hard braking / slowing) are the hybrid's own and out of reach. So from 40 km/h the engine-off ceiling is
0.15 / 0.12 / 0.08 / 0.06 / 0.05 / 0.05 at 40 / 50 / 60 / 70 / 80 / 85 km/h (dpcr Cross 0.25 / 0.16 / 0.12 at 60-80,
the Altis restarts on much less), blended in over 32 -> 40 km/h and out over 85 -> 90 km/h (the Altis barely runs on
the battery above 85: 9 % of the time). Because that ceiling is low it also gives way on a free road (no lead within
max(60 m, 3.5 s)) while the car is well under the set speed (v_cruise - v 3 -> 8 km/h, not on dpcr), so regaining the
set speed after a slowdown is not a slow crawl. Behind a lead it keeps dpcr's rule only (gives way when the lead pulls
away): open loop over the 143 routes, 77 % of the engine-off time the request was above the line was behind a lead
with the set speed > 8 km/h above, where a set-speed rule alone would have given the line away.
EV glide entry: ev_glide.py (dpcr's, SOC_MIN 43 on the Altis); after it stops the engine the line above brings the
1-2 km/h back on the battery.
It gives way to the normal eco line, as dpcr:
  - when the battery is low: SOC 37 -> 33 % (unknown SOC: kept; the Altis has not been seen under 40 %);
  - while a lead pulls away quickly (vLead > vEgo + 1.0 m/s, released at vEgo) between 20 and 30 km/h and above,
    the weight moving at KEEP_UP_RATE so the catch-up builds smoothly.
"""
import math

import numpy as np

from openpilot.common.constants import CV
from dragonpilot.selfdrive.controls.lib.drive_mode import drive_mode_max_accel
from dragonpilot.selfdrive.controls.lib.ev_glide import EvGlide

ECO = 0  # carState.driveMode
_K = 1 / 3.6
# launch 0-25 km/h, cruise 40-85 km/h (plan; launch = delivered / gain x1.3 -> x1.0); 32 km/h only shapes the blend-in
ECO_ENGINE_OFF_BP = [0., 5 * _K, 10 * _K, 12 * _K, 15 * _K, 20 * _K, 25 * _K, 32 * _K, 40 * _K, 50 * _K, 60 * _K, 70 * _K,
                     80 * _K, 85 * _K]  # m/s
ECO_ENGINE_OFF_MAX_ACCEL = [1.08, 1.08, 1.00, 0.95, 0.92, 0.80, 0.70, 0.30, 0.15, 0.12, 0.08, 0.06, 0.05, 0.05]
# weight of the engine-off line by speed: launch handed back over 15 -> 25 km/h, cruise line in over 32 -> 40, out 85 -> 90
BLEND_V_BP = [15 * _K, 25 * _K, 32 * _K, 40 * _K, 85 * _K, 90 * _K]  # m/s
BLEND_V_W = [1.0, 0.0, 0.0, 1.0, 1.0, 0.0]
DEFICIT_BP = [3 * _K, 8 * _K]               # m/s: set speed - v; well under the set speed -> the normal eco line (cruise part)
FREE_D, FREE_T = 60.0, 3.5                  # m, s: no lead nearer than max(these) = free road (as ev_glide's lead check)
EV_SOC_BP = [33., 37.]                      # % hybrid battery: low -> the normal eco line
KEEP_UP_V_BP = [20 / 3.6, 30 / 3.6]         # m/s: a lead pulling away takes the normal eco line from 20 -> 30 km/h
KEEP_UP_ON, KEEP_UP_OFF = 1.0, 0.0          # m/s: vLead - vEgo hysteresis for "the lead pulls away quickly"
KEEP_UP_RATE = 0.5                          # 1/s: the keep-up weight moves at most this fast (2 s)
EV_RATE = 1.0                               # 1/s: the engine-off weight moves at most this fast (1 s)


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
    self.keep_w = 0.0  # smoothed keep-up weight (lead pulling away, or well under the set speed)
    self.ev_w = 0.0    # smoothed engine-off weight

  def max_accel(self, sm, long_control_off: bool, stock_max_accel: float) -> float:
    cs = sm['carState']
    eco = int(cs.driveMode) == ECO
    a_max = drive_mode_max_accel(cs.vEgo, cs.driveMode, stock_max_accel)
    lead = sm['radarState'].leadOne
    dv = lead.vLead - cs.vEgo if lead.status else -math.inf
    self.keep_up = dv > KEEP_UP_ON or (self.keep_up and dv > KEEP_UP_OFF)
    k_target = float(np.interp(cs.vEgo, KEEP_UP_V_BP, [0.0, 1.0])) if self.keep_up else 0.0
    free_road = not lead.status or lead.dRel > max(FREE_D, FREE_T * cs.vEgo)
    if cs.vEgo >= BLEND_V_BP[2] and free_road:  # cruise part, nothing ahead, well under the set speed -> normal eco line
      k_target = max(k_target, float(np.interp(cs.vCruise * CV.KPH_TO_MS - cs.vEgo, DEFICIT_BP, [0.0, 1.0])))
    self.keep_w = float(np.clip(k_target, self.keep_w - KEEP_UP_RATE * self.dt, self.keep_w + KEEP_UP_RATE * self.dt))
    ev_target = 1.0 if eco and cs.engineOff else 0.0
    self.ev_w = float(np.clip(ev_target, self.ev_w - EV_RATE * self.dt, self.ev_w + EV_RATE * self.dt))
    if self.ev_w > 0.0 and eco:
      v = max(0.0, cs.vEgo)
      a_ev = min(float(np.interp(v, ECO_ENGINE_OFF_BP, ECO_ENGINE_OFF_MAX_ACCEL)), a_max)
      w = self.ev_w * float(np.interp(v, BLEND_V_BP, BLEND_V_W))
      w *= float(np.interp(cs.hvSoc, EV_SOC_BP, [0.0, 1.0])) if cs.hvSoc >= 0.0 else 1.0
      w *= 1.0 - self.keep_w
      a_max = a_max + w * (a_ev - a_max)
    cc = sm['carControl']
    pitch = cc.orientationNED[1] if len(cc.orientationNED) == 3 else math.inf  # unknown pitch: no glide
    self.glide_cap = self.ev_glide.update(eco, not long_control_off, cs.gasPressed or cs.brakePressed, cs.vEgo, cs.aEgo,
                                          cs.vCruise * CV.KPH_TO_MS, cs.engineOff, 0.0 if cs.engineOff else 1000.0,
                                          cs.hvSoc, pitch, cs.steeringAngleDeg, _Lead(lead))
    return a_max if self.glide_cap is None else min(a_max, self.glide_cap)
