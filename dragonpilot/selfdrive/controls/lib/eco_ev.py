"""
dragonpilot (dppb, Corolla Altis Hybrid): eco launch on the battery (owner 2026-11-08: "eco launch like dpcr";
"an EV launch does not have to reach any particular speed - just start on electric as much as possible").

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
It gives way to the normal eco line, as dpcr:
  - when the battery is low: SOC 37 -> 33 % (unknown SOC: kept; the Altis has not been seen under 40 %);
  - while a lead pulls away quickly (vLead > vEgo + 1.0 m/s, released at vEgo) between 20 and 30 km/h and above,
    the weight moving at KEEP_UP_RATE so the catch-up builds smoothly.
"""
import math

import numpy as np

from dragonpilot.selfdrive.controls.lib.drive_mode import drive_mode_max_accel

ECO = 0  # carState.driveMode
ECO_ENGINE_OFF_BP = [0., 5 / 3.6, 10 / 3.6, 12 / 3.6, 15 / 3.6, 20 / 3.6, 25 / 3.6]  # m/s
ECO_ENGINE_OFF_MAX_ACCEL = [1.08, 1.08, 1.00, 0.95, 0.92, 0.80, 0.70]  # plan (delivered / gain x1.3 -> x1.0)
BLEND_V_BP = [15 / 3.6, 25 / 3.6]           # m/s: the engine-off line hands back to the normal eco line over 15 -> 25 km/h
EV_SOC_BP = [33., 37.]                      # % hybrid battery: low -> the normal eco line
KEEP_UP_V_BP = [20 / 3.6, 30 / 3.6]         # m/s: a lead pulling away takes the normal eco line from 20 -> 30 km/h
KEEP_UP_ON, KEEP_UP_OFF = 1.0, 0.0          # m/s: vLead - vEgo hysteresis for "the lead pulls away quickly"
KEEP_UP_RATE = 0.5                          # 1/s: the keep-up weight moves at most this fast (2 s)
EV_RATE = 1.0                               # 1/s: the engine-off weight moves at most this fast (1 s)


class EcoEv:
  def __init__(self, dt: float):
    self.dt = dt
    self.keep_up = False
    self.keep_w = 0.0  # smoothed keep-up weight
    self.ev_w = 0.0    # smoothed engine-off weight

  def max_accel(self, sm, stock_max_accel: float) -> float:
    cs = sm['carState']
    a_max = drive_mode_max_accel(cs.vEgo, cs.driveMode, stock_max_accel)
    lead = sm['radarState'].leadOne
    dv = lead.vLead - cs.vEgo if lead.status else -math.inf
    self.keep_up = dv > KEEP_UP_ON or (self.keep_up and dv > KEEP_UP_OFF)
    k_target = float(np.interp(cs.vEgo, KEEP_UP_V_BP, [0.0, 1.0])) if self.keep_up else 0.0
    self.keep_w = float(np.clip(k_target, self.keep_w - KEEP_UP_RATE * self.dt, self.keep_w + KEEP_UP_RATE * self.dt))
    ev_target = 1.0 if int(cs.driveMode) == ECO and cs.engineOff else 0.0
    self.ev_w = float(np.clip(ev_target, self.ev_w - EV_RATE * self.dt, self.ev_w + EV_RATE * self.dt))
    if self.ev_w <= 0.0 or int(cs.driveMode) != ECO:
      return a_max
    v = max(0.0, cs.vEgo)
    a_ev = min(float(np.interp(v, ECO_ENGINE_OFF_BP, ECO_ENGINE_OFF_MAX_ACCEL)), a_max)
    w = self.ev_w * float(np.interp(v, BLEND_V_BP, [1.0, 0.0]))
    w *= float(np.interp(cs.hvSoc, EV_SOC_BP, [0.0, 1.0])) if cs.hvSoc >= 0.0 else 1.0
    w *= 1.0 - self.keep_w
    return a_max + w * (a_ev - a_max)
