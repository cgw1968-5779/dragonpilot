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
"""
import math

import numpy as np

from openpilot.common.constants import CV
from dragonpilot.selfdrive.controls.lib.drive_mode import drive_mode_max_accel
from dragonpilot.selfdrive.controls.lib.ev_glide import EvGlide

ECO = 0  # carState.driveMode
ECO_ENGINE_OFF_BP = [0., 3., 12., 16.67, 19.44, 22.22, 24., 36.]  # m/s
ECO_ENGINE_OFF_MAX_ACCEL = [1.85, 1.55, 0.35, 0.25, 0.16, 0.12, 0.08, 0.06]


class _Lead:
  """radarState.leadOne as ev_glide expects it (dp: status instead of present)"""
  def __init__(self, lead):
    self.present, self.dRel, self.vLead = bool(lead.status), lead.dRel, lead.vLead


class EcoEv:
  def __init__(self, dt: float):
    self.ev_glide = EvGlide(dt)
    self.glide_cap: float | None = None

  def max_accel(self, sm, long_control_off: bool, stock_max_accel: float) -> float:
    cs, cc = sm['carState'], sm['carControl']
    eco = int(cs.driveMode) == ECO
    if eco and cs.engineOff:
      a_max = float(np.interp(max(0.0, cs.vEgo), ECO_ENGINE_OFF_BP, ECO_ENGINE_OFF_MAX_ACCEL))
    else:
      a_max = drive_mode_max_accel(cs.vEgo, cs.driveMode, stock_max_accel)
    pitch = cc.orientationNED[1] if len(cc.orientationNED) == 3 else math.inf  # unknown pitch: no glide
    self.glide_cap = self.ev_glide.update(eco, not long_control_off, cs.gasPressed or cs.brakePressed, cs.vEgo, cs.aEgo,
                                          cs.vCruise * CV.KPH_TO_MS, cs.engineOff, 0.0 if cs.engineOff else 1000.0,
                                          cs.hvSoc, pitch, cs.steeringAngleDeg, _Lead(sm['radarState'].leadOne))
    return a_max if self.glide_cap is None else min(a_max, self.glide_cap)
