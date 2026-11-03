"""
dragonpilot (dpcr, Corolla Cross Hybrid): EV glide entry, ported unchanged from the owner's sunnypilot branch tncr1
(sunnypilot/selfdrive/controls/lib/accel_controller/ev_glide.py). Driven by dragonpilot/selfdrive/controls/lib/eco_ev.py.
"""
import math

import numpy as np

# EV glide entry (tncr, Corolla Cross hybrid, owner 2026-10-27). Route 000000ba 21:44: eco cruise at 68 km/h true
# (dash 75) on a flat road, the engine never stopped. The owner's own way into EV mode: "release the accelerator for
# half a second, then press it back in slowly"; asked for the system to do it, preferably with the battery at least
# half charged. Route ba, flat cruise 60-85 km/h with the engine running: whenever the request went to <= -0.3 the
# engine stopped 15 of 17 times (-0.1..-0.3: 13 of 32; ~0: 6 of 253), median ~1.2 s after the lift. The hybrid
# ECU decides; this only gives it the same cue the owner's foot does.
#
# While cruising steadily at the set speed in eco, engine running, flat and straight, no lead near, battery at least
# SOC_MIN: the planner's accel ceiling is taken down to GLIDE_A at GLIDE_JERK (a light lift, no step), held until the
# engine reports stopped (+ CONFIRM_T) or for at most HOLD_T, then given back at RELEASE_JERK. The eco engine-off
# ceiling (accel_controller ECO_ENGINE_OFF_*) then brings the speed back gently on the battery. Cost: ~1-2 km/h. A try
# that did not stop the engine waits RETRY_FAIL_T before the next; a successful one RETRY_T. Anything the driver does
# (gas, brake, set speed change, disengage) or a lead coming close ends it at once (released at RELEASE_JERK).
#
# SOC: CAN 0x380 byte 6 (HYBRID_BATTERY.HV_SOC_PCT, carStateSP.hvSoc). On the Cross it ranged 32-52 over 10-15..10-27
# (route medians: p10 38, median 44, p90 49); how it maps to the dash's 8 bars is not known yet. At 21:44 on route ba
# the owner saw a full battery while it read 35-36, so SOC_MIN is 35 for now (40 would never have fired on route ba,
# 35 gives 7 tries in 2.5 h open loop, 21:45 among them), to be calibrated against bar readings. A low battery is also
# protected by the hybrid ECU itself (it simply keeps the engine on) and by RETRY_FAIL_T.
V_MIN = 55.0 / 3.6          # m/s
V_MAX = 85.0 / 3.6          # m/s
SOC_MIN = 35.0              # HV_SOC_PCT
STEADY_DV = 2.0 / 3.6       # m/s: |v - set speed| within this ...
STEADY_A = 0.25             # m/s^2: ... and |aEgo| below this ...
STEADY_T = 5.0              # s: ... for this long
PITCH_MAX = math.radians(1.2)  # route ba 21:44, a road the owner calls flat, read 0.8 deg (~1.4 %)
ANGLE_MAX = 5.0             # deg steering wheel
LEAD_T = 3.5                # s: a lead nearer than this time gap (or LEAD_D_MIN) blocks it ...
LEAD_D_MIN = 60.0           # m
LEAD_DV = 1.0               # m/s: ... or any lead slower than us by more than this
GLIDE_A = -0.30             # m/s^2: the lift
GLIDE_JERK = 0.6            # m/s^3: down to it
HOLD_T = 2.0                # s: at most this long at GLIDE_A
CONFIRM_T = 0.3             # s: engine reported off this long -> done
RELEASE_JERK = 0.5          # m/s^3
RETRY_T = 60.0              # s after an attempt that stopped the engine
RETRY_FAIL_T = 180.0        # s after one that did not
ENGINE_ON_T = 20.0          # s the engine must have been running (not a restart we would fight at once)


class EvGlide:
  def __init__(self, dt: float):
    self.dt = dt
    self.cap: float | None = None  # accel ceiling while gliding / releasing, None = no effect
    self.state = "idle"            # idle -> lift -> hold -> release -> idle
    self.steady_t = 0.0
    self.engine_on_t = 0.0
    self.hold_t = 0.0
    self.off_t = 0.0
    self.cooldown = 0.0
    self.stopped_engine = False
    self.v_cruise = None

  def _ready(self, v_ego, a_ego, v_cruise, engine_off, soc, pitch, angle, lead) -> bool:
    if not (V_MIN <= v_ego <= V_MAX) or engine_off or not math.isfinite(soc) or soc < SOC_MIN or soc > 100.0:
      return False
    if abs(pitch) > PITCH_MAX or abs(angle) > ANGLE_MAX:
      return False
    if lead is not None and lead.present and (lead.dRel < max(LEAD_D_MIN, LEAD_T * v_ego) or lead.vLead < v_ego - LEAD_DV):
      return False
    return abs(v_ego - v_cruise) <= STEADY_DV and abs(a_ego) < STEADY_A

  def update(self, eco: bool, long_active: bool, driver: bool, v_ego: float, a_ego: float, v_cruise: float,
             engine_off: bool, engine_rpm: float, soc: float, pitch: float, angle: float, lead) -> float | None:
    """v_cruise: the set speed (m/s). driver: gas or brake pressed. Returns the accel ceiling, or None."""
    self.cooldown = max(self.cooldown - self.dt, 0.0)
    self.engine_on_t = 0.0 if (engine_off or engine_rpm < 300.0) else self.engine_on_t + self.dt
    set_changed = self.v_cruise is not None and abs(v_cruise - self.v_cruise) > 0.1
    self.v_cruise = v_cruise
    enabled = eco and long_active and not driver
    ready = enabled and self._ready(v_ego, a_ego, v_cruise, engine_off, soc, pitch, angle, lead)
    self.steady_t = self.steady_t + self.dt if ready else 0.0

    if self.state in ("lift", "hold"):
      lead_near = lead is not None and lead.present and lead.dRel < max(LEAD_D_MIN, LEAD_T * v_ego) * 0.8
      if not enabled or set_changed or lead_near:
        self._end(False)
      elif engine_off:
        self.off_t += self.dt
        if self.off_t >= CONFIRM_T:
          self._end(True)
      else:
        self.off_t = 0.0
        if self.state == "lift":
          self.cap = max(self.cap - GLIDE_JERK * self.dt, GLIDE_A)
          if self.cap <= GLIDE_A:
            self.state = "hold"
        else:
          self.hold_t += self.dt
          if self.hold_t >= HOLD_T:
            self._end(False)
      return self.cap

    if self.state == "release":
      self.cap += RELEASE_JERK * self.dt
      if self.cap >= 0.5 or not long_active:  # above any eco cruise ceiling at these speeds: no effect left
        self.state, self.cap = "idle", None
      return self.cap

    if ready and self.steady_t >= STEADY_T and self.engine_on_t >= ENGINE_ON_T and self.cooldown <= 0.0:
      self.state, self.cap, self.hold_t, self.off_t = "lift", float(np.clip(a_ego, GLIDE_A, 0.0)), 0.0, 0.0
      return self.cap
    return None

  def _end(self, success: bool):
    self.stopped_engine = success
    self.cooldown = RETRY_T if success else RETRY_FAIL_T
    self.state = "release"
    self.steady_t = 0.0
