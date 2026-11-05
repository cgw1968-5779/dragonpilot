"""
dragonpilot (dpcr, Corolla Cross Hybrid): how the car slows to a LOWERED set speed with no lead in charge.

Ported from sunnypilot tncr1 accel_controller.get_cruise_target (owner requests 2026-09-25/26, 10-27) plus one new
part (owner 2026-11-03). Stock dp hands the lowered set speed straight to the MPC, which brakes for it: d2 route
11-03 20:04:58 (81 km/h, 82 -> 77) sent -0.17 -> -0.49 -> -0.58, 20:28:16 (62 -> 58) went to -1.09. Instead the
planner's cruise target becomes the speed one step of a SCHEDULED decel ahead (the MPC input, not a layer after it):

- eco (driveMode 0): never hurry, however far the set speed dropped or how many presses. 70-80 km/h: pure coasting
  (the planner's pitch-aware get_coast_accel: no throttle, no regen; -0.30 on the flat). 70 -> 60 km/h it blends
  into -0.40, then -0.40 at 60 grows to -0.50 at 30 km/h (owner 10-27: several -5 presses from 70-80 down to 30 km/h
  should slow a little firmer as the speed falls), easing again to -0.34 at 0. With the set speed at 30 km/h or below
  a descent never coasts gentler than the flat (-0.30).
- normal / sport: one constant decel latched when the gap opens (gap / 3.5 s, 3.0 s), never gentler than -0.50 /
  -0.65, held to the target.
- NEW, all modes, 80 km/h and up (owner 2026-11-03: "at high speed -5 must slow very smoothly - not lifting off the
  throttle completely but only easing it"): HIGH_FRAC of the pitch-aware coast (half: -0.15 on the flat instead of
  -0.30), so the PCM keeps some throttle; blended from the mode's value between 75 and 80 km/h.
- set speed down to 0 (owner 2026-11-05, both cars): "set 0 from 35 km/h and up follows the -5 principle; the brake is
  firmest at 30 and 20 km/h, then eases linearly to the stop". The eco line holds -0.50 from 30 down to 20 km/h and
  eases linearly to the end value at 0 (-0.30); a target below LOW_TARGET_MAX (the new 25 ... 0 settings) follows the
  eco line in every drive mode (a latched normal / sport decel would be firmest at the start, gap / 3.5 s from 60 km/h
  is beyond -1.2), so its firmest part is always at 30-20 km/h.
- the last second before the set speed: the decel shrinks in proportion to the gap left, and the target never goes
  below the set speed itself (lands without overshoot).

Only while the set speed is below the current speed and the MPC's cruise term is the one in charge; with a lead in
charge, force_slow_decel, an unset cruise or no drive mode (-1) the set speed passes unchanged. It only lowers the
target gently - never asks for more than the mode's value - and leaves following, stopping and accelerating as they are.
"""
import numpy as np

ECO, NORMAL, SPORT = 0, 1, 2  # carState.driveMode

ECO_DECEL_BP = [0., 5.56, 8.33, 16.67, 19.44, 25.0, 33.3]       # m/s (0, 20, 30, 60, 70, 90, 120 km/h)
ECO_DECEL_V = [-0.30, -0.50, -0.50, -0.40, -0.22, -0.30, -0.40]  # m/s^2; >= 70 km/h only the fallback coast estimate
LOW_TARGET_MAX = 22.0 / 3.6                              # m/s: targets below it (dial 25 ... 0) use the eco line in every mode
ECO_COAST_BLEND_BP = [16.67, 19.44]                      # m/s: -0.40 at 60 km/h -> pure coast at 70 km/h
COAST_MIN, COAST_MAX = -1.2, -0.05                       # m/s^2: sanity clip on the planner's coast accel
ECO_COAST_FLAT = -0.30                                   # m/s^2: descent floor once the set speed is <= 30 km/h
ECO_COAST_FLAT_SET_MAX = 30.0 / 3.6                      # m/s
LATCH_RESPONSE_T = {NORMAL: 3.5, SPORT: 3.0}             # s to close the gap
LATCH_MIN_DECEL = {NORMAL: -0.50, SPORT: -0.65}          # m/s^2: gentlest latched decel
HIGH_BP = [75.0 / 3.6, 80.0 / 3.6]                       # m/s: blend into the high-speed easing
HIGH_FRAC = 0.5                                          # of the coast decel: throttle eased, not lifted
TAPER_T = 1.0                                            # s
# dp's MPC follows the cruise speed like a lead moving at it, closing a speed difference with a ~3 s time constant: a
# target LOOKAHEAD_T of the decel ahead makes the MPC deliver that decel (x86 closed-loop sim, eco, flat: 100/85/65/50/
# 40 km/h -0.14/-0.14/-0.37/-0.43/-0.47 for -0.15/-0.15/-0.39/-0.45/-0.49; 1 s gave a third of it)
LOOKAHEAD_T = 3.0                                        # s: cruise target = v_ego + decel * LOOKAHEAD_T


class CruiseDecel:
  def __init__(self):
    self.latched: float | None = None
    self.latched_target: float | None = None

  def _reset(self):
    self.latched, self.latched_target = None, None

  def decel(self, drive_mode: int, v_ego: float, v_target: float, accel_coast: float | None) -> float:
    gap = v_target - v_ego  # < 0
    coast = None
    if accel_coast is not None and np.isfinite(accel_coast) and accel_coast < 0.0:
      coast = float(np.clip(accel_coast, COAST_MIN, COAST_MAX))

    if drive_mode == ECO or v_target < LOW_TARGET_MAX:
      d = float(np.interp(v_ego, ECO_DECEL_BP, ECO_DECEL_V))
      if coast is not None:
        c = min(coast, ECO_COAST_FLAT) if v_target <= ECO_COAST_FLAT_SET_MAX else coast
        w = float(np.interp(v_ego, ECO_COAST_BLEND_BP, [0.0, 1.0]))
        low = float(np.interp(min(v_ego, ECO_COAST_BLEND_BP[0]), ECO_DECEL_BP, ECO_DECEL_V))
        d = (1.0 - w) * low + w * c
    else:
      if self.latched is None or self.latched_target != v_target:
        self.latched = min(LATCH_MIN_DECEL[drive_mode], gap / LATCH_RESPONSE_T[drive_mode])
        self.latched_target = v_target
      d = self.latched

    # 80 km/h and up: ease the throttle instead of lifting it (half the coast decel), all modes
    high = HIGH_FRAC * (coast if coast is not None else float(np.interp(v_ego, ECO_DECEL_BP, ECO_DECEL_V)))
    w_high = float(np.interp(v_ego, HIGH_BP, [0.0, 1.0]))
    d = (1.0 - w_high) * d + w_high * high

    return max(d, gap / TAPER_T)  # both negative: the gentler one, so the speed lands on the set value

  def update(self, drive_mode: int, v_ego: float, v_cruise: float, accel_coast: float | None, bypass: bool) -> float:
    if bypass or drive_mode not in (ECO, NORMAL, SPORT) or not np.isfinite(v_cruise) or v_cruise < 0.0 or v_cruise >= v_ego:
      self._reset()
      return v_cruise
    return float(max(v_cruise, v_ego + self.decel(drive_mode, v_ego, v_cruise, accel_coast) * LOOKAHEAD_T))  # never below the set speed
