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
- set speed 0 firmer (owner 2026-11-09, dppb): "from about 70 km/h set to 0, below 40 I want the strongest slowing,
  can it go past -1.8? -2.2: from -1.0 at 60 km/h grow linearly to -2.2 at 30, hold -2.2 down to 10 km/h, then the
  end phase into a smooth stop". With the set speed at 0 only: the eco value at 70 km/h (coasting) blends into -1.0
  at 60, grows linearly to SET0_DECEL_V[1] at 30, holds it down to 6 km/h and eases linearly to the Altis end value
  (-0.30) at 0; the soft stop end in longcontrol takes the last ~1 s as before. The MPC smooths any corner over ~1-2 s
  (jerk / accel-change costs), so the hold is drawn to 6 km/h at -2.15 for it to deliver the owner's shape without
  passing -2.2 (x86 closed loop incl. longcontrol, set 0 at 60-100 km/h: -0.93 at 60, -1.44 at 50, -1.98 at 40, peak
  -2.19 at 30, -2.07 at 20, -1.45 at 10, -1.06 at 6, -0.27 sent at the stop; hold to 10 km/h at -2.2 gave -2.00 at 20
  and -1.37 at 10). The look-ahead target above lags a schedule this firm
  (x86 sim: -0.72 at 60, -2.52 at 20 instead of -2.2) and the MPC's cruise floor (CRUISE_MIN_ACCEL -1.2) caps it, so for
  set 0 the cruise term gets the schedule itself: profile() integrates it from the MPC's current speed over the MPC
  horizon and the MPC follows that speed curve (its cruise "obstacle") in place of the stock -1.2 straight line. Every
  other case keeps the stock line (profile is None; output identical).
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
SET0_TARGET_MAX = 1.0 / 3.6                              # m/s: a set speed below this is "set to 0"
SET0_DECEL_BP = [0., 6.0 / 3.6, 30.0 / 3.6, 60.0 / 3.6]    # m/s (0, 6, 30, 60 km/h)
SET0_DECEL_V = [-0.30, -2.15, -2.15, -1.00]              # m/s^2: end, firm hold, -1.0 at 60 (see above)
SET0_BLEND_BP = [60.0 / 3.6, 70.0 / 3.6]                 # m/s: -1.0 at 60 km/h -> the eco value (coasting) at 70
PROFILE_DT = 0.05                                        # s: integration step of the set-0 speed curve
PROFILE_JERK = 1.0                                       # m/s^3: the set-0 curve grows from the current accel no faster
PROFILE_TABLE_N = 161                                    # speeds 0 .. v0 at which the set-0 schedule is tabled per call


class CruiseDecel:
  def __init__(self):
    self.latched: float | None = None
    self.latched_target: float | None = None
    self.profile = None  # set 0: callable(v0, a0, t_idxs) -> the MPC's cruise speed curve; None: stock
    self._coast_now: float | None = None

  def _reset(self):
    self.latched, self.latched_target = None, None

  @staticmethod
  def _coast(accel_coast: float | None) -> float | None:
    if accel_coast is not None and np.isfinite(accel_coast) and accel_coast < 0.0:
      return float(np.clip(accel_coast, COAST_MIN, COAST_MAX))
    return None

  @staticmethod
  def _set0_decel(v: np.ndarray, coast: float | None) -> np.ndarray:
    """set-0 decel (m/s^2) at the speeds v (m/s, array): the eco line (all modes, as for every target below
    LOW_TARGET_MAX) blended into the SET0 schedule below 70 km/h, then the 80 km/h-and-up throttle easing."""
    d = np.interp(v, ECO_DECEL_BP, ECO_DECEL_V)
    if coast is not None:
      w = np.interp(v, ECO_COAST_BLEND_BP, [0.0, 1.0])
      low = np.interp(np.minimum(v, ECO_COAST_BLEND_BP[0]), ECO_DECEL_BP, ECO_DECEL_V)
      d = (1.0 - w) * low + w * min(coast, ECO_COAST_FLAT)  # set speed 0 <= 30 km/h: never gentler than the flat
    w0 = np.interp(v, SET0_BLEND_BP, [0.0, 1.0])
    d = (1.0 - w0) * np.interp(v, SET0_DECEL_BP, SET0_DECEL_V) + w0 * d
    high = HIGH_FRAC * (coast if coast is not None else np.interp(v, ECO_DECEL_BP, ECO_DECEL_V))
    w_high = np.interp(v, HIGH_BP, [0.0, 1.0])
    return (1.0 - w_high) * d + w_high * high  # no gap / TAPER_T: the schedule's own end line (-> -0.30) is the landing

  def decel(self, drive_mode: int, v_ego: float, v_target: float, accel_coast: float | None) -> float:
    gap = v_target - v_ego  # < 0
    coast = self._coast(accel_coast)
    if v_target < SET0_TARGET_MAX:
      return float(self._set0_decel(np.array([v_ego]), coast)[0])

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
    self.profile = None
    if bypass or drive_mode not in (ECO, NORMAL, SPORT) or not np.isfinite(v_cruise) or v_cruise < 0.0 or v_cruise >= v_ego:
      self._reset()
      return v_cruise
    if v_cruise < SET0_TARGET_MAX:
      self._coast_now = self._coast(accel_coast)
      self.profile = self._set0_profile  # the MPC follows the owner's set-0 curve
      return v_cruise
    return float(max(v_cruise, v_ego + self.decel(drive_mode, v_ego, v_cruise, accel_coast) * LOOKAHEAD_T))  # never below the set speed

  def _set0_profile(self, v0: float, a0: float, t_idxs: np.ndarray) -> np.ndarray:
    """the MPC's cruise speeds at t_idxs (s) for set 0: the set-0 schedule integrated from the MPC's current speed v0,
    growing from its current accel a0 no faster than PROFILE_JERK (smooth onset). Each value is the mean speed over the
    interval before it: the MPC builds the cruise position as cumsum(T_DIFFS * v), which with end-of-interval speeds
    puts a decelerating target too close (x86 sim: -2.51 instead of -2.2). The schedule is tabled once per call
    (vectorized) so the step loop is plain float math (~0.2 ms on x86)."""
    v0 = max(float(v0), 0.0)
    step = max(v0, 0.1) / (PROFILE_TABLE_N - 1)
    table = self._set0_decel(np.arange(PROFILE_TABLE_N) * step, self._coast_now).tolist()
    n = int(np.ceil(t_idxs[-1] / PROFILE_DT)) + 1
    x = [0.0] * n
    v, d = v0, min(float(a0), 0.0)
    for k in range(1, n):
      v_prev = v
      if v > 0.0:
        f = v / step
        i = min(int(f), PROFILE_TABLE_N - 2)
        sched = table[i] + (table[i + 1] - table[i]) * (f - i)
        d = max(sched, d - PROFILE_JERK * PROFILE_DT)
        v = max(v + d * PROFILE_DT, 0.0)
      x[k] = x[k - 1] + 0.5 * (v_prev + v) * PROFILE_DT
    xs = np.interp(t_idxs, np.arange(n) * PROFILE_DT, x)
    dt = np.diff(t_idxs)
    return np.concatenate([[v0], np.diff(xs) / np.where(dt > 0.0, dt, 1.0)])
