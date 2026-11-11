"""
dragonpilot (dppb, Corolla Altis; owner 2026-11-11): slow a little for a sharp curve ahead - the smallest amount.

Route 000001a7 21:45:42-51: a left curve tightening to R 65 m was entered at 59 km/h with no slowing at all (dppb has
no curve speed control outside experimental mode); the model wanted 3.1-4.1 m/s^2 of lateral acceleration, the Altis
EPS tops out near 2.7, the torque sat at 1.0 and "Turn Exceeds Steering Limit" ended in a takeover. Owner: "curve
slowing, but only the smallest amount - I do not want to slow much".

The model's predicted path (orientationRate.z and velocity.x over the next PREVIEW_T s) gives the curvature at each
point ahead; the speed that keeps the lateral acceleration at LAT_MAX there, carried back to now with a gentle
PRE_DECEL, is the most the car should be doing now. Only a LOWER cruise target goes to the planner (in front of
cruise_decel, which then slows with its own gentle schedule, ~-0.3..-0.5 m/s^2), never below V_FLOOR, only from
V_ACTIVE up, and it rises again at RISE once the curve is passed. A lead, a stop, the set speed and everything else
are untouched; with a straight road ahead the target is the set speed itself (identical output).
"""
import numpy as np

LAT_MAX = 2.5            # m/s^2 at the sharpest point ahead (the owner's "smallest": 2.0 slowed more and more often)
PRE_DECEL = 0.5          # m/s^2: how the curve's speed is carried back to now
PREVIEW_T = 6.0          # s of the model's path used
V_ACTIVE = 40.0 / 3.6    # m/s: no curve target below this speed
V_FLOOR = 30.0 / 3.6     # m/s: never a target below this
RISE = 1.0               # m/s^2: how fast the target may rise again
K_MIN = 1e-4             # 1/m: below this a point is straight


class CurveSpeed:
  def __init__(self, dt: float):
    self.dt = dt
    self.v_target: float | None = None  # last curve target (None: inactive)

  def allowed(self, model_v2) -> float | None:
    """the highest speed now that keeps the curve ahead at LAT_MAX (None: no curve / no model)"""
    try:
      yaw = np.abs(np.asarray(model_v2.orientationRate.z, dtype=float))
      vel = np.asarray(model_v2.velocity.x, dtype=float)
      pos = np.asarray(model_v2.position.x, dtype=float)
      t = np.asarray(model_v2.position.t, dtype=float)
    except AttributeError:
      return None
    n = min(len(yaw), len(vel), len(pos), len(t))
    if n < 4:
      return None
    m = t[:n] <= PREVIEW_T
    k = yaw[:n][m] / np.maximum(vel[:n][m], 5.0)
    if not np.any(k > K_MIN):
      return None
    v_req = np.sqrt(LAT_MAX / np.maximum(k, K_MIN))
    return float(np.min(np.sqrt(v_req ** 2 + 2.0 * PRE_DECEL * np.maximum(pos[:n][m], 0.0))))

  def update(self, model_v2, v_ego: float, v_cruise: float) -> float:
    a = self.allowed(model_v2) if v_ego >= V_ACTIVE else None
    if a is None or a >= v_cruise:
      if self.v_target is not None:  # rise back to the set speed at RISE, then hand it back
        self.v_target += RISE * self.dt
        if self.v_target >= v_cruise:
          self.v_target = None
      return v_cruise if self.v_target is None else min(v_cruise, self.v_target)
    a = max(a, V_FLOOR)
    self.v_target = a if self.v_target is None or a < self.v_target else min(a, self.v_target + RISE * self.dt)
    return min(v_cruise, self.v_target)
