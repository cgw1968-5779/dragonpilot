"""
dragonpilot (dppb, Corolla Altis): a very small relief when the Altis brakes harder than asked at 28-12 km/h.

Owner 2026-11-08: "normal mid/end braking on dppb is good; only when a lead slows from about 40-30 km/h, the brake at
around 20-15 km/h is a little too heavy. Without touching normal mid/end braking, make that slightly heavy brake just
a little lighter - almost the smallest step."

dppb rlogs 11-04..11-08 (engaged, moving lead slowing): at 25-16 km/h the car delivers 0.4-0.7 m/s^2 MORE than the
command (11-04 13:19:32 cmd -1.88 / aEgo -2.47, 13:15:07 -2.18 / -2.72); the TSS2 PID only integrates (ki 0.25 above
18 km/h), too slowly to catch it. Normal stops overshoot <= 0.3 there or not at all.

So the existing send-side loop gets a small proportional term with no memory (gone the moment the overshoot is):
relief = KP x (overshoot - DEADBAND), only while braking firmer than CMD_MAX, faded in 12-15 km/h and out 22-28 km/h.
Overshoot = command - filtered aEgo (the controller's own aEgo filter). It only ever makes the command lighter by at
most MAX_RELIEF, and only while the car is ALREADY decelerating more than the planner asked (by > DEADBAND), so the
delivered decel stays at least the planner's. Replay over 11 dppb drives: 7 events, max 0.16 (13:12 / 13:15 / 13:19
/ 13:24 0.10-0.16); normal stops 0-0.04.

Owner 2026-11-08 after driving it: "20-15 a little lighter worked, almost perfect - try one more tiny step lighter".
KP 0.4 -> 0.5 (deadband, window, cap and rate unchanged). Same replay: 13:12:43 0.16 -> 0.20 (the cap), 13:15:07 0.10 ->
0.12, 13:19:32 0.11 -> 0.14, 13:24:42 0.12 -> 0.15, 13:26:13 0.08 -> 0.10; normal stops 11-05 09:22:55 / 11-08 08:32:50
stay 0.04, the rest 0.01-0.06. relief = 0.5 x (overshoot - 0.3) is always smaller than the overshoot, so the car still
decelerates at least as much as the planner asked.
"""
import numpy as np

KP = 0.5           # 2026-11-08: was 0.4 (owner: one more tiny step lighter at 20-15 km/h)
DEADBAND = 0.3      # m/s^2: overshoot below this is untouched (normal stops)
CMD_MAX = -0.5      # m/s^2: only while the command brakes firmer than this
MAX_RELIEF = 0.2    # m/s^2
RATE = 0.5          # m/s^3: the relief itself moves at most this fast (aEgo noise must not be felt)
V_BP = [12. / 3.6, 15. / 3.6, 22. / 3.6, 28. / 3.6]   # m/s
V_W = [0.0, 1.0, 1.0, 0.0]


def relief_target(v_ego: float, accel_cmd: float, a_ego_filtered: float) -> float:
  if accel_cmd > CMD_MAX:
    return 0.0
  w = float(np.interp(v_ego, V_BP, V_W))
  if w == 0.0:
    return 0.0
  over = accel_cmd - a_ego_filtered   # > 0: the car decelerates more than asked
  return float(np.clip(KP * (over - DEADBAND) * w, 0.0, MAX_RELIEF))


class BrakeOvershootRelief:
  def __init__(self, dt: float):
    self.step = RATE * dt
    self.relief = 0.0

  def update(self, active: bool, v_ego: float, accel_cmd: float, a_ego_filtered: float) -> float:
    target = relief_target(v_ego, accel_cmd, a_ego_filtered) if active else 0.0
    if not active:
      self.relief = 0.0
    self.relief = float(np.clip(target, self.relief - self.step, self.relief + self.step))
    return self.relief
