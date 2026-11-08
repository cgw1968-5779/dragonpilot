"""
dragonpilot (dpcr, Corolla Cross): open a short following gap early from 40 km/h up (owner 2026-11-08, after route 000000f8
00:39: "between 40 and 60 km/h, as soon as the following distance is too short, start braking gently to open it - even
when both cars run at the same speed, keep a safe distance early").

f8 00:39: after a lane change behind a car 10.8-14.7 m ahead the gap opened only to 29.4-31.6 m at 58 km/h (1.8-2.0 s;
the aggressive personality aims ~26 m) and stayed there at equal speeds for 11 s; the lead then braked ~-4.5 m/s^2 and
the driver had to take over at 13.1 m. Counterfactual: the old braking needed a ~2.4 s gap to avoid contact.

While a moving lead is closer than HW_MIN x v (+ STOP_DISTANCE) from 40 km/h up (faded in over 35-40 km/h; first 40-60,
owner later the same night: "it should be 40 km/h and above"):
  - no acceleration toward it (the plan's gas is capped, back to the plan over GAS_BAND metres beyond the gap);
  - a gentle brake that opens the gap at about the speed it is short by: the wanted opening speed is the shortfall /
    T_OPEN (<= DV_OPEN_MAX), and the decel is KP x (wanted - actual opening speed), at most A_MAX, built at <= J_ON and
    released at <= J_OFF (the plan's own firmer brake always wins: this is a min).
A lead pulling away fast enough needs nothing; a stopped lead is the stop target's (stop_approach.py).

Smooth following near the gap (owner 2026-11-08 13:11, route 000000fa: "at 100 km/h behind a lead the gas presses and
lets go, again and again"): at 92-100 km/h, 60-67 m behind a lead wandering +/-5 km/h, the plan flipped from +0.2..+0.35
gas to the full -0.40 brake within 4 s six times in two minutes. The wanted gap is 67 m there; a 1-4 m shortfall (2-6 %)
with the lead 2-3 km/h slower already asked for the full A_MAX, and the 5 m gas band (0.18 s at 100 km/h) let the full
cruise gas back in the moment the gap was reached, so the car overtook the lead's speed and braked again. Now:
  - the gain grows with how short the gap is: K_NEAR x KP right at the wanted gap, the full KP from F_FULL (20 %) short
    (a cut-in or the f8 gap, 25-30 % short, is unchanged);
  - the gas back toward the lead is the same law: it may close in no faster than the excess / T_OPEN, and the cap only
    fades out from half to all of a band of T_GAS_BAND x v (at least GAS_BAND);
Ease off at once while the gap already opens (owner 2026-11-08 13:05: "at mid/low speed, when the extra brake for a
cut-in makes the gap grow, ease the brake off smoothly right away"): 13:04:47 at 38 km/h the gap grew 0.5-0.6 m/s and
this brake still held -0.24..-0.30 for 2 s (wanted opening 1 m/s). The opening speed now counts G_OPEN x while it is
positive, so the brake is gone by about +0.3 m/s (1 km/h), released at J_OFF; at equal speeds it is unchanged.
Gas while a short gap already opens (owner 2026-11-08, route 000000fc 13:51: "the lead pulled away quickly and we
did not keep up"; chose this over raising the drive mode ceiling): at 44 km/h, 26 m (wanted 32.7 m) the lead ran away
at +0.5 m/s and accelerating, but the gas side wanted the full shortfall opening (0.78 m/s) first, so the plan held 0
for 3 s. The gas side now only wants DV_GAS_OPEN: once the gap opens faster than that, gas follows the lead in
proportion (kp x the excess); the brake side and the wanted gap are unchanged.
Recorded lead 13:11:25-13:13:25 through this module (plan = cruise gas): gas->brake flips within 4 s 6 -> 0, largest
2 s swing 0.60 -> 0.40, rms jerk 0.098 -> 0.054; the recorded drive itself had 6 / 0.58.

Rewrite 2026-11-08 16:13 (owner, route 00000103: "following still lags far behind ... following must be as tight as
before these changes; the brake should only start earlier at 40-50 km/h"): replayed, the gas cap held the plan at
~0 for 8.75 s in 10 min at 35-45 km/h with the lead 20-25 m ahead (it fades out only one band beyond the gap). The
module is now brake-only: no gas cap and no brake at equal speeds or while the gap opens - only a gentle brake
(KP x closing speed, gain from K_NEAR at the gap to full F_FULL short, <= A_MAX, J_ON / J_OFF) while the car closes
in on a moving lead inside HW_MIN x v + STOP_DISTANCE. The MPC's own firmer brake always wins (min).
"""
import numpy as np

HW_MIN = 1.45           # s total gap = the MPC's aggressive 1.25 s + 0.2 s (owner 2026-11-08 ~15:20: "add 0.2 s back";
                        # ~15:10 a total 0.5 s, shorter than the MPC's own gap, so it only acted on cut-ins;
                        # ~15:00 1.3 s, before 2.2 s)
STOP_DISTANCE = 6.0     # m (as the MPC)
# m/s: full at 40 km/h (faded in over 35-40), linearly out to zero at 50 km/h (owner 2026-11-08 ~15:00; ~14:30 it was
# zero at 55, 13:30 full 40-60 and zero at 70; before that 40 km/h and up)
V_BP = [35.0 / 3.6, 40.0 / 3.6, 50.0 / 3.6]
V_W = [0.0, 1.0, 0.0]
LEAD_MOVING_V = 3.0     # m/s
KP = 0.5                # m/s^2 per m/s
A_MAX = 0.4             # m/s^2: gentle
J_ON = 0.3              # m/s^3
J_OFF = 0.5             # m/s^3
F_FULL = 0.2            # shortfall / wanted gap from which the full KP applies
K_NEAR = 0.2            # KP fraction right at the wanted gap


class FollowGap:
  def __init__(self, dt: float):
    self.dt = dt
    self.brake = 0.0  # >= 0, the gentle brake now applied

  def update(self, v_ego: float, lead, a_plan: float, off: bool) -> float:
    """Brake earlier only (owner 2026-11-08 16:13, route 00000103): no gas cap - the gas is the MPC's own, so the
    car follows as tight as before; a gentle brake only while closing in on a moving lead inside HW_MIN x v + 6 m."""
    w = float(np.interp(v_ego, V_BP, V_W))
    target = 0.0
    if not off and w > 0.0 and lead is not None and lead.status and lead.vLead >= LEAD_MOVING_V:
      d_min = HW_MIN * v_ego + STOP_DISTANCE
      short = d_min - lead.dRel
      closing = v_ego - lead.vLead
      if short > 0.0 and closing > 0.0:
        kp = KP * float(np.interp(short / d_min, [0.0, F_FULL], [K_NEAR, 1.0]))
        target = w * float(np.clip(kp * closing, 0.0, A_MAX))
    self.brake = float(np.clip(target, self.brake - J_OFF * self.dt, self.brake + J_ON * self.dt))
    if self.brake > 0.0:
      return min(a_plan, -self.brake)
    return a_plan
