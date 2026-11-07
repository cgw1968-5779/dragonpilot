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
"""
import numpy as np

HW_MIN = 2.2            # s
STOP_DISTANCE = 6.0     # m (as the MPC)
V_BP = [35.0 / 3.6, 40.0 / 3.6]   # m/s: from 40 km/h up (owner 2026-11-08: "40 km/h and above, all of it"; was 40-60)
V_W = [0.0, 1.0]
LEAD_MOVING_V = 3.0     # m/s
T_OPEN = 8.0            # s: open the shortfall over about this long
DV_OPEN_MAX = 1.0       # m/s
KP = 0.5                # m/s^2 per m/s
A_MAX = 0.4             # m/s^2: gentle
J_ON = 0.3              # m/s^3
J_OFF = 0.5             # m/s^3
GAS_BAND = 5.0          # m beyond the gap over which the gas cap fades out


class FollowGap:
  def __init__(self, dt: float):
    self.dt = dt
    self.brake = 0.0  # >= 0, the gentle brake now applied

  def update(self, v_ego: float, lead, a_plan: float, off: bool) -> float:
    w = float(np.interp(v_ego, V_BP, V_W))
    target = 0.0
    gas_cap = None
    if not off and w > 0.0 and lead is not None and lead.status and lead.vLead >= LEAD_MOVING_V:
      d_min = HW_MIN * v_ego + STOP_DISTANCE
      short = d_min - lead.dRel
      if short > -GAS_BAND:
        gas_cap = float(np.interp(-short, [0.0, GAS_BAND], [0.0, 2.0]))
      if short > 0.0:
        want = min(short / T_OPEN, DV_OPEN_MAX)
        opening = lead.vLead - v_ego
        target = w * float(np.clip(KP * (want - opening), 0.0, A_MAX))
    self.brake = float(np.clip(target, self.brake - J_OFF * self.dt, self.brake + J_ON * self.dt))
    out = a_plan
    if gas_cap is not None:
      out = min(out, w * gas_cap + (1.0 - w) * max(a_plan, 0.0)) if out > 0.0 else out
    if self.brake > 0.0:
      out = min(out, -self.brake)
    return out
