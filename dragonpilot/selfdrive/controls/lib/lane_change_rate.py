"""
dragonpilot (dpcr, Corolla Cross): ease the commanded curvature into a lane change (ported from tncr1 2218295ad2,
owner 2026-11-08: "bring over tncr1's smooth lane change").

tncr18 / Corolla Cross feedback 2026-09-23: the steering snapped over as a lane change began. During
laneChangeStarting the commanded curvature may move away from the previous command at most RATE (1/m per second),
which ramps with the time in the lane change - gentle for the first second, then opening up so the move still
completes - and with speed (2026-09-24, route 00000042: at 70-80 km/h the gentle ramp left too little steering to
finish the change): <= 50 km/h the flat 0.004, >= 100 km/h 0.001 (t <= 1 s) -> 0.003 (t >= 3 s), each ramp point
interpolated linearly with speed in between. Outside laneChangeStarting nothing changes.
"""
import numpy as np

from cereal import log
from openpilot.common.constants import CV

T = (1.0, 3.0)                                   # s since laneChangeStarting began
V_BP = (50. * CV.KPH_TO_MS, 100. * CV.KPH_TO_MS)  # m/s
RATE_T1 = (0.004, 0.001)                         # 1/m per second at t <= 1 s, low / high speed
RATE_T3 = (0.004, 0.003)                         # 1/m per second at t >= 3 s, low / high speed
STARTING = log.LaneChangeState.laneChangeStarting


def start_rate(t: float, v_ego: float) -> float:
  r1 = float(np.interp(v_ego, V_BP, RATE_T1))
  r3 = float(np.interp(v_ego, V_BP, RATE_T3))
  return float(np.interp(t, T, (r1, r3)))


class LaneChangeRate:
  def __init__(self, dt: float):
    self.dt = dt
    self.t = 0.0

  def update(self, new_curvature: float, prev_curvature: float, lane_change_state, v_ego: float) -> float:
    if lane_change_state == STARTING:
      self.t += self.dt
      d = start_rate(self.t, v_ego) * self.dt
      return min(max(new_curvature, prev_curvature - d), prev_curvature + d)
    self.t = 0.0
    return new_curvature
