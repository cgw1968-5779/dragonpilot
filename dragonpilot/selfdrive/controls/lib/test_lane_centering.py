import itertools
import math
import unittest
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.constants import CV
from dragonpilot.selfdrive.controls.lib import lane_centering
from dragonpilot.selfdrive.controls.lib.lane_centering import (LaneCentering,
                                                                                     LANE_CENTER_DEADBAND,
                                                                                     LANE_CENTER_FADE_BP,
                                                                                     LANE_CENTER_MAX_CURV)

DT = 0.01


@pytest.fixture(autouse=True)
def _centre_on_the_lines(monkeypatch):
  # dppb: the module holds the Altis 5 cm right of the line centre (LINE_CENTRE_SHIFT); these tests check the loop
  # itself around the plain line centre
  monkeypatch.setattr(lane_centering, 'LINE_CENTRE_SHIFT', 0.0)


_FRAME = itertools.count()


def model(offset_left, heading_left=0.0, width=3.6, probs=(0.9, 0.9)):
  """laneLines for a car `offset_left` m LEFT of the lane centre, pointing `heading_left` rad LEFT (model y+ = right)."""
  x = np.arange(0.0, 30.0, 2.0)
  centre = offset_left + math.tan(heading_left) * x   # pointing left: the lane centre runs off to the right ahead
  ll = [SimpleNamespace(x=list(x), y=list(centre - width / 2 + 3.0)), SimpleNamespace(x=list(x), y=list(centre - width / 2)),
        SimpleNamespace(x=list(x), y=list(centre + width / 2)), SimpleNamespace(x=list(x), y=list(centre + width / 2 + 3.0))]
  # a unique frameId like a real modelV2 message: LaneCentering caches its lane fits per (id, frameId), and a
  # recycled id() of a freed test object must not hit a stale cache entry
  return SimpleNamespace(laneLines=ll, laneLineProbs=[0.1, probs[0], probs[1], 0.1], frameId=next(_FRAME))


class TestLaneCentering(unittest.TestCase):
  def test_sign_and_fade(self):
    v = 110 * CV.KPH_TO_MS
    lc = LaneCentering(DT)
    for _ in range(300):
      c = lc.update(model(0.3), v, True, False)
    self.assertGreater(c, 0.0)                       # car left of centre -> positive (right) curvature
    self.assertLessEqual(c, LANE_CENTER_MAX_CURV)
    lc2 = LaneCentering(DT)
    for _ in range(300):
      c2 = lc2.update(model(-0.3), v, True, False)
    self.assertAlmostEqual(c2, -c, places=6)         # symmetric
    lc3 = LaneCentering(DT)
    for _ in range(300):
      c3 = lc3.update(model(0.3), 60 * CV.KPH_TO_MS, True, False)
    self.assertEqual(c3, 0.0)                        # below the 70 km/h fade-in: no correction at all
    self.assertAlmostEqual(LANE_CENTER_FADE_BP[0], 70 * CV.KPH_TO_MS, delta=0.1)

  def test_deadband_and_lost_lines_wind_back(self):
    v = 110 * CV.KPH_TO_MS
    lc = LaneCentering(DT)
    for _ in range(300):
      c = lc.update(model(LANE_CENTER_DEADBAND * 0.5), v, True, False)
    self.assertAlmostEqual(c, 0.0, places=9)
    for _ in range(300):
      c = lc.update(model(0.4), v, True, False)
    self.assertGreater(c, 0.0)
    prev = c
    for _ in range(200):
      c = lc.update(model(0.4, probs=(0.2, 0.2)), v, True, False)   # lines lost
      self.assertLessEqual(c, prev + 1e-12)
      prev = c
    self.assertEqual(c, 0.0)
    self.assertFalse(lc.valid)

  def test_closed_loop_converges_at_highway_speed(self):
    """Pure-feedback bicycle model with 0.3 s steering lag (no model path helping): 0.4 m off at 110 km/h is back
    within 10 cm and the overshoot stays under 15 cm. On the car the model's own centring adds to this."""
    v = 110 * CV.KPH_TO_MS
    lc = LaneCentering(DT)
    # y: metres LEFT of centre; psi: heading LEFT of the lane (rad)
    y = 0.4
    psi = 0.0
    k_act = 0.0
    hist = []
    for _ in range(int(20 / DT)):
      k_cmd = lc.update(model(y, psi), v, True, False)
      k_act += (k_cmd - k_act) * DT / 0.3           # steering lag
      psi += -k_act * v * DT                        # curvature positive = right turns the heading to the right
      y += v * math.sin(psi) * DT                   # pointing left drifts the car left
      hist.append(y)
    self.assertLess(abs(hist[-1]), 0.10)
    self.assertGreater(min(hist), -0.15)


if __name__ == "__main__":
  unittest.main()


def test_no_position_feedback_during_a_lane_change():
  # dpcr 2026-11-05: the PD must not push toward the new lane's centre or wind up its integral while changing lanes
  from types import SimpleNamespace as N
  import numpy as np
  X = [0., 2., 4., 6., 8., 10., 12., 15., 18., 21., 25., 30., 35., 40., 45., 50., 55., 60., 70., 80.]
  def model(offset):
    c = np.array([offset for _ in X])
    left = N(x=X, y=list(c - 1.65)); right = N(x=X, y=list(c + 1.65))
    return N(laneLines=[left, left, right, right], laneLineProbs=[0.9] * 4, frameId=0, meta=N(laneChangeState=2))
  lc = LaneCentering(0.01)
  v = 100 / 3.6
  for _ in range(300):
    lc.update(model(0.4), v, True, False, 0.0, 0.0, False)
  assert lc.correction > 0.0 and lc.integral > 0.0
  for _ in range(500):
    lc.update(model(-0.8), v, True, False, 0.0, 0.0, True)  # changing lanes: the new lane's centre far away
  assert lc.integral == 0.0 and abs(lc.correction) < 1e-9
