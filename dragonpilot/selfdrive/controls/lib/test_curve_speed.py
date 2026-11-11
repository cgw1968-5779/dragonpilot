from types import SimpleNamespace as N

import numpy as np

from dragonpilot.selfdrive.controls.lib import curve_speed as CS

T = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0])


def model(v, k_at=None, k=0.0):
  """constant speed v; curvature k from time k_at on (None: straight)"""
  yaw = np.array([v * k if (k_at is not None and t >= k_at) else 0.0 for t in T])
  return N(orientationRate=N(z=list(yaw)), velocity=N(x=[v] * len(T)), position=N(x=list(v * T), t=list(T)))


def test_straight_is_untouched():
  c = CS.CurveSpeed(0.05)
  for _ in range(100):
    assert c.update(model(20.0), 20.0, 22.0) == 22.0


def test_sharp_curve_ahead_lowers_the_target_gently_and_rises_back():
  c = CS.CurveSpeed(0.05)
  v = 59 / 3.6
  out = c.update(model(v, 4.0, 1 / 65), v, 70 / 3.6)          # R 65 m 4 s ahead (route 1a7 21:45)
  v_req = np.sqrt(CS.LAT_MAX * 65)
  assert v_req < out < v                                       # lower, but the gentle carry-back leaves it above v_req
  low = out
  for _ in range(20):
    out = c.update(model(v), v, 70 / 3.6)                      # curve passed
  assert out > low and out <= low + CS.RISE * 20 * 0.05 + 1e-9  # rises at RISE
  for _ in range(400):
    out = c.update(model(v), v, 70 / 3.6)
  assert out == 70 / 3.6 and c.v_target is None


def test_floor_and_low_speed():
  c = CS.CurveSpeed(0.05)
  assert c.update(model(15.0, 0.0, 1 / 20), 15.0, 20.0) == CS.V_FLOOR   # never below the floor
  c = CS.CurveSpeed(0.05)
  assert c.update(model(10.0, 0.0, 1 / 20), 10.0, 20.0) == 20.0        # below V_ACTIVE: off
