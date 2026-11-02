from types import SimpleNamespace

from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import longcontrol as lc

CP = CarInterface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2)
LIM = (-3.5, 2.0)


def cs(v, a, standstill=False):
  return SimpleNamespace(vEgo=v, aEgo=a, standstill=standstill, brakePressed=False, cruiseState=SimpleNamespace(standstill=False))


def drive(a_target_fn, v0=8.0, a_car=-0.9, steps=3000, should_stop_v=0.0):
  """open-loop: the car decelerates at a_car; returns (v, out) per frame until it stops"""
  LC = lc.LongControl(CP)
  v, outs = v0, []
  for k in range(steps):
    out = LC.update(True, cs(v, a_car if v > 0 else 0.0, standstill=v <= 0), a_target_fn(v), v < should_stop_v, LIM)
    outs.append((v, out))
    v = max(0.0, v + a_car * DT_CTRL)
  return outs


def test_far_from_stop_unchanged():
  for v, out in drive(lambda v: -0.9):
    if v / 0.9 > lc.SOFT_END_LAG + (lc.SOFT_END_ACCEL - -0.9) / lc.SOFT_END_MAX_RATE + 0.05:
      assert abs(out - -0.9) < 1e-9


def test_eases_to_end_value_and_never_firms_again():
  outs = [o for v, o in drive(lambda v: -0.9) if v > 0]
  assert abs(outs[-1] - lc.SOFT_END_ACCEL) < 1e-6
  for a, b in zip(outs, outs[1:]):
    assert b >= a - 1e-9  # never firmer frame to frame while moving


def test_release_rate_limited():
  outs = [o for v, o in drive(lambda v: -1.4, a_car=-1.4) if v > 0]
  for a, b in zip(outs, outs[1:]):
    assert b - a <= lc.SOFT_END_MAX_RATE * DT_CTRL + 1e-9


def test_urgent_not_eased():
  outs = [o for v, o in drive(lambda v: -2.0, a_car=-2.0) if v > 0]
  assert all(abs(o - -2.0) < 1e-9 for o in outs)


def test_light_planner_untouched():
  outs = [o for v, o in drive(lambda v: -0.2, a_car=-0.3) if v > 0]
  assert all(abs(o - -0.2) < 1e-9 for o in outs)


def test_reset_when_lead_moves_off_before_stopping():
  LC = lc.LongControl(CP)
  for _ in range(50):  # crawling, nearly stopped, braking -> easing latched
    LC.update(True, cs(0.3, -0.6), -0.8, False, LIM)
  assert LC.soft_end_frac < 1.0
  LC.update(True, cs(0.3, -0.1), 0.3, False, LIM)  # lead moved: planner no longer braking
  assert LC.soft_end_frac == 1.0
  out = LC.update(True, cs(2.5, -0.5), -1.0, False, LIM)  # lead stops again further on: full brake
  assert abs(out - -1.0) < 1e-9


def test_stopping_state_does_not_add_brake_before_standstill():
  LC = lc.LongControl(CP)
  LC.update(True, cs(0.5, -0.4), -0.4, False, LIM)
  first = LC.update(True, cs(0.2, -0.4), -0.4, True, LIM)  # stopping state, still rolling
  outs = [LC.update(True, cs(0.1, -0.4), -0.4, True, LIM) for _ in range(30)]
  assert all(o >= first - 1e-9 for o in outs)
  held = [LC.update(True, cs(0.0, 0.0, standstill=True), 0.0, True, LIM) for _ in range(100)]
  assert held[-1] < held[0]  # once stopped it firms toward stopAccel to hold the car
