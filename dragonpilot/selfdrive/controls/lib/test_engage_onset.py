from types import SimpleNamespace

from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import longcontrol as lc

CP = CarInterface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2)
LIM = (-3.5, 2.0)


def cs(v=8.0, a=0.0, standstill=False):
  return SimpleNamespace(vEgo=v, aEgo=a, standstill=standstill, brakePressed=False, cruiseState=SimpleNamespace(standstill=False))


def run(a_target_fn, frames=150, off_frames=10, v=8.0):
  """off for a while, then engaged; returns the engaged outputs per frame (frame 0 = engagement)"""
  LC = lc.LongControl(CP)
  for _ in range(off_frames):
    LC.update(False, cs(v), 0.0, False, LIM)
  return [LC.update(True, cs(v), a_target_fn(k), False, LIM) for k in range(frames)]


def at(outs, t):
  return outs[round(t / DT_CTRL) - 1]


def test_first_tenth_of_a_second_is_soft():
  outs = run(lambda k: -1.0)
  assert at(outs, 0.1) > -0.16          # stock (carcontroller 4 m/s^3): -0.4
  assert -0.45 < at(outs, 0.2) < -0.35
  # rising jerk, never above the stock 4 m/s^3, never a step
  steps = [outs[0]] + [b - a for a, b in zip(outs, outs[1:])]
  assert all(-4.0 * DT_CTRL - 1e-9 <= s <= 1e-9 for s in steps[:30])
  # after the onset the planner's request is applied (carcontroller keeps its own 4 m/s^3)
  assert all(o == -1.0 for o in outs[40:])


def test_never_firmer_than_the_planner():
  for target in (-0.05, -0.3, -1.0, -2.5):
    for o in run(lambda k: target):
      assert o >= target - 1e-9


def test_light_request_untouched_after_reached():
  outs = run(lambda k: -0.1)
  assert all(o == -0.1 for o in outs[20:])


def test_only_after_engaging():
  # a brake that starts later than the window: unchanged
  outs = run(lambda k: 0.0 if k * DT_CTRL < 1.2 else -1.0, frames=200)
  assert outs[round(1.2 / DT_CTRL)] == -1.0
  # only once per engagement: a second brake inside the window after the first onset ended is unchanged
  outs = run(lambda k: -0.5 if k < 40 else (0.0 if k < 60 else -1.0), frames=100)
  assert outs[60] == -1.0


def test_standstill_resume_untouched():
  LC = lc.LongControl(CP)
  LC.update(False, cs(0.0, standstill=True), 0.0, False, LIM)
  out = LC.update(True, cs(0.0, standstill=True), -1.0, False, LIM)
  assert LC.onset_t is None
  assert out <= 0.0


def test_gas_override_release_untouched():
  LC = lc.LongControl(CP)
  LC.update(True, cs(), 0.0, False, LIM)
  for _ in range(200):
    LC.update(True, cs(), 0.0, False, LIM)
  gas = cs(); gas.gasPressed = True
  for _ in range(20):
    LC.update(False, gas, 0.0, False, LIM)  # controlsd: longActive False while the driver is on the gas
  assert LC.update(True, cs(), -1.0, False, LIM) == -1.0
