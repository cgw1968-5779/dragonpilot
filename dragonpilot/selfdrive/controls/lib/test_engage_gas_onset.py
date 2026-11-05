from types import SimpleNamespace

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import longcontrol as lc
from dragonpilot.selfdrive.controls.lib.test_soft_stop_end import CP, LIM


def cs(v_kph, gas=False):
  return SimpleNamespace(vEgo=v_kph / 3.6, aEgo=0.0, standstill=False, brakePressed=False, gasPressed=gas,
                         cruiseState=SimpleNamespace(standstill=False))


def engage(a_target, v_kph=63.0, n=100, gas_before=False):
  LC = lc.LongControl(CP)
  for _ in range(5):
    LC.update(False, cs(v_kph, gas_before), a_target, False, LIM)
  outs = []
  for k in range(n):
    outs.append(LC.update(True, cs(v_kph), a_target, False, LIM))
  return outs


def at(outs, t):
  return outs[int(round(t / DT_CTRL))]


def test_owner_curve_then_planner():
  outs = engage(1.0)
  assert at(outs, 0.1) <= 0.05 + 1e-6 and at(outs, 0.2) <= 0.2 + 1e-6 and at(outs, 0.3) <= 0.4 + 1e-6
  assert abs(at(outs, 0.1) - 0.05) < 0.02 and abs(at(outs, 0.3) - 0.4) < 0.03
  assert abs(outs[-1] - 1.0) < 0.05  # joined the planner


def test_route_e4_set_at_63():
  outs = engage(0.22)
  assert at(outs, 0.1) <= 0.05 + 1e-6 and max(outs) <= 0.22 + 1e-6
  assert abs(outs[-1] - 0.22) < 0.02


def test_not_below_30_or_after_gas_override_or_braking():
  assert engage(0.5, v_kph=25.0)[1] > 0.3
  assert engage(0.5, gas_before=True)[1] > 0.3
  assert engage(-0.5)[-1] < 0.0
