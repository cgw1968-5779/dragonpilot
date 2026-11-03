from types import SimpleNamespace

import numpy as np

from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import longcontrol as lc

CP = CarInterface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2)
LIM = (-3.5, 2.0)


def cs(v, a, standstill=False):
  return SimpleNamespace(vEgo=v, aEgo=a, standstill=standstill, brakePressed=False, gasPressed=False,
                         cruiseState=SimpleNamespace(standstill=False))


def lead(d, v=0.0, status=True):
  return SimpleNamespace(status=status, dRel=d, vLead=v, aLeadK=0.0)


def make():
  LC = lc.LongControl(CP)
  LC.engage_t = 10.0  # engaged long ago: the engage onset is not part of these stops
  return LC


def run(LC, v_kmh, a_target, lead_obj, n=200):
  LC.lead = lead_obj
  return [LC.update(True, cs(v_kmh / 3.6, a_target), a_target, False, LIM) for _ in range(n)]


def level(v_kmh):
  return float(np.interp(v_kmh, lc.STOP_LEVEL_V, lc.STOP_LEVEL_A))


def test_eases_to_tncr1_level_with_room():
  LC = make()
  outs = run(LC, 5.0, -1.0, lead(12.0))
  assert abs(outs[-1] - level(5.0)) < 1e-6
  for a, b in zip(outs, outs[1:]):
    assert b - a <= lc.STOP_LEVEL_RATE * DT_CTRL + 1e-9  # linear easing, no step
    assert b >= -1.0 - 1e-9                               # never firmer than the planner


def test_short_room_eases_only_as_far_as_the_gap_allows():
  LC = make()
  v = 5.0 / 3.6
  d = 4.2  # room = 4.2 - 0.69 - 3.3 = 0.2 m: needs ~4.8 m/s^2 >= STOP_LEVEL_MAX_B -> left to the planner
  outs = run(LC, 5.0, -1.0, lead(d))
  assert all(abs(o - -1.0) < 1e-9 for o in outs)  # the planner's -1.0 stays
  d = 5.0  # room 1.0 m: needs ~0.96 m/s^2 (~-0.81 sent): eases, but not all the way to the level
  outs = run(make(), 5.0, -1.0, lead(d))
  room = d - v * lc.STOP_LEVEL_LAG - lc.STOP_LEVEL_GAP
  assert room < lc._level_distance(5.0)
  assert -1.0 < outs[-1] < level(5.0)


def test_no_lead_moving_lead_or_lost_radar_untouched():
  for lo in (None, lead(12.0, v=3.0), lead(12.0, status=False)):
    outs = run(make(), 5.0, -1.0, lo)
    assert all(abs(o - -1.0) < 1e-9 for o in outs)


def test_urgent_untouched():
  outs = run(make(), 5.0, -2.0, lead(12.0))
  assert all(abs(o - -2.0) < 1e-9 for o in outs)


def test_lighter_plan_untouched():
  outs = run(make(), 5.0, -0.3, lead(12.0))
  assert all(abs(o - -0.3) < 1e-9 for o in outs)


def test_above_arm_speed_untouched():
  outs = run(make(), lc.STOP_LEVEL_V_ARM + 1.0, -1.0, lead(30.0))
  assert all(abs(o - -1.0) < 1e-9 for o in outs)


def test_accel_and_reaccel_untouched():
  for a_t in (0.5, 1.2):  # launch / re-acceleration behind a stopped-then-moving lead
    outs = run(make(), 5.0, a_t, lead(12.0))
    assert all(abs(o - a_t) < 1e-9 for o in outs)


def test_planner_firming_followed_at_once():
  LC = make()
  run(LC, 5.0, -1.0, lead(12.0))  # eased to the level (room allows it, so a firmer plan alone stays at the level)
  LC.lead = lead(3.5)              # the lead is suddenly close: the planner's request wins at once, no rate limit
  out = LC.update(True, cs(5.0 / 3.6, -0.6), -1.4, False, LIM)
  assert abs(out - -1.4) < 1e-9


def test_level_line_never_firmer_than_end_value_at_zero():
  assert lc.STOP_LEVEL_A[0] == lc.SOFT_END_ACCEL
  assert all(a < 0 for a in lc.STOP_LEVEL_A)
