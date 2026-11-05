from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import longcontrol as lc
from dragonpilot.selfdrive.controls.lib.test_soft_stop_end import CP, LIM, cs, drive


def test_no_release_from_1_kmh_to_the_stop():
  # the planner lets go at the very end (as on route e0: -0.04 just before the stop): the command stays put
  outs = [(v, o) for v, o in drive(lambda v: -0.9 if v > 0.5 else -0.04, a_car=-0.9) if v > 0]
  last = [o for v, o in outs if v * 3.6 <= 1.0]  # owner: 1 -> 0 km/h constant
  assert last and all(abs(o - lc.SOFT_END_ACCEL) < 1e-6 for o in last)
  for a, b in zip([o for v, o in outs], [o for v, o in outs][1:]):
    assert b <= a + lc.SOFT_END_MAX_RATE * DT_CTRL + 1e-9  # never a sudden release


def test_lighter_command_held_never_firmed():
  # a stop already lighter than the end value: held where it is, never raised to -0.34
  outs = [(v, o) for v, o in drive(lambda v: -0.25 if v > 0.2 else 0.0, a_car=-0.3) if v > 0]
  last = [o for v, o in outs if v * 3.6 <= lc.END_HOLD_V]
  assert all(abs(o - -0.25) < 1e-9 for o in last)


def test_released_when_planner_wants_to_move():
  LC = lc.LongControl(CP)
  LC.engage_t = 10.0
  for _ in range(100):
    LC.update(True, cs(0.2, -0.3), -0.5, False, LIM)
  out = 0.0
  for _ in range(100):
    out = LC.update(True, cs(0.2, 0.0), 0.5, False, LIM)
  assert out > 0.0


def test_nothing_above_end_hold_v():
  LC = lc.LongControl(CP)
  LC.engage_t = 10.0
  LC.update(True, cs(2.0, -0.9), -0.9, False, LIM)
  assert LC.end_hold is None
