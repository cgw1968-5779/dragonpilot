# dppb (owner 2026-11-13): below 30 km/h follow 1 m closer, except on a continuous slow-down from above 31 km/h;
# a stopped lead always gets the unchanged stop target, and every change of the offset is rate limited
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc as lm

DT = 0.05


def mpc():
  return SimpleNamespace(dt=DT, follow_offset=None, from_high=False, low=False, v_min=0.0)


def lead(v=5.0, status=True):
  return SimpleNamespace(status=status, dRel=10.0, vLead=v)


def step(m, ld, v_ego):
  return lm.LongitudinalMpc.follow_gap(m, ld, v_ego)


def run(m, vs, vl=5.0):
  return [step(m, lead(v=vl if vl is not None else max(v, 0.0)), v) for v in vs]


LOW = lm.LOW_CLOSER - lm.FOLLOW_EXTRA


def test_low_speed_following_is_1m_closer():
  m = mpc()
  out = run(m, [15 / 3.6] * 100)
  assert abs(out[-1] - LOW) < 1e-9 and abs(out[0] - LOW) < 1e-9   # a new lead: no ramp needed


def test_above_31_is_the_old_following():
  m = mpc()
  out = run(m, [40 / 3.6] * 100, vl=11.0)
  assert abs(out[-1] + lm.FOLLOW_EXTRA) < 1e-9


def test_slow_down_from_above_31_keeps_the_old_following():
  m = mpc()
  vs = list(np.linspace(50, 8, 200) / 3.6)
  out = run(m, vs, vl=None)
  assert not m.low and all(o <= 1e-12 for o in out)            # never closer than the old following
  assert abs(out[0] + lm.FOLLOW_EXTRA) < 1e-9


def test_slow_down_then_speeding_up_again_turns_low_on_smoothly():
  m = mpc()
  run(m, list(np.linspace(50, 10, 200) / 3.6), vl=None)
  assert not m.low
  out = run(m, list(np.linspace(10, 20, 100) / 3.6) + [20 / 3.6] * 60, vl=5.0)
  assert m.low and abs(out[-1] - LOW) < 1e-9
  assert np.max(np.abs(np.diff(out))) <= lm.FOLLOW_RATE * DT + 1e-12


def test_a_stop_ends_the_slow_down_and_a_stopped_lead_has_the_stop_target():
  m = mpc()
  run(m, list(np.linspace(50, 0, 300) / 3.6), vl=None)
  out = run(m, [0.0] * 100, vl=0.0)
  assert m.low and abs(out[-1]) < 1e-12
  out = run(m, list(np.linspace(0, 15, 100) / 3.6), vl=None)
  assert out[-1] > 0.0 and np.max(np.abs(np.diff(out))) <= lm.FOLLOW_RATE * DT + 1e-12


def test_speeding_past_31_returns_to_the_old_following_smoothly():
  m = mpc()
  run(m, [20 / 3.6] * 50)
  out = run(m, list(np.linspace(20, 40, 200) / 3.6), vl=11.0)
  assert not m.low and abs(out[-1] + lm.FOLLOW_EXTRA) < 1e-9
  assert np.max(np.abs(np.diff(out))) <= lm.FOLLOW_RATE * DT + 1e-12


def test_hysteresis_30_31():
  m = mpc()
  run(m, [20 / 3.6] * 10)
  run(m, [30.6 / 3.6] * 50)
  assert m.low


def test_no_lead_no_offset():
  m = mpc()
  assert step(m, lead(status=False), 5.0) == 0.0 and m.follow_offset is None


def test_a_slowing_lead_hands_back_the_stop_target_smoothly():
  m = mpc()
  run(m, [15 / 3.6] * 20)
  out = [step(m, lead(v=v), 15 / 3.6) for v in np.linspace(5.0, 0.0, 100)] + [step(m, lead(v=0.0), 0.0) for _ in range(60)]
  assert abs(out[-1]) < 1e-12 and np.max(np.abs(np.diff(out))) <= lm.FOLLOW_RATE * DT + 1e-12
