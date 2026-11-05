# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from openpilot.common.realtime import DT_CTRL
from dragonpilot.selfdrive.controls.lib.tncr1_long.mid_speed_brake_add import MidSpeedBrakeAdd, ADD, RATE_IN, RATE_OUT

AMIN = -3.5


def run(m, accel, v, n, **kw):
  out = None
  for _ in range(n):
    out = m.update(accel, v, False, True, AMIN, **kw)
  return out


def test_full_add_in_band():
  for v_kph in (10, 20, 30):
    m = MidSpeedBrakeAdd()
    assert abs(run(m, -1.0, v_kph / 3.6, 200) - (-1.0 - ADD)) < 1e-6


def test_none_outside_band_and_light_touch():
  for v_kph in (3, 5, 6, 35, 60):
    assert run(MidSpeedBrakeAdd(), -1.0, v_kph / 3.6, 200) == -1.0
  assert run(MidSpeedBrakeAdd(), -0.25, 20 / 3.6, 200) == -0.25
  assert run(MidSpeedBrakeAdd(), 0.3, 20 / 3.6, 200) == 0.3


def test_ramp_rate_no_step():
  m = MidSpeedBrakeAdd()
  prev = -1.0
  for _ in range(100):
    out = m.update(-1.0, 20 / 3.6, False, True, AMIN)
    assert abs(out - prev) <= RATE_IN * DT_CTRL + 1e-9
    prev = out


def test_fades_out_before_the_end_during_a_stop():
  # stops from 30 km/h at -1.0 and a hard -2.5: the add is gone by 6 km/h and never jumps
  for req in (-1.0, -2.5):
    m = MidSpeedBrakeAdd()
    v, prev_add = 30 / 3.6, 0.0
    while v > 0.1:
      out = m.update(req, v, False, True, AMIN)
      add = req - out
      assert -RATE_OUT * DT_CTRL - 1e-9 <= add - prev_add <= RATE_IN * DT_CTRL + 1e-9
      if v < 6 / 3.6:
        assert add < 0.02
      if v < 5 / 3.6:
        assert add < 1e-6
      prev_add = add
      v += out * DT_CTRL


def test_hold_off_inactive_standstill_and_floor():
  assert run(MidSpeedBrakeAdd(), -1.0, 20 / 3.6, 200, hold_off=True) == -1.0
  m = MidSpeedBrakeAdd()
  run(m, -1.0, 20 / 3.6, 200)
  assert m.update(-1.0, 20 / 3.6, False, False, AMIN) == -1.0 and m.add == 0.0
  assert MidSpeedBrakeAdd().update(-1.0, 0.0, True, True, AMIN) == -1.0
  assert abs(run(MidSpeedBrakeAdd(), -3.45, 20 / 3.6, 200) - AMIN) < 1e-6
  assert run(MidSpeedBrakeAdd(), -3.8, 20 / 3.6, 200) == -3.8
