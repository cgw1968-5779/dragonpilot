# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from types import SimpleNamespace as NS

from opendbc.car.toyota.tncr1_send.engage_close_stop import EngageCloseStop, b_need
from opendbc.car.toyota.tncr1_send.end_force import end_force_cap
from opendbc.car.toyota.tncr1_send.brake_onset import BrakeOnsetShaper


def lead(d, v=0.0, status=True):
  return NS(dRel=d, vLead=v, status=status)


def test_close_engage_is_detected_far_is_not():
  c = EngageCloseStop()
  c.update(lead(4.2), 7.2 / 3.6, False, -1.2, 0.0)  # route 0000015a 18:51:24
  assert c.active
  c = EngageCloseStop()
  c.update(lead(10.0), 9.6 / 3.6, False, -0.9, 0.0)  # route 00000159 08:54:34: keeps the soft start
  assert not c.active
  c = EngageCloseStop()
  c.update(lead(4.2), 7.2 / 3.6, False, -1.2, None)  # not in the engage window
  assert not c.active


def test_cap_is_what_the_stop_needs_and_ends_at_standstill():
  c = EngageCloseStop()
  c.update(lead(4.2), 2.0, False, -1.23, 0.0)
  cap = c.cap(-1.23, lead(4.2), 2.0)
  assert abs(cap - -(b_need(2.0, 4.2) * 1.15 + 0.1)) < 1e-9 and cap > -1.23
  assert c.cap(-0.4, lead(4.2), 2.0) == -0.4               # lighter requests pass
  c.update(lead(2.0), 0.0, True, -0.3, 1.0)
  assert not c.active


def test_close_engage_builds_linearly_from_the_first_frame():
  c = EngageCloseStop()
  dt = 0.03
  s, s_close = BrakeOnsetShaper(dt, 4.0 / dt), BrakeOnsetShaper(dt, 4.0 / dt)
  a, a_close, t = 0.0, 0.0, 0.0
  while t < 0.5 - 1e-9:
    a += s.down_step(-1.2, a, v_ego=2.0, t_engaged=t)
    a_close += s_close.down_step(-1.2, a_close, v_ego=2.0, t_engaged=t, engage_j_down=c.j_down)
    t += dt
  assert a >= -0.26                    # far SET-: still the -0.25 soft start
  assert a_close <= -0.45              # close stop: ~1.0 m/s^3 from the first frame (-0.5 at 0.5 s)


def test_end_force_cap_below_2kph_only():
  assert end_force_cap(-0.48, -0.30, 1.0 / 3.6, False) == -0.30   # sent command never firmer than the request
  assert end_force_cap(-1.31, -1.09, 4.8 / 3.6, False) == -1.31   # above 2 km/h untouched
  assert end_force_cap(-1.5, -1.2, 0.0, True) == -1.5             # standstill hold untouched
  assert end_force_cap(-0.2, -0.3, 1.0 / 3.6, False) == -0.2


def test_tncr_cap_never_lighter_than_the_cross_end():
  # route 0000006b 09:06:07 (true): the cap fell to -0.23 at 2 km/h / -0.12 at 0.7 km/h and the Cross crept at 0.7 km/h
  # for ~1 s, then stopped with a nod. Now -0.50 down to 2 km/h, then a line to -0.35 at 0 km/h
  c = EngageCloseStop()
  c.update(lead(6.0), 8.6 / 3.6, False, -0.86, 0.0)   # the radar read 6.0-8.0 m in the first 0.5 s
  assert c.active
  assert abs(c.cap(-0.85, lead(3.6), 2.0 / 3.6) - -0.50) < 1e-9
  assert abs(c.cap(-0.55, lead(3.4), 1.0 / 3.6) - -0.425) < 1e-9
  assert abs(c.cap(-0.55, lead(3.1), 0.0) - -0.35) < 1e-9
  assert c.cap(-0.30, lead(3.4), 1.0 / 3.6) == -0.30         # a lighter request still passes
  assert abs(c.cap(-1.23, lead(4.2), 7.2 / 3.6) - -(b_need(7.2 / 3.6, 4.2) * 1.15 + 0.1)) < 1e-9   # needs more: unchanged
