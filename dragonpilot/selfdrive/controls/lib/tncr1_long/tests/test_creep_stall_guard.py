# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from dragonpilot.selfdrive.controls.lib.tncr1_long.creep_stall_guard import CreepStallGuard

V2 = 2.4 / 3.6


def run(g, n, accel, v, a_ego, standstill=False, active=True):
  out = accel
  for _ in range(n):
    out = g.update(accel, v, a_ego, standstill, active, -3.5)
  return out


def test_nothing_while_the_car_slows():
  assert run(CreepStallGuard(), 300, -0.57, V2, -0.44) == -0.57
  assert run(CreepStallGuard(), 300, -0.3, V2, 0.0) == -0.3        # light request: the end line is left alone
  assert run(CreepStallGuard(), 300, -0.57, 8 / 3.6, 0.0) == -0.57  # above 5 km/h
  assert run(CreepStallGuard(), 300, 0.5, V2, 0.5) == 0.5           # launch


def test_stall_builds_extra_slowly_and_capped():
  g = CreepStallGuard()
  assert run(g, 29, -0.57, V2, 0.0) == -0.57                 # not before 0.3 s
  out = run(g, 21, -0.57, V2, 0.0)                           # then 1.0 m/s^3: 0.2 s -> -0.2 more (+1 frame)
  assert -0.79 < out < -0.76
  assert abs(run(g, 200, -0.57, V2, 0.0) - (-0.97)) < 1e-6   # capped at +0.4


def test_kept_while_weak_then_fades():
  g = CreepStallGuard()
  run(g, 100, -0.57, V2, 0.0)
  assert abs(run(g, 50, -0.57, V2, -0.2) - (-0.97)) < 1e-6  # still weak: kept
  out = run(g, 20, -0.57, V2, -0.4)                          # slowing again: fades at 1.0 m/s^3
  assert abs(out - (-0.77)) < 1e-6
  assert run(g, 100, -0.57, V2, -0.4) == -0.57


def test_end_of_stop_untouched():
  g = CreepStallGuard()
  run(g, 100, -0.57, V2, 0.0)
  assert abs(run(g, 1, -0.5, 1.5 / 3.6, 0.0) - (-0.7)) < 0.011   # 1.5 km/h: half the extra
  assert run(g, 1, -0.4, 1.0 / 3.6, 0.0) == -0.4                 # 1 km/h and below: the plain end line
  assert run(g, 1, -0.4, 0.0, 0.0, standstill=True) == -0.4


def test_reset_when_not_active():
  g = CreepStallGuard()
  run(g, 100, -0.57, V2, 0.0)
  assert run(g, 1, 0.0, V2, 0.0, active=False) == 0.0
  assert g.extra == 0.0
