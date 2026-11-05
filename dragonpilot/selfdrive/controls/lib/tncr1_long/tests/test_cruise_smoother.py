# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
import math

import dragonpilot.selfdrive.controls.lib.tncr1_long.cruise_smoother as CSM
from dragonpilot.selfdrive.controls.lib.tncr1_long.cruise_smoother import CruiseSmoother

V = 90 / 3.6


def test_steady_ripple_is_flattened():
  s = CruiseSmoother()
  outs = [s.update(0.02 * math.sin(2 * math.pi * 2.0 * i * 0.01), V, False, True, False) for i in range(600)]  # +-0.02 at 2 Hz
  assert s.active and max(abs(o) for o in outs[200:]) < 0.004


def test_a_lead_or_a_jump_passes_straight_through():
  s = CruiseSmoother()
  for _ in range(100):
    s.update(0.0, V, False, True, False)
  assert s.update(0.25, V, False, True, False) == 0.25 and not s.active   # set-speed change: a 0.25 jump
  assert s.update(-0.5, V, True, True, False) == -0.5 and not s.active    # a lead
  assert s.update(0.1, 20 / 3.6, False, True, False) == 0.1               # below V_MIN
  assert s.update(0.0, V, False, True, True) == 0.0                       # stopping


def test_slow_drift_follows_with_a_lag_not_a_step():
  s = CruiseSmoother()
  for _ in range(100):
    s.update(0.0, V, False, True, False)
  out = [s.update(0.1, V, False, True, False) for _ in range(300)]        # +0.1 step (within JUMP): filtered
  assert 0.05 < out[149] < 0.07 and out[-1] > 0.08                        # ~63 % after one TAU


def test_switch_off_is_stock(monkeypatch):
  monkeypatch.setattr(CSM, "ENABLED", False)
  s = CruiseSmoother()
  assert all(s.update(0.02, V, False, True, False) == 0.02 for _ in range(50)) and not s.active
