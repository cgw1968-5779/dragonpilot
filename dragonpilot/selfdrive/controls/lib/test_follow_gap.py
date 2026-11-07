from types import SimpleNamespace as N

import numpy as np

from dragonpilot.selfdrive.controls.lib import follow_gap as G

DT = 0.05
KPH = 1 / 3.6


def lead(d, v_kph, status=True):
  return N(status=status, dRel=d, vLead=v_kph * KPH)


def test_f8_equal_speed_short_gap_brakes_gently():
  # f8 00:39 -12 s: 58 km/h, 30.7 m, lead 58 km/h -> wanted gap 2.2 x 16.1 + 6 = 41.4 m
  fg = G.FollowGap(DT)
  outs = [fg.update(58 * KPH, lead(30.7, 58), 0.05, False) for _ in range(60)]
  assert outs[0] <= 0.0                                     # no gas toward it
  assert -G.A_MAX - 1e-9 <= min(outs) < -0.3                # a gentle brake builds up
  assert np.min(np.diff(outs)) >= -G.J_ON * DT - 1e-9       # softly


def test_opening_already_or_far_enough_needs_nothing():
  fg = G.FollowGap(DT)
  assert fg.update(58 * KPH, lead(45.0, 58), 0.2, False) == 0.2           # beyond the gap + band
  fg = G.FollowGap(DT)
  for _ in range(40):
    out = fg.update(58 * KPH, lead(36.0, 63), 0.1, False)               # lead pulling away 1.4 m/s
  assert out >= -1e-9


def test_band_40_60_only():
  for v in (30.0, 70.0):
    fg = G.FollowGap(DT)
    assert fg.update(v * KPH, lead(10.0, v), 0.3, False) == 0.3
  fg = G.FollowGap(DT)
  assert fg.update(58 * KPH, lead(30.0, 0.0), 0.3, False) == 0.3           # stopped lead: not this module


def test_plan_firmer_wins_and_disengaged_untouched():
  fg = G.FollowGap(DT)
  assert fg.update(50 * KPH, lead(20.0, 50), -1.5, False) == -1.5
  assert G.FollowGap(DT).update(50 * KPH, lead(20.0, 50), 0.4, True) == 0.4
