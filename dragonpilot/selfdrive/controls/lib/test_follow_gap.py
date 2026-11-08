from types import SimpleNamespace as N

import numpy as np

from dragonpilot.selfdrive.controls.lib import follow_gap as G

DT = 0.05
KPH = 1 / 3.6


def lead(d, v_kph, status=True):
  return N(status=status, dRel=d, vLead=v_kph * KPH)


def gap(v_kph):
  return G.HW_MIN * v_kph * KPH + G.STOP_DISTANCE


def test_gas_is_never_capped():
  # 00000103 16:12:29 / 16:17:16: 40 km/h, lead 20-25 m ahead -> the MPC's gas passes untouched
  for d, vl in ((20.0, 40), (24.6, 43.4), (8.0, 40), (gap(40) + 1.0, 43)):
    fg = G.FollowGap(DT)
    assert all(fg.update(40 * KPH, lead(d, vl), 0.6, False) == 0.6 for _ in range(40))


def test_equal_speed_or_opening_needs_no_brake():
  for vl in (40.0, 42.0):
    fg = G.FollowGap(DT)
    assert all(fg.update(40 * KPH, lead(10.0, vl), -0.05, False) == -0.05 for _ in range(60))


def test_closing_inside_gap_brakes_early_and_gently():
  fg = G.FollowGap(DT)
  outs = [fg.update(40 * KPH, lead(gap(40) * 0.75, 35), 0.3, False) for _ in range(60)]
  assert outs[0] <= 0.0                                    # no gas while closing inside the gap
  assert -G.A_MAX - 1e-9 <= min(outs) < -0.3                # gentle brake builds
  assert np.min(np.diff(outs[1:])) >= -G.J_ON * DT - 1e-9   # softly


def test_closing_beyond_gap_untouched():
  fg = G.FollowGap(DT)
  assert all(fg.update(40 * KPH, lead(gap(40) + 0.5, 30), 0.2, False) == 0.2 for _ in range(40))


def test_small_shortfall_light_brake():
  fg = G.FollowGap(DT)
  outs = [fg.update(40 * KPH, lead(gap(40) * 0.98, 37.7), 0.2, False) for _ in range(60)]
  assert -0.15 < min(outs) <= 0.0


def test_speed_window():
  for v, ok in ((30.0, False), (40.0, True), (45.0, True), (50.0, False), (70.0, False)):
    fg = G.FollowGap(DT)
    outs = [fg.update(v * KPH, lead(gap(v) * 0.7, v - 6), 0.2, False) for _ in range(40)]
    assert (min(outs) < 0.0) == ok, v
  fg = G.FollowGap(DT)
  assert fg.update(45 * KPH, lead(15.0, 0.0), 0.3, False) == 0.3          # stopped lead: not this module


def test_release_when_closing_stops():
  fg = G.FollowGap(DT)
  for _ in range(60):
    fg.update(40 * KPH, lead(gap(40) * 0.75, 35), -0.1, False)
  assert fg.brake > 0.35
  outs = [fg.update(38 * KPH, lead(gap(40) * 0.75, 38.5), -0.1, False) for _ in range(30)]
  assert outs[-1] == -0.1                                                   # back to the plan
  assert np.max(np.diff(outs)) <= G.J_OFF * DT + 1e-9                       # smoothly


def test_plan_firmer_wins_and_disengaged_untouched():
  fg = G.FollowGap(DT)
  assert fg.update(42 * KPH, lead(10.0, 30), -1.5, False) == -1.5
  assert G.FollowGap(DT).update(42 * KPH, lead(10.0, 30), 0.4, True) == 0.4
