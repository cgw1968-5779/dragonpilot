from types import SimpleNamespace as N

import numpy as np

from dragonpilot.selfdrive.controls.lib import follow_gap as G

DT = 0.05
KPH = 1 / 3.6


def lead(d, v_kph, status=True):
  return N(status=status, dRel=d, vLead=v_kph * KPH)


def test_f8_equal_speed_short_gap_brakes_gently():
  # 40 km/h: 12 m, lead 40 km/h -> wanted gap 1.3 x 11.1 + 6 = 20.4 m
  fg = G.FollowGap(DT)
  outs = [fg.update(40 * KPH, lead(12.0, 40), 0.05, False) for _ in range(60)]
  assert outs[0] <= 0.0                                     # no gas toward it
  assert -G.A_MAX - 1e-9 <= min(outs) < -0.3                # a gentle brake builds up
  assert np.min(np.diff(outs)) >= -G.J_ON * DT - 1e-9       # softly


def test_opening_already_or_far_enough_needs_nothing():
  fg = G.FollowGap(DT)
  assert fg.update(58 * KPH, lead(58.0, 58), 0.2, False) == 0.2           # beyond the gap + band
  fg = G.FollowGap(DT)
  for _ in range(40):
    out = fg.update(58 * KPH, lead(36.0, 63), 0.1, False)               # lead pulling away 1.4 m/s
  assert out >= -1e-9


def test_from_40_kmh_up():
  fg = G.FollowGap(DT)
  assert fg.update(30 * KPH, lead(10.0, 30), 0.3, False) == 0.3            # below 35 km/h: untouched
  fg = G.FollowGap(DT)
  outs = [fg.update(40 * KPH, lead(0.8 * 40 * KPH + 6, 40), 0.2, False) for _ in range(40)]
  assert outs[0] <= 0.0 and min(outs) < -0.2                               # 40 km/h full
  fg = G.FollowGap(DT)
  outs = [fg.update(45 * KPH, lead(0.8 * 45 * KPH + 6, 45), 0.2, False) for _ in range(40)]
  assert -0.21 < min(outs) < 0.0                                            # 45 km/h half
  for v in (50.0, 70.0, 100.0):                                             # 50 km/h and up: zero (owner ~15:00)
    fg = G.FollowGap(DT)
    assert all(fg.update(v * KPH, lead(0.8 * v * KPH + 6, v), 0.2, False) == 0.2 for _ in range(40))
  fg = G.FollowGap(DT)
  assert fg.update(45 * KPH, lead(30.0, 0.0), 0.3, False) == 0.3           # stopped lead: not this module


def test_plan_firmer_wins_and_disengaged_untouched():
  fg = G.FollowGap(DT)
  assert fg.update(50 * KPH, lead(20.0, 50), -1.5, False) == -1.5
  assert G.FollowGap(DT).update(50 * KPH, lead(20.0, 50), 0.4, True) == 0.4


def test_small_shortfall_no_full_brake():
  # 13:12:52 scaled to 58 km/h: 2 % short, lead 2.3 km/h slower -> a light brake, not the full A_MAX
  fg = G.FollowGap(DT)
  v = 40 * KPH; d = (G.HW_MIN * v + G.STOP_DISTANCE) * 0.98
  outs = [fg.update(v, lead(d, 37.7), 0.2, False) for _ in range(60)]
  assert -0.2 < min(outs) <= 0.0


def test_gap_opening_eases_brake_at_once():
  # 13:04:47: 38 km/h, 11.2 m, after braking for a cut-in: brake held, then the lead runs 2 km/h faster
  fg = G.FollowGap(DT)
  for _ in range(60):
    fg.update(40 * KPH, lead(11.0, 40), -0.1, False)
  assert fg.brake > 0.35
  outs = [fg.update(38 * KPH, lead(11.2, 40.1), -0.1, False) for _ in range(30)]
  assert outs[-1] >= -0.1 - 1e-9                                            # back to the plan within 1.5 s
  assert np.max(np.diff(outs)) <= G.J_OFF * DT + 1e-9                         # smoothly


def test_gas_follows_lead_gently_near_gap():
  # at the wanted gap with the lead 3 km/h faster: some gas, not the full cruise gas
  fg = G.FollowGap(DT)
  v = 40 * KPH; d = G.HW_MIN * v + G.STOP_DISTANCE + 1.0
  out = fg.update(v, lead(d, 43), 0.3, False)
  assert 0.0 < out < 0.3



def test_short_gap_opening_follows_with_gas():
  # fc 13:51:43 scaled to 1.3 s: 40 km/h, 15 m (wanted 20.4 m), lead pulling away at 0.7 m/s -> some gas, not 0
  fg = G.FollowGap(DT)
  out = fg.update(40 * KPH, lead(15.0, 42.5), 0.7, False)
  assert 0.0 < out < 0.3
  fg = G.FollowGap(DT)
  assert fg.update(40 * KPH, lead(15.0, 40.8), 0.7, False) <= 0.0     # opening < 0.3 m/s: still no gas
