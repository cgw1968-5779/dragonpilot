import numpy as np

from dragonpilot.selfdrive.controls.lib import cruise_decel as cd

KPH = 1 / 3.6
FLAT_COAST = -0.30


def test_passthrough_when_not_slowing_or_bypassed():
  c = cd.CruiseDecel()
  assert c.update(0, 20.0, 25.0, FLAT_COAST, False) == 25.0   # set speed above: accelerate as before
  assert c.update(0, 20.0, 15.0, FLAT_COAST, True) == 15.0    # lead in charge / force decel / not engaged
  assert c.update(-1, 20.0, 15.0, FLAT_COAST, False) == 15.0  # no drive mode
  assert c.update(0, 20.0, float('inf'), FLAT_COAST, False) == float('inf')


def test_high_speed_eases_throttle_half_coast():
  c = cd.CruiseDecel()
  for mode in (cd.ECO, cd.NORMAL, cd.SPORT):
    d = c.decel(mode, 100 * KPH, 80 * KPH, FLAT_COAST)
    assert abs(d - cd.HIGH_FRAC * FLAT_COAST) < 1e-9
    c._reset()


def test_eco_schedule_grows_toward_30():
  c = cd.CruiseDecel()
  d70 = c.decel(cd.ECO, 70 * KPH, 20 * KPH, FLAT_COAST)
  d60 = c.decel(cd.ECO, 60 * KPH, 20 * KPH, FLAT_COAST)
  d45 = c.decel(cd.ECO, 45 * KPH, 20 * KPH, FLAT_COAST)
  d31 = c.decel(cd.ECO, 31 * KPH, 20 * KPH, FLAT_COAST)
  assert abs(d70 - FLAT_COAST) < 1e-3 and abs(d60 - -0.40) < 1e-3
  assert d70 > d60 > d45 > d31 >= -0.50 - 1e-9


def test_no_step_at_80():
  c = cd.CruiseDecel()
  v = np.arange(70, 90, 0.1) * KPH
  d = [c.decel(cd.ECO, x, 40 * KPH, FLAT_COAST) for x in v]
  assert max(abs(np.diff(d))) < 0.01


def test_lands_without_overshoot():
  c = cd.CruiseDecel()
  for v in np.arange(60.0, 50.0, -0.1):
    assert c.update(cd.ECO, v * KPH, 50 * KPH, FLAT_COAST, False) >= 50 * KPH - 1e-9


def test_normal_latches_one_decel():
  c = cd.CruiseDecel()
  d1 = c.decel(cd.NORMAL, 60 * KPH, 55 * KPH, FLAT_COAST)
  d2 = c.decel(cd.NORMAL, 58 * KPH, 55 * KPH, FLAT_COAST)
  assert d1 == cd.LATCH_MIN_DECEL[cd.NORMAL]
  assert d2 == max(d1, (55 - 58) * KPH / cd.TAPER_T)


def test_set_zero_schedule_owner_1109():
  # owner 2026-11-09: set 0 from ~70 km/h: -1.0 at 60 growing linearly to the firm hold at 30, held below 10, then the end
  c = cd.CruiseDecel()
  d = {k: c.decel(cd.ECO, k * KPH, 0.0, FLAT_COAST) for k in (70, 65, 60, 45, 30, 20, 10, 6, 3, 0.01)}
  assert abs(d[70] - FLAT_COAST) < 1e-3                       # coasting at 70 as before
  assert d[70] > d[65] > d[60] and abs(d[60] - -1.0) < 1e-9   # blends into -1.0 at 60
  assert abs(d[45] - (-1.0 + 0.5 * (cd.SET0_DECEL_V[2] - -1.0))) < 1e-9  # linear 60 -> 30
  assert d[30] == d[20] == d[10] == d[6] == cd.SET0_DECEL_V[1] >= -2.2
  assert d[6] < d[3] < d[0.01] and abs(d[0.01] - -0.30) < 0.01  # linear end to the Altis -0.30
  assert min(d.values()) >= -2.2


def test_set_zero_profile_follows_the_schedule_from_a_smooth_onset():
  c = cd.CruiseDecel()
  assert c.update(cd.ECO, 50 * KPH, 0.0, FLAT_COAST, False) == 0.0 and c.profile is not None
  t = np.array([0.0, 0.5, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0])
  v = c.profile(50 * KPH, 0.0, t)
  assert abs(v[0] - 50 * KPH) < 1e-9 and np.all(np.diff(v) <= 1e-9) and v[-1] < 0.1  # stopped at ~8 s (last value: mean over 8-10 s)
  # mean speed over 0-0.5 s with a 1 m/s^3 onset from 0: v0 - J t^2 / 6
  assert abs(v[1] - (50 * KPH - cd.PROFILE_JERK * 0.25 / 6)) < 0.01
  # a lowered (non-zero) set speed, a bypass or a set speed above: the stock MPC line (profile None)
  for args in ((cd.ECO, 50 * KPH, 20 * KPH, FLAT_COAST, False), (cd.ECO, 50 * KPH, 0.0, FLAT_COAST, True),
               (cd.ECO, 20 * KPH, 30 * KPH, FLAT_COAST, False)):
    c.update(*args)
    assert c.profile is None


def test_low_target_uses_the_eco_line_in_every_mode():
  for mode in (cd.NORMAL, cd.SPORT):
    c = cd.CruiseDecel()
    assert abs(c.decel(mode, 60 * KPH, 0.0, FLAT_COAST) - cd.CruiseDecel().decel(cd.ECO, 60 * KPH, 0.0, FLAT_COAST)) < 1e-9
    assert c.decel(mode, 60 * KPH, 30 * KPH, FLAT_COAST) <= cd.LATCH_MIN_DECEL[mode]  # 30 and up: latched as before


def test_set_zero_lands_at_zero():
  c = cd.CruiseDecel()
  for v in np.arange(3.0, 0.0, -0.05):
    out = c.update(cd.ECO, v * KPH, 0.0, FLAT_COAST, False)
    assert 0.0 <= out < v * KPH
