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
    d = c.decel(mode, 105 * KPH, 80 * KPH, FLAT_COAST)
    assert abs(d - cd.HIGH_FRAC * FLAT_COAST) < 1e-9
    c._reset()


def test_eco_line_100_to_20():
  # owner 2026-11-05 night: below 100 km/h the decel rises slowly until 20 km/h
  c = cd.CruiseDecel()
  d = {k: c.decel(cd.ECO, k * KPH, 10 * KPH, FLAT_COAST) for k in (99, 90, 80, 70, 60, 50, 40, 30, 21)}
  assert abs(d[90] - -0.325) < 1e-3 and abs(d[60] - -0.40) < 1e-3 and abs(d[30] - -0.475) < 1e-3
  ks = sorted(d)
  for a, b in zip(ks, ks[1:]):
    assert d[a] < d[b]  # firmer at lower speed
  assert min(d.values()) >= -0.50 - 1e-9


def test_no_step_at_95_100():
  c = cd.CruiseDecel()
  v = np.arange(85, 110, 0.1) * KPH
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


def test_set_zero_firmest_at_30_to_20_then_linear_to_the_end():
  # owner 2026-11-05: set 0 from 35 km/h and up: firmest at 30-20 km/h (11-05 night: line ends at 20), then easing linearly
  c = cd.CruiseDecel()
  d = {k: c.decel(cd.ECO, k * KPH, 0.0, FLAT_COAST) for k in (60, 45, 35, 30, 25, 20, 15, 10, 5)}
  assert abs(d[30] - -0.475) < 1e-3 and abs(d[20] - -0.50) < 1e-3
  assert d[60] > d[45] > d[35] > d[30] > d[25] > d[20]
  assert d[20] < d[15] < d[10] < d[5] < cd.ECO_DECEL_V[0] + 1e-9
  assert abs((d[15] - d[10]) - (d[10] - d[5])) < 1e-9  # linear
  assert min(d.values()) >= -0.50 - 1e-9


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
