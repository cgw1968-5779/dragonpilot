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
