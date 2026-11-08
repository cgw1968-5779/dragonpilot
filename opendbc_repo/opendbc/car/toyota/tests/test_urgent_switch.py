from types import SimpleNamespace as N

from opendbc.car.toyota.urgent_switch import UrgentSwitch, need_decel

KPH = 1 / 3.6


def lead(d, v_kph, a=0.0):
  return N(status=True, dRel=d, vLead=v_kph * KPH, aLeadK=a)


def test_regime_on_and_off_with_hysteresis():
  sw = UrgentSwitch(0.03)
  sw.prev_d = 44.0
  assert sw.update(100.8 * KPH, lead(43.8, 94.1, -1.93), False)        # fe 14:17 regime
  assert sw.active and not sw.cutin
  sw.update(98 * KPH, lead(42.0, 90, -2.0), False)                     # need ~2.0: still on (off below 1.5)
  assert sw.active
  sw.update(87 * KPH, lead(40.3, 86, -0.3), False)
  assert not sw.active


def test_disengaged_and_need():
  sw = UrgentSwitch(0.03)
  assert not sw.update(100 * KPH, lead(43.8, 94.1, -1.93), True)
  assert need_decel(20.0, 30.0, 20.0, 0.0) == 0.0
