from types import SimpleNamespace as N

from dragonpilot.selfdrive.controls.lib import urgent_brake as U

DT = 0.05
KPH = 1 / 3.6


def lead(d, v_kph, a=0.0, status=True):
  return N(status=status, dRel=d, vLead=v_kph * KPH, aLeadK=a)


def test_f8_0039_switches_to_full_brake_at_once():
  # route f8 00:39:02.5: 58 km/h, lead 27.4 m at 45.6 km/h braking -2.54 -> need ~2.7
  ub = U.UrgentBrake(DT)
  assert ub.update(57.6 * KPH, lead(27.4, 45.6, -2.54), -2.07, False) == U.ACCEL_MIN
  assert ub.active


def test_normal_following_and_a_firm_lead_brake_do_not_switch():
  ub = U.UrgentBrake(DT)
  # 58 km/h, 29.5 m, lead braking -2.0 from the same speed: need ~1.7
  assert ub.update(58 * KPH, lead(29.5, 58, -2.0), -1.2, False) == -1.2
  # steady following / lead faster
  assert ub.update(58 * KPH, lead(29.5, 58, 0.0), 0.1, False) == 0.1
  # below 40 km/h never
  assert ub.update(35 * KPH, lead(8.0, 10, -3.0), -2.5, False) == -2.5
  assert not ub.active


def test_stopped_lead_needs_more():
  ub = U.UrgentBrake(DT)
  v = 50 * KPH  # need = v^2 / 2 / (d - 3.2 - v 0.5)
  assert ub.update(v, lead(45.0, 0), -2.0, False) == -2.0          # need ~2.8 < 3.0: the stop target's job
  assert ub.update(v, lead(38.0, 0), -2.0, False) == U.ACCEL_MIN   # need ~3.5 > 3.0


def test_off_and_smooth_handback():
  ub = U.UrgentBrake(DT)
  ub.update(57.6 * KPH, lead(27.4, 45.6, -2.54), -2.07, False)
  out = ub.update(30 * KPH, lead(30.0, 30, 0.0), -1.0, False)     # need gone -> hand back at J_HANDBACK
  assert not ub.active and abs(out - (U.ACCEL_MIN + U.J_HANDBACK * DT)) < 1e-9
  for _ in range(100):
    out = ub.update(30 * KPH, lead(30.0, 30, 0.0), -1.0, False)
  assert out == -1.0
  ub.update(57.6 * KPH, lead(27.4, 45.6, -2.54), -2.07, False)
  assert ub.update(57.6 * KPH, lead(27.4, 45.6, -2.54), 0.3, True) == 0.3   # disengaged: untouched
