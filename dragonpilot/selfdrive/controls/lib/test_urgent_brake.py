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
  # 58 km/h, 2.5 s gap (40.3 m), lead braking -2.0 from the same speed: need ~1.3 (at 1.8 s this is the f8 regime)
  assert ub.update(58 * KPH, lead(40.3, 58, -2.0), -1.2, False) == -1.2
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


def test_f8_regime_switches_earlier():
  # f8 00:39:02.25: 58 km/h, 28.2 m (1.75 s), lead 51 km/h, aLeadK -1.75 -> on (need alone ~2.0 not yet)
  ub = U.UrgentBrake(DT)
  assert ub.update(58.4 * KPH, lead(28.2, 51, -1.75), -0.5, False) == U.ACCEL_MIN
  # same lead decel with a 2.5 s gap, or at 45 km/h: not the regime
  for v, d in ((58.4, 40.6), (45.0, 20.0)):
    ub = U.UrgentBrake(DT)
    assert ub.update(v * KPH, lead(d, v - 7, -1.75), -0.5, False) == -0.5
