from types import SimpleNamespace as N

from dragonpilot.selfdrive.controls.lib.drive_mode import drive_mode_max_accel
from dragonpilot.selfdrive.controls.lib.eco_ev import EcoEv

DT = 0.05
K = 1 / 3.6


def sm(v, mode=0, off=True, soc=45.0, lead=None, v_cruise_kph=None, a=0.0, pitch=0.0):
  lead = lead or N(status=False, vLead=0.0, dRel=0.0)
  vc = v / K if v_cruise_kph is None else v_cruise_kph
  return {'carState': N(vEgo=v, aEgo=a, driveMode=mode, engineOff=off, hvSoc=soc, vCruise=vc, gasPressed=False,
                        brakePressed=False, steeringAngleDeg=0.0),
          'radarState': N(leadOne=lead), 'carControl': N(orientationNED=[0.0, pitch, 0.0])}


def settle(e, s, n=40, lc_off=False):
  for _ in range(n):
    a = e.max_accel(s, lc_off, 2.0)
  return a


def normal(v, mode=0):
  return drive_mode_max_accel(v, mode, 2.0)


def test_engine_off_eco_launch_lighter():
  for kph, plan in ((0, 1.08), (10, 1.00), (15, 0.92), (20, (0.80 + normal(20 * K)) / 2), (25, normal(25 * K))):
    a = settle(EcoEv(DT), sm(kph * K))
    assert abs(a - plan) < 0.01, (kph, a)
    assert a <= normal(kph * K)


def test_engine_off_cruise_line():
  for kph, plan in ((40, 0.15), (50, 0.12), (60, 0.08), (70, 0.06), (80, 0.05), (85, 0.05), (90, normal(90 * K)),
                    (30, normal(30 * K))):
    a = settle(EcoEv(DT), sm(kph * K), lc_off=True)
    assert abs(a - plan) < 0.01, (kph, a)


def test_well_under_set_speed_gives_way():
  v = 60 * K
  assert abs(settle(EcoEv(DT), sm(v, v_cruise_kph=62), lc_off=True) - 0.08) < 0.01
  assert abs(settle(EcoEv(DT), sm(v, v_cruise_kph=70), n=60, lc_off=True) - normal(v)) < 1e-6
  mid = settle(EcoEv(DT), sm(v, v_cruise_kph=65.5), n=60, lc_off=True)
  assert 0.08 < mid < normal(v)
  # behind a steady lead (not pulling away) the line stays even far under the set speed
  lead = N(status=True, vLead=v, dRel=30.0)
  assert abs(settle(EcoEv(DT), sm(v, v_cruise_kph=90, lead=lead), n=60, lc_off=True) - 0.08) < 0.01
  far = N(status=True, vLead=v, dRel=80.0)
  assert abs(settle(EcoEv(DT), sm(v, v_cruise_kph=90, lead=far), n=60, lc_off=True) - normal(v)) < 1e-6


def test_untouched_cases():
  for s in (sm(3.0, mode=1), sm(3.0, mode=2), sm(3.0, off=False), sm(3.0, mode=-1), sm(8.0), sm(3.0, soc=30.0),
            sm(18.0, mode=1), sm(18.0, off=False, soc=-1.0)):
    e = EcoEv(DT)
    assert settle(e, s, lc_off=True) == normal(s['carState'].vEgo, s['carState'].driveMode)


def test_engine_start_eases_back_in_1s():
  e = EcoEv(DT)
  a0 = settle(e, sm(4.0))
  steps = [e.max_accel(sm(4.0, off=False), False, 2.0) for _ in range(25)]
  assert a0 < steps[0] < steps[9] < normal(4.0) and abs(steps[-1] - normal(4.0)) < 1e-6


def test_lead_pulling_away_keeps_up_from_20kph():
  lead = N(status=True, vLead=12.0, dRel=20.0)
  assert abs(settle(EcoEv(DT), sm(10 * K, lead=lead)) - 1.00) < 0.01   # under 20 km/h: still EV launch
  free = settle(EcoEv(DT), sm(22 * K))
  kept = settle(EcoEv(DT), sm(22 * K, lead=lead), n=60)
  assert free < kept < normal(22 * K)


def test_glide_fires_in_steady_eco_cruise_engine_on():
  e, s = EcoEv(DT), sm(70 * K, off=False, soc=46.0)
  caps = [e.max_accel(s, False, 2.0) for _ in range(int(30 / DT))]
  assert min(caps) <= -0.29                       # the lift to -0.30
  # steady cruise asks ~0, so the ceiling stepping onto 0 changes nothing; below 0 it moves at <= 0.6 m/s^3
  assert all(b - a >= -0.6 * DT - 1e-6 for a, b in zip(caps, caps[1:]) if a <= 0.0)
  for blocked in (sm(70 * K, off=False, soc=42.0), sm(70 * K, off=False, mode=1), sm(70 * K, off=False, pitch=0.05)):
    e = EcoEv(DT)
    assert min(e.max_accel(blocked, False, 2.0) for _ in range(int(30 / DT))) > 0.0
