from types import SimpleNamespace as N

from dragonpilot.selfdrive.controls.lib.drive_mode import drive_mode_max_accel
from dragonpilot.selfdrive.controls.lib.eco_ev import EcoEv

DT = 0.05


def sm(v, mode=0, off=True, soc=45.0, lead=None):
  lead = lead or N(status=False, vLead=0.0, dRel=0.0)
  return {'carState': N(vEgo=v, driveMode=mode, engineOff=off, hvSoc=soc), 'radarState': N(leadOne=lead)}


def settle(e, s, n=40):
  for _ in range(n):
    a = e.max_accel(s, 2.0)
  return a


def test_engine_off_eco_launch_lighter():
  for kph, plan in ((0, 1.08), (10, 1.00), (15, 0.92), (20, (0.80 + drive_mode_max_accel(20 / 3.6, 0, 2.0)) / 2),
                    (25, drive_mode_max_accel(25 / 3.6, 0, 2.0))):
    a = settle(EcoEv(DT), sm(kph / 3.6))
    assert abs(a - plan) < 0.01, (kph, a)
    assert a <= drive_mode_max_accel(kph / 3.6, 0, 2.0)


def test_untouched_cases():
  for s in (sm(3.0, mode=1), sm(3.0, mode=2), sm(3.0, off=False), sm(3.0, mode=-1), sm(8.0), sm(3.0, soc=30.0)):
    e = EcoEv(DT)
    assert settle(e, s) == drive_mode_max_accel(s['carState'].vEgo, s['carState'].driveMode, 2.0)


def test_engine_start_eases_back_in_1s():
  e = EcoEv(DT)
  a0 = settle(e, sm(4.0))
  normal = drive_mode_max_accel(4.0, 0, 2.0)
  steps = [e.max_accel(sm(4.0, off=False), 2.0) for _ in range(25)]
  assert a0 < steps[0] < steps[9] < normal and abs(steps[-1] - normal) < 1e-6
  assert max(b - a for a, b in zip([a0] + steps, steps)) <= (normal - a0) * DT / 1.0 + 1e-6


def test_lead_pulling_away_keeps_up_from_20kph():
  lead = N(status=True, vLead=12.0, dRel=20.0)
  assert abs(settle(EcoEv(DT), sm(10 / 3.6, lead=lead)) - 1.00) < 0.01   # under 20 km/h: still EV launch
  e, normal = EcoEv(DT), drive_mode_max_accel(22 / 3.6, 0, 2.0)
  free = settle(EcoEv(DT), sm(22 / 3.6))
  kept = settle(e, sm(22 / 3.6, lead=lead), n=60)
  assert free < kept < normal   # keep-up weight 0.2 at 22 km/h
