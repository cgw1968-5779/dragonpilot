from types import SimpleNamespace as N

import numpy as np

from dragonpilot.selfdrive.controls.lib import eco_ev as E
from dragonpilot.selfdrive.controls.lib.drive_mode import drive_mode_max_accel

V68 = 68.0 / 3.6


def sm(drive_mode=0, engine_off=False, v=V68, soc=45.0, v_cruise_kph=68.0, lead=False, v_lead=None):
  return {
    'carState': N(driveMode=drive_mode, engineOff=engine_off, vEgo=v, aEgo=0.0, gasPressed=False, brakePressed=False,
                  vCruise=v_cruise_kph, hvSoc=soc, steeringAngleDeg=0.0),
    'carControl': N(orientationNED=[0.0, 0.0, 0.0]),
    'radarState': N(leadOne=N(status=lead, dRel=30.0, vLead=v if v_lead is None else v_lead)),
  }


def test_engine_off_eco_line():
  e = E.EcoEv(0.05)
  for v_kph, want in ((60.0, 0.25), (70.0, 0.16), (80.0, 0.12)):
    assert abs(e.max_accel(sm(engine_off=True, v=v_kph / 3.6, soc=45.0), False, 2.0) - want) < 0.01
  # engine running: the normal eco line
  assert abs(e.max_accel(sm(engine_off=False, soc=10.0), False, 2.0) - drive_mode_max_accel(V68, 0, 2.0)) < 1e-9


def test_normal_and_sport_untouched():
  for mode in (1, 2):
    e = E.EcoEv(0.05)
    for _ in range(int(30 / 0.05)):
      out = e.max_accel(sm(drive_mode=mode, engine_off=True), False, 2.0)
      assert out == drive_mode_max_accel(V68, mode, 2.0)


def test_glide_lift_in_eco_cruise():
  e = E.EcoEv(0.05)
  outs = [e.max_accel(sm(), False, 2.0) for _ in range(int(25 / 0.05))]
  assert min(outs) < 0.0 and min(outs) >= -0.30 - 1e-9   # the light lift came
  # with a lead close ahead it never comes
  e = E.EcoEv(0.05)
  outs = [e.max_accel(sm(lead=True), False, 2.0) for _ in range(int(25 / 0.05))]
  assert min(outs) > 0.0
  # nor with unknown pitch, a low battery or long control off
  for kw, off in (({}, True), ({'soc': 20.0}, False)):
    e = E.EcoEv(0.05)
    assert min(e.max_accel(sm(**kw), off, 2.0) for _ in range(int(25 / 0.05))) > 0.0


def test_ev_launch_follows_the_owners_manual_ev_launch():
  # owner 2026-11-08, route f8 00:28 (manual EV launch, delivered): 0-5 1.39, 5-10 1.16, 10-15 0.91, 15-20 0.73,
  # 20-25 0.63, 25-30 0.51, 30-35 0.44; the car gives ~x1.25 the plan below ~10 km/h, 1.0 from 18 km/h
  for v_kph, delivered in ((2.5, 1.39), (7.5, 1.16), (12.5, 0.91), (17.5, 0.73), (22.5, 0.63), (27.5, 0.51), (32.5, 0.44)):
    v = v_kph / 3.6
    a = E.EcoEv(0.05).max_accel(sm(engine_off=True, v=v), False, 2.0)
    gain = float(np.interp(v, [10 / 3.6, 18 / 3.6], [1.25, 1.0]))
    assert abs(a * gain - delivered) < 0.12, (v_kph, a * gain)
    assert a < drive_mode_max_accel(v, 0, 2.0)                    # lighter than the eco line


def test_ev_launch_gives_way_on_low_battery():
  v = 10 / 3.6
  e = E.EcoEv(0.05)
  assert e.max_accel(sm(engine_off=True, v=v, soc=30.0), False, 2.0) == drive_mode_max_accel(v, 0, 2.0)
  assert abs(e.max_accel(sm(engine_off=True, v=v, soc=-1.0), False, 2.0) - 0.80) < 1e-9   # unknown SOC: EV kept


def test_keep_up_with_a_lead_pulling_away_quickly_from_20_kmh():
  def run(v_kph, v_lead_kph, n=100):
    e = E.EcoEv(0.05)
    outs = [e.max_accel(sm(engine_off=True, v=v_kph / 3.6, lead=True, v_lead=v_lead_kph / 3.6), False, 2.0) for _ in range(n)]
    return outs
  ev = lambda kph: run(kph, kph)[-1]
  assert run(15, 30)[-1] == ev(15)                                   # below 20 km/h: always EV
  assert run(30, 32)[-1] == ev(30)                                   # a lead only 2 km/h faster: still EV
  outs = run(30, 45)
  assert abs(outs[-1] - drive_mode_max_accel(30 / 3.6, 0, 2.0)) < 1e-9   # pulling away quickly at 30: the eco line
  assert max(abs(b - a) for a, b in zip(outs, outs[1:])) <= E.KEEP_UP_RATE * 0.05 * \
    (drive_mode_max_accel(30 / 3.6, 0, 2.0) - ev(30)) + 1e-9         # built smoothly
  assert ev(25) < run(25, 40)[-1] < drive_mode_max_accel(25 / 3.6, 0, 2.0)  # 20-30: part way
