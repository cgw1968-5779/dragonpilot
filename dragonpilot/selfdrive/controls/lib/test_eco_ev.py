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


def test_ev_launch_is_lighter_and_stays_under_7kw():
  # owner 2026-11-07: lighter eco launch on the battery; the car delivers ~x1.25 the plan below ~12 km/h
  m, kw = 1500.0, 1000.0
  for v_kph in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0):
    v = v_kph / 3.6
    e = E.EcoEv(0.05)
    a = e.max_accel(sm(engine_off=True, v=v, lead=True), False, 2.0)
    assert a <= drive_mode_max_accel(v, 0, 2.0) - 0.2           # lighter than the eco line
    gain = float(np.interp(v, [10 / 3.6, 18 / 3.6], [1.25, 1.0]))
    road = m * 9.81 * 0.010 * v + 0.5 * 1.2 * 0.82 * v ** 3
    assert (m * a * gain * v + road) / kw <= 7.05, v_kph          # under the ~7.8 kW lowest engine start seen
  assert abs(E.EcoEv(0.05).max_accel(sm(engine_off=True, v=0.0), False, 2.0) - 1.20) < 1e-9


def test_ev_launch_gives_way_on_low_battery():
  v = 10 / 3.6
  e = E.EcoEv(0.05)
  assert e.max_accel(sm(engine_off=True, v=v, soc=30.0), False, 2.0) == drive_mode_max_accel(v, 0, 2.0)
  assert abs(e.max_accel(sm(engine_off=True, v=v, soc=-1.0), False, 2.0) - 1.20) < 1e-9   # unknown SOC: EV kept


def test_keep_up_with_a_lead_pulling_away_from_20_kmh():
  def cap(v_kph, v_lead_kph):
    e = E.EcoEv(0.05)
    return e.max_accel(sm(engine_off=True, v=v_kph / 3.6, lead=True, v_lead=v_lead_kph / 3.6), False, 2.0)
  ev15, ev25, ev30 = cap(15, 15), cap(25, 25), cap(30, 30)
  assert cap(15, 30) == ev15                                          # below 20 km/h: always EV
  assert ev25 < cap(25, 35) < drive_mode_max_accel(25 / 3.6, 0, 2.0)  # 20-30: blending toward the eco line
  assert cap(30, 45) == drive_mode_max_accel(30 / 3.6, 0, 2.0) > ev30   # 30 and up: keep up
  assert cap(60, 75) == drive_mode_max_accel(60 / 3.6, 0, 2.0)
  # hysteresis: once pulling away, a lead only 0.3 m/s faster keeps it
  e = E.EcoEv(0.05)
  e.max_accel(sm(engine_off=True, v=30 / 3.6, lead=True, v_lead=35 / 3.6), False, 2.0)
  assert e.max_accel(sm(engine_off=True, v=30 / 3.6, lead=True, v_lead=30 / 3.6 + 0.3), False, 2.0) == \
    drive_mode_max_accel(30 / 3.6, 0, 2.0)
