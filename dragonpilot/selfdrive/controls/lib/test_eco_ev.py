from types import SimpleNamespace as N

from dragonpilot.selfdrive.controls.lib import eco_ev as E
from dragonpilot.selfdrive.controls.lib.drive_mode import drive_mode_max_accel

V68 = 68.0 / 3.6


def sm(drive_mode=0, engine_off=False, v=V68, soc=45.0, v_cruise_kph=68.0, lead=False):
  return {
    'carState': N(driveMode=drive_mode, engineOff=engine_off, vEgo=v, aEgo=0.0, gasPressed=False, brakePressed=False,
                  vCruise=v_cruise_kph, hvSoc=soc, steeringAngleDeg=0.0),
    'carControl': N(orientationNED=[0.0, 0.0, 0.0]),
    'radarState': N(leadOne=N(status=lead, dRel=30.0, vLead=v)),
  }


def test_engine_off_eco_line():
  e = E.EcoEv(0.05)
  for v_kph, want in ((60.0, 0.25), (70.0, 0.16), (80.0, 0.12)):
    assert abs(e.max_accel(sm(engine_off=True, v=v_kph / 3.6, soc=10.0), False, 2.0) - want) < 0.01
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
