from dragonpilot.selfdrive.ui.mici.dial_hud import dial_kph, OFFSET_FULL_KPH
from dragonpilot.selfdrive.car.toyota_set_speed import toyota_dial_from_canonical


def test_same_line_as_the_set_speed_above_30():
  for v in (40.0, 72.4, 91.1, 109.8):
    assert abs(dial_kph(v) - toyota_dial_from_canonical(v)) < 0.16


def test_zero_at_standstill_and_offset_fades_in():
  assert dial_kph(0.0) == 0.0
  assert dial_kph(-1.0) == 0.0
  assert dial_kph(OFFSET_FULL_KPH / 2) < toyota_dial_from_canonical(OFFSET_FULL_KPH / 2)


def test_dial_hud_numbers(monkeypatch):
  from types import SimpleNamespace as N
  from opendbc.car.toyota.interface import CarInterface
  from opendbc.car.toyota.values import CAR
  from dragonpilot.selfdrive.ui.mici import dial_hud
  CP = CarInterface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2)
  CP.openpilotLongitudinalControl = True
  CP.pcmCruise = True
  monkeypatch.setattr(dial_hud, "ui_state", N(CP=CP, is_metric=True))
  hud = dial_hud.DialHud()
  def car(v_kph, pcm_ui_kph=120.0):
    return N(vEgo=v_kph / 3.6, cruiseState=N(speedCluster=pcm_ui_kph / 3.6, speed=0.0))
  hud.update(car(72.4))
  assert hud.active and hud.shown == 80          # the dial reads 80 at canonical 72.4 (set speed 80 drives 72.4)
  hud.update(car(72.6))
  assert hud.shown == 80                         # hysteresis: no flicker
  assert abs(hud.pcm_ceiling - 120.0) < 1e-6
  hud.update(car(0.0))
  assert hud.shown == 0
  CP.carFingerprint = CAR.TOYOTA_RAV4_TSS2      # not the Cross: stock box
  hud.update(car(72.4))
  assert not hud.active
