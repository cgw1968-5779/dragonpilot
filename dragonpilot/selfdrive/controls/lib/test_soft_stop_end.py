from types import SimpleNamespace

from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import longcontrol as lc

CP = CarInterface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2)
LIM = (-3.5, 2.0)


def cs(v, a, standstill=False):
  return SimpleNamespace(vEgo=v, aEgo=a, standstill=standstill, brakePressed=False, cruiseState=SimpleNamespace(standstill=False))


def drive(a_target_fn, v0=8.0, a_car=-0.9, steps=3000, should_stop_v=0.0):
  """open-loop: the car decelerates at a_car; returns (v, out) per frame until it stops"""
  LC = lc.LongControl(CP)
  LC.engage_t = 10.0  # engaged long ago: the engage onset (test_engage_onset.py) is not part of these stops
  v, outs = v0, []
  for k in range(steps):
    out = LC.update(True, cs(v, a_car if v > 0 else 0.0, standstill=v <= 0), a_target_fn(v), v < should_stop_v, LIM)
    outs.append((v, out))
    v = max(0.0, v + a_car * DT_CTRL)
  return outs


def test_far_from_stop_unchanged():
  for v, out in drive(lambda v: -0.9):
    if v / 0.9 > lc.SOFT_END_LAG + (lc.SOFT_END_ACCEL - -0.9) / lc.SOFT_END_MAX_RATE + 0.05:
      assert abs(out - -0.9) < 1e-9


def test_eases_to_end_value_and_never_firms_again():
  outs = [o for v, o in drive(lambda v: -0.9) if v > 0]
  assert abs(outs[-1] - lc.SOFT_END_ACCEL) < 1e-6
  for a, b in zip(outs, outs[1:]):
    assert b >= a - 1e-9  # never firmer frame to frame while moving


def test_release_rate_limited():
  outs = [o for v, o in drive(lambda v: -1.4, a_car=-1.4) if v > 0]
  for a, b in zip(outs, outs[1:]):
    assert b - a <= lc.SOFT_END_MAX_RATE * DT_CTRL + 1e-9


def test_urgent_not_eased():
  outs = [o for v, o in drive(lambda v: -2.0, a_car=-2.0) if v > 0]
  assert all(abs(o - -2.0) < 1e-9 for o in outs)


def test_light_planner_untouched():
  outs = [o for v, o in drive(lambda v: -0.2, a_car=-0.3) if v > 0]
  assert all(abs(o - -0.2) < 1e-9 for o in outs)


def test_reset_when_lead_moves_off_before_stopping():
  LC = lc.LongControl(CP)
  LC.engage_t = 10.0  # engaged long ago: the engage onset (test_engage_onset.py) is not part of these stops
  for _ in range(50):  # crawling, nearly stopped, braking -> easing latched
    LC.update(True, cs(0.3, -0.6), -0.8, False, LIM)
  assert LC.soft_end_frac < 1.0
  LC.update(True, cs(0.3, -0.1), 0.3, False, LIM)  # lead moved: planner no longer braking
  assert LC.soft_end_frac == 1.0
  out = LC.update(True, cs(2.5, -0.5), -1.0, False, LIM)  # lead stops again further on: full brake
  assert abs(out - -1.0) < 1e-9


def test_stopping_state_does_not_add_brake_before_standstill():
  LC = lc.LongControl(CP)
  LC.engage_t = 10.0  # engaged long ago: the engage onset (test_engage_onset.py) is not part of these stops
  LC.update(True, cs(0.5, -0.4), -0.4, False, LIM)
  first = LC.update(True, cs(0.2, -0.4), -0.4, True, LIM)  # stopping state, still rolling
  outs = [LC.update(True, cs(0.1, -0.4), -0.4, True, LIM) for _ in range(30)]
  assert all(o >= first - 1e-9 for o in outs)
  held = [LC.update(True, cs(0.0, 0.0, standstill=True), 0.0, True, LIM) for _ in range(100)]
  assert held[-1] < held[0]  # once stopped it firms toward stopAccel to hold the car


# dppb (owner 2026-11-04 night, 21:54:16): -0.30 only in the last ~0.5 s; before that the end eases no further than
# SOFT_END_HOLD_ACCEL behind a stopped lead (the Altis creep cancels a steady -0.30 at 1-3 km/h)

def creep_stop(lead_on=True, lead_v=0.0, plan=-0.9):
  """the car decelerates while the command is firm, but with a -0.30-ish command it creeps (decel collapses)"""
  LC = lc.LongControl(CP)
  LC.engage_t = 10.0
  v, outs = 1.2, []
  for k in range(8000):
    LC.lead = SimpleNamespace(status=True, dRel=4.0, vLead=lead_v, aLeadK=0.0) if lead_on else None
    a_car = -0.05 if outs and outs[-1][1] > -0.36 else -0.5  # Altis: -0.30 is cancelled by the creep
    out = LC.update(True, cs(v, a_car if v > 0 else 0.0, standstill=v <= 0), plan(v) if callable(plan) else plan, False, LIM)
    outs.append((v, out))
    v = max(0.0, v + a_car * DT_CTRL)
    if v <= 0:
      break
  return outs


def test_end_held_at_hold_level_until_the_last_lag():
  outs = creep_stop()
  moving = [(v, o) for v, o in outs if v > 0]
  # once eased to the hold level, never lighter while more than SOFT_END_LAG from the stop (car decel 0.5)
  reached = False
  for v, o in moving:
    reached = reached or o <= lc.SOFT_END_HOLD_ACCEL + 1e-9
    if reached and v / 0.5 > lc.SOFT_END_LAG + 0.02:
      assert o <= lc.SOFT_END_HOLD_ACCEL + 1e-9
  assert reached
  # never firmer frame to frame (item 0) and the car actually stops
  for (_, a), (_, b) in zip(moving, moving[1:]):
    assert b >= a - 1e-9
  assert outs[-1][0] < 1e-6


def test_planner_release_held_not_firmed():
  # the planner drops to -0.22 at 2 km/h (it does not know the creep): eased no further than the hold level, never firmer
  outs = creep_stop(plan=lambda v: -0.9 if v > 0.6 else -0.22)
  moving = [o for v, o in outs if v > 0]
  for a, b in zip(moving, moving[1:]):
    assert b >= a - 1e-9
  assert all(o <= lc.SOFT_END_HOLD_ACCEL + 1e-9 for v, o in outs if 0.3 < v < 0.6)


def test_no_hold_without_a_stopped_lead():
  held = creep_stop()
  for kw in (dict(lead_on=False), dict(lead_v=3.0)):
    outs = creep_stop(**kw)
    # the plain soft end: lighter than the hold level while the held stop is still at it
    assert any(o > lc.SOFT_END_HOLD_ACCEL + 0.03 for v, o in outs if v > 0.27)
  assert all(o <= lc.SOFT_END_HOLD_ACCEL + 1e-9 for v, o in held if 0.27 < v < 0.5)
