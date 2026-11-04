from types import SimpleNamespace

from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import longcontrol as lc

CP = CarInterface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2)
LIM = (-3.5, 2.0)


def cs(v=8.0, a=0.0, standstill=False):
  return SimpleNamespace(vEgo=v, aEgo=a, standstill=standstill, brakePressed=False, cruiseState=SimpleNamespace(standstill=False))


def run(a_target_fn, frames=150, off_frames=10, v=8.0):
  """off for a while, then engaged; returns the engaged outputs per frame (frame 0 = engagement)"""
  LC = lc.LongControl(CP)
  for _ in range(off_frames):
    LC.update(False, cs(v), 0.0, False, LIM)
  return [LC.update(True, cs(v), a_target_fn(k), False, LIM) for k in range(frames)]


def at(outs, t):
  return outs[round(t / DT_CTRL) - 1]


def test_first_tenth_of_a_second_is_soft():
  outs = run(lambda k: -1.0)
  assert at(outs, 0.1) > -0.16          # stock (carcontroller 4 m/s^3): -0.4
  assert -0.45 < at(outs, 0.2) < -0.35
  # rising jerk, never above the stock 4 m/s^3, never a step
  steps = [outs[0]] + [b - a for a, b in zip(outs, outs[1:])]
  assert all(-4.0 * DT_CTRL - 1e-9 <= s <= 1e-9 for s in steps[:30])
  # after the onset the planner's request is applied (carcontroller keeps its own 4 m/s^3)
  assert all(o == -1.0 for o in outs[40:])


def test_never_firmer_than_the_planner():
  for target in (-0.05, -0.3, -1.0, -2.5):
    for o in run(lambda k: target):
      assert o >= target - 1e-9


def test_light_request_untouched_after_reached():
  outs = run(lambda k: -0.1)
  assert all(o == -0.1 for o in outs[20:])


def test_only_after_engaging():
  # a brake that starts later than the window after coasting: the driving brake onset (same soft start), not the engage one
  outs = run(lambda k: 0.0 if k * DT_CTRL < 1.2 else -1.0, frames=200)
  assert -0.16 < outs[round(1.3 / DT_CTRL) - 1] < -0.1
  # only once per engagement: a second brake inside the window after the first onset ended is unchanged
  outs = run(lambda k: -0.5 if k < 40 else (0.0 if k < 60 else -1.0), frames=100)
  assert outs[60] == -1.0


def test_standstill_resume_untouched():
  LC = lc.LongControl(CP)
  LC.update(False, cs(0.0, standstill=True), 0.0, False, LIM)
  out = LC.update(True, cs(0.0, standstill=True), -1.0, False, LIM)
  assert LC.onset_t is None
  assert out <= 0.0


def test_gas_override_release_untouched():
  LC = lc.LongControl(CP)
  LC.update(True, cs(), 0.0, False, LIM)
  for _ in range(200):
    LC.update(True, cs(), 0.0, False, LIM)
  gas = cs(); gas.gasPressed = True
  for _ in range(20):
    LC.update(False, gas, 0.0, False, LIM)  # controlsd: longActive False while the driver is on the gas
  assert LC.update(True, cs(), -1.0, False, LIM) == -1.0


# dppb (owner 2026-11-04): after the soft start the brake keeps climbing linearly, and the firmest brake is held a
# little longer behind a stopped lead so an engage stop ends at 3-3.5 m

def lead(d, v=0.0):
  return SimpleNamespace(status=True, dRel=d, vLead=v, aLeadK=0.0)


def run_hold(plan, d_fn, v_fn, frames=600, lead_v=0.0, lead_on=True):
  LC = lc.LongControl(CP)
  for _ in range(10):
    LC.lead = lead(d_fn(0), lead_v) if lead_on else None
    LC.update(False, cs(v_fn(0)), 0.0, False, LIM)
  outs = []
  for k in range(frames):
    LC.lead = lead(d_fn(k), lead_v) if lead_on else None
    outs.append(LC.update(True, cs(v_fn(k)), plan(k), False, LIM))
  return outs


def test_keeps_climbing_linearly_after_the_soft_start():
  outs = run(lambda k: -2.5)
  assert -0.45 < at(outs, 0.2) < -0.35
  assert -0.85 < at(outs, 0.3) < -0.75       # owner: -0.76
  assert -1.65 < at(outs, 0.5) < -1.55       # the stock 4 m/s^3 cannot reach -1.7 at 0.5 s: it keeps climbing after
  assert at(outs, 0.75) == -2.5


def test_hold_only_behind_a_stopped_lead_and_never_deeper_than_the_peak():
  # plan peaks at -2.5 then releases fast, at 40 km/h with a stopped lead 30 m ahead (needs the brake held)
  plan = lambda k: -2.5 if k < 100 else min(-2.5 + (k - 100) * 0.02, -1.0)
  outs = run_hold(plan, lambda k: 30.0, lambda k: 40 / 3.6)
  assert min(outs) >= -2.5 - 1e-9                       # never deeper than the stop's own firmest request
  assert any(o < plan(k) - 0.3 for k, o in enumerate(outs))  # held beyond the released plan
  steps = [b - a for a, b in zip(outs[150:], outs[151:])]
  assert max(steps) <= lc.ENGAGE_HOLD_DOWN * DT_CTRL + 0.02 * 1.0 + 1e-9  # lets go no faster than the plan + DOWN
  # moving lead / no lead: nothing held (never firmer than the planner)
  for kw in (dict(lead_v=5.0), dict(lead_on=False)):
    outs = run_hold(plan, lambda k: 30.0, lambda k: 40 / 3.6, **kw)
    assert all(o >= plan(k) - 1e-9 for k, o in enumerate(outs) if k > 80)


def test_no_hold_at_low_speed_and_none_later():
  plan = lambda k: -2.0 if k < 100 else -0.8
  outs = run_hold(plan, lambda k: 10.0, lambda k: 9 / 3.6)  # 9 km/h: the normal end's
  assert all(o >= plan(k) - 1e-9 for k, o in enumerate(outs) if k > 80)
  outs = run_hold(plan, lambda k: 30.0, lambda k: 40 / 3.6, frames=1700)
  assert outs[-1] == -0.8                                    # ENGAGE_HOLD_MAX_T after the onset: the plan as it is


# dppb (owner 2026-11-05): a brake that starts while driving (cut-in) gets the engage soft start, then climbs quickly
# (stock 4 m/s^3) to the planner's request

def run_drive(plan, frames=300, v=15.0, lead=None):
  LC = lc.LongControl(CP)
  for _ in range(5):
    LC.update(False, cs(v), 0.0, False, LIM)
  for _ in range(200):  # engaged long ago, cruising
    LC.lead = lead
    LC.update(True, cs(v), 0.3, False, LIM)
  outs = []
  for k in range(frames):
    LC.lead = lead
    outs.append(LC.update(True, cs(v), plan(k), False, LIM))
  return outs


def test_cut_in_soft_start_then_quick_climb():
  lead = SimpleNamespace(status=True, dRel=19.0, vLead=13.3, aLeadK=0.0)  # 21:53:33: 48 km/h car 19 m ahead
  outs = run_drive(lambda k: -2.0, lead=lead)
  first = next(k for k, o in enumerate(outs) if o < 0.0)
  assert outs[first + 9] > -0.16                         # -0.15 at 0.1 s
  assert -0.45 < outs[first + 19] < -0.35                # -0.40 at 0.2 s
  assert -1.65 < outs[first + 49] < -1.55                # then 4 m/s^3: -1.6 at 0.5 s
  assert outs[first + 70] == -2.0                        # the planner's force
  assert all(o >= -2.0 - 1e-9 for o in outs)             # never firmer than the planner


def test_gradual_planner_untouched():
  outs = run_drive(lambda k: max(-0.005 * k, -1.0))      # planner builds at 0.5 m/s^3
  assert all(abs(o - max(-0.005 * k, -1.0)) < 1e-9 for k, o in enumerate(outs))


def test_no_onset_while_already_braking():
  outs = run_drive(lambda k: -0.5 if k < 100 else -1.5)  # braking, then a firmer request: planner as it is
  assert outs[150] == -1.5
