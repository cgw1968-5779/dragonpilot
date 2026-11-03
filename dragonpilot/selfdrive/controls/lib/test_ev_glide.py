from types import SimpleNamespace

import dragonpilot.selfdrive.controls.lib.ev_glide as EG
from dragonpilot.selfdrive.controls.lib.ev_glide import EvGlide

DT = 0.05
V = 68.0 / 3.6
NO_LEAD = SimpleNamespace(present=False, dRel=0.0, vLead=0.0)


def run(g, n, **kw):
  args = dict(eco=True, long_active=True, driver=False, v_ego=V, a_ego=0.0, v_cruise=V, engine_off=False,
              engine_rpm=1300.0, soc=45.0, pitch=0.0, angle=0.0, lead=NO_LEAD)
  args.update(kw)
  out = []
  for _ in range(n):
    out.append(g.update(**args))
  return out


def warm(g):
  # engine running and steady at the set speed long enough
  return run(g, int(max(EG.STEADY_T, EG.ENGINE_ON_T) / DT) + 1)


def test_light_lift_after_steady_cruise_then_hold():
  g = EvGlide(DT)
  out = warm(g)
  assert out[-1] is not None and g.state == "lift"
  caps = [c for c in out if c is not None] + run(g, 40)
  steps = [b - a for a, b in zip(caps, caps[1:], strict=False)]
  assert all(-EG.GLIDE_JERK * DT - 1e-9 <= s <= 1e-9 for s in steps[:8])  # down at GLIDE_JERK, no step
  assert min(caps) >= EG.GLIDE_A - 1e-9
  assert g.state == "hold"


def test_engine_stops_then_released_smoothly_and_cooldown():
  g = EvGlide(DT)
  warm(g)
  run(g, 20)
  out = run(g, int(EG.CONFIRM_T / DT) + 2, engine_off=True, engine_rpm=0.0)
  assert g.state == "release" and g.stopped_engine
  caps = [c for c in out if c is not None] + [c for c in run(g, 60, engine_off=True, engine_rpm=0.0) if c is not None]
  assert all(0.0 <= b - a <= EG.RELEASE_JERK * DT + 1e-9 for a, b in zip(caps[-30:], caps[-29:], strict=False))
  assert g.state == "idle" and g.cap is None
  assert abs(g.cooldown - (EG.RETRY_T - 60 * DT)) < 0.2


def test_no_engine_stop_ends_after_hold_and_waits_longer():
  g = EvGlide(DT)
  warm(g)
  run(g, int((EG.HOLD_T + 1.0) / DT))
  assert g.state in ("release", "idle") and not g.stopped_engine
  assert g.cooldown > EG.RETRY_T


def test_blocked_conditions():
  near_lead = SimpleNamespace(present=True, dRel=40.0, vLead=V)
  slow_lead = SimpleNamespace(present=True, dRel=150.0, vLead=V - 3.0)
  for kw in (dict(eco=False), dict(soc=30.0), dict(soc=-1.0), dict(lead=near_lead), dict(lead=slow_lead),
             dict(pitch=0.03), dict(angle=10.0), dict(v_ego=45 / 3.6, v_cruise=45 / 3.6), dict(v_cruise=V + 3.0),
             dict(a_ego=0.4), dict(engine_off=True, engine_rpm=0.0), dict(long_active=False), dict(driver=True)):
    g = EvGlide(DT)
    assert all(c is None for c in run(g, 600, **kw)), kw


def test_driver_or_set_speed_change_ends_it():
  for kw in (dict(driver=True), dict(v_cruise=V + 5 / 3.6)):
    g = EvGlide(DT)
    warm(g)
    run(g, 5)
    run(g, 1, **kw)
    assert g.state in ("release", "idle"), kw
