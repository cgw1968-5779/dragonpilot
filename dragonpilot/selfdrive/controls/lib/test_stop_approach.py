from types import SimpleNamespace as N

import numpy as np

import dragonpilot.selfdrive.controls.lib.stop_approach as SA
from dragonpilot.selfdrive.controls.lib.stop_approach import StopApproach

DT = 0.05


def lead(d, v=0.0, status=True):
  return N(status=status, dRel=d, vLead=v)


def drive(v0, d0, a_mpc=-1.0, t_max=40.0):
  """point-mass car that follows the target exactly, approaching a stopped lead"""
  sa = StopApproach(DT)
  v, x, out = v0, 0.0, []
  for _ in range(int(t_max / DT)):
    a = sa.update(v, lead(d0 - x), a_mpc, False)
    out.append((v, d0 - x, a, sa.phase))
    v = max(0.0, v + a * DT)
    x += v * DT
    if v <= 0.0:
      break
  return out


def test_no_lead_and_moving_lead_pass_the_plan_through():
  sa = StopApproach(DT)
  for a in (-1.3, 0.2, -0.4):
    assert sa.update(15.0, lead(40.0, status=False), a, False) == a
    assert sa.update(15.0, lead(40.0, v=8.0), a, False) == a
    assert sa.phase is None


def test_far_stopped_lead_no_early_brake():
  # 50 km/h, stopped car 100 m ahead: the needed decel is ~1.0 -> only the light lift, not the MPC's early -1.0
  sa = StopApproach(DT)
  a = 0.0
  for _ in range(40):
    a = sa.update(50 / 3.6, lead(100.0), -1.0, False)
  assert sa.phase == 'wait'
  assert abs(a - SA.WAIT_ACCEL) < 1e-6


def test_build_is_linear_and_limited():
  out = drive(50 / 3.6, 90.0)
  a = np.array([o[2] for o in out if o[0] > 0.3])  # below 0.3 m/s the stopping controller owns the request
  assert np.min(np.diff(a)) >= -SA.J_BUILD * DT - 1e-9  # never firmer faster than J_BUILD
  assert any(o[3] == 'brake' for o in out)


def test_stops_near_the_gap_and_eases_at_the_end():
  for v0, d0 in ((50 / 3.6, 90.0), (40 / 3.6, 60.0), (30 / 3.6, 40.0)):
    out = drive(v0, d0)
    # an ideal lag-free car stops a little short of the aim (the end release); on the Cross model with the stopping
    # controller's end line the closed-loop sim ends 3.4-4.1 m (tncr1_curves/sim/sim_lead.py)
    d_end = out[-1][1]
    assert SA.STOP_GAP - 1.0 < d_end < SA.STOP_GAP + 1.0, (v0, d0, d_end)
    rel = [o for o in out if o[3] == 'release' and o[0] > 0.3]
    assert rel, 'no end release'
    a = np.array([o[2] for o in rel])
    assert np.all(np.diff(a) >= -1e-9)  # the release never firms up again (no second bite)
    assert np.max(np.diff(a)) <= SA.J_RELEASE * DT + 1e-9
    assert rel[-1][2] >= -0.7  # eased onto the end line before the stopping controller takes over


def test_urgent_follows_a_firmer_plan():
  sa = StopApproach(DT)
  a = sa.update(50 / 3.6, lead(12.0), -3.0, False)
  assert a <= -3.0


def test_handback_is_rate_limited():
  sa = StopApproach(DT)
  for _ in range(60):
    sa.update(40 / 3.6, lead(35.0), -1.0, False)
  a0 = sa.a
  a1 = sa.update(40 / 3.6, lead(35.0, v=5.0), 0.5, False)  # the lead pulls away
  assert sa.phase == 'handback'
  assert abs(a1 - a0) <= SA.J_HANDBACK * DT + 1e-9


def test_disengaged_is_untouched():
  sa = StopApproach(DT)
  assert sa.update(15.0, lead(30.0), 0.3, True) == 0.3
