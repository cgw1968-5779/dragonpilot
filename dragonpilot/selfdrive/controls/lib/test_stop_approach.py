import pytest
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


def test_far_stopped_lead_brakes_gently_at_once():
  # owner 2026-11-08: slow down as soon as the stopped car is seen. 50 km/h, 100 m: need ~0.99 -> K_EARLY x need,
  # built at J_EARLY (soft first touch), not the MPC's -1.0
  sa = StopApproach(DT)
  outs = [sa.update(50 / 3.6, lead(100.0), 0.2, False) for _ in range(40)]
  assert sa.phase == 'wait'
  need = (50 / 3.6) ** 2 / (2 * (100.0 - SA.STOP_GAP))
  assert abs(outs[-1] + SA.K_EARLY * need) < 1e-6
  assert outs[0] >= -SA.J_EARLY * DT - 1e-9 and np.min(np.diff(outs)) >= -SA.J_EARLY * DT - 1e-9


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


# the committed stop target before the experimental-only changes (11-07), kept here so experimental-off stays identical
class _Before11_07(StopApproach):
  def update(self, v_ego, lead, a_mpc, off, exp=False, a_ego=None):
    stopped_lead = (not off and lead is not None and bool(lead.status) and lead.vLead < SA.LEAD_STOPPED_V
                    and 0.0 < lead.dRel < SA.MAX_DIST and v_ego > 0.3)
    if not stopped_lead:
      if self.phase in (None, 'handback') and (self.phase is None or abs(self.a - a_mpc) < 0.05):
        self.phase = None
        self.a = a_mpc
        return a_mpc
      self.phase = 'handback'
      return self._step(a_mpc, SA.J_HANDBACK, SA.J_HANDBACK)
    need = SA._need(v_ego, lead.dRel)
    if self.phase in (None, 'handback'):
      self.a = a_mpc if self.phase is None else self.a
      self.phase = 'wait'
    if self.phase == 'wait' and SA._need(v_ego, lead.dRel - v_ego * 0.5) >= SA.START_NEED:
      self.phase = 'brake'
    if self.phase == 'wait':
      out = self._step(max(a_mpc, SA.WAIT_ACCEL), SA.J_EASE, SA.J_BUILD)
    else:
      goal = float(np.interp(v_ego, SA.END_CURVE_V, SA.END_CURVE_A))
      if self.phase == 'brake':
        a_end = float(np.interp(SA.END_V, SA.END_CURVE_V, SA.END_CURVE_A))
        t_rel = max(a_end - self.a, 0.0) / SA.J_RELEASE
        dv = -(self.a + a_end) / 2.0 * t_rel
        if v_ego - SA.END_V <= dv or (v_ego < SA.END_CURVE_V[-1] and self.a >= goal):
          self.phase = 'release'
      if self.phase == 'brake':
        out = self._step(-SA.K_NEED * need, SA.J_EASE, SA.J_BUILD)
      else:
        target = self.a if need > -self.a + 0.3 else min(goal, self.a + SA.J_RELEASE * self.dt)
        self.a = out = max(target, self.a)
    if need >= SA.URGENT_NEED and a_mpc < out:
      self.a = out = a_mpc
    return out


def test_exp_differs_only_in_the_wait_feedback():
  # owner 2026-11-07: experimental and normal share the stop target; only the WAIT feedback (needs a_ego) is experimental
  rng = np.random.default_rng(7)
  for _ in range(200):
    on, offm = StopApproach(DT), StopApproach(DT)
    v, d, vl = rng.uniform(1, 20), rng.uniform(5, 120), rng.choice([0.0, 0.0, 3.0])
    for _ in range(300):
      a_mpc = rng.uniform(-3.5, 0.5)
      ln = lead(d, vl, status=rng.random() > 0.05)
      assert on.update(v, ln, a_mpc, False, True, None) == offm.update(v, ln, a_mpc, False, False, None)
      v = max(0.0, v + rng.uniform(-0.15, 0.05)); d = max(0.5, d - v * DT + vl * DT)


def test_stopped_lead_starts_earlier_than_before_11_07():
  # 23:09:52: the brake started at 38.6 m from 43 km/h; the start now looks 1.0 s ahead (was 0.5 s)
  def start_d(cls):
    sa, v, x = cls(DT), 43 / 3.6, 0.0
    while True:
      sa.update(v, lead(110.0 - x), -0.3, False)
      if sa.phase == 'brake':
        return 110.0 - x
      x += v * DT
  assert start_d(StopApproach) > start_d(_Before11_07) + 4.0


def test_exp_wait_follows_the_real_decel():
  # 11-06 21:51: 8 km/h, stopped lead 12.7 m ahead, the car slows -0.8 on the -0.30 lift (uphill) -> the lift eases
  sa = StopApproach(DT)
  outs = [sa.update(8.3 / 3.6, lead(12.7), -0.3, False, True, -0.8) for _ in range(20)]
  assert sa.phase == 'wait'
  assert outs[-1] > SA.WAIT_ACCEL + 0.2 and max(outs) <= 0.0  # eased, never gas
  assert np.max(np.diff(outs)) <= SA.WAIT_FB_RATE * DT + 1e-9
  # experimental off: unchanged light lift
  sa = StopApproach(DT)
  assert sa.update(8.3 / 3.6, lead(12.7), -0.3, False, False, -0.8) == SA.WAIT_ACCEL
  # the car no longer slows harder than needed -> back to the light lift
  sa = StopApproach(DT)
  for _ in range(20):
    sa.update(8.3 / 3.6, lead(12.7), -0.3, False, True, -0.8)
  for _ in range(20):
    a = sa.update(8.3 / 3.6, lead(12.7), -0.3, False, True, -0.1)
  assert abs(a - SA.WAIT_ACCEL) < 1e-9


def test_lead_that_stops_while_braking_keeps_the_brake():
  # 11-07 000000f0 23:13:04: 15 km/h, the lead (moving) drops under LEAD_STOPPED_V 14.2 m ahead, plan -1.07:
  # no lift to WAIT_ACCEL, BRAKE on the need straight away, smooth ease to the end
  sa = StopApproach(DT)
  sa.update(15 / 3.6, lead(14.2, v=3.0), -1.07, False)
  a = sa.update(15 / 3.6, lead(14.2), -1.07, False)
  assert sa.phase == 'brake' and a < -0.9
  v, x, out = 15 / 3.6, 0.0, []
  for _ in range(200):
    a = sa.update(v, lead(14.2 - x), -0.8, False)
    out.append(a); v = max(0.0, v + a * DT); x += v * DT
    if v <= 0.3:
      break
  assert max(out[:20]) < -0.8                                      # held near the need at 15 -> 11 km/h (was -0.30)
  assert np.max(np.diff(out)) <= SA.J_RELEASE * DT + 1e-9          # eases, no jump
  # a lead first seen already stopped is unchanged: WAIT
  sa = StopApproach(DT)
  sa.update(15 / 3.6, lead(14.2), -1.07, False)
  assert sa.phase == 'wait'



def test_slow_lead_braking_to_a_stop_aims_at_its_stop_point():
  # 13:05 route 000000fa: 9.5 km/h, lead 5.0 m ahead at 6.8 km/h braking -1.76 -> it stands ~1.0 m further
  from types import SimpleNamespace as N2
  from dragonpilot.selfdrive.controls.lib import stop_approach as SA2
  sa = SA2.StopApproach(0.05)
  ld = N2(status=True, dRel=5.0, vLead=6.8 / 3.6, aLeadK=-1.76)
  sa.update(9.5 / 3.6, N2(status=True, dRel=5.3, vLead=7.5 / 3.6, aLeadK=-0.1), -1.5, False)   # tracked moving before
  out = [sa.update(9.5 / 3.6, ld, -2.06, False) for _ in range(10)][-1]
  need = (9.5 / 3.6) ** 2 / (2 * (5.0 + (6.8 / 3.6) ** 2 / (2 * 1.76) - SA2.STOP_GAP))
  assert sa.stopping and out > -2.06                       # lighter than the MPC's 6 m aim
  assert out == pytest.approx(-SA2.K_NEED * need, abs=0.05)


def test_slow_lead_not_braking_stays_with_mpc():
  from types import SimpleNamespace as N2
  from dragonpilot.selfdrive.controls.lib import stop_approach as SA2
  sa = SA2.StopApproach(0.05)
  ld = N2(status=True, dRel=8.0, vLead=10 / 3.6, aLeadK=-0.1)
  assert sa.update(12 / 3.6, ld, -0.4, False) == -0.4
  assert not sa.stopping


def test_latched_slow_lead_that_stops_braking_hands_back():
  # 00000103 16:13:24.2: 5.6 km/h, lead 7.5 m at 9.7 km/h braking -0.69 -> latched; by 16:13:25.5 it brakes only
  # -0.22 and keeps rolling at 8-9 km/h -> no longer a stop point, back to the MPC (smoothly)
  from types import SimpleNamespace as N2
  from dragonpilot.selfdrive.controls.lib import stop_approach as SA2
  sa = SA2.StopApproach(0.05)
  sa.update(6.2 / 3.6, N2(status=True, dRel=7.3, vLead=10.1 / 3.6, aLeadK=-0.58), -0.55, False)
  sa.update(5.6 / 3.6, N2(status=True, dRel=7.5, vLead=9.7 / 3.6, aLeadK=-0.69), -0.50, False)
  assert sa.stopping
  sa.update(4.6 / 3.6, N2(status=True, dRel=8.4, vLead=9.3 / 3.6, aLeadK=-0.35), -0.10, False)
  assert sa.stopping                                       # still braking >= A_KEEP: kept
  outs = [sa.update(4.1 / 3.6, N2(status=True, dRel=9.1, vLead=8.9 / 3.6, aLeadK=-0.22), 0.2, False) for _ in range(20)]
  assert not sa.stopping
  assert outs[-1] == 0.2                                   # back to the MPC
  assert np.max(np.diff(outs)) <= SA2.J_HANDBACK * 0.05 + 1e-9
