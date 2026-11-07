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


def test_exp_off_is_unchanged():
  # owner 2026-11-07: tuning experimental mode must not touch the non-experimental longitudinal - bit for bit
  rng = np.random.default_rng(7)
  for _ in range(200):
    new, old = StopApproach(DT), _Before11_07(DT)
    v, d, vl = rng.uniform(1, 20), rng.uniform(5, 120), rng.choice([0.0, 0.0, 3.0])
    for _ in range(300):
      a_mpc, a_ego = rng.uniform(-3.5, 0.5), rng.uniform(-2.0, 0.5)
      ln = lead(d, vl, status=rng.random() > 0.05)
      assert new.update(v, ln, a_mpc, False, False, a_ego) == old.update(v, ln, a_mpc, False)
      assert new.phase == old.phase
      v = max(0.0, v + rng.uniform(-0.15, 0.05)); d = max(0.5, d - v * DT + vl * DT)


def test_exp_starts_earlier():
  # experimental looks 1.0 s ahead: the BRAKE phase starts further back than with experimental off
  def start_d(exp):
    sa, v, x = StopApproach(DT), 50 / 3.6, 0.0
    while True:
      sa.update(v, lead(100.0 - x), -0.3, False, exp, None)
      if sa.phase == 'brake':
        return 100.0 - x
      x += v * DT
  assert start_d(True) > start_d(False) + 3.0


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
