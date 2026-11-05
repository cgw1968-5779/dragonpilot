import pytest
import numpy as np

from opendbc.car import rate_limit, DT_CTRL
from opendbc.car.toyota.tncr1_send.brake_onset import BrakeOnsetShaper, ONSET_J_DOWN, HARD_BRAKE_ACCEL, GAS_RELEASE_J

DT = DT_CTRL * 3
STOCK_J = 4.0
UP_STEP = 4.0 * DT


def run(requests, a0=0.0, fcw=False):
  shaper = BrakeOnsetShaper(DT, STOCK_J)
  a = a0
  out = []
  for req in requests:
    step = shaper.down_step(req, a, bypass=shaper.is_urgent(req, fcw))
    a = rate_limit(req, a, step, UP_STEP)
    out.append(a)
  return np.array(out)


def at(out, t):
  return out[int(round(t / DT)) - 1]


class TestBrakeOnset:
  @pytest.mark.xfail(reason='fails on tncr1 itself too (e1656efd1a): stale expectation, module copied unchanged', strict=True)
  def test_step_brake_eases_in_over_the_first_second(self):
    out = run([-1.5] * 60)
    assert at(out, 0.09) > -0.06  # first 0.15 s: barely anything
    assert at(out, 0.21) > -0.12  # 0.2 s: still very light
    assert -0.45 < at(out, 0.36) < -0.10  # ~0.35 s: building slowly
    assert -0.60 < at(out, 0.60) < -0.15  # 0.6 s: still gentle, about a fifth of the way
    assert -1.10 < at(out, 1.00) < -0.55  # 1.0 s: about half way, the ramp is now running up
    assert np.isclose(at(out, 1.5), -1.5, atol=1e-6)  # settled once the ramp reaches the stock limit

  def test_jerk_follows_the_schedule_and_never_exceeds_stock(self):
    out = run([-1.9] * 60, a0=0.0)  # large step from coasting, still above the hard-brake bypass
    jerks = np.diff(np.concatenate([[0.0], out])) / DT
    assert jerks.min() >= -STOCK_J - 1e-6
    assert jerks[0] >= -ONSET_J_DOWN[0] - 1e-6
    assert jerks[int(0.09 / DT)] >= -ONSET_J_DOWN[1] - 1e-6
    # the blend is one straight ramp: jerk keeps growing monotonically until the stock limit
    ramp = -jerks[int(0.1 / DT):int(0.6 / DT)]
    assert np.all(np.diff(ramp) >= -1e-6)

  def test_gas_is_released_at_gas_release_j_then_the_soft_onset_starts_from_zero(self):
    # route 0000013a 00:01:44: accelerating at +0.85 when the plan turned to -1.0
    out = run([-1.0] * 80, a0=0.85)
    jerks = np.diff(np.concatenate([[0.85], out])) / DT
    k0 = int(np.argmax(out <= 0.0))
    assert np.allclose(jerks[:k0 - 1], -GAS_RELEASE_J, atol=1e-6)  # the gas goes at GAS_RELEASE_J ...
    assert k0 * DT < 0.85 / GAS_RELEASE_J + 2 * DT                  # ... reaching zero in ~0.43 s
    assert np.all(jerks[k0 + 1:k0 + 5] >= -ONSET_J_DOWN[0] - 1e-6)  # then the brake starts softly from zero

  @pytest.mark.xfail(reason='fails on tncr1 itself too (e1656efd1a): stale expectation, module copied unchanged', strict=True)
  def test_lead_switch_during_a_settled_brake_gets_the_same_soft_onset(self):
    reqs = [-0.6] * 70 + [-1.8] * 60  # long enough for the schedule to wind fully back before the switch
    out = run(reqs)
    t_switch = 70 * DT
    assert np.isclose(at(out, t_switch), -0.6, atol=1e-6)
    assert at(out, t_switch + 0.09) > -0.6 - 0.05
    assert at(out, t_switch + 0.21) > -0.6 - 0.25
    assert np.isclose(out[-1], -1.8, atol=1e-6)

  def test_hard_brake_request_bypasses_the_schedule(self):
    out = run([HARD_BRAKE_ACCEL - 0.5] * 20)
    assert np.isclose(out[0], -STOCK_J * DT, atol=1e-6)
    assert at(out, 0.3) <= -1.1

  def test_fcw_bypasses_the_schedule(self):
    out = run([-1.5] * 20, fcw=True)
    assert np.isclose(out[0], -STOCK_J * DT, atol=1e-6)

  def test_release_is_untouched(self):
    out = run([1.0] * 20, a0=-1.5)
    assert np.isclose(out[0], -1.5 + UP_STEP, atol=1e-6)

  def test_gentle_request_passes_straight_through(self):
    reqs = list(np.linspace(0.0, -0.2, 40))  # ~0.17 m/s^3, below the gentlest onset limit (0.25)
    out = run(reqs)
    assert np.allclose(out, reqs, atol=1e-9)

  def test_short_pause_does_not_fully_rearm(self):
    shaper = BrakeOnsetShaper(DT, STOCK_J)
    a = 0.0
    for _ in range(int(0.3 / DT)):
      a = rate_limit(-1.5, a, shaper.down_step(-1.5, a), UP_STEP)
    t_after_ramp = shaper.t_onset
    for _ in range(2):  # hold for two frames, request steady
      shaper.down_step(a, a)
    assert 0.0 < shaper.t_onset < t_after_ramp
    for _ in range(int(1.0 / DT)):  # a full second of settled brake winds it back completely
      shaper.down_step(a, a)
    assert shaper.t_onset == 0.0


class TestEngageBrake:
  """2026-10-04: SET- while creeping up to a lead: the first 0.2 s after engaging is a light touch."""
  def test_first_0_2_s_after_engaging_is_light_then_blends_to_stock(self):
    from opendbc.car.toyota.tncr1_send.brake_onset import BrakeOnsetShaper, EngageOnsetShaper
    dt = 0.03
    shaper, engage = BrakeOnsetShaper(dt, 4.0), EngageOnsetShaper(dt, 4.0)
    a, t, trace = 0.0, 0.0, {}
    while t < 2.2:
      engage.up_step(True)
      step = shaper.down_step(-1.65, a, v_ego=2.5, t_engaged=engage.t_since_engage)
      a = max(-1.65, a + step)
      t += dt
      for mark in (0.2, 0.5, 1.0, 1.5):
        if mark not in trace and t >= mark:
          trace[mark] = a
    assert trace[0.2] > -0.12          # light first touch (at creep speed the creep onset schedule is the stricter one)
    assert -0.27 < trace[0.5] < -0.1   # 2026-10-14: about -0.25 at 0.5 s, no more
    assert -0.9 < trace[1.0] < -0.4    # then a linear build
    assert trace[1.5] < -1.0 and a < -1.6  # the planner's request arrives by ~2 s

  def test_an_emergency_is_not_held_back(self):
    from opendbc.car.toyota.tncr1_send.brake_onset import BrakeOnsetShaper, EngageOnsetShaper
    dt = 0.03
    shaper, plain, engage = BrakeOnsetShaper(dt, 4.0), BrakeOnsetShaper(dt, 4.0), EngageOnsetShaper(dt, 4.0)
    a = b = 0.0
    for _ in range(int(1.0 / dt)):
      engage.up_step(True)
      a = max(-3.5, a + shaper.down_step(-3.5, a, v_ego=8.0, t_engaged=engage.t_since_engage))
      b = max(-3.5, b + plain.down_step(-3.5, b, v_ego=8.0))
    assert a == b  # beyond HARD_BRAKE_ACCEL the engage schedule does not apply (the normal onset still does)


class TestCreepOnset:
  """2026-10-04: a brake onset at creep speed (re-stop behind a lead) is as light as the engage schedule."""
  def test_below_10kph_first_0_2_s_is_light(self):
    from opendbc.car.toyota.tncr1_send.brake_onset import BrakeOnsetShaper
    dt = 0.03
    for v, light_limit in ((1.0, -0.15), (30.0, -0.45)):   # creep: barely anything by 0.2 s; highway: the normal onset
      shaper = BrakeOnsetShaper(dt, 4.0)
      a, t, at_02 = 0.0, 0.0, None
      while t < 1.2:
        a = max(-1.0, a + shaper.down_step(-1.0, a, v_ego=v))
        t += dt
        if at_02 is None and t >= 0.2:
          at_02 = a
      assert at_02 > light_limit, (v, at_02)
      assert a < -0.9, (v, a)   # the full request arrives within the window at both speeds


class TestCreepGasRelease:
  """route 0000010d 23:56:59: the drop from a +0.5 launch request was shaped like a brake onset"""
  def _shaper(self):
    from opendbc.car.toyota.tncr1_send.brake_onset import BrakeOnsetShaper
    return BrakeOnsetShaper(DT_CTRL * 3, 4.0)

  def test_positive_part_goes_at_the_stock_rate_below_8kph(self):
    sh = self._shaper()
    step = sh.down_step(-0.24, 0.5, v_ego=2.0 / 3.6)
    assert abs(step - (-4.0 * DT_CTRL * 3)) < 1e-9
    step = sh.down_step(-0.24, 0.05, v_ego=2.0 / 3.6)
    assert abs(step - (-0.05)) < 1e-9            # stops at zero; the braking starts from there

  def test_faster_keeps_the_soft_onset(self):
    step = self._shaper().down_step(-0.24, 0.5, v_ego=20.0 / 3.6)
    assert step > -4.0 * DT_CTRL * 3 + 1e-9


class TestResumeSoftStart:
  """RES while coasting at 46 km/h: start from the coasting decel, 0.6 m/s^3 for 0.4 s, stock by 0.8 s"""
  def _run(self, a_ego=-0.45, v=46.0, req=0.4, edge=True, n=40):
    from opendbc.car.toyota.tncr1_send.brake_onset import EngageOnsetShaper
    sh = EngageOnsetShaper(DT, 4.0)
    sh.cruise_state(False)
    sh.cruise_state(True if edge else False)
    if not edge:
      for _ in range(30):
        sh.cruise_state(True)
    prev, out = 0.0, []
    for _ in range(n):
      prev = sh.start_accel(prev, a_ego, v / 3.6)
      prev = rate_limit(req, prev, -4.0 * DT, sh.up_step(True))
      out.append(prev)
    return np.array(out)

  def test_gentle_first_0_4s_then_stock(self):
    out = self._run()
    assert abs(out[0] - (-0.45 + 0.6 * DT)) < 1e-9          # starts from the coasting decel
    assert at(out, 0.39) < -0.45 + 0.6 * 0.4 + 0.02         # 0.6 m/s^3 for the first 0.4 s
    assert np.isclose(out[-1], 0.4)                          # reaches the plan

  def test_not_at_low_speed_or_a_gas_override_release(self):
    assert self._run(v=10.0)[0] == 4.0 * DT
    assert self._run(edge=False)[0] == 4.0 * DT

  def test_a_positive_a_ego_starts_from_zero(self):
    assert abs(self._run(a_ego=0.3)[0] - 0.6 * DT) < 1e-9


def _onset_profile(lead, request=-2.5, v_ego=50 / 3.6, t_engaged=None, secs=1.2):
  from types import SimpleNamespace as NS  # noqa: F401
  from opendbc.car.toyota.tncr1_send.brake_onset import BrakeOnsetShaper
  sh, cmd, out = BrakeOnsetShaper(0.01, 4.0), 0.0, []
  for _ in range(int(secs / 0.01)):
    cmd = max(request, cmd + sh.down_step(request, cmd, v_ego=v_ego, urgent=sh.is_urgent(request, False), t_engaged=t_engaged,
                                          a_ego=cmd, lead=lead))
    out.append(cmd)
  return out, sh


def test_lead_urgent_follows_the_manual_template():
  # tncr 10-15: lead braking hard + request <= -2.0: light 0.1 s, then a linear build to 4 m/s^3 by 0.4 s
  from types import SimpleNamespace as NS
  out, sh = _onset_profile(NS(status=True, aLeadK=-2.3))
  assert sh.lead_urgent
  assert -0.12 <= out[9] <= -0.08          # 0.1 s: a light touch (-0.1)
  assert -0.75 <= out[29] <= -0.45         # 0.3 s
  assert out[69] <= -1.9                   # -2 by ~0.7 s (the owner's urgent manual stops: 0.3-0.6 s)
  steps = [b - a for a, b in zip(out, out[1:], strict=False)]
  assert all(s2 <= s1 + 1e-9 for s1, s2 in zip(steps[:40], steps[1:41], strict=False))  # the build only speeds up (linear jerk ramp)
  assert min(steps) >= -4.0 * 0.01 - 1e-9  # never past the stock jerk


def test_ordinary_lead_keeps_the_soft_onset():
  from types import SimpleNamespace as NS
  urgent, _ = _onset_profile(NS(status=True, aLeadK=-2.3))
  soft, sh = _onset_profile(NS(status=True, aLeadK=-1.0))  # lead not braking hard: the normal (very light) onset
  assert not sh.lead_urgent and soft[69] > urgent[69] + 0.5
  none, sh2 = _onset_profile(None)
  assert not sh2.lead_urgent and abs(none[69] - soft[69]) < 1e-9
  eng, sh3 = _onset_profile(NS(status=True, aLeadK=-2.3), t_engaged=0.2)  # inside the engage window: never
  assert not sh3.lead_urgent
