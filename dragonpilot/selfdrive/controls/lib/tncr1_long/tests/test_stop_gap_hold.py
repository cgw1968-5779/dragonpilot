# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from cereal import log
from dragonpilot.selfdrive.controls.lib.tncr1_long.long_ext import LeadView
from dragonpilot.selfdrive.controls.lib.tncr1_long.stop_gap_hold import StopGapHold


def lead(d, v=0.0, present=True):
  l = log.RadarState.LeadData.new_message()
  l.status = present  # dp schema: status (tncr1: present)
  l.vLead = v
  l.dRel = d
  return LeadView(l)


def run(h, n, accel, v, a_ego, l, standstill=False):
  out = accel
  for _ in range(n):
    out = h.update(accel, v, a_ego, standstill, l, True)
  return out


V12 = 12 / 3.6  # 3.33 m/s: at -1.2 the stop takes 4.6 m


def test_passes_through_when_the_stop_is_safe():
  h = StopGapHold()
  assert run(h, 200, -1.5, V12, -1.5, lead(15.0)) == -1.5
  assert run(h, 10, -1.0, V12, -1.5, lead(15.0)) == -1.0     # easing with room to spare: untouched
  assert run(StopGapHold(), 200, -1.0, V12, -1.0, lead(5.0, v=3.0)) == -1.0  # moving lead: not this rule
  assert run(StopGapHold(), 200, -1.0, 35 / 3.6, -1.0, lead(5.0)) == -1.0    # above 30 km/h


def test_holds_instead_of_easing_when_too_close():
  h = StopGapHold()
  run(h, 150, -1.5, V12, -1.2, lead(7.0))                  # 7 - 3.33^2 / 2.4 = 2.4 m < 3.5
  assert h.holding
  assert run(h, 10, -1.2, V12, -1.2, lead(7.0)) == -1.5    # the plan eases: held
  assert run(h, 10, -1.8, V12, -1.2, lead(7.0)) == -1.8    # firmer is always allowed
  assert run(h, 10, -1.2, V12, -1.2, lead(7.0)) == -1.8


def test_not_at_the_first_touch():
  h = StopGapHold()
  assert run(h, 50, -0.9, 9 / 3.6, 0.0, lead(9.0)) == -0.9  # just engaged, still within the first second
  assert run(h, 10, -0.6, 9 / 3.6, 0.0, lead(9.0)) == -0.6  # the first touch may ease


def test_linear_release_once_safe_again():
  h = StopGapHold()
  run(h, 150, -1.5, V12, -1.2, lead(7.0))
  out = run(h, 20, -1.0, V12, -1.2, lead(20.0))            # safe again: releases at 1.0 m/s^3, 0.2 s -> 0.2
  assert abs(out - (-1.3)) < 1e-6
  assert run(h, 100, -1.0, V12, -1.2, lead(20.0)) == -1.0  # back on the plain request


def test_2_to_0_end_untouched():
  # the hold's own blend (the tncr roll guard, which acts this close to a stopped lead, is tested below)
  import dragonpilot.selfdrive.controls.lib.tncr1_long.stop_gap_hold as m
  saved, m.ROLL_TRIGGER = m.ROLL_TRIGGER, -99.0
  try:
    h = StopGapHold()
    run(h, 150, -1.5, V12, -1.2, lead(5.0))
    assert abs(run(h, 1, -0.8, 4.5 / 3.6, -1.0, lead(2.0)) - (-1.15)) < 1e-6  # 4.5 km/h: half way back to the request
    assert run(h, 1, -0.5, 2.0 / 3.6, -0.6, lead(1.5)) == -0.5                # 2 km/h and below: the end line as it is
    assert run(h, 1, -0.4, 0.0, 0.0, lead(1.5), standstill=True) == -0.4
    assert h.held is None
  finally:
    m.ROLL_TRIGGER = saved


def test_tncr_urgent_lead_brake_holds_only_below_2m():
  # owner 10-15: in a hard lead brake the stop may end down to 2 m; the MPC's emergency trim aims ~2.4 m there
  from types import SimpleNamespace as NS
  from dragonpilot.selfdrive.controls.lib.tncr1_long.stop_gap_hold import StopGapHold
  h = StopGapHold()
  lead_braking = NS(present=True, dRel=20.0, vLead=8.0, aLeadK=-2.3)
  for _ in range(50):
    h.update(-2.4, 12.0, -2.3, False, lead_braking, True)    # urgent: lead -2.3 and we ask -2.4
  assert h.urgent
  stopped = NS(present=True, dRel=3.0, vLead=0.0, aLeadK=0.0)
  for _ in range(300):
    out = h.update(-0.8, 3.0 / 3.6, -0.8, False, stopped, True)  # predicted ~2.6 m: below 3.5 but above 2.0
  assert out == -0.8 and not h.holding


def sim_roll(h, accel_fn, d0=3.57, creep=0.45, n=1500):
  """Cross creeping off a standstill behind a stopped lead (route 0000006b 09:21): plant decel ~ 1.2|u| - creep below
  5 km/h, 0.2 s delay, 0.6 s lag; accel_fn(i, v) is the request reaching the guard; returns (closest gap, trace)"""
  v = a = x = 0.0
  buf, trace = [0.0] * 20, []
  for i in range(n):
    ss = v < 0.01
    cmd = h.update(accel_fn(i, v), v, a, ss, lead(d0 - x, v=0.25), True)
    trace.append((v, d0 - x, cmd))
    buf.append(cmd)
    u = buf.pop(0)
    tgt = (1.2 * u + creep) if (v < 5 / 3.6 and u < 0) else u
    if ss and tgt < 0:
      tgt = 0.0
    a += (tgt - a) * 0.01 / 0.6
    v = max(v + a * 0.01, 0.0)
    x += v * 0.01
  return min(d for _, d, _ in trace), trace


def false_launch(i, v):
  # route 0000006b 09:21: the plan launched (+0.5 for 0.8 s) toward a lead that never moved, then asked only -0.24
  return 0.5 if i < 80 else -0.24


def test_roll_guard_keeps_the_car_off_a_lead_that_never_moved():
  closest, trace = sim_roll(StopGapHold(), false_launch)
  assert closest >= 1.8, closest
  # and once it stands again it stays put, held firmly, while the lead does not move
  tail = trace[-300:]
  assert all(v < 0.01 for v, _, _ in tail)
  assert tail[-1][2] <= -1.2 + 1e-6


def test_roll_guard_off_without_it_the_car_reaches_the_lead():
  import dragonpilot.selfdrive.controls.lib.tncr1_long.stop_gap_hold as m
  saved = m.ROLL_TRIGGER
  m.ROLL_TRIGGER = -99.0
  try:
    closest, _ = sim_roll(StopGapHold(), false_launch)
  finally:
    m.ROLL_TRIGGER = saved
  assert closest < 0.5


def test_roll_guard_leaves_a_normal_launch_alone():
  # the lead pulls away (vLead >= 0.5): the launch passes untouched
  h = StopGapHold()
  outs = [h.update(0.8, 0.5 * i * 0.01, 0.8, i == 0, lead(3.5 + 0.8 * i * 0.01, v=0.8), True) for i in range(200)]
  assert all(o == 0.8 for o in outs)


def test_roll_guard_leaves_an_ordinary_stop_alone():
  # firm enough to stop ~3 m out: never touched (the end of the stop keeps the owner's end force)
  h = StopGapHold()
  v, d = 8 / 3.6, 6.0
  while v > 0:
    req = -(v ** 2 / (2 * (d - 3.0)))
    assert h.update(req, v, req, False, lead(d), True) == req or h.roll_out is None
    d -= v * 0.01
    v = max(v + req * 0.01, 0.0)
  assert h.roll_out is None


def test_roll_guard_hold_releases_when_the_lead_moves():
  h = StopGapHold()
  sim_roll(h, false_launch)
  assert h.roll_caught
  outs = [h.update(0.5, 0.0, 0.0, True, lead(2.2, v=1.0), True) for _ in range(200)]
  assert outs[-1] == 0.5
  steps = [b - a for a, b in zip(outs, outs[1:], strict=False)]
  assert max(steps) <= 1.5 * 0.01 + 1e-9                     # a linear release, no step


def test_roll_guard_targets_2m_in_an_urgent_stop():
  # tncr 2026-10-15 evening (17:16:08): an urgent lead brake ending at 3.1 m / 6 km/h must not be driven to -2.x by the
  # guard's 2.5 m target; with the urgent 2.0 m gap the guard asks far less
  import dragonpilot.selfdrive.controls.lib.tncr1_long.stop_gap_hold as m
  v, d = 6.0 / 3.6, 3.14
  plain = StopGapHold()
  out_plain = run(plain, 60, -0.97, v, -1.1, lead(d))                     # 0.6 s at the same point: where each settles
  urgent = StopGapHold()
  urgent.urgent = True
  out_urgent = run(urgent, 60, -0.97, v, -1.1, lead(d))
  assert out_plain < -0.97 and plain.roll_out is not None                   # the 2.5 m guard acts
  assert out_urgent > out_plain + 0.3, (out_plain, out_urgent)               # the urgent guard asks much less
  assert m.ROLL_GAP_URGENT == 2.0
