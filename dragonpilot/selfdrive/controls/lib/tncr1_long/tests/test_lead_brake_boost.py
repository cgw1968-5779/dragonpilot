# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from cereal import log
from dragonpilot.selfdrive.controls.lib.tncr1_long.long_ext import LeadView
from dragonpilot.selfdrive.controls.lib.tncr1_long.lead_brake_boost import LeadBrakeBoost


def lead(a, v=0.0, d=10.0, present=True):
  l = log.RadarState.LeadData.new_message()  # real capnp struct (the field is `present`, not `status`)
  l.status = present  # dp schema: status (tncr1: present)
  l.aLeadK = a
  l.vLead = v
  l.dRel = d
  return LeadView(l)


def run(b, n, accel, v, l):
  out = accel
  for _ in range(n):
    out = b.update(accel, v, False, l, True, -3.5)
  return out


V15 = 15 / 3.6
URGENT = lead(-3.0, v=0.0, d=8.0)


def test_no_boost_for_an_ordinary_lead_stop():
  assert run(LeadBrakeBoost(), 300, -1.5, V15, lead(-2.5, v=1.0, d=20.0)) == -1.5   # TTC too long
  assert run(LeadBrakeBoost(), 300, -1.5, V15, lead(-3.0, v=V15 - 5 / 3.6, d=3.0)) == -1.5  # closing too slowly
  assert run(LeadBrakeBoost(), 300, -1.5, V15, lead(-1.5, v=0.0, d=5.0)) == -1.5   # lead brakes only moderately


def test_not_in_the_light_first_phase():
  b = LeadBrakeBoost()
  assert run(b, 300, -0.8, V15, URGENT) == -0.8      # light request: never boosted
  b = LeadBrakeBoost()
  assert run(b, 40, -1.5, V15, URGENT) == -1.5       # firm but the first 0.4 s of braking: not yet
  out = run(b, 100, -1.5, V15, URGENT)               # after 0.5 s it ramps in (0.4 s) to x1.2
  assert abs(out - (-1.8)) < 1e-6


def test_held_after_the_peak_and_faded_at_the_end():
  b = LeadBrakeBoost()
  run(b, 200, -2.5, V15, URGENT)
  assert abs(run(b, 10, -0.9, 8 / 3.6, lead(0.0, d=4.0)) - (-1.08)) < 1e-6  # easing after the peak: still x1.2
  assert abs(run(b, 300, -0.5, 1.8 / 3.6, lead(0.0, d=2.0)) - (-0.5)) < 1e-9  # 2-0 km/h untouched


def test_release_and_limits():
  b = LeadBrakeBoost()
  run(b, 200, -1.5, V15, URGENT)
  assert b.latched
  b.update(-1.5, V15, False, lead(1.0, v=20 / 3.6, d=8.0), True, -3.5)
  assert not b.latched
  assert run(LeadBrakeBoost(), 200, -3.4, V15, URGENT) == -3.5  # clipped to the accel limit
  assert run(LeadBrakeBoost(), 200, 0.5, V15, URGENT) == 0.5    # never touches a positive request
  b4 = LeadBrakeBoost()
  run(b4, 200, -1.5, V15, URGENT)
  b4.update(-1.5, 0.0, True, lead(0.0), True, -3.5)
  assert not b4.latched


def test_no_compounding_through_the_stopping_controller():
  # route 00000159 08:54:37: the boosted request was fed back as prev_accel; below BLEND_V the stopping controller's end
  # request is rate-limited around prev_accel, so x1.2 compounded every frame and ran to -3.5 (hard stop, -3.5 hold).
  # controlsd now keeps last_output_accel un-boosted and boosts only what is sent; reproduce that loop here.
  from types import SimpleNamespace
  from opendbc.car.structs import car
  from dragonpilot.selfdrive.controls.lib.tncr1_long.stopping_controller import StoppingController
  pid = car.CarControl.Actuators.LongControlState.pid
  sc = StoppingController(-1.0)
  b = LeadBrakeBoost()
  for _ in range(100):  # latched earlier, at 15 km/h while braking firmly (as on the road)
    b.update(-2.0, V15, False, URGENT, True, -3.5)
  assert b.gain > 1.1
  last = -2.0
  sent = []
  for k in range(150):
    v = max(9.6 - 0.05 * k, 3.5) / 3.6  # 9.6 -> 3.5 km/h, inside the end-of-stop window
    cs = SimpleNamespace(vEgo=v, standstill=False)
    _, accel = sc.update(pid, pid, cs, -1.37, last, -1.37, (-3.5, 2.0), True)
    last = accel                                                   # un-boosted, as controlsd keeps it
    sent.append(b.update(accel, v, False, URGENT, True, -3.5))   # boosted, only sent
  assert min(sent) > -2.6, min(sent)  # around 1.2x a ~-2 request, never running to -3.5
  assert sent[-1] > -2.0, sent[-1]    # settles near 1.2x the end request (feeding the boosted value back gave -3.5)


def test_hold_off_passes_the_guard_output_unchanged():
  from cereal import log
  b = LeadBrakeBoost()
  l = log.RadarState.LeadData.new_message()
  l.status, l.dRel, l.vLead, l.aLeadK = True, 5.0, 0.0, -3.0
  l = LeadView(l)
  for _ in range(200):  # latch + light phase over + gain ramped to 1.2
    out = b.update(-1.5, 12 / 3.6, False, l, True, -3.5, a_ego=-1.4)
  assert out < -1.5 - 0.2
  assert b.update(-2.15, 4 / 3.6, False, l, True, -3.5, a_ego=-1.8, hold_off=True) == -2.15
  assert b.update(-2.15, 4 / 3.6, False, l, True, -3.5, a_ego=-1.8) < -2.15   # back on when the guard lets go
