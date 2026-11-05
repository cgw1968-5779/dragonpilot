# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from dragonpilot.selfdrive.controls.lib.tncr1_long.stopping_controller import StoppingController

LIM = (-3.5, 2.0)


def cs(v, standstill=False):
  return NS(vEgo=v, aEgo=-0.5, brakePressed=False, standstill=standstill, cruiseState=NS(standstill=False))


def run(sc, n, v, a_target, lead_d, lead_v, prev=-0.5):
  out = prev
  for _ in range(n):
    _, out = sc.update(LongCtrlState.pid, LongCtrlState.pid, cs(v), a_target, out, a_target, LIM, True, lead_d=lead_d, lead_v=lead_v)
  return out


def test_end_line_stays_off_while_the_lead_stop_point_is_far():
  # route 75 20:59:34.4: 9.6 km/h, plan -0.49, lead 12.2 m ahead rolling at 1.1 m/s -> the plan is sent, not the -0.65 line
  sc = StoppingController(-2.0)
  assert abs(run(sc, 50, 9.6 / 3.6, -0.49, 12.2, 1.1) - (-0.49)) < 1e-9
  assert not sc.end_active
  assert abs(run(sc, 50, 4.8 / 3.6, -0.35, 9.5, 0.0) - (-0.35)) < 1e-9       # 20:59:36: lead stopped 9.5 m ahead


def test_end_line_arms_once_the_stop_point_is_near():
  sc = StoppingController(-2.0)
  out = run(sc, 100, 8.0 / 3.6, -0.49, 5.0, 0.0)                                # 8 km/h, stopped lead 5 m: own stop 5 m -> near
  assert sc.end_active and out < -0.49                                           # the line (-0.65 at 5 km/h) is the floor


def test_end_line_arms_without_a_lead_as_before():
  sc = StoppingController(-2.0)
  out = run(sc, 100, 8.0 / 3.6, -0.49, None, None)
  assert sc.end_active and out < -0.49


def test_far_gate_uses_the_leads_run_out():
  sc = StoppingController(-2.0)
  # lead 6 m ahead but still rolling at 3 m/s (runs out 4.5 m more at 1 m/s^2): the stop point is 10.5 m away -> far
  assert abs(run(sc, 50, 8.0 / 3.6, -0.49, 6.0, 3.0) - (-0.49)) < 1e-9
  assert not sc.end_active


def step(sc, v, a_target, prev, has_lead, lead_d=None, lead_v=None):
  return sc.update(LongCtrlState.pid, LongCtrlState.pid, cs(v), a_target, prev, a_target, LIM, has_lead, lead_d=lead_d, lead_v=lead_v)[1]


def test_no_lead_ceiling_starts_at_blend_v():
  # route 000000ac 15:45:23 (experimental, red light, no lead): the e2e plan was still -1.65 at 8 km/h. The no-lead
  # ceiling now starts at BLEND_V, so between 13 and 8 km/h the request already comes down along the line from the
  # request at 13 km/h toward END_HI, eased at END_CAP_RATE - never firmer than the line
  sc = StoppingController(-2.0)
  out = step(sc, 12.9 / 3.6, -2.0, -2.0, False)                                 # arm at 12.9 km/h with -2.0
  assert sc.end_active
  line_10 = float(sc._blend(10.0 / 3.6))
  assert -2.0 < line_10 < sc.END_HI
  outs = [out]
  for _ in range(300):
    outs.append(step(sc, 10.0 / 3.6, -1.9, outs[-1], False))
  assert abs(outs[-1] - line_10) < 1e-6                                          # settles on the line, lighter than the plan
  steps = [b - a for a, b in zip(outs, outs[1:], strict=False)]
  assert max(steps) <= sc.END_CAP_RATE * 0.01 + 1e-9                             # eased, no step


def test_lead_stop_between_13_and_8_kmh_unchanged():
  # with a lead the plan still wins wherever it is firmer than the line
  sc = StoppingController(-2.0)
  out = step(sc, 12.9 / 3.6, -2.0, -2.0, True, 6.0, 0.0)
  for _ in range(200):
    out = step(sc, 10.0 / 3.6, -1.9, out, True, 6.0, 0.0)
  assert abs(out - (-1.9)) < 1e-6
