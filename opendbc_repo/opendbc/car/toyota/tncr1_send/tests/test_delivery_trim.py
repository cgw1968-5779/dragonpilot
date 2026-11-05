# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from opendbc.car.toyota.tncr1_send.delivery_trim import DeliveryTrim, STEADY_V, RELEASE_EXTRA, HARD_CEIL


def settle(t, cmd, v, n=100):
  out = cmd
  for _ in range(n):
    out = t.apply(cmd, v, False, True)
  return out


def test_steady_part_is_off_the_pid_already_closes_it():
  # 10-15 late: the steady residual was double-counted with the stock PID (plan-vs-delivered) -> zeros for now.
  assert settle(DeliveryTrim(), -1.0, 11 / 3.6) == -1.0
  assert settle(DeliveryTrim(), -1.0, 20 / 3.6) == -1.0
  assert settle(DeliveryTrim(), -1.0, 30 / 3.6) == -1.0            # 30 km/h: matched, untouched
  assert settle(DeliveryTrim(), -1.0, 4 / 3.6) == -1.0             # below 6 km/h: nothing (end rules own it)
  assert settle(DeliveryTrim(), -0.2, 11 / 3.6) == -0.2            # a light touch is untouched


def test_release_adds_a_little_more_while_the_command_rises():
  t = DeliveryTrim()
  settle(t, -1.2, 12 / 3.6)
  outs = [t.apply(-1.2 + 0.02 * i, 12 / 3.6, False, True) for i in range(1, 40)]   # releasing at 0.67 m/s^3
  trims = [o - (-1.2 + 0.02 * i) for i, o in enumerate(outs, 1)]
  assert min(trims) <= -(STEADY_V[2] + RELEASE_EXTRA) + 0.03 and min(trims) >= -(STEADY_V[2] + RELEASE_EXTRA) - 1e-9


def test_hard_brake_at_speed_is_lifted_but_stays_hard():
  assert abs(settle(DeliveryTrim(), -2.5, 30 / 3.6) - (-2.5 + 0.25)) < 1e-9
  assert abs(settle(DeliveryTrim(), -3.0, 40 / 3.6) - (-3.0 + 0.35)) < 1e-9
  assert settle(DeliveryTrim(), -1.6, 30 / 3.6) >= -1.6 and settle(DeliveryTrim(), -1.6, 30 / 3.6) <= HARD_CEIL
  assert settle(DeliveryTrim(), -2.5, 10 / 3.6) == -2.5            # below 15 km/h no lift


def test_trim_moves_at_its_rate_and_resets_at_standstill():
  t = DeliveryTrim()
  first = t.apply(-2.5, 30 / 3.6, False, True)
  assert abs(first - (-2.5 + 0.03)) < 1e-9                          # 1.0 m/s^3 * 0.03 s
  settle(t, -2.5, 30 / 3.6)
  assert t.apply(-2.5, 0.0, True, True) == -2.5 and t.trim == 0.0
  assert t.apply(-2.5, 30 / 3.6, False, False) == -2.5
