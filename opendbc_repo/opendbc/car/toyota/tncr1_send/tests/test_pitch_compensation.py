# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from opendbc.car.toyota.tncr1_send.pitch_compensation import apply_pitch_deadband, PITCH_COMP_DEADBAND


def test_small_ripple_is_removed_and_real_grade_passes_less_the_band():
  for x in (0.0, 0.03, -0.05, 0.079, -0.08):
    assert apply_pitch_deadband(x) == 0.0
  assert abs(apply_pitch_deadband(0.2) - (0.2 - PITCH_COMP_DEADBAND)) < 1e-12
  assert abs(apply_pitch_deadband(-0.5) - (-0.5 + PITCH_COMP_DEADBAND)) < 1e-12
  assert abs(apply_pitch_deadband(1.5) - (1.5 - PITCH_COMP_DEADBAND)) < 1e-12


def test_continuous_at_the_band_edge():
  assert abs(apply_pitch_deadband(PITCH_COMP_DEADBAND + 1e-6) - 1e-6) < 1e-9
