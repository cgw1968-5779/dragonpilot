# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

# Dead band on the PCM pitch compensation (tncr, Corolla Cross; owner 2026-10-15 night: "at 70 km/h cruise, why does the
# throttle go up and down?"). The stock carcontroller amplifies the request by sin(high-passed pitch) * g so the PCM
# does not lag on hills. On a flat road the band-passed IMU pitch still moves a few hundredths of a m/s^2 with every
# road ripple, and the hybrid flips between slight drive and slight regen with it. Route 0000007d 23:23-23:26 (set 66,
# 63 km/h, no lead, 10 s blocks): the planner's accel std 0.013-0.06, the command sent to the PCM std 0.02-0.085, sign
# flips 1-20 per 10 s. Altis 2026-09-27 (route 000000a5 at 79 km/h): out std 0.044 vs plan 0.012, |pitch comp| p50 0.032,
# p90 0.076, p99 0.113; the residual (out - cmd) was 0.99 correlated with the compensation. A soft dead band of
# PITCH_COMP_DEADBAND removes that part and leaves real grade changes (> 0.1 m/s^2) compensated, less the band:
# sign(x) * max(|x| - DEADBAND, 0). Projected on the Altis data: out std 0.044 -> 0.015, flips 27 -> 11 per minute.
PITCH_COMP_DEADBAND = 0.08  # m/s^2


def apply_pitch_deadband(pitch_compensation: float) -> float:
  return math.copysign(max(abs(pitch_compensation) - PITCH_COMP_DEADBAND, 0.0), pitch_compensation)
