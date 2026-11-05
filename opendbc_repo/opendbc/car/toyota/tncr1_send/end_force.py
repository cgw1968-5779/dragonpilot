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
# Owner's end-of-stop rule (2026-10-14): the request at the 0 km/h stop is never firmer than -0.30, all modulation above
# 2 km/h. controlsd limits actuators.accel (end_force_limit.py); this keeps the command actually sent to the PCM from
# going firmer than that request below V_LINE while rolling - route 0000015a 18:51:26 the long PID integral made the
# command -0.48 for a -0.46 request at the stop, and 0.22 firmer than the request at 4-5 km/h.
V_LINE = 2.0 / 3.6  # m/s


def end_force_cap(pcm_accel_cmd: float, request: float, v_ego: float, standstill: bool) -> float:
  if standstill or v_ego >= V_LINE or request >= 0.0:
    return pcm_accel_cmd
  return max(pcm_accel_cmd, request)
