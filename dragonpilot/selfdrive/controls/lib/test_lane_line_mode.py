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
from types import SimpleNamespace

import numpy as np

from dragonpilot.selfdrive.controls.lib.lane_centering import LaneCentering, LINE_T_LOOK, LINE_W_MAX, LINE_PRED_T

DT = 0.01
X = [0., 2., 4., 6., 8., 10., 12., 15., 18., 21., 25., 30., 35., 40., 45., 50., 55., 60., 70., 80.]


def _model(offset=0.0, curv=0.0, prob=0.9):
  """lane centre at y = offset + curv / 2 * x^2 (model y: positive = right)"""
  centre = np.array([offset + 0.5 * curv * x ** 2 for x in X])
  left = SimpleNamespace(x=X, y=list(centre - 1.65))
  right = SimpleNamespace(x=X, y=list(centre + 1.65))
  return SimpleNamespace(laneLines=[left, left, right, right], laneLineProbs=[prob, prob, prob, prob])


def _run(lc, md, v, k_model, seconds, lane_changing=False):
  out = 0.0
  for _ in range(int(seconds / DT)):
    out = lc.update(md, v, True, False, 0.0, k_model, lane_changing)
  return out


def test_takes_over_with_confident_lines_and_pursues_the_centre():
  lc = LaneCentering(DT)
  v = 100 / 3.6
  out = _run(lc, _model(offset=0.2), v, 0.0, 3.0)  # car 0.2 m LEFT of centre, model goes straight
  assert lc.line_w == LINE_W_MAX
  look = v * LINE_T_LOOK
  # dpcr: pursuit from the pose LINE_PRED_T ahead on the arc of the last command (the car already turning right)
  xd, k = v * LINE_PRED_T, lc.k_prev
  dy = 0.2 - k * xd ** 2 / 2 - k * xd * look
  pursuit = 2 * dy / (look ** 2 + dy ** 2)  # steers right, pure pursuit
  assert 0.0 < pursuit < 2 * 0.2 / (look ** 2 + 0.04)  # less than from the current pose: no overshoot push
  assert abs(out - (LINE_W_MAX * pursuit + (1 - LINE_W_MAX) * lc.correction)) < 1e-5


def test_follows_the_lane_curvature_not_the_model():
  lc = LaneCentering(DT)
  v = 100 / 3.6
  out = _run(lc, _model(curv=1 / 1000), v, 1 / 800, 4.0)  # model asks tighter than the lane (4 s: the command history settles)
  assert abs(out - (LINE_W_MAX * (1 / 1000 - 1 / 800) + (1 - LINE_W_MAX) * lc.correction)) < 2e-5


def test_no_line_mode_below_70_kph_or_without_model_curvature():
  lc = LaneCentering(DT)  # 2026-10-27: fades in 70 -> 80 km/h (was 80 -> 90)
  _run(lc, _model(offset=0.2), 68 / 3.6, 0.0, 3.0)
  assert lc.line_w == 0.0
  lc_half = LaneCentering(DT)
  _run(lc_half, _model(offset=0.2), 75 / 3.6, 0.0, 3.0)
  assert abs(lc_half.line_w - 0.5 * LINE_W_MAX) < 1e-6
  lc_full = LaneCentering(DT)
  _run(lc_full, _model(offset=0.2), 82 / 3.6, 0.0, 3.0)
  assert lc_full.line_w == LINE_W_MAX
  lc2 = LaneCentering(DT)
  for _ in range(300):
    out = lc2.update(_model(offset=0.2), 100 / 3.6, True, False)
  assert lc2.line_w == 0.0 and out == lc2.correction


def test_hands_back_on_weak_lines_lane_change_or_far_target():
  v = 100 / 3.6
  for md, k_model, lane_changing in ((_model(prob=0.6), 0.0, False), (_model(), 0.0, True), (_model(curv=1 / 300), 0.0, False)):
    lc = LaneCentering(DT)
    _run(lc, _model(), v, 0.0, 3.0)
    assert lc.line_w == LINE_W_MAX
    _run(lc, md, v, k_model, 0.6, lane_changing)
    assert lc.line_w == 0.0


def test_weight_rises_in_two_seconds():
  lc = LaneCentering(DT)
  _run(lc, _model(), 100 / 3.6, 0.0, 1.0)
  assert 0.45 < lc.line_w < 0.55


def test_weight_capped_at_line_w_max():
  lc = LaneCentering(DT)
  _run(lc, _model(), 100 / 3.6, 0.0, 5.0)
  assert LINE_W_MAX == 0.85 and lc.line_w == LINE_W_MAX  # dpcr 0.85 (tncr1 0.7)


def test_centre_fits_made_once_per_model_frame(monkeypatch):
  import dragonpilot.selfdrive.controls.lib.lane_centering as lcm
  calls = []
  real = np.polyfit
  monkeypatch.setattr(lcm.np, 'polyfit', lambda *a, **k: calls.append(1) or real(*a, **k))
  lc = LaneCentering(DT)
  md = _model(offset=0.2)
  for _ in range(5):  # one 20 Hz model frame = five 100 Hz cycles
    lc.update(md, 100 / 3.6, True, False, 0.0, 0.0, False)
  assert len(calls) == 2  # the near fit (measure) and the line-mode fit, once each
  lc.update(_model(offset=0.1), 100 / 3.6, True, False, 0.0, 0.0, False)  # a new frame refits
  assert len(calls) == 4



def test_constant_command_history_is_the_old_arc():
  from dragonpilot.selfdrive.controls.lib.lane_centering import LINE_PRED_T as PT
  lc = LaneCentering(DT)
  v, k = 100 / 3.6, 1 / 900
  for _ in range(200):
    lc._push_command(k)
  y, psi = lc.predicted_pose(v)
  xd = v * PT
  assert abs(psi - k * xd) < 1e-4 * k * xd + 1e-9
  assert abs(y - k * xd * xd / 2) < 0.02 * k * xd * xd / 2


def test_command_history_sees_a_correction_already_sent():
  # the car was sent a right correction in the last 0.3 s: the pose after the delay is further right than the arc of
  # the (now straight) last command says, so the pursuit asks less
  lc = LaneCentering(DT)
  v = 100 / 3.6
  n = int(round(LINE_PRED_T / DT))
  for i in range(n):
    lc._push_command(2e-4 if i >= n - 30 else 0.0)
  lc._push_command(0.0)
  y, psi = lc.predicted_pose(v)
  assert y > 0.0 and psi > 0.0
