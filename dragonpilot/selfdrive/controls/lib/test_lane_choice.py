from types import SimpleNamespace as N

import numpy as np

from dragonpilot.selfdrive.controls.lib import lane_choice as LC

XS = [float(x) for x in np.arange(0.0, 60.0, 2.0)]
V = 40.0 / 3.6


def frame(fid, lines=(-5.0, -1.6, 1.6, 4.8), probs=(0.6, 0.9, 0.9, 0.6), path=0.0):
  """straight lanes; model y positive = right"""
  return N(frameId=fid, laneLines=[N(x=XS, y=[y] * len(XS)) for y in lines], laneLineProbs=list(probs),
           position=N(x=XS, y=[path] * len(XS)), roadEdges=[N(x=XS, y=[-6.0] * len(XS)), N(x=XS, y=[6.0] * len(XS))],
           roadEdgeStds=[0.5, 0.5])


def run(lc, frames, pressed=None, blinker=False, k_act=0.0, sub=5):
  out = []
  for i, md in enumerate(frames):
    p = pressed[i] if pressed is not None else False
    for _ in range(sub):   # 100 Hz control cycles per 20 Hz model frame
      out.append(lc.update(md, V, 0.0, k_act, p, blinker, False))
  return out


def test_no_event_no_output():
  lc = LC.LaneChoice(0.01)
  out = run(lc, [frame(i) for i in range(200)])
  assert max(abs(o) for o in out) == 0.0 and not lc.active


def test_release_aimed_into_the_right_lane_goes_there_and_hands_back():
  # the driver steered right and let go with the car's centre already 1.8 m right (inside the right lane 1.6..4.8)
  lc = LC.LaneChoice(0.01)
  shifted = (-6.8, -3.4, -0.2, 3.0)   # the car moved 1.8 m right: lines appear 1.8 m further left
  frames = [frame(i, lines=shifted) for i in range(10)]
  pressed = [True] * 9 + [False]
  run(lc, frames, pressed)
  assert lc.active and lc.mode == 'lane' and lc.events[-1] == ('release', 'right')
  out = run(lc, [frame(10 + i, lines=shifted) for i in range(20)])
  assert max(out) > 0.0                                       # pulls right toward the right lane's centre (+1.4 m)
  assert all(o >= -1e-12 for o in out)
  lim = LC.MAX_DIFF_ACCEL / V ** 2
  assert max(out) <= lim + 1e-9                               # never more than the cap beyond the model
  steps = np.diff(out)
  assert np.max(np.abs(steps)) <= LC.JERK / V ** 2 * 0.01 + 1e-12   # jerk-limited
  # the model now takes that lane as its own (lines re-indexed around it): hand back after AGREE_N frames
  agreed = (-3.4, -0.2, 3.0, 6.2)
  run(lc, [frame(30 + i, lines=agreed) for i in range(LC.AGREE_N + 1)])
  assert not lc.active
  out = run(lc, [frame(50 + i, lines=agreed) for i in range(100)])
  assert abs(out[-1]) < 1e-12                                  # wound out


def test_release_inside_the_own_lane_does_nothing():
  lc = LC.LaneChoice(0.01)
  frames = [frame(i) for i in range(10)]
  run(lc, frames, [True] * 9 + [False])
  assert not lc.active and lc.events[-1] == ('release', 'ego')


def test_blinker_ends_it():
  lc = LC.LaneChoice(0.01)
  shifted = (-6.8, -3.4, -0.2, 3.0)
  run(lc, [frame(i, lines=shifted) for i in range(10)], [True] * 9 + [False])
  assert lc.active
  run(lc, [frame(10, lines=shifted)], blinker=True)
  assert not lc.active


def test_split_aimed_in_the_models_lane_does_nothing():
  lc = LC.LaneChoice(0.01)
  wide = [frame(i, lines=(-6.0, -2.2, 2.2, 5.5)) for i in range(20)]          # 4.4 m lane
  split = [frame(20 + i, lines=(-2.2, -1.5, 1.5, 2.2)) for i in range(5)]     # a divider: 3.0 m lane
  run(lc, wide + split)
  assert ('split', 'ego') in lc.events and not lc.active


def test_bay_is_off():
  assert not LC.BAY_ENABLED
  lc = LC.LaneChoice(0.01)
  frames = []
  for i in range(80):
    md = frame(i, path=-0.6)
    e = -3.0 - (2.0 if i > 40 else 0.0)
    md.roadEdges[0] = N(x=XS, y=[e] * len(XS))
    frames.append(md)
  out = run(lc, frames)
  assert max(abs(o) for o in out) == 0.0
