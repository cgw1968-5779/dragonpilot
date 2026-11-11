from types import SimpleNamespace as N

from dragonpilot.selfdrive.controls.lib.radar_match import plausible_tracks

V_EGO = 123 / 3.6


def test_other_lane_slower_track_is_not_offered():
  # route 1a7 21:25:40: camera lead in our lane ~110 m at 116 km/h; radar track 93 m, 3.8 m left, 81 km/h
  lead = N(y=[0.2], v=[116 / 3.6])
  tracks = {9780: N(dRel=93.0, yRel=-3.8, vRel=(81 - 123) / 3.6), 1: N(dRel=105.0, yRel=-0.3, vRel=(115 - 123) / 3.6)}
  assert set(plausible_tracks(V_EGO, lead, tracks)) == {1}


def test_any_one_condition_alone_keeps_the_track():
  lead = N(y=[0.0], v=[100 / 3.6])
  near = {1: N(dRel=35.0, yRel=-3.8, vRel=-40 / 3.6)}                   # within 40 m: stock
  same_lane = {1: N(dRel=90.0, yRel=-1.0, vRel=-40 / 3.6)}              # not to the side
  same_speed = {1: N(dRel=90.0, yRel=-3.8, vRel=(100 - 123) / 3.6)}     # speed agrees
  for t in (near, same_lane, same_speed):
    assert plausible_tracks(V_EGO, lead, t) == t
