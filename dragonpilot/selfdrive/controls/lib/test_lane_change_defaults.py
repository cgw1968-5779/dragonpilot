import time
from types import SimpleNamespace

from cereal import log
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from dragonpilot.selfdrive.controls.lib.lane_change_defaults import AUTO_LANE_CHANGE_SEC, ROAD_EDGE_DETECTION

LCS = log.LaneChangeState


def cs(left=True, bsm=False):
  return SimpleNamespace(vEgo=25.0, leftBlinker=left, rightBlinker=False, steeringPressed=False, steeringTorque=0.0,
                         leftBlindspot=bsm, rightBlindspot=False)


def run(seconds, bsm=False, edge=False):
  dh = DesireHelper(dp_lat_lca_speed=20, dp_lat_lca_auto_sec=AUTO_LANE_CHANGE_SEC)
  t0 = time.monotonic()
  dh.update(cs(bsm=bsm), True, 1.0, edge, False)
  while time.monotonic() - t0 < seconds:
    dh.update(cs(bsm=bsm), True, 1.0, edge, False)
    time.sleep(0.01)
  return dh.lane_change_state


def test_fixed_values():
  assert AUTO_LANE_CHANGE_SEC == 0.5 and ROAD_EDGE_DETECTION is True


def test_auto_change_after_half_second():
  assert run(0.3) == LCS.preLaneChange
  assert run(0.6) == LCS.laneChangeStarting


def test_bsm_blocks():
  assert run(1.0, bsm=True) == LCS.preLaneChange


def test_road_edge_blocks():
  assert run(1.0, edge=True) == LCS.preLaneChange
