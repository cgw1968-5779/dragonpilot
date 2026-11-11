from cereal import log

from dragonpilot.selfdrive.controls.lib import lane_change_rate as L

DT = 0.01
KPH = 1 / 3.6


def test_off_outside_lane_change():
  lc = L.LaneChangeRate(DT)
  assert lc.update(0.01, 0.0, log.LaneChangeState.off, 100 * KPH) == 0.01
  assert lc.update(0.01, 0.0, log.LaneChangeState.preLaneChange, 100 * KPH) == 0.01


def test_highway_start_is_gentle_then_opens():
  lc = L.LaneChangeRate(DT)
  k, ks = 0.0, []
  for _ in range(400):
    k = lc.update(0.002, k, L.STARTING, 100 * KPH)
    ks.append(k)
  assert abs(ks[99] - 0.00085) < 2e-5          # first second at 0.00085 1/m per s (dppb 11-11, was 0.001)
  assert ks[-1] == 0.002                        # still reaches the model's curvature


def test_low_speed_flat_rate_and_reset():
  lc = L.LaneChangeRate(DT)
  k = lc.update(0.01, 0.0, L.STARTING, 40 * KPH)
  assert abs(k - 0.004 * DT) < 1e-9
  lc.update(0.0, k, log.LaneChangeState.off, 40 * KPH)
  assert lc.t == 0.0


def test_mid_speed_interpolates():
  assert abs(L.start_rate(0.5, 75 * KPH) - 0.002425) < 1e-9  # halfway (dppb 11-11: first-second high-speed rate -15 %)
  assert abs(L.start_rate(3.5, 75 * KPH) - 0.0035) < 1e-9
