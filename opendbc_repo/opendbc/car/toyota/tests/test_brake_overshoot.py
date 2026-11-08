from opendbc.car.toyota import brake_overshoot as B

DT = 0.03


def test_no_relief_without_overshoot_or_outside_the_window():
  r = B.BrakeOvershootRelief(DT)
  for _ in range(100):
    assert r.update(True, 18 / 3.6, -1.5, -1.75) == 0.0     # overshoot 0.25 < deadband
    assert r.update(True, 35 / 3.6, -2.0, -2.8) == 0.0      # above 28 km/h
    assert r.update(True, 10 / 3.6, -2.0, -2.8) == 0.0      # below 12 km/h
    assert r.update(True, 18 / 3.6, -0.3, -1.0) == 0.0      # light command
    assert r.update(True, 18 / 3.6, -1.5, -1.2) == 0.0      # car brakes less than asked: never firmer


def test_small_relief_for_a_heavy_overshoot_rate_limited_and_capped():
  r = B.BrakeOvershootRelief(DT)
  out = [r.update(True, 18 / 3.6, -1.9, -2.5) for _ in range(100)]
  assert abs(out[-1] - B.KP * (0.6 - 0.3)) < 1e-6
  assert out[-1] < 0.6 - 0.3                                   # still at least the planner's decel
  assert max(abs(b - a) for a, b in zip([0.0] + out, out)) <= B.RATE * DT + 1e-9
  out = [r.update(True, 18 / 3.6, -2.0, -4.0) for _ in range(100)]
  assert out[-1] == B.MAX_RELIEF


def test_gone_when_the_overshoot_is_and_reset_when_inactive():
  r = B.BrakeOvershootRelief(DT)
  for _ in range(100):
    r.update(True, 18 / 3.6, -1.9, -2.5)
  for _ in range(20):
    x = r.update(True, 18 / 3.6, -1.9, -1.9)
  assert x == 0.0
  r.update(True, 18 / 3.6, -1.9, -2.5)
  assert r.update(False, 18 / 3.6, -1.9, -2.5) == 0.0
