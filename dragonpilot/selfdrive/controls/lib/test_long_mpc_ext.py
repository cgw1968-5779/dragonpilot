import numpy as np
from cereal import log
from dragonpilot.selfdrive.controls.lib import long_mpc_ext as ext

T = np.array([0.0, 0.05, 0.25, 0.5, 1.0, 2.0, 4.0, 6.0, 10.0])
STD = log.LongitudinalPersonality.standard


def lead_xv(x0, v, a=0.0):
  vv = np.maximum(v + a * T, 0.0)
  return np.column_stack([x0 + np.concatenate([[0.0], np.cumsum(np.diff(T) * vv[1:])]), vv])


def obstacles(l0, l1):
  return np.column_stack([l0[:, 0] + l0[:, 1] ** 2 / 5.0, l1[:, 0] + l1[:, 1] ** 2 / 5.0, np.full(len(T), 500.0)])


def test_equal_speed_following_unchanged_and_cruise_untouched():
  e = ext.LongMpcExt()
  v = np.full(len(T), 50 / 3.6)
  l0, l1 = lead_xv(30.0, 50 / 3.6), lead_xv(80.0, 50 / 3.6)
  x = obstacles(l0, l1)
  out = e.adjust(x, l0, l1, v, T, STD)
  assert np.allclose(out, x)  # light-onset margins cancel; no dyn gap with a calm lead; no cap without braking
  assert e.min_accel(-3.5) == -3.5


def test_stopped_lead_brakes_earlier_but_the_stop_point_is_kept():
  e = ext.LongMpcExt()
  v = np.full(len(T), 45 / 3.6)
  l0, l1 = lead_xv(60.0, 0.0), lead_xv(200.0, 0.0)
  x = obstacles(l0, l1)
  out = e.adjust(x, l0, l1, v, T, STD)
  assert out[0, 0] < x[0, 0] - 10.0  # ~1.1 s of travel at 45 km/h: the brake starts earlier
  out0 = ext.LongMpcExt().adjust(x, l0, l1, np.zeros(len(T)), T, STD)
  assert np.allclose(out0[:, 0], x[:, 0])  # planned speed 0 (the stop): no margin, same stop point


def test_onset_cap_starts_soft_and_never_below_the_need():
  e = ext.LongMpcExt()
  v0 = 40 / 3.6
  v = v0 - 1.0 * T  # the plan is braking
  l0, l1 = lead_xv(150.0, 0.0), lead_xv(250.0, 0.0)  # far: the need (~0.44) is under the soft start
  e.adjust(obstacles(l0, l1), l0, l1, v, T, STD)
  a_min = e.min_accel(-3.5)
  assert abs(a_min[0] - -ext.CAP_A0) < 1e-6  # soft start
  assert np.all(np.diff(a_min) <= 1e-9)      # grows along the horizon
  e2 = ext.LongMpcExt()
  l0 = lead_xv(35.0, 0.0)
  e2.adjust(obstacles(l0, l1), l0, l1, v, T, STD)
  need = v0 ** 2 / (2 * (35.0 - ext.CAP_STOP_D - v0 * ext.CAP_REACT_T))
  assert e2.min_accel(-3.5)[0] <= -need + 1e-6  # a close stopped lead: the cap never sits below the need


def test_lead_decel_trust():
  from dragonpilot.selfdrive.controls.lib.long_mpc_ext import lead_decel_trust
  kph = 1 / 3.6
  assert abs(lead_decel_trust(-1.4, 32 * kph) - -0.7) < 1e-9          # 21:59:07: half the lead's decel at 20-40 km/h
  assert lead_decel_trust(-1.4, 10 * kph) == -1.4                     # low speed untouched
  assert lead_decel_trust(-1.4, 60 * kph) == -1.4                     # highway untouched
  assert lead_decel_trust(-3.2, 32 * kph) == -3.2                     # a hard brake is never discounted
  assert -3.0 < lead_decel_trust(-2.5, 32 * kph) < -2.5 * 0.5         # fades back in between
  assert lead_decel_trust(0.8, 32 * kph) == 0.8                       # acceleration untouched
