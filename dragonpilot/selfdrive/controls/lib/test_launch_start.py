from types import SimpleNamespace as N

from dragonpilot.selfdrive.controls.lib.launch_start import LaunchStart, T_ON

DT = 0.05


def lead(v, status=True):
  return N(status=status, vLead=v, dRel=3.5)


def test_lead_drives_off_releases_the_hold_after_t_on():
  ls = LaunchStart(DT)
  outs = [ls.update(True, 0.0, lead(0.42), 0.05) for _ in range(10)]   # 00000103 16:03:43: 1.5 km/h, plan +0.05
  k = outs.index(False)
  assert abs((k + 1) * DT - T_ON) < 1e-6
  assert not any(outs[k:])


def test_radar_noise_behind_a_stopped_lead_never_releases():
  ls = LaunchStart(DT)
  for i in range(1200):                                                # 60 s, noise up to 0.25 m/s, spikes
    v = 0.25 if i % 7 == 0 else (0.35 if i % 50 == 0 else 0.1)
    assert ls.update(True, 0.0, lead(v), 0.05) is True


def test_negative_plan_or_no_lead_keeps_the_hold():
  ls = LaunchStart(DT)
  assert all(ls.update(True, 0.0, lead(1.0), -0.2) for _ in range(20))
  ls = LaunchStart(DT)
  assert all(ls.update(True, 0.0, lead(1.0, status=False), 0.3) for _ in range(20))
  ls = LaunchStart(DT)
  assert all(ls.update(True, 0.0, None, 0.3) for _ in range(20))


def test_moving_ego_untouched():
  ls = LaunchStart(DT)
  assert all(ls.update(True, 1.0, lead(1.0), 0.3) for _ in range(20))
  assert all(ls.update(False, 1.0, lead(1.0), 0.3) is False for _ in range(5))
