import math

from opendbc.car.toyota.tncr1_send.brake_onset import BrakeOnsetShaper, ENTRY_J, CATCHUP_J, ENTRY_T_MAX, HARD_BRAKE_ACCEL

DT = 0.03


def test_command_crawls_while_the_friction_enters():
  # route 00000119 09:23:02: the car sits at the regen limit (-0.55) while the command is already -0.9, no friction yet
  sh = BrakeOnsetShaper(DT, 4.0)
  sh.t_onset = 1.0  # past the soft first phase of the onset schedule (2.0 m/s^3 allowed there)
  step = sh.down_step(-1.6, -0.9, v_ego=12.8, a_ego=-0.55, brake_force=100.0)
  assert math.isclose(-step, ENTRY_J * DT)


def test_after_the_friction_is_in_the_gap_closes_at_catchup():
  sh = BrakeOnsetShaper(DT, 4.0)
  sh.down_step(-1.6, -0.9, v_ego=12.8, a_ego=-0.55, brake_force=100.0)
  step = sh.down_step(-1.6, -0.95, v_ego=12.8, a_ego=-0.7, brake_force=900.0)
  assert -step <= CATCHUP_J * DT + 1e-9


def test_no_limit_when_the_car_keeps_up():
  sh = BrakeOnsetShaper(DT, 4.0)
  sh.t_onset = 1.0
  assert -sh.down_step(-1.6, -0.9, v_ego=12.8, a_ego=-0.85, brake_force=100.0) > ENTRY_J * DT


def test_urgent_and_no_measurement_are_not_limited():
  sh = BrakeOnsetShaper(DT, 4.0)
  assert -sh.down_step(HARD_BRAKE_ACCEL - 0.5, -0.9, v_ego=12.8, urgent=True, a_ego=-0.55, brake_force=0.0) > ENTRY_J * DT
  sh2 = BrakeOnsetShaper(DT, 4.0)
  sh2.t_onset = 1.0
  assert -sh2.down_step(-1.6, -0.9, v_ego=12.8) > ENTRY_J * DT


def test_entry_limit_ends_after_entry_t_max():
  sh = BrakeOnsetShaper(DT, 4.0)
  cmd = -0.9
  for _ in range(int(ENTRY_T_MAX / DT) + 5):
    cmd += sh.down_step(-2.0, cmd, v_ego=12.8, a_ego=-0.55, brake_force=0.0)
  assert sh.t_entry >= ENTRY_T_MAX - DT
  assert -sh.down_step(-2.0, cmd, v_ego=12.8, a_ego=-0.55, brake_force=0.0) >= CATCHUP_J * DT - 1e-9
