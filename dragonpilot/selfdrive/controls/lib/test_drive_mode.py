import numpy as np

from opendbc.car.toyota.drive_mode import drive_mode_from_gear_packet, DRIVE_MODE_UNKNOWN, DRIVE_MODE_ECO, \
                                          DRIVE_MODE_NORMAL, DRIVE_MODE_SPORT
from dragonpilot.selfdrive.controls.lib.drive_mode import drive_mode_max_accel, MAX_ACCEL_BP, MAX_ACCEL_PROFILES


class TestDriveModeSignal:
  def test_corolla_signals(self):
    assert drive_mode_from_gear_packet({"GEAR": 0, "SPORT_ON": 0, "ECON_ON": 0}) == DRIVE_MODE_NORMAL
    assert drive_mode_from_gear_packet({"GEAR": 0, "SPORT_ON": 0, "ECON_ON": 1}) == DRIVE_MODE_ECO
    assert drive_mode_from_gear_packet({"GEAR": 0, "SPORT_ON": 1, "ECON_ON": 0}) == DRIVE_MODE_SPORT

  def test_rav4_sport_2(self):
    assert drive_mode_from_gear_packet({"SPORT_ON": 0, "SPORT_ON_2": 1, "ECON_ON": 0}) == DRIVE_MODE_SPORT

  def test_no_signals(self):
    assert drive_mode_from_gear_packet({"GEAR": 0}) == DRIVE_MODE_UNKNOWN


class TestDriveModeMaxAccel:
  def test_unknown_keeps_stock(self):
    for v in (0., 5., 20., 40.):
      assert drive_mode_max_accel(v, -1, 1.234) == 1.234

  def test_tables(self):
    for mode, vals in MAX_ACCEL_PROFILES.items():
      for bp, val in zip(MAX_ACCEL_BP, vals, strict=True):
        assert np.isclose(drive_mode_max_accel(bp, mode, 9.9), val)

  def test_order(self):
    for v in np.linspace(0., 40., 81):
      eco, normal, sport = (drive_mode_max_accel(v, m, 0.) for m in (0, 1, 2))
      assert eco <= normal <= sport

  def test_capnp_default_is_unknown(self):
    from opendbc.car import structs
    cs = structs.CarState.new_message()
    assert cs.driveMode == -1
    cs.driveMode = 2
    assert cs.as_reader().driveMode == 2
