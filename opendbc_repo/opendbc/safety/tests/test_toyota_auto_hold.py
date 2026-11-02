#!/usr/bin/env python3
"""dragonpilot: Toyota auto brake hold (safety param AUTO_HOLD). openpilot's PRE_COLLISION_2 (0x344) may only replace the
camera's while the car stands, gas is released, ACC main is on and cruise is not engaged; everywhere else the camera's
0x344 is forwarded as stock and openpilot's is refused. All stock Toyota torque tests run again with the flag set."""
import unittest

from opendbc.car.toyota.values import ToyotaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.test_toyota import TestToyotaSafetyTorque, TestToyotaStockLongitudinalTorque


class TestToyotaAutoHoldSafety(TestToyotaSafetyTorque):
  def setUp(self):
    self.packer = CANPackerSafety("toyota_nodsu_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, self.EPS_SCALE | ToyotaSafetyFlags.AUTO_HOLD)
    self.safety.init_tests()

  def _main_msg(self, on):
    return self.packer.make_can_msg_safety("PCM_CRUISE_2", 0, {"MAIN_ON": on})

  def _hold_msg(self):
    return self.packer.make_can_msg_safety("PRE_COLLISION_2", 0, {"DSS1GDRV": -1.0, "PBRTRGR": 1})

  def _state(self, moving=False, gas=False, main=True, engaged=False):
    self.safety.set_controls_allowed(False)
    self._rx(self._main_msg(main))
    self._rx(self._speed_msg(5.0 if moving else 0.0))
    values = {"GAS_RELEASED": not gas, "CRUISE_ACTIVE": engaged}
    self._rx(self.packer.make_can_msg_safety("PCM_CRUISE", 0, values))

  def _camera_aeb_blocked(self):
    return self.safety.safety_fwd_hook(2, 0x344) == -1

  def test_hold_window(self):
    self._state()
    self.assertTrue(self._tx(self._hold_msg()))
    self.assertTrue(self._camera_aeb_blocked())

  def test_outside_the_window(self):
    for kwargs in ({"moving": True}, {"gas": True}, {"main": False}, {"engaged": True}):
      with self.subTest(**kwargs):
        self._state(**kwargs)
        self.assertFalse(self._tx(self._hold_msg()))
        self.assertFalse(self._camera_aeb_blocked())

  def test_window_closes_when_the_car_moves(self):
    self._state()
    self.assertTrue(self._camera_aeb_blocked())
    self._rx(self._speed_msg(0.5))
    self.assertFalse(self._camera_aeb_blocked())
    self.assertFalse(self._tx(self._hold_msg()))

  def test_main_on_is_read_without_alka(self):
    # 0x1D3 is not in the RX checks; the hold must still see ACC main (rx_ext), ALKA off
    self._state(main=False)
    self.assertFalse(self._camera_aeb_blocked())
    self._rx(self._main_msg(True))
    self.assertTrue(self._camera_aeb_blocked())


class TestToyotaNoAutoHoldSafety(TestToyotaSafetyTorque):
  """flag off (any car without the hold): camera 0x344 always forwarded, openpilot's 0x344 not gated"""
  def test_flag_off_never_blocks(self):
    self.safety.set_controls_allowed(False)
    self._rx(self.packer.make_can_msg_safety("PCM_CRUISE_2", 0, {"MAIN_ON": 1}))
    self._rx(self._speed_msg(0.0))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x344), 0)


class TestToyotaAutoHoldStockLongSafety(TestToyotaStockLongitudinalTorque):
  """stock longitudinal: the flag is ignored (0x344 is not in the TX list, camera 0x344 forwarded)"""
  def setUp(self):
    self.packer = CANPackerSafety("toyota_nodsu_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota,
                                 self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.AUTO_HOLD)
    self.safety.init_tests()

  def test_stock_long_ignores_the_flag(self):
    self._rx(self.packer.make_can_msg_safety("PCM_CRUISE_2", 0, {"MAIN_ON": 1}))
    self._rx(self._speed_msg(0.0))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x344), 0)


if __name__ == "__main__":
  unittest.main()
