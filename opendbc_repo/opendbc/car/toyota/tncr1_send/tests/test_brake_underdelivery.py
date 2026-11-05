import unittest
from types import SimpleNamespace

from opendbc.car.toyota.tncr1_send.brake_onset import (BrakeHandoverFeedforward, HANDOVER_EXTRA_MAX,
                                                       HANDOVER_EXTRA_FRAC, HANDOVER_RATE, PID_BLEED_NEG, PID_BLEED_POS,
                                                       LOW_EXTRA)

DT = 0.03
class TestHandoverFeedforward(unittest.TestCase):
  def run_steady(self, req, v_kph, n=200):
    c = BrakeHandoverFeedforward(DT)
    for _ in range(n):
      c.update(req, v_kph / 3.6)
    return c

  def test_full_extra_in_the_dip(self):
    c = self.run_steady(-0.84, 12.0)   # route 0000010e: asked -0.84 at 12-9 km/h, the car gave -0.61
    self.assertAlmostEqual(c.extra, HANDOVER_EXTRA_MAX, places=9)
    self.assertAlmostEqual(c.apply(-0.84), -0.84 - HANDOVER_EXTRA_MAX, places=9)

  def test_lighter_request_gets_a_quarter(self):
    self.assertAlmostEqual(self.run_steady(-0.5, 12.0).extra, HANDOVER_EXTRA_FRAC * 0.5, places=9)

  def test_fades_at_the_edges(self):
    self.assertAlmostEqual(self.run_steady(-1.0, 8.0).extra, 0.5 * HANDOVER_EXTRA_MAX + 0.25 * LOW_EXTRA[2], places=6)
    self.assertEqual(self.run_steady(-1.0, 2.5).extra, 0.0)
    self.assertEqual(self.run_steady(-1.0, 20.0).extra, 0.0)
    self.assertEqual(self.run_steady(-0.25, 12.0).extra, 0.0)

  def test_low_speed_extra(self):
    # route 0000011c 00:19:10: asked -0.67 at 5-4 km/h, the car gave -0.28
    # tncr (Corolla Cross): LOW_EXTRA is zero - route 0000006b showed the Cross delivers the sent command at 4-7 km/h
    self.assertAlmostEqual(self.run_steady(-0.67, 5.0).extra, LOW_EXTRA[2], places=6)
    self.assertAlmostEqual(self.run_steady(-0.67, 6.0).extra, LOW_EXTRA[2], places=6)
    self.assertAlmostEqual(self.run_steady(-0.62, 4.0).extra, LOW_EXTRA[1], places=6)
    self.assertEqual(self.run_steady(-0.6, 3.0).extra, 0.0)   # the last km/h already brakes on target
    self.assertEqual(self.run_steady(-0.25, 5.0).extra, 0.0)  # not for a request lighter than HANDOVER_MIN_REQUEST
    self.assertEqual(self.run_steady(-1.5, 5.0).extra, 0.0)   # nor for a firm one

  def test_fades_out_for_a_firm_request(self):
    self.assertEqual(self.run_steady(-1.5, 12.0).extra, 0.0)   # 0000010d: at -1.46 the car delivered -1.72
    self.assertAlmostEqual(self.run_steady(-1.25, 12.0).extra, 0.5 * HANDOVER_EXTRA_MAX, places=6)

  def test_gradual(self):
    c = BrakeHandoverFeedforward(DT)
    prev = 0.0
    for _ in range(50):
      e = c.update(-1.0, 12 / 3.6)
      self.assertLessEqual(e - prev, HANDOVER_RATE * DT + 1e-9)
      prev = e

class TestPidHold(unittest.TestCase):
  def test_braking_integral_bleeds_and_freezes_below_9kph(self):
    c = BrakeHandoverFeedforward(DT)
    pid = SimpleNamespace(i=-0.15)
    self.assertTrue(c.condition_pid(pid, -0.6, 5 / 3.6))
    self.assertAlmostEqual(pid.i, -0.15 + PID_BLEED_NEG * DT, places=9)
    for _ in range(100):
      c.condition_pid(pid, -0.6, 5 / 3.6)
    self.assertEqual(pid.i, 0.0)

  def test_gas_integral_left_from_a_launch_bleeds_fast(self):
    c = BrakeHandoverFeedforward(DT)
    pid = SimpleNamespace(i=0.3)
    c.condition_pid(pid, -0.2, 3 / 3.6)
    self.assertAlmostEqual(pid.i, 0.3 - PID_BLEED_POS * DT, places=9)

  def test_untouched_above_9kph_or_when_not_braking(self):
    c = BrakeHandoverFeedforward(DT)
    pid = SimpleNamespace(i=-0.15)
    self.assertFalse(c.condition_pid(pid, -0.6, 12 / 3.6))
    self.assertFalse(c.condition_pid(pid, 0.3, 3 / 3.6))
    self.assertEqual(pid.i, -0.15)


if __name__ == "__main__":
  unittest.main()
