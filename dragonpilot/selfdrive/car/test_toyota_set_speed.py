"""Toyota (Corolla Cross TSS2) software set speed, ported from sunnypilot tncr1 test_toyota_software_cruise.py and
test_cruise_stalk_buttons.py: short press +/-5 on openpilot's own target, long press only moves the PCM ceiling,
target capped at PCM + headroom, CANCEL/RES keep the target, dial calibration on the HUD."""
import unittest

from cereal import car
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_UNSET
from opendbc.car.toyota.software_set_speed import CruiseStalk, software_set_speed
from dragonpilot.selfdrive.car.toyota_set_speed import (TOYOTA_PCM_SET_SPEED_HEADROOM_KPH, TOYOTA_VIRTUAL_CRUISE_LONG_PRESS,
                                                        toyota_dial_from_canonical, toyota_canonical_from_dial,
                                                        pcm_ceiling_base_kph)

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type

PCM_KPH = 80.      # the PCM's own set speed (0xB4 units) after a SET at ~80
PCM_DIAL_KPH = 86. # what the cluster shows for it (UI_SET_SPEED)


def cross_cp(**kw):
  d = dict(brand="toyota", carFingerprint="TOYOTA_COROLLA_TSS2", pcmCruise=True, openpilotLongitudinalControl=True)
  d.update(kw)
  return car.CarParams.new_message(**d)


class TestGate(unittest.TestCase):
  def test_only_cross_with_op_long(self):
    self.assertTrue(software_set_speed(cross_cp()))
    self.assertFalse(software_set_speed(cross_cp(openpilotLongitudinalControl=False)))
    self.assertFalse(software_set_speed(cross_cp(carFingerprint="TOYOTA_RAV4_TSS2")))
    self.assertIsNone(VCruiseHelper(cross_cp(carFingerprint="TOYOTA_RAV4_TSS2")).toyota_sss)


class TestToyotaSoftwareCruise(unittest.TestCase):
  def setUp(self):
    self.helper = VCruiseHelper(cross_cp())
    self.assertIsNotNone(self.helper.toyota_sss)
    self.pcm_kph = PCM_KPH
    self.pcm_dial_kph = PCM_DIAL_KPH
    self.engaged = True
    self.v_ego_kph = 80.

  def cs(self, events=()):
    CS = car.CarState.new_message()
    CS.cruiseState.available = True
    CS.cruiseState.enabled = self.engaged
    CS.cruiseState.speed = self.pcm_kph * CV.KPH_TO_MS
    CS.cruiseState.speedCluster = self.pcm_dial_kph * CV.KPH_TO_MS
    CS.vEgo = self.v_ego_kph * CV.KPH_TO_MS
    CS.buttonEvents = [ButtonEvent.new_message(type=t, pressed=p) for t, p in events]
    return CS

  def step(self, enabled=True, events=(), n=1):
    for _ in range(n):
      self.helper.update_v_cruise(self.cs(events), enabled, is_metric=True)
      events = ()

  def engage_with_set(self):
    self.engaged = True
    self.step(enabled=False, events=[(ButtonType.decelCruise, True)])
    self.step(enabled=False, events=[(ButtonType.decelCruise, False)])
    self.step(enabled=True, n=3)

  def tap(self, button, hold_frames=5):
    self.step(events=[(button, True)])
    self.step(n=hold_frames - 1)
    self.step(events=[(button, False)])

  def test_dial_calibration_round_trips(self):
    for dial in (30., 55., 80., 100., 120.):
      self.assertAlmostEqual(toyota_dial_from_canonical(toyota_canonical_from_dial(dial)), dial, places=6)
    self.assertAlmostEqual(toyota_canonical_from_dial(86.), 78.2, delta=0.3)
    self.assertAlmostEqual(toyota_dial_from_canonical(112. / 1.021), 120., delta=0.6)

  def test_set_mirrors_pcm_on_dial_calibration(self):
    self.engage_with_set()
    self.assertEqual(self.helper.v_cruise_cluster_kph, PCM_DIAL_KPH)
    self.assertAlmostEqual(self.helper.v_cruise_kph, toyota_canonical_from_dial(PCM_DIAL_KPH), delta=0.1)
    self.assertLess(self.helper.v_cruise_kph, PCM_KPH)

  def test_engaging_press_is_not_a_set_speed_change(self):
    self.engage_with_set()
    self.assertEqual(self.helper.v_cruise_cluster_kph, PCM_DIAL_KPH)  # the SET/- that engaged did not step to 85

  def test_short_press_steps_the_display_by_the_increment(self):
    self.engage_with_set()
    self.tap(ButtonType.accelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 90.)  # 86 snaps up to 90
    self.assertAlmostEqual(self.helper.v_cruise_kph, toyota_canonical_from_dial(90.), delta=0.1)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 85.)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 80.)

  def test_rapid_taps_accumulate(self):
    self.engage_with_set()
    for _ in range(3):
      self.tap(ButtonType.decelCruise, hold_frames=3)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 75.)
    self.tap(ButtonType.accelCruise, hold_frames=2)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 80.)

  def test_press_and_release_in_one_cycle_counts(self):
    self.engage_with_set()
    self.step(events=[(ButtonType.decelCruise, True)])
    self.step(events=[(ButtonType.decelCruise, False), (ButtonType.decelCruise, True)])
    self.step(events=[(ButtonType.decelCruise, False)])
    self.assertEqual(self.helper.v_cruise_cluster_kph, 80.)

  def test_minimum_set_speed(self):
    self.engage_with_set()
    for _ in range(20):
      self.tap(ButtonType.decelCruise, hold_frames=3)
    self.assertEqual(self.helper.v_cruise_kph, 30.)  # no stop target yet (owner 2026-11-02)

  def test_set_below_minimum_is_raised_like_tncr1(self):
    # PCM SET at dial 30 (canonical 25.7): tncr1 keeps the canonical target >= 30, so the HUD shows 34.6 (route ae 20:00:05)
    self.pcm_kph, self.pcm_dial_kph = 26., 30.
    self.engage_with_set()
    self.assertEqual(self.helper.v_cruise_kph, 30.)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 34.6)

  def test_long_press_leaves_the_target_alone(self):
    self.engage_with_set()
    target, cluster = self.helper.v_cruise_kph, self.helper.v_cruise_cluster_kph
    self.step(events=[(ButtonType.accelCruise, True)])
    for i in range(TOYOTA_VIRTUAL_CRUISE_LONG_PRESS + 40):
      if i % 25 == 0:
        self.pcm_kph += 1
        self.pcm_dial_kph += 1
      self.step()
    self.step(events=[(ButtonType.accelCruise, False)])
    self.assertEqual(self.helper.v_cruise_kph, target)
    self.assertEqual(self.helper.v_cruise_cluster_kph, cluster)
    self.assertGreater(self.pcm_kph, PCM_KPH)

  def test_target_capped_at_pcm_plus_headroom(self):
    self.engage_with_set()
    for _ in range(6):
      self.tap(ButtonType.accelCruise)
    # ceiling base = the UI set speed on the speedometer model (tncr1 set_speed_target): dial 86 -> 80.8
    self.assertAlmostEqual(pcm_ceiling_base_kph(self.cs(), True), (86. + 0.3 - 1.47) / 1.05, places=3)
    cap = round(round(pcm_ceiling_base_kph(self.cs(), True), 1) + TOYOTA_PCM_SET_SPEED_HEADROOM_KPH, 1)
    self.assertEqual(self.helper.v_cruise_kph, cap)
    self.assertEqual(self.helper.v_cruise_cluster_kph, toyota_dial_from_canonical(cap))
    self.pcm_kph, self.pcm_dial_kph = 120., 128.
    self.tap(ButtonType.accelCruise)
    self.assertGreater(self.helper.v_cruise_kph, cap)
    self.assertEqual(self.helper.v_cruise_cluster_kph % 5, 0)

  def test_cancel_then_res_keeps_the_target(self):
    self.engage_with_set()
    self.tap(ButtonType.decelCruise)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 80.)
    target = self.helper.v_cruise_kph
    self.engaged = False
    self.pcm_kph, self.pcm_dial_kph = 120., 128.
    self.step(enabled=False, n=20)
    self.assertEqual(self.helper.v_cruise_kph, target)
    self.step(enabled=False, events=[(ButtonType.accelCruise, True)])
    self.engaged = True
    self.step(enabled=False, events=[(ButtonType.accelCruise, False)])
    self.step(enabled=False, n=5)
    self.step(enabled=True, n=5)
    self.assertEqual(self.helper.v_cruise_kph, target)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 80.)

  def test_cancel_then_set_mirrors_the_pcm(self):
    self.engage_with_set()
    self.tap(ButtonType.decelCruise)
    self.engaged = False
    self.step(enabled=False, n=20)
    self.pcm_kph, self.pcm_dial_kph = 70., 76.
    self.step(enabled=False, events=[(ButtonType.decelCruise, True)])
    self.engaged = True
    self.step(enabled=False, events=[(ButtonType.decelCruise, False)])
    self.step(enabled=False, n=5)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 76.)
    self.assertAlmostEqual(self.helper.v_cruise_kph, toyota_canonical_from_dial(76.), delta=0.1)

  def test_unset_when_cruise_unavailable(self):
    self.engage_with_set()
    CS = self.cs()
    CS.cruiseState.available = False
    self.helper.update_v_cruise(CS, True, is_metric=True)
    self.assertEqual(self.helper.v_cruise_kph, V_CRUISE_UNSET)


class FakeParser:
  def __init__(self, res, set_):
    self.vl_all = {"CLUTCH": {"CRUISE_BTN_RES": res, "CRUISE_BTN_SET": set_}}


class TestCruiseStalkButtons(unittest.TestCase):
  def setUp(self):
    self.stalk = CruiseStalk()

  def events(self, res, set_):
    return [(e.type, e.pressed) for e in self.stalk.update(FakeParser(res, set_))]

  def test_short_res_press_gives_one_press_and_release(self):
    self.assertEqual(self.events([1], [0]), [(ButtonType.accelCruise, True)])
    for _ in range(6):
      self.assertEqual(self.events([1], [0]), [])
    self.assertEqual(self.events([0], [0]), [(ButtonType.accelCruise, False)])
    self.assertEqual(self.events([], []), [])

  def test_set_press(self):
    self.assertEqual(self.events([0], [1]), [(ButtonType.decelCruise, True)])
    self.assertEqual(self.events([0], [0]), [(ButtonType.decelCruise, False)])

  def test_press_and_release_in_one_iteration(self):
    self.assertEqual(self.events([1, 0], [0, 0]), [(ButtonType.accelCruise, True), (ButtonType.accelCruise, False)])

  def test_both_bits_is_not_a_press(self):
    self.assertEqual(self.events([1], [1]), [])

  def test_switching_buttons_releases_the_first(self):
    self.assertEqual(self.events([1], [0]), [(ButtonType.accelCruise, True)])
    self.assertEqual(self.events([0], [1]), [(ButtonType.accelCruise, False), (ButtonType.decelCruise, True)])


if __name__ == "__main__":
  unittest.main()
