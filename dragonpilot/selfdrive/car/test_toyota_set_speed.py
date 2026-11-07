"""dppb: Toyota (Corolla Altis Hybrid TSS2) software set speed (copy of dpcr's tests, Altis dial numbers), ported from sunnypilot tncr1 test_toyota_software_cruise.py and
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

PCM_KPH = 60.      # the Altis PCM's own set speed after a SET (11-05 route: 60 -> 65 on the dash)
PCM_DIAL_KPH = 65. # what the cluster shows for it (UI_SET_SPEED)


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
    self.v_ego_kph = 60.

  def cs(self, events=()):
    CS = car.CarState.new_message()
    CS.cruiseState.standstill = getattr(self, 'standstill', False)
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
    # Altis PCM pairs (11-05): the canonical target is the PCM speed itself
    for pcm, dial in ((26., 30.), (31., 35.), (36., 40.), (41., 45.), (45., 50.), (50., 55.), (55., 60.), (60., 65.)):
      self.assertAlmostEqual(toyota_canonical_from_dial(dial), pcm, delta=0.5)

  def test_set_mirrors_pcm_on_dial_calibration(self):
    self.engage_with_set()
    self.assertEqual(self.helper.v_cruise_cluster_kph, PCM_DIAL_KPH)
    self.assertAlmostEqual(self.helper.v_cruise_kph, toyota_canonical_from_dial(PCM_DIAL_KPH), delta=0.1)
    self.assertAlmostEqual(self.helper.v_cruise_kph, PCM_KPH, delta=0.1)  # drives exactly the PCM speed, as before

  def test_engaging_press_is_not_a_set_speed_change(self):
    self.engage_with_set()
    self.assertEqual(self.helper.v_cruise_cluster_kph, PCM_DIAL_KPH)  # the SET/- that engaged did not step to 85

  def test_short_press_steps_the_display_by_the_increment(self):
    self.engage_with_set()
    self.tap(ButtonType.accelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 70.)
    self.assertAlmostEqual(self.helper.v_cruise_kph, toyota_canonical_from_dial(70.), delta=0.1)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 65.)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 60.)

  def test_rapid_taps_accumulate(self):
    self.engage_with_set()
    for _ in range(3):
      self.tap(ButtonType.decelCruise, hold_frames=3)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 50.)
    self.tap(ButtonType.accelCruise, hold_frames=2)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 55.)

  def test_press_and_release_in_one_cycle_counts(self):
    self.engage_with_set()
    self.step(events=[(ButtonType.decelCruise, True)])
    self.step(events=[(ButtonType.decelCruise, False), (ButtonType.decelCruise, True)])
    self.step(events=[(ButtonType.decelCruise, False)])
    self.assertEqual(self.helper.v_cruise_cluster_kph, 55.)

  def test_minus_from_30_goes_straight_to_0(self):
    # owner 2026-11-07: from 30 on the dial one short - goes straight to 0 (0 = stop target); above 30 still -5
    self.engage_with_set()
    seen = []
    for _ in range(25):
      self.tap(ButtonType.decelCruise, hold_frames=3)
      seen.append(self.helper.v_cruise_cluster_kph)
    self.assertEqual(self.helper.v_cruise_kph, 0.)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 0.)
    self.assertEqual([x for x in seen if x < 41][:4], [40., 35., 30., 0.])
    self.v_ego_kph = 10.  # rolling: RES/+ from 0 goes straight back to the ceiling
    self.tap(ButtonType.accelCruise)
    self.assertAlmostEqual(self.helper.v_cruise_kph, pcm_ceiling_base_kph(self.cs(), True), delta=0.15)

  def test_set_at_pcm_minimum_mirrors_the_dial(self):
    # PCM SET at dial 30 (canonical 25.7): no 30 km/h canonical floor any more, the HUD shows the dash's 30
    self.pcm_kph, self.pcm_dial_kph = 26., 30.
    self.engage_with_set()
    self.assertEqual(self.helper.v_cruise_cluster_kph, 30.)
    self.assertAlmostEqual(self.helper.v_cruise_kph, toyota_canonical_from_dial(30.), delta=0.1)

  def long_press(self, button, frames=TOYOTA_VIRTUAL_CRUISE_LONG_PRESS + 20):
    self.step(events=[(button, True)])
    self.step(n=frames)
    self.step(events=[(button, False)])

  def test_long_press_minus_jumps_to_30_and_stops(self):
    # owner 2026-11-07: from 40-130 a long - goes straight to 30 and stays there however long it is held
    for pcm, dial in ((63., 68.), (120., 127.), (36., 40.)):
      self.setUp()
      self.pcm_kph, self.pcm_dial_kph = pcm, dial
      self.engage_with_set()
      start = self.helper.v_cruise_cluster_kph
      L = TOYOTA_VIRTUAL_CRUISE_LONG_PRESS
      self.step(events=[(ButtonType.decelCruise, True)])
      self.step(n=L - 5)
      self.assertEqual(self.helper.v_cruise_cluster_kph, start)  # nothing before 1 s
      self.step(n=10)
      self.assertEqual(self.helper.v_cruise_cluster_kph, 30.)
      self.step(n=5 * L)  # still held: stays 30
      self.assertEqual(self.helper.v_cruise_cluster_kph, 30.)
      self.step(events=[(ButtonType.decelCruise, False)])  # the release adds nothing
      self.assertEqual(self.helper.v_cruise_cluster_kph, 30.)
      self.tap(ButtonType.decelCruise, hold_frames=3)  # then one short - : 0
      self.assertEqual(self.helper.v_cruise_kph, 0.)

  def test_long_press_minus_at_30_or_35(self):
    # 30-40 unchanged: 35 -> 30; at 30 a long press does nothing (0 only with a short press); at 0 stays 0
    self.engage_with_set()
    while self.helper.v_cruise_cluster_kph > 35.:
      self.tap(ButtonType.decelCruise, hold_frames=3)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 35.)
    for expect in (30., 30.):
      self.long_press(ButtonType.decelCruise)
      self.assertEqual(self.helper.v_cruise_cluster_kph, expect)
    self.tap(ButtonType.decelCruise, hold_frames=3)
    self.long_press(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_kph, 0.)

  def test_res_from_zero_stopped_without_standstill_flag_returns_to_the_ceiling(self):
    # Altis route 190: the PCM never sets cruiseState.standstill while openpilot holds the car -> RES stepped 5, 10
    self.engage_with_set()
    for _ in range(25):
      self.tap(ButtonType.decelCruise, hold_frames=3)
    self.v_ego_kph, self.standstill = 0., False
    self.tap(ButtonType.accelCruise)
    self.assertAlmostEqual(self.helper.v_cruise_kph, pcm_ceiling_base_kph(self.cs(), True), delta=0.15)

  def test_long_res_from_zero_returns_to_the_raised_ceiling(self):
    for v, standstill in ((0., False), (0., True), (14., False)):
      self.setUp()
      self.engage_with_set()
      for _ in range(25):
        self.tap(ButtonType.decelCruise, hold_frames=3)
      self.v_ego_kph, self.standstill = v, standstill
      self.pcm_kph, self.pcm_dial_kph = 26., 30.  # the Altis PCM is at its 30 minimum after going down
      self.step(events=[(ButtonType.accelCruise, True)])
      self.step(n=TOYOTA_VIRTUAL_CRUISE_LONG_PRESS + 50)
      self.pcm_kph, self.pcm_dial_kph = 41., 45.  # the PCM climbed its own number while held
      self.step(events=[(ButtonType.accelCruise, False)])
      self.assertAlmostEqual(self.helper.v_cruise_kph, pcm_ceiling_base_kph(self.cs(), True), delta=0.15, msg=(v, standstill))
      self.assertGreater(self.helper.v_cruise_cluster_kph, 40.)

  def test_res_from_zero_at_standstill_returns_to_the_ceiling(self):
    self.engage_with_set()
    for _ in range(25):
      self.tap(ButtonType.decelCruise, hold_frames=3)
    self.assertEqual(self.helper.v_cruise_kph, 0.)
    self.v_ego_kph, self.standstill = 0., True
    self.tap(ButtonType.accelCruise)
    self.assertEqual(self.helper.v_cruise_kph, round(pcm_ceiling_base_kph(self.cs(), True), 1))
    self.assertEqual(self.helper.v_cruise_cluster_kph, 65.)

  def test_cancel_at_zero_then_res_returns_to_the_ceiling(self):
    self.engage_with_set()
    for _ in range(25):
      self.tap(ButtonType.decelCruise, hold_frames=3)
    self.engaged = False
    self.step(enabled=False, n=20)
    self.assertEqual(self.helper.v_cruise_kph, 0.)
    self.step(enabled=False, events=[(ButtonType.accelCruise, True)])
    self.engaged = True
    self.step(enabled=False, events=[(ButtonType.accelCruise, False)])
    self.step(enabled=False, n=5)
    self.step(enabled=True, n=5)
    self.assertEqual(self.helper.v_cruise_kph, round(pcm_ceiling_base_kph(self.cs(), True), 1))

  def test_long_press_plus_follows_the_altis_pcm_climb(self):
    # 11-05 09:15:06: RES held 1.16 s, the PCM climbed 30 -> 45; the target follows it as it always did on the Altis
    self.engage_with_set()
    self.step(events=[(ButtonType.accelCruise, True)])
    for i in range(TOYOTA_VIRTUAL_CRUISE_LONG_PRESS + 40):
      if i % 40 == 12:
        self.pcm_kph += 5
        self.pcm_dial_kph += 5
      self.step()
    self.step(events=[(ButtonType.accelCruise, False)])
    self.assertEqual(self.pcm_dial_kph, 85.)  # four PCM steps while held
    self.assertEqual(self.helper.v_cruise_cluster_kph, 85.)

  def test_target_capped_at_pcm_plus_headroom(self):
    self.engage_with_set()
    for _ in range(6):
      self.tap(ButtonType.accelCruise)
    # ceiling base = the PCM number on the Altis dial line: dial 65 -> 60.0
    self.assertAlmostEqual(pcm_ceiling_base_kph(self.cs(), True), (65. - 3.24) / 1.029, places=3)
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
    self.assertEqual(self.helper.v_cruise_cluster_kph, 55.)
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
    self.assertEqual(self.helper.v_cruise_cluster_kph, 55.)

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
