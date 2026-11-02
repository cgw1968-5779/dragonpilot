# dragonpilot: Toyota software set speed (Corolla Cross short press +/-5), CAN side.
# Ported from sunnypilot tncr1 (tncr18 7bbad13422 + cfd771a7b8).
#
# openpilot tracks the cruise set speed itself instead of mirroring PCM_CRUISE_2->SET_SPEED. The PCM still owns
# engagement (pcmCruise stays True, so cancel and panda safety are unchanged); it only stops owning the target speed.
# The set speed logic lives in dragonpilot/selfdrive/car/toyota_set_speed.py.
#
# Toyota has no dedicated cruise button message on the powertrain bus, but CLUTCH (0x361) byte 0 carries the raw
# stalk switch state: bit 5 RES/+, bit 4 SET/-, bit 3 CANCEL, held for the real ~0.5 s of a physical short press.
# PCM_CRUISE->CRUISE_STATE 9/10 ("click up/down") must NOT be used as the button source: it reflects the PCM
# accelerating/decelerating towards its own set speed and persists for seconds whenever the car is above or below
# that number, which openpilot then mistakes for a long press.
from opendbc.car import create_button_events, structs
from opendbc.car.toyota.values import CAR

ButtonType = structs.CarState.ButtonEvent.Type

CLUTCH_FREQ = 15  # Hz, 0x361 runs at ~15.5 Hz on the Corolla Cross TSS2


class CruiseButton:
  none = 0
  plus = 1
  minus = 2


CRUISE_BUTTONS_DICT = {
  CruiseButton.plus: ButtonType.accelCruise,
  CruiseButton.minus: ButtonType.decelCruise,
}


def software_set_speed(CP) -> bool:
  """Corolla Cross (fingerprints as COROLLA_TSS2) with openpilot longitudinal: openpilot owns the set speed."""
  return CP.carFingerprint == CAR.TOYOTA_COROLLA_TSS2 and CP.openpilotLongitudinalControl and CP.pcmCruise


class CruiseStalk:
  def __init__(self):
    self.button = CruiseButton.none

  def update(self, cp) -> list[structs.CarState.ButtonEvent]:
    """accelCruise/decelCruise events from the raw RES/SET stalk bits in CLUTCH (0x361).

    Every sample received this cycle is walked in order, so a press and its release landing in the same 100 Hz
    iteration still produce both edges. RES and SET together is physically impossible and is treated as unpressed.
    CANCEL is left to the PCM (it disengages cruise on its own)."""
    events: list[structs.CarState.ButtonEvent] = []
    for res, set_ in zip(cp.vl_all["CLUTCH"]["CRUISE_BTN_RES"], cp.vl_all["CLUTCH"]["CRUISE_BTN_SET"], strict=True):
      if bool(res) != bool(set_):
        button = CruiseButton.plus if res else CruiseButton.minus
      else:
        button = CruiseButton.none
      events += create_button_events(button, self.button, CRUISE_BUTTONS_DICT, unpressed_btn=CruiseButton.none)
      self.button = button
    return events
