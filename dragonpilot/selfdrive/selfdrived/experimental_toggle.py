"""Hold the distance button to toggle Experimental Mode, as on tnpb3 / tncr1 (sunnypilot CruiseHelper).

- With ACC MAIN on and openpilot longitudinal, holding the distance button for DISTANCE_LONG_PRESS frames
  (0.5 s at 100 Hz) flips ExperimentalMode once per hold.
- The alert says which mode it is now: "Experimental Mode ON" / "OFF" (tnpb3 d27879ee / 8d80eeb4: the text uses
  the value the hold switched to, not a param read-back, because the non-blocking param write can land late).
- The release that ends a long hold does not also step the driving personality; a short press still does.
"""
from cereal import car, log
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName

DISTANCE_LONG_PRESS = 50

# the mode the last hold switched to, for the alert text (selfdrived runs this and builds the alerts)
last_experimental_toggle: bool | None = None


def experimental_toggle_text() -> str:
  on = last_experimental_toggle
  if on is None:
    on = Params().get_bool("ExperimentalMode")
  return f"Experimental Mode {'ON' if on else 'OFF'}"


class ExperimentalModeToggle:
  def __init__(self, CP, params=None):
    self.CP = CP
    self.params = params if params is not None else Params()
    self.frames = 0
    self.switched = False

  def update(self, CS, events, experimental_mode: bool) -> None:
    if not (self.CP.openpilotLongitudinalControl and CS.cruiseState.available):
      return

    if self.frames > 0:
      self.frames += 1
    for be in CS.buttonEvents:
      if be.type == ButtonType.gapAdjustCruise:
        self.frames = int(be.pressed)

    if self.frames >= DISTANCE_LONG_PRESS and not self.switched:
      global last_experimental_toggle
      last_experimental_toggle = not experimental_mode
      self.params.put_bool("ExperimentalMode", last_experimental_toggle)  # non-blocking by default
      events.add(EventName.experimentalModeSwitched)
      self.switched = True

  def release_after_hold(self) -> bool:
    """Called on a distance button release: True (skip the personality step) if that press was a long hold."""
    held, self.switched = self.switched, False
    return held
