from types import SimpleNamespace

from cereal import car, log
from openpilot.selfdrive.selfdrived.events import Events, ET, EVENTS
import dragonpilot.selfdrive.selfdrived.experimental_toggle as et

ButtonType = car.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName


class FakeParams:
  def __init__(self):
    self.store = {}

  def put_bool(self, k, v, block=False):
    self.store[k] = v

  def get_bool(self, k, block=False):
    return self.store.get(k, False)


def cs(main=True, press=None):
  bes = [] if press is None else [SimpleNamespace(type=ButtonType.gapAdjustCruise, pressed=press)]
  return SimpleNamespace(cruiseState=SimpleNamespace(available=main), buttonEvents=bes)


def make(long=True):
  p = FakeParams()
  return et.ExperimentalModeToggle(SimpleNamespace(openpilotLongitudinalControl=long), p), p


def hold(t, frames, exp=False, main=True):
  """press, hold for `frames` frames; returns the events of every frame"""
  out = []
  for i in range(frames):
    ev = Events()
    t.update(cs(main, True if i == 0 else None), ev, exp)
    out.append(ev.names)
  return out


def test_long_hold_toggles_on_once():
  t, p = make()
  frames = hold(t, 200, exp=False)
  hits = [i for i, n in enumerate(frames) if EventName.experimentalModeSwitched in n]
  assert hits == [et.DISTANCE_LONG_PRESS - 1]
  assert p.store["ExperimentalMode"] is True
  assert et.experimental_toggle_text() == "Experimental Mode ON"


def test_long_hold_toggles_off():
  t, p = make()
  hold(t, 60, exp=True)
  assert p.store["ExperimentalMode"] is False
  assert et.experimental_toggle_text() == "Experimental Mode OFF"


def test_short_press_no_toggle_and_personality_steps():
  t, p = make()
  frames = hold(t, 20)
  assert not any(EventName.experimentalModeSwitched in n for n in frames)
  t.update(cs(press=False), Events(), False)
  assert t.release_after_hold() is False  # personality still steps on a short press


def test_release_after_long_hold_skips_personality_once():
  t, p = make()
  hold(t, 60)
  t.update(cs(press=False), Events(), True)
  assert t.release_after_hold() is True
  hold(t, 10, exp=True)
  t.update(cs(press=False), Events(), True)
  assert t.release_after_hold() is False


def test_main_off_or_no_op_long_no_toggle():
  t, p = make()
  assert not any(hold(t, 100, main=False))
  t, p = make(long=False)
  assert not any(hold(t, 100))


def test_alert_defined():
  assert ET.WARNING in EVENTS[EventName.experimentalModeSwitched]
