from cereal import log
from openpilot.selfdrive.selfdrived.events import Events, ET, EVENTS
from dragonpilot.selfdrive.selfdrived.alka_sounds import alka_sounds

EventName = log.OnroadEvent.EventName


def run(names, alka=True, main_on=True, main_on_prev=True):
  ev = Events()
  for n in names:
    ev.add(n)
  alka_sounds(ev, alka, main_on, main_on_prev)
  return ev.names


def test_main_on_engage_chime():
  assert EventName.alkaEnable in run([], main_on=True, main_on_prev=False)


def test_main_on_blocked_by_no_entry_is_silent():
  # like MADS: gear in P -> no engage chime
  names = run([EventName.wrongGear], main_on=True, main_on_prev=False)
  assert EventName.alkaEnable not in names


def test_main_off_disengage_chime():
  assert EventName.alkaDisable in run([], main_on=False, main_on_prev=True)


def test_main_off_while_engaged_plays_only_alka_disengage():
  names = run([EventName.pcmDisable], main_on=False, main_on_prev=True)
  assert EventName.pcmDisable not in names and EventName.alkaDisable in names


def test_set_is_silent():
  assert run([EventName.pcmEnable]) == []
  assert run([EventName.buttonEnable]) == []


def test_cancel_brake_pcm_off_silent():
  assert run([EventName.pcmDisable]) == []
  assert run([EventName.buttonCancel]) == []
  assert run([EventName.pedalPressed]) == []


def test_pedal_refusal_kept_on_engage_attempt():
  names = run([EventName.pcmEnable, EventName.pedalPressed])
  assert names == [EventName.pedalPressed]


def test_steady_main_no_chime():
  assert run([], main_on=True, main_on_prev=True) == []
  assert run([], main_on=False, main_on_prev=False) == []


def test_alka_off_keeps_stock_sounds():
  names = [EventName.pcmEnable, EventName.pcmDisable, EventName.buttonCancel, EventName.pedalPressed]
  assert run(names, alka=False, main_on=True, main_on_prev=False) == sorted(names)


def test_chime_alerts_defined():
  assert ET.PERMANENT in EVENTS[EventName.alkaEnable]
  assert ET.PERMANENT in EVENTS[EventName.alkaDisable]
