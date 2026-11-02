"""ALKA engage / disengage sounds, the same as tnpb3 / tncr1 (sunnypilot MADS, default settings).

With ALKA on, ACC MAIN is the lane keeping switch, so the chimes follow MAIN instead of SET:
- MAIN on   -> engage chime (MADS lkasEnable). Not when openpilot can't engage right now (any NO_ENTRY
               event, e.g. gear in P), like MADS, which then stays paused silently.
- MAIN off  -> disengage chime (MADS lkasDisable).
- SET / RES while MAIN is on -> silent (MADS unified engagement: lateral is already on).
- CANCEL, PCM cruise off, brake/gas pedal disengage -> silent (MADS removes pcmDisable / buttonCancel /
  pedalPressed). A pedal held while trying to engage still gives the "Pedal Pressed" refusal, like MADS.

Runs after the state machine and before the alerts are built, so it only changes which sounds play,
never whether openpilot engages or disengages.
"""
from cereal import log
from openpilot.selfdrive.selfdrived.events import ET

EventName = log.OnroadEvent.EventName

ENABLE_EVENTS = (EventName.pcmEnable, EventName.buttonEnable)
SILENT_DISABLE_EVENTS = (EventName.pcmDisable, EventName.buttonCancel)


def _remove(events, names) -> None:
  events.events = [e for e in events.events if e not in names]


def alka_sounds(events, alka: bool, main_on: bool, main_on_prev: bool) -> None:
  if not alka:
    return

  enable_attempt = any(e in events.events for e in ENABLE_EVENTS)

  if main_on:
    _remove(events, ENABLE_EVENTS)
  _remove(events, SILENT_DISABLE_EVENTS)
  if not enable_attempt:
    _remove(events, (EventName.pedalPressed,))

  if main_on and not main_on_prev and not events.contains(ET.NO_ENTRY):
    events.add(EventName.alkaEnable)
  elif main_on_prev and not main_on:
    events.add(EventName.alkaDisable)
