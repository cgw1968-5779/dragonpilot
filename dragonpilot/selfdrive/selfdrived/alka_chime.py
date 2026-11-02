"""ALKA engage chime.

With ALKA on, pressing ACC MAIN starts lane keeping silently (dp only chimes
when SET engages openpilot). This plays the normal engage chime on the MAIN
press, like sunnypilot MADS (lkasEnable) on tnpb3 / tncr1.
"""


def alka_chime_on_main(alka: bool, main_on: bool, main_on_prev: bool, enabled: bool) -> bool:
  """True on the frame ACC MAIN turns on while ALKA is enabled and openpilot is not engaged.

  SET already plays its own engage chime, so it is not doubled.
  """
  return alka and main_on and not main_on_prev and not enabled
