from dragonpilot.selfdrive.selfdrived.alka_chime import alka_chime_on_main


def test_chime_on_main_rising_edge():
  assert alka_chime_on_main(True, True, False, False)


def test_no_chime_while_main_stays_on():
  assert not alka_chime_on_main(True, True, True, False)


def test_no_chime_on_main_off():
  assert not alka_chime_on_main(True, False, True, False)


def test_no_chime_without_alka():
  assert not alka_chime_on_main(False, True, False, False)


def test_no_double_chime_when_engaged():
  assert not alka_chime_on_main(True, True, False, True)
