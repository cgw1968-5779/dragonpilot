from types import SimpleNamespace

from dragonpilot.selfdrive.ui.mici import lane_change_hud as lch


def test_keeps_hud_only_for_lane_change_alerts():
  lch.set_active(True)
  for ev in ('preLaneChangeLeft', 'preLaneChangeRight', 'laneChange', 'laneChangeBlocked', 'laneChangeRoadEdge'):
    assert lch.keeps_hud(SimpleNamespace(alert_type=f'{ev}/warning'))
  for ev in ('steerSaturated/warning', 'promptDriverDistracted/permanent', ''):
    assert not lch.keeps_hud(SimpleNamespace(alert_type=ev))
  assert not lch.keeps_hud(None)
  lch.set_active(False)  # no software set speed: stock
  assert not lch.keeps_hud(SimpleNamespace(alert_type='laneChange/warning'))


def test_make_room():
  lch.set_room(200.4)
  # no icon / right icon: text starts right of the box
  assert lch.make_room('laneChange', None, 18, 458, 20, 0) == (201, 275, 20)
  # left icon: icon and text both move by the room
  assert lch.make_room('preLaneChangeLeft', 'left', 104, 354, 2, 0) == (305, 153, 203)
  # other alerts untouched
  assert lch.make_room('steerSaturated', None, 18, 458, 20, 0) == (18, 458, 20)
  # box hidden: untouched
  lch.set_room(0.0)
  assert lch.make_room('laneChange', None, 18, 458, 20, 0) == (18, 458, 20)
