"""
dragonpilot (dpcr, Corolla Cross Hybrid): the set speed box of tncr1's mici HUD, ported from the owner's sunnypilot
branch tncr1 (selfdrive/ui/sunnypilot/mici/onroad/hud_renderer.py + dial_speed.py, mici hud_renderer.py PCM ceiling).
Owner 2026-11-03: "tncr1's speed display - set speed and real speed - to dpcr too".

While openpilot owns the Toyota set speed (opendbc toyota/software_set_speed.py) the box in the top left is:
- the large number: the CURRENT speed on the speedometer's scale (owner 2026-10-25: "the true speed in place of MAX,
  in sync with the speedometer"), dial = TOYOTA_DIAL_SLOPE * vEgo + TOYOTA_DIAL_OFFSET_KPH (Corolla Cross route 47
  2026-09-25, road check 80 -> 80, 86 -> 86, 107 -> 107), the offset fading in up to OFFSET_FULL_KPH because the dial
  reads 0 at a standstill; a HYST hysteresis keeps the number from flickering between two integers;
- below it, where stock writes MAX: the SET speed (already on the speedometer's scale, vCruiseCluster), 54 px, green
  (owner 2026-10-25 / 10-27);
- (the PCM's own set speed right of it was removed 2026-11-07, owner: "I don't need to see the PCM number on the C4")
- the box stays up for the whole engagement instead of fading out 2.5 s after the last change.
Other cars (no software set speed) keep the stock box.
"""
import numpy as np
import pyray as rl

from openpilot.common.constants import CV
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.ui_state import ui_state
from opendbc.car.toyota.software_set_speed import software_set_speed
from dragonpilot.selfdrive.car.toyota_set_speed import TOYOTA_DIAL_SLOPE, TOYOTA_DIAL_OFFSET_KPH
from dragonpilot.selfdrive.ui.mici import lane_change_hud

KM_TO_MILE = 0.621371
OFFSET_FULL_KPH = 30.0  # km/h (vEgo) from which the full dial offset applies
HYST = 0.6  # display units
SPEED_LABEL_SIZE = 54  # px, the set speed under the current speed
SPEED_LABEL_RGB = (0, 230, 90)  # green
LARGE_LABEL_RAISE = 12  # px
LARGE_LABEL_FREE = 60  # px: labels up to this size fit above the wheel icon unraised
BOX_MARGIN = 10  # px kept free right of the box for a lane change banner (lane_change_hud.py)


def dial_kph(v_ego_kph: float) -> float:
  v = max(v_ego_kph, 0.0)
  return TOYOTA_DIAL_SLOPE * v + TOYOTA_DIAL_OFFSET_KPH * min(v / OFFSET_FULL_KPH, 1.0)


class DialHud:
  def __init__(self):
    self.active = False
    self.shown = 0

  def update(self, car_state) -> None:
    CP = ui_state.CP
    self.active = CP is not None and software_set_speed(CP)
    lane_change_hud.set_active(self.active)
    lane_change_hud.set_room(0.0)  # set again by draw() when the box is on screen
    if not self.active:
      return
    value = dial_kph(car_state.vEgo * CV.MS_TO_KPH)
    if not ui_state.is_metric:
      value *= KM_TO_MILE
    if abs(value - self.shown) >= HYST or value < 0.5:
      self.shown = int(round(value))

  def draw(self, hud, rect: rl.Rectangle) -> None:
    """replaces HudRenderer._draw_set_speed (same position, fonts and drop shadow)"""
    alpha = hud._set_speed_alpha_filter.update(hud._can_draw_top_icons and hud._engaged and hud.is_cruise_set)
    if alpha < 1e-2:
      lane_change_hud.set_room(0.0)
      return
    x, y = rect.x, rect.y
    big_size = 112  # FONT_SIZES.set_speed
    circle_radius = 162 // 2
    rl.draw_circle_gradient(rl.Vector2(x + circle_radius, y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    rl.draw_text_ex(hud._font_display, str(self.shown), rl.Vector2(x + 13 + 4, y + 3 - 8 - 3 + 4), big_size, 0,
                    rl.Color(255, 255, 255, int(255 * 0.9 * alpha)))

    set_speed = hud.set_speed if ui_state.is_metric else hud.set_speed * KM_TO_MILE
    label = str(round(set_speed))
    label_y = y + big_size - 7 + 4 - float(np.clip(SPEED_LABEL_SIZE - LARGE_LABEL_FREE, 0, LARGE_LABEL_RAISE))
    rl.draw_text_ex(hud._font_semi_bold, label, rl.Vector2(x + 25, label_y), SPEED_LABEL_SIZE, 0,
                    rl.Color(*SPEED_LABEL_RGB, int(255 * 0.9 * alpha)))

    lane_change_hud.set_room(self.box_width(hud, label))

  def box_width(self, hud, label: str) -> float:
    """width of the box as drawn above (same x offsets and fonts) plus BOX_MARGIN"""
    w = 17 + measure_text_cached(hud._font_display, str(self.shown), 112).x
    label_w = 25 + measure_text_cached(hud._font_semi_bold, label, SPEED_LABEL_SIZE).x
    return max(w, label_w) + BOX_MARGIN
