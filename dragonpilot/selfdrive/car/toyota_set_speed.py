"""
dragonpilot: Toyota software set speed - dppb (Corolla Altis Hybrid) copy of dpcr's module (owner 2026-11-05: set
speed down to 0, settable 25/20/15/10/5 below 30, "dppb and dpcr both"). The Altis PCM itself already steps 5 km/h per
press (ALLOW_LONG_PRESS 2) but stops at 30, so openpilot owns the target as on the Cross. Altis differences:
- dial calibration from the Altis's own PCM pairs (11-05 route, cruiseState.speed -> speedCluster: 26->30, 31->35,
  36->40, 41->45, 45->50, 50->55, 55->60, 60->65): dial = 1.029 * PCM speed + 3.24. The canonical target IS the
  PCM speed, so from 30 up the Altis drives exactly as before (it used cruiseState.speed directly).
- a long press of + keeps the Altis's stock "hold + to climb": the target goes up to the PCM's new number.
Cross (dpcr) notes below apply unless stated.
Ported from sunnypilot tncr1 (selfdrive/car/cruise.py + sunnypilot cruise_ext.py, tncr18 7bbad13422..f89b54e8e0).

- A short press moves openpilot's own target by 5 km/h; the displayed number snaps onto the 5 km/h grid in the press
  direction (86 -> 90 -> 85 ...). Rapid taps accumulate: every release in a cycle counts.
- A long press (held > 1 s) only moves the PCM's own number, which acts as a ceiling: the PCM will not hold the car
  more than ~8 km/h above it, so openpilot's target is capped at PCM set speed + 7 km/h. The driver parks the
  ceiling high once per drive with a long press and uses short presses underneath it.
- The target is kept through a cancel (brake, CANCEL) and a RES resume; a fresh SET mirrors the PCM number.
- The number the driver sees is on the speedometer (dial) scale; the planner drives the canonical speed under it.
- The increment is fixed at 5 km/h (no params).
- Set speed down to 0 (owner 2026-11-05, port of tncr1 cadbfd2 2026-10-27, both cars): below 30 the - presses keep
  stepping 25, 20, 15, 10, 5, 0 (the PCM's own number stays at its 30 minimum: it is only the ceiling), and 0 is a stop
  target (cruise_decel slows to it). A long press of - (held > 1 s) lowered the target by 10 km/h (on
  the dial, floor 0) instead of leaving it alone. From 0, RES at standstill or a RES resume returns to the PCM ceiling
  (the number on the dash) so the car can move off; while rolling, + from 0 steps 5, 10, ...

- Owner 2026-11-08 ("the dash set speed drifts away from the C3X's with use - keep them the same"): from 30 up the
  target now MIRRORS the Altis PCM's own number. The Altis PCM steps 5 km/h per press AND again every 0.25-0.45 s while
  held, while openpilot counted a press on release and only > 1 s as long, so a ~0.5 s press moved the dash twice and
  openpilot once (route 19c 11:48: dash 110 -> 100, C3X 110 -> 105, then 5 off for the rest of the drive; routes
  193 / 194 the same, both directions, up to 15 km/h). Now every change of the PCM's dash number is copied (whatever
  the press length), short presses above 30 no longer step openpilot's own number, and openpilot only owns what the
  PCM cannot do: the stop target 0 below its 30 minimum (short - with the PCM already at 30 -> 0; RES/+ from 0 -> the
  PCM number; - at 0 stays 0). A long - simply follows the PCM down to its 30 (the old "jump to 30 at 1 s" is gone:
  it set 30 while the dash still showed e.g. 60).

VCruiseHelper.update_v_cruise hands the whole update to ToyotaSoftwareSetSpeed.update() (one dragonpilot hook); the
targets and button timers stay on the helper, so card.py publishes them unchanged.
- Owner 2026-11-07 (dpcr a5080c12 + cb9fca2c, "dppb too"): from 30 on the dial (or below) one short - press goes
  straight to 0 (no 25 ... 5 steps); a long press of - (held 1 s) from above 30 goes straight to 30 and stops there
  (to reach 0: release, then one short -); RES/+ from 0 always goes straight back to the PCM ceiling (the number on
  the dash), short or long, stopped or rolling. On the Altis this matters even more than on the Cross: the PCM never
  sets cruiseState.standstill while openpilot holds the car (route 190, 8 stops, 184 s: always 0), so the old
  "RES at standstill" branch never ran and RES from 0 at a stop stepped 5, 10, ... The Altis PCM steps its own
  number 5 km/h per press and every 0.25-0.45 s while held (route 190: 65 -> 30 in 1.3 s), so after going down to 0
  its number (the RES ceiling) is usually near 30.
"""
import math
import numpy as np

from cereal import car
from openpilot.common.constants import CV

ButtonType = car.CarState.ButtonEvent.Type

V_CRUISE_MIN = 0  # km/h: 0 is the stop target (owner 2026-11-05; was 30 metric / 20 mph)
V_CRUISE_MIN_IMPERIAL = 0
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)
SHORT_PRESS_INCREMENT = 5
ZERO_FROM_DIAL = 30.  # km/h on the dial: a short - press from here or below goes straight to 0 (owner 2026-11-07)

# A physical short press is 450-520 ms on the bus; the PCM itself treats a longer hold as a long press and steps its
# own set speed. Err on the long side: if the PCM sees a long press and we see a short one both step by 5, the other
# way round we merely resync to the PCM's +1 (tncr18 cefe9d7fe4).
TOYOTA_VIRTUAL_CRUISE_LONG_PRESS = 100  # frames at 100 Hz

# The PCM will not hold the car more than roughly 8 km/h above its OWN set speed even while openpilot commands positive
# acceleration: it cuts throttle above about +11 and resumes below about +7.5, which turns a larger gap into a speed
# oscillation. 7 km/h keeps the gap below the throttle-cut point while leaving the + button room (tncr18 a2e52997a8).
TOYOTA_PCM_SET_SPEED_HEADROOM_KPH = 7.

# The PCM keeps its set speed in two units: SET_SPEED (0xB4 units, ~true speed) and UI_SET_SPEED (what the cluster
# shows). Corolla Cross pairs (rlog 2026-09-25, route 47): 26->30, 50->55, 74->80, 93->100, 112->120, 120->128;
# UI = 1.0457 * internal + 2.73. With 0xB4 / vEgo = 1.021: dial = TOYOTA_DIAL_SLOPE * canonical + offset.
# Road check: displayed 80 drove canonical 78.4 (dial 86), displayed 100 drove 97.9 (dial 107) (tncr18 f89b54e8e0).
TOYOTA_DIAL_SLOPE = 1.029       # dppb: Altis PCM pairs 11-05 (see the top); Cross 1.0457 * 1.021
TOYOTA_DIAL_OFFSET_KPH = 3.24   # dppb: Altis; Cross 2.53

# Ceiling base = the PCM set speed on the speedometer model, as tncr1 feeds it in cruiseState.speed
# (carstate_ext.set_speed_target, owner 2026-10-07): the true speed at which the dash reads the UI set speed + 0.3.
# Corolla Cross route ae 2026-10-25: UI 105 -> 98.9 (raw SET_SPEED 98). Metric dash only, else the raw SET_SPEED.
CLUSTER_SPEED_GAIN = TOYOTA_DIAL_SLOPE           # dppb: the PCM number on the Altis's own dial line (Cross 1.05)
CLUSTER_SPEED_OFFSET_KPH = TOYOTA_DIAL_OFFSET_KPH  # (Cross 1.47)
SET_SPEED_SCREEN_MARGIN_KPH = 0.0                 # (Cross 0.3)
CLUSTER_MIN_KPH = 5.0

CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}
ENABLE_BUTTON_TIMER_KEYS = (ButtonType.decelCruise, ButtonType.accelCruise, ButtonType.setCruise,
                            ButtonType.resumeCruise, ButtonType.cancel, ButtonType.mainCruise)
RES_RESUME_WINDOW = 150  # frames: an engagement within 1.5 s of a RES/+ press is a resume


def toyota_dial_from_canonical(kph: float) -> float:
  if kph <= 0.0:
    return 0.0
  dial = TOYOTA_DIAL_SLOPE * kph + TOYOTA_DIAL_OFFSET_KPH
  # the canonical target is kept to 0.1, which can put the dial 0.1 off the integer the driver dialed; snap back
  # onto it so the HUD shows the round number and the +/- snap grid keeps working
  nearest = round(dial)
  return float(nearest) if abs(dial - nearest) < 0.15 else round(dial, 1)


def toyota_canonical_from_dial(kph: float) -> float:
  return max(0.0, (kph - TOYOTA_DIAL_OFFSET_KPH) / TOYOTA_DIAL_SLOPE)


def pcm_ceiling_base_kph(CS, is_metric: bool) -> float:
  ui_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
  if not is_metric or CS.cruiseState.speed <= 0.0 or ui_kph < 2 * CLUSTER_MIN_KPH:
    return CS.cruiseState.speed * CV.MS_TO_KPH
  return (ui_kph + SET_SPEED_SCREEN_MARGIN_KPH - CLUSTER_SPEED_OFFSET_KPH) / CLUSTER_SPEED_GAIN


class ToyotaSoftwareSetSpeed:
  def __init__(self):
    self.v_cruise_min = V_CRUISE_MIN
    self.enable_button_timers = dict.fromkeys(ENABLE_BUTTON_TIMER_KEYS, 0)
    self.enabled_prev = False
    self.frame = 0
    self.last_press: tuple[int | None, int] = (None, -10**9)
    self.engaged_prev = False
    self.engaged_by_set = True
    self.pcm_dial_prev: float | None = None  # the PCM's dash number last frame (km/h), to mirror its changes
    self.pcm_dial_at_press = 0.0  # the PCM's dash number just before the last - press

  @staticmethod
  def initialized(h) -> bool:
    return 0 <= h.v_cruise_kph < V_CRUISE_UNSET and 0 <= h.v_cruise_cluster_kph < V_CRUISE_UNSET

  def update(self, h, CS, enabled: bool, is_metric: bool) -> None:
    """h is the VCruiseHelper; v_cruise_kph_last is already stored by the caller."""
    self.v_cruise_min = V_CRUISE_MIN if is_metric else V_CRUISE_MIN_IMPERIAL
    _enabled = self.update_enabled_state(CS, enabled)
    hold_own_target = self.hold_own_target(h, CS, _enabled, is_metric)

    if CS.cruiseState.available:
      if hold_own_target:
        pass
      elif _enabled and self.initialized(h):
        self.follow_pcm(h, CS, is_metric)
        self.short_presses(h, CS, is_metric)
        # tncr1 re-applies a zero delta every frame (its speed limit assist hook): keeps the canonical target inside
        # [30, 145] km/h and the displayed number on the dial line even when the PCM was SET below 30 canonical
        self.apply_dial_delta(h, 0.)
      else:
        # a fresh SET (or before the first engagement) mirrors the PCM: keep the cluster's number and drive the
        # canonical speed that puts the dial on it (SET_SPEED itself is in 0xB4 units, ~2 km/h high on the dial)
        h.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        h.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speedCluster > 0:
          h.v_cruise_kph = round(toyota_canonical_from_dial(h.v_cruise_cluster_kph), 1)
        if CS.cruiseState.speed == 0:
          h.v_cruise_kph = V_CRUISE_UNSET
          h.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          h.v_cruise_kph = -1
          h.v_cruise_cluster_kph = -1
    else:
      h.v_cruise_kph = V_CRUISE_UNSET
      h.v_cruise_cluster_kph = V_CRUISE_UNSET

    self.update_button_timers(h, CS, enabled)
    self.pcm_ceiling(h, CS, is_metric)
    self.pcm_dial_prev = CS.cruiseState.speedCluster * CV.MS_TO_KPH if CS.cruiseState.speed > 0 else None

  def follow_pcm(self, h, CS, is_metric: bool) -> None:
    """dppb (owner 2026-11-08): copy every change of the PCM's dash number, so the C3X always shows the dash's number.
    Only the stop target 0 is kept when the PCM steps down (it is openpilot's own number below the PCM's 30)."""
    for b in CS.buttonEvents:
      if b.pressed and b.type == ButtonType.decelCruise:
        self.pcm_dial_at_press = self.pcm_dial_prev if self.pcm_dial_prev is not None else 0.0
    if CS.cruiseState.speed <= 0 or self.pcm_dial_prev is None:
      return
    pcm_dial = CS.cruiseState.speedCluster * CV.MS_TO_KPH
    if abs(pcm_dial - self.pcm_dial_prev) < 0.5:
      return
    if h.v_cruise_kph == 0 and pcm_dial < self.pcm_dial_prev:
      return
    self.to_ceiling(h, CS, is_metric)

  def update_enabled_state(self, CS, enabled: bool) -> bool:
    """Ignore the engagement while the button that engaged is still held, so the press that engages (SET/-) is not
    also counted as a set speed change on its release."""
    for k in self.enable_button_timers:
      if self.enable_button_timers[k] > 0:
        self.enable_button_timers[k] += 1
    for b in CS.buttonEvents:
      if b.type.raw in self.enable_button_timers:
        self.enable_button_timers[b.type.raw] = 1 if b.pressed else 0
    button_pressed = any(t > 0 for t in self.enable_button_timers.values())

    if enabled and not self.enabled_prev:
      self.enabled_prev = not button_pressed
      enabled = False
    elif not enabled:
      self.enabled_prev = enabled
    return enabled and self.enabled_prev

  def to_ceiling(self, h, CS, is_metric: bool) -> None:
    """From the stop target 0: back to the PCM's own number (the dash), so RES moves the car off."""
    if CS.cruiseState.speed > 0:
      h.v_cruise_kph = round(pcm_ceiling_base_kph(CS, is_metric), 1)
      h.v_cruise_cluster_kph = toyota_dial_from_canonical(h.v_cruise_kph)

  def hold_own_target(self, h, CS, _enabled: bool, is_metric: bool = True) -> bool:
    """While cruise is cancelled (brake, CANCEL) and through a RES resume, openpilot's own target is kept. Only a fresh
    SET (or an engagement without a RES press just before) mirrors the PCM. Never holds before the target is set."""
    if not self.initialized(h):
      return False

    self.frame += 1
    for b in CS.buttonEvents:
      if b.pressed and b.type in (ButtonType.accelCruise, ButtonType.decelCruise):
        self.last_press = (b.type.raw, self.frame)

    engaged = CS.cruiseState.enabled
    if engaged and not self.engaged_prev:
      press_type, press_frame = self.last_press
      resumed_with_res = press_type == ButtonType.accelCruise and self.frame - press_frame < RES_RESUME_WINDOW
      self.engaged_by_set = not resumed_with_res
      if resumed_with_res:
        self.to_ceiling(h, CS, is_metric)  # dppb 11-08: a RES resume takes the dash's number (also from the stop target)
    self.engaged_prev = engaged

    if _enabled:
      return False  # the software set speed owns the target
    if not engaged:
      return True   # cancelled: hold
    return not self.engaged_by_set  # engagement transition frames: hold after RES, mirror after SET

  def short_presses(self, h, CS, is_metric: bool) -> None:
    """dppb (owner 2026-11-08): from 30 up the PCM's own steps are mirrored (follow_pcm), so a release only handles the
    stop target the PCM cannot reach: a short - released while the PCM was already at its minimum (dash <= 30) goes
    to 0, and RES/+ from 0 (short or long, stopped or rolling) goes back to the PCM's number."""
    for b in CS.buttonEvents:
      if b.type.raw not in h.button_timers or b.pressed:
        continue
      hold = h.button_timers[b.type.raw]
      if hold <= 0:
        continue  # spurious
      st = h.button_change_states[b.type.raw]
      if b.type.raw == ButtonType.accelCruise and h.v_cruise_kph == 0:
        self.to_ceiling(h, CS, is_metric)
        continue
      if b.type.raw != ButtonType.decelCruise or hold > TOYOTA_VIRTUAL_CRUISE_LONG_PRESS or not st["enabled"]:
        continue
      if self.pcm_dial_at_press <= ZERO_FROM_DIAL + 0.05 and h.v_cruise_cluster_kph <= ZERO_FROM_DIAL + 0.05:
        # if - is tapped while overriding, do not lower the target below the current speed
        target = max(0., CS.vEgo * CV.MS_TO_KPH) if CS.gasPressed else 0.
        if target < h.v_cruise_kph:
          h.v_cruise_kph = round(target, 1)
          h.v_cruise_cluster_kph = toyota_dial_from_canonical(h.v_cruise_kph)

  def apply_dial_delta(self, h, delta_kph: float) -> None:
    """Apply a delta in DISPLAY (speedometer) space and keep the canonical target on the dial's calibration line."""
    new_kph = toyota_canonical_from_dial(h.v_cruise_cluster_kph + delta_kph)
    h.v_cruise_kph = round(float(np.clip(new_kph, self.v_cruise_min, V_CRUISE_MAX)), 1)
    h.v_cruise_cluster_kph = toyota_dial_from_canonical(h.v_cruise_kph)

  def update_button_timers(self, h, CS, enabled: bool) -> None:
    if not enabled or not CS.cruiseState.available or not self.initialized(h):
      for k in h.button_timers:
        h.button_timers[k] = 0
        h.button_change_states[k] = {"standstill": False, "enabled": False}
      return
    h.update_button_timers(CS, enabled)

  def pcm_ceiling(self, h, CS, is_metric: bool) -> None:
    """Cap the driver's target at the PCM number + headroom; the HUD shows it on the speedometer's scale."""
    if not (self.initialized(h) and CS.cruiseState.speed > 0):
      return
    pcm_kph = round(pcm_ceiling_base_kph(CS, is_metric), 1)
    h.v_cruise_kph = min(h.v_cruise_kph, round(pcm_kph + TOYOTA_PCM_SET_SPEED_HEADROOM_KPH, 1))
    h.v_cruise_cluster_kph = toyota_dial_from_canonical(h.v_cruise_kph)
