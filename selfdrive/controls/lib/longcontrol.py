import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# dragonpilot (dppb, Corolla Altis Hybrid, owner 2026-11-03): soft end of a stop.
# The planner already eases toward the stop, but below ~10 km/h the Altis decel lags the command by ~0.5 s
# (dppb routes 17f-188: best-fit lag 0.5 s, 0.1 s at 10-25 km/h), so a stop after a firmer middle (lead braked
# a little) still ends at -0.8..-0.9 m/s^2 and the car lurches at the stop. The decel at the stop follows the
# command ~0.6 s earlier almost 1:1 (7 stops). So when the time to the stop, from the actual speed and decel,
# drops below SOFT_END_LAG + SOFT_END_RAMP, the command eases to SOFT_END_ACCEL (owner's Altis end value
# at 0 km/h) and reaches it SOFT_END_LAG before the stop. Once easing starts it never firms again until the car
# has stopped (no stop-go-stop). Only ever lighter than the planner, only in the last ~1 s, so the start and the
# middle of every stop are unchanged.
SOFT_END_ACCEL = -0.30  # m/s^2
SOFT_END_LAG = 0.5      # s
SOFT_END_RAMP = 0.5     # s, shortest easing time
SOFT_END_MAX_RATE = 1.0  # m/s^3, easing never releases faster than this: a firmer brake starts easing earlier
SOFT_END_MIN_DECEL = 0.3  # m/s^2, floor of the decel used for the time to the stop
SOFT_END_RESET_V = 3.0  # m/s, a new stop can start easing again above this speed
SOFT_END_URGENT = -1.5  # m/s^2, never ease while the planner asks for more than this (urgent stop)

# dragonpilot (dpcr, owner 2026-11-03; dppb port 2026-11-04): "at mid / low speed, the first 0.1 s of the brake on the first SET / RESUME press
# should be softer". Engaging with a lead (or a set speed) that needs braking, the planner's target is applied at once
# and carcontroller's 4 m/s^3 wind-down is the only limit: -0.4 m/s^2 after 0.1 s, which is felt as a bite. A brake
# that starts within ENGAGE_ONSET_WINDOW of engaging now builds with a rising jerk: ENGAGE_ONSET_J0 at its start,
# growing linearly to ENGAGE_ONSET_J1 (= the stock 4 m/s^3 limit) over ENGAGE_ONSET_T, after which nothing is changed.
# -0.15 at 0.1 s, -0.4 at 0.2 s, -0.75 at 0.3 s (stock: -0.4 / -0.8 / -1.2): a -1.0 request is reached ~0.1 s later.
# Dynamic only: it follows the request (a light request is reached in the soft part and is not touched), only ever
# lighter than the planner, only for the first 0.3 s of the first brake after engaging; normal stops are unchanged.
# Not tied to a speed: a brake right at engagement only happens at mid / low speed in practice.
ENGAGE_ONSET_WINDOW = 1.0  # s after engaging in which a brake onset gets the soft start
ENGAGE_ONSET_J0 = 1.0      # m/s^3 at the start of the brake
ENGAGE_ONSET_J1 = 4.0      # m/s^3 at ENGAGE_ONSET_T (stock carcontroller wind-down limit)
ENGAGE_ONSET_T = 0.3       # s

# dragonpilot (dpcr, owner 2026-11-03): the end of a stop behind a stopped lead at tncr1's level. Owner: "the brake gets
# heavier at 3-5 km/h, too much; tncr1's slight firming there is comfortable - make dpcr the same as tncr1, check it
# against the old records". Real stops (11-03 dpcr route d2 vs 16 normal tncr1 stops 10-28/29, median): tncr1 SENT
# -0.57/-0.55/-0.57/-0.45/-0.37 at 6/5/4/3/2 km/h and the car gave -0.70/-0.41/-0.58/-0.69/-0.50 (the Cross gives
# ~1.23 x the command at 2-5 km/h: that is the slight firming the owner likes); dpcr sent -0.92/-0.70/-0.60/-0.49 at
# 6/5/4/3 and the car gave -0.96/-0.88/-0.88/-0.88, stopping 4.6 m out (tncr1 3.6 m). So below STOP_LEVEL_V_ARM, behind
# a (nearly) stopped radar lead, the command is never firmer than tncr1's sent level (STOP_LEVEL_A) - but only as far
# as the distance allows: the stop it predicts (lead distance less the lag travel, the distance the level line needs
# with the Cross's measured delivery) must still end STOP_LEVEL_GAP or more from the lead; where it would not, the
# command eases only to the decel that does. Dynamic: lead distance, speed and the planner's own request.
# Never firmer than the planner (it only ever lightens; when the planner firms it is followed at once), easing at
# most STOP_LEVEL_RATE, not in an urgent stop (planner below SOFT_END_URGENT), not without a stopped lead (light /
# creep stops and stops with the gap already short are unchanged). Closed-loop sim of the 17 d2 stops (x86 MPC):
# sent 5/4/3/2 km/h -0.55/-0.57/-0.45/-0.36 (tncr1 -0.55/-0.57/-0.45/-0.37), car 4/3 km/h -0.94/-0.82 -> -0.72/-0.61,
# gaps 18:53 4.6 -> 3.4, 19:47 4.5 -> 3.6, 19:54 4.1 -> 3.0, 19:57 3.2 -> 2.8, 20:42 3.3 -> 3.0 m; the other 12 stops
# (urgent 18:43 included) identical. Same method as the soft end (no new layer); the lead comes from controlsd.
STOP_LEVEL_V = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]                  # km/h
# dppb (Altis), owner 2026-11-04: "push these to dppb too; I test it and dppb is adjusted if needed". Same line as the
# Cross (tncr1) except the end: the Altis end value -0.30 at 0 km/h (SOFT_END_ACCEL), -0.33 at 1 km/h. The delivery
# map below is the Cross's; Altis values come from its road test.
STOP_LEVEL_A = [-0.30, -0.33, -0.37, -0.45, -0.57, -0.55, -0.57, -0.69, -0.93]  # m/s^2, tncr1 sent (median), Altis end
STOP_LEVEL_V_MID = 10.0   # km/h: top of the level line
STOP_LEVEL_V_ARM = 12.0   # km/h: from here down to STOP_LEVEL_V_MID it blends from the planner's request to the line
STOP_LEVEL_GAP = 3.3      # m (radar dRel): the closest the eased stop may be predicted to end
STOP_LEVEL_LAG = 0.5      # s: Cross low-speed actuator lag, travelled before an eased command takes effect
STOP_LEVEL_LEAD_V = 1.0   # m/s: lead counted as stopped below this
STOP_LEVEL_RATE = 1.0     # m/s^3: easing never faster than this
STOP_LEVEL_MAX_B = 1.5    # m/s^2: a stop needing this much decel or more is left to the planner entirely
STOP_LEVEL_BLEND_M = 1.0  # m per m/s above STOP_LEVEL_V_MID: room kept for the 12 -> 10 km/h blend
# the Cross's delivered decel for a command (dpcr route d2, command 0.5 s earlier vs aEgo): 1-2 km/h |c| - 0.12,
# 2-5 km/h 1.23 |c| - 0.04, from ~6 km/h as asked, 9-16 km/h 0.08-0.18 less than asked
_DLV_GAIN_V, _DLV_GAIN = [1.5, 3.0, 5.5, 6.5], [1.0, 1.23, 1.23, 1.0]
_DLV_OFF_V, _DLV_OFF = [1.5, 3.0, 6.0], [0.12, 0.04, 0.0]
_DLV_HOLE_V, _DLV_HOLE = [5.0, 8.0, 10.0, 12.0, 14.5, 18.0, 25.0], [0.0, 0.08, 0.13, 0.15, 0.18, 0.10, 0.0]


def _delivered_decel(v_kmh, cmd):
  g = float(np.interp(v_kmh, _DLV_GAIN_V, _DLV_GAIN))
  return max(g * -cmd - float(np.interp(v_kmh, _DLV_OFF_V, _DLV_OFF)) - float(np.interp(v_kmh, _DLV_HOLE_V, _DLV_HOLE)), 0.05)


def _cmd_for_decel(v_kmh, b):
  off = float(np.interp(v_kmh, _DLV_OFF_V, _DLV_OFF)) + float(np.interp(v_kmh, _DLV_HOLE_V, _DLV_HOLE))
  return -(b + off) / float(np.interp(v_kmh, _DLV_GAIN_V, _DLV_GAIN))


# distance (m) the car needs to stop from a speed (km/h) following the level line, with the delivered decel
_LEVEL_D_V = np.linspace(0.0, STOP_LEVEL_V_MID, 81)
_LEVEL_D = np.concatenate(([0.0], np.cumsum([(_LEVEL_D_V[k] / 3.6) * ((_LEVEL_D_V[k] - _LEVEL_D_V[k - 1]) / 3.6) /
                                             _delivered_decel(_LEVEL_D_V[k], float(np.interp(_LEVEL_D_V[k], STOP_LEVEL_V, STOP_LEVEL_A)))
                                             for k in range(1, len(_LEVEL_D_V))])))


def _level_distance(v_kmh):
  return float(np.interp(v_kmh, _LEVEL_D_V, _LEVEL_D))


LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.soft_end_frac = 1.0
    self.engage_t = 0.0  # s since engaging
    self.onset_t: float | None = None  # s since the soft brake onset began (None: not in one)
    self.lead = None  # radarState.leadOne, set by controlsd every frame (None: no radarState)

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      # dragonpilot: firm toward stopAccel only once stopped (holding); never add brake before the stop
      if output_accel > self.CP.stopAccel and CS.standstill:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    output_accel = self.soft_stop_end(output_accel, a_target, CS)
    output_accel = self.stop_end_level(output_accel, a_target, CS)
    output_accel = self.engage_onset(output_accel, CS)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

  def soft_stop_end(self, output_accel, a_target, CS):
    # a new stop (or the lead moving off before we stopped) starts from scratch: no easing left over
    if CS.vEgo > SOFT_END_RESET_V or output_accel >= SOFT_END_ACCEL or \
       self.long_control_state not in (LongCtrlState.pid, LongCtrlState.stopping):
      self.soft_end_frac = 1.0
    if CS.standstill or output_accel >= SOFT_END_ACCEL or a_target < SOFT_END_URGENT or \
       self.long_control_state not in (LongCtrlState.pid, LongCtrlState.stopping):
      return output_accel
    t_stop = CS.vEgo / max(-CS.aEgo, SOFT_END_MIN_DECEL)
    ramp = max(SOFT_END_RAMP, (SOFT_END_ACCEL - output_accel) / SOFT_END_MAX_RATE)
    self.soft_end_frac = min(self.soft_end_frac, float(np.clip((t_stop - SOFT_END_LAG) / ramp, 0.0, 1.0)))
    soft = SOFT_END_ACCEL + (output_accel - SOFT_END_ACCEL) * self.soft_end_frac
    # the easing itself never releases faster than SOFT_END_MAX_RATE from the last command (the planner's own release
    # is left as it is: never firmer than the planner's output)
    return max(output_accel, min(soft, self.last_output_accel + SOFT_END_MAX_RATE * DT_CTRL))

  def stop_end_level(self, output_accel, a_target, CS):
    lead = self.lead
    v_kmh = CS.vEgo * 3.6
    if lead is None or not lead.status or lead.vLead >= STOP_LEVEL_LEAD_V or CS.standstill or \
       v_kmh > STOP_LEVEL_V_ARM or not (SOFT_END_URGENT < a_target < 0.0) or \
       self.long_control_state not in (LongCtrlState.pid, LongCtrlState.stopping):
      return output_accel
    if v_kmh > STOP_LEVEL_V_MID:
      line = float(np.interp(v_kmh, [STOP_LEVEL_V_MID, STOP_LEVEL_V_ARM], [STOP_LEVEL_A[-1], output_accel]))
    else:
      line = float(np.interp(v_kmh, STOP_LEVEL_V, STOP_LEVEL_A))
    if output_accel >= line:
      return output_accel
    room = lead.dRel - CS.vEgo * STOP_LEVEL_LAG - STOP_LEVEL_GAP
    need_d = _level_distance(min(v_kmh, STOP_LEVEL_V_MID)) + max(v_kmh - STOP_LEVEL_V_MID, 0.0) / 3.6 * STOP_LEVEL_BLEND_M
    if room >= need_d:
      target = line
    else:  # not enough room for the level line: ease only to the decel that still ends STOP_LEVEL_GAP out
      b = CS.vEgo ** 2 / (2.0 * max(room, 0.2))
      if b >= STOP_LEVEL_MAX_B:  # no room for a light stop at all: the planner's request, untouched
        return output_accel
      target = min(line, _cmd_for_decel(v_kmh, b))
    # never firmer than the planner; easing at most STOP_LEVEL_RATE from the last command
    return max(output_accel, min(target, self.last_output_accel + STOP_LEVEL_RATE * DT_CTRL))

  def engage_onset(self, output_accel, CS):
    if self.long_control_state == LongCtrlState.off:
      # off because the driver is on the gas (override) is not an engagement: releasing the gas gets no soft onset
      self.engage_t, self.onset_t = (ENGAGE_ONSET_WINDOW + 1.0 if getattr(CS, "gasPressed", False) else 0.0), None
      return output_accel
    self.engage_t += DT_CTRL
    if self.onset_t is None:
      # the onset starts with the first frame that asks for more brake than the last one, below zero, within the window
      if self.engage_t > ENGAGE_ONSET_WINDOW or CS.standstill or self.long_control_state != LongCtrlState.pid or \
         output_accel >= min(self.last_output_accel, 0.0):
        return output_accel
      self.onset_t = 0.0
    if self.onset_t >= ENGAGE_ONSET_T:
      return output_accel  # done: stock limits from here on (onset_t stays, so only once per engagement)
    jerk = ENGAGE_ONSET_J0 + (ENGAGE_ONSET_J1 - ENGAGE_ONSET_J0) * self.onset_t / ENGAGE_ONSET_T
    self.onset_t += DT_CTRL
    return max(output_accel, min(self.last_output_accel, 0.0) - jerk * DT_CTRL)
