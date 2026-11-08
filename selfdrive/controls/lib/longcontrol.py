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
# dppb (owner 2026-11-04 night, 21:54:16 behind a BMW at a red light: 3.9 m at 2 km/h, eased to -0.30 4.4 s before the
# stop; the Altis creep cancels a steady -0.30 at 1-3 km/h (steady cmd -0.30 -> aEgo -0.03, -0.36..-0.42 -> -0.32), so
# the car rolled 2.1 m on to 1.8 m; same at 13:21 / 13:23): "-0.30 only in the last 0.3-0.5 s". Behind a stopped radar
# lead, below SOFT_END_RESET_V, while the stop predicted from the actual speed and decel ends within SOFT_END_HOLD_GAP of
# the lead and is still more than SOFT_END_LAG away, the easing (soft end, stop end level, or the planner's own) stops at SOFT_END_HOLD_ACCEL (the lightest command the Altis
# still brakes on at 1-3 km/h) and a command already lighter is not released any further (held: never firmed). The last
# step to -0.30 is taken in the last SOFT_END_LAG.
SOFT_END_HOLD_ACCEL = -0.40  # m/s^2
SOFT_END_HOLD_MIN_DECEL = 0.05  # m/s^2: floor of the decel for the hold's time to the stop
SOFT_END_HOLD_GAP = 5.7  # (11-05 owner's 4.0 m standard stop: was 5.0) m (radar dRel): only while the stop, predicted from the actual decel, ends closer than this

# dragonpilot (dpcr, owner 2026-11-03; dppb port 2026-11-04): "at mid / low speed, the first 0.1 s of the brake on the
# first SET / RESUME press should be softer". Engaging with a lead (or a set speed) that needs braking, the planner's
# target is applied at once and carcontroller's 4 m/s^3 wind-down is the only limit: -0.4 m/s^2 after 0.1 s, felt as a
# bite. A brake that starts within ENGAGE_ONSET_WINDOW of engaging builds with a jerk rising from ENGAGE_ONSET_J0 to
# ENGAGE_ONSET_J1 over ENGAGE_ONSET_T (-0.15 at 0.1 s, -0.40 at 0.2 s; stock -0.4 / -0.8), then keeps climbing
# linearly at the stock ENGAGE_ONSET_J_STOCK until the planner's request is reached (-0.8 at 0.3 s, ~-1.6 at 0.5 s).
# dppb (Altis, owner 2026-11-04, after 11-04 13:24: engaged 44 m behind a stopped car at 45 km/h, stopped 2.1 m out):
# "keep the start soft, then climb linearly; if it cannot be higher at 0.5 s keep climbing after it, and hold the
# firmest brake a little longer, so the stop ends at 3-3.5 m". So after the onset, for the rest of this stop, behind a
# radar lead that has been stopped for ENGAGE_HOLD_LEAD_T: the decel the part of the stop above ENGAGE_HOLD_V_TAIL
# needs (lead distance less the lag travel, ENGAGE_HOLD_GAP and the distance the normal end from V_TAIL takes,
# ENGAGE_HOLD_D_TAIL - Altis 33 stops: median 7.4 m, p75 11.3) is compared with the planner's request; where the plan
# releases earlier the brake is held, at most ENGAGE_HOLD_MAX more and NEVER firmer than this stop's own firmest
# planner request so far (it holds the peak longer, it never deepens it). It builds at ENGAGE_HOLD_UP, lets go at
# ENGAGE_HOLD_DOWN, fades out between ENGAGE_HOLD_FADE_V km/h so the normal end (soft end, stop end level) is left as
# it is, and stops ENGAGE_HOLD_MAX_T after the onset. Owner-approved exception to "never firmer than the planner",
# only for this engage stop. Closed-loop sim of 13:24 (dppb x86 MPC, recorded car + 0.4 s lagged command difference):
# stock 2.53 m, soft onset only 2.39 m, with the hold 3.07 m; peak command -2.85 (stock -2.78), held to ~15 km/h.
ENGAGE_ONSET_WINDOW = 1.0   # s after engaging in which a brake onset gets the soft start
ENGAGE_ONSET_J0 = 1.0       # m/s^3 at the start of the brake
ENGAGE_ONSET_J1 = 3.0       # m/s^3 at ENGAGE_ONSET_T
ENGAGE_ONSET_T = 0.2        # s: the soft part
ENGAGE_ONSET_J_STOCK = 4.0  # m/s^3 after it (stock carcontroller wind-down limit), until the request is reached

# dppb (owner 2026-11-05, exception to "never lighter than the planner" approved): "when a car cuts in, start the brake
# soft but climb quickly to the planner's force; keep it simple". 21:53:33 at 56 km/h, accelerating +0.6, a car cut in
# 19 m ahead at 48 km/h and the brake felt too strong at the front. So a brake that starts after BRAKE_ONSET_COAST_T
# without braking (accelerating or coasting: every cut-in) gets the same curve as the engage onset: -0.15 at 0.1 s,
# -0.40 at 0.2 s, then the stock ENGAGE_ONSET_J_STOCK until the planner's request is reached. A planner that builds
# slower is untouched.
BRAKE_ONSET_FROM = -0.1      # m/s^2: "not braking" is a command at or above this
BRAKE_ONSET_COAST_T = 0.5    # s not braking before a new onset
# dppb (owner 2026-11-08, route 00000198 12:34:20 bookmark: a car cut in 28 m ahead at ~42 km/h, us 50 km/h; "too heavy -
# it should be the same: soft first, then firmer only as the situation needs"). The 0.2 s soft part was too short to be
# felt: the stock 4 m/s^3 climb after it SENT -0.93 at 0.4 s, the Altis did nothing for ~0.4 s and then bit, -0.38 ->
# -2.00 in 0.3 s while sending -1.22 (planner -1.39). dppb rlogs 11-04..11-08, 55 brake onsets at 25-75 km/h after
# >= 0.5 s without braking: sent at 0.5 s -0.4..0 -> the car braked a median 0.11 more than sent (delivered jerk 1.5
# m/s^3), -0.7..-0.4 -> 0.14 (2.2), firmer than -0.7 -> 0.56-0.67 (3.7-4.4): a fast climb is what makes the Altis bite.
# So after the soft part the climb follows what the lead actually needs, from radarState now (dynamic, no table):
#   need = max(-aLeadK, 0) + max(vEgo - vLead, 0)^2 / (2 * (dRel - ONSET_S0 - vEgo * ONSET_LAG))
# climb jerk = (need - current brake) / ONSET_T_CATCH, at least ONSET_J_SLOW, at most the stock ENGAGE_ONSET_J_STOCK:
# already braking as much as needed -> 1.0 m/s^3 on to the planner's request (it is still reached); braking less than
# needed -> catch up within ONSET_T_CATCH; no radar lead, or within ONSET_S0 + the lag travel -> the stock climb.
# Replay: 12:34:20 sent at 0.5 s -0.70 instead of -1.07, planner's -1.37 reached at 1.2 s instead of ~0.6 s; over the 11
# dppb drives only 11-04 21:53:34 (the 11-05 cut-in) changes (at most 0.42 lighter for 1.1 s, same peak); no normal stop
# changes (their planner builds slower than 1 m/s^3). Extends the owner's 11-05 cut-in exception (lighter than the
# planner only during the onset climb, never firmer).
ONSET_J_SLOW = 1.0           # m/s^3
ONSET_T_CATCH = 0.5          # s
ONSET_S0 = 4.0               # m: the owner's standard gap
ONSET_LAG = 0.5              # s of travel before a firmer command takes effect
ENGAGE_HOLD_MAX_T = 15.0    # s after the onset
ENGAGE_HOLD_LEAD_V = 1.0    # m/s: lead counted as stopped below this
ENGAGE_HOLD_LEAD_T = 0.5    # s the lead must have been stopped
ENGAGE_HOLD_GAP = 4.0       # m (11-05 owner's 4.0 m standard stop: was 3.3)
ENGAGE_HOLD_LAG = 0.5       # s
ENGAGE_HOLD_V_TAIL = 12.0   # km/h: below this the stop is the normal end's
ENGAGE_HOLD_D_TAIL = 9.0    # m the normal end takes from ENGAGE_HOLD_V_TAIL
ENGAGE_HOLD_FADE_V = [12.0, 20.0]  # km/h: no hold below the first, full above the second (11-05: was 10-15)
ENGAGE_HOLD_MAX = 0.6       # m/s^2 at most held beyond the planner
ENGAGE_HOLD_UP = 2.0        # m/s^3
ENGAGE_HOLD_DOWN = 0.5      # m/s^3 (11-05: was 1.0)

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
STOP_LEVEL_GAP = 4.0      # m (radar dRel): the closest the eased stop may be predicted to end (11-05: was 3.3)
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
    self.end_hold = False  # the end is held (not released) this frame
    self.end_zone = False  # this stop's end has been held
    self.engage_t = 0.0  # s since engaging
    self.onset_t: float | None = None  # s since the soft brake onset began (None: not in one)
    self.onset_done = False  # the onset ramp reached the request
    self.hold = 0.0  # <= 0: brake held beyond the planner in this engage stop
    self.hold_peak = 0.0  # this engage stop's firmest planner request so far
    self.hold_lead_t = 0.0  # s the lead has been stopped
    self.bo_t: float | None = None  # s since a brake onset (while driving) began
    self.bo_ramp = 0.0
    self.coast_t = 0.0  # s without braking
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
    output_accel = self.end_stop_hold(output_accel)
    output_accel = self.engage_onset(output_accel, CS)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

  def soft_stop_end(self, output_accel, a_target, CS):
    held = self.end_hold
    out = self._soft_stop_end(output_accel, a_target, CS, held)
    # the end of a stop behind a stopped lead: end_stop_hold (after stop_end_level) applies it. Only while the planner
    # brakes: acceleration / re-acceleration is never touched
    zone = not CS.standstill and CS.vEgo <= SOFT_END_RESET_V and a_target < 0.0 and \
      self.long_control_state in (LongCtrlState.pid, LongCtrlState.stopping) and self._stopped_lead()
    self.end_hold = zone and self._end_not_yet(CS)
    self.end_zone = zone and (self.end_zone or self.end_hold)  # this stop has been held (until it is over)
    return out

  def end_stop_hold(self, output_accel):
    # applied after both easings: a command firmer than the hold level is left as it is; a lighter one is eased toward
    # SOFT_END_HOLD_ACCEL at most SOFT_END_MAX_RATE (or kept at the last command if that is already lighter): never firmer
    # than the last command, never firmer than the easings' own result
    if self.end_hold and output_accel > SOFT_END_HOLD_ACCEL:
      return min(output_accel, max(min(self.last_output_accel + SOFT_END_MAX_RATE * DT_CTRL, SOFT_END_HOLD_ACCEL),
                                   self.last_output_accel))
    if self.end_zone and not self.end_hold and output_accel > SOFT_END_ACCEL:
      # the last SOFT_END_LAG: to -0.30, as smooth as the easing, not lighter
      return min(output_accel, max(self.last_output_accel + SOFT_END_MAX_RATE * DT_CTRL, SOFT_END_ACCEL))
    return output_accel

  def _soft_stop_end(self, output_accel, a_target, CS, held):  # the 11-03 soft end
    # a new stop (or the lead moving off before we stopped) starts from scratch: no easing left over
    if CS.vEgo > SOFT_END_RESET_V or (output_accel >= SOFT_END_ACCEL and not held) or \
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

  def _stopped_lead(self):
    lead = self.lead
    return lead is not None and lead.status and lead.vLead < STOP_LEVEL_LEAD_V

  def _end_not_yet(self, CS):
    b = max(-CS.aEgo, SOFT_END_HOLD_MIN_DECEL)
    if CS.vEgo / b <= SOFT_END_LAG:
      return False
    # only when this stop ends at this lead (predicted from the actual decel), not while still closing on a far one
    return self.lead.dRel - CS.vEgo * SOFT_END_LAG - CS.vEgo ** 2 / (2.0 * b) < SOFT_END_HOLD_GAP

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
      self.onset_done, self.hold, self.hold_peak, self.hold_lead_t = False, 0.0, 0.0, 0.0
      self.bo_t, self.coast_t = None, 0.0
      return output_accel
    self.engage_t += DT_CTRL
    if self.onset_t is None:
      # the onset starts with the first frame that asks for more brake than the last one, below zero, within the window
      if self.engage_t > ENGAGE_ONSET_WINDOW or CS.standstill or self.long_control_state != LongCtrlState.pid or \
         output_accel >= min(self.last_output_accel, 0.0):
        return self.brake_onset(output_accel, CS)
      self.onset_t = 0.0
    t = self.onset_t
    self.onset_t += DT_CTRL
    if not self.onset_done:
      jerk = ENGAGE_ONSET_J0 + (ENGAGE_ONSET_J1 - ENGAGE_ONSET_J0) * t / ENGAGE_ONSET_T if t < ENGAGE_ONSET_T else ENGAGE_ONSET_J_STOCK
      ramp = min(self.last_output_accel, 0.0) - jerk * DT_CTRL
      if output_accel < ramp:
        return ramp
      self.onset_done = True  # request reached: the hold below from here on
    return self.brake_onset(self.engage_hold(output_accel, CS), CS)

  def brake_onset(self, output_accel, CS):
    if self.bo_t is None:
      self.coast_t = 0.0 if self.last_output_accel < BRAKE_ONSET_FROM else self.coast_t + DT_CTRL
      if self.coast_t < BRAKE_ONSET_COAST_T or output_accel >= BRAKE_ONSET_FROM or CS.standstill or \
         self.long_control_state != LongCtrlState.pid:
        return output_accel
      self.bo_t, self.bo_ramp = 0.0, min(self.last_output_accel, 0.0)
    t = self.bo_t
    self.bo_t += DT_CTRL
    if t < ENGAGE_ONSET_T:
      jerk = ENGAGE_ONSET_J0 + (ENGAGE_ONSET_J1 - ENGAGE_ONSET_J0) * t / ENGAGE_ONSET_T
    else:
      jerk = float(np.clip((self._onset_need(CS) + min(self.bo_ramp, 0.0)) / ONSET_T_CATCH, ONSET_J_SLOW, ENGAGE_ONSET_J_STOCK))
    self.bo_ramp -= jerk * DT_CTRL
    if output_accel < self.bo_ramp:
      return self.bo_ramp
    self.bo_t, self.coast_t = None, 0.0  # the planner's request is reached
    return output_accel

  def _onset_need(self, CS) -> float:
    """decel (m/s^2, >= 0) the radar lead needs now; inf = unknown / too close (stock climb)"""
    lead = self.lead
    if lead is None or not lead.status:
      return float('inf')
    room = lead.dRel - ONSET_S0 - CS.vEgo * ONSET_LAG
    if room <= 0.5:
      return float('inf')
    return max(-float(getattr(lead, 'aLeadK', 0.0)), 0.0) + max(CS.vEgo - lead.vLead, 0.0) ** 2 / (2.0 * room)

  def engage_hold(self, output_accel, CS):
    lead = self.lead
    still = lead is not None and lead.status and lead.vLead < ENGAGE_HOLD_LEAD_V
    self.hold_lead_t = self.hold_lead_t + DT_CTRL if still else 0.0
    want = 0.0
    if self.onset_t < ENGAGE_HOLD_MAX_T and not CS.standstill and self.hold_lead_t >= ENGAGE_HOLD_LEAD_T and output_accel < 0.0:
      vt = ENGAGE_HOLD_V_TAIL / 3.6
      room = lead.dRel - CS.vEgo * ENGAGE_HOLD_LAG - ENGAGE_HOLD_GAP - ENGAGE_HOLD_D_TAIL
      b_need = max(CS.vEgo ** 2 - vt ** 2, 0.0) / (2.0 * max(room, 0.5))
      cap = ENGAGE_HOLD_MAX * float(np.interp(CS.vEgo * 3.6, ENGAGE_HOLD_FADE_V, [0.0, 1.0]))
      want = float(np.clip(-b_need - output_accel, -cap, 0.0))
    if want < self.hold:
      self.hold = max(want, self.hold - ENGAGE_HOLD_UP * DT_CTRL)
    else:
      self.hold = min(want, self.hold + ENGAGE_HOLD_DOWN * DT_CTRL)
    # hold, never deepen: never firmer than this stop's own firmest planner request so far
    self.hold_peak = min(self.hold_peak, output_accel)
    return max(output_accel + self.hold, min(self.hold_peak, output_accel))
