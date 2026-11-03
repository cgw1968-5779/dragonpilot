import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# dragonpilot (dpcr, Corolla Cross Hybrid, owner 2026-11-03): soft end of a stop, same as dppb (Altis).
# The planner already eases toward the stop, but at low speed the car's decel lags the command, so a stop after a
# firmer middle (lead braked a little) still ends firm and the car lurches at the stop. Cross (C4 tncr1 routes
# c3-ce, the car's own response): best-fit lag 0.6 s below 10 km/h (rms 0.37 -> 0.22), 0.2 s at 10-25 km/h; the
# decel is ~0.85-0.9 x the command (no extra low-speed brake like the Altis). So when the time to the stop, from
# the actual speed and decel, drops below SOFT_END_LAG + SOFT_END_RAMP, the command eases to SOFT_END_ACCEL (owner's
# Cross end value at 0 km/h) and reaches it SOFT_END_LAG before the stop. Once easing starts it never firms again until the car
# has stopped (no stop-go-stop). Only ever lighter than the planner, only in the last ~1 s, so the start and the
# middle of every stop are unchanged.
SOFT_END_ACCEL = -0.34  # m/s^2, Cross
SOFT_END_LAG = 0.6      # s, Cross
SOFT_END_RAMP = 0.5     # s, shortest easing time
SOFT_END_MAX_RATE = 1.0  # m/s^3, easing never releases faster than this: a firmer brake starts easing earlier
SOFT_END_MIN_DECEL = 0.3  # m/s^2, floor of the decel used for the time to the stop
SOFT_END_RESET_V = 3.0  # m/s, a new stop can start easing again above this speed
SOFT_END_URGENT = -1.5  # m/s^2, never ease while the planner asks for more than this (urgent stop)

# dragonpilot (dpcr, owner 2026-11-03): "at mid / low speed, the first 0.1 s of the brake on the first SET / RESUME press
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
