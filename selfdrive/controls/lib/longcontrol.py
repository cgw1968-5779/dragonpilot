# dragonpilot (dpcr, owner 2026-11-06: "copy tncr1's mid/low-speed longitudinal control exactly"): this file is tncr1's
# LongControl (sunnypilot newer-openpilot version: no starting state, kp 0, stopping winds toward stopAccel at
# 1.0 m/s^3) with only the cereal import and the optional CP_SP adapted. dp's own layers that lived here (soft stop end,
# stop end level, 1 -> 0 km/h hold, engage brake onset + hold, cut-in onset) are removed (the engage gas onset came
# back 11-06 as dragonpilot/.../engage_gas_onset.py, one hook below); tncr1's stopping controller and send shaping (dragonpilot/selfdrive/controls/lib/tncr1_long) and its Toyota send side
# (opendbc toyota carcontroller hooks) replace them.
import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from dragonpilot.selfdrive.controls.lib.engage_gas_onset import EngageGasOnset

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP_SP, active, long_control_state,
                             should_stop, brake_pressed, cruise_standstill):
  # Gas Interceptor (dp: no CarParamsSP, never a gas interceptor on the Cross)
  cruise_standstill = cruise_standstill and not getattr(CP_SP, 'enableGasInterceptor', False)

  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if should_stop:
        long_control_state = LongCtrlState.stopping

  return long_control_state

class LongControl:
  def __init__(self, CP, CP_SP=None):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.engage_gas_onset = EngageGasOnset()  # dragonpilot hook

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP_SP, active, self.long_control_state,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # TODO: can we just go straight to stopAccel?
        output_accel -= 1.0 * DT_CTRL  # m/s^2/s while trying to stop
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    output_accel = self.engage_gas_onset.update(output_accel, CS, self.long_control_state == LongCtrlState.off,
                                                self.long_control_state == LongCtrlState.pid)  # dragonpilot hook
    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
