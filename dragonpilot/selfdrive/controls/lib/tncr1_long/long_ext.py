"""
dragonpilot (dpcr): tncr1's controlsd_ext longitudinal hooks (long_target_sp / long_accel_sp), line for line.

tncr1 (sunnypilot/selfdrive/controls/controlsd_ext.py, e1656efd1a):
- before LongControl.update(): the steady-cruise smoother on aTarget, and LongControl's state from before the update;
- after it: the stopping controller (end-of-stop policy, standstill hold), then the send-only shaping layers
  (stop-gap hold, stop-gap close, creep-stall guard, lead brake boost, mid-speed brake add) and the owner's end-force
  ceiling last.
The only dp difference: dp's radarState LeadData has `status` where tncr1's newer schema has `present`; LeadView carries
both names (tncr1 itself copies src.present into CC_SP.leadOne.status for the car side).
"""
from dragonpilot.selfdrive.controls.lib.tncr1_long.creep_stall_guard import CreepStallGuard
from dragonpilot.selfdrive.controls.lib.tncr1_long.cruise_smoother import CruiseSmoother
from dragonpilot.selfdrive.controls.lib.tncr1_long.end_force_limit import EndForceLimit
from dragonpilot.selfdrive.controls.lib.tncr1_long.lead_brake_boost import LeadBrakeBoost
from dragonpilot.selfdrive.controls.lib.tncr1_long.mid_speed_brake_add import MidSpeedBrakeAdd
from dragonpilot.selfdrive.controls.lib.tncr1_long.stop_gap_close import StopGapClose
from dragonpilot.selfdrive.controls.lib.tncr1_long.stop_gap_hold import StopGapHold
from dragonpilot.selfdrive.controls.lib.tncr1_long.stopping_controller import StoppingController


class LeadView:
  """a dp radarState LeadData under tncr1's field names (present) and the car side's (status)"""
  __slots__ = ('present', 'status', 'dRel', 'yRel', 'vRel', 'aRel', 'vLead', 'vLeadK', 'aLeadK', 'aLeadTau', 'modelProb',
               'radar', 'radarTrackId', 'dPath', 'vLat', 'fcw')

  def __init__(self, src=None):
    self.present = self.status = bool(getattr(src, 'status', False)) if src is not None else False
    for k in ('dRel', 'yRel', 'vRel', 'aRel', 'vLead', 'vLeadK', 'aLeadK', 'dPath', 'vLat', 'modelProb'):
      setattr(self, k, float(getattr(src, k, 0.0)) if src is not None else 0.0)
    self.aLeadTau = float(getattr(src, 'aLeadTau', 1.5)) if src is not None else 1.5
    self.radar = bool(getattr(src, 'radar', False)) if src is not None else False
    self.radarTrackId = int(getattr(src, 'radarTrackId', -1)) if src is not None else -1
    self.fcw = bool(getattr(src, 'fcw', False)) if src is not None else False


class Tncr1Long:
  def __init__(self, CP):
    self.lead_brake_boost = LeadBrakeBoost()  # tnpb2: 3-30 km/h x1.2 after a hard lead brake, see lead_brake_boost.py
    self.mid_speed_brake_add = MidSpeedBrakeAdd()  # tncr: +0.2 m/s^2 braking 30-10 km/h, see mid_speed_brake_add.py
    self.stop_gap_hold = StopGapHold()  # tnpb3: <30 km/h no lighter while the stop gap < 3.5 m, see stop_gap_hold.py
    self.creep_stall_guard = CreepStallGuard()  # tnpb3: 1-5 km/h adds back brake the hybrid creep eats
    self.stop_gap_close = StopGapClose()  # tnpb3: <12 km/h a stop predicted beyond 4 m crawls on at 2.3 km/h to 3.25 m
    self.end_force_limit = EndForceLimit()  # tnpb3: owner rule, <2 km/h linear to the end value at 0 km/h, never firmer
    self.stopping_controller = StoppingController(CP.stopAccel)  # end of stop / standstill hold, see stopping_controller.py
    self.cruise_smoother = CruiseSmoother()  # steady-cruise ripple, see cruise_smoother.py
    self._stopping_prev = None
    self._a_target_sp = 0.0

  def long_target(self, LoC, long_plan, CS, long_active: bool, lead_one: LeadView) -> float:
    """tncr1 long_target_sp: the aTarget LongControl should track, and LongControl's state from before the update"""
    self._stopping_prev = (LoC.long_control_state, LoC.last_output_accel)
    self._a_target_sp = self.cruise_smoother.update(long_plan.aTarget, CS.vEgo, lead_one.present,
                                                    long_active, long_plan.shouldStop)
    return self._a_target_sp

  def long_accel(self, LoC, CS, long_plan, accel_limits, long_active: bool, lead_one: LeadView, calibrated_pose) -> float:
    """tncr1 long_accel_sp: returns actuators.accel"""
    prev_state, prev_accel = self._stopping_prev
    stock_state = LoC.long_control_state
    LoC.long_control_state, accel = self.stopping_controller.update(
      prev_state, stock_state, CS, self._a_target_sp, prev_accel, LoC.last_output_accel, accel_limits, long_plan.hasLead,
      pitch=calibrated_pose.orientation.pitch if calibrated_pose is not None else None,
      a_long=calibrated_pose.acceleration.x if calibrated_pose is not None else None,
      lead_d=lead_one.dRel if lead_one.present else None,
      lead_v=lead_one.vLead if lead_one.present else None)
    if LoC.long_control_state != stock_state:
      LoC.reset()
    LoC.last_output_accel = accel
    # the stop-gap hold, creep-stall guard and boost only shape what is sent; last_output_accel (the next frame's
    # prev_accel for the stopping controller and LongControl's stopping state) keeps the plain request
    accel_shaped = self.stop_gap_hold.update(accel, CS.vEgo, CS.aEgo, CS.standstill, lead_one, long_active)
    accel_shaped = self.stop_gap_close.update(accel_shaped, CS.vEgo, CS.aEgo, CS.standstill, lead_one, long_active)
    # the gap-close crawl rolls on purpose: not a creep stall
    accel_shaped = self.creep_stall_guard.update(accel_shaped, CS.vEgo, CS.aEgo, CS.standstill,
                                                 long_active and not self.stop_gap_close.active, accel_limits[0])
    accel_shaped = self.lead_brake_boost.update(accel_shaped, CS.vEgo, CS.standstill, lead_one, long_active,
                                                accel_limits[0], a_ego=CS.aEgo,
                                                hold_off=self.stop_gap_hold.roll_out is not None)
    # tncr: +0.2 m/s^2 on braking 30-10 km/h (gone by 6 km/h, before the end ceiling below)
    accel_shaped = self.mid_speed_brake_add.update(accel_shaped, CS.vEgo, CS.standstill, long_active, accel_limits[0],
                                                   hold_off=self.stop_gap_hold.roll_out is not None or
                                                   self.stop_gap_close.active)
    # owner's end-force rule, last: never firmer than the end ceiling; the standstill hold then firms up at HOLD_RATE
    return float(self.end_force_limit.update(accel_shaped, CS.vEgo, CS.standstill, long_active, CS.aEgo,
                                             pitch=calibrated_pose.orientation.pitch if calibrated_pose is not None else None,
                                             lead_d=lead_one.dRel if lead_one.present else None,
                                             lead_v=lead_one.vLead if lead_one.present else None))
