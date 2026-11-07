"""
dragonpilot (dpcr, Corolla Cross): full brake at once when a fast, close lead brakes hard (owner 2026-11-08: "00:39 the
brake was not enough and I had to take over. When the speed is fairly high and the lead is very close, the brake must
go straight to the maximum 0.1 s after it starts - even skipping the soft brake onset. Switch on the present speed and
following distance.").

Route 000000f8 00:39:05 (rlog): following at 58 km/h, 29.5 m (1.8 s), the lead braked about -4.5 m/s^2 from 58 km/h to
a stop. The plan took 1.25 s to build -0.1 -> -3.5 (the MPC's own rate), and on the send side the Toyota PID saw the
car already at -3.3 and backed the request off to -2.6..-3.1 - it never sent -3.5. The driver braked at 14 m / 32 km/h.

The switch is the decel needed to stop GAP behind where the lead will stop, from the present speed and distance:
  need = v^2 / (2 (dRel + lead's stopping distance - GAP - v T_REACT)),  lead's stopping distance = vLead^2 / (2 |aLead|)
  (a lead that is not braking: the closing speed against the gap)
ON at v >= V_ON when need >= NEED_ON (a moving lead) or NEED_ON_STOPPED (a stopped lead - the stop target handles the
rest); OFF when need < NEED_OFF or v < V_OFF. While ON the plan is ACCEL_MIN (-3.5) at once (the send side then skips its
soft onset and PID back-off, toyota carcontroller hook); on OFF the plan comes back at J_HANDBACK. f8 00:39 crosses
NEED_ON 2.5 at 2.4 s before the driver braked (the plan reached -3.5 0.5 s later, the send side never did). Over ~9 min
of engaged lead following >= 30 km/h on routes d6-f8 nothing else switches on.
"""
from opendbc.car.interfaces import ACCEL_MIN

GAP = 3.2               # m
T_REACT = 0.5           # s
V_ON = 40.0 / 3.6       # m/s
V_OFF = 20.0 / 3.6      # m/s
NEED_ON = 2.5           # m/s^2, a moving lead
NEED_ON_STOPPED = 3.0   # m/s^2, a stopped lead
NEED_OFF = 1.5          # m/s^2
LEAD_MOVING_V = 3.0     # m/s
J_HANDBACK = 2.0        # m/s^3


def need_decel(v_ego: float, d_rel: float, v_lead: float, a_lead: float) -> float:
  if a_lead < -0.5:
    d_lead = v_lead * v_lead / (2.0 * -a_lead)
    return v_ego * v_ego / (2.0 * max(d_rel + d_lead - GAP - v_ego * T_REACT, 0.5))
  if v_lead < v_ego:
    dv = v_ego - v_lead
    return dv * dv / (2.0 * max(d_rel - GAP - dv * T_REACT, 0.5))
  return 0.0


class UrgentBrake:
  def __init__(self, dt: float):
    self.dt = dt
    self.active = False
    self.handback = False
    self.a = 0.0

  def update(self, v_ego: float, lead, a_plan: float, off: bool) -> float:
    if off or lead is None or not lead.status:
      need = 0.0
      moving = False
    else:
      need = need_decel(v_ego, lead.dRel, lead.vLead, lead.aLeadK)
      moving = lead.vLead >= LEAD_MOVING_V
    if off:
      self.active = self.handback = False
      return a_plan
    if not self.active:
      if v_ego >= V_ON and need >= (NEED_ON if moving else NEED_ON_STOPPED):
        self.active, self.handback = True, False
    elif need < NEED_OFF or v_ego < V_OFF:
      self.active, self.handback = False, True
    if self.active:
      self.a = ACCEL_MIN
      return self.a
    if self.handback:
      self.a = min(a_plan, self.a + J_HANDBACK * self.dt)
      if self.a >= a_plan - 1e-6:
        self.handback = False
        return a_plan
      return self.a
    return a_plan
