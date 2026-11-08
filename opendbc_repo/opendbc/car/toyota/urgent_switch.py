"""
dragonpilot (dpcr, Corolla Cross): when the urgent brake is on - shared by the planner (dragonpilot urgent_brake.py,
which shapes the brake) and the Toyota send side (carcontroller, which then sends that brake as is: no soft onset, no
PID back-off), so both decide from the same radar lead with the same rules.

The switch is the decel needed to stop GAP behind where the lead will stop, from the present speed and distance:
  need = v^2 / (2 (dRel + lead's stopping distance - GAP - v T_REACT)),  lead's stopping distance = vLead^2 / (2 |aLead|)
  (a lead that is not braking: the closing speed against the gap)
ON at v >= V_ON when need >= NEED_ON (a moving lead) or NEED_ON_STOPPED (a stopped lead); in the f8 00:39 speed + gap
regime (>= REGIME_V, time gap <= REGIME_HW, lead aLeadK <= REGIME_A) at once; within CUTIN_T of a cut-in (a lead
appearing / jumping CUTIN_JUMP closer within CUTIN_HW x v at >= CUTIN_V) when need >= CUTIN_NEED (then without the
soft start). OFF when need < NEED_OFF or v < V_OFF. History and numbers: see urgent_brake.py.
"""
GAP = 3.2               # m
T_REACT = 0.5           # s
V_ON = 40.0 / 3.6       # m/s
V_OFF = 20.0 / 3.6      # m/s
NEED_ON = 2.5           # m/s^2, a moving lead
NEED_ON_STOPPED = 3.0   # m/s^2, a stopped lead
NEED_OFF = 1.5          # m/s^2
LEAD_MOVING_V = 3.0     # m/s
REGIME_V = 50.0 / 3.6   # m/s
REGIME_HW = 2.0         # s
REGIME_A = -1.75        # m/s^2
CUTIN_V = 20.0 / 3.6    # m/s
CUTIN_HW = 1.0          # s
CUTIN_JUMP = 5.0        # m
CUTIN_T = 2.0           # s
CUTIN_NEED = 1.5        # m/s^2


def need_decel(v_ego: float, d_rel: float, v_lead: float, a_lead: float) -> float:
  if a_lead < -0.5:
    d_lead = v_lead * v_lead / (2.0 * -a_lead)
    return v_ego * v_ego / (2.0 * max(d_rel + d_lead - GAP - v_ego * T_REACT, 0.5))
  if v_lead < v_ego:
    dv = v_ego - v_lead
    return dv * dv / (2.0 * max(d_rel - GAP - dv * T_REACT, 0.5))
  return 0.0


class UrgentSwitch:
  def __init__(self, dt: float):
    self.dt = dt
    self.active = False
    self.cutin = False      # this switch-on came from a cut-in (no soft start)
    self.need = 0.0
    self.prev_d = None      # last lead distance (cut-in detection)
    self.cutin_t = 0.0      # time left in the cut-in window

  def update(self, v_ego: float, lead, off: bool) -> bool:
    """returns True on the frame it switches on"""
    ok = not off and lead is not None and bool(lead.status)
    self.need = need_decel(v_ego, lead.dRel, lead.vLead, lead.aLeadK) if ok else 0.0
    moving = ok and lead.vLead >= LEAD_MOVING_V
    d_now = lead.dRel if ok else None
    appeared = d_now is not None and (self.prev_d is None or self.prev_d - d_now >= CUTIN_JUMP)
    if appeared and v_ego >= CUTIN_V and d_now <= CUTIN_HW * v_ego:
      self.cutin_t = CUTIN_T
    else:
      self.cutin_t = max(self.cutin_t - self.dt, 0.0)
    self.prev_d = d_now
    if off:
      self.active = False
      return False
    if not self.active:
      cutin = self.cutin_t > 0.0 and self.need >= CUTIN_NEED
      regime = moving and v_ego >= REGIME_V and lead.dRel <= REGIME_HW * v_ego and lead.aLeadK <= REGIME_A
      if cutin or regime or (v_ego >= V_ON and self.need >= (NEED_ON if moving else NEED_ON_STOPPED)):
        self.active, self.cutin = True, cutin
        return True
    elif self.need < NEED_OFF or v_ego < V_OFF:
      self.active = False
    return False
