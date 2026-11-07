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
# the speed + gap regime of f8 00:39 (owner 2026-11-08: "record that distance and speed - in that state no soft onset,
# emergency braking"): from REGIME_V with a time gap <= REGIME_HW a lead braking at REGIME_A or harder switches on at
# once, before the need above catches up (aLeadK lags the real decel). f8: 58 km/h, 28.2 m (1.75 s), aLeadK -1.75 at
# -2.75 s (need ON was -2.47 s). Scan of ~13 min following >= 40 km/h (11-03..11-08): only f8 00:39 and f0 23:17
# (53 km/h, 26.8 m, lead -2.1) switch on.
REGIME_V = 50.0 / 3.6   # m/s
REGIME_HW = 2.0         # s
REGIME_A = -1.75        # m/s^2
# owner 2026-11-08 (later): "back to the soft-brake principle - every brake starts soft (40-60 km/h keeps a safe gap,
# follow_gap.py); only an unusual cut-in that suddenly puts a lead very close skips the soft start and goes to a
# possible emergency brake". So an ordinary switch-on (need / regime above) starts with SOFT_T of soft build at
# SOFT_J, then goes to ACCEL_MIN ("0.1 s after the start, straight to the maximum"); a CUT-IN goes there at once.
SOFT_T = 0.1            # s
SOFT_J = 3.0            # m/s^3 (-0.3 in the first 0.1 s)
# cut-in: a lead that appears (newly tracked, or the lead jumps CUTIN_JUMP closer between radar frames) within
# CUTIN_HW x v at >= CUTIN_V; for CUTIN_T after it, need >= CUTIN_NEED switches on without the soft start
CUTIN_V = 20.0 / 3.6    # m/s
CUTIN_HW = 1.0          # s
CUTIN_JUMP = 5.0        # m
CUTIN_T = 2.0           # s
CUTIN_NEED = 1.5        # m/s^2
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
    self.t_on = 0.0         # time since switching on
    self.soft = True        # this switch-on starts soft
    self.prev_d = None      # last lead distance (cut-in detection)
    self.cutin_t = 0.0      # time left in the cut-in window

  def update(self, v_ego: float, lead, a_plan: float, off: bool) -> float:
    if off or lead is None or not lead.status:
      need = 0.0
      moving = False
    else:
      need = need_decel(v_ego, lead.dRel, lead.vLead, lead.aLeadK)
      moving = lead.vLead >= LEAD_MOVING_V
    # cut-in detection: a lead appearing / jumping much closer, already within CUTIN_HW
    d_now = lead.dRel if (not off and lead is not None and lead.status) else None
    appeared = d_now is not None and (self.prev_d is None or self.prev_d - d_now >= CUTIN_JUMP)
    if appeared and v_ego >= CUTIN_V and d_now <= CUTIN_HW * v_ego:
      self.cutin_t = CUTIN_T
    else:
      self.cutin_t = max(self.cutin_t - self.dt, 0.0)
    self.prev_d = d_now
    if off:
      self.active = self.handback = False
      return a_plan
    regime = (not off and lead is not None and lead.status and moving and v_ego >= REGIME_V and
              lead.dRel <= REGIME_HW * v_ego and lead.aLeadK <= REGIME_A)
    if not self.active:
      cutin = self.cutin_t > 0.0 and need >= CUTIN_NEED
      if cutin or regime or (v_ego >= V_ON and need >= (NEED_ON if moving else NEED_ON_STOPPED)):
        self.soft = not cutin
        if not self.handback:
          self.a = a_plan
        self.active, self.handback, self.t_on = True, False, 0.0
    elif need < NEED_OFF or v_ego < V_OFF:
      self.active, self.handback = False, True
    if self.active:
      self.t_on += self.dt
      if self.soft and self.t_on <= SOFT_T + 1e-9:
        self.a = min(a_plan, self.a - SOFT_J * self.dt)  # the soft start (never lighter than the plan)
      else:
        self.a = ACCEL_MIN
      return self.a
    if self.handback:
      self.a = min(a_plan, self.a + J_HANDBACK * self.dt)
      if self.a >= a_plan - 1e-6:
        self.handback = False
        return a_plan
      return self.a
    return a_plan
