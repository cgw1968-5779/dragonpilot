"""
dragonpilot (dpcr, Corolla Cross Hybrid, owner 2026-11-05): "dpcr's mid / low-speed braking is still a little late and
abrupt - analyse the existing data and make it smooth like tncr1". Port of the MPC-side parts of sunnypilot tncr1's
long_mpc_ext.py (origin/tncr1 77cd57017b and earlier) that shape a brake behind a lead. Both act INSIDE the MPC (its
obstacle distance and its per-node brake limit), so the MPC plans the shape itself - no layer after the planner.

1. Light-onset margin (tnpb2 10-03..10-05, tncr): LIGHT_ONSET_T_V seconds of travel at the planned speed are taken off
   the room in front of the car, and the same term is added to each lead's stopped-equivalence obstacle by that lead's
   speed. Behind a lead at equal speed the two cancel (the following gap is unchanged); behind a slower / stopped lead
   the brake starts that much earlier and lighter. At the stop the planned speed is 0, so the stop point is unchanged.
2. Dynamic close-follow gap (tncr 10-15): only while the lead brakes harder than DYN_A0, scaled to full at DYN_A1, by the
   planned speed (0 below 20 km/h): more room to brake softly when the lead suddenly brakes; gone when the lead nearly
   stops, so the stop point is unchanged.
3. Emergency stop-distance trim (tncr 10-15 / 10-23 / 10-27): when the lead brakes hard and the plan is already
   fierce, the stop may end closer (down to ~2 m) instead of braking even harder.
4. Soft onset cap with a firm-early mid (tncr 10-15 / 10-27 / 10-28): the plan's brake limit (the MPC's a_min parameter
   per node) starts at CAP_A0 and grows only while the plan presses against it, its rate itself building up (no jerk
   step), but never below CAP_NEED_MARGIN x the constant decel that stops CAP_STOP_D short of where the lead itself will
   stop, leaving out the distance covered while the brake responds (CAP_REACT_T). So the start is soft and the middle
   reaches the force the stop really needs early (owner's 10-28 09:19 three-bookmark template: up to -2.0 in 1-2 s,
   steady mid, smooth end). Lead stops only, full 25-55 km/h, none below 15 (the end belongs to the stop end rules) or
   above 70 km/h.
5. Aggressive close follow +1 m at 30-60 km/h (tncr 10-27, the Cross runs aggressive), per node at the planned speed.

Not ported: the static distance trim table (-1 m at 5 km/h would move every stop 1 m closer; dpcr's stop end is tuned
to 3-3.5 m without it), the highway relaxed / standard gaps and the night gap (highway, not this request), the relaxed
jerk factor (the Cross runs aggressive).
6. Later brake behind a mid-speed lead that slows (dpcr, owner 2026-11-06, route 000000ec 21:59:07, bookmark: following a
   lead at ~32 km/h that slowed for a red light, the brake started at 19.8 m - "can it start about 1 m closer?"). The
   start is set by the lead's predicted decel (aLeadK), not by the gap: the light-onset margin and the aggressive +1 m
   barely moved it in a closed-loop replay. The lead's decel is trusted LEAD_TRUST_W (x0.5) at 20-40 km/h of ego speed,
   fading back to full by 15 and 50 km/h, and fully again while the lead brakes hard (LEAD_TRUST_A0 -> A1). Closed-loop
   replay of 21:59:07 (aggressive): plan at -0.3 / -0.6 / -1.0 reached at 20.3 / 19.8 / 17.9 m -> 19.7 / 19.2 / 15.8 m,
   peak -1.52 -> -1.89, closest gap 14.1 -> 12.3 m (owner chose "1 m later" knowing the firmer, closer follow).

Hooks: three lines in LongitudinalMpc (process_lead: the lead decel; update: the x_obstacles stack and params[:, 0]).
"""
import numpy as np
from cereal import log

LIGHT_ONSET_BP = [35.0 / 3.6, 40.0 / 3.6, 55.0 / 3.6, 80.0 / 3.6]  # m/s
LIGHT_ONSET_T_V = [0.8, 1.1, 1.1, 0.0]                              # s of travel reserved for the light first press

DYN_GAP_BP = [20.0 / 3.6, 30.0 / 3.6, 40.0 / 3.6, 50.0 / 3.6, 70.0 / 3.6]  # m/s planned ego speed per node
DYN_GAP_V = [0.0, 2.0, 5.0, 5.0, 0.0]  # m added at full urgency
DYN_A0 = -0.8        # m/s^2 lead decel: no add while the lead brakes lighter than this
DYN_A1 = -2.0        # m/s^2: full add from here
DYN_RATE_ON = 4.0    # 1/s
DYN_RATE_OFF = 0.67  # 1/s
DYN_V_LEAD_OFF = 10.0 / 3.6  # m/s: the lead is nearly stopped -> no add
DT = 0.05            # s: the planner runs at 20 Hz

EMERG_A = -1.5           # m/s^2: the lead brakes at least this hard
EMERG_PLAN_A0 = -2.0     # m/s^2: no trim while the plan is lighter than this
EMERG_PLAN_A1 = -2.5     # m/s^2: full trim from here
EMERG_TRIM = 2.0         # m taken off the desired distance at full trim
EMERG_LEAD_A0 = -2.2     # m/s^2
EMERG_LEAD_A1 = -3.0     # m/s^2
EMERG_TRIM_HARD = 1.0    # m
EMERG_RATE_ON = 4.0      # m/s
EMERG_RATE_OFF = 1.0     # m/s
EMERG_CALM_A = -0.5      # m/s^2
EMERG_CALM_T = 2.0       # s

CAP_V_BP = [15.0 / 3.6, 25.0 / 3.6, 55.0 / 3.6, 70.0 / 3.6]  # m/s
CAP_V_W = [0.0, 1.0, 1.0, 0.0]
CAP_A0 = 0.6            # m/s^2: the brake the plan may ask at the start of a brake event
CAP_RATE = 0.4          # m/s^3: growth with a calm lead
CAP_RATE_URGENT = 2.0   # m/s^3: at full lead urgency
CAP_NEED_MARGIN = 1.0   # never below this times the constant decel needed to stop CAP_STOP_D short
CAP_STOP_D = 3.5        # m
CAP_STOP_D_URGENT = 2.0  # m when the lead brakes hard
CAP_BRAKING_A = -0.2    # m/s^2: the plan counts as braking below this
CAP_RELEASE_T = 1.0     # s without braking -> the brake event ends
CAP_HEAD = 0.2          # m/s^2: the cap sits at most this far beyond the brake the plan uses
CAP_RATE_START = 0.3    # m/s^3
CAP_RATE_RAMP = 0.7     # m/s^4
CAP_BIND = 0.1          # m/s^2
CAP_LEAD_A_MIN = 0.5    # m/s^2: the lead's own stop point uses at least this decel
CAP_REACT_T = 0.5       # s: distance covered while the brake responds is left out of the need
ACCEL_MIN_MPC = -3.5    # m/s^2: stock limit when no cap applies

AGG_GAP_BP = [20.0 / 3.6, 30.0 / 3.6, 60.0 / 3.6, 70.0 / 3.6]  # m/s
AGG_GAP_V = [0.0, 1.0, 1.0, 0.0]                              # m added (aggressive)

LEAD_TRUST_V_BP = [15.0 / 3.6, 20.0 / 3.6, 40.0 / 3.6, 50.0 / 3.6]  # m/s ego speed
LEAD_TRUST_W = [1.0, 0.5, 0.5, 1.0]                                 # share of the lead's decel the MPC predicts with
LEAD_TRUST_A0 = -2.0     # m/s^2 lead decel: the discount fades out from here
LEAD_TRUST_A1 = -3.0     # m/s^2: full trust from here (a hard brake is never discounted)


def lead_decel_trust(a_lead: float, v_ego: float) -> float:
  """the lead's decel as the MPC predicts with it (6.): x LEAD_TRUST_W at 20-40 km/h, full for a hard brake"""
  if a_lead >= 0.0:
    return a_lead
  w = float(np.interp(v_ego, LEAD_TRUST_V_BP, LEAD_TRUST_W))
  hard = float(np.interp(a_lead, [LEAD_TRUST_A1, LEAD_TRUST_A0], [1.0, 0.0]))
  return a_lead * (w + (1.0 - w) * hard)


def get_light_onset_margin(v):
  v = np.maximum(v, 0.0)
  return v * np.interp(v, LIGHT_ONSET_BP, LIGHT_ONSET_T_V)


class LongMpcExt:
  def __init__(self):
    self.emerg = False
    self.emerg_trim = 0.0
    self.emerg_plan = 0.0
    self.emerg_lead = 0.0
    self.calm_t = 0.0
    self.dyn = 0.0
    self.cap_t_on = 0.0
    self.cap_t_off = 0.0
    self.cap_level = None
    self.cap_rate = CAP_RATE_START
    self.a_min = None

  def update_onset_cap(self, x_obstacle_0: float, v_plan, t1: float, t_idxs, closing: float, dyn: float):
    v_ego = float(v_plan[0])
    a0 = (float(v_plan[1]) - v_ego) / t1
    if a0 < CAP_BRAKING_A:
      self.cap_t_on += DT
      self.cap_t_off = 0.0
    else:
      self.cap_t_off += DT
      if self.cap_t_off >= CAP_RELEASE_T:
        self.cap_t_on = 0.0
    w = float(np.interp(v_ego, CAP_V_BP, CAP_V_W))
    if w <= 0.0 or closing < -1.0 or self.cap_t_on <= 0.0:
      self.a_min, self.cap_level, self.cap_rate = None, None, CAP_RATE_START
      return None
    used = max(-a0, 0.0)
    rate_target = CAP_RATE + (CAP_RATE_URGENT - CAP_RATE) * dyn
    if self.cap_level is None:
      self.cap_level, self.cap_rate = CAP_A0, min(CAP_RATE_START, rate_target)
    elif used >= self.cap_level - CAP_BIND:
      self.cap_rate = min(self.cap_rate + CAP_RATE_RAMP * DT, max(rate_target, CAP_RATE_START))
      self.cap_level += self.cap_rate * DT
    else:
      self.cap_rate = min(CAP_RATE_START, rate_target)
    self.cap_level = min(self.cap_level, max(CAP_A0, used + CAP_HEAD))
    t = np.asarray(t_idxs, dtype=float)
    r_top = max(rate_target, self.cap_rate)
    t_ramp = (r_top - self.cap_rate) / CAP_RATE_RAMP
    tr = np.minimum(t, t_ramp)
    cap = self.cap_level + self.cap_rate * tr + 0.5 * CAP_RATE_RAMP * tr ** 2 + r_top * np.maximum(t - t_ramp, 0.0)
    stop_d = CAP_STOP_D + (CAP_STOP_D_URGENT - CAP_STOP_D) * dyn
    need = v_ego ** 2 / (2.0 * max(x_obstacle_0 - stop_d - v_ego * CAP_REACT_T, 0.5))
    cap = np.minimum(np.maximum(cap, CAP_NEED_MARGIN * need), -ACCEL_MIN_MPC)
    self.a_min = -(w * cap + (1.0 - w) * -ACCEL_MIN_MPC)
    return self.a_min

  def min_accel(self, accel_min: float):
    """params[:, 0]: the onset cap when one applies, else the stock limit"""
    return self.a_min if self.a_min is not None else accel_min

  def update_dynamic_gap(self, lead_xv_0, v_plan, t1: float) -> float:
    v_ego = float(v_plan[0])
    v_lead = float(lead_xv_0[0, 1])
    a_lead = (float(lead_xv_0[1, 1]) - v_lead) / t1
    closing = v_ego - v_lead
    if v_ego < 0.3 or closing < -1.0 or v_lead < DYN_V_LEAD_OFF:
      target = 0.0
    else:
      target = float(np.interp(a_lead, [DYN_A1, DYN_A0], [1.0, 0.0]))
    self.dyn = float(np.clip(target, self.dyn - DYN_RATE_OFF * DT, self.dyn + DYN_RATE_ON * DT))
    return self.dyn

  def update_emergency(self, lead_xv_0, v_plan, t1: float, t_idxs) -> float:
    v_ego = float(v_plan[0])
    v_lead = float(lead_xv_0[0, 1])
    a_lead = (float(lead_xv_0[1, 1]) - v_lead) / t1
    closing = v_ego - v_lead
    a_plan = float(np.min(np.diff(v_plan) / np.diff(t_idxs)))
    self.calm_t = self.calm_t + DT if (a_lead > EMERG_CALM_A and closing < 1.0) else 0.0
    if v_ego < 0.3 or closing < -1.0 or self.calm_t >= EMERG_CALM_T:
      self.emerg, self.emerg_plan, self.emerg_lead = False, 0.0, 0.0
    elif a_lead <= EMERG_A and a_plan <= EMERG_PLAN_A0:
      self.emerg = True
    if self.emerg:
      self.emerg_plan = min(self.emerg_plan, a_plan)
      self.emerg_lead = min(self.emerg_lead, a_lead)
    trim = float(np.interp(self.emerg_lead, [EMERG_LEAD_A1, EMERG_LEAD_A0], [EMERG_TRIM_HARD, EMERG_TRIM]))
    target = trim * float(np.interp(self.emerg_plan, [EMERG_PLAN_A1, EMERG_PLAN_A0], [1.0, 0.0])) if self.emerg else 0.0
    self.emerg_trim = float(np.clip(target, self.emerg_trim - EMERG_RATE_OFF * DT, self.emerg_trim + EMERG_RATE_ON * DT))
    return self.emerg_trim

  def adjust(self, x_obstacles, lead_xv_0, lead_xv_1, v_plan, t_idxs, personality):
    """x_obstacles (N+1, 3: lead 0, lead 1, cruise) -> shifted obstacles; also sets the onset cap for min_accel()"""
    v_plan = np.asarray(v_plan, dtype=float)
    t1 = float(t_idxs[1])
    x_obstacle_raw = float(np.min(x_obstacles[0]))  # the real obstacle, before any margin
    lead_margin = np.column_stack([get_light_onset_margin(lead_xv_0[:, 1]), get_light_onset_margin(lead_xv_1[:, 1]),
                                   np.zeros(len(v_plan))])
    emerg = self.update_emergency(lead_xv_0, v_plan, t1, t_idxs)
    dyn = self.update_dynamic_gap(lead_xv_0, v_plan, t1)
    v = np.maximum(v_plan, 0.0)
    gap = dyn * np.interp(v, DYN_GAP_BP, DYN_GAP_V)
    if personality == log.LongitudinalPersonality.aggressive:
      gap = gap + np.interp(v, AGG_GAP_BP, AGG_GAP_V)
    shift = emerg - get_light_onset_margin(v) - gap  # per node, applied to the leads only (cruise is unchanged)
    out = x_obstacles + lead_margin
    out[:, :2] += shift[:, None]
    v_lead = float(lead_xv_0[0, 1])
    a_lead = (float(lead_xv_0[1, 1]) - v_lead) / t1
    x_stop = float(lead_xv_0[0, 0]) + v_lead ** 2 / (2.0 * max(-a_lead, CAP_LEAD_A_MIN))  # where the lead itself stops
    self.update_onset_cap(x_stop if v_lead > 0.0 else x_obstacle_raw, v_plan, t1, t_idxs, float(v_plan[0]) - v_lead, dyn)
    return out


# Aggressive following gap at highway speed (owner 2026-11-08: "standard's gap above 90 km/h suits the aggressive mode
# better - let aggressive grow its gap smoothly over 80-100 km/h and reach standard's at 100"; owner-approved exception
# to "do not change T_FOLLOW", aggressive only). Below 80 km/h and every other personality: unchanged.
AGGR_T_V_BP = [80.0 / 3.6, 100.0 / 3.6]
AGGR_T_V = [1.25, 1.45]   # aggressive -> standard


def aggressive_t_follow(t_follow: float, personality, v_ego: float) -> float:
  if personality != log.LongitudinalPersonality.aggressive:
    return t_follow
  return max(t_follow, float(np.interp(v_ego, AGGR_T_V_BP, AGGR_T_V)))
