"""
dragonpilot (dpcr, Corolla Cross): the planner's own target for a stop behind a STOPPED lead, shaped like the owner's
manual stops (owner 2026-11-06: "do not slow early at 40-60 km/h; when a firm brake is needed, build it up linearly and
smoothly; at the end, from whatever level, ease off smoothly into a stop with no jolt").

Measured on the C4 (manual stops behind a stopped lead 10-27 21:01, 10-28 09:19 template, 10-29 19:46 vs the same lead
replayed through dpcr a0114478): the MPC starts a light brake ~80-100 m out and its decel shrinks as the gap closes
(09:19 at 15/10/5 m: dpcr 12/8/2 km/h at -0.82/-0.53/-0.37, the owner 23/15/6 km/h at -2.31/-2.42/-1.13), 10 km/h -> stop
~7 s vs the owner's 2.6-4.5 s. The owner waits until the decel needed to stop at the gap reaches ~1.7 m/s^2, builds it
at 1.5-3 m/s^3, holds about that decel, and eases off only in the last ~1.5 s.

This is the planner's stop target (not a layer after it): while the lead is stopped,
  need = v^2 / (2 (dRel - STOP_GAP))   the constant decel that stops exactly at STOP_GAP
  WAIT   need < START_NEED: no brake beyond a light lift (WAIT_ACCEL) - the plan's early light brake is not used
  BRAKE  -K_STOPPED * need (K_NEED for a lead that slowed to a stop, below), built at <= J_BUILD, eased at <= J_EASE
  RELEASE once easing at J_RELEASE would just reach the stopping controller's end line by END_V: ease linearly at
         J_RELEASE onto that line and follow it (never firmer than needed again: no second bite)
Anything else - a moving lead, no lead, an urgent need (>= URGENT_NEED) where the MPC brakes harder, standstill,
disengaged - hands back to the MPC target at <= J_HANDBACK.

A lead that slows to a stop while we already brake for it (owner 2026-11-07, route 000000f0 23:13:04 / 23:14:05:
"the halved brake start needs a little more around 15 km/h, mid toward the end, so it does not stop too close"): the
lead dropped under LEAD_STOPPED_V at 15 km/h / 14 m while the plan was at -1.2; WAIT eased it to -0.30 at J_EASE and
BRAKE never started (need ~0.8), the car ended 2.9 / 2.3 m back. When the stopped lead was tracked MOVING the frame
before and the target is already firmer than WAIT_ACCEL, the approach goes straight to BRAKE (-K_NEED x need, eased at
J_EASE), so it keeps the decel that stops at the gap and ends through the normal RELEASE. A lead first seen already
stopped (the far approach) is unchanged.

Earlier start behind a stopped lead (first experimental only 0383453b; owner 2026-11-07 night for every mode: "stops
behind a stopped car start braking far too late"; route 000000f0 23:09:52, non-experimental, stopped lead seen at 110 m
at 43 km/h, brake from 38.6 m, peak -2.69, stopped 2.4 m then crept to 1.5 m):
  - the start looks T_PRE (1.0 s, was 0.5) ahead: 11-06 21:55 / 11-07 23:09 the brake started at 37-39 m and the send
    side's soft onset lagged ~0.5 s behind it, so the car peaked at -2.6/-2.7;
  - it brakes K_STOPPED (1.05) x the need: with the earlier start K_NEED 1.2 eased the mid brake -1.7 -> -1.1 and the sim
    ended 3.7-3.9 m back; 1.05 holds the mid brake steady (sim 23:09:52 3.53 -> 3.32 m, peak -2.54 -> -1.86, 40 m
    -0.40 -> -1.01; 09:19 template 3.44 -> 3.30 m);
Experimental mode only (owner 2026-11-07: "tuning experimental mode must not touch the non-experimental longitudinal"):
  - WAIT follows the car's real decel: 11-06 21:51 (uphill ~4 %, 8 km/h, lead stopped 13.8 m ahead) the light lift
    -0.30 delivered -0.8 and the car stopped 8.7 m back. While the car slows harder than the need (+ WAIT_FB_MARGIN),
    the lift eases toward 0 at WAIT_FB_RATE; as soon as it does not, it comes back to WAIT_ACCEL at J_BUILD and the
    normal BRAKE start (need >= START_NEED) is unchanged. Never positive (no gas toward a stopped car).
"""
import numpy as np

from openpilot.common.realtime import DT_MDL

STOP_GAP = 3.2          # m: radar gap the plan aims at; the car ends ~0.2-0.4 m further back (Cross rule 3-3.5 m; owner manual 3.6-3.9 m)
LEAD_STOPPED_V = 1.0    # m/s: lead slower than this counts as stopped
START_NEED = 1.8        # m/s^2: start braking once the needed decel (0.5 s ahead) reaches this (owner's manual: 1.7-1.75)
URGENT_NEED = 2.8       # m/s^2: from here the MPC may brake harder than this target
WAIT_ACCEL = -0.30      # m/s^2: light lift while waiting
K_NEED = 1.2            # brake a little more than needed so the need falls slowly (owner's template ran 1.1-1.3)
J_BUILD = 1.5           # m/s^3: brake build-up (owner 1.5-3)
J_EASE = 0.8            # m/s^3: easing while braking
J_RELEASE = 0.8         # m/s^3: end release (owner's last 1.5 s: <= ~0.9)
J_HANDBACK = 2.0        # m/s^3
END_V = 5.0 / 3.6       # m/s: the release reaches the end line by here
# the stopping controller's end line (tncr1_long/stopping_controller.py END_CURVE), so the release lands on it
END_CURVE_V = [0.0, 1.0 / 3.6, 2.0 / 3.6, 3.0 / 3.6, 4.0 / 3.6, 5.0 / 3.6, 6.0 / 3.6, 7.0 / 3.6, 8.0 / 3.6]
END_CURVE_A = [-0.34, -0.36, -0.37, -0.37, -0.40, -0.45, -0.50, -0.57, -0.65]
MAX_DIST = 150.0        # m
T_PRE = 1.0             # s: the start looks this far ahead (the build and the car's lag; 0.5 until 11-07)
K_STOPPED = 1.05        # a lead first seen stopped: the earlier start already has margin, so brake closer to the need
WAIT_FB_MARGIN = 0.15   # m/s^2: experimental WAIT eases only while the car slows this much harder than the need
WAIT_FB_RATE = 0.6      # m/s^3: how fast the experimental WAIT lift eases toward 0


def _need(v_ego, d_rel):
  return v_ego ** 2 / (2.0 * max(d_rel - STOP_GAP, 0.3))


class StopApproach:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.phase = None  # None (MPC), 'wait', 'brake', 'release', 'handback'
    self.a = 0.0
    self.wait_a = WAIT_ACCEL  # experimental WAIT lift (follows the car's real decel)
    self.lead_moving = False  # the lead was tracked moving the frame before
    self.moved = False        # this approach started from a lead that slowed to a stop (K_NEED, not K_STOPPED)

  def _step(self, target, up, down):
    """move self.a toward target: up = easing rate (toward 0/positive), down = firming rate"""
    self.a = float(np.clip(target, self.a - down * self.dt, self.a + up * self.dt))
    return self.a

  def update(self, v_ego, lead, a_mpc, off, exp=False, a_ego=None):
    stopped_lead = (not off and lead is not None and bool(lead.status) and lead.vLead < LEAD_STOPPED_V
                    and 0.0 < lead.dRel < MAX_DIST and v_ego > 0.3)
    was_moving = self.lead_moving
    self.lead_moving = not off and lead is not None and bool(lead.status) and lead.vLead >= LEAD_STOPPED_V
    if not stopped_lead:
      self.wait_a = WAIT_ACCEL
      if self.phase in (None, 'handback') and (self.phase is None or abs(self.a - a_mpc) < 0.05):
        self.phase = None
        self.a = a_mpc
        return a_mpc
      self.phase = 'handback'
      return self._step(a_mpc, J_HANDBACK, J_HANDBACK)

    need = _need(v_ego, lead.dRel)
    if self.phase in (None, 'handback'):
      self.a = a_mpc if self.phase is None else self.a
      self.moved = was_moving and self.a < WAIT_ACCEL
      self.phase = 'brake' if self.moved else 'wait'
    need_pre = _need(v_ego, lead.dRel - v_ego * T_PRE)
    if self.phase == 'wait' and need_pre >= START_NEED:
      self.phase = 'brake'

    if self.phase == 'wait':
      if exp and a_ego is not None and a_ego < -(need + WAIT_FB_MARGIN):
        self.wait_a = min(self.wait_a + WAIT_FB_RATE * self.dt, 0.0)
      else:
        self.wait_a = max(self.wait_a - J_BUILD * self.dt, WAIT_ACCEL)
      out = self._step(max(a_mpc, self.wait_a), J_EASE, J_BUILD)
    else:
      goal = float(np.interp(v_ego, END_CURVE_V, END_CURVE_A))
      if self.phase == 'brake':
        a_end = float(np.interp(END_V, END_CURVE_V, END_CURVE_A))
        t_rel = max(a_end - self.a, 0.0) / J_RELEASE
        dv = -(self.a + a_end) / 2.0 * t_rel  # speed lost while easing to the end line
        if v_ego - END_V <= dv or (v_ego < END_CURVE_V[-1] and self.a >= goal):
          self.phase = 'release'
      if self.phase == 'brake':
        out = self._step(-(K_NEED if self.moved else K_STOPPED) * need, J_EASE, J_BUILD)
      else:
        # ease at J_RELEASE onto the end line, then follow it; hold (do not ease further) while the need runs ahead,
        # never firm up again
        target = self.a if need > -self.a + 0.3 else min(goal, self.a + J_RELEASE * self.dt)
        self.a = out = max(target, self.a)
    if need >= URGENT_NEED and a_mpc < out:
      self.a = out = a_mpc
    return out
