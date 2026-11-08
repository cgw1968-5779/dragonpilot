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

Soft start in proportion to the need (owner 2026-11-08 ~14:30, route 000000fe 14:17:20: "a cut-in lead, it braked far
harder than needed" -> "the urgent brake should start soft too; brake in proportion to the need"). 14:17: after a
cut-in the lead slowed 103 -> 86 km/h; at 101 km/h, 43.8 m (1.56 s), aLeadK -1.93 the regime switched on and the plan
went to -3.5 after 0.1 s (car -3.08, 28 km/h lost) though the need was 1.9-2.1 and the lead stopped braking 1.5 s later.
At its trigger f8 00:39 looked the same (58 km/h, 28 m, aLeadK -1.75, need 1.7, closing 2 m/s) - only what the lead did
next differed - so the switch is unchanged and the brake is shaped instead: after the SOFT_T start it builds at
J(need) (J_NEED_BP -> J_NEED_V: 6 m/s^3 at need 1.5, 15 at 2.5) toward K_NEED x need, the full ACCEL_MIN only from
NEED_FULL; while the need falls it eases toward that at J_EASE; never lighter than the plan. A cut-in still goes to
ACCEL_MIN at once. The send side passes this brake as is while the switch is on (carcontroller hook, shared
opendbc toyota/urgent_switch.py) so the car gets the designed build, not the lead-urgent onset template (<= 4 m/s^3).
Counterfactual f8 00:39 (cf_soft4.py): min gap 4.89 -> 3.54 m; 14:17 (point mass): car peak -3.49 -> -2.69, speed
lost 28 -> 16 km/h.
"""

from opendbc.car.interfaces import ACCEL_MIN
from opendbc.car.toyota.urgent_switch import UrgentSwitch
import numpy as np

SOFT_T = 0.1            # s
SOFT_J = 3.0            # m/s^3 (-0.3 in the first 0.1 s)
J_NEED_BP = [1.5, 2.5]  # m/s^2 need
J_NEED_V = [6.0, 15.0]  # m/s^3 build after the soft start
K_NEED = 1.2            # brake this x the need ...
NEED_FULL = 2.5         # m/s^2 ... and the full ACCEL_MIN from this need
J_EASE = 2.0            # m/s^3 while the need falls
J_HANDBACK = 2.0        # m/s^3


class UrgentBrake:
  def __init__(self, dt: float):
    self.dt = dt
    self.switch = UrgentSwitch(dt)
    self.active = False
    self.handback = False
    self.a = 0.0
    self.t_on = 0.0         # time since switching on
    self.soft = True        # this switch-on starts soft

  def update(self, v_ego: float, lead, a_plan: float, off: bool) -> float:
    on_now = self.switch.update(v_ego, lead, off)
    if off:
      self.active = self.handback = False
      return a_plan
    if on_now:
      self.soft = not self.switch.cutin
      if not self.handback:
        self.a = a_plan
      self.handback, self.t_on = False, 0.0
    was_active, self.active = self.active, self.switch.active
    if self.active:
      self.handback = False
      self.t_on += self.dt
      need = self.switch.need
      if not self.soft:
        self.a = ACCEL_MIN
      elif self.t_on <= SOFT_T + 1e-9:
        self.a = self.a - SOFT_J * self.dt  # the soft start
      else:
        target = ACCEL_MIN if need >= NEED_FULL else max(ACCEL_MIN, -K_NEED * need)
        if self.a > target:
          self.a = max(target, self.a - float(np.interp(need, J_NEED_BP, J_NEED_V)) * self.dt)
        else:
          self.a = min(target, self.a + J_EASE * self.dt)
      self.a = min(a_plan, self.a)  # never lighter than the plan
      return self.a
    if was_active:
      self.handback = True
    if self.handback:
      self.a = min(a_plan, self.a + J_HANDBACK * self.dt)
      if self.a >= a_plan - 1e-6:
        self.handback = False
        return a_plan
      return self.a
    return a_plan
