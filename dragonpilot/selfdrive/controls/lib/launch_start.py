"""
dragonpilot (dpcr, Corolla Cross): leave the standstill hold as soon as the lead drives off (owner 2026-11-08, route
00000103 16:03:43: "when the lead drives off the car does not follow at once"; chose option A).

16:03:43, stopped 3.2 m behind the lead: the lead crept off (> 1 km/h 16:03:42.7, > 3 km/h 16:03:44.4) and the plan
went positive at 16:03:43.5, but only +0.02..+0.10 while the lead crept at 1.5-2.3 km/h. should_stop is "own speed
below 0.3 m/s and target below 0.1", so the -1.20 standstill hold stayed until 16:03:44.8, the command went positive at
16:03:45.1 and the car moved at 16:03:45.7 (1.3 s after the lead passed 3 km/h, 3.0 s after it began to creep).

Now, while we are stopped (v_ego < V_EGO), a lead tracked faster than LEAD_V for T_ON and a positive plan clear
should_stop, so the controller starts on the plan's own (small) acceleration at once. Nothing changes the plan itself,
and a negative plan still holds. Measured radar noise behind a stopped lead (00000103 16:02:42-16:03:42, 60 s):
vLead <= 0.25 m/s, never above LEAD_V.
"""
LEAD_V = 0.3     # m/s
T_ON = 0.2       # s the lead must be tracked faster than LEAD_V
V_EGO = 0.3      # m/s: "we are stopped" (as should_stop)


class LaunchStart:
  def __init__(self, dt: float):
    self.dt = dt
    self.t = 0.0

  def update(self, should_stop: bool, v_ego: float, lead, a_target: float) -> bool:
    if v_ego < V_EGO and lead is not None and lead.status and lead.vLead > LEAD_V:
      self.t += self.dt
    else:
      self.t = 0.0
    if should_stop and self.t >= T_ON - 1e-6 and a_target > 0.0:
      return False
    return should_stop
