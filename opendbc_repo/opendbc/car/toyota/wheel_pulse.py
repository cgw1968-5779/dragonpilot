# dragonpilot: ported from the owner's sunnypilot branch tnpb3 (opendbc/sunnypilot/car/toyota/wheel_pulse.py).
"""
Wheel pulse creep detection (Toyota SPEED 0xB4 ENCODER).

The wheel speeds read 0 below ~0.5 km/h, so the standstill flag is set while a slowly braking car still rolls. The
wheel pulse counter keeps counting: C3X route 000000e6 calibration 0.049 m per count, forward steps of 4 counts
(~0.2 m), backward steps of -1 (a rebound at the stop). Normal openpilot stops show no forward step after the wheel
speeds read 0 (the car stops 0.2-0.6 s later within the last step); a forward step there proves the car is still
creeping (driver 2026-10-01 22:30: a very slow manual stop, the auto brake hold cut in while it still rolled).
Driver: "from now on the wheel pulses should be used".

While the wheel speeds read 0, a forward step yields a creep speed of (distance of the step) / (time since the previous
step), capped at the speed floor, that decays as one step's distance over the time since the step and is dropped
CREEP_HOLD_T after it. On sunnypilot tnpb3 the car state reported this speed (so standstill was False for every consumer). dragonpilot keeps
vEgo and standstill untouched: only the auto brake hold (auto_hold.py) uses it, to know the car really stands.
"""
import math

M_PER_COUNT = 0.049     # m
STEP_M = 4 * M_PER_COUNT  # m, one forward step
V_FLOOR = 0.14          # m/s (0.5 km/h): below this the wheel speeds read 0
CREEP_HOLD_T = 2.5      # s after a step during which the car still counts as creeping
# Firm driver brake: a car held by a firm press cannot keep creeping, so a step it takes while settling counts only
# briefly. C3X route 00000119 2026-10-09 09:26:28 and 09:26:31 (ramp, -9 deg, ACC main on): after the stop the car
# settled one step (4 counts) forward under 3000 N, then stood still under 4000-7900 N with the counter unchanged for
# 1.4 s - but the step kept it "creeping" at 0.5 km/h for the full CREEP_HOLD_T, so the auto brake hold never saw a
# standstill; the driver lifted after 1.4 s expecting the hold and the car rolled to 4 km/h. With FIRM_BRAKE_N or more
# (above any light stop, 600-1050 N, and the 1400 N firm-press hold trigger) the creep ends FIRM_SETTLE_T after the
# step and stays ended while the driver lifts (that is when the hold takes over); a car that really still moves steps
# again and is creeping again at once.
FIRM_BRAKE_N = 2000.0   # N (BRAKE 0xA6 BRAKE_FORCE)
FIRM_SETTLE_T = 0.5     # s
DT = 0.01               # s, carstate step


class WheelPulseCreep:
  def __init__(self):
    self.prev = math.nan
    self.t = 0.0
    self.t_step = -1e9    # time of the last forward step (any speed)
    self.v_step = 0.0     # creep speed estimated at the last step taken while the wheels read 0
    self.t_zero_step = -1e9
    self.settled = False  # a firm press held the car still after the last step (FIRM_*)

  def update(self, encoder: float, wheels_zero: bool, brake_force: float = math.nan) -> float:
    """Returns the creep speed (m/s) to report while the wheel speeds read 0, else 0."""
    self.t += DT
    if math.isnan(encoder):
      return 0.0
    if not math.isnan(self.prev) and encoder != self.prev:
      delta = (encoder - self.prev + 128) % 256 - 128
      if 0 < delta <= 16:
        if wheels_zero:
          self.v_step = min(delta * M_PER_COUNT / max(self.t - self.t_step, DT), V_FLOOR)
          self.t_zero_step = self.t
          self.settled = False
        self.t_step = self.t
    self.prev = encoder
    if not wheels_zero:
      self.t_zero_step = -1e9
      self.settled = False
      return 0.0
    since = self.t - self.t_zero_step
    if brake_force >= FIRM_BRAKE_N and since > FIRM_SETTLE_T:
      self.settled = True
    if since > CREEP_HOLD_T or self.settled:
      return 0.0
    return min(self.v_step, STEP_M / max(since, DT))
