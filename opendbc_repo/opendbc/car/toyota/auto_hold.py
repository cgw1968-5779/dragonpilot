"""
dragonpilot: Toyota auto brake hold, ported from the owner's sunnypilot branch tnpb3
(opendbc/sunnypilot/car/toyota/auto_brake_hold.py; original channel by @AlexandreSato and @rav4kumar, Corolla Altis
behaviour tuned with the owner 2026-09-30 .. 10-09, history kept in the comments below).

With ACC engaged-off and the car standing in D, the hold keeps the car still by overriding PRE_COLLISION_2 (0x344) -
the only channel on this platform that can command the brake independent of ACC engagement. panda forwards the
camera's 0x344 normally and only blocks it (and lets ours through) while the car stands, gas is released, ACC main is
on and cruise is not engaged (safety param TOYOTA_PARAM_AUTO_HOLD, toyota.h). Any genuine PCS activation is passed
through unchanged.

dragonpilot differences: no Params switch (on for TSS2 cars without radar ACC / SecOC, with openpilot longitudinal);
the wheel pulse creep check only decides the hold's "really stands" (vEgo and standstill stay stock); the inputs come
from AutoHoldInputs filled by carstate (one hook) and the controller is called from carcontroller (one hook).
"""

import copy
import math

from opendbc.car import structs
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import TSS2_CAR, RADAR_ACC_CAR, SECOC_CAR
from opendbc.car.toyota.wheel_pulse import WheelPulseCreep

GearShifter = structs.CarState.GearShifter

# Two ways to engage (driver 2026-09-30 night, modelled on the factory EPB hold of the Corolla Cross):
#  - firm press at the stop: BRAKE_HOLD_ALLOWED_TIMER after the moment the brake force first reaches
#    BRAKE_HOLD_MIN_FORCE while standing still;
#  - no firm press: BRAKE_HOLD_LIGHT_TIMER after the wheels stopped.
# The hold's pre-brake clamp is a step (PBRTRGR, not a ramped request). rlog 000000dc-e0: with the old fixed 1 s after
# the standstill flag it showed as a -0.4..-0.6 m/s^2 jolt on 2 of 8 light manual stops, felt as part of stopping;
# the light stop then waited 2.5 s. Since the standstill flag comes from the wheel pulse counter (the car really
# stands), the waits are no longer needed (driver 2026-10-03, after a slope stop where the hold came 2.5 s late and he
# had to stomp: "with a real wheel-stop signal the delay is unnecessary, never more than 0.2 s"): 0.1 s after a firm
# press, 0.2 s after a light stop.
# Route 000000ee 2026-10-03 (23 held manual stops, hold_timing.py): the standstill flag came a median 0.42 s after
# the last wheel pulse, the hold 0.2 s after that, and the wheels never turned forward after a hold - but the body
# was still settling: the IMU came to rest a median 0.54 s (p75 ~0.7 s) after the flag, so the 1000-1400 -> 2040 N
# clamp landed under the driver's foot during the nod and felt like an intervention. Driver: "engage only when the
# wheels are certainly not turning; if it still cuts in before the real stop, delay it to the real stop - the system
# must determine the time from the data". The light stop therefore waits 0.7 s after the flag (~1.1 s after the last
# pulse, past the settle in 3 of 4 stops); a firm press 0.3 s.
BRAKE_HOLD_ALLOWED_TIMER = 30  # frames (0.3 s) after the firm press
# Like the factory electronic-parking-brake hold (Corolla Cross), the hold only arms when the driver has pressed the
# pedal firmly at the stop: brake pressure (BRAKE 0xA6 BRAKE_FORCE) must reach BRAKE_HOLD_MIN_FORCE at some point
# during the standstill. Altis 2026-09-30 night: an ordinary light stop sits at 600-1050 N, a deliberate firm press is
# well above. A light stop stays unheld, so nothing is clamped under a driver who is just easing to a halt. When the
# car does not broadcast the force (nan) the hold arms as before.
BRAKE_HOLD_MIN_FORCE = 1400.0  # N
# ... and without a firm press the hold still engages once the car has stood still this long (driver 2026-09-30:
# a long stop at a light should be held too, a short creep-stop should not)
BRAKE_HOLD_LIGHT_TIMER = 70  # frames (0.7 s)
# ... but only if the driver is holding the car with at least this force at that moment (driver 2026-10-02, after the
# 22:30 four-bookmark stop): pressing ~760-800 N the brake only just balanced the hybrid's creep torque, so when the hold
# clamped to its fixed ~1360 N the body rocked (IMU +0.61 / -0.42); at 960 N (21:17:44, 00:13:48) the same clamp was not
# felt. Lighter than this the hold waits (it engages as soon as the driver presses a little more); released lightly, the
# car creeps as it would without the hold. Once engaged the hold stays on whatever the pedal does.
BRAKE_HOLD_LIGHT_MIN_FORCE = 900.0  # N
# Both timers count from the REAL stop (driver 2026-10-01 22:30, four bookmarks: a very slow, very gentle manual stop
# and the hold "suddenly cut in" while the car still rolled below the ~0.5 km/h wheel-speed floor). The standstill flag
# now comes from the wheel pulse counter as well (wheel_pulse.py): while the car still creeps it is not standstill, so
# these timers restart until it really stands.
# Once ENGAGED the hold must not give up at the first wheel pulse. Route 000000ec 2026-10-03 23:51:59 and 23:52:06
# (steep downhill, driver's bookmarks): the driver lifted off, the hold clamped its fixed ~1360 N, 0.4-0.7 s later the
# pulse counter stepped once (the car gave a few cm), standstill went false, the old code dropped the hold at once, the
# PCM released the brake entirely and the car rolled away freely (0 -> 6-11 km/h at ~1.7 m/s^2 with no pedal) until
# the driver caught it. Driver: "I felt the car held, then the slope was too steep, the hold gave up and it slid
# forward". So the hold now stays on while the WHEEL SPEEDS still read zero (the window in which panda lets the 0x344
# override through at all: toyota_tx_hook refuses it once vehicle_moving), whatever the pulse counter says; a creep
# before engagement still restarts the timers. On a slope where 1360 N is less than the grade force the PCM's hold
# cannot physically hold the car - it can only drag - and the driver must keep the pedal; a stronger hold needs ACC
# engaged (stopAccel) or a different PCM channel.

DISALLOWED_GEARS = (GearShifter.park, GearShifter.reverse)

# The hold message is PRE_COLLISION_2 with PBRTRGR set and DSS1GDRV, the PCS deceleration request (signed 10 bit,
# 0.1 m/s^2 per count). The original code wrote the magic 0x3FF, which packs to raw -10 = -1.0 m/s^2; the PCM answers
# that with a fixed ~1360 N (BRAKE 0xA6), not enough on a steep slope (000000ec 23:51:59: rolled away at ~1.7 m/s^2).
# Driver 2026-10-03: "can the hold not ask for more, say 1800 N?" - untested whether the PCM scales the clamp with
# this request; try -1.5 / -2.0 here parked on a flat lot and read 0xA6 before relying on it.
# Owner 2026-10-05: "can the system tell that it stopped on a slope, and only then clamp harder; on the flat keep the old
# force". carControl.orientationNED pitch (deg, + = nose up) at every stop of routes ec/ee/f2 (pitch_at_stops.py):
# flat stops sit at +0.5..+1.9 (mount/crown bias ~+1.0), the 2026-10-03 roll-aways at -6.4..-9.3, ramps and driveways
# at -2.5..-3.9 and +2.8..+4.0. Slope = |pitch - FLAT_PITCH_DEG| >= SLOPE_PITCH_DEG, decided when the hold engages and
# kept for that hold (no flip-flop while held). Flat: the old -1.0 request (~1360 N); slope: -1.5 (~2040 N, confirmed
# holding on the 10-03 hill on 2026-10-04).
BRAKE_HOLD_DECEL = -1.0  # m/s^2 requested in DSS1GDRV while holding on the flat (~1360 N)
BRAKE_HOLD_DECEL_SLOPE = -1.5  # m/s^2 on a slope (~2040 N) - kept for reference, superseded by the pitch schedule below
FLAT_PITCH_DEG = 1.0  # deg, what this car reports standing on the flat
SLOPE_PITCH_DEG = 2.0  # deg away from flat that counts as a slope (either direction)
# Pitch-scaled hold (owner 2026-10-09, route 00000118 00:03:39, parking-garage ramp at -9.2 deg, D gear: the hold
# engaged at -1.5 (2040 N) as the driver lifted from 3760 N, held 1.1 s, then the grade plus the creep torque turned the
# wheels; once the wheels move panda refuses the 0x344 override (vehicle_moving), the PCM let go (0 N) and the car
# rolled to 6 km/h until the driver braked. At -8.3 / -8.4 deg (00:03:56, 00:04:09) the same 2040 N held; the driver
# alone held -9.2 deg at 2600 N. Owner: "find the cause; is there room for more force on a slope?" -> "do it".)
# The request now follows the grade: -(HOLD_BASE + HOLD_PER_G * g * sin|pitch - flat|), clipped to HOLD_MIN..HOLD_MAX:
# flat -1.0 (unchanged), 4 deg -1.5 (as before), 8 deg ~-2.4 (~3300 N), 10 deg ~-2.8. On a slope it is also never
# lighter than the force the driver was holding (DSS1GDRV ~ BRAKE_HOLD_N_PER_DECEL N per m/s^2). UNTESTED above -1.5:
# check BRAKE 0xA6 on the first slope holds that the PCM honours the larger request.
HOLD_BASE = 0.6  # m/s^2
HOLD_PER_G = 1.3  # x the grade's g * sin(pitch)
HOLD_MIN = 1.0  # m/s^2 (flat)
HOLD_MAX = 3.0  # m/s^2


def hold_decel_for_pitch(pitch_deg: float) -> float:
  if not math.isfinite(pitch_deg):
    return BRAKE_HOLD_DECEL
  grade = 9.81 * math.sin(math.radians(abs(pitch_deg - FLAT_PITCH_DEG)))
  return -min(max(HOLD_BASE + HOLD_PER_G * grade, HOLD_MIN), HOLD_MAX)

# Route 00000100 2026-10-05 19:53:53 (driver's bookmarks "3+1", manual stop): the wheels stopped, the IMU settled in
# 0.13 s, no wheel pulse after the hold - the stop detection was right. The jerk was the clamp itself: the driver's
# foot crept from 640 to 920 N, crossed BRAKE_HOLD_LIGHT_MIN_FORCE 1.4 s after the stop, and the hold stepped the brake
# 920 -> 1360 N under the foot (IMU fore/aft wiggle for 0.6 s). Delaying the hold only moves that step. Owner
# 2026-10-06 chose both fixes:
#  B) while the driver is still holding the car, the hold waits; it takes over when the driver starts to LIFT: the
#     brake force falls BRAKE_HOLD_LIFT_DROP below the highest force of this stop, or the pedal is released (the
#     factory hold of the Corolla Cross takes over the same way, the car never feels it);
#  A) the clamp then starts at the force the driver was holding (DSS1GDRV ~ BRAKE_HOLD_N_PER_DECEL N per m/s^2:
#     -1.0 -> ~1360 N, -1.5 -> ~2040 N measured) and ramps to the flat/slope request over BRAKE_HOLD_RAMP_FRAMES,
#     never starting below BRAKE_HOLD_START_DECEL (~820 N, above the ~760-800 N the hybrid's creep torque needs).
#     Requests lighter than -1.0 are untested on the PCM: check 0xA6 on the first stops.
# With the hand-over at the driver's own force there is no step, so the light-stop force gate (900 N) is no longer used.
BRAKE_HOLD_LIFT_DROP = 200.0  # N below this stop's highest brake force counts as lifting
BRAKE_HOLD_N_PER_DECEL = 1360.0  # N of brake force per m/s^2 of DSS1GDRV
BRAKE_HOLD_START_DECEL = -0.6  # m/s^2, the lightest request the hold starts from
BRAKE_HOLD_RAMP_FRAMES = 50  # 0.5 s from the start request to the hold request

# PRE_COLLISION_2 fields that go high when the camera's own PCS/AEB is genuinely intervening this
# frame (PCSALM mirrors PRECOLLISION_ACTIVE; IBTRGR/PBATRGR/PREFILL/AVSTRGR/PBRTRGR/PPTRGR are its
# actuation triggers - see create_pcs_commands for the same field set on the stock-DSU PCS path).
# Deliberately over-inclusive: a false positive here just means we pass a quiescent frame through
# instead of holding it, never the other way around, so err toward checking more fields, not fewer.
PCS_TRIGGER_FIELDS = ("PCSALM", "IBTRGR", "PBATRGR", "PREFILL", "AVSTRGR", "PBRTRGR", "PPTRGR")


def pcs_is_active(pre_collision_2: dict) -> bool:
  return any(pre_collision_2.get(field, 0) for field in PCS_TRIGGER_FIELDS) or pre_collision_2.get("DSS1GDRV", 0) != 0


def auto_hold_available(CP) -> bool:
  """TSS2 Toyotas without radar ACC or SecOC (camera PRE_COLLISION_2 on bus 2), with openpilot longitudinal (0x344 is
  only in panda's longitudinal TX list)."""
  return CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR - SECOC_CAR) and CP.openpilotLongitudinalControl


def auto_hold_pt_messages(CP) -> list:
  # BRAKE 0xA6 (brake force, hybrids) and SPEED 0xB4 (wheel pulse counter); no alive check, as in tnpb3
  return [("BRAKE", float('nan')), ("SPEED", float('nan'))] if auto_hold_available(CP) else []


def auto_hold_cam_messages(CP) -> list:
  return [("PRE_COLLISION_2", float('nan'))] if auto_hold_available(CP) else []


class AutoHoldInputs:
  """carstate side: what the hold needs besides CarState (filled every carstate frame)."""
  def __init__(self):
    self.pre_collision_2: dict = {}
    self.brake_force = float('nan')
    self.wheel_encoder = float('nan')

  def update(self, cp, cp_cam) -> None:
    self.pre_collision_2 = copy.copy(cp_cam.vl["PRE_COLLISION_2"])
    self.brake_force = float(cp.vl["BRAKE"]["BRAKE_FORCE"])
    self.wheel_encoder = float(cp.vl["SPEED"]["ENCODER"])


# Holds the car at a stop with cruise off by overriding PRE_COLLISION_2. Yields to any genuine PCS activation this
# frame - the real message is only ever overridden while it's quiescent - and releases for the rest of the current
# standstill episode on a brake press, rather than for a single frame.
class AutoBrakeHold:
  def __init__(self):
    self.active = False
    self._counter = 0
    self._released = False
    self._armed = False
    self._firm_frame = 0
    self._prev_brake_pressed = False
    self._engaged = False
    self._hold_decel = BRAKE_HOLD_DECEL
    self._hold_req = BRAKE_HOLD_DECEL  # the request actually sent: ramps from the driver's force to _hold_decel
    self._hold_step = 0.0
    self._peak_force = 0.0
    self.wheel_pulse = WheelPulseCreep()

  def update(self, inp: AutoHoldInputs, out: structs.CarState, frame: int, packer, pitch_deg: float = FLAT_PITCH_DEG) -> list:
    force = inp.brake_force
    # "really stands": the wheel speeds read 0 and the wheel pulse counter shows no creep below the 0.5 km/h floor
    creep_v = self.wheel_pulse.update(inp.wheel_encoder, abs(out.vEgoRaw) < 1e-3, force)
    standstill = out.standstill and creep_v == 0.0

    ws = out.wheelSpeeds
    wheels_zero = max(abs(ws.fl), abs(ws.fr), abs(ws.rl), abs(ws.rr)) < 1e-3  # panda's vehicle_moving == false
    relay_blocked = (wheels_zero and out.cruiseState.available and not out.cruiseState.enabled and
                      not out.gasPressed)
    # before engagement the car must really stand (wheel pulse standstill); an ACTIVE hold persists for as long as the
    # wheel speeds read zero, so one pulse step on a slope does not drop it. A hold that is not active (released by a
    # fresh press, or never engaged) ends its episode as soon as the pulse counter shows the car moving: route 000000ee
    # 2026-10-03 11:55 - hold, fresh press (released), the car crept ~1 km/h to the next stop with the wheel speeds
    # still reading zero, and the "released" flag carried over, so the second stop was never held.
    hold_allowed = relay_blocked and out.gearShifter not in DISALLOWED_GEARS and (standstill or self.active)

    if hold_allowed:
      # only a fresh press releases hold - the press that caused the stop is already reflected in
      # _prev_brake_pressed by the time standstill is reached, so it doesn't count as a release
      if out.brakePressed and not self._prev_brake_pressed:
        self._released = True
      self._counter += 1
      if not self._armed and (math.isnan(force) or force >= BRAKE_HOLD_MIN_FORCE):
        self._armed = True
        self._firm_frame = self._counter
      firm_ready = self._armed and self._counter - self._firm_frame >= BRAKE_HOLD_ALLOWED_TIMER
      held_long = self._counter > BRAKE_HOLD_LIGHT_TIMER
      if math.isfinite(force) and not self._engaged:
        self._peak_force = max(self._peak_force, force)
      # B: hand over only when the driver lifts (force falling or pedal released); no force signal -> as before
      lifting = (not out.brakePressed or math.isnan(force) or force <= self._peak_force - BRAKE_HOLD_LIFT_DROP)
      if (firm_ready or held_long) and lifting and not self._released and not self._engaged:
        on_slope = math.isfinite(pitch_deg) and abs(pitch_deg - FLAT_PITCH_DEG) >= SLOPE_PITCH_DEG
        self._hold_decel = hold_decel_for_pitch(pitch_deg)
        if on_slope and math.isfinite(force) and self._peak_force > 0.0:
          # on a slope never hold lighter than the driver was holding (up to HOLD_MAX)
          self._hold_decel = max(-HOLD_MAX, min(self._hold_decel, -self._peak_force / BRAKE_HOLD_N_PER_DECEL))
        # A: start at the force the driver was holding, then ramp to the hold request
        start = -self._peak_force / BRAKE_HOLD_N_PER_DECEL if math.isfinite(force) else self._hold_decel
        self._hold_req = max(self._hold_decel, min(BRAKE_HOLD_START_DECEL, start))
        self._hold_step = (self._hold_req - self._hold_decel) / BRAKE_HOLD_RAMP_FRAMES
        self._engaged = True
      elif self._engaged:
        self._hold_req = max(self._hold_decel, self._hold_req - self._hold_step)
      self.active = self._engaged and not self._released
    else:
      self._counter = 0
      self.active = False
      self._released = False
      self._armed = False
      self._engaged = False
      self._peak_force = 0.0

    self._prev_brake_pressed = out.brakePressed

    can_sends = []
    if relay_blocked and frame % 2 == 0:
      override = self.active and not pcs_is_active(inp.pre_collision_2)
      can_sends.append(toyotacan.create_brake_hold_command(packer, frame, inp.pre_collision_2, override,
                                                           hold_decel=self._hold_req))

    return can_sends
