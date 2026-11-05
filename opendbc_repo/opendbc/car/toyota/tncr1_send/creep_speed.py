"""
dpcr: tncr1's wheel_pulse_creep_sp (opendbc/sunnypilot/car/toyota/carstate_ext.py, e1656efd1a), unchanged logic:
below the ~0.5 km/h wheel-speed floor the SPEED (0xB4) ENCODER pulse counter still shows a creeping car; the car state
reports that speed (vEgoRaw, vEgo), so standstill stays False while the car still creeps - the stopping controller,
stop-gap layers and end-force ceiling see the real end of the stop, as on tncr1.
"""
from opendbc.car.toyota.wheel_pulse import WheelPulseCreep


class Tncr1CreepSpeed:
  def __init__(self):
    self.wheel_pulse = WheelPulseCreep()
    self.wheel_encoder = float('nan')  # SPEED (0xB4) ENCODER: wheel pulse counter

  def update(self, ret, cp, brake_force: float) -> None:
    if "SPEED" in cp.vl:
      self.wheel_encoder = float(cp.vl["SPEED"]["ENCODER"])
    # brake force of the previous frame (parsed later in the update); nan when not parsed -> unchanged behaviour
    creep_v = self.wheel_pulse.update(self.wheel_encoder, abs(ret.vEgoRaw) < 1e-3, brake_force)
    if creep_v > 0.0:
      ret.vEgoRaw = creep_v
      ret.vEgo = max(ret.vEgo, creep_v)
