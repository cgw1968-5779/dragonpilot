# dragonpilot (dpcr, Corolla Cross Hybrid): hybrid engine state and battery charge for the eco EV corrections
# (dragonpilot/selfdrive/controls/lib/eco_ev.py), ported from the owner's sunnypilot branch tncr1
# (opendbc/sunnypilot/car/toyota/carstate_ext.py: carStateSP.engineOff / hvSoc).
# - ENGINE_RPM (0x1C4): engine off = RPM below ENGINE_RUNNING_RPM and ENGINE_RUNNING clear (Corolla rlogs: 0 when off,
#   >= 800 when on).
# - HYBRID_BATTERY (0x380, 5 Hz) HV_SOC_PCT: the last valid value (0 / 128 = not valid yet), -1 until one arrives.
# Both are registered without an alive check (frequency nan): only hybrids send them, and a missing one only leaves the
# eco corrections idle (engine treated as running, battery unknown).
from opendbc.car.toyota.values import ToyotaFlags

ENGINE_RUNNING_RPM = 300.0


def hybrid_pt_messages(CP) -> list:
  return [("ENGINE_RPM", float('nan')), ("HYBRID_BATTERY", float('nan'))] if CP.flags & ToyotaFlags.HYBRID.value else []


class HybridState:
  def __init__(self, CP):
    self.enabled = bool(CP.flags & ToyotaFlags.HYBRID.value)
    self.soc = -1.0

  def update(self, cp, ret) -> None:
    if not self.enabled:
      return
    rpm = float(cp.vl["ENGINE_RPM"]["RPM"])
    ret.engineOff = rpm < ENGINE_RUNNING_RPM and not bool(cp.vl["ENGINE_RPM"]["ENGINE_RUNNING"])
    soc = float(cp.vl["HYBRID_BATTERY"]["HV_SOC_PCT"])
    if 0.0 < soc <= 100.0:
      self.soc = soc
    ret.hvSoc = self.soc
