# dragonpilot (dppb, Corolla Altis Hybrid): hybrid engine state and battery charge for the eco EV launch
# (dragonpilot/selfdrive/controls/lib/eco_ev.py), same signals as dpcr's hybrid_state.py (from the owner's sunnypilot
# tncr1 carStateSP.engineOff / hvSoc).
# - ENGINE_RPM (0x1C4): engine off = RPM below ENGINE_RUNNING_RPM and ENGINE_RUNNING clear (Altis rlogs: 0 when off,
#   >= 800 when on).
# - HYBRID_BATTERY (0x380, 5 Hz) HV_SOC_PCT: the last valid value (0 / 128 = not valid yet), -1 until one arrives
#   (Altis 11-01..11-08: 40-51 %).
# Unlike dpcr, NOT gated on ToyotaFlags.HYBRID: on the Altis that flag comes from a firmware query the hybrid ECU
# answers only about half the boots. Both messages are registered without an alive check (frequency nan) and are only
# used once they have actually arrived (ts_nanos), so a car that never sends them keeps engineOff False / hvSoc -1 and
# the eco launch idle.

ENGINE_RUNNING_RPM = 300.0


def hybrid_pt_messages(CP) -> list:
  return [("ENGINE_RPM", float('nan')), ("HYBRID_BATTERY", float('nan'))]


class HybridState:
  def __init__(self):
    self.soc = -1.0

  def update(self, cp, ret) -> None:
    if cp.ts_nanos["ENGINE_RPM"]["RPM"] > 0:
      rpm = float(cp.vl["ENGINE_RPM"]["RPM"])
      ret.engineOff = rpm < ENGINE_RUNNING_RPM and not bool(cp.vl["ENGINE_RPM"]["ENGINE_RUNNING"])
    if cp.ts_nanos["HYBRID_BATTERY"]["HV_SOC_PCT"] > 0:
      soc = float(cp.vl["HYBRID_BATTERY"]["HV_SOC_PCT"])
      if 0.0 < soc <= 100.0:
        self.soc = soc
    ret.hvSoc = self.soc
