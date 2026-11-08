from types import SimpleNamespace as N

from opendbc.can import CANPacker, CANParser
from opendbc.car.toyota.hybrid_state import HybridState, hybrid_pt_messages

DBC = 'toyota_nodsu_pt_generated'


def parser():
  return CANParser(DBC, hybrid_pt_messages(None), 0)


def frames(rpm, running, soc):
  packer = CANPacker(DBC)
  return [packer.make_can_msg("ENGINE_RPM", 0, {"RPM": rpm, "ENGINE_RUNNING": running}),
          packer.make_can_msg("HYBRID_BATTERY", 0, {"HV_SOC_PCT": soc})]


def test_engine_and_soc():
  hs, cp = HybridState(), parser()
  ret = N(engineOff=False, hvSoc=-1.0)
  cp.update([(1, frames(0, 0, 44))])
  hs.update(cp, ret)
  assert ret.engineOff and ret.hvSoc == 44
  cp.update([(2, frames(1200, 1, 0))])   # SOC 0 = not valid: keep the last one
  hs.update(cp, ret)
  assert not ret.engineOff and ret.hvSoc == 44


def test_never_received_stays_idle():
  # a car that never sends 0x1C4 / 0x380 (or the hybrid flag missed at boot does not matter): engine treated as running
  hs, cp = HybridState(), parser()
  ret = N(engineOff=False, hvSoc=-1.0)
  cp.update([(1, [])])
  hs.update(cp, ret)
  assert not ret.engineOff and ret.hvSoc == -1.0
