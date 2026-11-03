from types import SimpleNamespace as N

from opendbc.can import CANPacker, CANParser
from opendbc.car.toyota.hybrid_state import HybridState
from opendbc.car.toyota.values import ToyotaFlags

DBC = 'toyota_nodsu_pt_generated'


def parse(rpm, running, soc):
  packer = CANPacker(DBC)
  cp = CANParser(DBC, [("ENGINE_RPM", float('nan')), ("HYBRID_BATTERY", float('nan'))], 0)
  msgs = [packer.make_can_msg("ENGINE_RPM", 0, {"RPM": rpm, "ENGINE_RUNNING": running}),
          packer.make_can_msg("HYBRID_BATTERY", 0, {"HV_SOC_PCT": soc})]
  cp.update([(0, msgs)])
  return cp


def test_engine_and_soc():
  hs = HybridState(N(flags=ToyotaFlags.HYBRID.value))
  ret = N(engineOff=False, hvSoc=-1.0)
  hs.update(parse(0, 0, 44), ret)
  assert ret.engineOff and ret.hvSoc == 44
  hs.update(parse(1200, 1, 0), ret)   # 0 = not valid: keep the last one
  assert not ret.engineOff and ret.hvSoc == 44


def test_not_hybrid_untouched():
  hs = HybridState(N(flags=0))
  ret = N(engineOff=False, hvSoc=-1.0)
  hs.update(None, ret)
  assert ret.hvSoc == -1.0
