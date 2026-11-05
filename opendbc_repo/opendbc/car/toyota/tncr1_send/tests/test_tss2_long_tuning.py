# dpcr: tncr1's TSS2_LONG_TUNING ki table (ToyotaTSS2Long was on for every C4 tncr1 route) is the dpcr TSS2 table
from opendbc.car.toyota.carcontroller import get_long_tune
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, CarControllerParams


def test_tss2_uses_tncr1_long_tuning():
  CP = CarInterface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2)
  pid = get_long_tune(CP, CarControllerParams(CP))
  assert list(pid._k_i[0]) == [0., 0.3, 5.0, 12., 27., 36.]
  assert list(pid._k_i[1]) == [0.50, 0.52, 0.25, 0.23, 0.10, 0.09]
