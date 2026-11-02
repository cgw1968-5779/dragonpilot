import os

from cereal import car
from dragonpilot.selfdrive.car.fixed_platform import fixed_platform_car_params, FIXED_PLATFORM

VIN = "RKLKCAAG3T0381660"


def cp_bytes(platform=FIXED_PLATFORM, vin=VIN, n_fw=8):
  cp = car.CarParams.new_message(carFingerprint=platform, brand="toyota", carVin=vin)
  cp.init("carFw", n_fw)
  return cp.to_bytes()


class FakeParams(dict):
  def get(self, k):
    return dict.get(self, k)


def test_cache_first():
  assert fixed_platform_car_params(FakeParams(CarParamsCache=b"c", CarParamsPersistent=cp_bytes())) == b"c"


def test_persistent_of_the_fixed_platform_skips_the_query():
  p = cp_bytes()
  assert fixed_platform_car_params(FakeParams(CarParamsPersistent=p)) == p
  assert os.environ["FINGERPRINT"] == FIXED_PLATFORM


def test_no_usable_persistent_runs_the_stock_query():
  assert fixed_platform_car_params(FakeParams()) is None
  assert fixed_platform_car_params(FakeParams(CarParamsPersistent=cp_bytes(platform="TOYOTA_RAV4_TSS2"))) is None
  assert fixed_platform_car_params(FakeParams(CarParamsPersistent=cp_bytes(n_fw=0))) is None
  assert fixed_platform_car_params(FakeParams(CarParamsPersistent=cp_bytes(vin="0" * 3))) is None
  assert fixed_platform_car_params(FakeParams(CarParamsPersistent=b"garbage")) is None
