"""
dragonpilot (dpcr, Corolla Cross Hybrid): fixed platform, no VIN / firmware query at boot.
Ported from the owner's sunnypilot branch tncr1 (sunnypilot/selfdrive/car/interfaces.cached_car_params, 7312710).

Why (tncr 2026-10-29, Corolla Cross Hybrid, dealer CarMate head unit with Toyota Drive+ Link): the head unit polls the
car's ECUs over UDS all through the ignition cycle and feeds the Drive+ Link app. It stops polling the moment another
tester's diagnostic frames appear on the bus and stays stopped into the next ignition cycle, so the app's vehicle
status freezes. Routes 000000bd/000000be: the last poll was at 6.3 s, inside card's VIN + ECU presence + firmware query
(~60 ECUs from 2.0 s); route 000000cb: with no query and nothing else sent, polling ran the whole drive and the app kept
updating. The query runs on every cold boot because CarParamsCache is cleared at every manager start, and the C4
(powered from the OBD port) cold boots on most drives.

The platform is fixed (owner 2026-11-02: always "Toyota Corolla Cross Hybrid (Non-US only) 2020-22" =
TOYOTA_COROLLA_TSS2), so the identity cannot come from the query anyway: the last drive's CarParams
(CarParamsPersistent: same VIN and firmware, incl. the hybrid ECU that sets ToyotaFlags.HYBRID) are handed to opendbc's
fingerprint() as the cache, which then skips the query. FINGERPRINT pins the platform. Only when there is no usable
CarParamsPersistent (very first boot, or another car) does the stock query run once; the result is stored as
CarParamsPersistent for the following boots.
"""
import os

from cereal import car

FIXED_PLATFORM = "TOYOTA_COROLLA_TSS2"  # Toyota Corolla Cross Hybrid (Non-US only) 2020-22


def usable(raw: bytes | None) -> bool:
  if not raw:
    return False
  try:
    with car.CarParams.from_bytes(raw) as cp:
      return cp.carFingerprint == FIXED_PLATFORM and len(cp.carFw) > 0 and len(cp.carVin) == 17
  except Exception:
    return False


def fixed_platform_car_params(params) -> bytes | None:
  """CarParams bytes for fingerprint()'s cache: CarParamsCache (stock), else CarParamsPersistent of the fixed platform."""
  os.environ.setdefault("FINGERPRINT", FIXED_PLATFORM)
  cached = params.get("CarParamsCache")
  if cached is not None:
    return cached
  persistent = params.get("CarParamsPersistent")
  return persistent if usable(persistent) else None
