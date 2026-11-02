from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.toyota.cluster_speed import cluster_speed, CLUSTER_HYST_GAP


def test_altis_dash_points_round_to_the_dash():
  # (true km/h from the logs, dash km/h read by the driver) - routes 000000e9/ea 2026-10-02 and 2026-09-30
  for v, dash in ((103.00, 110), (92.99, 99), (63.96, 69), (98.0, 104)):
    assert round(cluster_speed(v * CV.KPH_TO_MS) * CV.MS_TO_KPH) == dash


def test_crawl_and_standstill_stay_exact():
  for v in (0.0, 1.0, 4.9):
    assert cluster_speed(v * CV.KPH_TO_MS) == v * CV.KPH_TO_MS


def test_hysteresis_is_0_1_kph():
  assert abs(CLUSTER_HYST_GAP * CV.MS_TO_KPH - 0.1) < 1e-9


def test_carstate_uses_it():
  from opendbc.car.toyota.interface import CarInterface
  CP = CarInterface.get_non_essential_params("TOYOTA_COROLLA_TSS2")
  CI = CarInterface(CP)
  assert CI.CS.cluster_speed_hyst_gap == CLUSTER_HYST_GAP
