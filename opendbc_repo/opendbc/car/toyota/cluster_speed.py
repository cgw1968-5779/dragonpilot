# dragonpilot: on-screen speed matched to the car's own speedometer.
# Ported from the owner's sunnypilot branches tnpb3 / tncr1 (opendbc/sunnypilot/car/toyota/carstate_ext.py,
# cluster_speed_sp + cluster_hyst_gap_sp; both branches use the same constants).
#
# Stock: vEgo * 1.015. Corolla Altis Hybrid 2026-09-30 22:30: at a set speed of 105 the dash read 104 while the screen
# (1.015 * vEgo) read 100, i.e. the dash is ~5% above the true speed. Fitted 2026-10-02 to the Altis dash at steady
# cruise (true speed from the logs, dash read by the driver): v 103.00 -> dash 110, 92.99 -> 99, 63.96 -> 69
# (routes 000000e9/ea), 98.0 -> 104 (09-30). 1.05 v + 1.47 rounds to every one of them (margin 0.12 km/h).
# Display hysteresis 0.1 km/h (stock 0.5): steady-cruise vEgo noise is ~+-0.08 km/h, and 0.5 kept the value up to
# 0.5 below after accelerating to the set speed. Below CLUSTER_MIN_KPH the screen shows the true speed (crawl and
# standstill stay exact). Display only: vEgo, the planner and the cruise target are untouched.
from opendbc.car.common.conversions import Conversions as CV

CLUSTER_SPEED_GAIN = 1.05        # dash km/h per true km/h (stock openpilot: 1.015)
CLUSTER_SPEED_OFFSET_KPH = 1.47  # km/h added on top
CLUSTER_HYST_KPH = 0.1           # display hysteresis (stock 0.5)
CLUSTER_MIN_KPH = 5.0            # below this the screen shows the true speed


def cluster_speed(v_ego: float) -> float:
  """Speed for the on-screen readout, matched to the car's own speedometer (m/s in, m/s out)."""
  v_kph = v_ego * CV.MS_TO_KPH
  if v_kph < CLUSTER_MIN_KPH:
    return v_ego
  return (CLUSTER_SPEED_GAIN * v_kph + CLUSTER_SPEED_OFFSET_KPH) * CV.KPH_TO_MS


CLUSTER_HYST_GAP = CLUSTER_HYST_KPH * CV.KPH_TO_MS  # m/s
