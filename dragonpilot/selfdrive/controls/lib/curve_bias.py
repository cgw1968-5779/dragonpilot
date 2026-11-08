"""
dragonpilot (dppb, Corolla Altis): curve outward bias (FRAC), the dpcr / tncr1 method (dpcr curve_bias.py, 3ab18ca5)
with the Altis's own schedule from dppb drives (owner 2026-11-08: "low-speed centring on dppb, strengthened like dpcr").

dppb rlogs 11-02..11-08 (10 dppb builds, ALKA/engaged, no steering touch, both lane lines > 0.7; lane centre at 8 m
less the curve's own bend): straights centred (-0.02..+0.03 m); curves (|k| > 0.002) at 25-55 km/h centred
(median -0.08..+0.02 m), 55-65 km/h 0.16 m INSIDE (n 211, both directions), 65-75 km/h 0.30 m inside, 75-90 km/h
0.10 m inside. Steady state of the same model pull-back as the dpcr sim (dy = FRAC k TM^2 v^2 / 2, TM 2 s) at the
measured curves: 50 km/h 0.00, 63 km/h 0.09, 69 km/h 0.14, 83 km/h 0.04. So no bias up to 50 km/h (already centred -
the dpcr 0.13 there would push those curves outward), 0.08 / 0.13 at 60 / 70 km/h, 0.06 at 80, none from 90 km/h
(highway lateral left as it is). Applied to the model's desired curvature in controlsd (one hook line), before the
stock curvature clip, only while lateral is active.
"""
import numpy as np

from openpilot.common.constants import CV

CURVE_OUTWARD_DEADZONE = 0.0005  # 1/m (~radius 2000 m): straights and small lane corrections untouched
CURVE_OUTWARD_V_BP = [v * CV.KPH_TO_MS for v in (50., 60., 70., 80., 90.)]   # m/s
CURVE_OUTWARD_FRAC_V = [0.0, 0.08, 0.13, 0.06, 0.0]


def apply_curve_outward_bias(desired_curvature: float, v_ego: float) -> float:
  # in a 55-85 km/h curve, command a slightly smaller curvature so the car runs wider and stops cutting the inside
  frac = float(np.interp(v_ego, CURVE_OUTWARD_V_BP, CURVE_OUTWARD_FRAC_V))
  if frac == 0.0 or abs(desired_curvature) <= CURVE_OUTWARD_DEADZONE:
    return desired_curvature
  return desired_curvature * (1.0 - frac)
