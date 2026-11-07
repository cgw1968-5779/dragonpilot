"""
dragonpilot (dpcr, Corolla Cross Hybrid): curve outward bias (FRAC), ported unchanged from the owner's sunnypilot branch
tncr1 (sunnypilot/selfdrive/controls/lib/lateral_path/curve_bias.py, the values tncr1 runs today, 636f66c3df).
Owner 2026-11-03 (dpcr route 000000d2, a gentle left curve at 65 km/h): "corners clearly sit on the inside; port tncr1's FRAC".
Applied to the model's desired curvature in controlsd (one hook line), before the stock curvature clip.
"""
import numpy as np

from openpilot.common.constants import CV

# Corner-cutting fix (Altis rlog 2026-09-20: the e2e path sits ~0.6 m inside on curves, both directions). Relax the
# commanded curvature by a fraction so the car runs a little wider (outward) in proportion to how tight the curve is.
# Straights and small lane corrections below the deadzone are untouched. Schedule ported from tncr18 (Corolla Cross
# road tests 2026-09-23..25): the outward bias only helps at higher speed, where the path cuts the inside. On tight
# LOW-speed curves the car tends to run WIDE instead, so relaxing the curvature there makes it worse.
CURVE_OUTWARD_DEADZONE = 0.0005  # 1/m (~radius 2000 m): below this is a straight / small correction, untouched.
# Speed schedule (m/s -> fraction): 0 up to 40 km/h, 0.09 at 70, 0.155 at 90 km/h, 0.19 at 120 km/h.
# tncr, owner 2026-10-15 night: "port tncr19's FRAC values and the other lateral corrections" - tncr was cut with the
# Altis (tnpb3) lateral values; the Corolla Cross keeps its own road-tested tune (tncr18 -> tncr19 693947f88b):
# -0.05 at 30 km/h (inward), 0.02 at 40, 0.06 at 50, 0.085 at 60, 0.09 at 70, flat 0.155 from 90 km/h.
# (Altis/tnpb3 values were -0.01 @30, 0 @35, 0.03 @40, 0.063 @50, 0.087 @60.)
# dpcr, owner 2026-11-05 night: the 40 km/h point removed, 30 -> 50 km/h interpolated (0.04 at 40 km/h, was 0.05).
CURVE_OUTWARD_V_BP = [v * CV.KPH_TO_MS for v in (30., 50., 60., 70., 90., 100., 120.)]   # m/s
# tncr, owner 2026-10-27 (route b8 mountain ALKA, signs re-checked): in 30-60 km/h curves the car sits 0.25-0.4 m toward
# the INSIDE (straights centred) and the owner's own drive steered outward in 27 of 32 curves to keep the centre, so
# the low-speed inward bias goes: 30 km/h -0.05 -> 0, 40 km/h 0.02 -> 0.05 (50 km/h and up unchanged).
# dpcr, owner 2026-11-03 night (after the first road test of these values: "FRAC clearly improved curve centring"):
# 60 and 70 km/h +0.02 (0.085 -> 0.105, 0.09 -> 0.11); then 60 km/h +0.005 more (0.105 -> 0.11).
# dpcr, owner 2026-11-05 night: 50 km/h +0.02 (0.06 -> 0.08).
# dpcr, owner 2026-11-07 (route 000000f2 23:41-23:42, ALKA: right curve R 120-190 m at 50-68 km/h, left curve R 70-80 m
# at 46 km/h, "close to / riding the inside line - centre it"): f2 segs 7-9 sat 0.4-0.7 m inside, the 11-03..11-06 routes
# 0.1-0.6 m inside in 40-70 km/h curves. 50 / 60 / 70 km/h 0.08 / 0.11 / 0.11 -> 0.13 / 0.16 / 0.13. Closed-loop
# perturbation sim (routes d3 / e3 / e4 rlog, model pull-back 1.5-2.5 s): 55-70 km/h curves 6-30 cm further out, 40-55
# km/h 3-11 cm, straights and command jerk unchanged (p95 3.6 -> 3.5). Lane-line feedback (the PD centering or line mode)
# below 70 km/h was tried first: in curves the model's near and far lane estimates disagree and both pushed INWARD.
CURVE_OUTWARD_FRAC_V = [0.0, 0.13, 0.16, 0.13, 0.155, 0.155, 0.155]


def apply_curve_outward_bias(desired_curvature: float, v_ego: float) -> float:
  # In a fast curve, command a slightly smaller curvature so the car runs wider and stops cutting the inside. Faded
  # out at low speed (tight curves there need the full turn-in or the car runs wide). Straights and small lane
  # corrections below the deadzone are untouched.
  frac = float(np.interp(v_ego, CURVE_OUTWARD_V_BP, CURVE_OUTWARD_FRAC_V))
  if frac == 0.0 or abs(desired_curvature) <= CURVE_OUTWARD_DEADZONE:
    return desired_curvature
  return desired_curvature * (1.0 - frac)   # frac < 0 scales the curvature UP (inward)
