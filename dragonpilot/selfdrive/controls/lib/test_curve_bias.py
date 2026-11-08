from openpilot.common.constants import CV
from dragonpilot.selfdrive.controls.lib.curve_bias import apply_curve_outward_bias as F, CURVE_OUTWARD_DEADZONE


def test_untouched_at_and_below_50_and_from_90_kmh_and_on_straights():
  for kmh in (0, 20, 40, 50, 90, 110):
    assert F(0.004, kmh * CV.KPH_TO_MS) == 0.004
  assert F(CURVE_OUTWARD_DEADZONE * 0.9, 65 * CV.KPH_TO_MS) == CURVE_OUTWARD_DEADZONE * 0.9


def test_outward_in_55_85_kmh_curves_both_directions():
  assert abs(F(0.004, 60 * CV.KPH_TO_MS) - 0.004 * 0.92) < 1e-9
  assert abs(F(-0.004, 70 * CV.KPH_TO_MS) + 0.004 * 0.87) < 1e-9
  assert abs(F(0.004, 80 * CV.KPH_TO_MS) - 0.004 * 0.94) < 1e-9
