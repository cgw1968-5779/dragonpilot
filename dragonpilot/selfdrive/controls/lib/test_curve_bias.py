from dragonpilot.selfdrive.controls.lib.curve_bias import apply_curve_outward_bias, CURVE_OUTWARD_DEADZONE

KPH = 1 / 3.6


def test_tncr1_values():
  # owner 2026-11-09: 60 / 70 km/h 0.22 / 0.17 (11-07: 0.16 / 0.13; 50 km/h 0.13 unchanged)
  for kph, frac in ((30, 0.0), (40, 0.065), (50, 0.13), (55, 0.175), (60, 0.22), (65, 0.195), (70, 0.17), (80, 0.1625), (90, 0.155), (130, 0.155)):
    for k in (0.002, -0.002):
      assert abs(apply_curve_outward_bias(k, kph * KPH) - k * (1 - frac)) < 1e-12


def test_straights_untouched():
  for k in (0.0, CURVE_OUTWARD_DEADZONE, -CURVE_OUTWARD_DEADZONE / 2):
    assert apply_curve_outward_bias(k, 80 * KPH) == k
