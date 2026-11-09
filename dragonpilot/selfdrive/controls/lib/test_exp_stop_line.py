from types import SimpleNamespace as N

from dragonpilot.selfdrive.controls.lib import exp_stop_line as XL
from dragonpilot.selfdrive.controls.lib.stop_approach import STOP_GAP

DT = 0.01


def model(frame, stop_x):
  xs = [i * 0.5 for i in range(200)]
  vs = [8.0 if (stop_x is None or x < stop_x) else 0.0 for x in xs]
  return N(frameId=frame, position=N(x=xs), velocity=N(x=vs))


def sm(exp, frame, stop_x):
  return {'selfdriveState': N(experimentalMode=exp), 'modelV2': model(frame, stop_x)}


REAL = N(status=False, dRel=0.0, vLead=0.0)


def test_experimental_off_passes_the_radar_lead_through():
  xl = XL.ExpStopLine(DT)
  for i in range(200):
    assert xl.lead(sm(False, i // 5, 40.0), 10.0, REAL) is REAL


def test_virtual_lead_at_the_model_stop_line():
  xl = XL.ExpStopLine(DT)
  ld = None
  for i in range(50):
    ld = xl.lead(sm(True, i // 5, 40.0), 0.0, REAL)  # standing still: odometer fixed
  assert ld.status and ld.vLead == 0.0
  assert abs(ld.dRel - (40.0 + STOP_GAP - XL.LINE_MARGIN)) < 1e-6


def test_stop_point_frozen_close_in():
  # the model re-plans its stop toward the car close in: the stop position is frozen inside FREEZE_D
  xl = XL.ExpStopLine(DT)
  v, odo = 5.0, 0.0
  for i in range(400):
    stop_x = max(30.0 - odo, 0.5) + (2.0 if 30.0 - odo < XL.FREEZE_D else 0.0)  # model drifts 2 m ahead close in
    ld = xl.lead(sm(True, i, stop_x), v, REAL)
    odo += v * DT
    if 30.0 - odo < 3.0:
      break
  assert abs(ld.dRel - (30.0 - odo + STOP_GAP - XL.LINE_MARGIN)) < 0.6  # 0.5 m model grid here


def test_a_closer_real_lead_wins_and_no_stop_drops_it():
  xl = XL.ExpStopLine(DT)
  real = N(status=True, dRel=10.0, vLead=0.0)
  assert xl.lead(sm(True, 0, 40.0), 0.0, real) is real
  for i in range(1, 30):
    ld = xl.lead(sm(True, i, None), 0.0, REAL)
  assert ld is REAL


def test_gas_clears_the_stop_point_and_relearns_after_release():
  # owner 2026-11-09 12:57: the driver's gas moves the car up; after release the stop point is the model's new one
  xl = XL.ExpStopLine(DT)
  for i in range(50):
    xl.lead(sm(True, i // 5, 5.0), 0.0, REAL)
  assert xl.stop_pos is not None
  s = sm(True, 100, 5.0)
  s['carState'] = N(gasPressed=True)
  assert xl.lead(s, 1.0, REAL) is REAL and xl.stop_pos is None
  ld = None
  for i in range(10):
    s = sm(True, 200 + i, 1.0)
    s['carState'] = N(gasPressed=False)
    ld = xl.lead(s, 0.0, REAL)
  assert ld.virtual and abs(ld.dRel - max(1.0 + STOP_GAP - XL.LINE_MARGIN, 0.1)) < 1e-6
