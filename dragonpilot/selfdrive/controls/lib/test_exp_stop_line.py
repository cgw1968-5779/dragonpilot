from types import SimpleNamespace as N

from dragonpilot.selfdrive.controls.lib import exp_stop_line as XL

DT = 0.01


def model(frame, stop_x):
  xs = [i * 0.5 for i in range(200)]
  vs = [8.0 if (stop_x is None or x < stop_x) else 0.0 for x in xs]
  return N(frameId=frame, position=N(x=xs), velocity=N(x=vs))


REAL = N(status=False, dRel=0.0, vLead=0.0)
TWO = N(status=False, dRel=0.0, vLead=0.0)


def sm(exp, frame, stop_x, lead=REAL):
  return {'selfdriveState': N(experimentalMode=exp), 'modelV2': model(frame, stop_x), 'radarState': N(leadOne=lead, leadTwo=TWO)}


def test_experimental_off_passes_the_radar_state_through_unchanged():
  xl = XL.ExpStopLine(DT)
  for i in range(200):
    s = sm(False, i // 5, 40.0)
    assert xl.lead(s, 10.0, REAL) is REAL
    assert xl.radar_state(s, 10.0) is s['radarState']


def test_experimental_without_a_planned_stop_is_unchanged():
  xl = XL.ExpStopLine(DT)
  for i in range(100):
    s = sm(True, i // 5, None)
    assert xl.radar_state(s, 10.0) is s['radarState']


def test_virtual_lead_at_the_model_stop_line():
  xl = XL.ExpStopLine(DT)
  for i in range(50):
    rs = xl.radar_state(sm(True, i // 5, 40.0), 0.0)  # standing still: odometer fixed
  ld = rs.leadOne
  assert ld.status and ld.vLead == 0.0 and ld.aLeadK == 0.0 and ld.modelProb == 0.0  # no FCW from a virtual lead
  assert abs(ld.dRel - (40.0 + XL.END_GAP - XL.LINE_MARGIN)) < 1e-6
  assert rs.leadTwo is TWO


def test_stop_point_frozen_close_in():
  xl = XL.ExpStopLine(DT)
  v, odo = 5.0, 0.0
  for i in range(400):
    stop_x = max(30.0 - odo, 0.5) + (2.0 if 30.0 - odo < XL.FREEZE_D else 0.0)  # model drifts 2 m ahead close in
    ld = xl.lead(sm(True, i, stop_x), v, REAL)
    odo += v * DT
    if 30.0 - odo < 3.0:
      break
  assert abs(ld.dRel - (30.0 - odo + XL.END_GAP - XL.LINE_MARGIN)) < 0.6  # 0.5 m model grid here


def test_a_closer_real_lead_wins_and_no_stop_drops_it():
  xl = XL.ExpStopLine(DT)
  real = N(status=True, dRel=10.0, vLead=0.0)
  assert xl.lead(sm(True, 0, 40.0), 0.0, real) is real
  for i in range(1, 30):
    ld = xl.lead(sm(True, i, None), 0.0, REAL)
  assert ld is REAL


def test_none_lead_from_controlsd():
  xl = XL.ExpStopLine(DT)
  assert xl.lead(sm(False, 0, 40.0), 0.0, None) is None
  ld = None
  for i in range(20):
    ld = xl.lead(sm(True, i, 40.0), 0.0, None)
  assert ld.status


def test_a_real_lead_just_beyond_the_line_wins():
  # the model plans its stop ~5 m behind a stopped car: the car's own 4.0 m stop, not a line 2 m further back
  xl = XL.ExpStopLine(DT)
  real = N(status=True, dRel=40.0 + XL.END_GAP - XL.LINE_MARGIN + XL.LEAD_NEAR - 0.1, vLead=0.0)
  for i in range(20):
    ld = xl.lead(sm(True, i, 40.0), 0.0, real)
  assert ld is real
  real.dRel += 0.2
  assert xl.lead(sm(True, 21, 40.0), 0.0, real) is not real
