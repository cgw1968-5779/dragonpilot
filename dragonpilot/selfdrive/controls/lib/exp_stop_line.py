"""
dragonpilot (dppb, Corolla Altis): experimental mode only - stop before the first white stop line ahead, at the spot
the owner stops at by hand (owner 2026-11-08: "experimental mode on dppb often crosses the white stop line at lights;
stop before the line nearest the car, where I stopped by hand at 08:28 (two bookmarks)").

Route 00000194 (C3X on cb4676d6):
- 08:24:38 (engaged, experimental, no lead, red light, bookmark): the model's planned stop point (modelV2 position
  where its planned speed drops under 0.3 m/s) was seen 25 m out; the plan braked -1.3..-1.4 but the car reached that
  point still at ~6 km/h, slowed to 0.8 km/h and then CREPT at 1-2 km/h for 5 s (planner -0.05..-0.1, no stopped lead
  for the stop end hold) and ended 5.6 m past the model's stop point, on the crosswalk.
- 08:25:45 and 08:28:34 (owner's manual stops before the line, 08:28 with two bookmarks): the stop point kept as below
  (median of 10 model frames, frozen within 8 m) sat 1.22 / 1.26 m beyond where the owner stopped.

While experimental mode is on, no real lead is closer and the model plans a stop within MAX_X, the stop point is kept
as a position on the car's own odometer (median of the last MED_N model frames, frozen once within FREEZE_D - close in
the model re-plans toward the car) and given to the MPC and to the stop end (LongControl) as a virtual stopped lead,
placed so the stop that normally ends END_GAP behind a stopped lead ends LINE_MARGIN before the model's stop point.
The MPC and the stop end are unchanged; they only see this lead. Experimental off, no planned stop, or a real radar
lead closer than LEAD_NEAR beyond the virtual one: the real lead passes through unchanged. The last because the
model also plans its stop ~5 m behind a stopped car (08:30:00: 5.2 m), which as a line would end the stop ~6 m behind
the car instead of the Altis's 4.0 m standard; scooters waiting just past the line (08:28: their rear 4.7 m beyond the
model's point) look the same, so there the radar lead (4.0 m behind them) is kept, as today.
"""
from types import SimpleNamespace

import numpy as np

MAX_X = 80.0        # m: a planned stop further than this is not used
STOP_V = 0.3        # m/s: the model's planned speed under this marks its stop point
MED_N = 10          # model frames (0.5 s) in the median of the stop position
FREEZE_D = 8.0      # m: closer than this the stop position is frozen
END_GAP = 3.5       # m (radar dRel): where a stop behind a stopped lead ends on dppb (longcontrol STOP_LEVEL_GAP, owner's 3.5 m since 11-12, was 4.0)
LINE_MARGIN = 1.2   # m: aim this far before the model's stop point (owner's manual stops 08:25:45 / 08:28:34: 1.22 / 1.26 m)
LOST_N = 20         # model frames without a planned stop before the virtual lead is dropped (1 s)
LEAD_NEAR = 3.0     # m: a real lead less than this beyond the virtual one is what the model stops for -> the real lead


class ExpStopLine:
  def __init__(self, dt: float):
    self.dt = dt
    self.odo = 0.0
    self.hist: list[float] = []
    self.stop_pos: float | None = None
    self.lost = 0
    self.frame = None

  @staticmethod
  def _model_stop_x(model_v2) -> float | None:
    vx, px = model_v2.velocity.x, model_v2.position.x
    for i in range(min(len(vx), len(px))):
      if vx[i] < STOP_V:
        return float(px[i]) if px[i] <= MAX_X else None
    return None

  def lead(self, sm, v_ego: float, real_lead):
    """the lead for the MPC / the stop end: the real radar lead, or in experimental mode a virtual stopped lead at the
    model's stop line"""
    self.odo += max(v_ego, 0.0) * self.dt
    if not sm['selfdriveState'].experimentalMode:
      self.hist, self.stop_pos, self.lost, self.frame = [], None, 0, None
      return real_lead
    md = sm['modelV2']
    frame = getattr(md, 'frameId', None)
    if frame != self.frame:
      self.frame = frame
      x = self._model_stop_x(md)
      if x is None:
        self.lost += 1
        if self.lost >= LOST_N:
          self.hist, self.stop_pos = [], None
      else:
        self.lost = 0
        pos = self.odo + x
        if self.stop_pos is None or self.stop_pos - self.odo > FREEZE_D:
          self.hist = (self.hist + [pos])[-MED_N:]
          self.stop_pos = float(np.median(self.hist))
    if self.stop_pos is None:
      return real_lead
    d_virtual = self.stop_pos - self.odo + END_GAP - LINE_MARGIN
    if real_lead is not None and real_lead.status and real_lead.dRel < d_virtual + LEAD_NEAR:
      return real_lead
    return SimpleNamespace(status=True, dRel=max(d_virtual, 0.1), vLead=0.0, vLeadK=0.0, aLeadK=0.0, aLeadTau=0.3,
                           modelProb=0.0, radar=False)

  def radar_state(self, sm, v_ego: float):
    """radarState for the MPC with leadOne replaced by lead()"""
    rs = sm['radarState']
    lead_one = self.lead(sm, v_ego, rs.leadOne)
    return rs if lead_one is rs.leadOne else SimpleNamespace(leadOne=lead_one, leadTwo=rs.leadTwo)
