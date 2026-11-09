"""
dragonpilot (dpcr, Corolla Cross): experimental mode only - stop at the first white stop line ahead, never past it and
not more than ~2 m short of it (owner 2026-11-08: "tonight's first experimental stop crossed the line; stop inside the
first white stop line in front of the car, never across it and not more than two metres back from it").

Route 000000ed 23:05:12 (experimental, no lead, red light): the model's planned stop point (modelV2 position where its
planned speed drops under 0.3 m/s), seen from 25-10 m out, sat 3.3-5.0 m before where the car finally stopped. Below
10 km/h the plan asked -0.97..-0.5 but the end limit sent only -0.69..-0.37 and the car delivered -0.2..-0.4, so it
crawled at 4-6 km/h for ~4 s / 8 m and rolled ~4 m past the model's stop point. The end limit is never exempted
(owner rule 0), so the stop has to be ARRANGED earlier, not finished harder.

While experimental mode is on, no real lead is closer, and the model plans a stop within MAX_X: the stop point is kept
as a position on the car's own odometer (median of the last MED_N model frames, frozen once within FREEZE_D - close in,
the model re-plans toward the car), and handed to the stop target (stop_approach.py) as a virtual stopped lead placed
so the car aims LINE_MARGIN before it. Everything else (experimental off, a real lead, no planned stop) passes the
real radar lead through unchanged.

Owner 2026-11-09 (route 00000000--41925a5692 12:57, "experimental stops still look random: it stopped too early, I gave
it gas to move up, then it stopped too late, past the white line"): the stop point frozen at 5.8 km/h was ~8-10 m short
of the line (the model itself slowed early beside a scooter); the owner pushed the car forward with the gas twice, and
after the second push the frozen point was already behind the car, the virtual lead sat at 0.1 m and nothing braked
(stop_approach treated it as urgent and handed back to the MPC, which never sees this virtual lead) - the car crept over
the line at 4.5 km/h until the owner braked. Now:
  - the driver's gas is the driver correcting where to stop: it clears the remembered stop point, and after the pedal
    is released the model's stop point is learned again from where the car is;
  - the virtual lead is marked (virtual=True) so stop_approach never hands it to the MPC and, once the car is at or past
    the aim, stops it on the end line.
"""
from types import SimpleNamespace

import numpy as np

from dragonpilot.selfdrive.controls.lib.stop_approach import STOP_GAP

MAX_X = 80.0        # m: a planned stop further than this is not used
STOP_V = 0.3        # m/s: the model's planned speed under this marks its stop point
MED_N = 10          # model frames (0.5 s) in the median of the stop position
FREEZE_D = 8.0      # m: closer than this the stop position is frozen
LINE_MARGIN = 2.0   # m: aim this far before the model's stop point (sim ed 23:05: 1.5 -> 2.0 m short; real stops end ~1 m closer than the sim, and tonight two stops crept 0.7 m after stopping)
LOST_N = 20         # frames without a planned stop before the virtual lead is dropped (1 s)


class ExpStopLine:
  def __init__(self, dt: float):
    self.dt = dt
    self.odo = 0.0
    self.hist: list[float] = []
    self.stop_pos: float | None = None
    self.lost = 0
    self.frame = None

  def _model_stop_x(self, model_v2) -> float | None:
    vx, px = model_v2.velocity.x, model_v2.position.x
    n = min(len(vx), len(px))
    for i in range(n):
      if vx[i] < STOP_V:
        return float(px[i]) if px[i] <= MAX_X else None
    return None

  def lead(self, sm, v_ego: float, real_lead):
    """the lead handed to the stop target: the real radar lead, or in experimental mode a virtual stopped lead at
    the model's stop line"""
    self.odo += max(v_ego, 0.0) * self.dt
    exp = sm['selfdriveState'].experimentalMode
    md = sm['modelV2']
    try:
      gas = bool(sm['carState'].gasPressed)
    except KeyError:
      gas = False
    if not exp or gas:
      self.hist, self.stop_pos, self.lost = [], None, 0
      return real_lead
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
    d_virtual = self.stop_pos - self.odo + STOP_GAP - LINE_MARGIN
    if real_lead is not None and real_lead.status and real_lead.dRel < d_virtual:
      return real_lead
    return SimpleNamespace(status=True, dRel=max(d_virtual, 0.1), vLead=0.0, virtual=True)
