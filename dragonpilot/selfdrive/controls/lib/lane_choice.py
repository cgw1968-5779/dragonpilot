"""
dragonpilot (dpcr, Corolla Cross Hybrid): lane choice at lane splits, widening turn bays and after the driver's own
steering, 15-70 km/h. Called from lane_centering.apply_lane_centering (no extra controlsd hook).

Owner 2026-11-09: "when crossing an intersection, check quickly which lane the car should enter: aim at the centre of
the lane the car's path points at; once more than 50 % of the car aims into one lane, start moving into that lane. If I
aim the car at a lane by hand while crossing, that lane wins. The same for a turn bay that opens before the
intersection - the car should go straight but often wants to cut into the left-turn lane."
Route 00000000--41925a5692 13:28:00-13:28:07 (off a ramp, the road opens to L, L, S, S): the model's path drifted
0.4 m left (30 m ahead) while the lane widened, before the divider was painted; when the divider appeared the car was
mostly in the left-turn lane and the model kept it there.

Events (per model frame, 20 Hz):
  - RELEASE: the driver steered (steeringPressed >= PRESS_MIN_T) and let go: the lane the car aims at wins;
  - SPLIT:   a divider appears inside the ego lane (its width drops by SPLIT_DROP within SPLIT_T, from >= SPLIT_WIDE):
             the lane the car aims at wins (> 50 % = the car's centre point after T_AIM is inside it);
  - REACQUIRE: both ego lines come back after being lost for >= LOST_T (crossing an intersection): same rule;
  - BAY (OFF, BAY_ENABLED): a left-turn bay opens on the left (Taiwan drives on the right): the LEFT road edge moves out by BAY_OPEN
             within BAY_T while the RIGHT road edge stays (within BAY_RIGHT_TOL) and the model's path heads left. The
             model's own lane lines are no help there - route 41925a5692 13:00:08-13:00:10 (straight road, 35 km/h, no
             steering by the driver): both ego lines bent left with the bay (30 m ahead -1.8 -> -3.7 / +1.3 -> -0.7 m)
             and the car went into the left-turn lane, while the left edge went -2.9 -> -4.6 m (20 m ahead) in 2 s.
             Even the road edges bend left far out there (right edge 40 m ahead +7.0 -> +4.6 m) and the near right
             edge jumps +-0.8 m, so nothing the model draws says "straight". On a road that WAS straight (the car's
             own curvature small for BAY_STRAIGHT_T) the car keeps the straight line it was on when the bay opened,
             by dead reckoning (speed + actual curvature), until BAY_HOLD_T, the driver steers or signals, or below
             V_MIN (the stop line), or the model asks a real curve's lateral accel (BAY_ABORT_ACCEL). A curve (route
             41925a5692 13:15:59, an S-bend whose left edge also moved out) is let go within ~1 s. A road that widens on the RIGHT
             is left alone: 13:28 is that case (the straight lanes grew on the right and the old lane became the turn
             lane) and nothing but the driver's aim (RELEASE) tells which side goes straight there.
The aim point is the car's own path: T_AIM ahead on the arc of its actual curvature (controlsd's, from the steering
angle with the learned offset; same sign as the model's y and curvature), not the model's.
When the aimed lane is the model's own ego lane nothing happens. Otherwise the car is steered by pure pursuit to the
aimed lane's centre (or the held distance to the straight line), added at the jerk limit JERK and capped (MAX_DIFF_ACCEL vs
the model), until the model's ego lane is that lane (AGREE_N frames) - then it hands back. Blinker, lane change,
driver steering, lost lines, out of speed range or HOLD_MAX_T end it at once (the added curvature winds out at JERK).
"""
import numpy as np

V_MIN = 15.0 / 3.6         # m/s
V_MAX = 70.0 / 3.6         # m/s (the lane centering, from 30 km/h, hands back while this is active)
T_AIM = 1.5                # s: the car's aim point (its own path)
X_REF = 10.0               # m: where lanes are compared / tracked
P_LINE = 0.4               # a line counts with at least this probability
LANE_W_RANGE = (2.4, 4.6)  # m: a pair of lines this far apart is a lane
PRESS_MIN_T = 0.3          # s: a driver steering input at least this long is an aim
SPLIT_WIDE = 3.5           # m: a lane at least this wide ...
SPLIT_DROP = 0.8           # m: ... that narrows by this much ...
SPLIT_T = 1.0              # s: ... within this time got a divider
LOST_T = 0.5               # s: ego lines lost this long, then back = a new lane set (intersection)
# OFF (2026-11-09): every rule that caught 13:00 also fired 12 times in 20 min on route 000000f0 (crossing
# intersections, the model's normal sideways moves around parked cars / scooters) and held the car against the model
# with up to 0.8 m/s^2 for 6 s; from the model's outputs alone a turn bay cannot be told from those. Kept for later.
BAY_ENABLED = False
BAY_X = 20.0               # m: where the road edges are watched
BAY_OPEN = 1.0             # m: the left road edge moves out (left) this much ...
BAY_T = 2.0                # s: ... within this time ...
BAY_PATH = -0.3            # m: ... while the model's path 30 m ahead heads left at least this much ...
BAY_STRAIGHT_T = 2.0       # s: ... on a road that was straight this long:
BAY_STRAIGHT_K = 0.0012    # 1/m: mean |actual curvature| over BAY_STRAIGHT_T at most this (R ~830 m), and now 0.001
BAY_EDGE_STD = 1.5         # m: a left road edge estimate this uncertain or worse is not used
BAY_HOLD_T = 6.0           # s: the straight line is kept at most this long
BAY_START_ACCEL = 0.5      # m/s^2: the model's own lateral accel must be under this when the bay opens ...
BAY_ABORT_ACCEL = 0.8      # m/s^2: ... and the hold ends at once when the model asks this much (a real curve or turn:
                           # 13:15:59 asked 2+ m/s^2 within 2 s; drifting into the bay at 13:00 only ~0.3)
LOOK_T = 1.5               # s: pursuit lookahead
LOOK_RANGE = (12.0, 35.0)  # m
MAX_DIFF_ACCEL = 0.8       # m/s^2: the lane choice never asks more lateral accel than this beyond the model
JERK = 1.0                 # m/s^3: rate limit of the added curvature (as lateral jerk)
AGREE_N = 10               # frames the model's ego lane must match the chosen lane before handing back (0.5 s)
AGREE_TOL = 0.6            # m: chosen lane centre within this of the model's ego lane centre = same lane
HOLD_MAX_T = 10.0          # s (lane mode)


def _lines_at(model_v2, x: float):
  ys, ps = [], []
  for ln, p in zip(model_v2.laneLines, model_v2.laneLineProbs, strict=False):
    lx = ln.x
    if len(lx) < 4:
      ys.append(None)
      ps.append(0.0)
      continue
    ys.append(float(np.interp(x, np.asarray(lx, dtype=float), np.asarray(ln.y, dtype=float))))
    ps.append(float(p))
  return ys, ps


def _lanes(ys, ps):
  """the lanes (left, right) between adjacent confident lines, left to right, with their index (1 = model's ego)"""
  out = []
  for i in range(len(ys) - 1):
    if ys[i] is None or ys[i + 1] is None or ps[i] < P_LINE or ps[i + 1] < P_LINE:
      continue
    w = ys[i + 1] - ys[i]
    if LANE_W_RANGE[0] <= w <= LANE_W_RANGE[1]:
      out.append((i, ys[i], ys[i + 1]))
  return out


class LaneChoice:
  def __init__(self, dt: float):
    self.dt = dt
    self.frame = None
    self.active = False
    self.mode = None            # 'lane' (pursue a lane centre) or 'hold' (keep the straight line, dead reckoning)
    self.target = 0.0           # 'lane': chosen lane centre at X_REF (car frame, + right)

    self.t_active = 0.0
    self.agree = 0
    self.k_goal = 0.0           # pursuit curvature (absolute) from the last model frame
    self.out = 0.0              # curvature added to the model's
    self.press_t = 0.0
    self.lost_t = 0.0
    self.width_hist: list[float] = []
    self.edge_hist: list[float] = []  # left road edge at BAY_X per frame
    self.k_hist: list[float] = []     # |actual curvature| per frame
    self.dr_y = 0.0                   # dead reckoning from the moment the bay opened: lateral (+ right), heading
    self.dr_psi = 0.0
    self.events = []            # (event, chosen) for logging / tests

  def reset(self):
    self._end()
    self.out, self.press_t, self.lost_t, self.width_hist, self.edge_hist, self.k_hist = 0.0, 0.0, 0.0, [], [], []

  def _end(self):
    self.active, self.mode, self.agree, self.t_active = False, None, 0, 0.0

  def _bay_check(self, model_v2, k_act, k_model, v):
    n = int(round(BAY_T / 0.05))
    self.k_hist = (self.k_hist + [abs(k_act)])[-int(round(BAY_STRAIGHT_T / 0.05)):]
    re = getattr(model_v2, 'roadEdges', None)
    stds = getattr(model_v2, 'roadEdgeStds', None)
    if re is None or len(re) < 1 or len(re[0].x) < 4 or (stds is not None and len(stds) >= 1 and float(stds[0]) > BAY_EDGE_STD):
      self.edge_hist = []
      return
    e_l = float(np.interp(BAY_X, np.asarray(re[0].x, dtype=float), np.asarray(re[0].y, dtype=float)))
    self.edge_hist = (self.edge_hist + [e_l])[-n:]
    if len(self.edge_hist) < n or len(self.k_hist) < int(round(BAY_STRAIGHT_T / 0.05)):
      return
    px = np.asarray(model_v2.position.x, dtype=float)
    path = float(np.interp(30.0, px, np.asarray(model_v2.position.y, dtype=float))) if len(px) >= 4 else 0.0
    l0 = max(self.edge_hist[: n // 2])   # the left edge's innermost in the first half of the window
    straight = float(np.mean(self.k_hist)) <= BAY_STRAIGHT_K and abs(k_act) <= 0.001
    gentle = abs(k_model) * v * v < BAY_START_ACCEL
    if l0 - e_l >= BAY_OPEN and path <= BAY_PATH and straight and gentle:
      self.active, self.mode, self.agree, self.t_active = True, 'hold', 0, 0.0
      self.dr_y, self.dr_psi = 0.0, 0.0
      self.edge_hist = []
      self.events.append(('bay', 'straight'))

  def _aim_y(self, v: float, k_act: float) -> tuple[float, float]:
    x = max(v * T_AIM, X_REF)
    return x, 0.5 * k_act * x * x

  def _choose(self, model_v2, v, k_act, event):
    x_aim, y_aim = self._aim_y(v, k_act)
    ys, ps = _lines_at(model_v2, x_aim)
    lanes = _lanes(ys, ps)
    for i, yl, yr in lanes:
      if yl <= y_aim <= yr:
        if i == 1:
          self.events.append((event, 'ego'))
          return  # the model's own lane: nothing to do
        ys_ref, ps_ref = _lines_at(model_v2, X_REF)
        if ys_ref[i] is None or ys_ref[i + 1] is None:
          return
        self.active, self.mode, self.agree, self.t_active = True, 'lane', 0, 0.0
        self.target = (ys_ref[i] + ys_ref[i + 1]) / 2.0
        self.events.append((event, 'left' if i < 1 else 'right'))
        return
    self.events.append((event, 'none'))

  def on_frame(self, model_v2, v, k_model, k_act, pressed, blinker, lane_changing):
    """once per model frame"""
    ys, ps = _lines_at(model_v2, X_REF)
    ok_speed = V_MIN <= v <= V_MAX
    ego_ok = len(ys) >= 4 and ys[1] is not None and ys[2] is not None and min(ps[1], ps[2]) >= P_LINE
    width = ys[2] - ys[1] if ego_ok else None
    frame_dt = 0.05

    if not ok_speed or blinker or lane_changing:
      self._end()
      self.press_t, self.lost_t, self.width_hist, self.edge_hist, self.k_hist = 0.0, 0.0, [], [], []
      return
    # driver steering: no lane choice while the hands work; on release the aimed lane wins
    if pressed:
      self.press_t += frame_dt
      self._end()
      return
    if self.press_t >= PRESS_MIN_T:
      self.press_t = 0.0
      self._choose(model_v2, v, k_act, 'release')
    self.press_t = 0.0
    look = min(max(v * LOOK_T, LOOK_RANGE[0]), LOOK_RANGE[1])
    # a left-turn bay opening: the left road edge only (the model's lane lines follow the bay)
    if not self.active and BAY_ENABLED:
      self._bay_check(model_v2, k_act, k_model, v)
    if self.active and self.mode == 'hold':
      self.t_active += frame_dt
      if self.t_active > BAY_HOLD_T or abs(k_model) * v * v >= BAY_ABORT_ACCEL:
        self._end()
        return
      # dead reckoning of the car against the straight line it was on: y + right, heading + right
      self.dr_psi += k_act * v * frame_dt
      self.dr_y += self.dr_psi * v * frame_dt
      self._pursue(-self.dr_y - self.dr_psi * look, look, v, k_model)
      return
    # lost / reacquired ego lines (intersection)
    if not ego_ok:
      self.lost_t += frame_dt
      self.width_hist = []
      if self.active and self.lost_t > 1.0:
        self._end()
      return
    if self.lost_t >= LOST_T and not self.active:
      self.lost_t = 0.0
      self._choose(model_v2, v, k_act, 'reacquire')
    self.lost_t = 0.0
    # a divider appears in a wide lane
    n = int(round(SPLIT_T / frame_dt))
    self.width_hist = (self.width_hist + [width])[-n:]
    if not self.active and max(self.width_hist) >= SPLIT_WIDE and max(self.width_hist) - width >= SPLIT_DROP:
      self.width_hist = [width]
      self._choose(model_v2, v, k_act, 'split')
    if not self.active:
      return
    self.t_active += frame_dt
    if self.t_active > HOLD_MAX_T:
      self._end()
      return
    y_look, p_look = _lines_at(model_v2, look)
    lanes = _lanes(ys, ps)
    if not lanes:
      self._end()
      return
    # follow the chosen lane from frame to frame: the lane whose centre is nearest the last target
    i, yl, yr = min(lanes, key=lambda ln: abs((ln[1] + ln[2]) / 2.0 - self.target))
    c = (yl + yr) / 2.0
    if abs(c - self.target) > 1.0:
      self._end()   # lost track of the chosen lane
      return
    self.target = c
    if i == 1 and abs(c - (ys[1] + ys[2]) / 2.0) < AGREE_TOL:
      self.agree += 1
      if self.agree >= AGREE_N:
        self._end()   # the model's own lane is the chosen one now: hand back
        return
    else:
      self.agree = 0
    if y_look[i] is None or y_look[i + 1] is None:
      self._end()
      return
    self._pursue((y_look[i] + y_look[i + 1]) / 2.0, look, v, k_model)

  def _pursue(self, goal, look, v, k_model):
    """pure pursuit of the point goal metres to the side, look metres ahead; capped against the model's curvature"""
    k = 2.0 * goal / (look * look + goal * goal)
    lim = MAX_DIFF_ACCEL / max(v, V_MIN) ** 2
    self.k_goal = k_model + float(np.clip(k - k_model, -lim, lim))

  def update(self, model_v2, v_ego, k_model, k_act, pressed, blinker, lane_changing) -> float:
    """curvature to add to the model's (100 Hz); k_act: the car's actual curvature. The model-frame logic runs once
    per frame."""
    frame = getattr(model_v2, 'frameId', None) if model_v2 is not None else None
    if model_v2 is not None and frame != self.frame:
      self.frame = frame
      self.on_frame(model_v2, v_ego, k_model, k_act, pressed, blinker, lane_changing)
    # in at the jerk limit, back out the same way (no snap when it ends)
    want = self.k_goal - k_model if self.active else 0.0
    step = JERK / max(v_ego, V_MIN) ** 2 * self.dt
    self.out = float(np.clip(want, self.out - step, self.out + step))
    return self.out
