"""
dragonpilot (dpcr, Corolla Cross Hybrid): highway lane centering, ported from the owner's sunnypilot branch tncr1
(sunnypilot/selfdrive/controls/lib/lateral_path/lane_centering.py, class LaneCentering with its line mode, the values
tncr1 runs today). Owner 2026-11-05 (dpcr routes 000000e0/e1, 21:53, 91 km/h left curve sitting ~17 cm right of the
lane centre, up to 33 cm; 85-100 km/h curves on average ~9 cm right while straights are centred): "do it, make the
centring as strong as possible". dpcr had no lane-line feedback at all (stock dp lateral + the tncr1 curve outward
bias), so the model's own curve offset was never corrected; the learners (calibration, steer ratio, torque, delay) were
all complete and cannot fix a curve-only offset.

Not ported: tncr1's straight mode (its faster straight-road filter/dead band/jerk) - the normal values below are used
at all times - and the low-speed trim / low-speed centering (below 70 km/h dpcr keeps the model as is).
Applied in controlsd after the curve outward bias (one hook line), before the stock curvature clip.
"""
import math

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter

from dragonpilot.selfdrive.controls.lib.lane_choice import LaneChoice


def blend(normal: float, straight: float, w: float) -> float:
  return normal + (straight - normal) * w


def _clip(x, lo, hi) -> float:
  # scalar clip: np.clip on a single number costs ~4 us per call (6 per 100 Hz cycle here); same result
  return float(min(max(x, lo), hi))


# tncr1 straight mode (not ported: straight weight is always 0, these are only the blend's other end)
STRAIGHT_CENTER_DEADBAND = 0.015  # m
STRAIGHT_CENTER_TAU = 0.25       # s
STRAIGHT_CENTER_JERK = 1.0       # m/s^3

# Lane centering feedback, ported from tncr18 (Corolla Cross) on 2026-09-30 for the Altis: rlog 2026-09-27 (6 routes,
# Macrostiff) above 100 km/h the car sat 0.10 m left of the lane centre on average with a 0.23 m std (22% of the time more
# than 0.3 m off; 80-100 km/h: 4%), while the torque loop tracked the model's curvature closely (0.73 vs 0.84 e-3 std) -
# the wandering is the model path drifting, so close the loop on the lane lines.
# Original tncr18 note (route 4d seg 9): in a 70 m-radius curve at 47 km/h the car sat
# ~1 m inside the lane (riding the inner line) although the torque loop tracked the model's desired curvature almost
# exactly (14.2 vs 14.4 e-3). The e2e action itself holds the car inside, and the outward bias above is a blind
# percentage. Close the loop on the lane lines instead: measure the car's offset from the lane centre and add a small
# curvature correction that walks it back to the middle. Conventions (verified on seg 1): model y is positive to the
# RIGHT and curvature is positive to the RIGHT, so a car left of centre ((yl + yr) / 2 > 0) needs a positive
# correction. The correction is a lateral-acceleration request, so the same gain gives the same feel at any speed.
# PD on offset + lane heading (damping), plus a slow capped integral for what the model keeps pulling. Simulated
# (bicycle model, 0.3 s steering lag, 2 cm line noise): 1 m inside at 47 km/h in a 70 m curve is back within
# 10 cm in ~8 s with no overshoot, peak correction 1.9e-3 (0.32 m/s^2); 20 cm at 90 km/h in ~10 s.
LANE_CENTER_MIN_SPEED = 8.0        # m/s (~30 km/h): below this the lines are too close / too curved to trust
# Only at highway speed. Route 55 seg 8 (2026-09-26, 33-47 km/h right after the driver released the wheel out of a
# turn): the correction sat on its cap (0.6 m/s^2 = 3.5e-3 1/m at 12 m/s, ~30% of the model's own swing) in phase
# with the model's correction, and the car swung left/right with a ~3 s period for 10 s. The driver wants low speed
# left exactly as stock; the feedback fades in between 70 and 80 km/h.
# dpcr (owner 2026-11-10, "what else can improve the low-speed centring" -> "do it"): from 30 km/h, but on STRAIGHTS only
# below 70 km/h (see LOW_STRAIGHT_*). Cross cmd -> yaw delay measured with the gyro 0.54 s (Altis 0.40-0.44 s): the
# Altis's full 30 km/h centering (dppb c4d851b9) swayed in the Cross sim once the delay reached 0.65 s and pushed the
# curves 7-24 -> 14-51 cm inside (the line pursuit takes over the curve_bias FRAC). Gated to straights, closed-loop sim
# (routes 11-09 + d3/e3/e4, delay 0.55-0.74 s, model pull-back 1.5/2.5 s): straights 30-40 km/h +-22.3 -> 20.0 cm,
# 40-55 +-18.9 -> 15.6, 55-70 +-14.2 -> 12.3; curves within ~4 cm of before; command jerk p95 3.17 -> 2.79; no sway at
# any delay tried. Above 70 km/h unchanged.
LANE_CENTER_FADE_BP = [30.0 / 3.6, 40.0 / 3.6]  # m/s (30, 40 km/h); was 70, 80 km/h
LANE_CENTER_FADE_V = [0.0, 1.0]
LANE_CENTER_MIN_LINE_PROB = 0.5    # the better of the two near lane lines must be at least this confident
LANE_CENTER_MIN_OTHER_PROB = 0.4   # ...and the weaker one at least this. 0.3 let a lost outer line (prob 0.31-0.41, route 58
                                   # seg 6) put the lane centre 1 m off while the car was actually centred; in the
                                   # inside-cutting curves the outer line sits at 0.41-0.61
LANE_CENTER_WIDTH_CONF_PROB = 0.6  # both lines at least this to teach the width filter
LANE_CENTER_WIDTH_TAU = 3.0        # s; filter of the lane width, fed only by confident frames
LANE_CENTER_WIDTH_TOL = 0.7        # m; a width this far from the filtered one means one line is not the ego lane's
LANE_CENTER_WIDTH_RANGE = (2.6, 4.6)  # m; outside this the pair is not the ego lane
LANE_CENTER_FIT_RANGE = 12.0       # m ahead used to fit the lane centre (offset + heading + curvature)
LANE_CENTER_DEADBAND = 0.03        # m; tncr19 Cross value (Altis 0.015); no position correction inside this (0.03 -> 0.015, 2026-09-30: the driver felt the
                                   # ... -> 0.0075 2026-10-04: route 000000f2 10:34 at 82-88 km/h he felt the corrections again
                                   # and asked for a smaller dead band once more. Note: the offset there wandered -20..+15 cm
                                   # with a 4-6 s rhythm, far outside any dead band, so the next lever is the gain, not this.
                                   # corrections at 93 km/h as a regular drift-correct rhythm (~4 s, +-1.3 deg of steering) and
                                   # asked for smaller, more continuous ones; a narrower dead band starts them earlier and smaller)
# 2026-10-06: back to 0.015 (from 0.0075) with the highway smoothing below; recomputed on the highway logs the smaller dead
# band added only ~10% more centering correction, and the driver felt more side-to-side motion on tnpb2.
LANE_CENTER_KP = 0.35              # m/s^2 of lateral accel per metre of offset
# 1.20 -> 0.90 2026-10-08 (owner: "it centres, but the corrections are a little large - as small as possible, but held
# firmly in the centre"). Open-loop over routes 00000110 + 00000114 (> 75 km/h straights): the correction splits into
# position 0.018, DAMPING 0.043, integral 0.029 m/s^2 (std); the position dead band (0-1.5 cm) changes nothing (p95
# 0.090-0.094). The firm hold is position + integral, the visible activity mostly damping: KD 0.9 -> p95 |correction|
# 0.090 -> 0.074 (-18%), 0.3-1 Hz -26%, active 26% -> 18% of the time; damping ratio 1.0 -> 0.76 (~3% overshoot);
# 0.6 would halve the 0.3-1 Hz part but at 0.51 (~16% overshoot) risks a slow weave with the car/model lag.
LANE_CENTER_KD = 1.20              # tncr19 Cross value (Altis 0.90); m/s^2 per m/s of lateral speed toward/away from the centre (damping)
LANE_CENTER_KI = 0.05              # m/s^2 per metre per second: slowly takes over what the model keeps pulling
LANE_CENTER_I_LIMIT = 0.25         # tncr19 Cross value (Altis 0.45); m/s^2; cap on the integrated part
# 2026-10-06 (route 00000109 23:27-23:34, ~103 km/h): the car sat LEFT of the lane centre the whole stretch (straight
# -10 cm, left curves -20..-35, right curves -13..-24 cm) with the centering saturated near 0.35-0.43 m/s^2 against the
# model's pull; the integral cap 0.25 -> 0.45 m/s^2 lets it finish the job. The total stays capped by MAX_LAT_ACCEL.
# The centering has no upper speed limit (full from 80 km/h up, e.g. 140 km/h).
LANE_CENTER_MAX_LAT_ACCEL = 0.6    # m/s^2; cap on the total correction as felt by the driver
LANE_CENTER_MAX_CURV = 3.5e-3      # 1/m; cap on the total correction (radius ~290 m)
LANE_CENTER_JERK = 0.5             # tncr19 Cross value (Altis 0.3); m/s^3; how fast the correction may change, as lateral jerk so it feels the same at
                                   # every speed (driver 2026-09-26: corrections at ~90 km/h felt stiff; was 3e-3 1/m/s = 1.9 m/s^3 there)
                                   # 0.5 -> 0.3 (owner 2026-10-10: "highway corrections still slightly noticeable"; route 0000011b
                                   # 85 km/h, 514 s: unlimited correction rate p95 0.12 / p99 0.20 m/s^3, the limit binds 0.1% -> 0.4%)
LANE_CENTER_FILTER_TAU = 0.5       # s; low-pass on the measured offset and heading

# Line mode (tncr, owner 2026-10-25: "on the highway, use the absolute centre line from the two seen lane lines instead of
# the planned path, so the centring is more precise"). Route 000000a9 (2026-10-23, > 85 km/h, 6.3 min, both lines
# >= 0.5 98% of the time): the car sat 4.2 cm left of the line centre with a 12.0 cm std (26% of the time > 10 cm); the
# model's path 20 m ahead was 5.1 cm left of the line centre with a 10.3 cm std, while the line centre itself moved only
# 1.3 cm (std) from frame to frame - the wander is the plan, not the lines. The owner's own steady hold is 3-4 cm std.
# With both lines confident the target curvature is taken from the line centre itself (pure pursuit of the centre
# LINE_T_LOOK ahead, which carries the lane's own curvature) instead of correcting the model's action: weight w blends
# from the PD correction above (w = 0) to the line curvature (w = 1). Equivalent gains of the pursuit: 2 / T^2 = 0.89
# m/s^2 per metre (KP above 0.35), 2 / T = 1.33 m/s^2 per m/s (KD 1.20), damping ratio 0.71. The model's action is the
# fallback whenever the lines are not trusted (exits, merges, worn paint, lane changes): w falls back in 0.5 s.
LINE_V_BP = [30.0 / 3.6, 40.0 / 3.6]  # m/s: line mode fades in with the PD centering (dpcr 11-10: 30-40, was 70-80; straights only below 70)
# 80-90 -> 70-80 km/h (owner 2026-10-27, route 000000ba): "at 100 km/h the highway is steady, at 90 it corrects left and
# right more". Dash 90 is ~82 km/h true (dial = 1.068 x true + 2.5), where the line weight was only ~0.2: steering
# angle jitter 0.31 deg at true 78-84 km/h vs 0.20 at 88-95 (lateral-accel jitter about equal; the same correction
# also takes ~25 % more wheel angle at the lower speed). Now full line mode (x LINE_W_MAX) from 80 km/h true (dash ~88).
LINE_V_W = [0.0, 1.0]
LINE_MIN_PROB = 0.7         # both near lines at least this (the PD centering accepts 0.5 / 0.4)
LINE_T_LOOK = 1.7           # s of travel to the pursued point on the line centre (dpcr 11-05 late, with LINE_PRED_T; tncr1 2.0)
# 1.5 -> 2.0 s (owner 2026-10-25 after route b5): at 100-110 km/h the car swayed +-10..15 cm with a ~4.5 s period (22:02,
# angle 2-5 deg); 61-84 % of the offset's energy sat in the 3-7 s band vs 39-56 % with the model's path (route a9), and
# the actual lateral accel lagged the request by 0.15-0.25 m/s^2 at the peaks: the pursuit loop is lightly damped on
# the real car (the closed-loop sketch, which lacks the perception/steering delays, did not reproduce it). A longer
# lookahead lowers the gains (2 / T^2 = 0.50 m/s^2 per m, 2 / T = 1.0 per m/s, was 0.89 / 1.33) and the sway with them.
LINE_L_RANGE = (20.0, 65.0)  # m; lookahead limits (45 m capped 1.5 s at 108 km/h: at 100 km/h it was really 1.62 s)
LINE_FIT_MARGIN = 10.0      # m beyond the lookahead included in the centre fit
LINE_MAX_DIFF_ACCEL = 1.0   # m/s^2; a line target this far from the model's is not trusted (a line of another lane)
LINE_W_MAX = 0.85  # dpcr (owner 2026-11-05 "as strong as possible"): between tncr1 0.7 (owner: smooth) and 1.0 (owner 10-26: corrections too large)
# 1.0 -> 0.7 (owner 2026-10-26, highway): smaller, more even corrections. At most 70 % of the line pursuit; the other
# 30 % stays with the PD correction of the model's path, so each pursuit correction is cut to 0.7x and a jumpy line
# estimate moves the wheel less (effective pursuit gain 0.35 m/s^2 per metre at T 2.0 s, was 0.50).
# dpcr (owner 2026-11-05 late, "try it"): pursue from where the car will be after the steering delay. Route 000000e4
# (rlog segs 26-41, 91 km/h, this centering on): the fast wobble was already tiny (position < 2 s +-0.5 cm, 2-4 s +-1.5),
# what was left was a slow weave (4-10 s +-6.8 cm) and drift (> 10 s +-6.5), and the actual curvature lagged the command
# by 0.36 s - the pursuit kept pushing after the car was already coming back. So the chord starts at the pose LINE_PRED_T
# ahead on the arc of the last command (no extra fit), and the lookahead is 1.7 s (was 2.0). Closed-loop sim on route
# e0, 85-100 km/h, std of the 15 m-ahead offset: straight 12.6 -> 11.4, left curves 11.8 -> 10.7, right 11.6 -> 10.9 cm;
# command jerk p95 0.86 -> 0.81 (75-85 km/h right curves 8.6 -> 9.0 cm inside). Return point: tag dpcr-lat-good-20261105.
# dpcr (owner 2026-11-08, route 000000fa 13:17 "slow left-right ping-pong on a dead straight road at 105 km/h"):
# 6.4 s weave, lane offset std 8.5 cm (+-12-15 cm), on every straight of the route and on fb too, not shrinking with
# drive time (learned params steady: lagd 0.455 s, steerRatio 14.4). Fit on fa rlogs: command -> steering-angle
# curvature 0.30 s dead time + 0.30 s first order, gyro yaw another 0.15-0.2 s -> ~0.75 s in total, so 0.36 s
# under-predicted by half and the pursuit loop is under-damped (sim: natural period 6.0 s, overshoot 31-38 %).
# Noise closed-loop sim with the real LaneCentering: offset std -10..13 %, command lat accel std -30..40 %; line
# pursuit still available 100 % of fa frames >= 60 km/h (the fit needs xd further). Lookahead/gains/filter unchanged
# (no added lag). First, minimal step; if it feels late or lags: back to 0.36 / tag dpcr-lat-good-20261105.
LOW_STRAIGHT_V = 70.0 / 3.6   # m/s: below this the centering works on straights only (handed over across 60-70 km/h)
LOW_STRAIGHT_K = [5e-4, 1.5e-3]  # 1/m: |lane-centre curvature| full centering (R >= 2000 m) -> none (R <= 670 m)
LINE_PRED_T = 0.75          # s: measured command -> actual yaw delay (route fa; 0.36 from route e4 until 11-08)
# dpcr (owner 2026-11-08 after 0.75 s: "it improved, try B too"): the pose after the delay comes from the commands
# actually sent during the last LINE_PRED_T (they are still "in the pipe"), not from the arc of the last command:
# heading psi_i = sum(k_j) v dt, lateral shift = sum(psi_i) v dt. A constant history gives the old arc exactly (and
# the history starts filled with the current command). Noise closed-loop sim with the real class and the fa lag fit:
# step overshoot 22.6 % -> 3.7 %, settle to 2 cm 10.5 -> 3.8 s, offset std -25 %, 5-10 s weave share 68 -> 41 %,
# command jerk p95 0.22 -> 0.25.
# dpcr (owner 2026-11-08, route 000000ff 14:35-14:36: "100-120 km/h curves still sit on the inside"): above 70 km/h
# the line pursuit carries 0.85 of the command, so the curve outward FRAC (curve_bias.py, 0.155 at 90-120 km/h, on the
# model's curvature only) is left with ~2 % (route ff 95-106 km/h curves: command / model 0.924). The pursuit's own
# curvature gets an outward fraction too, LINE_OUT_FRAC_V over LINE_OUT_V_BP, faded in with the curve over
# LINE_OUT_K_BP (no step at the straight / curve edge). Closed loop it settles outward of the lane centre by about
# (1/(1-frac) - 1) k L^2 / 2: at 100 km/h (L 47 m) with 0.08: R 2000 m ~6 cm, R 1000 m ~12 cm, R 500 m ~25 cm.
LINE_OUT_V_BP = [85.0 / 3.6, 100.0 / 3.6]  # m/s
LINE_OUT_FRAC_V = [0.0, 0.08]
LINE_OUT_K_BP = [1e-4, 3e-4]               # 1/m: |lane-centre curvature| (centre fit) over which the fraction fades in;
                                           # not the pursuit's own curvature (it wiggles +-0.26e-3 on straights)
LINE_W_RISE = 0.5           # 1/s: 2 s to take over
LINE_W_FALL = 2.0           # 1/s: 0.5 s to hand back
LINE_FILTER_TAU = 0.1       # s; low-pass on the line curvature (0.3 oscillated in the closed-loop sim with 0.5 s steering lag)


class LaneCentering:
  def __init__(self, dt: float):
    self.dt = dt
    self.offset_filter = FirstOrderFilter(0.0, LANE_CENTER_FILTER_TAU, dt)
    self.heading_filter = FirstOrderFilter(0.0, LANE_CENTER_FILTER_TAU, dt)
    self.width_filter = FirstOrderFilter(0.0, LANE_CENTER_WIDTH_TAU, dt)  # x == 0 until the first confident width
    self.integral = 0.0
    self.correction = 0.0
    self.offset = 0.0
    self.heading = 0.0
    self.valid = False
    self.last_meas: tuple[float, float] | None = None  # this frame's (offset, heading), for the low-speed trim
    self.line_w = 0.0  # line mode weight (0 = PD correction of the model, 1 = line centre curvature)
    self.line_filter = FirstOrderFilter(0.0, LINE_FILTER_TAU, dt)
    self.line_valid = False
    self.output = 0.0  # what update() returned (correction blended with line mode)
    self.k_prev = 0.0  # last commanded curvature (model + output), for the delay prediction
    self.k_hist: np.ndarray | None = None  # commanded curvature of the last LINE_PRED_T s (ring buffer)
    self.k_i = 0  # next slot = the oldest command
    self.lane_k = 0.0  # |lane-centre curvature| of the last line fit
    self.fit_key = None  # model frame the lane-centre fits below belong to
    self.fit_x = np.zeros(0)
    self.fit_centre = np.zeros(0)
    self.fits: dict[int, np.ndarray] = {}
    self.info_key = None  # model frame self.info belongs to
    self.info = None
    self.choice = LaneChoice(dt)  # 15-70 km/h lane choice (splits, left-turn bays, the driver's aim)

  def centre_fit(self, model_v2, x_max: float) -> tuple[int, np.ndarray | None]:
    """Quadratic fit (c2, c1, c0) of the lane-line centre over the points with x <= x_max, and how many points that is.
    The model publishes at 20 Hz and this runs at 100 Hz: the fits are made once per model frame and reused (each one
    costs ~0.38 ms on the C4, on core 4 that card, controlsd and selfdrived share - route b8 10-26 had core 4 at ~100 %
    and five 'System Lagging' alerts). Same numbers as fitting every cycle."""
    key = (id(model_v2), getattr(model_v2, 'frameId', None))
    if key != self.fit_key:
      lines = model_v2.laneLines
      self.fit_key = key
      self.fit_x = np.asarray(lines[1].x, dtype=float)
      y1, y2 = np.asarray(lines[1].y, dtype=float), np.asarray(lines[2].y, dtype=float)
      m = min(len(y1), len(y2))
      self.fit_centre = (y1[:m] + y2[:m]) / 2.0  # model y: positive = right (capnp lists do not slice)
      self.fits = {}
    n = int(np.sum(self.fit_x <= x_max))
    if n < 4 or n > len(self.fit_centre):
      return n, None
    if n not in self.fits:
      self.fits[n] = np.polyfit(self.fit_x[:n], self.fit_centre[:n], 2)
    return n, self.fits[n]

  def reset(self):
    self.offset_filter.x = 0.0
    self.heading_filter.x = 0.0
    self.integral = 0.0
    self.correction = 0.0
    self.valid = False
    self.line_w = 0.0
    self.line_valid = False
    self.output = 0.0
    self.k_hist = None

  def _straight_gate(self, v_ego: float) -> float:
    """1 above LOW_STRAIGHT_V; below it 1 on a straight and 0 in a curve (the curve stays with the model + curve_bias)"""
    if v_ego >= LOW_STRAIGHT_V:
      return 1.0
    g = float(np.interp(self.lane_k, LOW_STRAIGHT_K, [1.0, 0.0]))
    return g + (1.0 - g) * float(np.interp(v_ego, [LOW_STRAIGHT_V - 10.0 / 3.6, LOW_STRAIGHT_V], [0.0, 1.0]))

  def _push_command(self, k: float):
    n = max(int(round(LINE_PRED_T / self.dt)), 1)
    if self.k_hist is None or len(self.k_hist) != n:
      self.k_hist = np.full(n, k, dtype=float)
      self.k_i = 0
    else:
      self.k_hist[self.k_i] = k
      self.k_i = (self.k_i + 1) % n

  def predicted_pose(self, v_ego: float) -> tuple[float, float]:
    """(lateral shift, heading) after LINE_PRED_T from the commands sent in the last LINE_PRED_T (positive = right)"""
    if self.k_hist is None:
      xd = v_ego * LINE_PRED_T
      return self.k_prev * xd * xd / 2.0, self.k_prev * xd
    psi = np.cumsum(np.roll(self.k_hist, -self.k_i)) * (v_ego * self.dt)
    return float(np.sum(psi)) * v_ego * self.dt, float(psi[-1])

  def line_curvature(self, model_v2, v_ego: float) -> float | None:
    """Pure pursuit of the lane-line centre LINE_T_LOOK ahead (curvature, positive = right), from the pose LINE_PRED_T
    ahead predicted from the commands sent in the last LINE_PRED_T. None unless both near lines are confident and seen far enough; call only
    after measure() accepted the pair (width checks)."""
    info = self._frame_info(model_v2)
    if info is None or info[0] < LINE_MIN_PROB:
      return None
    look = min(max(v_ego * LINE_T_LOOK, LINE_L_RANGE[0]), LINE_L_RANGE[1])
    xd = v_ego * LINE_PRED_T
    nx, x_last, same_len = info[4:]
    if nx < 4 or x_last < xd + look + LINE_FIT_MARGIN or not same_len:
      return None
    _, fit = self.centre_fit(model_v2, xd + look + LINE_FIT_MARGIN)
    if fit is None:
      return None
    self.lane_k = abs(2.0 * float(fit[0]))  # |curvature| of the lane centre ahead
    y_d, psi_d = self.predicted_pose(v_ego)
    x = xd + look
    dy = float((fit[0] * x + fit[1]) * x + fit[2]) - y_d - psi_d * look
    return 2.0 * dy / (look ** 2 + dy ** 2)

  def _line_mode(self, model_v2, v_ego: float, meas_ok: bool, model_curvature: float | None, lane_changing: bool) -> float:
    """Blend the PD correction (self.correction) with the line-centre curvature; returns the curvature to add."""
    if model_curvature is None:
      return self.correction
    self.k_prev = model_curvature + self.output  # the last command: the arc the car is on through the delay
    self._push_command(self.k_prev)
    k_line = self.line_curvature(model_v2, v_ego) if meas_ok and not lane_changing else None
    if k_line is not None:
      k_line *= 1.0 - float(np.interp(v_ego, LINE_OUT_V_BP, LINE_OUT_FRAC_V)) * float(np.interp(self.lane_k, LINE_OUT_K_BP, [0.0, 1.0]))
    trusted = k_line is not None and abs(k_line - model_curvature) * v_ego ** 2 <= LINE_MAX_DIFF_ACCEL
    if trusted:
      if not self.line_valid and self.line_w <= 0.0:
        self.line_filter.x = k_line
      self.line_filter.update(k_line)
    else:
      # hand back: the target follows the model's curvature while the weight falls
      if self.line_w <= 0.0:
        self.line_filter.x = model_curvature
      self.line_filter.update(model_curvature)
    self.line_valid = trusted
    w_target = LINE_W_MAX * float(np.interp(v_ego, LINE_V_BP, LINE_V_W)) if trusted else 0.0
    w_target *= self._straight_gate(v_ego)
    rate = LINE_W_RISE if w_target > self.line_w else LINE_W_FALL
    self.line_w = _clip(w_target, self.line_w - rate * self.dt, self.line_w + rate * self.dt)
    return (1.0 - self.line_w) * self.correction + self.line_w * (self.line_filter.x - model_curvature)

  def _frame_info(self, model_v2):
    """per model frame (20 Hz), not per control cycle (100 Hz): the lane-line reads, the width and the near fit.
    dpcr 2026-11-05: with the centering on, the C4's core 4 went 68 -> 81 % (p95 88 -> 92 %) at highway speed, and
    tncr1 once lagged ('System Lagging', route b8 10-26) from per-cycle work here; the outputs are unchanged."""
    key = (id(model_v2), getattr(model_v2, 'frameId', None))
    if key == self.info_key:
      return self.info
    self.info_key = key
    self.info = None
    lines = model_v2.laneLines
    probs = model_v2.laneLineProbs
    if len(lines) < 4 or len(probs) < 4 or len(lines[1].y) < 4 or len(lines[2].y) < 4:
      return None
    p1, p2 = float(probs[1]), float(probs[2])
    width = float(lines[2].y[0]) - float(lines[1].y[0])
    _, fit = self.centre_fit(model_v2, LANE_CENTER_FIT_RANGE)
    # centre(x) ~ c0 + c1 x + c2 x^2: c0 is the centre's lateral position (right of the car), c1 its slope. A lane
    # that runs to the right ahead (c1 > 0) means the car is pointing LEFT of it.
    meas = None if fit is None else (float(fit[2]), float(math.atan(fit[1])))
    nx = len(lines[1].x)
    self.info = (min(p1, p2), max(p1, p2), width, meas, nx, float(lines[1].x[nx - 1]) if nx else 0.0, len(lines[2].y) == nx)
    return self.info

  def measure(self, model_v2) -> tuple[float, float] | None:
    """(offset, heading) of the car relative to the lane centre: offset positive = car LEFT of centre, heading
    positive = car pointing LEFT of the lane direction. None when the lines are unreliable."""
    if model_v2 is None:
      return None
    info = self._frame_info(model_v2)
    if info is None:
      return None
    p_lo, p_hi, width, meas = info[:4]
    if p_hi < LANE_CENTER_MIN_LINE_PROB or p_lo < LANE_CENTER_MIN_OTHER_PROB:
      return None
    if not (LANE_CENTER_WIDTH_RANGE[0] <= width <= LANE_CENTER_WIDTH_RANGE[1]):
      return None
    if self.width_filter.x > 0.0 and abs(width - self.width_filter.x) > LANE_CENTER_WIDTH_TOL:
      return None  # one of the two lines is not this lane's (a lost outer line placed somewhere else)
    if p_lo >= LANE_CENTER_WIDTH_CONF_PROB:
      if self.width_filter.x > 0.0:
        self.width_filter.update(width)
      else:
        self.width_filter.x = width
    return meas

  def update(self, model_v2, v_ego: float, active: bool, steering_pressed: bool, straight: float = 0.0,
             model_curvature: float | None = None, lane_changing: bool = False) -> float:
    """Curvature to ADD to the desired curvature (positive = right). straight: tncr straight mode weight (0..1).
    model_curvature: the desired curvature this is added to (enables line mode); lane_changing: hand back to the model."""
    # dpcr (owner 2026-11-05 late: "after a lane change, in one or two places the correction felt sharp"): no position
    # feedback during a lane change. Route e4 rlog: the PD kept pushing toward the NEW lane's centre (+-0.2..0.33 m/s^2,
    # on top of the model's own manoeuvre) and its integral wound up to +-0.10 m/s^2, which then pushed the car 10-13 cm
    # past the centre after the change. Like lost lines: the correction winds down at the jerk limit and the integral
    # restarts at 0 once the change is over. Closed-loop sim (route e0, 3 changes): offset peak in the 8 s after a change
    # 44 -> 24 cm, correction peak 0.32 -> 0.10 m/s^2.
    meas = self.measure(model_v2) if active and not steering_pressed and not lane_changing and v_ego > LANE_CENTER_MIN_SPEED else None
    self.last_meas = meas
    tau = blend(LANE_CENTER_FILTER_TAU, STRAIGHT_CENTER_TAU, straight)
    self.offset_filter.update_alpha(tau)
    self.heading_filter.update_alpha(tau)
    deadband = blend(LANE_CENTER_DEADBAND, STRAIGHT_CENTER_DEADBAND, straight)
    step = blend(LANE_CENTER_JERK, STRAIGHT_CENTER_JERK, straight) / max(v_ego, LANE_CENTER_MIN_SPEED) ** 2 * self.dt
    if meas is None:
      # lines lost or driver steering: wind the correction back to zero at the same jerk instead of snapping (the
      # snap showed up as a 3.9 m/s^3 spike when replaying route 53 seg 15)
      self.offset_filter.x = 0.0
      self.heading_filter.x = 0.0
      self.integral = 0.0
      self.valid = False
      self.correction = _clip(0.0, self.correction - step, self.correction + step)
      self.output = self._line_mode(model_v2, v_ego, False, model_curvature, lane_changing)
      return self.output

    self.valid = True
    self.offset = self.offset_filter.update(meas[0])
    self.heading = self.heading_filter.update(meas[1])
    error = self.offset - _clip(self.offset, -deadband, deadband)  # deadband
    lateral_speed = v_ego * math.sin(self.heading)  # positive = drifting LEFT

    self.integral = _clip(self.integral + LANE_CENTER_KI * error * self.dt, -LANE_CENTER_I_LIMIT, LANE_CENTER_I_LIMIT)
    lat_accel = LANE_CENTER_KP * error + LANE_CENTER_KD * lateral_speed + self.integral
    lat_accel = _clip(lat_accel, -LANE_CENTER_MAX_LAT_ACCEL, LANE_CENTER_MAX_LAT_ACCEL)
    target = _clip(lat_accel / max(v_ego, LANE_CENTER_MIN_SPEED) ** 2, -LANE_CENTER_MAX_CURV, LANE_CENTER_MAX_CURV)
    target *= float(np.interp(v_ego, LANE_CENTER_FADE_BP, LANE_CENTER_FADE_V)) * self._straight_gate(v_ego)
    self.correction = _clip(target, self.correction - step, self.correction + step)
    self.output = self._line_mode(model_v2, v_ego, True, model_curvature, lane_changing)
    return self.output


def apply_lane_centering(lc: LaneCentering, desired_curvature: float, model_v2, CS, lat_active: bool,
                         curvature: float = 0.0) -> float:
  """controlsd hook: the model's (curve-biased) desired curvature plus the centering correction while lateral is
  active; always updated so the correction winds down / resets when inactive. curvature: the car's actual curvature
  (controlsd's, for the lane choice's aim point)."""
  lane_changing = model_v2 is not None and model_v2.meta.laneChangeState != 0
  # dpcr 11-10: the centering now runs from 30 km/h, inside the lane choice's 15-70 km/h; while the lane choice steers
  # to another lane the centering hands back as in a lane change (it would pull to the model's lane)
  corr = lc.update(model_v2, CS.vEgo, lat_active, CS.steeringPressed, 0.0, desired_curvature, lane_changing or lc.choice.active)
  if lat_active:
    corr += lc.choice.update(model_v2, CS.vEgo, desired_curvature, curvature, CS.steeringPressed,
                             CS.leftBlinker or CS.rightBlinker, lane_changing)
  else:
    lc.choice.reset()
  return desired_curvature + corr if lat_active else desired_curvature
