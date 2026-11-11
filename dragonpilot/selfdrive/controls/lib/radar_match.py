"""
dragonpilot (dppb, Corolla Altis; owner 2026-11-11): do not pair the camera's lead with a radar track in another lane.

Route 000001a7 21:25:40, 123 km/h: the camera saw the lead in our lane (y ~0.1-0.25 m, ~110 m, ~116 km/h, prob 0.5-0.6)
and radard's match_vision_to_track paired it with radar track 9780 in the LEFT lane (yRel -3.8 m, 93 m, 81 km/h): the
stock match has no lateral check and its speed check passes any moving target (v_ego + vRel > 3). The planner then
saw a car 93 m ahead closing at 42 km/h and sent -1.74 (actual -2.62, 123 -> 116.5 km/h) for ~0.5 s - a phantom brake.

A track is not offered to the match when it is clearly ANOTHER object than the camera's lead - all three at once:
farther than FAR_D, more than LAT_D to the side of the camera's lead, and its speed more than DV from the camera's.
Then the best of the remaining tracks is matched, or - none left - the camera's own lead is used (the lead is never
dropped). Within FAR_D everything is as stock, and the low-speed override still sees every track.
11-10/11-11 rlogs (101 min of radar leads): 152 frames (7.6 s) qualify, all slower tracks in an adjacent lane; of the
36 tracks 3 later cut into our lane and were matched again as soon as they were in it (closest 56 / 21 / 15 m).
"""
FAR_D = 40.0          # m
LAT_D = 2.5           # m
DV = 15.0 / 3.6       # m/s


def plausible_tracks(v_ego: float, lead, tracks: dict) -> dict:
  """the tracks the camera's lead may be matched with (radard RADAR_TO_CAMERA offset is within the 40 m margin)"""
  y_cam, v_cam = -float(lead.y[0]), float(lead.v[0])
  return {k: c for k, c in tracks.items()
          if not (c.dRel > FAR_D and abs(c.yRel - y_cam) > LAT_D and abs(c.vRel + v_ego - v_cam) > DV)}
