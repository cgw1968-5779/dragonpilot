"""
dragonpilot (dpcr, Corolla Cross Hybrid): fixed camera offset - where the car sits in the lane.

Owner 2026-11-09: "the centring always feels a little right; I judge it by the distance from both door mirrors to the
lane lines, not from the driver's seat - I just want the car 1 cm further left." Route 00000000--41925a5692 (11-09,
rlog lane lines, lateral active, straights) put the car within +-1 cm of the model's lane centre at 50-90 km/h, so the
remaining bias is between the camera and the car body, which only a camera offset fixes.

Same method as sunnypilot's CameraOffset (tncr1: sunnypilot/modeld_v2/camera_offset_helper.py, Haibin Wen, sunnypilot
and contributors, MIT): a shear on the model's warp matrices so the model sees the road shifted and drives the car
CAMERA_OFFSET to the side. Lane lines and path shift together, so the highway lane centering (which centres the model's
lane lines) holds the same position; curves and corrections are unchanged. A constant, no Params switch (tncr1's
-0.03 does not apply: dpcr's centring is different and the owner set the target directly).
Sign (shear math, as sunnypilot's UI): + = the car sits LEFT. Applied in modeld whenever the warp matrices are rebuilt
(one hook line).
"""
import numpy as np

CAMERA_OFFSET = 0.01   # m, + = car sits left (owner 2026-11-09: 1 cm left)
DEFAULT_HEIGHT = 1.22  # m, camera height when liveCalibration has none yet


def v_horizon(intrinsics, rpy_calib) -> float:
  cy = intrinsics[1, 2]
  if len(rpy_calib) == 3 and np.isfinite(rpy_calib).all():
    return float(cy - intrinsics[1, 1] * np.tan(rpy_calib[1]))
  return float(cy)


def shear(model_transform: np.ndarray, height: float, offset: float, horizon: float) -> np.ndarray:
  s = np.eye(3, dtype=np.float32)
  s[0, 1] = offset / height
  s[0, 2] = -offset / height * horizon
  return (s @ model_transform).astype(np.float32)


def apply_camera_offset(model_transform_main, model_transform_extra, sm, dc, main_wide_camera: bool,
                        offset: float = CAMERA_OFFSET):
  """modeld hook: the main / extra warp matrices with the fixed camera offset applied (sm: modeld's SubMaster)"""
  if offset == 0.0:
    return model_transform_main, model_transform_extra
  live_calib = sm["liveCalibration"]
  height = float(live_calib.height[0]) if len(live_calib.height) and np.isfinite(live_calib.height[0]) and live_calib.height[0] > 0.5 \
    else DEFAULT_HEIGHT
  rpy = list(live_calib.rpyCalib)
  main_intr = dc.ecam.intrinsics if main_wide_camera else dc.fcam.intrinsics
  return (shear(model_transform_main, height, offset, v_horizon(main_intr, rpy)),
          shear(model_transform_extra, height, offset, v_horizon(dc.ecam.intrinsics, rpy)))
