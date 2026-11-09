import numpy as np
from types import SimpleNamespace as N

from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from dragonpilot.selfdrive.controls.lib import camera_offset as co

DC = DEVICE_CAMERAS[('tici', 'ox03c10')]


def _sampled_ground_x(m, intr, h, z):
  """world lateral position (m, + right) of the road point the model sees straight ahead (model column of x=0) at depth z"""
  # the camera pixel of the ground point (0, h, z) and the model pixel that samples it without the offset
  u0 = intr @ np.array([0.0, h, z]); u0 = u0 / u0[2]
  m0 = get_warp_matrix(np.zeros(3, dtype=np.float32), intr, False)
  p_model = np.linalg.solve(m0, u0); p_model /= p_model[2]
  u = m @ p_model; u /= u[2]  # the camera pixel the (sheared) warp samples there
  return (u[0] - intr[0, 2]) * z / intr[0, 0]


def test_one_cm_left_at_every_depth():
  calib = np.zeros(3, dtype=np.float32)
  sm = {'liveCalibration': N(height=[1.22], rpyCalib=[0.0, 0.0, 0.0])}
  m_main = get_warp_matrix(calib, DC.fcam.intrinsics, False).astype(np.float32)
  m_extra = get_warp_matrix(calib, DC.ecam.intrinsics, True).astype(np.float32)
  s_main, s_extra = co.apply_camera_offset(m_main, m_extra, sm, DC, False)
  assert co.CAMERA_OFFSET == 0.01
  for z in (8.0, 20.0, 50.0):
    # the model column that showed the road at x = 0 now shows the road at x = +1 cm (right): the road looks shifted
    # 1 cm left, so the model steers the car 1 cm left to centre it
    x = _sampled_ground_x(s_main, DC.fcam.intrinsics, 1.22, z)
    assert abs(x - co.CAMERA_OFFSET) < 1e-3, (z, x)
  assert not np.allclose(s_extra, m_extra)


def test_zero_offset_is_identity_and_no_height_falls_back():
  calib = np.zeros(3, dtype=np.float32)
  m = get_warp_matrix(calib, DC.fcam.intrinsics, False).astype(np.float32)
  a, b = co.apply_camera_offset(m, m, {'liveCalibration': N(height=[], rpyCalib=[])}, DC, False, offset=0.0)
  assert a is m and b is m
  a, _ = co.apply_camera_offset(m, m, {'liveCalibration': N(height=[], rpyCalib=[])}, DC, False)
  assert np.isfinite(a).all() and not np.allclose(a, m)
