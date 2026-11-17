import unittest

from opendbc.car import DT_CTRL, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP

LKAS_ALT = 0x110


def make_interface(candidate):
  fingerprint = {b: {} for b in range(8)}
  fingerprint[0] = {0x110: 32}
  fingerprint[1] = {0x1AA: 16, 0xFA: 32, 0x161: 32, 0x162: 32}
  fingerprint[2] = {0x110: 32, 0x161: 32, 0x162: 32}
  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


def steering_frames(sends):
  out = []
  for addr, dat, _ in sends:
    if addr == LKAS_ALT:
      torque = (((dat[6] & 0xF) << 7) | (dat[5] >> 1)) - 1024
      out.append((torque, (dat[6] >> 4) & 1))
  return out


class TestCarnivalSteering(unittest.TestCase):
  def run_turn(self, candidate, frames=300, angle=150.0):
    CI = make_interface(candidate)
    self.assertTrue(CI.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT)
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.actuators.torque = -1.0
    CC = CC.as_reader()
    now_nanos = 0
    result = []
    for _ in range(frames):
      CI.update([])
      CI.CS.out.steeringAngleDeg = angle
      _, sends = CI.apply(CC, structs.CarControlSP(), now_nanos)
      result.extend(steering_frames(sends))
      now_nanos += int(DT_CTRL * 1e9)
    return CI, result

  def test_carnival_limits(self):
    CI, frames = self.run_turn(CAR.KIA_CARNIVAL_4TH_GEN)
    params = CarControllerParams(CI.CP)
    self.assertEqual((params.STEER_MAX, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN), (384, 3, 7))
    self.assertTrue(CI.CP_SP.safetyParam & HyundaiSafetyFlagsSP.CANFD_ALT_LIMITS)
    self.assertEqual(min(t for t, _ in frames), -384)
    self.assertEqual(frames[3][0], -12)

  def test_carnival_cuts_request_above_angle_limit(self):
    _, frames = self.run_turn(CAR.KIA_CARNIVAL_4TH_GEN)
    cuts = [i for i, (_, req) in enumerate(frames[1:], 1) if not req]
    self.assertTrue(cuts)
    self.assertEqual(cuts[0], 79)

  def test_other_canfd_cars_unchanged(self):
    CI, frames = self.run_turn(CAR.KIA_EV6)
    params = CarControllerParams(CI.CP)
    self.assertEqual((params.STEER_MAX, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN), (270, 2, 3))
    self.assertFalse(CI.CP_SP.safetyParam & HyundaiSafetyFlagsSP.CANFD_ALT_LIMITS)
    self.assertEqual(min(t for t, _ in frames), -270)
    cuts = [i for i, (_, req) in enumerate(frames[1:], 1) if not req]
    self.assertEqual(cuts[0], 89)


if __name__ == "__main__":
  unittest.main()
