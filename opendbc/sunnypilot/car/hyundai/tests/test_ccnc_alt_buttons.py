import unittest

from opendbc.car import DT_CTRL, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, Buttons, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

CRUISE_BUTTONS_ALT = 0x1AA
CAM_BUS = 2


def make_interface(alpha_long=False):
  fingerprint = {b: {} for b in range(8)}
  fingerprint[0] = {0x1AA: 16, 0x1A0: 32, 0x35: 32, 0xEA: 24}
  fingerprint[2] = {0x161: 32, 0x162: 32, 0x1A0: 32}
  candidate = CAR.KIA_EV6
  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=alpha_long, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [], alpha_long=alpha_long, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


class TestCcncLfaAltButtons(unittest.TestCase):
  def setUp(self):
    self.CI = make_interface()
    self.now_nanos = 0

  def step(self, resume=False, cancel=False):
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.cruiseControl.resume = resume
    CC.cruiseControl.cancel = cancel
    self.CI.update([])
    _, sends = self.CI.apply(CC.as_reader(), structs.CarControlSP(), self.now_nanos)
    self.now_nanos += int(DT_CTRL * 1e9)
    return sends

  def button_sends(self, sends):
    return [(dat, bus) for addr, dat, bus in sends if addr == CRUISE_BUTTONS_ALT]

  def test_platform(self):
    self.assertTrue(self.CI.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS)
    self.assertTrue(self.CI.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC)
    self.assertFalse(self.CI.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG)
    self.assertTrue(self.CI.CP_SP.flags & HyundaiFlagsSP.CCNC)
    self.assertFalse(self.CI.CP.openpilotLongitudinalControl)

  def test_resume_sends_alt_buttons_to_camera(self):
    for _ in range(30):
      self.step()
    sends = self.button_sends(self.step(resume=True))
    self.assertEqual(len(sends), 10)
    for dat, bus in sends:
      self.assertEqual(bus, CAM_BUS)
      self.assertEqual((dat[4] >> 4) & 0x7, Buttons.RES_ACCEL)

  def test_resume_rate_limited(self):
    for _ in range(30):
      self.step()
    counts = [len(self.button_sends(self.step(resume=True))) for _ in range(int(1 / DT_CTRL))]
    self.assertEqual(sum(1 for c in counts if c), 4)

  def test_cancel_does_not_send_alt_buttons(self):
    for _ in range(100):
      self.assertEqual(self.button_sends(self.step(cancel=True)), [])


if __name__ == "__main__":
  unittest.main()
