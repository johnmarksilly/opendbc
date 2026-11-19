import unittest

from opendbc.car import DT_CTRL, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, Buttons
from opendbc.sunnypilot.car.hyundai.ccnc import ALT_RESUME_PRESS_FRAMES, ALT_RESUME_RELEASE_FRAMES

CRUISE_BUTTONS_ALT = 0x1AA


def make_interface():
  fingerprint = {b: {} for b in range(8)}
  fingerprint[0] = {0x110: 32}
  fingerprint[1] = {0x1AA: 16, 0xFA: 32, 0x161: 32, 0x162: 32}
  fingerprint[2] = {0x110: 32, 0x161: 32, 0x162: 32}
  candidate = CAR.KIA_CARNIVAL_4TH_GEN
  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


def run(CI, frames, resume, start_counter=250):
  CC = structs.CarControl()
  CC.enabled = True
  CC.cruiseControl.resume = resume
  CC = CC.as_reader()
  now_nanos = 0
  out = []
  for i in range(frames):
    CI.update([])
    CI.CS.buttons_counter = (start_counter + i // 2) % 256
    _, sends = CI.apply(CC, structs.CarControlSP(), now_nanos)
    for addr, dat, bus in sends:
      if addr == CRUISE_BUTTONS_ALT:
        out.append((i, dat[2], (dat[4] >> 4) & 0x7, bus, CI.CS.buttons_counter))
    now_nanos += int(DT_CTRL * 1e9)
  return out


class TestCcncAltResume(unittest.TestCase):
  def test_resume_holds_press_across_counters(self):
    CI = make_interface()
    self.assertTrue(CI.CC.ccnc_lka_alt_buttons)
    sends = run(CI, 200, True)
    self.assertTrue(sends)
    for _, counter, btn, bus, car_counter in sends:
      self.assertEqual(btn, Buttons.RES_ACCEL)
      self.assertEqual(bus, CI.CC.CAN.ECAN)
      self.assertEqual(counter, (car_counter + 1) % 256)

    pressed = sorted({i for i, *_ in sends})
    presses, run_start, prev = [], pressed[0], pressed[0]
    for i in pressed[1:]:
      if i != prev + 1:
        presses.append((run_start, prev))
        run_start = i
      prev = i
    presses.append((run_start, prev))
    self.assertGreaterEqual(len(presses), 3)
    for start, end in presses:
      self.assertEqual(end - start + 1, ALT_RESUME_PRESS_FRAMES)
    for (s1, _), (s2, _) in zip(presses, presses[1:], strict=False):
      self.assertEqual(s2 - s1, ALT_RESUME_PRESS_FRAMES + ALT_RESUME_RELEASE_FRAMES)

    counters = {c for _, c, *_ in sends[:ALT_RESUME_PRESS_FRAMES * 2]}
    self.assertGreaterEqual(len(counters), ALT_RESUME_PRESS_FRAMES // 2)
    self.assertIn(0, {c for _, c, *_ in sends})

  def test_no_buttons_without_resume(self):
    self.assertEqual(run(make_interface(), 200, False), [])


if __name__ == "__main__":
  unittest.main()
