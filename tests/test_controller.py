import math
import unittest

from controller import Config, Controller, Gains, PID


class PIDTests(unittest.TestCase):
    def test_pid_terms_and_first_sample(self):
        pid = PID(Gains(0.1, 0.2, 0.05))
        self.assertAlmostEqual(pid.update(1, 0.5), 0.2)
        self.assertAlmostEqual(pid.update(2, 0.5), 0.6)
        pid.reset()
        self.assertAlmostEqual(pid.update(1, 0.5), 0.2)

    def test_saturation_does_not_wind_up(self):
        pid = PID(Gains(1, 0.1))
        for _ in range(100):
            self.assertEqual(pid.update(10, 0.1), 1)
        self.assertEqual(pid.integral, 0)
        self.assertEqual(pid.update(0, 0.1), 0)
        self.assertEqual(pid.update(-10, 0.1), -1)

    def test_integral_limit(self):
        pid = PID(Gains(0, 0.01, integral_limit=2))
        for _ in range(20):
            pid.update(1, 1)
        self.assertEqual(pid.integral, 2)

    def test_invalid_dt_and_error(self):
        for dt in (0, -1, math.inf, math.nan):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                PID(Gains(1)).update(1, dt)
        with self.assertRaises(ValueError):
            PID(Gains(1)).update(math.nan, 1)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        gains = Gains(0.1)
        self.c = Controller(Config(roll=gains, pitch=gains, yaw=gains))

    def feed(self):
        for name, value in dict(pitch=2, pitchTarget=4, roll=5, rollTarget=0, slip=3).items():
            self.c.set_input(name, value)

    def test_starts_off_and_off_suppresses_updates(self):
        self.feed()
        self.assertEqual(self.c.step(0.1), {})
        self.c.set_input("on_off", True)
        commands = self.c.step(0.1)
        self.assertAlmostEqual(commands["controlPitch"], 0.2)
        self.assertAlmostEqual(commands["controlRoll"], -0.5)
        self.assertAlmostEqual(commands["controlYaw"], -0.3)
        self.c.set_input("on_off", False)
        self.assertEqual(self.c.step(0.1), {})

    def test_waits_per_axis_and_accepts_zero_targets(self):
        self.c.set_input("on_off", True)
        self.assertEqual(self.c.step(0.1), {})
        self.c.set_input("roll", 5)
        self.assertEqual(self.c.step(0.1), {"controlRoll": -0.5})
        self.c.set_input("rollTarget", 0)
        self.assertEqual(self.c.step(0.1), {"controlRoll": -0.5})

    def test_reset_clears_everything_and_requires_enable_and_fresh_data(self):
        self.feed()
        self.c.set_input("on_off", True)
        self.c.step(0.1)
        self.c.reset()
        self.assertFalse(self.c.enabled)
        self.assertEqual(self.c.inputs, {})
        self.assertEqual(self.c.outputs, {})
        for pid in self.c.pids.values():
            self.assertEqual(pid.integral, 0)
            self.assertIsNone(pid.previous_error)
        self.c.set_input("on_off", True)
        self.assertEqual(self.c.step(0.1), {})

    def test_unused_inputs_do_not_affect_control(self):
        self.feed()
        self.c.set_input("on_off", True)
        expected = self.c.step(0.1)
        for name in ("airspeed", "verticalSpeed"):
            self.c.set_input(name, 0)
            self.c.set_input(name + "Target", 1000)
        self.assertEqual(self.c.step(0.1), expected)

    def test_invalid_measurement_invalidates_cached_value(self):
        self.feed()
        self.c.set_input("on_off", True)
        for invalid in (None, True, "3", math.nan, math.inf):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                self.c.set_input("roll", invalid)
            self.assertNotIn("controlRoll", self.c.step(0.1))

    def test_enable_clears_pid_history(self):
        self.feed()
        self.c.set_input("on_off", True)
        self.c.step(1)
        self.c.set_input("on_off", False)
        self.assertIsNone(self.c.pids["roll"].previous_error)

    def test_simple_plant_reduces_error(self):
        self.c.set_input("on_off", True)
        self.c.set_input("rollTarget", 0)
        roll = 10
        for _ in range(400):
            self.c.set_input("roll", roll)
            roll += self.c.step(0.1)["controlRoll"] * 0.1
        self.assertLess(abs(roll), 0.2)

    def test_heading_overrides_roll_and_wraps(self):
        self.c.set_input("on_off", True)
        self.c.set_input("roll", 0)
        self.c.set_input("rollTarget", -5)
        self.c.set_input("headingTarget", 10)
        self.assertNotIn("controlRoll", self.c.step(0.1))
        for heading, expected in ((350, 1), (30, -1), (9, 0), (10, 0)):
            self.c.set_input("heading", heading)
            self.assertEqual(self.c.step(0.1)["controlRoll"], expected)
        self.c.set_input("headingTarget", 350)
        self.c.set_input("heading", 10)
        self.assertEqual(self.c._roll_target(), -15)

    def test_altitude_capture_latches_and_target_change_rearms(self):
        for name, value in dict(pitch=0, pitchTarget=10, altitude=900, altitudeTarget=1000).items():
            self.c.set_input(name, value)
        self.c.set_input("on_off", True)
        self.assertEqual(self.c.step(0.1)["controlPitch"], 1)
        self.assertFalse(self.c.altitude_hold)
        self.c.set_input("altitude", 1000)
        self.assertAlmostEqual(self.c.step(0.1)["controlPitch"], 0.2)
        self.assertTrue(self.c.altitude_hold)
        self.c.set_input("altitude", 1100)
        self.assertLess(self.c.step(0.1)["controlPitch"], 0)
        self.c.set_input("altitudeTarget", 1000)
        self.assertTrue(self.c.altitude_hold)
        self.c.set_input("altitude", 900)
        self.assertGreater(self.c.step(0.1)["controlPitch"], 0)
        self.assertTrue(self.c.altitude_hold)
        self.c.set_input("altitudeTarget", 2000)
        self.assertFalse(self.c.altitude_hold)
        self.assertEqual(self.c.step(0.1)["controlPitch"], 1)
        self.c.reset()
        self.assertFalse(self.c.altitude_hold)
        self.assertIsNone(self.c.previous_altitude_error)

    def test_capture_crossing_in_both_directions(self):
        for start, end in ((900, 1100), (1100, 900)):
            with self.subTest(start=start):
                self.c.reset()
                for name, value in dict(pitch=0, pitchTarget=10, altitude=start,
                                        altitudeTarget=1000, on_off=True).items():
                    self.c.set_input(name, value)
                self.c.step(0.1)
                self.c.set_input("altitude", end)
                self.c.step(0.1)
                self.assertTrue(self.c.altitude_hold)

    def test_capture_tolerance_and_pitch_limits(self):
        self.c.set_input("on_off", True)
        self.c.set_input("pitch", 0)
        self.c.set_input("altitudeTarget", 1000)
        self.assertNotIn("controlPitch", self.c.step(0.1))
        self.c.set_input("altitude", 995)
        self.assertIn("controlPitch", self.c.step(0.1))
        self.assertTrue(self.c.altitude_hold)
        for altitude, expected in ((-10000, 10), (10000, -15)):
            self.c.set_input("altitude", altitude)
            self.assertEqual(self.c._pitch_target(0.1), expected)


if __name__ == "__main__":
    unittest.main()
