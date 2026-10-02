"""Optional real binding smoke test; never starts networking."""

import unittest

try:
    import ingescape as igs
except ImportError:
    igs = None

from autopilot import Agent
from controller import Config


@unittest.skipIf(igs is None, "Ingescape runtime is not installed")
class RuntimeTests(unittest.TestCase):
    def test_real_callbacks_and_reset(self):
        self.addCleanup(igs.clear_context)
        agent = Agent(igs, Config())
        agent.setup("Autopilot_Test")
        self.assertFalse(igs.input_bool("on_off"))
        self.assertFalse(igs.output_bool("on_off"))
        igs.input_set_double("roll", 10.0)
        igs.input_set_int("rollTarget", 0)
        igs.input_set_bool("on_off", True)
        self.assertTrue(agent.controller.enabled)
        self.assertTrue(igs.output_bool("on_off"))
        agent.tick(1.0)
        self.assertLess(igs.output_double("controlRoll"), 0)
        igs.input_set_impulsion("reset")
        self.assertFalse(agent.controller.enabled)
        self.assertFalse(igs.input_bool("on_off"))
        self.assertFalse(igs.output_bool("on_off"))
        self.assertEqual(agent.controller.inputs, {})
        self.assertEqual(agent.controller.outputs, {})
        igs.input_set_bool("on_off", True)
        agent.tick(2.0)
        self.assertEqual(agent.controller.outputs, {})


if __name__ == "__main__":
    unittest.main()
