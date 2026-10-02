import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from autopilot import Agent, load_config
from controller import Config, MEASUREMENTS, OUTPUTS, TARGETS


class AgentTests(unittest.TestCase):
    def setUp(self):
        self.igs = Mock(IMPULSION_T=1, BOOL_T=2, DOUBLE_T=3, INTEGER_T=4)
        self.agent = Agent(self.igs, Config())
        self.agent.setup("test")

    def send(self, name, value):
        self.agent.on_input(None, name, None, value, None)

    def test_interface_preserved_except_removed_outputs(self):
        inputs = {call.args[0] for call in self.igs.input_create.call_args_list}
        outputs = {call.args[0] for call in self.igs.output_create.call_args_list}
        self.assertEqual(inputs, set(MEASUREMENTS + TARGETS) | {"reset", "on_off"})
        self.assertEqual(outputs, set(OUTPUTS) | {"on_off"})
        self.igs.output_create.assert_any_call("on_off", 2, False)
        self.assertEqual(self.igs.observe_input.call_count, len(inputs))

    def test_all_inputs_mapped_to_requested_sources(self):
        aircraft = ("heading", "airspeed", "altitude", "verticalSpeed", "roll", "pitch", "slip")
        cognitive = ("reset", "on_off", "headingTarget", "airspeedTarget",
                     "altitudeTarget", "verticalSpeedTarget", "rollTarget", "pitchTarget")
        expected = {(name, "Aircraft", name) for name in aircraft}
        expected.update((name, "Cognitive_Model", name) for name in cognitive)
        actual = [call.args for call in self.igs.mapping_add.call_args_list]
        self.assertEqual(set(actual), expected)
        self.assertEqual(len(actual), len(expected))
        inputs = {call.args[0] for call in self.igs.input_create.call_args_list}
        self.assertIn("slip", inputs)
        self.assertNotIn("skid", inputs)

    def test_mirror_and_disable_after_running(self):
        self.send("roll", 10)
        self.send("rollTarget", 0)
        self.agent.tick(1)
        self.igs.output_set_double.assert_not_called()
        self.send("on_off", True)
        self.igs.output_set_bool.assert_called_with("on_off", True)
        self.agent.tick(2)
        self.assertEqual(self.igs.output_set_double.call_count, 1)
        self.send("on_off", False)
        self.igs.output_set_bool.assert_called_with("on_off", False)
        self.assertEqual(
            [call.args for call in self.igs.output_set_double.call_args_list[-3:]],
            [("controlPitch", 0.0), ("controlRoll", 0.0), ("controlYaw", 0.0)],
        )
        self.agent.tick(3)
        self.assertEqual(self.igs.output_set_double.call_count, 4)

    def test_false_zeros_controls_even_when_already_off(self):
        for _ in range(2):
            self.igs.output_set_double.reset_mock()
            self.send("on_off", False)
            self.assertEqual(
                [call.args for call in self.igs.output_set_double.call_args_list],
                [("controlPitch", 0.0), ("controlRoll", 0.0), ("controlYaw", 0.0)],
            )

    def test_reset_clears_wire_values_and_mirrors_false(self):
        self.send("on_off", True)
        self.send("reset", None)
        self.igs.input_set_bool.assert_called_with("on_off", False)
        self.igs.output_set_bool.assert_called_with("on_off", False)
        self.assertEqual(self.igs.clear_input.call_count, len(MEASUREMENTS + TARGETS))
        self.assertEqual(self.igs.clear_output.call_count, len(OUTPUTS))
        self.assertFalse(self.agent.controller.enabled)

    def test_reset_with_synchronous_enable_callback(self):
        self.igs.input_set_bool.side_effect = self.send
        self.send("on_off", True)
        self.send("reset", None)
        self.assertFalse(self.agent.controller.enabled)

    def test_enable_uses_fresh_time_base(self):
        self.send("roll", 0)
        self.send("rollTarget", 1)
        self.agent.tick(1)
        self.send("on_off", True)
        self.agent.tick(1000)
        self.assertAlmostEqual(self.agent.controller.pids["roll"].integral, 0.025)

    def test_invalid_input_is_logged(self):
        with self.assertLogs("autopilot", level="ERROR"):
            self.send("pitch", float("nan"))

    def test_json_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps({"interval": 0.1, "pitch": {"kp": 0.2}}))
            config = load_config(path)
            self.assertEqual(config.interval, 0.1)
            self.assertEqual(config.pitch.kp, 0.2)
            self.assertEqual(config.roll, Config().roll)
            path.write_text('{"interval": 0}')
            with self.assertRaises(ValueError):
                load_config(path)


if __name__ == "__main__":
    unittest.main()
