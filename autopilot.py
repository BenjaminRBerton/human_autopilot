"""Ingescape adapter. Run with --help; importing has no side effects."""

import argparse
import json
import logging
import signal
import threading
import time

from controller import Config, Controller, Gains, MEASUREMENTS, OUTPUTS, TARGETS

log = logging.getLogger(__name__)


def load_config(path):
    if path is None:
        return Config()
    with open(path, encoding="utf-8") as source:
        values = json.load(source)
    for name in ("roll", "pitch", "yaw", "altitude"):
        if name in values:
            values[name] = Gains(**values[name])
    return Config(**values)


class Agent:
    """Serialize callbacks and computation so OFF cannot race with publication."""

    def __init__(self, igs, config):
        self.igs = igs
        self.controller = Controller(config)
        self.lock = threading.RLock()
        self.last_tick = None

    def setup(self, name):
        igs = self.igs
        igs.agent_set_name(name)
        igs.definition_set_version("2.0")
        inputs = {"reset": igs.IMPULSION_T, "on_off": igs.BOOL_T}
        inputs.update({name: igs.DOUBLE_T for name in MEASUREMENTS})
        # Preserve original integer target wire types for existing mappings.
        inputs.update({name: igs.INTEGER_T for name in TARGETS})
        for name, kind in inputs.items():
            igs.input_create(name, kind, False if name == "on_off" else None)
        igs.output_create("on_off", igs.BOOL_T, False)
        for name in OUTPUTS:
            igs.output_create(name, igs.DOUBLE_T, None)
        for name in inputs:
            igs.observe_input(name, self.on_input, None)
            source = "Aircraft" if name in MEASUREMENTS else "Cognitive_Model"
            igs.mapping_add(name, source, name)
            log.info("Mapped %s <- %s.%s", name, source, name)

    def on_input(self, io_type, name, value_type, value, user_data):
        with self.lock:
            try:
                was_enabled = self.controller.enabled
                self.controller.set_input(name, value)
                if name == "reset" or self.controller.enabled != was_enabled:
                    self.last_tick = None
                if name == "reset":
                    self.igs.input_set_bool("on_off", False)
                    self.igs.output_set_bool("on_off", False)
                    for input_name in MEASUREMENTS + TARGETS:
                        self.igs.clear_input(input_name)
                    for output in OUTPUTS:
                        self.igs.clear_output(output)
                elif name == "on_off":
                    if not self.controller.enabled:
                        for output in OUTPUTS:
                            self.igs.output_set_double(output, 0.0)
                        log.info("OFF: pitch, roll and yaw controls set to zero")
                    self.igs.output_set_bool("on_off", self.controller.enabled)
            except ValueError:
                log.exception("Rejected input %s=%r", name, value)

    def tick(self, now):
        with self.lock:
            dt = self.controller.config.interval if self.last_tick is None else now - self.last_tick
            self.last_tick = now
            for name, value in self.controller.step(dt).items():
                self.igs.output_set_double(name, value)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="Wi-Fi 2")
    parser.add_argument("--port", type=int, default=5670)
    parser.add_argument("--name", default="Human_Autopilot")
    parser.add_argument("--config", help="JSON controller parameters")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
    parser.add_argument("--log-file", help="Optional file in addition to console logging")
    args = parser.parse_args(argv)
    handlers = [logging.StreamHandler()]
    if args.log_file:
        handlers.append(logging.FileHandler(args.log_file, encoding="utf-8"))
    logging.basicConfig(level=args.log_level, handlers=handlers,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        config = load_config(args.config)
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    # Tests and --help do not need the runtime dependency installed.
    import ingescape as igs

    agent = Agent(igs, config)
    agent.setup(args.name)
    stop = threading.Event()
    previous_handler = signal.signal(signal.SIGINT, lambda *_: stop.set())
    try:
        if igs.start_with_device(args.device, args.port) != igs.SUCCESS:
            raise RuntimeError(f"Could not start Ingescape on {args.device}:{args.port}")
        log.info("Started OFF on %s:%s; config=%s", args.device, args.port, config)
        while not stop.is_set():
            started = time.monotonic()
            agent.tick(started)
            stop.wait(max(0.0, config.interval - (time.monotonic() - started)))
    finally:
        igs.stop()
        signal.signal(signal.SIGINT, previous_handler)
        log.info("Stopped")


if __name__ == "__main__":
    main()
