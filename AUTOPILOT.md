# PID autopilot

`autopilot.py` handles Ingescape and logging. `controller.py` contains deterministic
control logic, with no external dependencies. The legacy `PID.py` and `echo.py`
are no longer imported.

## Run and tune

With Ingescape installed in your Python environment:

```powershell
python autopilot.py --device "Wi-Fi 2" --config autopilot.example.json
python autopilot.py --config autopilot.example.json --log-level DEBUG --log-file autopilot.log
python -B -m unittest discover -s tests -v
```

The default agent name is `Human_Autopilot`, port 5670, and period 0.025 seconds
(40 Hz). Use `--name`, `--port`, and `--device` to override connection settings.
Edit the JSON to change each axis's gains and integration limit. Omitted axes
retain their defaults; an explicitly supplied axis uses `ki=kd=0` unless supplied.
Configuration is loaded at startup. `--help` and tests work without Ingescape.

INFO logs lifecycle and enable/reset changes. DEBUG logs received values, missing
inputs, and each axis's error, timestep, P/I/D terms and bounded output. File logging
is optional and appends; long debug sessions can produce large files.

## Interface and behavior

| Inputs | Output | Control error |
| --- | --- | --- |
| `pitch`, `pitchTarget`, `altitude`, `altitudeTarget` | `controlPitch` | guided pitch target minus pitch |
| `roll`, `rollTarget`, `heading`, `headingTarget` | `controlRoll` | guided roll target minus roll |
| `slip` | `controlYaw` | zero minus slip |
| `on_off` (boolean) | `on_off` (boolean) | mirrors enable input |
| `reset` (impulsion) | clears state and switches OFF | requires explicit re-enable |

The former `skid` input is now named `slip`, matching Aircraft's output.
All other original numeric inputs remain. Measurements are DOUBLE and targets remain
INTEGER for existing Ingescape mappings. `verticalSpeed`, `verticalSpeedTarget`, `airspeed`, and
`airspeedTarget` are cached/logged only. They do not affect the actuators.
The outputs `thrust`, `parkingBrake`, `alt_sel`, and `heading_sel` are removed.

Input mappings are registered automatically at startup and logged at INFO:

| Local inputs | Source agent | Source outputs |
| --- | --- | --- |
| `heading`, `airspeed`, `altitude`, `verticalSpeed`, `roll`, `pitch`, `slip` | `Aircraft` | Same names as local inputs |
| `headingTarget`, `airspeedTarget`, `altitudeTarget`, `verticalSpeedTarget`, `rollTarget`, `pitchTarget`, `on_off`, `reset` | `Cognitive_Model` | Same names as local inputs |

Agent names use the capitalization shown above, including `Aircraft` for roll.
Mappings for actuator outputs must be defined on the receiving agent.

- Startup is OFF. Inputs may be cached while OFF; no actuator values are published.
- ON computes each axis only when its required measurements/targets have arrived.
  Zero is a valid target. Roll defaults to zero; pitch requires `pitchTarget`
  until altitude hold is active.
- Receiving `on_off=False` immediately publishes zero to `controlPitch`,
  `controlRoll`, and `controlYaw`, even if already OFF. No further PID commands
  are published while OFF. Switching OFF clears PID history and preserves cached
  inputs for re-enable.
- Reset clears all cached numeric inputs, targets, PID history, and commands,
  clears Ingescape numeric input/output values, and sets the local `on_off` input
  and output false. Re-enable and fresh inputs are required. Reset does not change
  configuration. Its local OFF callback also publishes neutral controls.
- Inputs are held until updated or reset; there is no telemetry expiry timeout.
  Invalid/nonfinite input is logged and its cached value discarded.
- Outputs are absolute actuator commands in [-1, 1], not increments. PID uses
  error = target - measurement and actual monotonic elapsed seconds. Integral
  accumulation is bounded and blocked when it would deepen output saturation.
  The first sample after reset/enable has no derivative kick and uses one period.
- Slip control runs continuously toward zero; the old deadband that latched the
  last rudder command is removed. Takeoff actions and the airspeed gate are removed.

## Heading and altitude guidance

Without `headingTarget`, use `rollTarget` if supplied, otherwise zero degrees.
With `headingTarget`, override `rollTarget`: command +15 or -15 degrees of bank
along the shortest heading difference (including across north). Within 2 degrees
of the heading target, command zero bank. Corrections resume if heading drifts
outside this tolerance. Both `bank_angle` and `heading_tolerance` are configurable.
If a heading target exists but heading feedback is missing, roll commands wait.

With `altitudeTarget`, follow `pitchTarget` (for example 10 degrees) until altitude
is within `altitude_tolerance` (default 10 altitude units) or crosses the target
between samples. Then latch altitude hold: an outer altitude PID generates a pitch
correction around `level_pitch` (default 2 degrees), and the existing pitch PID
drives the elevator. The generated pitch is limited to `min_pitch`/`max_pitch`
(defaults -15/+10 degrees). Tune the `altitude` gains and `level_pitch` for the
aircraft; altitude and target must use the same units. No vertical-speed damping
is currently applied, so capture can overshoot and needs simulator tuning.

Altitude hold stays active after capture even if altitude drifts away. A changed
altitude target, OFF/ON, or reset rearms capture; repeated writes of the same target
do not. Before capture, the caller must choose a pitch that approaches the target
(including descent). If altitude feedback is missing while a target exists, pitch
commands wait. Without an altitude target, pitch directly follows `pitchTarget`.
Capture is logged at INFO; effective attitude targets and PID terms at DEBUG.

Pitch/roll/slip gain defaults come from the old script, but these changes still
require simulator tuning. Tests check the math and interface with a fake transport.
An additional real Ingescape callback/reset test runs when the binding is installed
(otherwise it is skipped); it does not start networking. The original adapter was
checked with Ingescape 4.6.8. Tests do not establish aircraft stability or verify
simulator actuator signs.
See the [official Ingescape Python API](https://ingescape.com/ingescape-python/)
for transport setup and callback conventions.
