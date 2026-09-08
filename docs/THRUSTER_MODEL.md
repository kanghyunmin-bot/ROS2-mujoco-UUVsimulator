# T200 / Basic ESC actuator model

The September 7, 2026 correction replaces the default SITL polynomial/gain
mapping with the measured T200 PWM/voltage surface. The owner confirmed T200
thrusters and Basic ESCs. Actual ESC bus voltage is still unconfirmed.

## Force path

1. Final JSON servo PWM is normalized using 1100/1500/1900 us and the existing,
   verified channel/direction mapping. No extra ArduSub thrust-expo inversion is
   applied: the input is already final motor PWM.
2. A command limit caps this input without stretching it to full output. Basic
   ESC commands within 1500 +/-25 us request stop.
3. Effective drive follows a first-order response. During reversal it uses the
   deceleration time constant until zero, then the acceleration time constant.
   This is a low-order transient approximation, not a motor electrical/RPM model.
   The existing 0.04/0.06 s constants remain uncalibrated priors.
4. The resulting PWM indexes the measured forward/reverse force curve in N.
   Linear interpolation across measured voltages preserves the 10/12/14/16/18/20 V
   knots. Unsupported voltages fail explicitly; the synthetic 22.2 V curve is
   excluded. Missing data fails instead of silently selecting another model.
5. Explicit mounting gain, immersion, and the optional inflow model act on force.
   The default mounting gains are unity. The uncalibrated inflow correction is
   disabled for distributed profiles; in the old log it multiplied force by
   0.45--1.20. There is no additional yaw moment amplification in these profiles.
6. MuJoCo applies the site actuator force once; moment follows the site position
   and direction. Integration tests compare the diagnostic wrench with MuJoCo's
   generalized actuator force.

The former horizontal 3.3 and vertical 2.425 speed-fit gains are retired. The
explicit `UUV_SITL_THRUSTER_FORCE_MODEL=polynomial` fallback uses unity gains and
the existing 21 N reference; it is not a calibrated physical actuator model.

## Battery voltage

`thruster_voltage` denotes voltage at the ESC power terminals, not pack nominal
voltage. The existing profile value **20 V is an upper measured reference
assumption**, not a confirmed vehicle measurement. No battery state of charge,
internal resistance, wiring drop, or regulator is invented. SITL battery
telemetry is synthetic and is not used as a physical voltage measurement.

Launch the existing Docker GUI using a known terminal voltage, for example:

```bash
# 16 V below is an example, not an estimate of this robot's battery.
UUV_THRUSTER_BUS_VOLTAGE_V=16 ./docker/ubuntu-dev/dev.sh web
```

For measured voltage sag, supply a CSV with increasing `sim_time,voltage_v`
columns starting at simulation time zero. At least two samples are required.
The runtime linearly interpolates samples and holds the endpoint voltages.
All samples must lie within the measured curve range. The trace already includes
terminal voltage drop; no second sag correction is applied.

```bash
UUV_THRUSTER_VOLTAGE_TRACE=/workspace/measurements/esc_voltage.csv ./docker/ubuntu-dev/dev.sh web
```

Equivalent runner arguments are `--thruster-voltage 16` and
`--thruster-voltage-trace /path/to/esc_voltage.csv`. Restart the simulation stack
after changing its voltage configuration. Do not treat a pack voltage above
20 V as a supported T200 measurement by silently clamping it or extrapolating.

## Evidence and remaining identification

The log `gui_start_stack_20260906_153237_thrusters.csv` contains 6,863 uninterrupted
100 Hz samples. It reaches 285 deg/s yaw and 40.92 Nm yaw moment. Its old static
horizontal mapping was 3.56 N at 1600 us and 69.3 N at 1900 us, whereas the
measured 16 V curve gives 5.92 N and 51.44 N. A uniform gain cannot correct both.

The accompanying tlog shows neutral RC in armed Stabilize with a horizontal
servo at an endpoint in 35 of 53 eligible low-rate servo samples. This establishes
controller saturation in the simulator, not an identified cause of the real
vehicle response. The correction does not claim that Stabilize oscillation is
resolved; controller gains, feedback delay, hull damping/inertia and mounted
transient behavior still require comparison after the actuator contract is set.

`tools/audit_recorded_thruster_response.py` audits the 100 Hz evidence and runs
the identical PWM through the production actuator code at fixed submerged
geometry. Its voltage cases are sensitivity comparisons, not closed-loop
predictions. `tools/test_t200_actuator_physics.py` checks data knots, neutral,
limits, voltage traces, reversal, gain ownership and MuJoCo force application.

Sources: [T200 data and specifications](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/),
[Basic ESC PWM specifications](https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/).
