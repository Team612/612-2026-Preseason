# Constants and drivetrain tuning

Values below match `src/main/java/frc/robot/Constants.java`. These are software settings, not measured guarantees about the robot. Wheel diameter, gearing, encoder offsets, motor directions, feedforward gains, traction, and stopping distance still require verification on the actual robot. Changes require rebuilding and deploying.

## How the limiting works

Teleop sticks → radial translation deadband / rotation deadband → feasible combined translation and rotation demand → input slew limiting → reference-frame conversion → discretized swerve kinematics → module direction optimization and cosine compensation → feedforward plus bounded speed feedback → voltage cap → independent voltage slew limiter for each module → Talon `VoltageOut`.

Autonomous `drive()` calls skip joystick shaping but still use the module voltage limiter. SysId deliberately bypasses software voltage slew limiting to preserve its requested characterization waveform. Keep Talon ramps zero during characterization, too.

For normal module operation:

```text
requestedVolts = clamp(kS * sign(targetSpeed) + kV * targetSpeed
                       + kA * targetAcceleration + boundedPCorrection,
                       -kMaxDriveVolts, +kMaxDriveVolts)
step = kDriveVoltageSlewRateVoltsPerSecond * min(elapsedSeconds, kDriveVoltageMaxStepSeconds)
commandVolts = previousCommandVolts + clamp(requestedVolts - previousCommandVolts, -step, +step)
```

Time comes from the FPGA clock. Repeated calls at the same timestamp do not accumulate ramp steps. A gap at least as long as `kMotorSafetyExpirationSeconds`, an invalid target, or a backward/nonfinite clock clears voltage history and outputs zero; subsequent valid calls ramp from zero. The voltage cap applies before the ramp so history cannot wind up beyond the cap. Each module has independent history, as required for independent input streams by [WPILib's slew limiter documentation](https://docs.wpilib.org/en/latest/docs/software/advanced-controls/filters/slew-rate-limiter.html).

**Higher positive V/s means faster changes and less smoothing. Lower positive V/s means slower changes and more smoothing. Zero explicitly disables the software ramp; it does not freeze output.** A negative or nonfinite rate is invalid. The same rate applies to increasing and decreasing signed voltage, including reversals. It limits requested voltage, not measured acceleration, current, jerk, or actual motor voltage under battery/current limitations.

| Software voltage rate | Maximum change at 20 ms | Ideal 0 → 12 V time | Ideal +12 → −12 V time |
|---|---:|---:|---:|
| 12 V/s | 0.24 V | 1.00 s | 2.00 s |
| **24 V/s (current)** | **0.48 V** | **0.50 s** | **1.00 s** |
| 48 V/s | 0.96 V | 0.25 s | 0.50 s |
| 0 (disabled) | Entire requested change | Immediate | Immediate |

These times assume a sustained voltage target, regular updates, and no bypass. At the 40 ms timestep cap, the current maximum step is 0.96 V. Reducing the rate can delay braking as well as acceleration; faster input release does not override the downstream voltage ramp.

**Intentional bypasses:** `stop()`, disable/fault paths, wheel pointing/X-brake, module speeds below the stop/start threshold, and effectively zero cosine-compensated speed all command zero drive voltage immediately and reset the limiter. The alignment cutoff avoids continuing to drive a wheel perpendicular to the requested direction. Thus the rate bound is a normal-driving property, not a promise that every zero-output transition is ramped. Zero commanded voltage does not mean instantaneous physical stopping. SysId sends capped raw voltage. Nonzero Talon open-loop ramps can further delay physical output changes, including zero requests.

## Timing and geometry (`Constants`)

| Constant | Current value | Purpose | Increasing / decreasing |
|---|---|---|---|
| `kLoopPeriodSeconds` | 0.02 s | TimedRobot period; steering PID/profile, kinematics discretization, and drive feedforward acceleration interval | Larger: slower control updates. Smaller: more CPU/CAN work. This is a system timing choice, not a smoothing knob; voltage slew uses measured time. |
| `trackWidth` | 0.550 m | Left/right steering-axis spacing | Use the measured spacing. Larger configured spacing increases the modeled wheel speed needed for rotation; an incorrect value distorts kinematics/odometry. |
| `wheelBase` | 0.555 m | Front/rear steering-axis spacing | Same geometry effect as track width, on the other axis. Do not tune to compensate for motor behavior. |
| `swerveKinematics` | Derived from half wheelbase/track width | Module locations in order **front left, front right, rear left, rear right**; +X forward, +Y left | Not a numeric tuning value. Module ordering must match the subsystem. |

## Module wiring and calibration (`Constants`)

IDs select devices; increasing an ID does not increase power. Talon IDs must be unique among Talons and CANcoder IDs among CANcoders on this bus. IDs 0–62 are accepted by the configuration validator; a Talon and CANcoder may share a number because they are different device types.

| Constant | Value | Meaning / effect of changing |
|---|---:|---|
| `frontLeftDriveMotorId` | 1 | Selects front-left drive Talon. Must match hardware. |
| `frontLeftSteerMotorId` | 2 | Selects front-left steer Talon. Must match hardware. |
| `frontLeftEncoderId` | 1 | Selects front-left CANcoder. Must match hardware. |
| `frontLeftDriveInverted` | false | Positive drive output uses counterclockwise-positive motor convention; true selects clockwise-positive. Verify forward wheel travel. |
| `frontLeftSteerInverted` | true | Positive steer output uses clockwise-positive motor convention; false reverses it. Must produce increasing measured module angle. |
| `frontLeftEncoderInverted` | false | CANcoder counterclockwise-positive; true reverses the measured angle and velocity. Recalibrate offset after changing. |
| `frontLeftEncoderOffsetRotations` | −0.284180 rotations | Forward reading after encoder direction correction. Code subtracts this offset: increasing it decreases reported angle by 360° per rotation; decreasing does the reverse. |
| `frontRightDriveMotorId` | 3 | Selects front-right drive Talon. |
| `frontRightSteerMotorId` | 4 | Selects front-right steer Talon. |
| `frontRightEncoderId` | 3 | Selects front-right CANcoder. |
| `frontRightDriveInverted` | true | Clockwise-positive drive; false reverses it. |
| `frontRightSteerInverted` | true | Clockwise-positive steering; false reverses it. |
| `frontRightEncoderInverted` | false | Counterclockwise-positive encoder; true reverses feedback and requires recalibration. |
| `frontRightEncoderOffsetRotations` | 0.364990 rotations | Forward offset subtracted from encoder reading. Increasing decreases reported angle; decreasing increases it. |
| `rearLeftDriveMotorId` | 7 | Selects rear-left drive Talon. |
| `rearLeftSteerMotorId` | 8 | Selects rear-left steer Talon. |
| `rearLeftEncoderId` | 7 | Selects rear-left CANcoder. |
| `rearLeftDriveInverted` | false | Counterclockwise-positive drive; true reverses it. |
| `rearLeftSteerInverted` | true | Clockwise-positive steering; false reverses it. |
| `rearLeftEncoderInverted` | false | Counterclockwise-positive encoder; true reverses feedback and requires recalibration. |
| `rearLeftEncoderOffsetRotations` | 0.226318 rotations | Forward offset subtracted from encoder reading. Increasing decreases reported angle; decreasing increases it. |
| `rearRightDriveMotorId` | 5 | Selects rear-right drive Talon. |
| `rearRightSteerMotorId` | 6 | Selects rear-right steer Talon. |
| `rearRightEncoderId` | 5 | Selects rear-right CANcoder. |
| `rearRightDriveInverted` | true | Clockwise-positive drive; false reverses it. |
| `rearRightSteerInverted` | true | Clockwise-positive steering; false reverses it. |
| `rearRightEncoderInverted` | false | Counterclockwise-positive encoder; true reverses feedback and requires recalibration. |
| `rearRightEncoderOffsetRotations` | −0.289307 rotations | Forward offset subtracted from encoder reading. Increasing decreases reported angle; decreasing increases it. |

With the robot disabled and each wheel physically forward, copy that module's `Calibration/OffsetToCopyWhenForward` only when `Calibration/CanCopyOffset` is true. Offsets are calibration values, not steering gain adjustments.

## Driver input (`OperatorConstants`)

| Constant | Current value | Purpose | Increasing / decreasing |
|---|---|---|---|
| `kDriverControllerPort` | 0 | Driver Station USB joystick slot | Selects a different controller slot; no speed effect. |
| `kDeadband` | 0.06 | Translation stick magnitude / rotation axis neutral threshold; rescales remaining input | Larger: more stick travel before motion, more tolerance of drift. Smaller: finer near-center response but more drift sensitivity. Keep 0 ≤ value < 1. |
| `kForwardSpeedScale` | 1.0 | Forward/reverse speed multiplier | Larger: more speed per stick input until wheel limits; smaller: less speed. Keep positive; frame conversion divides by this scale. |
| `kStrafeSpeedScale` | 1.0 | Left/right speed multiplier | Larger: more strafe speed per input; smaller: less. Keep positive for frame conversion. |
| `kRotationSpeedScale` | 1.0 | Rotation multiplier | Larger: more angular speed per input until wheel limits; smaller: less. Zero disables requested rotation. |
| `kTranslationSlewRatePerSecond` | 3.0 /s | Normalized translation-vector change budget, including reversals | Larger: faster acceleration/direction changes. Smaller: smoother/slower. Alone, full-scale ramp takes about 1/3 s; keep strictly positive. |
| `kRotationSlewRatePerSecond` | 3.0 /s | Normalized rotation change budget, including reversals | Larger: faster spin-up/reversal. Smaller: smoother/slower. Shares accelerating demand budget with translation; keep strictly positive. |
| `kTranslationReleaseSlewRatePerSecond` | 5.0 /s | Same-direction reduction or release of translation | Larger: faster target slowdown. Smaller: longer target slowdown. Alone, full-scale release takes 0.20 s; physical braking also depends on voltage limiting/traction. Keep positive. |
| `kRotationReleaseSlewRatePerSecond` | 5.0 /s | Same-sign reduction or release of rotation | Larger: faster spin target slowdown. Smaller: slower. Reversal uses the normal rate. Keep positive. |
| `kMaxLimiterStepSeconds` | 0.04 s | Caps elapsed time credited to joystick limiting | Larger: larger changes after delays. Smaller: more conservative recovery; below loop period slows effective response. Keep positive. Separate from voltage timestep cap. |

Translation is vector-limited, so diagonals do not get an extra acceleration allowance. Simultaneous accelerating translation and rotation share a budget; same-direction braking uses its own release rate. Feasible demand is capped before history is stored. At current scales, pure translation's normal target acceleration is up to 3 × 2.25 = 6.75 m/s² and release is up to 11.25 m/s²; these are target changes, not measured robot accelerations.

## Drive and hardware (`SwerveConstants`)

| Constant | Current value | Purpose | Increasing / decreasing |
|---|---|---|---|
| `kCanBus` | `CANBus.roboRIO()` | CAN network for all drivetrain devices | Change only to match wiring; not a tuning number. |
| `kPigeonCanId` | 0 | Pigeon2 device address | Must match the gyro; no speed effect. |
| `kWheelBaseMeters` | `wheelBase` = 0.555 m | Geometry alias | Change the source `wheelBase` so kinematics and derived limits remain consistent. |
| `kTrackWidthMeters` | `trackWidth` = 0.550 m | Geometry alias | Change the source `trackWidth`. |
| `kWheelDiameterMeters` | 0.1016 m | Converts wheel rotations to distance/speed | Larger configured diameter increases reported distance/speed for the same encoder motion; smaller decreases them. Use measured effective diameter. |
| `kDriveReduction` | 6.75 motor rotations/wheel rotation | Converts rotor rotations to wheel rotations | Larger configured reduction decreases reported distance/speed; smaller increases them. Must match actual gearing. |
| `kMaxSpeedMetersPerSecond` | 2.25 m/s | Wheel speed cap and driver translation scaling | Larger: faster targets and larger physical target acceleration at unchanged normalized slew. Smaller: slower. Also changes derived angular cap. Not a measured motor free speed. |
| `kMinimumModuleSpeedMetersPerSecond` | 0.03 m/s | Minimum requested module speed to start moving/steering | Larger: ignores more tiny commands. Smaller: more fine response but more twitching. Keep above stop threshold. |
| `kModuleStopSpeedMetersPerSecond` | 0.015 m/s | Lower cutoff while already moving | Larger: stops earlier on small demand. Smaller: continues to lower demand. Together with start threshold prevents chatter. |
| `kMaxAngularSpeedRadPerSec` | min(2π, speed/module radius) ≈ 5.759 rad/s | Driver rotation cap from chassis geometry | Derived; increasing underlying speed raises it until 2π. Larger radius lowers it. Mixed translation/rotation shares wheel capability. |
| `kMaxDriveVolts` | 12.0 V | Absolute software drive voltage cap | Larger: permits stronger effort up to available supply. Smaller: reduces effort and may prevent target speeds. Not a direct speed limit. |
| `kDriveKV` | 2.0 V/(m/s) | Velocity feedforward | Larger: more voltage for a given target speed; excessive value causes overspeed. Smaller: less voltage, more feedback correction needed. Measure with SysId. |
| `kDriveKS` | 0.0 V | Signed static-friction feedforward | Larger positive: more breakaway effort, possibly jerky starts. Smaller: less breakaway effort. Zero omits it; measure before enabling. |
| `kDriveKA` | 0.0 V/(m/s²) | Feedforward for target acceleration | Larger: stronger effort during target changes, including braking; may saturate. Smaller: less transient effort. Target acceleration currently uses nominal loop period. Measure before enabling. |
| `kDriveVoltageSlewRateVoltsPerSecond` | 24.0 V/s | Final normal-drive voltage change bound | Higher positive: faster response, less smoothing. Lower positive: slower response and potentially longer braking. Zero disables ramp. Negative/nonfinite invalid. |
| `kDriveVoltageMaxStepSeconds` | 0.04 s | Caps measured time credited to one voltage update | Larger: permits larger voltage changes after a delayed loop. Smaller: more conservative; below 0.02 s slows nominal response. Must be positive and below watchdog expiry. |
| `kDriveOpenLoopRampSeconds` | 0.0 s | Talon voltage ramp used by drive `VoltageOut` | Higher: slower hardware ramp, opposite to V/s. Zero disables. Adds limiting after software ramp and affects SysId; leave zero when tuning software ramp. |
| `kDriveClosedLoopRampSeconds` | 0.0 s | Talon onboard closed-loop voltage ramp | **No effect with this repository's `VoltageOut` requests**, even though software performs velocity feedback. With onboard closed-loop control, higher means slower. |
| `kTurnOpenLoopRampSeconds` | 0.0 s | Talon voltage ramp used by steering `VoltageOut` | Higher: slower steering voltage response and braking, can worsen tracking. Zero disables. |
| `kTurnClosedLoopRampSeconds` | 0.0 s | Talon onboard closed-loop steering voltage ramp | **No effect with current steering `VoltageOut` requests.** Higher would mean slower under onboard closed-loop control. |
| `kDriveP` | 2.0 V/(m/s) | Proportional correction for target minus measured wheel speed | Larger: corrects error harder, may oscillate or saturate. Smaller: gentler but more speed error. Zero removes P contribution. |
| `kMaxDriveCorrectionVolts` | 2.75 V | Cap on magnitude of P correction | Larger: more feedback authority and voltage disturbances before final ramp. Smaller: less correction; may leave steady error. Zero removes correction. |
| `kStoppedSpeedMetersPerSecond` | 0.05 m/s | Measured wheel-speed threshold for heading reset/SysId start | Larger: treats more motion as stopped. Smaller: requires closer to rest and can be sensitive to noise. Requested chassis speeds must also be zero. |
| `kEnableDriveVelocityCorrection` | true | Enables software drive P correction | False leaves feedforward plus caps/ramp. Drive sensors are still required for module readiness and odometry; false is not a sensor-fault override. |

CTRE defines the open-loop voltage ramp in seconds from 0 to 12 V for `VoltageOut`; see [OpenLoopRampsConfigs](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/configs/OpenLoopRampsConfigs.html). For example, 0.5 s corresponds to 24 V/s, but hardware ramping and this software limiter have different reset/bypass behavior. Avoid stacking ramps without a measured reason.

## Steering, feedback, and current limits (`SwerveConstants`)

| Constant | Current value | Purpose | Increasing / decreasing |
|---|---|---|---|
| `kTurnP` | 0.5 V/rad | Proportional steering correction to profiled angle | Larger: stronger tracking, possible overshoot/oscillation. Smaller: weaker/slower tracking. |
| `kTurnI` | 0.0 V/(rad·s) | Integral steering correction | Larger: accumulates effort against sustained error, with windup/overshoot risk. Zero disables; history resets inside tolerance and on stops/replans. |
| `kTurnD` | 0.0 V/(rad/s) | Derivative of steering profile error | Larger: more rate-error response and noise sensitivity. Smaller: less. Separate from measured velocity damping below. |
| `kTurnVelocityDamping` | 0.15 V/(rad/s) | Tracks profile velocity; brakes measured steering motion within angle tolerance | Larger: stronger velocity tracking/braking, possibly noisy or saturated. Smaller: weaker damping and more overshoot. |
| `kTurnSettledVelocityRadPerSec` | 0.1 rad/s | Steering rate considered settled inside tolerance | Larger: permits neutral with more residual motion. Smaller: damps closer to rest, potentially reacts to noise. |
| `kTurnOptimizationHysteresisRad` | 5° ≈ 0.08727 rad | Buffer around 90° when deciding whether to reverse wheel direction | Larger: fewer reversal toggles but more steering travel. Smaller: quicker optimization switches but more chatter. Current thresholds: enter reversed above 95°, leave at/below 85°. |
| `kTurnToleranceDegrees` | 2.7° | Final angular error accepted as aligned | Larger: less hunting, worse alignment accuracy. Smaller: tighter alignment but more twitching. Not encoder resolution. |
| `kTurnToleranceRad` | Derived ≈ 0.04712 rad | Radian form of tolerance | Change `kTurnToleranceDegrees`; do not independently change this conversion. |
| `kCancoderCountsPerRotation` | 4096 | Software quantization of corrected angle | Larger: finer rounding. Smaller: coarser angle steps. Cannot increase physical sensor resolution; not a steering tolerance. |
| `kSensorUpdateFrequencyHz` | 100 Hz | Requested critical CAN status update frequency | Larger: fresher feedback but more bus load. Smaller: less traffic but older feedback. Coordinate with stale timeout. |
| `kMaxSensorAgeSeconds` | 0.1 s | Maximum accepted signal latency | Larger: tolerates older data and delays stale-feedback shutdown. Smaller: detects missing updates sooner but may cause nuisance stops. |
| `kMotorSafetyExpirationSeconds` | 0.1 s | Motor watchdog timeout; teleop loop-gap cutoff; voltage-history gap cutoff | Larger: tolerates longer control gaps but delays response to lost updates. Smaller: quicker cutoff but more nuisance resets. Must exceed voltage maximum timestep. |
| `kTurnToleranceHysteresisRad` | 1° ≈ 0.01745 rad | Extra error allowed before leaving the within-tolerance state | Larger: less chatter but larger accepted drift. Smaller: tighter restart threshold. Current restart boundary is 3.7°. |
| `kMaxTurnRadPerSec` | 2π ≈ 6.283 rad/s | Steering profile velocity limit | Larger: faster angle targets and more tracking effort. Smaller: slower turning. Actual speed depends on gains, voltage, load. |
| `kMaxTurnAccelRadPerSecSquared` | 4π ≈ 12.566 rad/s² | Steering profile acceleration limit | Larger: faster profile acceleration/reversal. Smaller: gentler but slower steering. |
| `kMaxTurnVolts` | 12.0 V | Steering voltage magnitude cap | Larger: more available steering effort up to supply. Smaller: softer but may lag/stall. Does not itself increase PID gains. |
| `kDriveSupplyLimitAmps` | 40 A per motor | Drive current drawn from supply | Larger: allows more electrical power/battery loading. Smaller: reduces battery load but can limit performance. |
| `kDriveStatorLimitAmps` | 60 A per motor | Drive motor winding current/torque limit | Larger: more launch torque, traction demand, and heating. Smaller: less torque and potentially less wheel slip. |
| `kTurnSupplyLimitAmps` | 25 A per motor | Steering supply current limit | Larger: more available power/battery load. Smaller: lower power, possible slower steering. |
| `kTurnStatorLimitAmps` | 40 A per motor | Steering winding current/torque limit | Larger: more steering torque/heating. Smaller: reduced torque; may struggle under load. |

These are per-motor current limits, not whole-robot budgets. Current limiting and available supply can make measured applied voltage differ from the software request. The code enables both supply and stator limits and configures the secondary supply limit to equal the primary limit.

## Module configuration bundles (`SwerveConstants`)

| Constant | Current contents | Effect of changing |
|---|---|---|
| `kFrontLeft` | Drive 1, steer 2, encoder 1; offset −0.284180; inverted false/true/false | Bundles the front-left constants above; changing it remaps/calibrates that physical module. |
| `kFrontRight` | Drive 3, steer 4, encoder 3; offset 0.364990; inverted true/true/false | Bundles front-right settings. |
| `kRearLeft` | Drive 7, steer 8, encoder 7; offset 0.226318; inverted false/true/false | Bundles rear-left settings; third in kinematics order. |
| `kRearRight` | Drive 5, steer 6, encoder 5; offset −0.289307; inverted true/true/false | Bundles rear-right settings; fourth in kinematics order. |

`ModuleConfiguration` is a record, not an additional tuning constant. Its fields are `driveCanId`, `turnCanId`, `encoderCanId`, `encoderOffsetRotations`, `driveInverted`, `turnInverted`, and `encoderInverted`, with the meanings above. Its ID validation checks range; `Swerve` checks duplicates.

## Tuning and verification

1. Verify wiring, motor/encoder direction, wheel geometry, and offsets before changing gains. A larger gain cannot fix reversed feedback.
2. Keep Talon ramps at zero to evaluate the software limiter independently. Keep the current 24 V/s as an initial comparison point, not a universally optimal value.
3. Observe each module's `RequestedDriveVolts`, `DriveVolts`, `DriveAppliedVolts`, `DriveVoltageSlewLimited`, `MeasuredSpeedMetersPerSecond`, current limits, and supply voltage. `DriveVolts` is the software command; `DriveAppliedVolts` is the motor's measured voltage. Phoenix also logs requested/command voltage each control loop.
4. Change one parameter at a time. If target response is too sharp, reduce the relevant positive slew rate; if it is too sluggish, increase it while checking traction, current, and stopping distance. Verify forward/reverse, combined translation/rotation, release, direction changes, mode switches, and restart after stop.
5. Identify feedforward on the actual drivetrain before enabling nonzero `kDriveKS`/`kDriveKA`. Characterization uses test mode and raw capped voltages; software ramping must not reshape those experiments. Tune P only after calibration/feedforward are credible.
6. Run `./gradlew test` and `./gradlew build` before deploying. Tests cover limiter timing/reversals/reset/gaps and simulated command/module behavior. Simulation here does not model tire traction, robot mass, real stopping distance, or brownout dynamics; those need floor tests.
