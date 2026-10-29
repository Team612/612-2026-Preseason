package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

// warning, comments here are ai generated (this is the only file with that problem)

// Controls one wheel and its steering.
public class SwerveModule {
  // Dashboard and fault message
  private final String name;
  // CAN IDs, directions, and forward offset.
  private final Constants.ModuleConfiguration configuration;
  // Drive motor; null if hardware is disabled.
  private final TalonFX driveMotor;
  // Steering motor; null if hardware is disabled.
  private final TalonFX turnMotor;
  // Steering encoder; null if hardware is disabled.
  private final CANcoder encoder;
  // Cleared after configuration n reset
  private boolean configurationSuccessful;
  // Disabled configuration retry
  private double nextConfigurationRetrySeconds;
  // Reusable drive voltage request.
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  // Reusable steering voltage request.
  private final VoltageOut turnVoltageRequest = new VoltageOut(0.0);
  // Steering profile uses radians and rad/s.
  private final ProfiledPIDController turnController = new ProfiledPIDController(
      Constants.SwerveConstants.kTurnP, Constants.SwerveConstants.kTurnI,
      Constants.SwerveConstants.kTurnD,
      new TrapezoidProfile.Constraints(Constants.SwerveConstants.kMaxTurnRadPerSec,
          Constants.SwerveConstants.kMaxTurnAccelRadPerSecSquared), Constants.kLoopPeriodSeconds);
  // Previous target for replanning and telemetry.
  private Rotation2d lastTargetAngle = new Rotation2d();
  // Tracks whether the profile has been initialized.
  private boolean turnControllerInitialized;
  // Keeps tolerance hysteresis between loops.
  private boolean turnWithinTolerance;
  // Keeps wheel direction stable near 90 degrees.
  private boolean turnOptimizationReversed;
  // Counts steering profile replans.
  private int turnProfileReplanCount;
  // Last steering rate; NaN if unavailable.
  private double turnRateRadPerSec = Double.NaN;
  // Inertia can cause this; it does not prove incorrect inversion.
  private boolean turnMovingAwayFromTarget;
  // Uses a lower stop threshold once moving.
  private boolean moving;
  // Wheel speed after alignment compensation.
  private double targetSpeedMetersPerSecond;
  // Normalized wheel speed for telemetry.
  private double driveSpeedScale;
  // Drive feedforward voltage.
  private double driveBaseVolts;
  private double previousDriveTarget;
  // Limited drive feedback correction.
  private double driveCorrectionVolts;
  // Last capped drive voltage.
  private double driveVolts;
  // Last capped steering voltage.
  private double turnVolts;

  // Creates hardware and enables motor watchdogs.
  public SwerveModule(
      String name, Constants.ModuleConfiguration configuration, boolean hardwareEnabled) {
    this.name = name;
    this.configuration = configuration;
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    if (hardwareEnabled && configuration != null && configuration.hasValidIds()) {
      driveMotor = new TalonFX(configuration.driveCanId(), Constants.SwerveConstants.kCanBus);
      turnMotor = new TalonFX(configuration.turnCanId(), Constants.SwerveConstants.kCanBus);
      encoder = new CANcoder(configuration.encoderCanId(), Constants.SwerveConstants.kCanBus);
      configurationSuccessful = configureHardware();
      // Watchdogs stop motors if control requests stop.

      driveMotor.setExpiration(Constants.SwerveConstants.kMotorSafetyExpirationSeconds);
      turnMotor.setExpiration(Constants.SwerveConstants.kMotorSafetyExpirationSeconds);
      driveMotor.setSafetyEnabled(true);
      turnMotor.setSafetyEnabled(true);
      nextConfigurationRetrySeconds = Timer.getFPGATimestamp() + 1.0;
    } else {
      driveMotor = null;
      turnMotor = null;
      encoder = null;
      configurationSuccessful = false;
    }
  }

  // Configures hardware at startup or during a disabled retry.
  private boolean configureHardware() {
    StatusCode driveStatus =
        driveMotor.getConfigurator().apply(motorConfig(true));
    StatusCode turnStatus =
        turnMotor.getConfigurator().apply(motorConfig(false));
    CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
    encoderConfiguration.MagnetSensor.MagnetOffset = 0.0;
    if (configuration.encoderInverted()) {
      encoderConfiguration.MagnetSensor.SensorDirection =
          SensorDirectionValue.Clockwise_Positive;
    } else {
      encoderConfiguration.MagnetSensor.SensorDirection =
          SensorDirectionValue.CounterClockwise_Positive;
    }
    encoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    StatusCode encoderStatus = encoder.getConfigurator().apply(encoderConfiguration);
    StatusCode signalStatus = BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.SwerveConstants.kSensorUpdateFrequencyHz,
        encoder.getAbsolutePosition(), encoder.getPosition(), encoder.getVelocity(),
        driveMotor.getMotorVoltage(), turnMotor.getMotorVoltage(),
        encoder.getFault_BadMagnet(),
        driveMotor.getPosition(), driveMotor.getVelocity(),
        turnMotor.getPosition());
    turnControllerInitialized = false;
    turnWithinTolerance = false;
    turnOptimizationReversed = false;
    driveMotor.hasResetOccurred();
    turnMotor.hasResetOccurred();
    encoder.hasResetOccurred();
    boolean success = driveStatus.isOK() && turnStatus.isOK()
        && encoderStatus.isOK() && signalStatus.isOK();

    if (!success) {
      DriverStation.reportError(name + " configuration failed: drive=" + driveStatus
          + ", turn=" + turnStatus + ", encoder=" + encoderStatus
          + ", signals=" + signalStatus, false);
    }
    return success;
  }

  // Sets motor direction, brake mode, and current limits.
  private TalonFXConfiguration motorConfig(boolean isDriveMotor) {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    if (isDriveMotor) {
      motorConfiguration.CurrentLimits.SupplyCurrentLimit =
          Constants.SwerveConstants.kDriveSupplyLimitAmps;
    } else {
      motorConfiguration.CurrentLimits.SupplyCurrentLimit =
          Constants.SwerveConstants.kTurnSupplyLimitAmps;
    }
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    if (isDriveMotor) {
      motorConfiguration.CurrentLimits.StatorCurrentLimit =
          Constants.SwerveConstants.kDriveStatorLimitAmps;
    } else {
      motorConfiguration.CurrentLimits.StatorCurrentLimit =
          Constants.SwerveConstants.kTurnStatorLimitAmps;
    }
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    boolean inverted;
    if (isDriveMotor) {
      inverted = configuration.driveInverted();
    } else {
      inverted = configuration.turnInverted();
    }
    if (inverted) {
      motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    motorConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = isDriveMotor
        ? Constants.SwerveConstants.kDriveOpenLoopRampSeconds
        : Constants.SwerveConstants.kTurnOpenLoopRampSeconds;
    motorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = isDriveMotor
        ? Constants.SwerveConstants.kDriveClosedLoopRampSeconds
        : Constants.SwerveConstants.kTurnClosedLoopRampSeconds;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return motorConfiguration;
  }

  // Commands wheel speed and angle. False means stop the drivetrain.
  public boolean setState(SwerveModuleState targetState) {
    if (targetState == null || targetState.angle == null
        || !Double.isFinite(targetState.speedMetersPerSecond)
        || !Double.isFinite(targetState.angle.getRadians())) {
      stop();
      return false;
    }
    Rotation2d angle = getAngle();
    if (DriverStation.isDisabled() || !isReady() || angle == null
        || !isPositiveFinite(Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
        || !isPositiveFinite(Constants.SwerveConstants.kMaxDriveVolts)
        || !Double.isFinite(Constants.SwerveConstants.kDriveKV)
        || Constants.SwerveConstants.kDriveKV < 0.0
        || !isPositiveFinite(Constants.SwerveConstants.kMaxTurnVolts)) {
      stop();
      return false;
    }
    if (!turnControllerInitialized) {
      resetTurnController(angle);
      turnWithinTolerance = false;
      lastTargetAngle = angle;
      turnOptimizationReversed = false;
      turnControllerInitialized = true;
    }

    double speedThreshold = Constants.SwerveConstants.kMinimumModuleSpeedMetersPerSecond;
    if (moving) {
      speedThreshold = Constants.SwerveConstants.kModuleStopSpeedMetersPerSecond;
    }
    if (Math.abs(targetState.speedMetersPerSecond) < speedThreshold) {
      stop();
      return configurationSuccessful;
    }
    moving = true;

    boolean previouslyReversed = turnOptimizationReversed;
    SwerveModuleState moduleState = optimizeState(targetState, angle);
    double previousError = lastTargetAngle.minus(angle).getRadians();
    double nextError = moduleState.angle.minus(angle).getRadians();
    double targetChange = moduleState.angle.minus(lastTargetAngle).getRadians();
    boolean targetCrossedWheel = previousError * nextError < 0.0
        && Math.abs(targetChange) > Constants.SwerveConstants.kTurnToleranceRad;
    if (previouslyReversed != turnOptimizationReversed || targetCrossedWheel) {
      // Restart the profile when a changed target reverses the turn.

      // Overshoot alone does not trigger a reset.
      resetTurnController(angle);
      turnWithinTolerance = false;
      turnProfileReplanCount++;
    }
    targetSpeedMetersPerSecond = moduleState.speedMetersPerSecond;
    lastTargetAngle = moduleState.angle;
    if (!setAngle(angle, moduleState.angle)) {
      return false;
    }
    return setSpeed(moduleState.speedMetersPerSecond);
  }

  /** Steers at zero drive speed, bypassing the normal low-speed steering hold. */
  public boolean pointAt(Rotation2d target) {
    if (DriverStation.isDisabled() || !isReady() || target == null
        || !Double.isFinite(target.getRadians())) {
      stop();
      return false;
    }
    Rotation2d angle = getAngle();
    if (!turnControllerInitialized) {
      resetTurnController(angle);
      turnControllerInitialized = true;
    }
    moving = false;
    previousDriveTarget = 0.0;
    targetSpeedMetersPerSecond = 0.0;
    driveSpeedScale = driveBaseVolts = driveCorrectionVolts = driveVolts = 0.0;
    lastTargetAngle = target;
    return acceptControlStatus(driveMotor.setControl(driveVoltageRequest.withOutput(0.0)))
        && setAngle(angle, target);
  }

  /** Raw voltage characterization, with the same health checks and output caps as driving. */
  public boolean characterize(boolean steer, double volts) {
    if (!DriverStation.isTestEnabled() || !isReady() || !Double.isFinite(volts)) {
      stop();
      return false;
    }
    if (steer) {
      stop();
      turnVolts = MathUtil.clamp(volts, -Constants.SwerveConstants.kMaxTurnVolts,
          Constants.SwerveConstants.kMaxTurnVolts);
      return acceptControlStatus(turnMotor.setControl(turnVoltageRequest.withOutput(turnVolts)));
    }
    if (!pointAt(Rotation2d.kZero)) {
      return false;
    }
    driveVolts = MathUtil.clamp(volts, -Constants.SwerveConstants.kMaxDriveVolts,
        Constants.SwerveConstants.kMaxDriveVolts);
    return acceptControlStatus(driveMotor.setControl(driveVoltageRequest.withOutput(driveVolts)));
  }

  public boolean isPointedForward() {
    Rotation2d angle = getAngle();
    return angle != null && Math.abs(angle.getRadians()) <= Constants.SwerveConstants.kTurnToleranceRad
        && Math.abs(getTurnRateRadPerSec()) <= Constants.SwerveConstants.kTurnSettledVelocityRadPerSec;
  }

  public SwerveModuleState getState() {
    Rotation2d angle = getAngle();
    double speed = getSpeed();
    return angle == null || !Double.isFinite(speed) ? null : new SwerveModuleState(speed, angle);
  }

  public SwerveModuleState getTargetState() {
    return new SwerveModuleState(targetSpeedMetersPerSecond, lastTargetAngle);
  }

  /** Logs actual applied voltage, SI wheel travel, and unwrapped encoder angle for SysId. */
  public void logCharacterization(SysIdRoutineLog log, boolean steer) {
    if (!isReady()) {
      return;
    }
    if (steer) {
      double voltage = turnMotor.getMotorVoltage().getValueAsDouble();
      double angle = encoder.getPosition().getValueAsDouble() * 2.0 * Math.PI;
      double velocity = getTurnRateRadPerSec();
      log.motor(name + "Steer").voltage(Volts.of(voltage))
          .angularPosition(Radians.of(angle)).angularVelocity(RadiansPerSecond.of(velocity));
      SignalLogger.writeDouble(name + "/SysId/Voltage", voltage, "V");
      SignalLogger.writeDouble(name + "/SysId/Position", angle, "rad");
      SignalLogger.writeDouble(name + "/SysId/Velocity", velocity, "rad/s");
    } else {
      SwerveModulePosition position = getPosition();
      if (position == null) {
        return;
      }
      double voltage = driveMotor.getMotorVoltage().getValueAsDouble();
      log.motor(name + "Drive").voltage(Volts.of(voltage))
          .linearPosition(Meters.of(position.distanceMeters))
          .linearVelocity(MetersPerSecond.of(getSpeed()));
      SignalLogger.writeDouble(name + "/SysId/Voltage", voltage, "V");
      SignalLogger.writeDouble(name + "/SysId/Position", position.distanceMeters, "m");
      SignalLogger.writeDouble(name + "/SysId/Velocity", getSpeed(), "m/s");
    }
  }

  // Reduces steering travel and slows misaligned wheels.
  private SwerveModuleState optimizeState(
      SwerveModuleState requestedState, Rotation2d measuredAngle) {
    double error = Math.abs(requestedState.angle.minus(measuredAngle).getRadians());
    double hysteresis = Constants.SwerveConstants.kTurnOptimizationHysteresisRad;
    double threshold = Math.PI / 2.0 + hysteresis;
    if (turnOptimizationReversed) {
      threshold = Math.PI / 2.0 - hysteresis;
    }
    turnOptimizationReversed = error > threshold;
    SwerveModuleState optimizedState =
        new SwerveModuleState(requestedState.speedMetersPerSecond, requestedState.angle);
    if (turnOptimizationReversed) {
      optimizedState.speedMetersPerSecond *= -1.0;
      optimizedState.angle = optimizedState.angle.rotateBy(Rotation2d.kPi);
    }
    optimizedState.speedMetersPerSecond *=
        Math.max(0.0, optimizedState.angle.minus(measuredAngle).getCos());
    return optimizedState;
  }

  // Applies steering voltage; rejects invalid feedback or requests.
  private boolean setAngle(Rotation2d angle, Rotation2d targetAngle) {
    double turnRate = getTurnRateRadPerSec();
    turnRateRadPerSec = turnRate;
    if (!Double.isFinite(turnRate)) {
      stop();
      return false;
    }
    double targetError = targetAngle.minus(angle).getRadians();
    // Inertia or an external push can also cause this.
    turnMovingAwayFromTarget = Math.abs(targetError)
            > Constants.SwerveConstants.kTurnToleranceRad
        && Math.abs(turnRate) > Constants.SwerveConstants.kTurnSettledVelocityRadPerSec
        && targetError * turnRate < 0.0;
    double output = calculateTurnOutput(angle, targetAngle, turnRate);
    if (!Double.isFinite(output)) {
      stop();
      return false;
    }
    turnVolts =
        finiteOrZero(
            MathUtil.clamp(
                output,
                -Constants.SwerveConstants.kMaxTurnVolts,
                Constants.SwerveConstants.kMaxTurnVolts));
    return acceptControlStatus(turnMotor.setControl(turnVoltageRequest.withOutput(turnVolts)));
  }

  // Tracks the target and brakes within tolerance.
  private double calculateTurnOutput(Rotation2d angle, Rotation2d targetAngle, double turnRate) {
    // Use wrapped error to the final target.
    double tolerance = Constants.SwerveConstants.kTurnToleranceRad;
    if (turnWithinTolerance) {
      tolerance += Constants.SwerveConstants.kTurnToleranceHysteresisRad;
    }
    turnWithinTolerance = Math.abs(targetAngle.minus(angle).getRadians()) <= tolerance;
    if (turnWithinTolerance) {
      // Clear the old profile and brake remaining motion.

      turnController.reset(angle.getRadians(), turnRate);
      if (Math.abs(turnRate) <= Constants.SwerveConstants.kTurnSettledVelocityRadPerSec) {
        return 0.0;
      }
      return -Constants.SwerveConstants.kTurnVelocityDamping * turnRate;
    }
    double output = turnController.calculate(angle.getRadians(), targetAngle.getRadians());
    return output + Constants.SwerveConstants.kTurnVelocityDamping
        * (turnController.getSetpoint().velocity - turnRate);
  }

  // Applies drive feedforward, speed correction, and voltage limits.
  private boolean setSpeed(double targetSpeed) {
    driveSpeedScale =
        MathUtil.clamp(
            targetSpeed / Constants.SwerveConstants.kMaxSpeedMetersPerSecond, -1.0, 1.0);
    double acceleration = (targetSpeed - previousDriveTarget) / Constants.kLoopPeriodSeconds;
    previousDriveTarget = targetSpeed;
    driveBaseVolts = Math.signum(targetSpeed) * Constants.SwerveConstants.kDriveKS
        + targetSpeed * Constants.SwerveConstants.kDriveKV
        + acceleration * Constants.SwerveConstants.kDriveKA;
    if (!Double.isFinite(driveBaseVolts)) {
      stop();
      return false;
    }
    driveCorrectionVolts = 0.0;
    if (Math.abs(targetSpeed) < 1e-6) {
      driveVolts = 0.0;
      return acceptControlStatus(driveMotor.setControl(driveVoltageRequest.withOutput(0.0)));
    }
    if (Constants.SwerveConstants.kEnableDriveVelocityCorrection) {
      double measuredSpeed = getSpeed();
      if (!Double.isFinite(measuredSpeed)
          || !Double.isFinite(Constants.SwerveConstants.kDriveP)
          || Constants.SwerveConstants.kDriveP < 0.0
          || !Double.isFinite(Constants.SwerveConstants.kMaxDriveCorrectionVolts)
          || Constants.SwerveConstants.kMaxDriveCorrectionVolts < 0.0) {
        // Stop if drive feedback disappears.
        stop();
        return false;
      }
      driveCorrectionVolts = MathUtil.clamp(
          Constants.SwerveConstants.kDriveP * (targetSpeed - measuredSpeed),
          -Constants.SwerveConstants.kMaxDriveCorrectionVolts,
          Constants.SwerveConstants.kMaxDriveCorrectionVolts);
    }
    driveVolts =
        finiteOrZero(
            MathUtil.clamp(
                driveBaseVolts + driveCorrectionVolts,
                -Constants.SwerveConstants.kMaxDriveVolts,
                Constants.SwerveConstants.kMaxDriveVolts));
    return acceptControlStatus(driveMotor.setControl(driveVoltageRequest.withOutput(driveVolts)));
  }

  // Stops the module if a motor rejects its request.
  private boolean acceptControlStatus(StatusCode status) {
    if (recordControlStatus(status)) {
      return true;
    }
    stop();
    return false;
  }

  // Records a rejected request and schedules a disabled retry.
  private boolean recordControlStatus(StatusCode status) {
    if (status.isOK()) {
      return true;
    }
    if (configurationSuccessful) {
      DriverStation.reportError(name + " motor request failed: " + status
          + "; disable to reconfigure.", false);
      // Start the retry timer only once.
      nextConfigurationRetrySeconds = Timer.getFPGATimestamp() + 1.0;
    }
    configurationSuccessful = false;
    return false;
  }

  // Returns wheel travel for odometry, or null for invalid feedback.
  public SwerveModulePosition getPosition() {
    Rotation2d angle = getAngle();
    if (angle == null || driveMotor == null || !validWheelConfig()) {
      return null;
    }
    var position = driveMotor.getPosition();
    double motorRotations = position.getValueAsDouble();
    if (!isFresh(position) || !Double.isFinite(motorRotations)) {
      return null;
    }
    double distanceMeters =
        motorRotations
            / Constants.SwerveConstants.kDriveReduction
            * (Math.PI * Constants.SwerveConstants.kWheelDiameterMeters);
    if (!Double.isFinite(distanceMeters)) {
      return null;
    }
    return new SwerveModulePosition(distanceMeters, angle);
  }

  // Checks drive feedback and wheel settings.
  private boolean speedReady() {
    if (driveMotor == null || !validWheelConfig()) {
      return false;
    }
    var velocity = driveMotor.getVelocity();
    return isFresh(velocity) && Double.isFinite(velocity.getValueAsDouble());
  }

  // Detects resets and retries configuration while disabled.
  public void periodic() {
    if (driveMotor == null) {
      return;
    }
    boolean deviceReset = driveMotor.hasResetOccurred();
    // Read every reset flag; each read clears it.
    if (turnMotor.hasResetOccurred()) {
      deviceReset = true;
    }
    if (encoder.hasResetOccurred()) {
      deviceReset = true;
    }
    if (deviceReset) {
      configurationSuccessful = false;
      nextConfigurationRetrySeconds = Timer.getFPGATimestamp() + 1.0;
      stop();
      DriverStation.reportWarning(name + " device restarted; disable to reconfigure.", false);
    }
    if (DriverStation.isDisabled() && !configurationSuccessful
        && Timer.getFPGATimestamp() >= nextConfigurationRetrySeconds) {
      configurationSuccessful = configureHardware();
      nextConfigurationRetrySeconds = Timer.getFPGATimestamp() + 1.0;
    }
  }

  // Checks configuration, sensors, and motor connectivity.
  public boolean isReady() {
    if (!configurationSuccessful || getAngle() == null || !speedReady()
        || !Double.isFinite(getTurnRateRadPerSec())) {
      return false;
    }
    var position = driveMotor.getPosition();
    return isFresh(position) && Double.isFinite(position.getValueAsDouble())
        && isFresh(turnMotor.getPosition());
  }

  // Stops both motors and clears steering history. No active angle hold.
  public void stop() {
    moving = false;
    previousDriveTarget = 0.0;
    turnMovingAwayFromTarget = false;
    turnRateRadPerSec = Double.NaN;
    targetSpeedMetersPerSecond = 0.0;
    driveSpeedScale = 0.0;
    driveBaseVolts = 0.0;
    driveCorrectionVolts = 0.0;
    driveVolts = 0.0;
    Rotation2d angle = getAngle();
    if (driveMotor != null) {
      recordControlStatus(driveMotor.setControl(driveVoltageRequest.withOutput(0.0)));
    }
    turnControllerInitialized = false;
    turnWithinTolerance = false;
    turnOptimizationReversed = false;
    if (angle != null) {
      lastTargetAngle = angle;
      resetTurnController(angle);
    }
    turnVolts = 0.0;
    if (turnMotor != null) {
      recordControlStatus(turnMotor.setControl(turnVoltageRequest.withOutput(0.0)));
    }
  }

  // Publishes module commands, feedback, and diagnostics.
  public void updateDashboard() {
    Rotation2d angle = getAngle();
    updateCalibration();
    SmartDashboard.putNumber(
        name + "/TargetSpeedMetersPerSecond", targetSpeedMetersPerSecond);
    SmartDashboard.putBoolean(name + "/DriveMeasurementValid", speedReady());
    SmartDashboard.putBoolean(name + "/Ready", isReady());
    SmartDashboard.putBoolean(name + "/ConfigurationOK", configurationSuccessful);
    SmartDashboard.putBoolean(name + "/OptimizationReversed", turnOptimizationReversed);
    double measuredSpeed = Double.NaN;
    if (speedReady()) {
      measuredSpeed = getSpeed();
    }
    SmartDashboard.putNumber(name + "/MeasuredSpeedMetersPerSecond", measuredSpeed);
    SmartDashboard.putNumber(name + "/SpeedScale", driveSpeedScale);
    SmartDashboard.putNumber(name + "/BaseVolts", driveBaseVolts);
    SmartDashboard.putNumber(name + "/PVolts", driveCorrectionVolts);
    SmartDashboard.putNumber(name + "/DriveVolts", driveVolts);
    double angleRadians = Double.NaN;
    if (angle != null) {
      angleRadians = angle.getRadians();
    }
    SmartDashboard.putNumber(name + "/AngleRadians", angleRadians);
    SmartDashboard.putNumber(name + "/AngleDegrees", Math.toDegrees(angleRadians));
    SmartDashboard.putNumber(name + "/TargetAngleDegrees", lastTargetAngle.getDegrees());
    SmartDashboard.putNumber(name + "/TargetAngleRadians", lastTargetAngle.getRadians());
    double turnErrorRadians = Double.NaN;
    if (angle != null) {
      turnErrorRadians = lastTargetAngle.minus(angle).getRadians();
    }
    SmartDashboard.putNumber(
        name + "/TurnErrorRadians", turnErrorRadians);
    SmartDashboard.putNumber(name + "/TurnErrorDegrees", Math.toDegrees(turnErrorRadians));
    SmartDashboard.putNumber(name + "/TurnToleranceDegrees",
        Math.toDegrees(Constants.SwerveConstants.kTurnToleranceRad));
    SmartDashboard.putBoolean(name + "/TurnWithinTolerance", turnWithinTolerance);
    SmartDashboard.putNumber(name + "/TurnVolts", turnVolts);
    SmartDashboard.putNumber(name + "/TurnRateDegreesPerSecond",
        Math.toDegrees(turnRateRadPerSec));
    SmartDashboard.putBoolean(name + "/TurnMovingAwayFromTarget", turnMovingAwayFromTarget);
    SmartDashboard.putNumber(name + "/TurnProfileReplanCount", turnProfileReplanCount);
    double profileErrorDegrees;
    if (angle != null) {
      Rotation2d profileAngle = Rotation2d.fromRadians(turnController.getSetpoint().position);
      profileErrorDegrees = profileAngle.minus(angle).getDegrees();
    } else {
      profileErrorDegrees = Double.NaN;
    }
    SmartDashboard.putNumber(name + "/ProfileErrorDegrees", profileErrorDegrees);
    SmartDashboard.putNumber(name + "/ProfileVelocityDegreesPerSecond",
        Math.toDegrees(turnController.getSetpoint().velocity));
    SmartDashboard.putBoolean(name + "/DriveVoltageLimited",
        Math.abs(driveBaseVolts + driveCorrectionVolts) > Constants.SwerveConstants.kMaxDriveVolts);
    if (turnMotor != null) {
      SmartDashboard.putNumber(name + "/TurnStatorCurrentAmps",
          turnMotor.getStatorCurrent().getValueAsDouble());
    }
    double supplyCurrentAmps = 0.0;
    double statorCurrentAmps = 0.0;
    if (driveMotor != null) {
      SmartDashboard.putNumber(name + "/DriveAppliedVolts",
          driveMotor.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber(name + "/DriveSupplyVolts",
          driveMotor.getSupplyVoltage().getValueAsDouble());
      supplyCurrentAmps = finiteOrZero(driveMotor.getSupplyCurrent().getValueAsDouble());
      statorCurrentAmps = finiteOrZero(driveMotor.getStatorCurrent().getValueAsDouble());
    }
    SmartDashboard.putNumber(name + "/DriveSupplyCurrentAmps", supplyCurrentAmps);
    SmartDashboard.putNumber(name + "/DriveStatorCurrentAmps", statorCurrentAmps);
  }

  // Shows hardware settings and forward calibration offset.
  private void updateCalibration() {
    String key = name + "/Calibration/";
    double rotations = Double.NaN;
    double rate = Double.NaN;
    if (getAngle() != null) {
      var position = encoder.getAbsolutePosition();
      if (isFresh(position) && Double.isFinite(position.getValueAsDouble())) {
        rotations = position.getValueAsDouble();
      }
      var velocity = encoder.getVelocity();
      if (isFresh(velocity) && Double.isFinite(velocity.getValueAsDouble())) {
        rate = velocity.getValueAsDouble() * 360.0;
      }
    }
    boolean canCopyOffset = DriverStation.isDisabled() && Double.isFinite(rotations);
    SmartDashboard.putNumber(name + "/AbsoluteRotations", rotations);
    SmartDashboard.putBoolean(key + "EncoderOK", Double.isFinite(rotations));
    SmartDashboard.putBoolean(key + "CanCopyOffset", canCopyOffset);
    double offsetToCopy = Double.NaN;
    if (canCopyOffset) {
      offsetToCopy = rotations;
    }
    SmartDashboard.putNumber(key + "OffsetToCopyWhenForward", offsetToCopy);
    SmartDashboard.putNumber(key + "EncoderRateDegreesPerSecond", rate);
    if (configuration != null) {
      SmartDashboard.putNumber(key + "DriveMotorId", configuration.driveCanId());
      SmartDashboard.putNumber(key + "SteerMotorId", configuration.turnCanId());
      SmartDashboard.putNumber(key + "EncoderId", configuration.encoderCanId());
      SmartDashboard.putNumber(key + "ConfiguredOffsetRotations", configuration.encoderOffsetRotations());
      SmartDashboard.putBoolean(key + "DriveInverted", configuration.driveInverted());
      SmartDashboard.putBoolean(key + "SteerInverted", configuration.turnInverted());
      SmartDashboard.putBoolean(key + "EncoderInverted", configuration.encoderInverted());
    }
  }

  // Returns the calibrated, rounded, wrapped angle, or null.
  private Rotation2d getAngle() {
    if (encoder == null || !configurationSuccessful) {
      return null;
    }
    var magnetFault = encoder.getFault_BadMagnet();
    if (!isFresh(magnetFault) || magnetFault.getValue()) {
      return null;
    }
    var absolutePosition = encoder.getAbsolutePosition();
    double rotations = absolutePosition.getValueAsDouble();
    double offsetRotations = configuration.encoderOffsetRotations();
    if (!isFresh(absolutePosition)
        || !Double.isFinite(rotations)
        || !Double.isFinite(offsetRotations)
        || !isPositiveFinite(Constants.SwerveConstants.kCancoderCountsPerRotation)) {
      return null;
    }
    double correctedRotations = rotations - offsetRotations;
    correctedRotations = Math.rint(
        correctedRotations * Constants.SwerveConstants.kCancoderCountsPerRotation)
        / Constants.SwerveConstants.kCancoderCountsPerRotation;
    return Rotation2d.fromRadians(MathUtil.angleModulus(
        correctedRotations * 2.0 * Math.PI));
  }

  // Returns wheel speed in m/s, or NaN if unavailable.
  public double getSpeed() {
    if (!speedReady()) {
      return Double.NaN;
    }
    var velocity = driveMotor.getVelocity();
    double motorRotationsPerSecond = velocity.getValueAsDouble();
    if (!isFresh(velocity) || !Double.isFinite(motorRotationsPerSecond)) {
      return Double.NaN;
    }
    return motorRotationsPerSecond
        / Constants.SwerveConstants.kDriveReduction
        * (Math.PI * Constants.SwerveConstants.kWheelDiameterMeters);
  }

  // Returns steering rate in rad/s, or NaN if unavailable.
  private double getTurnRateRadPerSec() {
    if (encoder == null) {
      return Double.NaN;
    }
    var velocity = encoder.getVelocity();
    if (!isFresh(velocity) || !Double.isFinite(velocity.getValueAsDouble())) {
      return Double.NaN;
    }
    return velocity.getValueAsDouble() * 2.0 * Math.PI;
  }

  // Restarts the steering profile from measured angle and speed.
  private void resetTurnController(Rotation2d angle) {
    double turnRate = getTurnRateRadPerSec();
    if (!Double.isFinite(turnRate)) {
      turnRate = 0.0;
    }
    turnController.reset(angle.getRadians(), turnRate);
  }

  // Checks wheel diameter and gearing.
  private boolean validWheelConfig() {
    return isPositiveFinite(Constants.SwerveConstants.kWheelDiameterMeters)
        && isPositiveFinite(Constants.SwerveConstants.kDriveReduction);
  }

  // Checks for a positive, finite value.
  private static boolean isPositiveFinite(double value) {
    return value > 0.0 && Double.isFinite(value);
  }

  // Checks signal status and age.
  private static boolean isFresh(BaseStatusSignal signal) {
    return signal.getStatus().isOK()
        && signal.getTimestamp().getLatency() <= Constants.SwerveConstants.kMaxSensorAgeSeconds;
  }

  // Replaces nonfinite values with zero.
  private static double finiteOrZero(double value) {
    if (Double.isFinite(value)) {
      return value;
    }
    return 0.0;
  }
}
