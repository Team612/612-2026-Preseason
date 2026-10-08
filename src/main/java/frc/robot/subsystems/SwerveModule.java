package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Controls one wheel's drive motor, steering motor, and absolute angle encoder. */
public class SwerveModule {
  private final String name;
  private final Constants.ModuleConfiguration configuration;
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder encoder;
  private final boolean configurationSuccessful;
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut turnVoltageRequest = new VoltageOut(0.0);
  private final ProfiledPIDController turnController = new ProfiledPIDController(
      Constants.SwerveConstants.kTurnP, Constants.SwerveConstants.kTurnI,
      Constants.SwerveConstants.kTurnD,
      new TrapezoidProfile.Constraints(Constants.SwerveConstants.kMaxTurnRadPerSec,
          Constants.SwerveConstants.kMaxTurnAccelRadPerSecSquared));
  private Rotation2d lastTargetAngle = new Rotation2d();
  private boolean turnOptimizationReversed;
  private boolean turnControllerInitialized;
  private double targetSpeedMetersPerSecond;
  private double driveSpeedScale;
  private double driveBaseVolts;
  private double driveCorrectionVolts;
  private double driveVolts;
  private double turnVolts;

  public SwerveModule(
      String name, Constants.ModuleConfiguration configuration, boolean hardwareEnabled) {
    this.name = name;
    this.configuration = configuration;
    // Steering angles wrap at +/- pi; continuous input makes the controller take the short path.
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    if (hardwareEnabled && configuration.hasConfiguredCanIds()) {
      driveMotor = new TalonFX(configuration.driveCanId(), Constants.SwerveConstants.kCanBus);
      turnMotor = new TalonFX(configuration.turnCanId(), Constants.SwerveConstants.kCanBus);
      encoder = new CANcoder(configuration.encoderCanId(), Constants.SwerveConstants.kCanBus);
      configurationSuccessful = configureMotors();
    } else {
      driveMotor = null;
      turnMotor = null;
      encoder = null;
      configurationSuccessful = false;
    }
  }

  private boolean configureMotors() {
    StatusCode driveStatus =
        driveMotor.getConfigurator().apply(createMotorConfiguration(true));
    StatusCode turnStatus =
        turnMotor.getConfigurator().apply(createMotorConfiguration(false));
    return driveStatus.isOK() && turnStatus.isOK();
  }

  /** Builds common motor settings while choosing the correct limits and inversion per motor. */
  private TalonFXConfiguration createMotorConfiguration(boolean isDriveMotor) {
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
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return motorConfiguration;
  }

  /** Converts a chassis kinematics result into safe drive and steering motor requests. */
  public void setState(SwerveModuleState targetState) {
    if (targetState == null || targetState.angle == null
        || !Double.isFinite(targetState.speedMetersPerSecond)
        || !Double.isFinite(targetState.angle.getRadians())) {
      stop();
      return;
    }
    Rotation2d angle = getAngle();
    if (angle == null || !validDriveLimits() || !validTurnLimit()) {
      stop();
      return;
    }
    initializeTurnController(angle);
    turnOptimizationReversed =
        shouldReverseOptimization(targetState, angle, turnOptimizationReversed);
    SwerveModuleState moduleState =
        alignToMeasuredAngle(targetState, angle, turnOptimizationReversed);
    targetSpeedMetersPerSecond = moduleState.speedMetersPerSecond;
    lastTargetAngle = moduleState.angle;
    commandTurn(angle, moduleState.angle);
    commandDrive(moduleState.speedMetersPerSecond);
  }

  /** Optimizes the wheel direction and slows drive while the steering angle is catching up. */
  static SwerveModuleState alignToMeasuredAngle(
      SwerveModuleState requestedState, Rotation2d measuredAngle) {
    return alignToMeasuredAngle(requestedState, measuredAngle, false);
  }

  static SwerveModuleState alignToMeasuredAngle(
      SwerveModuleState requestedState,
      Rotation2d measuredAngle,
      boolean optimizationReversed) {
    SwerveModuleState optimizedState =
        optimizeState(requestedState, measuredAngle, optimizationReversed);
    return applySteeringAlignmentScale(optimizedState, measuredAngle);
  }

  /** Chooses the equivalent wheel direction that requires no more than 90 degrees of steering. */
  static SwerveModuleState optimizeState(
      SwerveModuleState requestedState, Rotation2d measuredAngle) {
    double angleError = requestedState.angle.minus(measuredAngle).getRadians();
    return optimizeState(requestedState, measuredAngle, Math.abs(angleError) > Math.PI / 2.0);
  }

  static SwerveModuleState optimizeState(
      SwerveModuleState requestedState,
      Rotation2d measuredAngle,
      boolean optimizationReversed) {
    if (optimizationReversed) {
      Rotation2d reversedAngle = requestedState.angle.rotateBy(Rotation2d.fromRadians(Math.PI));
      return new SwerveModuleState(-requestedState.speedMetersPerSecond, reversedAngle);
    }
    return new SwerveModuleState(
        requestedState.speedMetersPerSecond, requestedState.angle);
  }

  static boolean shouldReverseOptimization(
      SwerveModuleState requestedState,
      Rotation2d measuredAngle,
      boolean currentlyReversed) {
    double angleError = Math.abs(requestedState.angle.minus(measuredAngle).getRadians());
    double hysteresis = Constants.SwerveConstants.kTurnOptimizationHysteresisRad;
    double flipThreshold =
        currentlyReversed ? Math.PI / 2.0 - hysteresis : Math.PI / 2.0 + hysteresis;
    return angleError > flipThreshold;
  }

  /**
   * Slows the wheel while it turns toward the requested angle. State optimization should run first
   * so the steering controller always takes the shorter path.
   */
  static SwerveModuleState applySteeringAlignmentScale(
      SwerveModuleState requestedState, Rotation2d measuredAngle) {
    double angleError = requestedState.angle.minus(measuredAngle).getRadians();
    double alignmentScale = Math.max(0.0, Math.cos(angleError));
    return new SwerveModuleState(
        requestedState.speedMetersPerSecond * alignmentScale, requestedState.angle);
  }

  private void commandTurn(Rotation2d angle, Rotation2d targetAngle) {
    if (turnMotor == null || !validTurnLimit()) {
      turnVolts = 0.0;
      if (turnMotor != null) {
        turnMotor.setControl(turnVoltageRequest.withOutput(0.0));
      }
      return;
    }
    double output =
        turnController.calculate(angle.getRadians(), targetAngle.getRadians());
    turnVolts =
        finiteOrZero(
            MathUtil.clamp(
                output,
                -Constants.SwerveConstants.kMaxTurnVolts,
                Constants.SwerveConstants.kMaxTurnVolts));
    turnMotor.setControl(turnVoltageRequest.withOutput(turnVolts));
  }

  private void initializeTurnController(Rotation2d angle) {
    if (!turnControllerInitialized) {
      turnController.reset(angle.getRadians());
      lastTargetAngle = angle;
      turnControllerInitialized = true;
    }
  }

  private void commandDrive(double targetSpeed) {
    // Convert wheel speed to open-loop voltage, then add a small measured-speed correction.
    driveSpeedScale =
        MathUtil.clamp(
            targetSpeed / Constants.SwerveConstants.kMaxSpeedMetersPerSecond, -1.0, 1.0);
    driveBaseVolts = driveSpeedScale * Constants.SwerveConstants.kMaxDriveVolts;
    driveCorrectionVolts = 0.0;
    if (Constants.SwerveConstants.kEnableDriveVelocityCorrection && hasValidDriveMeasurement()
        && Double.isFinite(Constants.SwerveConstants.kDriveP) && validPVoltageLimit()) {
      double measuredSpeed = getSpeed();
      if (Double.isFinite(measuredSpeed)) {
        driveCorrectionVolts =
            MathUtil.clamp(
                Constants.SwerveConstants.kDriveP * (targetSpeed - measuredSpeed),
                -Constants.SwerveConstants.kMaxPVolts,
                Constants.SwerveConstants.kMaxPVolts);
      }
    }
    driveVolts =
        finiteOrZero(
            MathUtil.clamp(
                driveBaseVolts + driveCorrectionVolts,
                -Constants.SwerveConstants.kMaxDriveVolts,
                Constants.SwerveConstants.kMaxDriveVolts));
    driveMotor.setControl(driveVoltageRequest.withOutput(driveVolts));
  }

  public SwerveModuleState getState() {
    Rotation2d angle = getAngle();
    if (angle == null || !hasValidDriveMeasurement()) {
      return new SwerveModuleState(0.0, lastTargetAngle);
    }
    return new SwerveModuleState(getSpeed(), angle);
  }

  public SwerveModulePosition getPosition() {
    SwerveModulePosition position = getPositionIfValid();
    if (position == null) {
      return new SwerveModulePosition(0.0, lastTargetAngle);
    }
    return position;
  }

  public SwerveModulePosition getPositionIfValid() {
    Rotation2d angle = getAngle();
    if (angle == null || driveMotor == null || !hasValidDriveConversion()) {
      return null;
    }
    var position = driveMotor.getPosition();
    double motorRotations = position.getValueAsDouble();
    if (!position.getStatus().isOK() || !Double.isFinite(motorRotations)) {
      return null;
    }
    double distanceMeters =
        motorRotations
            / Constants.SwerveConstants.kDriveReduction
            * wheelCircumferenceMeters();
    if (!Double.isFinite(distanceMeters)) {
      return null;
    }
    return new SwerveModulePosition(distanceMeters, angle);
  }

  public boolean hasValidDriveMeasurement() {
    if (driveMotor == null || !hasValidDriveConversion()) {
      return false;
    }
    var velocity = driveMotor.getVelocity();
    return velocity.getStatus().isOK() && Double.isFinite(velocity.getValueAsDouble());
  }

  public boolean hasValidDrivePosition() {
    if (driveMotor == null || !hasValidDriveConversion()) {
      return false;
    }
    var position = driveMotor.getPosition();
    return position.getStatus().isOK() && Double.isFinite(position.getValueAsDouble());
  }

  public boolean hasValidPosition() {
    return getPositionIfValid() != null;
  }

  public boolean isReady() {
    return configurationSuccessful
        && getAngle() != null;
  }

  public void stop() {
    targetSpeedMetersPerSecond = 0.0;
    driveSpeedScale = 0.0;
    driveBaseVolts = 0.0;
    driveCorrectionVolts = 0.0;
    driveVolts = 0.0;
    Rotation2d angle = getAngle();
    if (driveMotor != null) {
      driveMotor.setControl(driveVoltageRequest.withOutput(0.0));
    }
    if (angle != null && turnMotor != null && validTurnLimit()) {
      initializeTurnController(angle);
      commandTurn(angle, lastTargetAngle);
    } else {
      turnVolts = 0.0;
      if (turnMotor != null) {
        turnMotor.setControl(turnVoltageRequest.withOutput(0.0));
      }
    }
  }

  /** Publishes measured values and the last requested setpoints for this module. */
  public void publishTelemetry() {
    Rotation2d angle = getAngle();
    SmartDashboard.putNumber(
        name + "/TargetSpeedMetersPerSecond", targetSpeedMetersPerSecond);
    SmartDashboard.putBoolean(name + "/DriveMeasurementValid", hasValidDriveMeasurement());
    SmartDashboard.putBoolean(name + "/Ready", isReady());
    double measuredSpeed = 0.0;
    if (hasValidDriveMeasurement()) {
      measuredSpeed = getSpeed();
    }
    SmartDashboard.putNumber(name + "/MeasuredSpeedMetersPerSecond", measuredSpeed);
    SmartDashboard.putNumber(name + "/SpeedScale", driveSpeedScale);
    SmartDashboard.putNumber(name + "/BaseVolts", driveBaseVolts);
    SmartDashboard.putNumber(name + "/PVolts", driveCorrectionVolts);
    SmartDashboard.putNumber(name + "/DriveVolts", driveVolts);
    double angleRadians = 0.0;
    if (angle != null) {
      angleRadians = angle.getRadians();
    }
    SmartDashboard.putNumber(name + "/AngleRadians", angleRadians);
    SmartDashboard.putNumber(name + "/TargetAngleRadians", lastTargetAngle.getRadians());
    double turnErrorRadians = 0.0;
    if (angle != null) {
      turnErrorRadians = turnController.getPositionError();
    }
    SmartDashboard.putNumber(
        name + "/TurnErrorRadians", turnErrorRadians);
    SmartDashboard.putNumber(name + "/TurnVolts", turnVolts);
    double supplyCurrentAmps = 0.0;
    double statorCurrentAmps = 0.0;
    if (driveMotor != null) {
      supplyCurrentAmps = finiteOrZero(driveMotor.getSupplyCurrent().getValueAsDouble());
      statorCurrentAmps = finiteOrZero(driveMotor.getStatorCurrent().getValueAsDouble());
    }
    SmartDashboard.putNumber(name + "/DriveSupplyCurrentAmps", supplyCurrentAmps);
    SmartDashboard.putNumber(name + "/DriveStatorCurrentAmps", statorCurrentAmps);
  }

  /** Reads the absolute encoder and subtracts its configured zero offset. */
  private Rotation2d getAngle() {
    if (encoder == null) {
      return null;
    }
    var absolutePosition = encoder.getAbsolutePosition();
    double rotations = absolutePosition.getValueAsDouble();
    double offsetRotations = configuration.encoderOffsetRotations();
    if (!absolutePosition.getStatus().isOK()
        || !Double.isFinite(rotations)
        || !Double.isFinite(offsetRotations)) {
      return null;
    }
    return Rotation2d.fromRotations(rotations - offsetRotations);
  }

  /** Converts motor rotations per second to wheel speed in meters per second. */
  private double getSpeed() {
    if (!hasValidDriveMeasurement()) {
      return 0.0;
    }
    var velocity = driveMotor.getVelocity();
    double motorRotationsPerSecond = velocity.getValueAsDouble();
    if (!velocity.getStatus().isOK() || !Double.isFinite(motorRotationsPerSecond)) {
      return 0.0;
    }
    return motorRotationsPerSecond
        / Constants.SwerveConstants.kDriveReduction
        * wheelCircumferenceMeters();
  }

  private boolean hasValidDriveConversion() {
    return isPositiveFinite(Constants.SwerveConstants.kWheelDiameterMeters)
        && isPositiveFinite(Constants.SwerveConstants.kDriveReduction);
  }

  private boolean validDriveLimits() {
    return isPositiveFinite(Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
        && isPositiveFinite(Constants.SwerveConstants.kMaxDriveVolts);
  }

  private boolean validPVoltageLimit() {
    return Constants.SwerveConstants.kMaxPVolts >= 0.0
        && Double.isFinite(Constants.SwerveConstants.kMaxPVolts);
  }

  private boolean validTurnLimit() {
    return isPositiveFinite(Constants.SwerveConstants.kMaxTurnVolts);
  }

  private double wheelCircumferenceMeters() {
    return Math.PI * Constants.SwerveConstants.kWheelDiameterMeters;
  }

  private static boolean isPositiveFinite(double value) {
    return value > 0.0 && Double.isFinite(value);
  }

  private double finiteOrZero(double value) {
    if (!Double.isFinite(value)) {
      return 0.0;
    }
    return value;
  }
}
