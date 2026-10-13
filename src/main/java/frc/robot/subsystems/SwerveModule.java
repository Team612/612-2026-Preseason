package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
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

public class SwerveModule {
  private final String name;
  private final Constants.ModuleConfiguration configuration;
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder encoder;
  private boolean configurationSuccessful;
  private double nextConfigurationRetrySeconds;
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut turnVoltageRequest = new VoltageOut(0.0);
  private final ProfiledPIDController turnController = new ProfiledPIDController(
      Constants.SwerveConstants.kTurnP, Constants.SwerveConstants.kTurnI,
      Constants.SwerveConstants.kTurnD,
      new TrapezoidProfile.Constraints(Constants.SwerveConstants.kMaxTurnRadPerSec,
          Constants.SwerveConstants.kMaxTurnAccelRadPerSecSquared), Constants.kLoopPeriodSeconds);
  private Rotation2d lastTargetAngle = new Rotation2d();
  private boolean turnControllerInitialized;
  private boolean turnOptimizationReversed;
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
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    if (hardwareEnabled && configuration != null && configuration.hasValidIds()) {
      driveMotor = new TalonFX(configuration.driveCanId(), Constants.SwerveConstants.kCanBus);
      turnMotor = new TalonFX(configuration.turnCanId(), Constants.SwerveConstants.kCanBus);
      encoder = new CANcoder(configuration.encoderCanId(), Constants.SwerveConstants.kCanBus);
      configurationSuccessful = configureHardware();
      nextConfigurationRetrySeconds = Timer.getFPGATimestamp() + 1.0;
    } else {
      driveMotor = null;
      turnMotor = null;
      encoder = null;
      configurationSuccessful = false;
    }
  }

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
        encoder.getAbsolutePosition(), encoder.getVelocity(),
        encoder.getFault_BadMagnet(),
        driveMotor.getPosition(), driveMotor.getVelocity(),
        turnMotor.getPosition());
    turnControllerInitialized = false;
    driveMotor.hasResetOccurred();
    turnMotor.hasResetOccurred();
    encoder.hasResetOccurred();
    boolean success = driveStatus.isOK() && turnStatus.isOK()
        && encoderStatus.isOK() && signalStatus.isOK();


        // logs if failed
        if (!success) {
      DriverStation.reportError(name + " configuration failed: drive=" + driveStatus
          + ", turn=" + turnStatus + ", encoder=" + encoderStatus
          + ", signals=" + signalStatus, false);
    }
    System.out.println(encoder.getAbsolutePosition().getValueAsDouble());
    return success;
  }

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
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return motorConfiguration;
  }

  public void setState(SwerveModuleState targetState) {
    if (targetState == null || targetState.angle == null
        || !Double.isFinite(targetState.speedMetersPerSecond)
        || !Double.isFinite(targetState.angle.getRadians())) {
      stop();
      return;
    }
    Rotation2d angle = getAngle();
    if (DriverStation.isDisabled() || !isReady() || angle == null
        || !isPositiveFinite(Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
        || !isPositiveFinite(Constants.SwerveConstants.kMaxDriveVolts)
        || !Double.isFinite(Constants.SwerveConstants.kDriveKV)
        || Constants.SwerveConstants.kDriveKV < 0.0
        || !isPositiveFinite(Constants.SwerveConstants.kMaxTurnVolts)) {
      stop();
      return;
    }
    if (!turnControllerInitialized) {
      turnController.reset(angle.getRadians());
      lastTargetAngle = angle;
      turnOptimizationReversed = false;
      turnControllerInitialized = true;
    }

    if (Math.abs(targetState.speedMetersPerSecond)
        < Constants.SwerveConstants.kMinimumModuleSpeedMetersPerSecond) {
      turnOptimizationReversed = false;
      stop();
      return;
    }

    SwerveModuleState moduleState = optimizeState(targetState, angle);
    targetSpeedMetersPerSecond = moduleState.speedMetersPerSecond;
    lastTargetAngle = moduleState.angle;
    setAngle(angle, moduleState.angle);
    setSpeed(moduleState.speedMetersPerSecond);
    System.out.println(name + " " + encoder.getAbsolutePosition().getValueAsDouble());
  }

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

  private void setAngle(Rotation2d angle, Rotation2d targetAngle) {
    double turnRate = getTurnRateRadPerSec();
    double output =
        turnController.calculate(angle.getRadians(), targetAngle.getRadians());
    output += Constants.SwerveConstants.kTurnVelocityDamping
        * (turnController.getSetpoint().velocity - turnRate);
    if (Math.abs(targetAngle.minus(angle).getRadians())
        <= Constants.SwerveConstants.kTurnToleranceRad
        && Math.abs(turnRate) <= Constants.SwerveConstants.kTurnSettledVelocityRadPerSec) {
      output = 0.0;
      turnController.reset(angle.getRadians());
    }
    turnVolts =
        finiteOrZero(
            MathUtil.clamp(
                output,
                -Constants.SwerveConstants.kMaxTurnVolts,
                Constants.SwerveConstants.kMaxTurnVolts));
    turnMotor.setControl(turnVoltageRequest.withOutput(turnVolts));
  }

  private void setSpeed(double targetSpeed) {
    driveSpeedScale =
        MathUtil.clamp(
            targetSpeed / Constants.SwerveConstants.kMaxSpeedMetersPerSecond, -1.0, 1.0);
    driveBaseVolts = targetSpeed * Constants.SwerveConstants.kDriveKV;
    driveCorrectionVolts = 0.0;
    if (Math.abs(targetSpeed) < 1e-6) {
      driveVolts = 0.0;
      driveMotor.setControl(driveVoltageRequest.withOutput(0.0));
      return;
    }
    if (Constants.SwerveConstants.kEnableDriveVelocityCorrection && speedReady()
        && Double.isFinite(Constants.SwerveConstants.kDriveP)
        && Constants.SwerveConstants.kMaxDriveCorrectionVolts >= 0.0
        && Double.isFinite(Constants.SwerveConstants.kMaxDriveCorrectionVolts)) {
      double measuredSpeed = getSpeed();
      if (Double.isFinite(measuredSpeed)) {
        driveCorrectionVolts =
            MathUtil.clamp(
                Constants.SwerveConstants.kDriveP * (targetSpeed - measuredSpeed),
                -Constants.SwerveConstants.kMaxDriveCorrectionVolts,
                Constants.SwerveConstants.kMaxDriveCorrectionVolts);
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

  private boolean speedReady() {
    if (driveMotor == null || !validWheelConfig()) {
      return false;
    }
    var velocity = driveMotor.getVelocity();
    return isFresh(velocity) && Double.isFinite(velocity.getValueAsDouble());
  }

  public void periodic() {
    if (driveMotor == null) {
      return;
    }
    boolean deviceReset = driveMotor.hasResetOccurred();
    deviceReset |= turnMotor.hasResetOccurred();
    deviceReset |= encoder.hasResetOccurred();
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

  public boolean isReady() {
    if (!configurationSuccessful || getAngle() == null || !speedReady()
        || !Double.isFinite(getTurnRateRadPerSec())) {
      return false;
    }
    var position = driveMotor.getPosition();
    return isFresh(position) && Double.isFinite(position.getValueAsDouble())
        && isFresh(turnMotor.getPosition());
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
    turnControllerInitialized = false;
    if (angle != null) {
      lastTargetAngle = angle;
      turnController.reset(angle.getRadians());
    }
    turnVolts = 0.0;
    if (turnMotor != null) {
      turnMotor.setControl(turnVoltageRequest.withOutput(0.0));
    }
  }

  public void updateDashboard() {
    Rotation2d angle = getAngle();
    updateCalibration();
    SmartDashboard.putNumber(
        name + "/TargetSpeedMetersPerSecond", targetSpeedMetersPerSecond);
    SmartDashboard.putBoolean(name + "/DriveMeasurementValid", speedReady());
    SmartDashboard.putBoolean(name + "/Ready", isReady());
    SmartDashboard.putBoolean(name + "/ConfigurationOK", configurationSuccessful);
    SmartDashboard.putBoolean(name + "/OptimizationReversed", turnOptimizationReversed);
    double measuredSpeed = 0.0;
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
    double turnErrorRadians = 0.0;
    if (angle != null) {
      turnErrorRadians = lastTargetAngle.minus(angle).getRadians();
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
        || !Double.isFinite(offsetRotations)) {
      return null;
    }
    return Rotation2d.fromRadians(MathUtil.angleModulus(
        (rotations - offsetRotations) * 2.0 * Math.PI));
  }

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

  private boolean validWheelConfig() {
    return isPositiveFinite(Constants.SwerveConstants.kWheelDiameterMeters)
        && isPositiveFinite(Constants.SwerveConstants.kDriveReduction);
  }

  private static boolean isPositiveFinite(double value) {
    return value > 0.0 && Double.isFinite(value);
  }

  private static boolean isFresh(BaseStatusSignal signal) {
    return signal.getStatus().isOK()
        && signal.getTimestamp().getLatency() <= Constants.SwerveConstants.kMaxSensorAgeSeconds;
  }

  private static double finiteOrZero(double value) {
    if (Double.isFinite(value)) {
      return value;
    }
    return 0.0;
  }
}
