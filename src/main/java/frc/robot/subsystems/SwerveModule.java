package frc.robot.subsystems;

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

public class SwerveModule {
  private final String name;
  private final Constants.ModuleConfiguration configuration;
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder encoder;
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut turnVoltageRequest = new VoltageOut(0.0);
  private final ProfiledPIDController turnController =
      new ProfiledPIDController(
          Constants.SwerveConstants.kTurnP,
          Constants.SwerveConstants.kTurnI,
          Constants.SwerveConstants.kTurnD,
          new TrapezoidProfile.Constraints(
              Constants.SwerveConstants.kMaxTurnRadPerSec,
              Constants.SwerveConstants.kMaxTurnAccelRadPerSecSquared));
  private Rotation2d lastTargetAngle = new Rotation2d();
  private double targetSpeed;
  private double speedScale;
  private double baseVolts;
  private double pVolts;
  private double driveVolts;
  private double turnVolts;

  public SwerveModule(String name, Constants.ModuleConfiguration configuration) {
    this.name = name;
    this.configuration = configuration;
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    if (configuration.hasConfiguredCanIds()) {
      driveMotor = new TalonFX(configuration.driveCanId(), Constants.SwerveConstants.kCanBus);
      turnMotor = new TalonFX(configuration.turnCanId(), Constants.SwerveConstants.kCanBus);
      encoder = new CANcoder(configuration.encoderCanId(), Constants.SwerveConstants.kCanBus);
      configureMotors();
    } else {
      driveMotor = null;
      turnMotor = null;
      encoder = null;
    }
  }

  private void configureMotors() {
    TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    driveConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.kDriveSupplyLimitAmps;
    driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfiguration.CurrentLimits.StatorCurrentLimit = Constants.SwerveConstants.kDriveStatorLimitAmps;
    driveConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfiguration.MotorOutput.Inverted =
        configuration.driveInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(driveConfiguration);

    TalonFXConfiguration turnConfiguration = new TalonFXConfiguration();
    turnConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.kTurnSupplyLimitAmps;
    turnConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnConfiguration.CurrentLimits.StatorCurrentLimit = Constants.SwerveConstants.kTurnStatorLimitAmps;
    turnConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfiguration.MotorOutput.Inverted =
        configuration.turnInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnMotor.getConfigurator().apply(turnConfiguration);
  }

  public void setState(SwerveModuleState targetState) {
    if (!Double.isFinite(targetState.speedMetersPerSecond)
        || !Double.isFinite(targetState.angle.getRadians())) {
      stop();
      return;
    }
    Rotation2d angle = getAngle();
    if (angle == null
        || !validDriveLimits()
        || !validTurnLimit()
        || !validLowSpeedThreshold()) {
      stop();
      return;
    }

    SwerveModuleState moduleState = new SwerveModuleState(targetState.speedMetersPerSecond, targetState.angle);
    moduleState.optimize(angle);
    moduleState.cosineScale(angle);
    if (Math.abs(moduleState.speedMetersPerSecond)
        < Constants.SwerveConstants.kLowSpeedThresholdMetersPerSecond) {
      moduleState = new SwerveModuleState(0.0, lastTargetAngle);
    }

    targetSpeed = moduleState.speedMetersPerSecond;
    lastTargetAngle = moduleState.angle;
    commandTurn(angle, moduleState.angle);
    commandDrive(moduleState.speedMetersPerSecond);
  }

  private void commandTurn(Rotation2d angle, Rotation2d targetAngle) {
    if (!validTurnLimit()) {
      stop();
      return;
    }
    double output = turnController.calculate(angle.getRadians(), targetAngle.getRadians());
    turnVolts =
        finiteOrZero(
            MathUtil.clamp(
                output,
                -Constants.SwerveConstants.kMaxTurnVolts,
                Constants.SwerveConstants.kMaxTurnVolts));
    turnMotor.setControl(turnVoltageRequest.withOutput(turnVolts));
  }

  private void commandDrive(double targetSpeed) {
    speedScale =
        MathUtil.clamp(
            targetSpeed / Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
            -1.0,
            1.0);
    baseVolts = speedScale * Constants.SwerveConstants.kMaxDriveVolts;
    pVolts = 0.0;
    if (Constants.SwerveConstants.kEnableDriveVelocityCorrection
        && hasValidDriveMeasurement()
        && Double.isFinite(Constants.SwerveConstants.kDriveP)
        && validPVoltageLimit()) {
      double measuredSpeed = getSpeed();
      if (Double.isFinite(measuredSpeed)) {
        pVolts =
            MathUtil.clamp(
                Constants.SwerveConstants.kDriveP * (targetSpeed - measuredSpeed),
                -Constants.SwerveConstants.kMaxPVolts,
                Constants.SwerveConstants.kMaxPVolts);
      }
    }
    driveVolts =
        finiteOrZero(
            MathUtil.clamp(
                baseVolts + pVolts,
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
    Rotation2d angle = getAngle();
    if (angle == null || !hasValidDrivePosition()) {
      return new SwerveModulePosition(0.0, lastTargetAngle);
    }
    var position = driveMotor.getPosition();
    double motorRotations = position.getValueAsDouble();
    if (!position.getStatus().isOK() || !Double.isFinite(motorRotations)) {
      return new SwerveModulePosition(0.0, lastTargetAngle);
    }
    double distanceMeters = motorRotations / Constants.SwerveConstants.kDriveReduction * wheelCircumferenceMeters();
    if (!Double.isFinite(distanceMeters)) {
      return new SwerveModulePosition(0.0, lastTargetAngle);
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

  public void stop() {
    targetSpeed = 0.0;
    speedScale = 0.0;
    baseVolts = 0.0;
    pVolts = 0.0;
    driveVolts = 0.0;
    turnVolts = 0.0;
    if (driveMotor != null) {
      driveMotor.setControl(driveVoltageRequest.withOutput(0.0));
      turnMotor.setControl(turnVoltageRequest.withOutput(0.0));
    }
  }

  public void publishTelemetry() {
    Rotation2d angle = getAngle();
    SmartDashboard.putNumber(name + "/TargetSpeedMetersPerSecond", targetSpeed);
    SmartDashboard.putBoolean(name + "/DriveMeasurementValid", hasValidDriveMeasurement());
    SmartDashboard.putNumber(
        name + "/MeasuredSpeedMetersPerSecond", hasValidDriveMeasurement() ? getSpeed() : 0.0);
    SmartDashboard.putNumber(name + "/SpeedScale", speedScale);
    SmartDashboard.putNumber(name + "/BaseVolts", baseVolts);
    SmartDashboard.putNumber(name + "/PVolts", pVolts);
    SmartDashboard.putNumber(name + "/DriveVolts", driveVolts);
    SmartDashboard.putNumber(name + "/AngleRadians", angle == null ? 0.0 : angle.getRadians());
    SmartDashboard.putNumber(name + "/TargetAngleRadians", lastTargetAngle.getRadians());
    SmartDashboard.putNumber(
        name + "/TurnErrorRadians", angle == null ? 0.0 : turnController.getPositionError());
    SmartDashboard.putNumber(name + "/TurnVolts", turnVolts);
    SmartDashboard.putNumber(
        name + "/DriveSupplyCurrentAmps",
        driveMotor == null ? 0.0 : finiteOrZero(driveMotor.getSupplyCurrent().getValueAsDouble()));
    SmartDashboard.putNumber(
        name + "/DriveStatorCurrentAmps",
        driveMotor == null ? 0.0 : finiteOrZero(driveMotor.getStatorCurrent().getValueAsDouble()));
  }

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

  private double getSpeed() {
    if (!hasValidDriveMeasurement()) {
      return 0.0;
    }
    var velocity = driveMotor.getVelocity();
    double motorRotationsPerSecond = velocity.getValueAsDouble();
    if (!velocity.getStatus().isOK() || !Double.isFinite(motorRotationsPerSecond)) {
      return 0.0;
    }
    return motorRotationsPerSecond / Constants.SwerveConstants.kDriveReduction * wheelCircumferenceMeters();
  }

  private boolean hasValidDriveConversion() {
    return Constants.SwerveConstants.kWheelDiameterMeters > 0.0
        && Constants.SwerveConstants.kDriveReduction > 0.0
        && Double.isFinite(Constants.SwerveConstants.kWheelDiameterMeters)
        && Double.isFinite(Constants.SwerveConstants.kDriveReduction);
  }

  private boolean validDriveLimits() {
    return Constants.SwerveConstants.kMaxSpeedMetersPerSecond > 0.0
        && Constants.SwerveConstants.kMaxDriveVolts > 0.0
        && Double.isFinite(Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
        && Double.isFinite(Constants.SwerveConstants.kMaxDriveVolts);
  }

  private boolean validPVoltageLimit() {
    return Constants.SwerveConstants.kMaxPVolts >= 0.0
        && Double.isFinite(Constants.SwerveConstants.kMaxPVolts);
  }

  private boolean validTurnLimit() {
    return Constants.SwerveConstants.kMaxTurnVolts > 0.0
        && Double.isFinite(Constants.SwerveConstants.kMaxTurnVolts);
  }

  private boolean validLowSpeedThreshold() {
    return Constants.SwerveConstants.kLowSpeedThresholdMetersPerSecond >= 0.0
        && Double.isFinite(Constants.SwerveConstants.kLowSpeedThresholdMetersPerSecond);
  }

  private double wheelCircumferenceMeters() {
    return Math.PI * Constants.SwerveConstants.kWheelDiameterMeters;
  }

  private double finiteOrZero(double value) {
    return Double.isFinite(value) ? value : 0.0;
  }
}
