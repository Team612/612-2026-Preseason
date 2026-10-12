package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDrive extends Command {
  private final Swerve swerve;
  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier controllerConnectedSupplier;

  private Translation2d limitedTranslation = new Translation2d();
  private final SlewRateLimiter rotationLimiter =
      new SlewRateLimiter(Constants.OperatorConstants.kRotationSlewRatePerSecond);

  private boolean fieldRelativeMode;

  public SwerveDrive(
      Swerve swerve,
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier controllerConnectedSupplier) {
    this.swerve = Objects.requireNonNull(swerve);
    this.forwardSupplier = Objects.requireNonNull(forwardSupplier);
    this.strafeSupplier = Objects.requireNonNull(strafeSupplier);
    this.rotationSupplier = Objects.requireNonNull(rotationSupplier);
    this.controllerConnectedSupplier = Objects.requireNonNull(controllerConnectedSupplier);
    addRequirements(this.swerve);
  }

  @Override
  public void initialize() {
    swerve.stop();
    resetLimiters();
  }

  @Override
  public void execute() {
    double forward = forwardSupplier.getAsDouble();
    double leftStickStrafe = strafeSupplier.getAsDouble();
    double rightStickX = rotationSupplier.getAsDouble();
    double strafe;
    double rotation;
    if (fieldRelativeMode) {
      strafe = leftStickStrafe;
      rotation = rightStickX;
    } else {
      strafe = rightStickX;
      rotation = 0.0;
    }
    boolean controllerConnected = controllerConnectedSupplier.getAsBoolean();

    SmartDashboard.putBoolean("Drive/FieldRelativeMode", fieldRelativeMode);
    SmartDashboard.putBoolean("Drive/ControllerConnected", controllerConnected);
    SmartDashboard.putNumber("Drive/ForwardInput", forward);
    SmartDashboard.putNumber("Drive/StrafeInput", strafe);
    SmartDashboard.putNumber("Drive/RotationInput", rotation);
    SmartDashboard.putNumber("Drive/LeftStickStrafeInput", leftStickStrafe);
    SmartDashboard.putNumber("Drive/RightStickXInput", rightStickX);

    if (!DriverStation.isTeleopEnabled() || !controllerConnected || !swerve.isReady()
        || (fieldRelativeMode && !swerve.hasHeading())
        || !Double.isFinite(forward)
        || !Double.isFinite(strafe)
        || !Double.isFinite(rotation)) {
      resetLimiters();
      updateDashboard(new ChassisSpeeds());
      swerve.stop();
      return;
    }

    ChassisSpeeds requestedSpeeds = getSpeeds(forward, strafe, rotation);
    updateDashboard(requestedSpeeds);

    if (fieldRelativeMode) {
      swerve.driveFieldRelative(requestedSpeeds);
    } else {
      swerve.drive(requestedSpeeds);
    }
  }

  public void toggleDriveMode() {
    if (controlsNeutral() && swerve.isStopped()) {
      if (!fieldRelativeMode && !swerve.hasHeading()) {
        DriverStation.reportWarning("Field-relative drive requires a valid gyro reading.", false);
        return;
      }
      fieldRelativeMode = !fieldRelativeMode;
      resetLimiters();
    } else {
      DriverStation.reportWarning("Center the sticks and stop before changing drive mode.", false);
    }
  }

  public boolean controlsNeutral() {
    double forward = forwardSupplier.getAsDouble();
    double strafe = strafeSupplier.getAsDouble();
    double rotation = rotationSupplier.getAsDouble();
    return controllerConnectedSupplier.getAsBoolean()
        && Double.isFinite(forward) && Double.isFinite(strafe) && Double.isFinite(rotation)
        && Math.hypot(forward, strafe) <= Constants.OperatorConstants.kDeadband
        && Math.abs(rotation) <= Constants.OperatorConstants.kDeadband;
  }

  private ChassisSpeeds getSpeeds(double forward, double strafe, double rotation) {
    double deadband = Constants.OperatorConstants.kDeadband;
    forward = MathUtil.clamp(forward, -1.0, 1.0);
    strafe = MathUtil.clamp(strafe, -1.0, 1.0);
    double stickMagnitude = Math.hypot(forward, strafe);
    if (stickMagnitude <= deadband) {
      forward = 0.0;
      strafe = 0.0;
    } else {
      double scale = MathUtil.applyDeadband(Math.min(stickMagnitude, 1.0), deadband)
          / stickMagnitude;
      forward *= scale;
      strafe *= scale;
    }
    rotation = MathUtil.applyDeadband(MathUtil.clamp(rotation, -1.0, 1.0), deadband);

    Translation2d requestedTranslation = new Translation2d(forward, strafe);
    Translation2d change = requestedTranslation.minus(limitedTranslation);
    double maxChange = Constants.OperatorConstants.kTranslationSlewRatePerSecond
        * Constants.kLoopPeriodSeconds;
    if (change.getNorm() <= maxChange) {
      limitedTranslation = requestedTranslation;
    } else {
      limitedTranslation = limitedTranslation.plus(change.times(maxChange / change.getNorm()));
    }
    return new ChassisSpeeds(
        limitedTranslation.getX()
            * Constants.OperatorConstants.kForwardSpeedScale
            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        limitedTranslation.getY()
            * Constants.OperatorConstants.kStrafeSpeedScale
            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        rotationLimiter.calculate(rotation)
            * Constants.OperatorConstants.kRotationSpeedScale
            * Constants.SwerveConstants.kMaxAngularSpeedRadPerSec);
  }

  private void resetLimiters() {
    limitedTranslation = new Translation2d();
    rotationLimiter.reset(0.0);
  }

  private static void updateDashboard(ChassisSpeeds requestedSpeeds) {
    SmartDashboard.putNumber(
        "Drive/RequestedVxMetersPerSecond", requestedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive/RequestedVyMetersPerSecond", requestedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive/RequestedOmegaRadiansPerSecond", requestedSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    resetLimiters();
  }
}
