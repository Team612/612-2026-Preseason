package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** Converts Xbox stick input into field-relative or robot-relative swerve commands. */
public class ArcadeDrive extends Command {
  private final Swerve swerve;
  private final CommandXboxController controller;
  private boolean fieldRelativeMode;

  public ArcadeDrive(Swerve swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.stop();
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("Drive/FieldRelativeMode", fieldRelativeMode);
    SmartDashboard.putBoolean("Drive/ControllerConnected", controller.getHID().isConnected());
    SmartDashboard.putNumber("Drive/LeftX", controller.getLeftX());
    SmartDashboard.putNumber("Drive/LeftY", controller.getLeftY());
    SmartDashboard.putNumber("Drive/RightX", controller.getRightX());
    Translation2d translationInput =
        applyTranslationDeadband(
            -controller.getLeftY(),
            -controller.getLeftX(),
            Constants.OperatorConstants.kDeadband);
    double forwardInput = translationInput.getX();
    double strafeInput = translationInput.getY();
    double rotationInput =
        MathUtil.applyDeadband(-controller.getRightX(), Constants.OperatorConstants.kDeadband);

    if (forwardInput == 0.0 && strafeInput == 0.0 && rotationInput == 0.0) {
      swerve.stop();
      return;
    }

    ChassisSpeeds requestedSpeeds =
        toChassisSpeeds(forwardInput, strafeInput, rotationInput);
    SmartDashboard.putNumber("Drive/RequestedVxMetersPerSecond", requestedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Drive/RequestedVyMetersPerSecond", requestedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive/RequestedOmegaRadiansPerSecond", requestedSpeeds.omegaRadiansPerSecond);

    if (fieldRelativeMode) {
      swerve.driveFieldRelative(requestedSpeeds);
    } else {
      swerve.drive(requestedSpeeds);
    }
  }

  /** Switches between field-relative and robot-relative control. */
  public void toggleDriveMode() {
    fieldRelativeMode = toggledMode(fieldRelativeMode);
  }

  static boolean toggledMode(boolean currentlyFieldRelative) {
    return !currentlyFieldRelative;
  }

  /** Deadbands each left-stick axis independently, then caps diagonal magnitude at one. */
  static Translation2d applyTranslationDeadband(
      double forwardInput, double strafeInput, double deadband) {
    double forward = MathUtil.applyDeadband(forwardInput, deadband);
    double strafe = MathUtil.applyDeadband(strafeInput, deadband);
    double magnitude = Math.hypot(forward, strafe);
    if (magnitude > 1.0) {
      forward /= magnitude;
      strafe /= magnitude;
    }
    return new Translation2d(forward, strafe);
  }

  /** Maps normalized stick axes to chassis velocities in meters per second and radians per second. */
  static ChassisSpeeds toChassisSpeeds(
      double forwardInput, double strafeInput, double rotationInput) {
    return new ChassisSpeeds(
        forwardInput
            * Constants.OperatorConstants.kForwardSpeedScale
            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        strafeInput
            * Constants.OperatorConstants.kStrafeSpeedScale
            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        rotationInput
            * Constants.OperatorConstants.kRotationSpeedScale
            * Constants.SwerveConstants.kMaxAngularSpeedRadPerSec);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
