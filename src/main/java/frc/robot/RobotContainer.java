package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final XboxController driverController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final Swerve swerve = new Swerve();

  // the slew rate limiters added should limit the ROC of joystick inputs so we dont go 0 to 1 mil and brownie
  private final SlewRateLimiter forwardLimiter =
      new SlewRateLimiter(Constants.SwerveConstants.kTranslationSlewMetersPerSecondSquared);
  private final SlewRateLimiter strafeLimiter =
      new SlewRateLimiter(Constants.SwerveConstants.kTranslationSlewMetersPerSecondSquared);
  private final SlewRateLimiter rotationLimiter =
      new SlewRateLimiter(Constants.SwerveConstants.kRotationSlewRadPerSecSquared);

  public RobotContainer() {
    swerve.setDefaultCommand(new RunCommand(this::driveRobotRelative, swerve).finallyDo(swerve::stop));
  }

  private void driveRobotRelative() {
    double forward = shape(-driverController.getLeftY());
    double strafe = shape(-driverController.getLeftX());
    double rotation = shape(-driverController.getRightX());
    swerve.drive(
        new ChassisSpeeds(
            forwardLimiter.calculate(forward * Constants.SwerveConstants.kMaxSpeedMetersPerSecond),
            strafeLimiter.calculate(strafe * Constants.SwerveConstants.kMaxSpeedMetersPerSecond),
            rotationLimiter.calculate(rotation * Constants.SwerveConstants.kMaxAngularSpeedRadPerSec)));
  }

  private double shape(double input) {
    double deadbanded = MathUtil.applyDeadband(input, Constants.OperatorConstants.kDeadband);
    return Math.copySign(deadbanded * deadbanded, deadbanded);
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
