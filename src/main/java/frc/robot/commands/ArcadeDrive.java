package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class ArcadeDrive extends Command {
  private final Swerve swerve;
  private final CommandXboxController controller;

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
    double forward = MathUtil.applyDeadband(-controller.getLeftY(), Constants.DEADBAND);
    double strafe = MathUtil.applyDeadband(-controller.getLeftX(), Constants.DEADBAND);
    double rotation = MathUtil.applyDeadband(-controller.getRightX(), Constants.DEADBAND);
    if (forward == 0.0 && strafe == 0.0 && rotation == 0.0) {
      swerve.stop();
      return;
    }

    double linearMagnitude = Math.hypot(forward, strafe);
    double linearScale = linearMagnitude > 1.0 ? 1.0 / linearMagnitude : 1.0;
    double normalizedForward = forward * linearScale;
    double normalizedStrafe = strafe * linearScale;

    ChassisSpeeds requestedSpeeds =
        new ChassisSpeeds(
            normalizedForward * Constants.xPercent * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
            normalizedStrafe * Constants.yPercent * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
            rotation
                * Constants.zPercent
                * Constants.SwerveConstants.kMaxAngularSpeedRadPerSec);

    if (controller.rightBumper().getAsBoolean()) {
      swerve.drive(requestedSpeeds);
    } else {
      swerve.driveFieldRelative(requestedSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
