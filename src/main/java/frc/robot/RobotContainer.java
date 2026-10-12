package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final Swerve swerve = new Swerve();
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final SwerveDrive swerveDrive =
      new SwerveDrive(
          swerve,
          // couple of da lambdas
          () -> -driverController.getLeftY(),
          () -> -driverController.getLeftX(),
          () -> -driverController.getRightX(),
          () -> driverController.getHID().isConnected());

  public RobotContainer() {
    swerve.setDefaultCommand(swerveDrive);
    driverController.rightBumper().onTrue(Commands.runOnce(swerveDrive::toggleDriveMode));
    driverController.start().onTrue(Commands.runOnce(() -> {
      if (swerveDrive.controlsNeutral()) {
        swerve.resetHeading();
      } else {
        DriverStation.reportWarning(
            "Center the sticks and stop before resetting heading.", false);
      }
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
