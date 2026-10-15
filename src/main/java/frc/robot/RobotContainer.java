package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;

// Connects the controller, drive command, and driver buttons
public class RobotContainer {
  // drivetrain subsystem
  private final Swerve swerve = new Swerve();
  
  // Xbox controller on the configured Driver Station USB port on da right side
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  // Default command that continuously reads the driver axes in teleop mode, need to config further later
  private final SwerveDrive swerveDrive =
      new SwerveDrive(
          swerve,
          // Each () -> expression reads the live axis when the command asks for it.
          // Xbox axes are negated: forward, left, and counterclockwise es positivo
          () -> -driverController.getLeftY(),
          () -> -driverController.getLeftX(),
          () -> -driverController.getRightX(),
          () -> driverController.getHID().isConnected());

  // Installs the default drive command and binds mode switching and heading reset buttons
  public RobotContainer() {
    swerve.setDefaultCommand(swerveDrive);
    driverController.rightBumper().and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(swerveDrive::toggleDriveMode));
    driverController.start()
        .and(() -> DriverStation.isDisabled() || DriverStation.isTeleopEnabled())
        .onTrue(Commands.runOnce(this::resetHeadingWhenNeutral).ignoringDisable(true));
  }

  
  private void resetHeadingWhenNeutral() {
    if (swerveDrive.controlsNeutral()) {
      // The subsystem also verifies that all measured wheel speeds are stopped...
      swerve.resetHeading();
    } else {
      DriverStation.reportWarning(
          "Center the sticks and stop before resetting heading.", false);
    }
  }
  
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
