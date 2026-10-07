package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Swerve;

/** Owns the robot subsystems, driver controls, and command bindings. */
public class RobotContainer {
  private final Swerve swerve = new Swerve();
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(swerve, driverController);

  public RobotContainer() {
    // Drive whenever no oher command currently owns the drivetrain.
    swerve.setDefaultCommand(arcadeDrive);
    driverController.rightBumper().onTrue(Commands.runOnce(arcadeDrive::toggleDriveMode));
    driverController.start().onTrue(Commands.runOnce(swerve::resetHeading, swerve));
  }

  /** Returns the selected autonomous command; no routine is configured yet. */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
