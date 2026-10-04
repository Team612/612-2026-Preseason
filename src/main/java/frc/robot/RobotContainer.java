package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final Swerve m_swerve = new Swerve();
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    m_swerve.setDefaultCommand(new ArcadeDrive(m_swerve, m_driverController));
    m_driverController
        .start()
        .onTrue(Commands.runOnce(m_swerve::resetHeading, m_swerve));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
