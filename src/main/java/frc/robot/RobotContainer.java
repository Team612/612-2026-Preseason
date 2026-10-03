// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.*;

public class RobotContainer {
  private Swerve m_swerve = new Swerve();

  //private CommandXboxController = new CommandXboxController(Constants.controllerPortNumber);
  //Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.controllerPOrtNumber);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerve.setDefaultCommand(new ArcadeDrive(m_swerve, m_driverController));
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.leftBumper().onTrue(new ResetEncoders(m_swerve));
    m_driverController.rightBumper().onTrue(new ResetEncoders(m_swerve));
    m_swerve.setDefaultCommand(new ArcadeDrive(m_swerve, m_driverController));

  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
