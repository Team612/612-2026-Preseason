// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;

/** Factories for autonomous commands. Add completed routines here as they are implemented. */
public final class Autos {
  /** Demonstrates returning a command supplied by a subsystem. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return subsystem.exampleMethodCommand();
  }

  private Autos() {}
}
