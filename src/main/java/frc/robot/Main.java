package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// Program beginning of life
public final class Main {
  // No instances needed.
  private Main() {}

  // Starts the WPILib robot loop
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
