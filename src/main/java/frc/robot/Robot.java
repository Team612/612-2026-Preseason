package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Runs commands every 20 ms 
public class Robot extends TimedRobot {
  
  private Command autonomousCommand;
  private final HootAutoReplay timeAndJoystickReplay = new HootAutoReplay()
      .withTimestampReplay().withJoystickReplay();

  // Owns the subsystem, controller, and button bindings 
  private final RobotContainer robotContainer;

  /** Creates controller bindings and selects the loop period from Constants. */
  public Robot() {
    super(Constants.kLoopPeriodSeconds);
    robotContainer = new RobotContainer();
  }

  /** Runs subsystem periodic methods and scheduled commands in every robot mode. */
  @Override
  public void robotPeriodic() {
    timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
  }

  /** Schedules the autonomous command when autonomous begins; selected on the dashboard. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** Cancels the autonomous command so driver control can take over. */
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

}
