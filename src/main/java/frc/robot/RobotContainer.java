package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;

// Connects the controller, drive command, and driver buttons
public class RobotContainer {
  // drivetrain subsystem
  private final Swerve swerve = new Swerve();
  
  private final SendableChooser<Boolean> autonomousChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> sysIdChooser = new SendableChooser<>();

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
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    autonomousChooser.setDefaultOption("Do nothing", false);
    autonomousChooser.addOption("Forward 0.5 m/s for 5 seconds", true);
    SmartDashboard.putData("Auto/Mode", autonomousChooser);
    sysIdChooser.setDefaultOption("Drive translation", false);
    sysIdChooser.addOption("Steering", true);
    SmartDashboard.putData("SysId/Mechanism", sysIdChooser);
    swerve.setDefaultCommand(swerveDrive);
    driverController.a().and(this::teleopControllerReady)
        .whileTrue(swerve.run(swerve::brake).finallyDo(swerve::stop));
    driverController.b().and(this::teleopControllerReady)
        .whileTrue(swerve.run(this::pointWheelsFromStick).finallyDo(swerve::stop));
    driverController.leftBumper()
        .and(() -> DriverStation.isDisabled() || DriverStation.isTeleopEnabled())
        .onTrue(Commands.runOnce(this::resetHeadingWhenNeutral).ignoringDisable(true));
    driverController.back().and(driverController.y()).and(this::testControllerReady)
        .whileTrue(characterizationCommand(true, Direction.kForward));
    driverController.back().and(driverController.x()).and(this::testControllerReady)
        .whileTrue(characterizationCommand(true, Direction.kReverse));
    driverController.start().and(driverController.y()).and(this::testControllerReady)
        .whileTrue(characterizationCommand(false, Direction.kForward));
    driverController.start().and(driverController.x()).and(this::testControllerReady)
        .whileTrue(characterizationCommand(false, Direction.kReverse));
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
  
  private boolean teleopControllerReady() {
    return DriverStation.isTeleopEnabled() && driverController.getHID().isConnected();
  }

  private boolean testControllerReady() {
    return DriverStation.isTestEnabled() && swerveDrive.controlsNeutral();
  }

  private void pointWheelsFromStick() {
    double x = -driverController.getLeftY();
    double y = -driverController.getLeftX();
    if (!Double.isFinite(x) || !Double.isFinite(y)) {
      swerve.stop();
    } else if (Math.hypot(x, y) > Constants.OperatorConstants.kDeadband) {
      swerve.pointWheelsAt(new Rotation2d(x, y));
    } else {
      swerve.stop();
    }
  }

  private Command characterizationCommand(boolean dynamic, Direction direction) {
    // Resolve the dashboard choice when the buttons are pressed, then keep it fixed for the test.
    return Commands.defer(() -> swerve.sysId(Boolean.TRUE.equals(sysIdChooser.getSelected()),
        dynamic, direction), Set.of(swerve));
  }

  public Command getAutonomousCommand() {
    if (!Boolean.TRUE.equals(autonomousChooser.getSelected())) {
      return swerve.runOnce(swerve::stop);
    }
    // Robot-forward motion avoids depending on an arbitrary startup gyro heading.
    return swerve.run(() -> swerve.drive(new ChassisSpeeds(0.5, 0.0, 0.0)))
        .withTimeout(5.0)
        .until(() -> !DriverStation.isAutonomousEnabled() || !swerve.isReady())
        .finallyDo(swerve::stop);
  }
}
