package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AprilTagAlignment;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.util.Set;

// Connects the controller, drive command, and driver buttons
public class RobotContainer {
  // Drivetrain & Vision subsystems
  private final Swerve swerve = new Swerve();

  // Pass swerve::addVisionMeasurement here once added to Swerve.java, or keep no-op consumer
  private final Vision vision = new Vision((pose, timestamp, stdDevs) -> {});

  private final SendableChooser<Boolean> autonomousChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> sysIdChooser = new SendableChooser<>();

  private final CommandXboxController driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  private final SwerveDrive swerveDrive =
      new SwerveDrive(
          swerve,
          () -> -driverController.getLeftY(),
          () -> -driverController.getLeftX(),
          () -> -driverController.getRightX(),
          () -> driverController.getHID().isConnected());

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

    // X-brake while holding A
    driverController
        .a()
        .and(this::teleopControllerReady)
        .whileTrue(swerve.run(swerve::brake).finallyDo(swerve::stop));

    // Point wheels in stick direction while holding B
    driverController
        .b()
        .and(this::teleopControllerReady)
        .whileTrue(swerve.run(this::pointWheelsFromStick).finallyDo(swerve::stop));

    // Align to best visible AprilTag (centered) while holding X
    driverController
        .x()
        .and(this::teleopControllerReady)
        .whileTrue(new AprilTagAlignment(swerve, vision, -1));

    // Align to best visible AprilTag with right-side offset while holding Y
    driverController
        .y()
        .and(this::teleopControllerReady)
        .whileTrue(
            new AprilTagAlignment(
                swerve,
                vision,
                -1,
                VisionConstants.kAlignTargetXDistanceMeters,
                VisionConstants.kAlignLateralOffsetRightMeters));

    // Reset field heading on Left Bumper
    driverController
        .leftBumper()
        .and(() -> DriverStation.isDisabled() || DriverStation.isTeleopEnabled())
        .onTrue(Commands.runOnce(this::resetHeadingWhenNeutral).ignoringDisable(true));

    // Toggle field-oriented / robot-oriented drive mode on Right Bumper
    driverController
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(swerveDrive::toggleDriveMode));

    // SysId characterization bindings (Test mode only)
    driverController
        .back()
        .and(driverController.y())
        .and(this::testControllerReady)
        .whileTrue(characterizationCommand(true, Direction.kForward));
    driverController
        .back()
        .and(driverController.x())
        .and(this::testControllerReady)
        .whileTrue(characterizationCommand(true, Direction.kReverse));
    driverController
        .start()
        .and(driverController.y())
        .and(this::testControllerReady)
        .whileTrue(characterizationCommand(false, Direction.kForward));
    driverController
        .start()
        .and(driverController.x())
        .and(this::testControllerReady)
        .whileTrue(characterizationCommand(false, Direction.kReverse));
  }

  private void resetHeadingWhenNeutral() {
    if (swerveDrive.controlsNeutral()) {
      swerve.resetHeading();
    } else {
      DriverStation.reportWarning("Center the sticks and stop before resetting heading.", false);
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
    return Commands.defer(
        () ->
            swerve.sysId(
                Boolean.TRUE.equals(sysIdChooser.getSelected()), dynamic, direction),
        Set.of(swerve));
  }

  public Command getAutonomousCommand() {
    if (!Boolean.TRUE.equals(autonomousChooser.getSelected())) {
      return swerve.runOnce(swerve::stop);
    }
    return swerve
        .run(() -> swerve.drive(new ChassisSpeeds(0.5, 0.0, 0.0)))
        .withTimeout(5.0)
        .until(() -> !DriverStation.isAutonomousEnabled() || !swerve.isReady())
        .finallyDo(swerve::stop);
  }
}
