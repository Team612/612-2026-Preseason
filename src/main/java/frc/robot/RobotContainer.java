package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AprilTagAlignment;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Vision vision = new Vision((pose, timestamp, stdDevs) -> {});
    private final SendableChooser<Boolean> autonomousChooser = new SendableChooser<>();

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

        swerve.setDefaultCommand(swerveDrive);

        driverController.x()
            .and(this::teleopControllerReady)
            .toggleOnTrue(
                new AprilTagAlignment(swerve, vision, -1)
                    .until(() -> !teleopControllerReady()));

        driverController.leftBumper()
            .and(() -> DriverStation.isDisabled() || DriverStation.isTeleopEnabled())
            .onTrue(Commands.runOnce(this::resetHeadingWhenNeutral).ignoringDisable(true));

        driverController.rightBumper()
            .and(DriverStation::isTeleopEnabled)
            .onTrue(Commands.runOnce(swerveDrive::toggleDriveMode));
    }

    private void resetHeadingWhenNeutral() {
        if (swerveDrive.controlsNeutral()) {
            swerve.resetHeading();
        } else {
            DriverStation.reportWarning(
                "Center the sticks and stop before resetting heading.",
                false
            );
        }
    }

    private boolean teleopControllerReady() {
        return DriverStation.isTeleopEnabled()
            && driverController.getHID().isConnected();
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