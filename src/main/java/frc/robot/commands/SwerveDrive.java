package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Reads driver input; only the translation reference frame changes between modes. */
public class SwerveDrive extends Command {
  // Subsystem controlled exclusively while this command is scheduled.
  private final Swerve swerve;
  // Reads normalized forward/backward input from the controller.
  private final DoubleSupplier forwardSupplier;
  // Reads normalized left/right translation input in both driving modes.
  private final DoubleSupplier strafeSupplier;
  // Reads normalized counterclockwise/clockwise rotation input in both modes.
  private final DoubleSupplier rotationSupplier;
  // Reports whether controller input can be trusted.
  private final BooleanSupplier controllerConnectedSupplier;

  // Stores the previous inputs for deadband shaping and acceleration limits.
  private final DriveInputLimiter inputLimiter = new DriveInputLimiter();
  // FPGA timestamp used for elapsed time and missed-loop detection.
  private double lastLimiterTimestampSeconds;
  // True at startup: translate relative to field heading rather than the robot nose.
  private boolean fieldRelativeMode = true;
  // Prevents held sticks from restarting motion after enable, a fault, or a long loop gap.
  private boolean waitingForNeutral = true;

  /**
   * Receives axis readers and reserves the drivetrain so competing drive commands cannot run
   * together.
   */
  public SwerveDrive(
      Swerve swerve,
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier controllerConnectedSupplier) {
    this.swerve = Objects.requireNonNull(swerve);
    this.forwardSupplier = Objects.requireNonNull(forwardSupplier);
    this.strafeSupplier = Objects.requireNonNull(strafeSupplier);
    this.rotationSupplier = Objects.requireNonNull(rotationSupplier);
    this.controllerConnectedSupplier = Objects.requireNonNull(controllerConnectedSupplier);
    addRequirements(this.swerve);
  }

  /** Stops previous output and requires neutral sticks whenever this command starts. */
  @Override
  public void initialize() {
    stopAndRequireNeutral();
  }

  // check sensors and shape inputs
  @Override
  public void execute() {
    double forward = forwardSupplier.getAsDouble();
    // The left stick translates and the right stick rotates in BOTH modes please tell me if this needs ot be adjusted
    double strafe = strafeSupplier.getAsDouble();
    double rotation = rotationSupplier.getAsDouble();
    boolean controllerConnected = controllerConnectedSupplier.getAsBoolean();


    // add to smartdashbaord
    SmartDashboard.putBoolean("Drive/FieldRelativeMode", fieldRelativeMode);
    SmartDashboard.putBoolean("Drive/ControllerConnected", controllerConnected);
    SmartDashboard.putNumber("Drive/ForwardInput", forward);
    SmartDashboard.putNumber("Drive/StrafeInput", strafe);
    SmartDashboard.putNumber("Drive/RotationInput", rotation);

    // if the driverstation is not in teleop or the controller isnt connceted or swerve is not ready or the rest, stop and wait for neutral joystick
    if (!DriverStation.isTeleopEnabled() || !controllerConnected || !swerve.isReady()
        || (fieldRelativeMode && !swerve.hasHeading())
        || !Double.isFinite(forward)
        || !Double.isFinite(strafe)
        || !Double.isFinite(rotation)) {
      stopAndRequireNeutral();
      return;
    }

    // prevent jumps
    if (Timer.getFPGATimestamp() - lastLimiterTimestampSeconds
        >= Constants.SwerveConstants.kMotorSafetyExpirationSeconds) {
      stopAndRequireNeutral();
      return;
    }

    if (waitingForNeutral) {
      resetLimiters();
      swerve.stop();
      if (controlsNeutral()) {
        waitingForNeutral = false;
      }
      SmartDashboard.putBoolean("Drive/WaitingForNeutral", waitingForNeutral);
      updateDashboard(new ChassisSpeeds());
      return;
    }

    ChassisSpeeds requestedSpeeds = getSpeeds(forward, strafe, rotation);
    updateDashboard(requestedSpeeds);

    if (fieldRelativeMode) {
      if (!swerve.driveOperatorRelative(requestedSpeeds)) {
        stopAndRequireNeutral();
      }
    } else {
      if (!swerve.drive(requestedSpeeds)) {
        stopAndRequireNeutral();
      }
    }
  }

  // to look over again, 
  public void toggleDriveMode() {
    if (controlsNeutral() && swerve.isStopped()) {
      // Robot-relative driving can still work when the gyro is unavailable.
      if (!fieldRelativeMode && !swerve.hasHeading()) {
        DriverStation.reportWarning("Field-relative drive requires a valid gyro reading.", false);
        return;
      }
      fieldRelativeMode = !fieldRelativeMode;
      resetLimiters();
    } else {
      DriverStation.reportWarning("Center the sticks and stop before changing drive mode.", false);
    }
  }

  // Checks controller connection and both deadbands before changing mode or resetting heading (should help)
  public boolean controlsNeutral() {
    double forward = forwardSupplier.getAsDouble();
    double strafe = strafeSupplier.getAsDouble();
    double rotation = rotationSupplier.getAsDouble();
    return controllerConnectedSupplier.getAsBoolean()
        && Double.isFinite(forward) && Double.isFinite(strafe) && Double.isFinite(rotation)
        && Math.hypot(forward, strafe) <= Constants.OperatorConstants.kDeadband
        && Math.abs(rotation) <= Constants.OperatorConstants.kDeadband;
  }

  
  // Uses a shared elapsed time for translation and rotation, before the field-to-robot conversio
  private ChassisSpeeds getSpeeds(double forward, double strafe, double rotation) {
    // Use one elapsed time for both limiters, including scheduler 
    // These inputs stay in the selected driving frame until after rate limiting
    double now = Timer.getFPGATimestamp();
    double elapsedSeconds = Math.max(0.0, now - lastLimiterTimestampSeconds);
    lastLimiterTimestampSeconds = now;
    return inputLimiter.calculate(forward, strafe, rotation, elapsedSeconds);
  }

  //Stops immediately after a fault
  
  private void stopAndRequireNeutral() {
    waitingForNeutral = true;
    swerve.stop();
    resetLimiters();
    SmartDashboard.putBoolean("Drive/WaitingForNeutral", true);
    updateDashboard(new ChassisSpeeds());
  }

  // Clears input history and restarts the elapsed time clock for my sanity
  private void resetLimiters() {
    inputLimiter.reset();
    lastLimiterTimestampSeconds = Timer.getFPGATimestamp();
  }

  /** Publishes the requested chassis velocities for driver feedback and problemo debugging */
  private static void updateDashboard(ChassisSpeeds requestedSpeeds) {
    SmartDashboard.putNumber(
        "Drive/RequestedVxMetersPerSecond", requestedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive/RequestedVyMetersPerSecond", requestedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive/RequestedOmegaRadiansPerSecond", requestedSpeeds.omegaRadiansPerSecond);
  }

  // Stops the drivetrain when this command finishes or another command interrupts it
  @Override
  public void end(boolean interrupted) {
    stopAndRequireNeutral();
  }
}
