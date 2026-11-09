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
  // Prevents held sticks from starting motion after enable, disconnect, or a hardware fault.
  private boolean waitingForNeutral = true;
  private boolean hardwareBlocked;
  private int loopGapCount;

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
    hardwareBlocked = false;
    stopAndRequireNeutral("Center sticks to arm drive");
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

    String blockedReason = getBlockedReason(forward, strafe, rotation, controllerConnected);
    if (blockedReason != null) {
      // Reconnection and invalid input always require a fresh neutral observation.
      // For a continuing hardware fault, remember neutral seen while waiting;
      // do not make the driver center a second time when sensors finally arrive.
      boolean trustedInput = DriverStation.isTeleopEnabled() && controllerConnected
          && Double.isFinite(forward) && Double.isFinite(strafe) && Double.isFinite(rotation);
      if (!hardwareBlocked || !trustedInput) {
        waitingForNeutral = true;
      }
      hardwareBlocked = true;
      if (trustedInput && inputsNeutral(forward, strafe, rotation)) {
        waitingForNeutral = false;
      }
      swerve.stop();
      resetLimiters();
      SmartDashboard.putBoolean("Drive/WaitingForNeutral", waitingForNeutral);
      SmartDashboard.putString("Drive/Status", blockedReason);
      SmartDashboard.putString("Drive/LastBlockedReason", blockedReason);
      updateDashboard(new ChassisSpeeds());
      return;
    }
    hardwareBlocked = false;

    double loopGap = Timer.getFPGATimestamp() - lastLimiterTimestampSeconds;
    SmartDashboard.putNumber("Drive/LoopElapsedSeconds", loopGap);
    if (loopGap >= Constants.SwerveConstants.kMotorSafetyExpirationSeconds) {
      // The motor watchdog may already have stopped ze output pls restart ramp
      // from rest, without latching a neutral interlock forda isolated slow loop
      loopGapCount++;
      SmartDashboard.putNumber("Drive/LoopGapCount", loopGapCount);
      SmartDashboard.putString("Drive/LastBlockedReason", "Drive loop gap: ramp restarted");
      swerve.stop();
      resetLimiters();
      updateDashboard(new ChassisSpeeds());
      SmartDashboard.putString("Drive/Status", "Drive loop gap: ramp restarted");
      return;
    }

    if (waitingForNeutral) {
      resetLimiters();
      swerve.stop();
      if (inputsNeutral(forward, strafe, rotation)) {
        waitingForNeutral = false;
      }
      SmartDashboard.putBoolean("Drive/WaitingForNeutral", waitingForNeutral);
      SmartDashboard.putString("Drive/Status",
          waitingForNeutral ? "Center sticks to arm drive" : "Ready");
      updateDashboard(new ChassisSpeeds());
      return;
    }

    SmartDashboard.putString("Drive/Status", "Ready");
    ChassisSpeeds requestedSpeeds = getSpeeds(forward, strafe, rotation);
    updateDashboard(requestedSpeeds);

    if (fieldRelativeMode) {
      if (!swerve.driveOperatorRelative(requestedSpeeds)) {
        stopAndRequireNeutral("Drivetrain rejected command");
      }
    } else {
      if (!swerve.drive(requestedSpeeds)) {
        stopAndRequireNeutral("Drivetrain rejected command");
      }
    }
  }

  /** Switches immediately while moving, preserving the current physical velocity demand. */
  public void toggleDriveMode() {
    var heading = swerve.getOperatorRelativeHeading();
    if (!fieldRelativeMode && heading == null) {
      DriverStation.reportWarning("Field-relative drive requires a valid gyro reading.", false);
      return;
    }
    if (heading != null) {
      inputLimiter.rotateReferenceFrame(fieldRelativeMode ? heading.unaryMinus() : heading);
    } else {
      // Leaving field mode remains possible after loss of heading.
      resetLimiters();
    }
    fieldRelativeMode = !fieldRelativeMode;
    SmartDashboard.putBoolean("Drive/FieldRelativeMode", fieldRelativeMode);
  }

  // Neutral is still required for heading resets and characterization.
  public boolean controlsNeutral() {
    double forward = forwardSupplier.getAsDouble();
    double strafe = strafeSupplier.getAsDouble();
    double rotation = rotationSupplier.getAsDouble();
    return controllerConnectedSupplier.getAsBoolean() && inputsNeutral(forward, strafe, rotation);
  }

  private static boolean inputsNeutral(double forward, double strafe, double rotation) {
    return Double.isFinite(forward) && Double.isFinite(strafe) && Double.isFinite(rotation)
        && Math.hypot(forward, strafe) <= Constants.OperatorConstants.kDeadband
        && Math.abs(rotation) <= Constants.OperatorConstants.kDeadband;
  }

  private String getBlockedReason(
      double forward, double strafe, double rotation, boolean controllerConnected) {
    if (!DriverStation.isTeleopEnabled()) return "Teleop not enabled";
    if (!controllerConnected) return "Controller disconnected";
    if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(rotation)) {
      return "Invalid controller input";
    }
    if (!swerve.isReady()) return "Modules not ready: check Swerve and module diagnostics";
    if (fieldRelativeMode && !swerve.hasHeading()) return "Field heading unavailable";
    return null;
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
  
  private void stopAndRequireNeutral(String reason) {
    SmartDashboard.putString("Drive/Status", reason);
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
    stopAndRequireNeutral("Drive command ended");
  }
}
