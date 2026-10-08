package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
// Uncomment this import to test the optional slew rate limiting coded
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** Converts Xbox stick input into field relative or robot relative swerve commands. i hope this works dont test unecessarily */
public class ArcadeDrive extends Command {
  private final Swerve swerve;
  private final CommandXboxController controller;

  /*
   * Optional acceleration limiting for future drive testing:
   * Uncomment the SlewRateLimiter import above and these fields, then uncomment the reset block
   * in initialize() and the calculation block in execute() i can do this later. Rates use normalized stick units/sec;
   * the negative rate must be negative. Translation has independent forward/strafe limiters, and
   * rotation has its own pair of acceleration/deceleration rates.
   *
   * private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(2.0, -4.0, 0.0);
   * private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.0, -4.0, 0.0);
   * private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0, -6.0, 0.0);
   */

  private boolean fieldRelativeMode;

  public ArcadeDrive(Swerve swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    /*
     * Uncomment with the limiter fields above so each command starts with no stored input:
     * forwardLimiter.reset(0.0);
     * strafeLimiter.reset(0.0);
     * rotationLimiter.reset(0.0);
     */
    swerve.stop();
  }

  @Override
  public void execute() {
    double leftX = controller.getLeftX();
    double leftY = controller.getLeftY();
    double rightX = controller.getRightX();
    boolean controllerConnected = controller.getHID().isConnected();

    SmartDashboard.putBoolean("Drive/FieldRelativeMode", fieldRelativeMode);
    SmartDashboard.putBoolean("Drive/ControllerConnected", controllerConnected);
    SmartDashboard.putNumber("Drive/LeftX", leftX);
    SmartDashboard.putNumber("Drive/LeftY", leftY);
    SmartDashboard.putNumber("Drive/RightX", rightX);

    if (!controllerConnected
        || !Double.isFinite(leftX)
        || !Double.isFinite(leftY)
        || !Double.isFinite(rightX)) {
      publishRequestedSpeeds(new ChassisSpeeds());
      swerve.stop();
      return;
    }

    ChassisSpeeds requestedSpeeds = fromControllerAxes(leftX, leftY, rightX);

    /*
     *
     *
     * Translation2d translationInput =
     *     applyTranslationDeadband(
     *         -leftY, -leftX, Constants.OperatorConstants.kDeadband);
     * double rotationInput =
     *     MathUtil.applyDeadband(
     *         MathUtil.clamp(-rightX, -1.0, 1.0), Constants.OperatorConstants.kDeadband);
     * double limitedForward = forwardLimiter.calculate(translationInput.getX());
     * double limitedStrafe = strafeLimiter.calculate(translationInput.getY());
     * double limitedRotation = rotationLimiter.calculate(rotationInput);
     * requestedSpeeds =
     *     toChassisSpeeds(limitedForward, limitedStrafe, limitedRotation);
     */

    publishRequestedSpeeds(requestedSpeeds);
    if (requestedSpeeds.vxMetersPerSecond == 0.0
        && requestedSpeeds.vyMetersPerSecond == 0.0
        && requestedSpeeds.omegaRadiansPerSecond == 0.0) {
      swerve.stop();
      return;
    }

    if (fieldRelativeMode) {
      swerve.driveFieldRelative(requestedSpeeds);
    } else {
      swerve.drive(requestedSpeeds);
    }
  }

  /** Switches between field relative and robot relative  */
  public void toggleDriveMode() {
    fieldRelativeMode = toggledMode(fieldRelativeMode);
  }

  static boolean toggledMode(boolean currentlyFieldRelative) {
    return !currentlyFieldRelative;
  }

  /** Deadbands each left stick axis independently, then caps diagonal magnitude at one. */
  static Translation2d applyTranslationDeadband(
      double forwardInput, double strafeInput, double deadband) {
    double forward = MathUtil.applyDeadband(MathUtil.clamp(forwardInput, -1.0, 1.0), deadband);
    double strafe = MathUtil.applyDeadband(MathUtil.clamp(strafeInput, -1.0, 1.0), deadband);
    double magnitude = Math.hypot(forward, strafe);
    if (magnitude > 1.0) {
      forward /= magnitude;
      strafe /= magnitude;
    }
    return new Translation2d(forward, strafe);
  }

  /** Applies Xbox stick-axis direction, deadband */
  static ChassisSpeeds fromControllerAxes(double leftX, double leftY, double rightX) {
    Translation2d translationInput =
        applyTranslationDeadband(
            -leftY, -leftX, Constants.OperatorConstants.kDeadband);
    double rotationInput =
        MathUtil.applyDeadband(
            MathUtil.clamp(-rightX, -1.0, 1.0), Constants.OperatorConstants.kDeadband);
    return toChassisSpeeds(
        translationInput.getX(), translationInput.getY(), rotationInput);
  }

  /** Maps normalized stick axes to chassis velocities in meters per second and radians per second units apparenlty do matter */
  static ChassisSpeeds toChassisSpeeds(
      double forwardInput, double strafeInput, double rotationInput) {
    double forward = MathUtil.clamp(forwardInput, -1.0, 1.0);
    double strafe = MathUtil.clamp(strafeInput, -1.0, 1.0);
    double translationMagnitude = Math.hypot(forward, strafe);
    if (translationMagnitude > 1.0) {
      forward /= translationMagnitude;
      strafe /= translationMagnitude;
    }
    return new ChassisSpeeds(
        forward
            * Constants.OperatorConstants.kForwardSpeedScale
            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        strafe
            * Constants.OperatorConstants.kStrafeSpeedScale
            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        MathUtil.clamp(rotationInput, -1.0, 1.0)
            * Constants.OperatorConstants.kRotationSpeedScale
            * Constants.SwerveConstants.kMaxAngularSpeedRadPerSec);
  }

  private static void publishRequestedSpeeds(ChassisSpeeds requestedSpeeds) {
    SmartDashboard.putNumber(
        "Drive/RequestedVxMetersPerSecond", requestedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive/RequestedVyMetersPerSecond", requestedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive/RequestedOmegaRadiansPerSecond", requestedSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
