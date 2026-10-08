package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
// Uncomment this import to test the optional slew rate limiting coded
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Converts Xbox stick input into field relative or robot relative swerve commands. i hope this works dont test unecessarily */
public class ArcadeDrive extends Command {
  private final Swerve swerve;
  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier controllerConnectedSupplier;

  private boolean fieldRelativeMode;

  public ArcadeDrive(
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

  @Override
  public void initialize() {
    swerve.stop();
  }

  @Override
  public void execute() {
    double forward = forwardSupplier.getAsDouble();
    double strafe = strafeSupplier.getAsDouble();
    double rotation = rotationSupplier.getAsDouble();
    boolean controllerConnected = controllerConnectedSupplier.getAsBoolean();

    SmartDashboard.putBoolean("Drive/FieldRelativeMode", fieldRelativeMode);
    SmartDashboard.putBoolean("Drive/ControllerConnected", controllerConnected);
    SmartDashboard.putNumber("Drive/ForwardInput", forward);
    SmartDashboard.putNumber("Drive/StrafeInput", strafe);
    SmartDashboard.putNumber("Drive/RotationInput", rotation);

    if (!controllerConnected
        || !Double.isFinite(forward)
        || !Double.isFinite(strafe)
        || !Double.isFinite(rotation)) {
      publishRequestedSpeeds(new ChassisSpeeds());
      swerve.stop();
      return;
    }

    ChassisSpeeds requestedSpeeds = fromControllerInputs(forward, strafe, rotation);

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

  /** Applies Xbox axis conventions before mapping the sticks to chassis velocity. */
  static ChassisSpeeds fromControllerAxes(double leftX, double leftY, double rightX) {
    return fromControllerInputs(-leftY, -leftX, -rightX);
  }

  /** Converts signed, normalized forward/left/counterclockwise inputs to chassis speeds. */
  static ChassisSpeeds fromControllerInputs(
      double forwardInput, double strafeInput, double rotationInput) {
    Translation2d translationInput =
        applyTranslationDeadband(
            forwardInput, strafeInput, Constants.OperatorConstants.kDeadband);
    double deadbandedRotation =
        MathUtil.applyDeadband(
            MathUtil.clamp(rotationInput, -1.0, 1.0), Constants.OperatorConstants.kDeadband);
    return toChassisSpeeds(
        translationInput.getX(), translationInput.getY(), deadbandedRotation);
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
