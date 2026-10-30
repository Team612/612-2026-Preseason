// slew rate limiting in this file

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

// Shapes and rate limits joystick inputs (for Junseo)
final class DriveInputLimiter {
  private Translation2d limitedTranslation = new Translation2d();
  private double limitedRotation;

  ChassisSpeeds calculate(double forward, double strafe, double rotation, double elapsedSeconds) {
    if (!Double.isFinite(forward) || !Double.isFinite(strafe)
        || !Double.isFinite(rotation)) {
      reset();
      return new ChassisSpeeds();
    }
    elapsedSeconds = Double.isFinite(elapsedSeconds)
        ? MathUtil.clamp(elapsedSeconds, 0.0, Constants.OperatorConstants.kMaxLimiterStepSeconds)
        : 0.0;
    double deadband = Constants.OperatorConstants.kDeadband;
    forward = MathUtil.clamp(forward, -1.0, 1.0);
    strafe = MathUtil.clamp(strafe, -1.0, 1.0);
    double stickMagnitude = Math.hypot(forward, strafe);
    if (stickMagnitude <= deadband) {
      forward = 0.0;
      strafe = 0.0;
    } else {
      double scale = MathUtil.applyDeadband(Math.min(stickMagnitude, 1.0), deadband)
          / stickMagnitude;
      forward *= scale;
      strafe *= scale;
    }
    rotation = MathUtil.applyDeadband(MathUtil.clamp(rotation, -1.0, 1.0), deadband);

    // Limit the requested chassis motion BEFORE storing slew history. Otherwise the
    // history winds up beyond feasible wheel speed and releasing rotation can
    // suddenly accelerate translation
    // |translation| +  da |omega| * radius bounds every wheel at every heading trust it works
    // keep field relative history feasible
    // as the chassis rotates under a fixed field translation demand
    double radius = Math.hypot(Constants.SwerveConstants.kWheelBaseMeters / 2.0,
        Constants.SwerveConstants.kTrackWidthMeters / 2.0);
    double translationDemand = Math.hypot(forward * Constants.OperatorConstants.kForwardSpeedScale,
        strafe * Constants.OperatorConstants.kStrafeSpeedScale);
    double rotationDemand = Math.abs(rotation * Constants.OperatorConstants.kRotationSpeedScale)
        * Constants.SwerveConstants.kMaxAngularSpeedRadPerSec * radius
        / Constants.SwerveConstants.kMaxSpeedMetersPerSecond;
    double scale = Math.max(1.0, translationDemand + rotationDemand);
    forward /= scale;
    strafe /= scale;
    rotation /= scale;

    Translation2d requestedTranslation = new Translation2d(forward, strafe);
    Translation2d change = requestedTranslation.minus(limitedTranslation);
    // Only a reduction along the same direction is braking. Direction changes
    // and reversals retain the acceleration limit rather than getting a faster ramp.
    double cross = requestedTranslation.getX() * limitedTranslation.getY()
        - requestedTranslation.getY() * limitedTranslation.getX();
    double dot = requestedTranslation.getX() * limitedTranslation.getX()
        + requestedTranslation.getY() * limitedTranslation.getY();
    boolean translationBraking = requestedTranslation.getNorm() <= limitedTranslation.getNorm()
        && dot >= 0.0 && Math.abs(cross) < 1e-9;
    boolean rotationBraking = Math.abs(rotation) <= Math.abs(limitedRotation)
        && rotation * limitedRotation >= 0.0;
    double translationRate = translationBraking
        ? Constants.OperatorConstants.kTranslationReleaseSlewRatePerSecond
        : Constants.OperatorConstants.kTranslationSlewRatePerSecond;
    double rotationRate = rotationBraking
        ? Constants.OperatorConstants.kRotationReleaseSlewRatePerSecond
        : Constants.OperatorConstants.kRotationSlewRatePerSecond;
    double rotationChange = rotation - limitedRotation;
    double translationSeconds = change.getNorm() / translationRate;
    double rotationSeconds = Math.abs(rotationChange) / rotationRate;
    // Share accelerating demand; braking keeps its own fast ramp, even when
    // the other stick requests acceleration. This avoids a long combined stop.
    double accelerationSeconds = (translationBraking ? 0.0 : translationSeconds)
        + (rotationBraking ? 0.0 : rotationSeconds);
    double translationFraction = rampFraction(elapsedSeconds,
        translationBraking ? translationSeconds : accelerationSeconds);
    double rotationFraction = rampFraction(elapsedSeconds,
        rotationBraking ? rotationSeconds : accelerationSeconds);
    limitedTranslation = limitedTranslation.plus(change.times(translationFraction));
    limitedRotation += rotationChange * rotationFraction;
    return new ChassisSpeeds(
        limitedTranslation.getX()
            * Constants.OperatorConstants.kForwardSpeedScale
            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        limitedTranslation.getY()
            * Constants.OperatorConstants.kStrafeSpeedScale
            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        limitedRotation
            * Constants.OperatorConstants.kRotationSpeedScale
            * Constants.SwerveConstants.kMaxAngularSpeedRadPerSec);
  }

  private static double rampFraction(double elapsedSeconds, double requiredSeconds) {
    return requiredSeconds > 0.0 ? Math.min(1.0, elapsedSeconds / requiredSeconds) : 1.0;
  }

  /** Expresses the existing translation demand in a new frame without stopping it. */
  void rotateReferenceFrame(Rotation2d rotation) {
    Translation2d scaled = new Translation2d(
        limitedTranslation.getX() * Constants.OperatorConstants.kForwardSpeedScale,
        limitedTranslation.getY() * Constants.OperatorConstants.kStrafeSpeedScale).rotateBy(rotation);
    limitedTranslation = new Translation2d(
        scaled.getX() / Constants.OperatorConstants.kForwardSpeedScale,
        scaled.getY() / Constants.OperatorConstants.kStrafeSpeedScale);
  }

  void reset() {
    limitedTranslation = new Translation2d();
    limitedRotation = 0.0;
  }
}
