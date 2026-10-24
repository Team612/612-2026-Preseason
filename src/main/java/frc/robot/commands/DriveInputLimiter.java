// slew rate limiting in this file

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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

    Translation2d requestedTranslation = new Translation2d(forward, strafe);
    Translation2d change = requestedTranslation.minus(limitedTranslation);
    double translationRate = requestedTranslation.getNorm() == 0.0
        ? Constants.OperatorConstants.kTranslationReleaseSlewRatePerSecond
        : Constants.OperatorConstants.kTranslationSlewRatePerSecond;
    double maxChange = translationRate * elapsedSeconds;
    if (change.getNorm() <= maxChange) {
      limitedTranslation = requestedTranslation;
    } else {
      limitedTranslation = limitedTranslation.plus(change.times(maxChange / change.getNorm()));
    }
    double rotationRate = rotation == 0.0
        ? Constants.OperatorConstants.kRotationReleaseSlewRatePerSecond
        : Constants.OperatorConstants.kRotationSlewRatePerSecond;
    double maxRotationChange = rotationRate * elapsedSeconds;
    limitedRotation += MathUtil.clamp(
        rotation - limitedRotation, -maxRotationChange, maxRotationChange);
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

  void reset() {
    limitedTranslation = new Translation2d();
    limitedRotation = 0.0;
  }
}
