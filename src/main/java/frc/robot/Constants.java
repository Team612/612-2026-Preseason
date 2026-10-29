package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public final class Constants {

  // Control loop period in seconds 
  public static final double kLoopPeriodSeconds = 0.02;

  // Steering axis spacing left to right, in meters.
  public static final double trackWidth = 0.550;

  // Steering-axis spacing front to rear, in meters.
  public static final double wheelBase = 0.555;

  // Module order: front left, front right, rear left, rear right.
  public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

  // Check offsets again after changing encoder direction.
  // Front left CAN IDs and directions.
  public static final int frontLeftDriveMotorId = 1;

  public static final int frontLeftSteerMotorId = 2;

  public static final int frontLeftEncoderId = 1;

  public static final boolean frontLeftDriveInverted = false;

  public static final boolean frontLeftSteerInverted = true;

  public static final boolean frontLeftEncoderInverted = false;

  // Forward encoder reading in rotations, after direction correction.
  public static final double frontLeftEncoderOffsetRotations = -0.284180;

  // Front right CAN IDs and directions
  public static final int frontRightDriveMotorId = 3;

  public static final int frontRightSteerMotorId = 4;

  public static final int frontRightEncoderId = 3;

  public static final boolean frontRightDriveInverted = true;

  public static final boolean frontRightSteerInverted = true;

  public static final boolean frontRightEncoderInverted = false;

  // Forward encoder reading in rotations, after direction correction.
  public static final double frontRightEncoderOffsetRotations = 0.364990;

  // Rear right CAN IDs and directions.
  public static final int rearRightDriveMotorId = 5;

  public static final int rearRightSteerMotorId = 6;

  public static final int rearRightEncoderId = 5;

  public static final boolean rearRightDriveInverted = true;

  public static final boolean rearRightSteerInverted = true;

  public static final boolean rearRightEncoderInverted = false;

  // Forward encoder reading in rotations, after direction correction.
  public static final double rearRightEncoderOffsetRotations = -0.289307;

  // Rear left CAN IDs and directions.
  public static final int rearLeftDriveMotorId = 7;

  public static final int rearLeftSteerMotorId = 8;

  public static final int rearLeftEncoderId = 7;

  public static final boolean rearLeftDriveInverted = false;

  public static final boolean rearLeftSteerInverted = true;

  public static final boolean rearLeftEncoderInverted = false;

  // Forward encoder reading in rotations, after direction correction.
  public static final double rearLeftEncoderOffsetRotations = 0.226318;

  // Driver input settings.
  public static final class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    // Shared by drive shaping and enable neutral detection, allowing small resting offsets.
    // Translation uses stick magnitude; rotation uses one axis.
    public static final double kDeadband = 0.06;

    // Stick speed multipliers.
    public static final double kForwardSpeedScale = 1.0;

    public static final double kStrafeSpeedScale = 1.0;

    public static final double kRotationSpeedScale = 1.0;

    // Normalized input change per second: about 0.33 s from rest to full demand.
    // Translation and rotation share this budget when commanded together.
    public static final double kTranslationSlewRatePerSecond = 3.0;
    public static final double kRotationSlewRatePerSecond = 3.0;

    // Same-direction slowdowns and release brake in about 0.20 s from full demand.
    public static final double kTranslationReleaseSlewRatePerSecond = 5.0;
    public static final double kRotationReleaseSlewRatePerSecond = 5.0;

    // Caps elapsed time to prevent input jumps after a delay.
    public static final double kMaxLimiterStepSeconds = 0.04;

    // No instances needed.
    private OperatorConstants() {}
  }

  
  public static final class SwerveConstants {

    public static final CANBus kCanBus = CANBus.roboRIO();

    public static final int kPigeonCanId = 0;

    public static final double kWheelBaseMeters = wheelBase;

    public static final double kTrackWidthMeters = trackWidth;

    // Verify wheel diameter and gearing on the robot.
    public static final double kWheelDiameterMeters = 0.1016; // 4 inch diameter may need to be tuned out

    // Motor rotations per wheel rotation
    public static final double kDriveReduction = 6.75;

    public static final double kMaxSpeedMetersPerSecond = 2.25; // last working version at 2.0 (this val based on gear ratio)

    // Start threshold to avoid steering at near zero speed (mucho slow)
    public static final double kMinimumModuleSpeedMetersPerSecond = 0.03;

    // Lower stop threshold prevents start/stop jitterin
    public static final double kModuleStopSpeedMetersPerSecond = 0.015;

    // Rotation cap based on wheel speed and chassis
    public static final double kMaxAngularSpeedRadPerSec = Math.min(
        Math.PI * 2.0,
        kMaxSpeedMetersPerSecond / Math.hypot(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0));

    public static final double kMaxDriveVolts = 12.0; // last working version at 3.0 (this val was at 25% power)

    // Feedforward volts per m/s; requires tuning
    public static final double kDriveKV = 2.0;

    // Drive feedforward: V and V/(m/s²). Zero until measured with SysId.
    public static final double kDriveKS = 0.0;
    public static final double kDriveKA = 0.0;

    // Voltage ramp periods (seconds from 0 to 12 V). Zero preserves existing tuning.
    public static final double kDriveOpenLoopRampSeconds = 0.0;
    public static final double kDriveClosedLoopRampSeconds = 0.0;
    public static final double kTurnOpenLoopRampSeconds = 0.0;
    public static final double kTurnClosedLoopRampSeconds = 0.0;

    // Speed correction volts per m/s.
    public static final double kDriveP = 2.0; // last working version at 1.0; tune these on blocks and make driving setup faster

    public static final double kMaxDriveCorrectionVolts = 2.75; // last working verison at 1.0 ; tune on blocks and make turning aggressive

    // Stationary threshold for heading or mode changes.
    public static final double kStoppedSpeedMetersPerSecond = 0.05;

    // Stop if required drive feedback is unavailable.
    public static final boolean kEnableDriveVelocityCorrection = true;

    // Steering PID gains; angles use radians.
    public static final double kTurnP = 0.5;

    public static final double kTurnI = 0.0;

    public static final double kTurnD = 0.0;

    // Velocity damping in volts per rad/s.
    public static final double kTurnVelocityDamping = 0.15;

    // Below this rate, steering can stop within tolerance.
    public static final double kTurnSettledVelocityRadPerSec = 0.1;

    // Buffer around 90 degrees before reversing wheel direction.
    public static final double kTurnOptimizationHysteresisRad = Math.toRadians(5.0);

    // Steering tolerance in degrees, not percent or encoder resolution. //ONE OF THE THINGS DONE 
    public static final double kTurnToleranceDegrees = 2.7;

    public static final double kTurnToleranceRad = Math.toRadians(kTurnToleranceDegrees);

    // Encoder rounding, separate from steering tolerance. // also glad this is here (from docs)
    public static final double kCancoderCountsPerRotation = 4096.0;

    // Critical sensor update rate.
    public static final double kSensorUpdateFrequencyHz = 100.0;

    // Reject older sensor feedback.
    public static final double kMaxSensorAgeSeconds = 0.1;

    // missed loop end cutoff
    public static final double kMotorSafetyExpirationSeconds = 0.1;

    // Extra error allowed before steering restarts.
    public static final double kTurnToleranceHysteresisRad = Math.toRadians(1.0);

    // Steering profile speed and acceleration limits.
    public static final double kMaxTurnRadPerSec = Math.PI * 2.0;

    public static final double kMaxTurnAccelRadPerSecSquared = Math.PI * 4.0;

    public static final double kMaxTurnVolts = 12.0; // last working version at 4.0; the new value will be more crispy

    // Supply limits battery current; stator limits motor torque. // no brownies
    public static final double kDriveSupplyLimitAmps = 40.0; // last working version at 12.0

    public static final double kDriveStatorLimitAmps = 60.0; // last working version at 12.0

    public static final double kTurnSupplyLimitAmps = 25.0; // last working version at 12.0

    public static final double kTurnStatorLimitAmps = 40.0; // last working version at 12.0

    public static final ModuleConfiguration kFrontLeft =
        new ModuleConfiguration(
            frontLeftDriveMotorId, frontLeftSteerMotorId, frontLeftEncoderId,
            frontLeftEncoderOffsetRotations,
            frontLeftDriveInverted, frontLeftSteerInverted, frontLeftEncoderInverted);

    public static final ModuleConfiguration kFrontRight =
        new ModuleConfiguration(
            frontRightDriveMotorId, frontRightSteerMotorId, frontRightEncoderId,
            frontRightEncoderOffsetRotations,
            frontRightDriveInverted, frontRightSteerInverted, frontRightEncoderInverted);

    public static final ModuleConfiguration kRearLeft =
        new ModuleConfiguration(
            rearLeftDriveMotorId, rearLeftSteerMotorId, rearLeftEncoderId,
            rearLeftEncoderOffsetRotations,
            rearLeftDriveInverted, rearLeftSteerInverted, rearLeftEncoderInverted);

    public static final ModuleConfiguration kRearRight =
        new ModuleConfiguration(
            rearRightDriveMotorId, rearRightSteerMotorId, rearRightEncoderId,
            rearRightEncoderOffsetRotations,
            rearRightDriveInverted, rearRightSteerInverted, rearRightEncoderInverted);

    // Nun needed
    private SwerveConstants() {}
  }

  // Settings for one module.
  public record ModuleConfiguration(
      int driveCanId, int turnCanId, int encoderCanId, double encoderOffsetRotations,
      boolean driveInverted, boolean turnInverted, boolean encoderInverted) {
    // Checks ID ranges; Swerve checks duplicates.
    public boolean hasValidIds() {
      return isValidCanId(driveCanId) && isValidCanId(turnCanId) && isValidCanId(encoderCanId);
    }

    // Valid CAN IDs are 0 through 62.
    private static boolean isValidCanId(int canId) {
      return canId >= 0 && canId <= 62;
    }
  }

  // No instances needed.
  private Constants() {}
}
