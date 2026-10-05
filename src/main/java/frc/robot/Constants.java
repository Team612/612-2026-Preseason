package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final double trackWidth = 0.550;
  public static final double wheelBase = 0.555;
  // Fractions of the configured drivetrain limits used for driver controley
  public static final double xPercent = 0.2;
  public static final double yPercent = 0.2;
  public static final double zPercent = 0.2;
  public static final double DEADBAND = 0.05;
  public static final int kDriverControllerPort = 0;

  public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

  public static final double frontLEncoderOffset = 0.277;
  public static final int frontLSteerMotorID = 4;
  public static final int frontLDriveMotorID = 3;
  public static final int frontLCANcoderID = 3;
  public static final double frontREncoderOffset = 0.398;
  public static final int frontRSteerMotorID = 6;
  public static final int frontRDriveMotorID = 5;
  public static final int frontRCANcoderID = 2;
  public static final double backLEncoderOffset = -0.260;
  
  public static final int backLSteerMotorID = 1;
  public static final int backLDriveMotorID = 2;
  public static final int backLCANcoderID = 4;
  public static final double backREncoderOffset = 0.022;
  public static final int backRSteerMotorID = 8;
  public static final int backRDriveMotorID = 7;
  public static final int backRCANcoderID = 1;

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = Constants.kDriverControllerPort;
    public static final double kDeadband = DEADBAND;

    private OperatorConstants() {}
  }

  public static final class SwerveConstants {
    public static final CANBus kCanBus = CANBus.roboRIO();
    public static final int kPigeonCanId = 0;
    public static final double kBatteryNominalVoltage = 12.0;
    public static final double kWheelBaseMeters = wheelBase;
    public static final double kTrackWidthMeters = trackWidth;
    public static final double kWheelDiameterMeters = 0.0889;
    public static final double kDriveReduction = 6.75;
    public static final double kMaxSpeedMetersPerSecond = 2.0;
    public static final double kMaxAngularSpeedRadPerSec = Math.PI * 2.0;
    public static final double kMaxDriveVolts = kBatteryNominalVoltage;
    public static final double kDriveP = 1.0;
    public static final double kMaxPVolts = 0.5;
    public static final boolean kEnableDriveVelocityCorrection = true;
    public static final double kTurnP = 0.5;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.0;
    public static final double kMaxTurnRadPerSec = Math.PI * 2.0;
    public static final double kMaxTurnAccelRadPerSecSquared = Math.PI * 4.0;
    public static final double kMaxTurnVolts = 4.0;
    public static final double kDriveSupplyLimitAmps = 30.0;
    public static final double kDriveStatorLimitAmps = 60.0;
    public static final double kTurnSupplyLimitAmps = 15.0;
    public static final double kTurnStatorLimitAmps = 25.0;
    public static final double kLowSpeedThresholdMetersPerSecond = 0.05;

    public static final ModuleConfiguration kFrontLeft =
        new ModuleConfiguration(frontLDriveMotorID, frontLSteerMotorID, frontLCANcoderID, frontLEncoderOffset, false, false);
    public static final ModuleConfiguration kFrontRight =
        new ModuleConfiguration(frontRDriveMotorID, frontRSteerMotorID, frontRCANcoderID, frontREncoderOffset, false, false);
    public static final ModuleConfiguration kRearLeft =
        new ModuleConfiguration(backLDriveMotorID, backLSteerMotorID, backLCANcoderID, backLEncoderOffset, false, false);
    public static final ModuleConfiguration kRearRight =
        new ModuleConfiguration(backRDriveMotorID, backRSteerMotorID, backRCANcoderID, backREncoderOffset, false, false);

    private SwerveConstants() {}
  }

  public record ModuleConfiguration(
      int driveCanId, int turnCanId, int encoderCanId, double encoderOffsetRotations,
      boolean driveInverted, boolean turnInverted) {
    public boolean hasConfiguredCanIds() {
      return isValidCanId(driveCanId) && isValidCanId(turnCanId) && isValidCanId(encoderCanId);
    }

    private static boolean isValidCanId(int canId) {
      return canId >= 0 && canId <= 62;
    }
  }

  private Constants() {}
}
