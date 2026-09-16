package frc.robot;

public final class Constants {
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.05;

    private OperatorConstants() {}
  }

  public static final class SwerveConstants {
    public static final String kCanBus = "rio"; // i think we havent switched to systemcore yet
    public static final int kPigeonCanId = 0;
    public static final double kBatteryNominalVoltage = 12.0;
    public static final double kBatteryFullVoltage = 12.6;
    public static final double kWheelBaseMeters = 0.555;
    public static final double kTrackWidthMeters = 0.550;
    public static final double kWheelDiameterMeters = 0.1016;
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
    public static final double kDriveSupplyLimitAmps = 30.0; // assumptions
    public static final double kDriveStatorLimitAmps = 60.0;
    public static final double kTurnSupplyLimitAmps = 15.0;
    public static final double kTurnStatorLimitAmps = 25.0;
    public static final double kTranslationSlewMetersPerSecondSquared = 3.0;
    public static final double kRotationSlewRadPerSecSquared = Math.PI * 2.0;
    public static final double kLowSpeedThresholdMetersPerSecond = 0.05;

    public static final ModuleConfiguration kFrontLeft = new ModuleConfiguration(4, 5, 3, 0.0, false, false);
    public static final ModuleConfiguration kFrontRight = new ModuleConfiguration(2, 3, 2, 0.0, false, false);
    public static final ModuleConfiguration kRearLeft = new ModuleConfiguration(6, 7, 4, 0.0, false, false);
    public static final ModuleConfiguration kRearRight = new ModuleConfiguration(8, 1, 1, 0.0, false, false);

    private SwerveConstants() {}
  }

  public record ModuleConfiguration(
      int driveCanId,
      int turnCanId,
      int encoderCanId,
      double encoderOffsetRotations,
      boolean driveInverted,
      boolean turnInverted) {
    public boolean hasConfiguredCanIds() {
      return driveCanId >= 0 && turnCanId >= 0 && encoderCanId >= 0;
    }
  }

  private Constants() {}
}
