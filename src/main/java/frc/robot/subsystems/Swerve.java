package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d(Constants.SwerveConstants.kWheelBaseMeters / 2.0, Constants.SwerveConstants.kTrackWidthMeters / 2.0),
          new Translation2d(Constants.SwerveConstants.kWheelBaseMeters / 2.0, -Constants.SwerveConstants.kTrackWidthMeters / 2.0),
          new Translation2d(-Constants.SwerveConstants.kWheelBaseMeters / 2.0, Constants.SwerveConstants.kTrackWidthMeters / 2.0),
          new Translation2d(-Constants.SwerveConstants.kWheelBaseMeters / 2.0, -Constants.SwerveConstants.kTrackWidthMeters / 2.0));
  private final SwerveModule[] modules;
  private final boolean hardwareEnabled;
  private final Pigeon2 gyro =
      new Pigeon2(Constants.SwerveConstants.kPigeonCanId, Constants.SwerveConstants.kCanBus);
  private final SwerveDriveOdometry odometry;
  private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
  private int telemetryCycles;
  private boolean odometryInitialized;
  private boolean previousModulesReady = true;

  public Swerve() {
    Constants.ModuleConfiguration[] configurations = moduleConfigurations();
    hardwareEnabled = hasValidCanIds(configurations);
    if (!hardwareEnabled) {
      DriverStation.reportError("Swerve outputs disabled: CAN IDs are missing or outside the valid Phoenix range.", false);
    }
    modules = new SwerveModule[] {
      new SwerveModule("FrontLeft", configurations[0], hardwareEnabled),
      new SwerveModule("FrontRight", configurations[1], hardwareEnabled),
      new SwerveModule("RearLeft", configurations[2], hardwareEnabled),
      new SwerveModule("RearRight", configurations[3], hardwareEnabled)
    };
    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getPositions());
  }

  public void drive(ChassisSpeeds robotRelativeSpeeds) {
    if (!areModulesReady() || robotRelativeSpeeds == null || !validGeometry()
        || !Double.isFinite(robotRelativeSpeeds.vxMetersPerSecond)
        || !Double.isFinite(robotRelativeSpeeds.vyMetersPerSecond)
        || !Double.isFinite(robotRelativeSpeeds.omegaRadiansPerSecond)) {
      stop();
      reportModuleHealth();
      return;
    }
    if (isZeroCommand(robotRelativeSpeeds)) {
      stop();
      return;
    }
    commandedSpeeds = robotRelativeSpeeds;
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
    for (int index = 0; index < modules.length; index++) {
      modules[index].setState(targetStates[index]);
    }
  }

  /** Drives using field relative translation and the current Pigeon 2 headings */
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    if (fieldRelativeSpeeds == null) {
      stop();
      return;
    }
    drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds.vxMetersPerSecond,
            fieldRelativeSpeeds.vyMetersPerSecond,
            fieldRelativeSpeeds.omegaRadiansPerSecond,
            getHeading()));
  }

  /** Returns the counter clockwise-positive robot heading (Pigeon 2) */
  public Rotation2d getHeading() {
    var yaw = gyro.getYaw().refresh();
    if (!yaw.getStatus().isOK() || !Double.isFinite(yaw.getValueAsDouble())) {
      return new Rotation2d();
    }
    return Rotation2d.fromDegrees(yaw.getValueAsDouble());
  }

  /** Set the current physical robot direction as the zero degree field heading */
  public void resetHeading() {
    gyro.reset();
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int index = 0; index < modules.length; index++) {
      states[index] = modules[index].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int index = 0; index < modules.length; index++) {
      positions[index] = modules[index].getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    if (pose != null && hasValidModulePositions()) {
      odometry.resetPosition(getHeading(), getPositions(), pose);
      odometryInitialized = true;
    }
  }

  public boolean hasValidModulePositions() {
    for (SwerveModule module : modules) {
      if (!module.hasValidPosition()) {
        return false;
      }
    }
    return true;
  }

  private boolean areModulesReady() {
    if (!hardwareEnabled) {
      return false;
    }
    for (SwerveModule module : modules) {
      if (!module.isReady()) {
        return false;
      }
    }
    return true;
  }

  private void reportModuleHealth() {
    boolean modulesReady = areModulesReady();
    if (modulesReady == previousModulesReady) {
      return;
    }
    if (modulesReady) {
      DriverStation.reportWarning("Swerve module health restored.", false);
    } else {
      DriverStation.reportError("Swerve outputs disabled: one or more modules have an invalid sensor or motor configuration status.", false);
    }
    previousModulesReady = modulesReady;
  }

  public void stop() {
    commandedSpeeds = new ChassisSpeeds();
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  private boolean validGeometry() {
    return Constants.SwerveConstants.kWheelBaseMeters > 0.0
        && Constants.SwerveConstants.kTrackWidthMeters > 0.0
        && Constants.SwerveConstants.kMaxSpeedMetersPerSecond > 0.0
        && Double.isFinite(Constants.SwerveConstants.kWheelBaseMeters)
        && Double.isFinite(Constants.SwerveConstants.kTrackWidthMeters)
        && Double.isFinite(Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
  }

  @Override
  public void periodic() {
    if (hasValidModulePositions()) {
      if (!odometryInitialized) {
        odometry.resetPosition(getHeading(), getPositions(), odometry.getPoseMeters());
        odometryInitialized = true;
      } else {
        odometry.update(getHeading(), getPositions());
      }
    }
    reportModuleHealth();
    telemetryCycles++;
    if (telemetryCycles % 5 != 0) {
      return;
    }
    SmartDashboard.putNumber("Swerve/CommandedVxMetersPerSecond", commandedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve/CommandedVyMetersPerSecond", commandedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve/CommandedOmegaRadiansPerSecond", commandedSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Swerve/BatteryVoltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Swerve/HardwareEnabled", hardwareEnabled);
    SmartDashboard.putBoolean("Swerve/PigeonConnected", gyro.getYaw().refresh().getStatus().isOK());
    SmartDashboard.putBoolean("Swerve/ModulesReady", areModulesReady());
    SmartDashboard.putNumber("Swerve/PoseXMeters", getPose().getX());
    SmartDashboard.putNumber("Swerve/PoseYMeters", getPose().getY());
    SmartDashboard.putNumber("Swerve/PoseHeadingDegrees", getPose().getRotation().getDegrees());
    for (SwerveModule module : modules) {
      module.publishTelemetry();
    }
  }

  static boolean hasValidCanIds(Constants.ModuleConfiguration[] configurations) {
    if (configurations == null || configurations.length != 4) {
      return false;
    }
    for (Constants.ModuleConfiguration configuration : configurations) {
      if (configuration == null || !configuration.hasConfiguredCanIds()) {
        return false;
      }
    }
    return true;
  }

  static boolean isZeroCommand(ChassisSpeeds speeds) {
    return speeds.vxMetersPerSecond == 0.0
        && speeds.vyMetersPerSecond == 0.0
        && speeds.omegaRadiansPerSecond == 0.0;
  }

  private static Constants.ModuleConfiguration[] moduleConfigurations() {
    return new Constants.ModuleConfiguration[] {
      Constants.SwerveConstants.kFrontLeft,
      Constants.SwerveConstants.kFrontRight,
      Constants.SwerveConstants.kRearLeft,
      Constants.SwerveConstants.kRearRight
    };
  }
}
