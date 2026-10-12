package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import java.util.HashSet;
import java.util.Set;

public class Swerve extends SubsystemBase {
  private static final SwerveDriveKinematics KINEMATICS = Constants.swerveKinematics;
  private final SwerveModule[] modules;
  private final boolean hardwareEnabled;
  private final Pigeon2 gyro =
      new Pigeon2(Constants.SwerveConstants.kPigeonCanId, Constants.SwerveConstants.kCanBus);
  private final SwerveDriveOdometry odometry;
  private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
  private int telemetryCycleCount;

  private boolean odometryInitialized;
  private boolean headingResetPending;
  private boolean previousModulesReady = true;
  private Rotation2d headingOffset = new Rotation2d();

  public Swerve() {
    var yawStatus = gyro.getYaw().setUpdateFrequency(
        Constants.SwerveConstants.kSensorUpdateFrequencyHz);
    if (!yawStatus.isOK()) {
      DriverStation.reportWarning("Pigeon yaw update configuration failed: " + yawStatus, false);
    }
    Constants.ModuleConfiguration[] configurations = {
      Constants.SwerveConstants.kFrontLeft,
      Constants.SwerveConstants.kFrontRight,
      Constants.SwerveConstants.kRearLeft,
      Constants.SwerveConstants.kRearRight
    };
    boolean validIds = true;
    Set<Integer> motorIds = new HashSet<>();
    Set<Integer> encoderIds = new HashSet<>();
    for (Constants.ModuleConfiguration configuration : configurations) {
      if (configuration == null || !configuration.hasValidIds()
          || !motorIds.add(configuration.driveCanId())
          || !motorIds.add(configuration.turnCanId())
          || !encoderIds.add(configuration.encoderCanId())) {
        validIds = false;
      }
    }
    hardwareEnabled = validIds;
    if (!hardwareEnabled) {
      DriverStation.reportError(
          "Swerve outputs disabled: CAN IDs must be 0 to 62, with unique Talon FX IDs and unique CANcoder IDs.",
          false);
    }
    modules = new SwerveModule[] {
      new SwerveModule("FrontLeft", configurations[0], hardwareEnabled),
      new SwerveModule("FrontRight", configurations[1], hardwareEnabled),
      new SwerveModule("RearLeft", configurations[2], hardwareEnabled),
      new SwerveModule("RearRight", configurations[3], hardwareEnabled)
    };
    SwerveModulePosition[] initialPositions = new SwerveModulePosition[modules.length];
    for (int index = 0; index < modules.length; index++) {
      initialPositions[index] = new SwerveModulePosition();
    }
    Rotation2d heading = readHeading();
    odometry = new SwerveDriveOdometry(
        KINEMATICS, heading != null ? heading : new Rotation2d(), initialPositions);
  }

  public void drive(ChassisSpeeds robotRelativeSpeeds) {
    if (DriverStation.isDisabled() || !isReady()
        || !validGeometry() || !validSpeeds(robotRelativeSpeeds)) {
      stop();
      reportStatus();
      return;
    }
    commandedSpeeds = robotRelativeSpeeds;
    SwerveModuleState[] targetStates = getWheelStates(robotRelativeSpeeds);
    for (int index = 0; index < modules.length; index++) {
      modules[index].setState(targetStates[index]);
    }
  }

  private static SwerveModuleState[] getWheelStates(ChassisSpeeds speeds) {
    double low = 0.0;
    double high = 1.0;
    double scale = 1.0;
    SwerveModuleState[] best = KINEMATICS.toSwerveModuleStates(new ChassisSpeeds());
    for (int attempt = 0; attempt < 13; attempt++) {
      ChassisSpeeds limitedSpeeds = speeds.times(scale);
      SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.discretize(limitedSpeeds, Constants.kLoopPeriodSeconds));
      double fastest = 0.0;
      for (SwerveModuleState state : states) {
        fastest = Math.max(fastest, Math.abs(state.speedMetersPerSecond));
      }
      if (fastest <= Constants.SwerveConstants.kMaxSpeedMetersPerSecond) {
        best = states;
        low = scale;
        if (scale == 1.0) {
          break;
        }
      } else {
        high = scale;
      }
      scale = (low + high) / 2.0;
    }
    return best;
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    Rotation2d heading = readHeading();
    if (heading == null || !validSpeeds(fieldRelativeSpeeds)) {
      stop();
      return;
    }
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, heading));
  }

  private Rotation2d readHeading() {
    var yaw = gyro.getYaw().refresh();
    double yawDegrees = yaw.getValueAsDouble();
    if (!yaw.getStatus().isOK()
        || !(yaw.getTimestamp().getLatency() <= Constants.SwerveConstants.kMaxSensorAgeSeconds)
        || !Double.isFinite(yawDegrees)) {
      return null;
    }
    return Rotation2d.fromDegrees(yawDegrees).minus(headingOffset);
  }

  public boolean hasHeading() {
    return readHeading() != null;
  }

  public void resetHeading() {
    if (!isStopped()) {
      DriverStation.reportWarning("Stop before resetting heading.", false);
      return;
    }
    Rotation2d heading = readHeading();
    if (heading == null) {
      DriverStation.reportWarning("Heading reset ignored: gyro reading unavailable.", false);
      return;
    }
    headingOffset = headingOffset.plus(heading);
    headingResetPending = true;
    odometryInitialized = false;
  }

  public boolean isStopped() {
    if (!isReady() || commandedSpeeds.vxMetersPerSecond != 0.0
        || commandedSpeeds.vyMetersPerSecond != 0.0
        || commandedSpeeds.omegaRadiansPerSecond != 0.0) {
      return false;
    }
    for (SwerveModule module : modules) {
      double speed = module.getSpeed();
      if (!Double.isFinite(speed)
          || Math.abs(speed) > Constants.SwerveConstants.kStoppedSpeedMetersPerSecond) {
        return false;
      }
    }
    return true;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public boolean isReady() {
    if (!hardwareEnabled) {
      return false;
    }
    for (SwerveModule module : modules) { // enhanced for loop to iterate throughout modules
      if (!module.isReady()) {
        return false;
      }
    }
    return true;
  }

  private void reportStatus() {
    boolean modulesReady = isReady();
    if (modulesReady == previousModulesReady) {
      return;
    }
    // more logs
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
    return isPositiveFinite(Constants.SwerveConstants.kWheelBaseMeters)
        && isPositiveFinite(Constants.SwerveConstants.kTrackWidthMeters)
        && isPositiveFinite(Constants.kLoopPeriodSeconds)
        && isPositiveFinite(Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
  }

  private static boolean validSpeeds(ChassisSpeeds speeds) {
    return speeds != null
        && Double.isFinite(speeds.vxMetersPerSecond)
        && Double.isFinite(speeds.vyMetersPerSecond)
        && Double.isFinite(speeds.omegaRadiansPerSecond);
  }

  private static boolean isPositiveFinite(double value) {
    return value > 0.0 && Double.isFinite(value);
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.periodic();
    }
    if (DriverStation.isDisabled() || !isReady()) {
      stop();
    }
    updateOdometry();
    reportStatus();
    updateDashboard();
  }

  private void updateOdometry() {
    Rotation2d heading = readHeading();
    if (heading == null || !isReady()) {
      odometryInitialized = false;
      return;
    }
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int index = 0; index < modules.length; index++) {
      positions[index] = modules[index].getPosition();
      if (positions[index] == null) {
        odometryInitialized = false;
        return;
      }
    }
    if (!odometryInitialized) {
      Pose2d pose = odometry.getPoseMeters();
      if (headingResetPending) {
        pose = new Pose2d(pose.getTranslation(), new Rotation2d());
        headingResetPending = false;
      }
      odometry.resetPosition(heading, positions, pose);
      odometryInitialized = true;
    } else {
      odometry.update(heading, positions);
    }
  }

  private void updateDashboard() {
    telemetryCycleCount++;
    if (telemetryCycleCount % 5 != 0) {
      return;
    }
    SmartDashboard.putNumber("Swerve/CommandedVxMetersPerSecond", commandedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve/CommandedVyMetersPerSecond", commandedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve/CommandedOmegaRadiansPerSecond", commandedSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Swerve/BatteryVoltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Swerve/HardwareEnabled", hardwareEnabled);
    SmartDashboard.putBoolean("Swerve/PigeonConnected", hasHeading());
    SmartDashboard.putBoolean("Swerve/ModulesReady", isReady());
    SmartDashboard.putNumber("Swerve/PoseXMeters", getPose().getX());
    SmartDashboard.putNumber("Swerve/PoseYMeters", getPose().getY());
    SmartDashboard.putNumber("Swerve/PoseHeadingDegrees", getPose().getRotation().getDegrees());
    for (SwerveModule module : modules) {
      module.updateDashboard();
    }
  }
}
