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

// Owns the four modules, gyro, kinematics, and measured robot position
public class Swerve extends SubsystemBase {
  // array order is front left, front right, rear left, rear right pls dont change
  private static final SwerveDriveKinematics KINEMATICS = Constants.swerveKinematics;
  // saem array order for kinematics and odometrty
  private final SwerveModule[] modules;
  // False when CAN IDs are invalid or duplicated (sanity)
  private final boolean hardwareEnabled;
  // Pigeon yaw for field-relative odometry
  private final Pigeon2 gyro =
      new Pigeon2(Constants.SwerveConstants.kPigeonCanId, Constants.SwerveConstants.kCanBus);
  // Integrates measured wheel travel and yaw doesnt fix slipping wheels
  private final SwerveDriveOdometry odometry;
  // Copy of the requested robot-relative velocity, used for stop checks and telemetry.
  private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
  // Divides the main loop rate by five for dashboard update frequecy
  private int telemetryCycleCount;

  // True after wheel positions have been synced with current odometry ref
  private boolean odometryInitialized;
  // Requests a zero pose heading on the next valid odom update (non valid updates exist because of lying bastard sensors)
  private boolean headingResetPending;
  // Previous status
  private boolean previousModulesReady = true;
  
  
  private Rotation2d headingOffset = Rotation2d.kZero;
  
  
  private boolean headingResetRequired;

  // validate can ids and create modules in array order from before
  public Swerve() {
    gyro.hasResetOccurred();
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
    Set<Integer> motorIds = new HashSet<>(); // should be the right data struct fro this
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
    if (!hardwareEnabled) { // if the hardware aint enabled, complain mucho
      DriverStation.reportError(
          // NEED UNIQUE IDS
          "Swerve outputs disabled: CAN IDs must be 0 to 62, with unique Talon FX IDs and unique CANcoder IDs.",
          false);
    }
    modules = new SwerveModule[] {
      new SwerveModule("FrontLeft", configurations[0], hardwareEnabled),
      new SwerveModule("FrontRight", configurations[1], hardwareEnabled),
      new SwerveModule("RearLeft", configurations[2], hardwareEnabled),
      new SwerveModule("RearRight", configurations[3], hardwareEnabled)
    };
    // initialize odometry with zero pose and zero module positions
    SwerveModulePosition[] initialPositions = new SwerveModulePosition[modules.length];
    for (int index = 0; index < modules.length; index++) {
      initialPositions[index] = new SwerveModulePosition();
    }

    Rotation2d heading = readHeading();
    Rotation2d initialHeading;
    if (heading != null) {
      initialHeading = heading;
    } else {
      initialHeading = new Rotation2d();
    }
    odometry = new SwerveDriveOdometry(KINEMATICS, initialHeading, initialPositions);
  }

  
   //Accepts robot-relative velocities; returns false and stops all modules if a request cannot fails 
  public boolean drive(ChassisSpeeds robotRelativeSpeeds) {
    if (DriverStation.isDisabled() || !isReady()
        || !validGeometry() || !validSpeeds(robotRelativeSpeeds)) {
      stop();
      reportStatus();
      return false;
    }
    // Keep a copy for stop detection and telemetry (FROM DOCS)
    commandedSpeeds = new ChassisSpeeds(robotRelativeSpeeds.vxMetersPerSecond,
        robotRelativeSpeeds.vyMetersPerSecond, robotRelativeSpeeds.omegaRadiansPerSecond);
    SwerveModuleState[] targetStates = getWheelStates(robotRelativeSpeeds);
    for (int index = 0; index < modules.length; index++) {
      if (!modules[index].setState(targetStates[index])) {
        // If any module rejects a request, stop the other modules in this same loop.
        stop();
        reportStatus();
        return false;
      }
    }
    return true;
  }
  /**
   * wheel speed limiting SHOULD BE preserving the intended
   * motion direction.
   */
  private static SwerveModuleState[] getWheelStates(ChassisSpeeds speeds) {
    // Binary search for the largest fraction of the request all four wheels can get
    // low is known to fit; high is the current upper bound (finally something i studied in csa)
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

  /**
   * Rotates field translation into robot coordinates using the trusted heading, then calls
   * drive(). // this is an ai generated comment
   */
  public boolean driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    Rotation2d heading = readHeading();
    if (heading == null || !validSpeeds(fieldRelativeSpeeds)) {
      stop();
      return false;
    }
    return drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, heading));
  }

  /**
   * Returns yaw relative to the driver-defined zero, or null if smth wrong
   */
  private Rotation2d readHeading() {
    Rotation2d rawHeading = readRawHeading();
    if (headingResetRequired || rawHeading == null) {
      return null;
    }
    return rawHeading.minus(headingOffset);
  }

  /** Checks gyro restart n status before returning its unadjusted yaw. */
  private Rotation2d readRawHeading() {
    if (gyro.hasResetOccurred()) {
      headingResetRequired = true;
      odometryInitialized = false;
      DriverStation.reportWarning(
          "Pigeon restarted: stop and reset heading before using field-relative drive.", false);
    }
    var yaw = gyro.getYaw();
    double yawDegrees = yaw.getValueAsDouble();
    if (!yaw.getStatus().isOK()
        || !(yaw.getTimestamp().getLatency() <= Constants.SwerveConstants.kMaxSensorAgeSeconds)
        || !Double.isFinite(yawDegrees)) {
      return null;
    }
    return Rotation2d.fromDegrees(yawDegrees);
  }

  /** Reports whether field-relative driving currently has a usable heading  */
  public boolean hasHeading() {
    return readHeading() != null;
  }

  /**
   * Makes the current nose direction field forward, only with valid stationary modules and zero
   * commands. // ai generated comment from docs also some more complaining to driver station
   */
  public void resetHeading() {
    if (!isStopped()) {
      DriverStation.reportWarning("Stop before resetting heading.", false);
      return;
    }
    Rotation2d heading = readRawHeading();
    if (heading == null) {
      DriverStation.reportWarning("Heading reset ignored: gyro reading unavailable.", false);
      return;
    }
    headingOffset = heading;
    headingResetRequired = false;
    headingResetPending = true;
    odometryInitialized = false;
  }


  //Requires zero requested chassis motion and low measured wheel speeds before changing the ref
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

  //Returns the last odometry pose
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // Requires valid CAN configuration to be right
  public boolean isReady() {
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

  /** Reports module status once instead of repeating the same warning every loop. */ // annoyed me earlier
  private void reportStatus() {
    boolean modulesReady = isReady();
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
  /**
   * Clears the stored chassis command and requests zero drive and steering voltage on every
   * module.
   */
  public void stop() {
    commandedSpeeds = new ChassisSpeeds();
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  // Rejects nonpositive or nonfinite geometry and loop period
  private boolean validGeometry() {
    return isPositiveFinite(Constants.SwerveConstants.kWheelBaseMeters)
        && isPositiveFinite(Constants.SwerveConstants.kTrackWidthMeters)
        && isPositiveFinite(Constants.kLoopPeriodSeconds)
        && isPositiveFinite(Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
  }

  // reject the bad shit
  private static boolean validSpeeds(ChassisSpeeds speeds) {
    return speeds != null
        && Double.isFinite(speeds.vxMetersPerSecond)
        && Double.isFinite(speeds.vyMetersPerSecond)
        && Double.isFinite(speeds.omegaRadiansPerSecond);
  }

  /** Checks constants that must be positive and real numbers */
  private static boolean isPositiveFinite(double value) {
    return value > 0.0 && Double.isFinite(value);
  }

  /**
   * Refreshes module status
   */
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

  /**
   * Integrates measured wheel positions
   */
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
    // if the odometry has not been initialized, reset it with the current heading and module
    // positions
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
  /** Publishes drivetrain and module telemetry every fifth loop to limit dashboard annoyances */
  private void updateDashboard() {
    telemetryCycleCount++;
    if (telemetryCycleCount % 5 != 0) { // cool stuff yall learned in csa
      return;
    }
    SmartDashboard.putNumber("Swerve/CommandedVxMetersPerSecond", commandedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve/CommandedVyMetersPerSecond", commandedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve/CommandedOmegaRadiansPerSecond", commandedSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Swerve/BatteryVoltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Swerve/HardwareEnabled", hardwareEnabled);
    Rotation2d fieldHeading = readHeading();
    SmartDashboard.putBoolean("Swerve/PigeonConnected", fieldHeading != null);
    double fieldHeadingDegrees;
    if (fieldHeading != null) {
      fieldHeadingDegrees = fieldHeading.getDegrees();
    } else {
      fieldHeadingDegrees = Double.NaN;
    }
    SmartDashboard.putNumber("Swerve/FieldHeadingDegrees", fieldHeadingDegrees);
    SmartDashboard.putBoolean("Swerve/HeadingResetRequired", headingResetRequired);
    SmartDashboard.putBoolean("Swerve/OdometryUpdating", odometryInitialized);
    SmartDashboard.putBoolean("Swerve/ModulesReady", isReady());
    SmartDashboard.putNumber("Swerve/PoseXMeters", getPose().getX());
    SmartDashboard.putNumber("Swerve/PoseYMeters", getPose().getY());
    SmartDashboard.putNumber("Swerve/PoseHeadingDegrees", getPose().getRotation().getDegrees());
    for (SwerveModule module : modules) {
      module.updateDashboard();
    }
  }
}
