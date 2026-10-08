package frc.robot.subsystems;

//imports
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

/** Coordinates the four modules, gyro, drive kinematics, and robot-pose estimate.
 * drives car
 */
public class Swerve extends SubsystemBase {
  private static final SwerveDriveKinematics KINEMATICS = createKinematics();
  private final SwerveModule[] modules;
  private final boolean hardwareEnabled;
  private final Pigeon2 gyro =
      new Pigeon2(Constants.SwerveConstants.kPigeonCanId, Constants.SwerveConstants.kCanBus);
  private final SwerveDriveOdometry odometry;
  private SwerveModulePosition[] lastOdometryPositions;
  private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
  private int telemetryCycleCount;
  /**Odometry waits for valid sensors, then rebases instead of integrating a bad position jump.
   *Make sure robot dont do big jump
   * */
  private boolean odometryInitialized;
  private boolean odometryNeedsRebase;
  // Applied after resetHeading() so the estimated pose keeps its location but resets rotation.
  private boolean headingResetPending;
  private boolean previousModulesReady = true;

  public Swerve() {
    Constants.ModuleConfiguration[] configurations = moduleConfigurations();
    hardwareEnabled = haveValidCanRanges(configurations);
    if (!hardwareEnabled) {
      DriverStation.reportError(
          "Swerve outputs disabled: module CAN IDs must be configured in the Phoenix range (0 to 62).",
          false);
    }
    modules = new SwerveModule[] {
      new SwerveModule("FrontLeft", configurations[0], hardwareEnabled),
      new SwerveModule("FrontRight", configurations[1], hardwareEnabled),
      new SwerveModule("RearLeft", configurations[2], hardwareEnabled),
      new SwerveModule("RearRight", configurations[3], hardwareEnabled)
    };
    lastOdometryPositions = getPositions();
    odometry = new SwerveDriveOdometry(KINEMATICS, getHeading(), getPositions());
  }

  /** Converts chassis motion into front left, front right, rear left, rear right wheel states. */
  static SwerveModuleState[] calculateModuleStates(ChassisSpeeds speeds) {
    return KINEMATICS.toSwerveModuleStates(speeds);
  }

  /** Module order here must match the order used by the modules array below. this was messed up*/
  private static SwerveDriveKinematics createKinematics() {
    double halfWheelBase = Constants.SwerveConstants.kWheelBaseMeters / 2.0;
    double halfTrackWidth = Constants.SwerveConstants.kTrackWidthMeters / 2.0;
    return new SwerveDriveKinematics(
        new Translation2d(halfWheelBase, halfTrackWidth),
        new Translation2d(halfWheelBase, -halfTrackWidth),
        new Translation2d(-halfWheelBase, halfTrackWidth),
        new Translation2d(-halfWheelBase, -halfTrackWidth));
  }


  public void drive(ChassisSpeeds robotRelativeSpeeds) {
    if (!areModulesReady() || !validGeometry() || !hasFiniteSpeeds(robotRelativeSpeeds)) {
      stop();
      reportModuleHealth();
      return;
    }
    if (isZeroCommand(robotRelativeSpeeds)) {
      stop();
      return;
    }
    commandedSpeeds = robotRelativeSpeeds;
    SwerveModuleState[] targetStates = calculateModuleStates(robotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        targetStates, Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
    for (int index = 0; index < modules.length; index++) {
      modules[index].setState(targetStates[index]);
    }
  }

  /** Converts field-relative speeds using the gyro, then sends them to the modules. */
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    ChassisSpeeds robotRelativeSpeeds = toRobotRelativeSpeeds(fieldRelativeSpeeds, getValidHeading());
    if (robotRelativeSpeeds == null) {
      stop();
      return; // do da retuirning yes i used ai to generate comments because nobody spends time makin em
    }
    drive(robotRelativeSpeeds);
  }

  /** Returns the counter-clockwise-positive gyro heading; returns zero if the gyro is unavailable. */
  public Rotation2d getHeading() {
    Rotation2d heading = getValidHeading();
    if (heading == null) {
      return new Rotation2d();
    }
    return heading;
  }

  private Rotation2d getValidHeading() {
    var yaw = gyro.getYaw().refresh();
    return headingFromSignal(yaw.getStatus().isOK(), yaw.getValueAsDouble());
  }

  static Rotation2d headingFromSignal(boolean statusOk, double yawDegrees) {
    if (!statusOk || !Double.isFinite(yawDegrees)) {
      return null;
    }
    return Rotation2d.fromDegrees(yawDegrees);
  }

  static ChassisSpeeds toRobotRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds, Rotation2d heading) {
    if (fieldRelativeSpeeds == null || heading == null) {
      return null;
    }
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond,
        fieldRelativeSpeeds.omegaRadiansPerSecond,
        heading);
  }

  /** Sets the robot's current direction as zero degrees for field-relative driving. */
  public void resetHeading() {
    gyro.reset();
    headingResetPending = true;
    odometryInitialized = false;
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

  /** Resets odometry only when the requested pose and sensor readings are valid. */
  public void resetPose(Pose2d pose) {
    Rotation2d heading = getValidHeading();
    if (pose != null && heading != null && hasValidModulePositions()) {
      odometry.resetPosition(heading, getPositions(), pose);
      lastOdometryPositions = getPositions();
      odometryInitialized = true;
      odometryNeedsRebase = false;
      headingResetPending = false;
    }
  }

  public boolean hasValidModulePositions() {
    for (SwerveModule module : modules) {
      if (module.getPositionIfValid() == null) {
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
    return isPositiveFinite(Constants.SwerveConstants.kWheelBaseMeters)
        && isPositiveFinite(Constants.SwerveConstants.kTrackWidthMeters)
        && isPositiveFinite(Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
  }

  private static boolean hasFiniteSpeeds(ChassisSpeeds speeds) {
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
    updateOdometry();
    reportModuleHealth();
    publishTelemetry();
  }

  /** Updates pose from the gyro and encoder deltas, holding bad samples until sensors recover. */
  private void updateOdometry() {
    Rotation2d heading = getValidHeading();
    if (heading == null) {
      odometryInitialized = false;
    } else {
      SwerveModulePosition[] currentPositions = new SwerveModulePosition[modules.length];
      boolean allPositionsValid = true;
      for (int index = 0; index < modules.length; index++) {
        currentPositions[index] = modules[index].getPositionIfValid();
        if (currentPositions[index] == null) {
          allPositionsValid = false;
        }
      }
      SwerveModulePosition[] positions =
          holdLastValidPositions(currentPositions, lastOdometryPositions);

      if (!odometryInitialized) {
        if (allPositionsValid) {
          Pose2d pose = odometry.getPoseMeters();
          if (headingResetPending) {
            pose = new Pose2d(pose.getTranslation(), new Rotation2d());
            headingResetPending = false;
          }
          odometry.resetPosition(heading, positions, pose);
          lastOdometryPositions = positions;
          odometryInitialized = true;
          odometryNeedsRebase = false;
        }
      } else if (allPositionsValid && odometryNeedsRebase) {
        odometry.resetPosition(heading, positions, odometry.getPoseMeters());
        lastOdometryPositions = positions;
        odometryNeedsRebase = false;
      } else {
        odometry.update(heading, positions);
        lastOdometryPositions = positions;
        if (!allPositionsValid) {
          odometryNeedsRebase = true;
        }
      }
    }
  }

  // delay the cycles
  private void publishTelemetry() {
    telemetryCycleCount++;
    if (telemetryCycleCount % 5 != 0) {
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

  static boolean haveValidCanRanges(Constants.ModuleConfiguration[] configurations) {
    if (configurations == null || configurations.length != 4) {
      return false;
    }
    if (!isValidCanId(Constants.SwerveConstants.kPigeonCanId)) {
      return false;
    }
    for (Constants.ModuleConfiguration configuration : configurations) {
      if (configuration == null || !configuration.hasConfiguredCanIds()) {
        return false;
      }
    }
    return true;
  }

  private static boolean isValidCanId(int canId) {
    return canId >= 0 && canId <= 62;
  }

  static boolean isZeroCommand(ChassisSpeeds speeds) {
    return speeds.vxMetersPerSecond == 0.0
        && speeds.vyMetersPerSecond == 0.0
        && speeds.omegaRadiansPerSecond == 0.0;
  }

  /** Reuses the last trusted encoder position when a current sensor sample is invalid. muy importante for the jumps */
  static SwerveModulePosition[] holdLastValidPositions(
      SwerveModulePosition[] currentPositions, SwerveModulePosition[] lastPositions) {
    SwerveModulePosition[] positions = new SwerveModulePosition[currentPositions.length];
    for (int index = 0; index < currentPositions.length; index++) {
      if (currentPositions[index] == null) {
        positions[index] = lastPositions[index];
      } else {
        positions[index] = currentPositions[index];
      }
    }
    return positions;
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
