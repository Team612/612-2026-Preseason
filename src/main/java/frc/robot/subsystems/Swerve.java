package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.ctre.phoenix6.SignalLogger;
import frc.robot.Telemetry;
import java.util.Optional;
import static edu.wpi.first.units.Units.*;
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
  // Estimates pose from wheel travel, yaw, and optional vision measurements.
  private final SwerveDrivePoseEstimator odometry;
  private final Telemetry telemetry = new Telemetry(Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
  private double lastTelemetryTimestamp;
  private boolean characterizationFailed;
  private boolean operatorPerspectiveApplied;
  private Rotation2d operatorPerspective = Rotation2d.kZero;
  // Copy of the requested robot-relative velocity, used for stop checks and telemetry.
  private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
  // Divides the main loop rate by five for dashboard update frequecy
  private int telemetryCycleCount;

  // True after wheel positions have been synced with current odometry ref
  private boolean odometryInitialized;
  // Applies the driver heading reset on the next valid odometry update.
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
    var gyroBusStatus = gyro.optimizeBusUtilization();
    if (!gyroBusStatus.isOK()) { // houston we are NOT OKAY MAYDAY
      DriverStation.reportWarning("Pigeon bus optimization failed: " + gyroBusStatus, false);
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
    odometry = new SwerveDrivePoseEstimator(KINEMATICS, initialHeading, initialPositions, new Pose2d());
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

  /** Robot heading relative to the driver's alliance perspective, or null if unavailable. */
  public Rotation2d getOperatorRelativeHeading() {
    Rotation2d heading = readHeading();
    return heading == null ? null : heading.minus(operatorPerspective);
  }

  /** Driver translation is relative to the alliance wall; autonomous uses field coordinates. */
  public boolean driveOperatorRelative(ChassisSpeeds speeds) {
    Rotation2d heading = getOperatorRelativeHeading();
    if (heading == null || !validSpeeds(speeds)) {
      stop();
      return false;
    }
    return drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading));
  }

  public boolean pointWheelsAt(Rotation2d direction) {
    return pointWheels(new Rotation2d[] {direction, direction, direction, direction});
  }

  /** Points wheels outward along the chassis diagonals to resist translation. */
  public boolean brake() {
    double angle = Math.atan2(Constants.trackWidth, Constants.wheelBase);
    return pointWheels(new Rotation2d[] {
        Rotation2d.fromRadians(angle), Rotation2d.fromRadians(-angle),
        Rotation2d.fromRadians(-angle), Rotation2d.fromRadians(angle)});
  }

  private boolean pointWheels(Rotation2d[] angles) {
    commandedSpeeds = new ChassisSpeeds();
    if (DriverStation.isDisabled() || !isReady()) {
      stop();
      return false;
    }
    for (int i = 0; i < modules.length; i++) {
      if (!modules[i].pointAt(angles[i])) {
        stop();
        return false;
      }
    }
    return true;
  }

  /** FPGA timestamps are already in the WPILib estimator's time base. */
  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    if (odometryInitialized) {
      odometry.addVisionMeasurement(pose, timestampSeconds);
    }
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    if (odometryInitialized) {
      odometry.addVisionMeasurement(pose, timestampSeconds, stdDevs);
    }
  }

  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return odometryInitialized ? odometry.sampleAt(timestampSeconds) : Optional.empty();
  }

  /** Test-mode-only characterization. All modules stop together on any rejected output. */
  public Command sysId(boolean steer, boolean dynamic, SysIdRoutine.Direction direction) {
    String name = steer ? "SwerveSteer" : "SwerveTranslation";
    SysIdRoutineLog stateLog = new SysIdRoutineLog(name);
    double limit = steer ? Constants.SwerveConstants.kMaxTurnVolts
        : Constants.SwerveConstants.kMaxDriveVolts;
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(Math.min(2.0, limit)),
            Seconds.of(Math.min(10.0, limit)), state -> {
              stateLog.recordState(state);
              SignalLogger.writeString(name + "_State", state.toString());
            }),
        new SysIdRoutine.Mechanism(voltage -> characterize(steer, voltage.in(Volts)),
            log -> {
              for (SwerveModule module : modules) {
                module.logCharacterization(log, steer);
              }
            }, this, name));
    Command align = steer ? Commands.none() : run(() -> {
      if (!pointWheelsAt(Rotation2d.kZero)) {
        characterizationFailed = true;
      }
    }).until(this::allWheelsForward).withTimeout(3.0);
    return runOnce(() -> {
      stop();
      characterizationFailed = false;
    }).andThen(align, (dynamic ? routine.dynamic(direction) : routine.quasistatic(direction))
        .onlyIf(() -> !characterizationFailed && (steer || allWheelsForward())))
        .until(() -> characterizationFailed || !DriverStation.isTestEnabled() || !isReady())
        .finallyDo(this::stop)
        .onlyIf(() -> DriverStation.isTestEnabled() && isStopped());
  }

  private boolean allWheelsForward() {
    for (SwerveModule module : modules) {
      if (!module.isPointedForward()) {
        return false;
      }
    }
    return true;
  }

  private void characterize(boolean steer, double volts) {
    if (characterizationFailed || !DriverStation.isTestEnabled() || !isReady()
        || (!steer && !allWheelsForward())) {
      characterizationFailed = true;
      stop();
      return;
    }
    for (SwerveModule module : modules) {
      if (!module.characterize(steer, volts)) {
        characterizationFailed = true;
        stop();
        return;
      }
    }
  }

  private void publishTelemetry() {
    boolean valid = odometryInitialized && isReady();
    SmartDashboard.putBoolean("DriveState/Valid", valid);
    if (!valid) {
      lastTelemetryTimestamp = 0.0;
      return;
    }
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    SwerveModuleState[] targets = new SwerveModuleState[modules.length];
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
      targets[i] = modules[i].getTargetState();
      positions[i] = modules[i].getPosition();
      if (states[i] == null || positions[i] == null) {
        SmartDashboard.putBoolean("DriveState/Valid", false);
        lastTelemetryTimestamp = 0.0;
        return;
      }
    }
    double now = Timer.getFPGATimestamp();
    telemetry.telemeterize(getPose(), KINEMATICS.toChassisSpeeds(states), states, targets,
        positions, now, lastTelemetryTimestamp == 0.0 ? 0.0 : now - lastTelemetryTimestamp);
    lastTelemetryTimestamp = now;
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
   * Makes the current nose direction driver-forward for this alliance, only while stopped.
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
    headingOffset = heading.minus(operatorPerspective);
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
    return odometry.getEstimatedPosition();
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
    publishTelemetry();
    if (!operatorPerspectiveApplied || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(alliance -> {
        operatorPerspective = alliance == DriverStation.Alliance.Red
            ? Rotation2d.k180deg : Rotation2d.kZero;
        operatorPerspectiveApplied = true;
      });
    }
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
      Pose2d pose = odometry.getEstimatedPosition();
      if (headingResetPending) {
        pose = new Pose2d(pose.getTranslation(), operatorPerspective);
        headingResetPending = false;
      }
      odometry.resetPosition(heading, positions, pose);
      odometryInitialized = true;
    } else {
      odometry.updateWithTime(Timer.getFPGATimestamp(), heading, positions);
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
    SmartDashboard.putBoolean("Swerve/BrownedOut", RobotController.isBrownedOut());
    var can = RobotController.getCANStatus();
    SmartDashboard.putNumber("Swerve/CANUtilization", can.percentBusUtilization);
    SmartDashboard.putNumber("Swerve/CANBusOffCount", can.busOffCount);
    SmartDashboard.putNumber("Swerve/CANTxFullCount", can.txFullCount);
    SmartDashboard.putNumber("Swerve/CANReceiveErrorCount", can.receiveErrorCount);
    SmartDashboard.putNumber("Swerve/CANTransmitErrorCount", can.transmitErrorCount);
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
