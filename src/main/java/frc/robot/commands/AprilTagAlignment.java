package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Aligns the robot relative to a specific AprilTag using robot-relative 3D transforms.
 * Accounts for the camera's physical mounting offset on the robot so alignment targets
 * the robot center (or scoring mechanism) rather than the camera lens.
 */
public class AprilTagAlignment extends Command {
  private final Swerve swerve;
  private final Vision vision;
  private final int targetTagId;
  private final double targetXDistanceMeters;
  private final double targetYLateralOffsetMeters;

  private final PIDController xController = new PIDController(1.8, 0.0, 0.05);
  private final PIDController yController = new PIDController(1.8, 0.0, 0.05);
  private final PIDController thetaController = new PIDController(3.5, 0.0, 0.10);

  private static final double X_TOLERANCE_METERS = 0.03;
  private static final double Y_TOLERANCE_METERS = 0.03;
  private static final double THETA_TOLERANCE_RADIANS = Units.degreesToRadians(1.5);

  private static final double MAX_LINEAR_SPEED = 1.75;
  private static final double MAX_ANGULAR_SPEED = Math.PI;
  private static final double LOST_TARGET_TIMEOUT_SECONDS = 0.35;

  private double lastSeenTimestamp = 0.0;

  /**
   * Aligns directly centered in front of the specified tag at default standoff distance.
   *
   * @param swerve Swerve subsystem
   * @param vision Vision subsystem
   * @param targetTagId AprilTag ID to align to, or -1 for any best visible tag
   */
  public AprilTagAlignment(Swerve swerve, Vision vision, int targetTagId) {
    this(
        swerve,
        vision,
        targetTagId,
        VisionConstants.kAlignTargetXDistanceMeters,
        0.0);
  }

  /**
   * Aligns to a tag with custom forward standoff and lateral offset (e.g., left/right scoring pole).
   *
   * @param swerve Swerve subsystem
   * @param vision Vision subsystem
   * @param targetTagId AprilTag ID to align to, or -1 for any best visible tag
   * @param targetXDistanceMeters Desired forward distance from tag to robot center (meters)
   * @param targetYLateralOffsetMeters Desired lateral offset from tag center (+ left, - right)
   */
  public AprilTagAlignment(
      Swerve swerve,
      Vision vision,
      int targetTagId,
      double targetXDistanceMeters,
      double targetYLateralOffsetMeters) {
    this.swerve = swerve;
    this.vision = vision;
    this.targetTagId = targetTagId;
    this.targetXDistanceMeters = targetXDistanceMeters;
    this.targetYLateralOffsetMeters = targetYLateralOffsetMeters;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(X_TOLERANCE_METERS);
    yController.setTolerance(Y_TOLERANCE_METERS);
    thetaController.setTolerance(THETA_TOLERANCE_RADIANS);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
    lastSeenTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    // Read from Vision's cached frame rather than draining getAllUnreadResults() directly
    Optional<PhotonTrackedTarget> targetOpt = vision.getFrontTarget(targetTagId);

    if (targetOpt.isPresent()) {
      lastSeenTimestamp = Timer.getFPGATimestamp();
      PhotonTrackedTarget target = targetOpt.get();

      // Transform from Robot Center -> Camera -> Target
      Transform3d robotToTarget =
          vision.getRobotToFrontCam().plus(target.getBestCameraToTarget());

      // Convert into Tag-relative coordinates to get robot pose in tag frame:
      // Tag sits at origin facing +X out of the wall; robot facing the tag has yaw = PI (180 deg)
      Pose3d tagOrigin = new Pose3d();
      Pose3d robotInTagFrame = tagOrigin.transformBy(robotToTarget.inverse());

      // Current error relative to desired tag-frame pose:
      // Desired robot pose in tag frame: X = +targetXDistanceMeters, Y = targetYLateralOffsetMeters, Yaw = PI
      double currentX = robotInTagFrame.getX();
      double currentY = robotInTagFrame.getY();
      double currentYaw = robotInTagFrame.getRotation().getZ();

      // Compute tag-frame corrections
      double tagFrameVx = xController.calculate(currentX, targetXDistanceMeters);
      double tagFrameVy = yController.calculate(currentY, targetYLateralOffsetMeters);
      double omegaSpeed = thetaController.calculate(currentYaw, Math.PI);

      // Rotate tag-frame velocities into robot-relative ChassisSpeeds
      // Since robot yaw in tag frame is currentYaw (~PI when facing tag), rotate by -currentYaw
      double cosYaw = Math.cos(-currentYaw);
      double sinYaw = Math.sin(-currentYaw);
      double robotVx = tagFrameVx * cosYaw - tagFrameVy * sinYaw;
      double robotVy = tagFrameVx * sinYaw + tagFrameVy * cosYaw;

      robotVx = MathUtil.clamp(robotVx, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
      robotVy = MathUtil.clamp(robotVy, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
      omegaSpeed = MathUtil.clamp(omegaSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

      SmartDashboard.putNumber("Align/ErrorX", targetXDistanceMeters - currentX);
      SmartDashboard.putNumber("Align/ErrorY", targetYLateralOffsetMeters - currentY);
      SmartDashboard.putNumber(
          "Align/ErrorDeg", Units.radiansToDegrees(MathUtil.angleModulus(Math.PI - currentYaw)));

      // Swerve.drive expects robot-relative ChassisSpeeds
      swerve.drive(new ChassisSpeeds(robotVx, robotVy, omegaSpeed));
    } else {
      // Stop drivetrain if tag is temporarily occluded
      swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    boolean timedOut =
        (Timer.getFPGATimestamp() - lastSeenTimestamp) > LOST_TARGET_TIMEOUT_SECONDS;
    if (timedOut) {
      return true;
    }
    return vision.getFrontTarget(targetTagId).isPresent()
        && xController.atSetpoint()
        && yController.atSetpoint()
        && thetaController.atSetpoint();
  }
}
