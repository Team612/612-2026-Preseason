package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  @FunctionalInterface
  public interface VisionMeasurementConsumer {
    void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }

  private final AprilTagFieldLayout fieldLayout;
  private final PhotonCamera frontCamera;
  private final PhotonCamera backCamera;
  private final PhotonPoseEstimator frontPoseEstimator;
  private final PhotonPoseEstimator backPoseEstimator;
  private final VisionMeasurementConsumer measurementConsumer;

  private PhotonPipelineResult frontLatestResult = new PhotonPipelineResult();
  private PhotonPipelineResult backLatestResult = new PhotonPipelineResult();
  private Pose2d latestEstimatedPose = new Pose2d();

  /**
   * Creates the Vision subsystem for 2026 REBUILT using WPILib 2026 + PhotonVision 2026 APIs.
   *
   * @param measurementConsumer Callback to feed vision estimates into your SwerveDrivePoseEstimator
   *     (pass `swerve::addVisionMeasurement` if supported, or `(pose, ts, std) -> {}`).
   */
  public Vision(VisionMeasurementConsumer measurementConsumer) {
    this.measurementConsumer =
        measurementConsumer != null ? measurementConsumer : (pose, ts, std) -> {};

    // Use k2026RebuiltWelded (default) or k2026RebuiltAndymark depending on event field type
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    this.frontCamera = new PhotonCamera(VisionConstants.kFrontCameraName);
    this.backCamera = new PhotonCamera(VisionConstants.kBackCameraName);

    // 2026 PhotonPoseEstimator constructor takes (AprilTagFieldLayout, Transform3d)
    this.frontPoseEstimator =
        new PhotonPoseEstimator(fieldLayout, VisionConstants.kRobotToFrontCam);
    this.backPoseEstimator =
        new PhotonPoseEstimator(fieldLayout, VisionConstants.kRobotToBackCam);
  }

  @Override
  public void periodic() {
    // Drain the unread result queues once per loop so commands don't steal frames from pose estimation
    List<PhotonPipelineResult> frontUnread = frontCamera.getAllUnreadResults();
    if (!frontUnread.isEmpty()) {
      frontLatestResult = frontUnread.get(frontUnread.size() - 1);
      processCameraResults(frontUnread, frontPoseEstimator);
    }

    List<PhotonPipelineResult> backUnread = backCamera.getAllUnreadResults();
    if (!backUnread.isEmpty()) {
      backLatestResult = backUnread.get(backUnread.size() - 1);
      processCameraResults(backUnread, backPoseEstimator);
    }

    SmartDashboard.putBoolean("Vision/FrontConnected", frontCamera.isConnected());
    SmartDashboard.putBoolean("Vision/BackConnected", backCamera.isConnected());
    SmartDashboard.putBoolean("Vision/FrontHasTarget", frontHasTarget());
    SmartDashboard.putBoolean("Vision/BackHasTarget", backHasTarget());
    SmartDashboard.putNumber("Vision/FrontBestTagId", getFrontBestTagId());
    SmartDashboard.putNumber("Vision/BackBestTagId", getBackBestTagId());
  }

  private void processCameraResults(
      List<PhotonPipelineResult> results, PhotonPoseEstimator estimator) {
    for (PhotonPipelineResult result : results) {
      if (!result.hasTargets()) {
        continue;
      }

      // 2026 PhotonVision API: prefer coprocessor MultiTag, fall back to Lowest Ambiguity
      Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = estimator.estimateLowestAmbiguityPose(result);
      }

      if (visionEst.isPresent()) {
        EstimatedRobotPose est = visionEst.get();
        Matrix<N3, N1> stdDevs = calculateStdDevs(est, result.getTargets());

        // Reject high-ambiguity single-tag estimates
        if (stdDevs.get(0, 0) < Double.MAX_VALUE) {
          Pose2d estPose2d = est.estimatedPose.toPose2d();
          latestEstimatedPose = estPose2d;
          measurementConsumer.accept(estPose2d, est.timestampSeconds, stdDevs);
        }
      }
    }
  }

  /**
   * Computes dynamic measurement standard deviations based on tag count and average distance.
   */
  private Matrix<N3, N1> calculateStdDevs(
      EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double avgDist = 0.0;

    for (PhotonTrackedTarget tgt : targets) {
      Optional<Pose3d> tagPose = fieldLayout.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }
      numTags++;
      avgDist +=
          tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    avgDist /= numTags;

    // Single-tag filtering: reject ambiguous or overly distant single-tag reads
    if (numTags == 1) {
      PhotonTrackedTarget bestTarget = targets.get(0);
      if (bestTarget.getPoseAmbiguity() > VisionConstants.kMaxSingleTagAmbiguity
          || avgDist > 4.0) {
        return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      }
    }

    Matrix<N3, N1> baseStdDevs =
        (numTags > 1) ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;

    // Scale uncertainty quadratically with distance
    double scale = 1.0 + (avgDist * avgDist / 30.0);
    return baseStdDevs.times(scale);
  }

  public PhotonCamera getFrontCamera() {
    return frontCamera;
  }

  public PhotonCamera getBackCamera() {
    return backCamera;
  }

  public PhotonPipelineResult getFrontLatestResult() {
    return frontLatestResult;
  }

  public PhotonPipelineResult getBackLatestResult() {
    return backLatestResult;
  }

  public boolean frontHasTarget() {
    return frontLatestResult != null && frontLatestResult.hasTargets();
  }

  public boolean backHasTarget() {
    return backLatestResult != null && backLatestResult.hasTargets();
  }

  public int getFrontBestTagId() {
    return frontHasTarget() ? frontLatestResult.getBestTarget().getFiducialId() : -1;
  }

  public int getBackBestTagId() {
    return backHasTarget() ? backLatestResult.getBestTarget().getFiducialId() : -1;
  }

  /**
   * Safely retrieves a specific AprilTag target from the cached latest front camera frame.
   *
   * @param tagId Desired AprilTag ID, or -1 to return the best visible target.
   */
  public Optional<PhotonTrackedTarget> getFrontTarget(int tagId) {
    if (!frontHasTarget()) {
      return Optional.empty();
    }
    if (tagId == -1) {
      return Optional.of(frontLatestResult.getBestTarget());
    }
    for (PhotonTrackedTarget target : frontLatestResult.getTargets()) {
      if (target.getFiducialId() == tagId) {
        return Optional.of(target);
      }
    }
    return Optional.empty();
  }

  /**
   * Safely retrieves a specific AprilTag target from the cached latest back camera frame.
   *
   * @param tagId Desired AprilTag ID, or -1 to return the best visible target.
   */
  public Optional<PhotonTrackedTarget> getBackTarget(int tagId) {
    if (!backHasTarget()) {
      return Optional.empty();
    }
    if (tagId == -1) {
      return Optional.of(backLatestResult.getBestTarget());
    }
    for (PhotonTrackedTarget target : backLatestResult.getTargets()) {
      if (target.getFiducialId() == tagId) {
        return Optional.of(target);
      }
    }
    return Optional.empty();
  }

  public Optional<Pose3d> getTagPose(int tagId) {
    return fieldLayout.getTagPose(tagId);
  }

  public Pose2d getLatestEstimatedPose() {
    return latestEstimatedPose;
  }

  public Transform3d getRobotToFrontCam() {
    return VisionConstants.kRobotToFrontCam;
  }

  public Transform3d getRobotToBackCam() {
    return VisionConstants.kRobotToBackCam;
  }
}
