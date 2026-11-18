package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagAlignment extends Command {
  private final Swerve swerve;
  private final Vision vision;
  private final int targetTagId;
  private final PIDController xController = new PIDController(1.5, 0.0, 0.05);
  private final PIDController yController = new PIDController(1.5, 0.0, 0.05);
  private final PIDController thetaController = new PIDController(3.0, 0.0, 0.1);

  private static final double TARGET_X_METERS = 1.0;
  private static final double TARGET_Y_METERS = 0.0;
  private static final double TARGET_YAW_RADIANS = Math.PI;

  private static final double X_TOLERANCE_METERS = 0.04;
  private static final double Y_TOLERANCE_METERS = 0.04;
  private static final double THETA_TOLERANCE_RADIANS = Units.degreesToRadians(2.0);

  private static final double MAX_LINEAR_SPEED = 2.0;
  private static final double MAX_ANGULAR_SPEED = Math.PI;


  public AprilTagAlignment(Swerve swerve, Vision vision, int targetTagId) {
    this.swerve = swerve;
    this.vision = vision;
    this.targetTagId = targetTagId;

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
  }

  @Override
  public void execute() {
    Optional<PhotonTrackedTarget> targetOpt = getTargetFromVision();

    if (targetOpt.isPresent()) {
      PhotonTrackedTarget target = targetOpt.get();

      Transform3d cameraToTarget = target.getBestCameraToTarget();

      double currentX = cameraToTarget.getX();
      double currentY = cameraToTarget.getY();
      double currentYaw = cameraToTarget.getRotation().getZ();

      double xSpeed = -xController.calculate(currentX, TARGET_X_METERS);
      double ySpeed = -yController.calculate(currentY, TARGET_Y_METERS);
      double omegaSpeed = -thetaController.calculate(currentYaw, TARGET_YAW_RADIANS);

      xSpeed = Math.max(-MAX_LINEAR_SPEED, Math.min(MAX_LINEAR_SPEED, xSpeed));
      ySpeed = Math.max(-MAX_LINEAR_SPEED, Math.min(MAX_LINEAR_SPEED, ySpeed));
      omegaSpeed = Math.max(-MAX_ANGULAR_SPEED, Math.min(MAX_ANGULAR_SPEED, omegaSpeed));

      swerve.drive(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
    } else {
      swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint()
        && yController.atSetpoint()
        && thetaController.atSetpoint();
  }

  private Optional<PhotonTrackedTarget> getTargetFromVision() {
    List<PhotonPipelineResult> results = vision.getFrontApriltagCamera().getAllUnreadResults();
    if (results.isEmpty()) {
      return Optional.empty();
    }

    PhotonPipelineResult latestResult = results.get(results.size() - 1);
    if (!latestResult.hasTargets()) {
      return Optional.empty();
    }

    if (targetTagId == -1) {
      return Optional.of(latestResult.getBestTarget());
    }

    for (PhotonTrackedTarget target : latestResult.getTargets()) {
      if (target.getFiducialId() == targetTagId) {
        return Optional.of(target);
      }
    }

    return Optional.empty();
  }
}
