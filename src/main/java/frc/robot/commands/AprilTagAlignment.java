package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagAlignment extends Command {

    private static final double TRANSLATION_KP = 1.8;
    private static final double ROTATION_KP = 2.5;

    private static final double MAX_LINEAR_SPEED = 0.75;
    private static final double MAX_ANGULAR_SPEED = 1.2;

    private static final double MIN_LINEAR_SPEED = 0.18;
    private static final double MIN_ANGULAR_SPEED = 0.30;

    private static final double POSITION_TOLERANCE = 0.05;
    private static final double ANGLE_TOLERANCE =
        Math.toRadians(3.0);

    private static final double MAX_FRAME_AGE = 0.35;
    private static final double MAX_TARGET_RANGE = 4.0;

    private final Swerve swerve;
    private final Vision vision;

    private final int requestedTagId;
    private final double targetDistance;
    private final double lateralOffset;

    private int lockedTagId;

    public AprilTagAlignment(
        Swerve swerve,
        Vision vision,
        int targetTagId
    ) {
        this(
            swerve,
            vision,
            targetTagId,
            VisionConstants.kAlignTargetXDistanceMeters,
            0.0
        );
    }

    public AprilTagAlignment(
        Swerve swerve,
        Vision vision,
        int targetTagId,
        double targetDistance,
        double lateralOffset
    ) {
        this.swerve = swerve;
        this.vision = vision;
        this.requestedTagId = targetTagId;
        this.targetDistance = targetDistance;
        this.lateralOffset = lateralOffset;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        lockedTagId = requestedTagId;

        SmartDashboard.putBoolean("Align/Active", true);
        SmartDashboard.putNumber(
            "Align/LockedTagId", lockedTagId
        );

        stop("Waiting for target");
    }

    @Override
    public void execute() {

        if (!DriverStation.isTeleopEnabled()) {
            stop("Teleop not enabled");
            return;
        }

        if (!swerve.isReady()) {
            stop("Drivetrain not ready");
            return;
        }

        if (!vision.getFrontCamera().isConnected()) {
            stop("Camera disconnected");
            return;
        }

        PhotonPipelineResult result =
            vision.getFrontLatestResult();

        if (result == null) {
            stop("No camera frame");
            return;
        }

        double timestamp = result.getTimestampSeconds();

        double frameAge =
            Timer.getFPGATimestamp() - timestamp;

        SmartDashboard.putNumber(
            "Align/FrameAgeSeconds", frameAge
        );

        if (!Double.isFinite(frameAge)
            || timestamp <= 0.0
            || frameAge < -0.02
            || frameAge > MAX_FRAME_AGE) {

            stop("Invalid or stale camera timestamp");
            return;
        }

        if (!result.hasTargets()) {
            stop("No AprilTags visible");
            return;
        }

        PhotonTrackedTarget target = null;

        for (PhotonTrackedTarget candidate :
            result.getTargets()) {

            if (candidate.getFiducialId() < 0) {
                continue;
            }

            if (lockedTagId == -1
                || candidate.getFiducialId() == lockedTagId) {

                target = candidate;
                break;
            }
        }

        if (target == null) {
            stop("Locked AprilTag not visible");
            return;
        }

        lockedTagId = target.getFiducialId();

        SmartDashboard.putNumber(
            "Align/LockedTagId", lockedTagId
        );

        SmartDashboard.putBoolean(
            "Align/HasValidTarget", true
        );

        double ambiguity = target.getPoseAmbiguity();

        SmartDashboard.putNumber(
            "Align/PoseAmbiguity", ambiguity
        );

        SmartDashboard.putNumber(
            "Align/TargetYawDegrees", target.getYaw()
        );

        SmartDashboard.putNumber(
            "Align/TargetArea", target.getArea()
        );

        Transform3d cameraToTarget =
            target.getBestCameraToTarget();

        if (!validTranslation(cameraToTarget)) {
            aimOnly(target);
            return;
        }

        Transform3d robotToTarget =
            vision.getRobotToFrontCam().plus(cameraToTarget);

        double targetX = robotToTarget.getX();
        double targetY = robotToTarget.getY();

        SmartDashboard.putNumber("Align/TargetX", targetX);
        SmartDashboard.putNumber("Align/TargetY", targetY);

        if (!Double.isFinite(targetX)
            || !Double.isFinite(targetY)
            || targetX <= 0.0) {

            aimOnly(target);
            return;
        }

        if (Double.isFinite(ambiguity)
            && ambiguity >= 0.0
            && ambiguity <=
                VisionConstants.kMaxSingleTagAmbiguity) {

            Pose3d robotInTagFrame =
                new Pose3d().transformBy(
                    robotToTarget.inverse()
                );

            if (Double.isFinite(robotInTagFrame.getX())
                && Double.isFinite(robotInTagFrame.getY())
                && Double.isFinite(
                    robotInTagFrame.getRotation().getZ()
                )
                && robotInTagFrame.getX() > 0.0) {

                alignPrecisely(robotInTagFrame);
                return;
            }
        }

        Transform3d alternate =
            target.getAlternateCameraToTarget();

        if (validTranslation(alternate)
            && cameraToTarget.getTranslation()
                .minus(alternate.getTranslation())
                .getNorm() <= 0.30) {

            followPosition(targetX, targetY);
        } else {
            aimOnly(target);
        }
    }

    private void alignPrecisely(Pose3d robotPose) {

        double yaw = robotPose.getRotation().getZ();

        double errorX =
            targetDistance - robotPose.getX();

        double errorY =
            lateralOffset - robotPose.getY();

        double errorTheta =
            MathUtil.angleModulus(Math.PI - yaw);

        double distanceError = Math.hypot(errorX, errorY);

        double speed = calculateOutput(
            distanceError,
            TRANSLATION_KP,
            MIN_LINEAR_SPEED,
            MAX_LINEAR_SPEED,
            POSITION_TOLERANCE
        );

        double vxTag = 0.0;
        double vyTag = 0.0;

        if (distanceError > POSITION_TOLERANCE) {
            vxTag = speed * errorX / distanceError;
            vyTag = speed * errorY / distanceError;
        }

        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);

        double vx = vxTag * cos + vyTag * sin;
        double vy = -vxTag * sin + vyTag * cos;

        double omega = calculateOutput(
            errorTheta,
            ROTATION_KP,
            MIN_ANGULAR_SPEED,
            MAX_ANGULAR_SPEED,
            ANGLE_TOLERANCE
        );

        publishErrors(errorX, errorY, errorTheta);

        drive(vx, vy, omega, "Precise alignment");
    }

    private void followPosition(double x, double y) {

        double errorX = x - targetDistance;
        double errorY = y + lateralOffset;

        double bearing = Math.atan2(y, x);

        double vx = calculateOutput(
            errorX,
            TRANSLATION_KP,
            MIN_LINEAR_SPEED,
            0.40,
            POSITION_TOLERANCE
        );

        double vy = calculateOutput(
            errorY,
            TRANSLATION_KP,
            MIN_LINEAR_SPEED,
            0.40,
            POSITION_TOLERANCE
        );

        double omega = calculateOutput(
            bearing,
            ROTATION_KP,
            MIN_ANGULAR_SPEED,
            MAX_ANGULAR_SPEED,
            ANGLE_TOLERANCE
        );

        if (Math.abs(bearing) > Math.toRadians(20.0)) {
            vx = Math.min(vx, 0.0);
        }

        double speed = Math.hypot(vx, vy);

        if (speed > 0.40) {
            vx *= 0.40 / speed;
            vy *= 0.40 / speed;
        }

        publishErrors(errorX, errorY, bearing);

        drive(vx, vy, omega, "Position following");
    }

    private void aimOnly(PhotonTrackedTarget target) {

        double yaw = -Math.toRadians(target.getYaw());

        if (!Double.isFinite(yaw)) {
            stop("Invalid target yaw");
            return;
        }

        double omega = calculateOutput(
            yaw,
            ROTATION_KP,
            MIN_ANGULAR_SPEED,
            MAX_ANGULAR_SPEED,
            ANGLE_TOLERANCE
        );

        publishErrors(Double.NaN, Double.NaN, yaw);

        drive(0.0, 0.0, omega, "2D aiming only");
    }

    private boolean validTranslation(
        Transform3d transform
    ) {
        if (transform == null) {
            return false;
        }

        double range =
            transform.getTranslation().getNorm();

        return Double.isFinite(range)
            && range >= 0.10
            && range <= MAX_TARGET_RANGE
            && transform.getX() > 0.0;
    }

    private double calculateOutput(
        double error,
        double kp,
        double minimum,
        double maximum,
        double tolerance
    ) {
        if (!Double.isFinite(error)
            || Math.abs(error) <= tolerance) {

            return 0.0;
        }

        double magnitude = MathUtil.clamp(
            Math.abs(error) * kp,
            minimum,
            maximum
        );

        return Math.copySign(magnitude, error);
    }

    private void drive(
        double vx,
        double vy,
        double omega,
        String status
    ) {
        boolean stationary =
            vx == 0.0
            && vy == 0.0
            && omega == 0.0;

        if (stationary) {
            swerve.stop();

            SmartDashboard.putBoolean(
                "Align/Aligned",
                status.equals("Precise alignment")
            );

            publishOutput(0.0, 0.0, 0.0, status + ": holding");
            return;
        }

        SmartDashboard.putBoolean("Align/Aligned", false);

        if (!swerve.drive(
            new ChassisSpeeds(vx, vy, omega)
        )) {
            stop("Drive request rejected");
            return;
        }

        publishOutput(vx, vy, omega, status);
    }

    private void publishErrors(
        double x,
        double y,
        double theta
    ) {
        SmartDashboard.putNumber("Align/ErrorX", x);
        SmartDashboard.putNumber("Align/ErrorY", y);

        SmartDashboard.putNumber(
            "Align/ErrorDeg", Math.toDegrees(theta)
        );
    }

    private void publishOutput(
        double vx,
        double vy,
        double omega,
        String status
    ) {
        SmartDashboard.putNumber("Align/Vx", vx);
        SmartDashboard.putNumber("Align/Vy", vy);
        SmartDashboard.putNumber("Align/Omega", omega);
        SmartDashboard.putString("Align/Status", status);
    }

    private void stop(String status) {
        swerve.stop();

        SmartDashboard.putBoolean(
            "Align/HasValidTarget", false
        );

        SmartDashboard.putBoolean(
            "Align/Aligned", false
        );

        publishErrors(
            Double.NaN,
            Double.NaN,
            Double.NaN
        );

        publishOutput(0.0, 0.0, 0.0, status);
    }

    @Override
    public void end(boolean interrupted) {
        stop("Inactive");

        SmartDashboard.putBoolean("Align/Active", false);
        SmartDashboard.putNumber("Align/LockedTagId", -1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
