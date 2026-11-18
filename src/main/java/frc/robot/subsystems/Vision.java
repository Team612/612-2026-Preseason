package frc.robot.subsystems;


import java.io.IOException;
import java.util.List;
import java.util.Optional;


import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import com.ctre.phoenix6.*;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Vision extends SubsystemBase {
 private static AprilTagFieldLayout aprilTagFieldLayout;
 private PhotonPoseEstimator photonPoseEstimator;


 private static Vision visionInstance = null;


 PhotonCamera frontCamera;
 PhotonCamera backCamera;
 List<PhotonPipelineResult> frontCameraResults;
 List<PhotonPipelineResult> backCameraResults;
 PhotonPipelineResult frontLatestResult;






 private Pose2d robotInTagPose;


 public Vision() {


   frontCamera = new PhotonCamera(Constants.AutoConstants.frontCamera);
   backCamera = new PhotonCamera(Constants.AutoConstants.backCamera);


  
   frontCameraResults = frontCamera.getAllUnreadResults();
   backCameraResults = backCamera.getAllUnreadResults();


   if (frontCameraResults.isEmpty()){
     frontLatestResult = new PhotonPipelineResult();
   }
  
   else {
     frontLatestResult = frontCameraResults.get(frontCameraResults.size()-1);
   }


   aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
   photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.AutoConstants.frontCameraTransform);
 }


 public static Vision getVisionInstance() {
   if (visionInstance == null) {
     visionInstance = new Vision();
   }
   return visionInstance;
 }




 public PhotonPoseEstimator getPhotonPoseEstimator(){
   return photonPoseEstimator;
 }
 public PhotonCamera getFrontApriltagCamera(){
   return frontCamera;
 }
 public PhotonCamera getBackApriltagCamera(){
   return backCamera;
 }


 public Transform3d getRobotToFrontCam(){
   return Constants.AutoConstants.frontCameraTransform;
 }


 public Transform3d getRobotToBackCam(){
   return Constants.AutoConstants.backCameraTransform;
 }


 public boolean frontHasCalibration(){
   if (frontCamera.getDistCoeffs().equals(Optional.empty())){
     return false;
   }
   return true;
 }


 public boolean backHasCalibration(){
   if (backCamera.getDistCoeffs().equals(Optional.empty())){
     return false;
   }
   return true;
 }


 public boolean frontHasTag(){
   if(frontCameraResults.isEmpty()){
     return false;
   }
   else{
       if(frontLatestResult.hasTargets() && frontLatestResult.getBestTarget().getFiducialId() >= 0){
           return true;
       }
   }
   return false;
 }


 public boolean backHasTag(){
   for (PhotonPipelineResult result : backCameraResults){
       if (result.hasTargets() && result.getBestTarget().getFiducialId() >= 0){
           return true;
       }
   }
   return false;
 }




 public int frontTagID(){
   if(frontCameraResults.isEmpty()){
     return -1;
   }
   else{
       if(frontLatestResult.hasTargets() && frontLatestResult.getBestTarget().getFiducialId() >= 0){
           return frontLatestResult.getBestTarget().getFiducialId();
       }
   }
    return -1;
 }
 public int backTagID(){
   for (PhotonPipelineResult result : backCameraResults){
       if (result.hasTargets()){
           return result.getBestTarget().getFiducialId();
       }
    }
    return -1;
 }


 public PhotonPipelineResult getFrontPipelineResult() {
   if(frontCameraResults.isEmpty()){
     return null;
   }
   else{
     return frontLatestResult;
   }
  } 


 public PhotonPipelineResult getBackPipelineResult() {
   for (PhotonPipelineResult result : backCameraResults){
       return result;
   }
   return null;
 } 
 public Pose2d getBackRelativeTagPose() {
   for (PhotonPipelineResult result : backCameraResults){
       if (result.hasTargets()){
           PhotonTrackedTarget bestTarget = result.getBestTarget();
           Transform3d tagSpace = bestTarget.getBestCameraToTarget().plus(getRobotToBackCam());
           return new Pose2d(tagSpace.getX(), tagSpace.getY(), tagSpace.getRotation().toRotation2d());
       }
   }
   return new Pose2d();
 }


 public Pose2d getRobotPose(){
   Pose2d tagPose = robotInTagPose;
   return new Pose2d().transformBy(new Transform2d(tagPose.getTranslation(), tagPose.getRotation()));
 }
  // return tag pose
 public Pose3d return_tag_pose(int id) {
   Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
   return pose_of_tag.get();
 }


 // self calculations
 public Pose3d return_camera_pose_tag(int id, PhotonPipelineResult results) {
   Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
   Pose3d tag_pose = pose_of_tag.get();
   Transform3d cameraTransform = results.getBestTarget().getBestCameraToTarget();
   return tag_pose.plus(cameraTransform);
 }
 public Optional<EstimatedRobotPose> return_photon_pose(Pose2d latestPose) {
   photonPoseEstimator.setReferencePose(latestPose);
   return photonPoseEstimator.update(new PhotonPipelineResult());
 }




 @Override
 public void periodic() {


  
   frontCameraResults = frontCamera.getAllUnreadResults();
   backCameraResults = backCamera.getAllUnreadResults();


   if (!frontCameraResults.isEmpty()){
     frontLatestResult = frontCameraResults.get(frontCameraResults.size()-1);
   }
   else {
     frontLatestResult = new PhotonPipelineResult();
   }




 }


 }



