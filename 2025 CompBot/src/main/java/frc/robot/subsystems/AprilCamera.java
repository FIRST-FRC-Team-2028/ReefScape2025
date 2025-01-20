// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CamConstants;


public class AprilCamera extends SubsystemBase {
  private PhotonCamera camera;
  boolean hasTargets;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget target;
  Pose3d robotPose;
  AprilTagFieldLayout aprilTagFieldLayout;
  double distanceToTarget;
  Transform3d robotToCam;
  PhotonPoseEstimator photonPoseEstimator;
  private Drivetrain drivetrain;
  Optional<Pose3d> targetPose;
  Pose3d truePose;
  Optional<EstimatedRobotPose> poseEstimate;
  EstimatedRobotPose poseEstimateTrue;
  Pose3d lastPose;
  Pose3d estimatedPose3d;
  boolean poseEstimated;
  EstimatedRobotPose estimatedPose;
  double estimatedPoseTime;
  //private final Solenoid blue;
  
  //private PhotonPipelineResult result;
  /** Creates a new AprilTags. */
  public AprilCamera() {

    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    //blue = new Solenoid(PneumaticsModuleType.CTREPCM, Lights.blue); //April tags
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
     //Cam mounted facing forward, 0.3302 meters in front of the center, 0 meters left/right of center, 
     // and 0.1778 meters of elevation (off floor)            on project X
    robotToCam = new Transform3d(new Translation3d(.381, 0.0, 0.0254),
                new Rotation3d(0,0,0));
 
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    PoseStrategy.LOWEST_AMBIGUITY, robotToCam);


  }



  /*public void aprilTagsOn() {
    blue.set(true);
  }

  public void aprilTagsOff() {
    blue.set(false);
  }*/


  public Pose3d getRobotPosition() {
    if (hasTargets) return robotPose;
    else return new Pose3d(-999., 999., -999., new Rotation3d(999, -999, -1000));
  }
  void print() {
    System.out.println(getRobotPosition());
  }

  public double tagYaw() {
    if (hasTargets){
    return target.getYaw();
    } else return -999;
  }
  public boolean generalTarget() {
    return hasTargets;
  }
  public double tagArea(){
    if (hasTargets){
    return target.getArea();
    }else return -999;
  }
  public double getTimestampSeconds(){
    return 0;
  }
   double getHeight(){
    targetPose = aprilTagFieldLayout.getTagPose(target.fiducialId);
    truePose = targetPose.get();
    
    return truePose.getZ();
    
  }


   double getDistanceToTarget(){
    return PhotonUtils.calculateDistanceToTargetMeters(CamConstants.camera_Height_Meters,
                                                      getHeight(),
                                                      CamConstants.camera_Pitch_Radians,
                                                      Units.degreesToRadians(target.getPitch()));
          
    
  }

 /* public Optional<EstimatedRobotPose> update(){
    poseEstimate = photonPoseEstimator.update(getLatestResult());
    return poseEstimate;
  } */

  public Pose3d getPose3d(){
    return estimatedPose3d;
  }

  public boolean isPoseEstimated(){
    return poseEstimated;
  }
 /* public EstimatedRobotPose getPoseTrue(){
  poseEstimateTrue = poseEstimate.orElse(poseEstimateTrue);
    return poseEstimateTrue;
  }
    */


  //public void showYaw() {
  //  SmartDashboard.putNumber("YE Yaw", target.getYaw());         IF DONT HAVE TARGET, DONT RUN SHOWYAW
  //}



  @Override
  public void periodic() {
    


    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    if (hasTargets) {
      targets = result.getTargets();
      target = result.getBestTarget();
      
      poseEstimate = photonPoseEstimator.update(result);
      if (poseEstimate.isPresent()){
      estimatedPose = poseEstimate.get();
      estimatedPoseTime = estimatedPose.timestampSeconds;
      estimatedPose3d = estimatedPose.estimatedPose;
      poseEstimated = true;
      } else poseEstimated = false; //estimatedPose3d = new Pose3d(999, 999, 999, new Rotation3d(999,999,999));
  


      robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotToCam);
      //showYaw();

      //SmartDashboard.putString("Robot Pose 1", photonPoseEstimator.getReferencePose().toString());
      //SmartDashboard.putString("Robot Pose 2", photonPoseEstimator.update(result).toString());
     
      SmartDashboard.putNumber("Robot Pose X", getPose3d().getX());
      SmartDashboard.putNumber("Robot Pose Y", getPose3d().getY());
      SmartDashboard.putNumber("April Tag X", target.getFiducialId());
      SmartDashboard.putNumber("Get Yaw", target.getYaw());
      SmartDashboard.putNumber("Get Distance", getDistanceToTarget());

      // This method will be called once per scheduler run
    } else {
      SmartDashboard.putNumber("April Tag X", 999.);
      SmartDashboard.putNumber("Get Yaw", 999.);
      SmartDashboard.putNumber("Get Distance", 999.);
      estimatedPose3d = new Pose3d(999, 999, 999, new Rotation3d(999,999,999));
      poseEstimated = false;
    }
    
  
  }
}