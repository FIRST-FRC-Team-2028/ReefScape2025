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

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CamConstants;
import frc.robot.Constants.DriveConstants;


public class AprilCamera extends SubsystemBase {
  private PhotonCamera camera;
  boolean hasTargets, reefTag = false;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget target;
  Pose3d robotPose;
  AprilTagFieldLayout aprilTagFieldLayout;
  double distanceToTarget;
  double matchTime;
  Transform3d robotToCam;
  PhotonPoseEstimator photonPoseEstimator;
  Optional<Pose3d> targetPose;
  Pose3d truePose;
  Optional<EstimatedRobotPose> poseEstimate;
  EstimatedRobotPose poseEstimateTrue;
  Pose3d lastPose;
  Pose3d estimatedPose3d;
  boolean poseEstimated;
  EstimatedRobotPose estimatedPose;
  double estimatedPoseTime;
  Pose2d poseTarget;
  PhotonPipelineResult result;
  RobotContainer m_RobotContainer;
  //private final Solenoid blue;
  
  //private PhotonPipelineResult result;
  /** Uses the PhotonVision VendorDep to process April Tags
   * <p>Methods<ul>
   * <li>getRobotPosition - Gets the position of the robot
   * <li>print - Prints the robot position
   * <li>tagYaw - Gets the yaw of an AprilTag
   * <li>generalTargets - Gets if the camera has targets
   * <li>tagArea - Gets the area of an Apriltag
   * <li>getHeight - Gets the height of an AprilTag
   * <li>getDistanceToTarget - Calculates the distance to targeted Apriltag
   * <li>getPoseToPose - Changes the position the robot is getting the distance to and calculates the distance there
   * <li>getPose3d - gets the estimated position of the robot as a Pose3d
   * <li>isPoseEstimated - gets the boolean to determine if a pose is estimated
   * </ul>
   * </p>
   */
  public AprilCamera(RobotContainer m_RobotContainer) {

    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    //blue = new Solenoid(PneumaticsModuleType.CTREPCM, Lights.blue); //April tags
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
     //Cam mounted facing forward, 0.3302 meters in front of the center, 0 meters left/right of center, 
     // and 0.1778 meters of elevation (off floor)            on project X
    robotToCam = new Transform3d(new Translation3d(CamConstants.robotToCamX, CamConstants.robotToCamY, CamConstants.robotToCamZ),
                new Rotation3d(0,CamConstants.camera_Pitch_Radians, CamConstants.cameraYawRadians));
    this.m_RobotContainer = m_RobotContainer;
            
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
  }

  /** Uses the PhotonVision VendorDep to process April Tags
   * <p>Methods<ul>
   * <li>getRobotPosition - Gets the position of the robot
   * <li>print - Prints the robot position
   * <li>tagYaw - Gets the yaw of an AprilTag
   * <li>generalTargets - Gets if the camera has targets
   * <li>tagArea - Gets the area of an Apriltag
   * <li>getHeight - Gets the height of an AprilTag
   * <li>getDistanceToTarget - Calculates the distance to targeted Apriltag
   * <li>getPoseToPose - Changes the position the robot is getting the distance to and calculates the distance there
   * <li>getPose3d - gets the estimated position of the robot as a Pose3d
   * <li>isPoseEstimated - gets the boolean to determine if a pose is estimated
   * </ul>
   * </p>
   */
  /*public AprilCamera(){
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");
    //blue = new Solenoid(PneumaticsModuleType.CTREPCM, Lights.blue); //April tags
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
     //Cam mounted facing forward, 0.3302 meters in front of the center, 0 meters left/right of center, 
     // and 0.1778 meters of elevation (off floor)            on project X
    robotToCam = new Transform3d(new Translation3d(CamConstants.robotToCamX, CamConstants.robotToCamY, CamConstants.robotToCamZ),
                new Rotation3d(0,CamConstants.camera_Pitch_Radians, CamConstants.cameraYawRadians));
            
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
  }*/




  /**
   * Gets the position of the robot
   * @return the position of the robot in a pose 3d if there is a target otherwise returns 999
   * 
   */
  public Pose3d getRobotPosition() {
    if (hasTargets) return robotPose;
    else return new Pose3d(-999., 999., -999., new Rotation3d(999, -999, -1000));
  }

  /** Prints the robot position */
  void print() {
    System.out.println(getRobotPosition());
  }

  /**
   * If the camera has a target 
   * @return the yaw of the target
   * <p>
   * Otherwise returns -999.
   */
  public double tagYaw() {
    if (hasTargets){
    return target.getYaw();
    } else return -999;
  }
  

  /**
   * Gets if the camera has targets
   * @return if the camera has targets
   */
  public boolean generalTarget() {
    return hasTargets;
  }

   /**
   * If the camera has a target 
   * @return the area of the target
   * <p>
   * Otherwise returns -999.
   */
  public double tagArea(){
    if (hasTargets){
      return target.getArea();
    }else return -999;
  }

  public double tagPitch(){
    if (hasTargets){
      return target.getPitch();
    } else return -999;
  }
  
  /**
   * Gets the height of an AprilTag 
   * @return the Z of an Apriltag
   */
   double getHeight(){
    targetPose = aprilTagFieldLayout.getTagPose(target.fiducialId);
    truePose = targetPose.get();
    
    return truePose.getZ();
  }
    
  /** return the relative angle between the camera axis and the target normal.
   * <p>
   * @return angle in degrees
   */
  public double getFaceVec()
  {
    if (hasTargets)
       return (target.getSkew()-360.)%360.;
    else return -999.;
    
  }

  /**
   * Calculates the distance to targeted Apriltag from the front of the robot
   * @return The distance from the front of the robot to the april tag
   */
  double getDistanceToTarget(){
    double distance = PhotonUtils.calculateDistanceToTargetMeters(CamConstants.camera_Height_Meters,
                                                      getHeight(),
                                                      CamConstants.camera_Pitch_Radians,
                                                      Units.degreesToRadians(target.getPitch()));
    return (Math.sqrt(distance*distance-(getHeight()-CamConstants.camera_Height_Meters)*(getHeight()-CamConstants.camera_Height_Meters))); //- CamConstants.robotToCamX);
          
    
  }

  /**
   * Modifies the position to one of the reef levels
   * Then calculates the distance to it
   * @param robotCurrentPose the current poisition of the robot
   * @param right if the robot will go to the right or left
   * @return the distance to the modified position
   */
  double getPoseToPose(Pose2d robotCurrentPose, boolean right){
    if(hasTargets){
      if(target.fiducialId == 17 || target.fiducialId == 11){
        if(right){
          poseTarget = truePose.toPose2d().plus(new Transform2d( -.1905,.1905, new Rotation2d(0)));
        } else poseTarget = truePose.toPose2d().plus(new Transform2d(.1905, -.1905, new Rotation2d(0)));
      return PhotonUtils.getDistanceToPose(robotCurrentPose, poseTarget);
      }
       else if(target.fiducialId == 18 || target.fiducialId == 10){
        if(right){
          poseTarget = truePose.toPose2d().plus(new Transform2d(0,-.1905, new Rotation2d(0)));
        } else poseTarget = truePose.toPose2d().plus(new Transform2d(0,.1905, new Rotation2d(0)));
      return PhotonUtils.getDistanceToPose(robotCurrentPose, poseTarget);
      }
      if(target.fiducialId == 19 || target.fiducialId == 9){
        if(right){
          poseTarget = truePose.toPose2d().plus(new Transform2d(.1905,.1905, new Rotation2d(0)));
        } else poseTarget = truePose.toPose2d().plus(new Transform2d(-.1905,-.1905, new Rotation2d(0)));
      return PhotonUtils.getDistanceToPose(robotCurrentPose, poseTarget);
      }
      else if(target.fiducialId == 20 || target.fiducialId == 8){
        if(right){
          poseTarget = truePose.toPose2d().plus(new Transform2d(-.1905,-.1905, new Rotation2d(0)));
        } else poseTarget = truePose.toPose2d().plus(new Transform2d(.1905,.1905, new Rotation2d(0)));
      return PhotonUtils.getDistanceToPose(robotCurrentPose, poseTarget);
      }
      else if(target.fiducialId == 21 || target.fiducialId == 7){
        if(right){
          poseTarget = truePose.toPose2d().plus(new Transform2d(0,.1905, new Rotation2d(0)));
        } else poseTarget = truePose.toPose2d().plus(new Transform2d(0,-.1905, new Rotation2d(0)));
      return PhotonUtils.getDistanceToPose(robotCurrentPose, poseTarget);
      }
      else if(target.fiducialId == 21 || target.fiducialId == 7){
        if(right){
          poseTarget = truePose.toPose2d().plus(new Transform2d(0,.1905, new Rotation2d(0)));
        } else poseTarget = truePose.toPose2d().plus(new Transform2d(0,-.1905, new Rotation2d(0)));
      return PhotonUtils.getDistanceToPose(robotCurrentPose, poseTarget);
      }
      else if(target.fiducialId == 22 || target.fiducialId == 6){
        if(right){
          poseTarget = truePose.toPose2d().plus(new Transform2d(.1905,.1905, new Rotation2d(0)));
        } else poseTarget = truePose.toPose2d().plus(new Transform2d(-.1905,-.1905, new Rotation2d(0)));
      return PhotonUtils.getDistanceToPose(robotCurrentPose, poseTarget);
      } 
      else return 999;
    }
    else return 999;
  }  
 /* public Optional<EstimatedRobotPose> update(){
    poseEstimate = photonPoseEstimator.update(getLatestResult());
    return poseEstimate;
  } */

  /**
   * gets the estimated position of the robot as a Pose3d
   * @return estimatedPose3d
   */
  public Pose3d getPose3d(){
    return estimatedPose3d;
  }
  
  PhotonTrackedTarget lowestAmbiguity(List<PhotonTrackedTarget> targets){
    PhotonTrackedTarget currentBest = null;
    double bestAmbiguity = 2000000000;
    for (PhotonTrackedTarget photonTrackedTarget : targets) {
      if (photonTrackedTarget.getPoseAmbiguity() < bestAmbiguity){
        currentBest = photonTrackedTarget;
        bestAmbiguity = photonTrackedTarget.getPoseAmbiguity();
        
        //System.out.println(photonTrackedTarget.getFiducialId() +": " + photonTrackedTarget.getPoseAmbiguity());
        
      }
    }
    return currentBest;
  }

  PhotonTrackedTarget getBestArea(List<PhotonTrackedTarget> targets){
    PhotonTrackedTarget currentBestArea = null;
    double bestArea = -2000000000;
    for (PhotonTrackedTarget photonTrackedTarget : targets) {
      if (photonTrackedTarget.getArea() > bestArea){
        currentBestArea = photonTrackedTarget;
        bestArea = photonTrackedTarget.getArea();
        
        //System.out.println(photonTrackedTarget.getFiducialId() +": " + photonTrackedTarget.getPoseAmbiguity());
        
      }
    }
    return currentBestArea;
  }
  

  /**
   * gets the boolean to determine if a pose is estimated
   * @return poseEstimated
   */
  public boolean isPoseEstimated(){
    return poseEstimated;
  }

  public PhotonTrackedTarget getResult(){
    return target;
  } 
  public Optional<Pose3d> getTagPose(Integer tagID){
    return aprilTagFieldLayout.getTagPose(tagID);
  }
  public boolean hasResult(){
    return hasTargets;
  }
 /* public EstimatedRobotPose getPoseTrue(){
  poseEstimateTrue = poseEstimate.orElse(poseEstimateTrue);
    return poseEstimateTrue;
  }
    */


  //public void showYaw() {
  //  SmartDashboard.putNumber("YE Yaw", target.getYaw());         IF DONT HAVE TARGET, DONT RUN SHOWYAW
  //}
  public double calculateXPose(int tagID){
    double tagAngle = getTagPose(tagID).get().getRotation().getAngle();
    double tagX = getTagPose(tagID).get().getX();
    return tagX + Units.inchesToMeters(38/2)*Math.cos(tagAngle);
  }
  public double calculateYPose(int tagID){
    double tagAngle = getTagPose(tagID).get().getRotation().getAngle();
    double tagY = getTagPose(tagID).get().getY();
    return tagY + Units.inchesToMeters(38/2)*Math.sin(tagAngle);
  }
  



  @Override
  public void periodic() {
    
    

    result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    if (hasTargets) {
      targets = result.getTargets();
      target = getBestArea(targets);
      /*if (Constants.DRIVE_AVAILABLE){
        photonPoseEstimator.addHeadingData(m_RobotContainer.getMatchTimer().matchTime(), m_RobotContainer.getDrivetrain().getHeading());
      }*/
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
      //SmartDashboard.putString("Robot Pose 2", photonPoseEstimator.update(result).toString());+

      
      reefTag = target.getFiducialId()>=17 || (target.getFiducialId()>=6 && target.getFiducialId()<=11);
      //SmartDashboard.putNumber("April Robot Pose X", getPose3d().getX());
      //SmartDashboard.putNumber("April Robot Pose Y", getPose3d().getY());
      SmartDashboard.putNumber("April Tag X", target.getFiducialId());
      //SmartDashboard.putNumber("Pitch", target.getPitch());
      //SmartDashboard.putNumber("Get Yaw", target.getYaw());
      //SmartDashboard.putNumber("Get Distance Inches ", Units.metersToInches(getDistanceToTarget()));
      
      

      // This method will be called once per scheduler run
    } else {
      reefTag = false;
      SmartDashboard.putNumber("April Tag X", 999);
      //SmartDashboard.putNumber("Get Yaw", 999.);
      //SmartDashboard.putNumber("Get Distance", 999.);
      poseEstimated = false;
    }
      SmartDashboard.putBoolean("Reef Tag?", reefTag);
  
  }
}
