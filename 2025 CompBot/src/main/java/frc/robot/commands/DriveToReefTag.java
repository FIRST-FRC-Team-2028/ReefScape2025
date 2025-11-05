// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CamConstants;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToReefTag extends Command {
  Drivetrain drive;
  AprilCamera camera;
  PIDController YdriveController, XdriveController;
  double yaw, pitch;
  double xSpeed = .5, tagLostCount = 0, ySpeed = 0.25;
  PIDController turnController;
  double faceDiff, tagAngle, driveHeading;
  Integer tagID = -1;
  Timer timer;
  boolean hasTagAtStart = false;
  double firstFaceDiff;
  /** Drive toward the april tag */
  public DriveToReefTag(Drivetrain drive, AprilCamera camera) {
    this.drive = drive;
    this.camera = camera;
    turnController = new PIDController(0.09, 0, 0);
    YdriveController = new PIDController(0.06, 0, 0); // presume at most 30 degrees away in yaw
    XdriveController = new PIDController(0.09, 0., 0.); // kp:0.08 //kp:0.06
    // TODO: although the target x distance is not accurate, it may offer a hint of how fast to drive
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive,camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (camera.hasResult()){
      hasTagAtStart=true;
      tagID = camera.getResult().getFiducialId();
      if (tagID>11){
        tagAngle = (Math.toDegrees(camera.getTagPose(camera.getResult().getFiducialId()).get().getRotation().getAngle())+180);
      } else {
        tagAngle = (Math.toDegrees(camera.getTagPose(camera.getResult().getFiducialId()).get().getRotation().getAngle()));
      }
      driveHeading = (drive.getHeading().getDegrees());
      faceDiff = (tagAngle - driveHeading)%360;
      faceDiff = (faceDiff<-180)?faceDiff+360:faceDiff;
      faceDiff = (faceDiff>180)?faceDiff-360:faceDiff;
      faceDiff = (Math.abs(faceDiff)>180)?0.:faceDiff;
      firstFaceDiff = faceDiff;
      SmartDashboard.putNumber("first Face Difference", firstFaceDiff);
    } else tagAngle = drive.getHeading().getDegrees()%360;
    
    


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    driveHeading = (drive.getHeading().getDegrees());
    faceDiff = (tagAngle - driveHeading)%360;
    faceDiff = (faceDiff<-180)?faceDiff+360:faceDiff;
    faceDiff = (faceDiff>180)?faceDiff-360:faceDiff;
    faceDiff = (Math.abs(faceDiff)>180)?0.:faceDiff;


    pitch = (pitch<-50)?8.:pitch;

    //xSpeed = (pitch>7)?0.:xSpeed;
    //System.out.println(xSpeed);
    //xSpeed = (pitch<-50.)?0.:xSpeed;
    //xSpeed = (yaw<-50)?0.:xSpeed;
    if(camera.hasResult()){
      yaw = camera.tagYaw();
      pitch = camera.tagPitch();
      tagLostCount = 0;
      xSpeed = XdriveController.calculate(pitch-.25);//-4.7); //6.25
      //drive.driveComponent(xSpeed, YdriveController.calculate(yaw), -turnController.calculate(faceDiff));
      drive.driveComponent(xSpeed, YdriveController.calculate(yaw), -turnController.calculate(faceDiff));
    }
    if (!camera.hasResult()){    //Checking when pitch>4 ensures we coast into the reef instead of driving into it
      tagLostCount +=1;
      if(yaw !=0){
        ySpeed = (yaw<0)?-0.25:.25;
      }
      xSpeed = 0.; //.6
      yaw = (yaw<-50.)?0.:yaw;
      ySpeed= -Math.copySign(.5, firstFaceDiff);
    drive.driveComponent(xSpeed, ySpeed, -turnController.calculate(faceDiff));
  }
  pitch = (xSpeed<0.01)?6.0:pitch; //0.0095 6.25
    //drive.driveComponent(0, driveController.calculate(yaw), -turnController.calculate(faceDiff));
    SmartDashboard.putNumber("yaw", yaw);
    SmartDashboard.putNumber("faceDiff", faceDiff);
    SmartDashboard.putNumber("pitch", pitch);
    //SmartDashboard.putNumber("tagLostCount", tagLostCount);
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tagLostCount = 0;
    /*if (!interrupted && tagID!=-1){
      Rotation2d heading = new Rotation2d(Math.toRadians((tagID>11)?(tagAngle+180):tagAngle));
      Pose2d resetPose = new Pose2d(camera.calculateXPose(tagID), camera.calculateYPose(tagID), heading);
      drive.resetPoseEstimatorPose(resetPose);
    }*/
  }

  // Returns true when the command should end.    getDistance()
  @Override
  public boolean isFinished() {
    return Math.abs(yaw)<.4 && pitch>=-.25 && Math.abs(faceDiff)<2.;
    
    //return Math.abs(yaw)<.3 && drive.getDistance()<16. && Math.abs(faceDiff)<2.;
    /*if (tagLostCount < 25.){
    return Math.abs(yaw)<.4 && pitch>=-.25 && Math.abs(faceDiff)<2.; //Math.abs(yaw)<.3 pitch>=6.25
    } else return true;*/   //if never started with tag end command
    //return Math.abs(yaw)<.3 && drive.getLoad() > 50. && Math.abs(faceDiff)<2.;
  }
}