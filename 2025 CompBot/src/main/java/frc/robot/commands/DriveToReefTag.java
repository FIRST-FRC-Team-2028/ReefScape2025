// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToReefTag extends Command {
  Drivetrain drive;
  AprilCamera camera;
  PIDController driveController;
  double yaw;
  double xSpeed = 0.7;
  PIDController turnController;
  double faceDiff, tagAngle, driveHeading;
  /** Drive toward the april tag */
  public DriveToReefTag(Drivetrain drive, AprilCamera camera) {
    this.drive = drive;
    this.camera = camera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive,camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (camera.hasResult()){
      if (camera.getResult().getFiducialId()>11){
        tagAngle = (Math.toDegrees(camera.getTagPose(camera.getResult().getFiducialId()).get().getRotation().getAngle())+180);
      } else {
        tagAngle = (Math.toDegrees(camera.getTagPose(camera.getResult().getFiducialId()).get().getRotation().getAngle()));
      }
    } else tagAngle = drive.getHeading().getDegrees()%360;
    turnController = new PIDController(0.06, 0, 0);
    driveController = new PIDController(0.06, 0, 0); // presume at most 30 degrees away in yaw
    // TODO: although the target x distance is not accurate, it may offer a hint of how fast to drive
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveHeading = (drive.getHeading().getDegrees());
    faceDiff = (tagAngle - driveHeading)%360;
    faceDiff = (faceDiff<-180)?faceDiff+360:faceDiff;
    faceDiff = (faceDiff>180)?faceDiff-360:faceDiff;
    faceDiff = (Math.abs(faceDiff)>180)?0.:faceDiff;

    yaw = camera.tagYaw();
    yaw = (yaw<-50.)?0.:yaw;
    xSpeed = (yaw<-50.)?0.:xSpeed;
    drive.driveComponent(xSpeed, driveController.calculate(yaw), -turnController.calculate(faceDiff));
    SmartDashboard.putNumber("yaw" ,yaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(yaw)<.3 && drive.getLoad() > 50. && Math.abs(faceDiff)<1.5;
  }
}
