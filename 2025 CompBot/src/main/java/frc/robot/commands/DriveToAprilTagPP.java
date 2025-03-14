// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAprilTagPP extends Command {
  Drivetrain drivetrain;
  AprilCamera camera;
  Pose2d targetPose;
  Double x = 0.; 
  Double y = 0.;
  Rotation2d rotation2d;
  double rotation = 0;
  Integer tagID;
  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTagPP(Drivetrain drivetrain, AprilCamera camera) {
    this.drivetrain = drivetrain;
    this.camera = camera;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tagID = camera.getResult().getFiducialId();
    if (tagID != -1){ 
      x = camera.getTagPose(tagID).get().getX();
      y = camera.getTagPose(tagID).get().getY();
      rotation = camera.getTagPose(tagID).get().getRotation().getAngle();
      SmartDashboard.putNumber("tag X", x);
      SmartDashboard.putNumber("tag Y", y);
      SmartDashboard.putNumber("rotation", rotation);
      rotation2d = new Rotation2d(camera.getTagPose(tagID).get().getRotation().getAngle());
      //targetPose = new Pose2d(x, y, rotation);
   
      System.out.println(tagID);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.pathfindToPose(x, y, rotation2d, 0);
    
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (tagID==-1){
      return true;
    }
    return false;
  }
}
