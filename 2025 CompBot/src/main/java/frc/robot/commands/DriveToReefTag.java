// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToReefTag extends Command {
  Drivetrain drive;
  AprilCamera camera;
  PIDController controller;
  double yaw;
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
    controller = new PIDController(1./2., 0, 0); // presume at most 30 degrees away in yaw
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yaw = camera.tagYaw();
    yaw = (yaw>-50.)?yaw:0.;
    drive.driveComponent(.4,controller.calculate(yaw),0.);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(yaw)<.3 && drive.getLoad() > 15.;
  }
}
