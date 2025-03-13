// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToReef extends Command {
  Drivetrain drive;
  AprilCamera april;
  PIDController controller;
  double faceDiff;
  /** Align robot x-axis with the facing reef april tag */
  public TurnToReef(Drivetrain drive, AprilCamera april) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, april);
    this.drive = drive;
    this.april = april;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new PIDController(0.3, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    faceDiff=april.getFaceVec()-180.;
    faceDiff= (faceDiff<-40.)?0.:faceDiff;
    drive.driveComponent(0.,0.,faceDiff);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(faceDiff)<.1;
  }
}
