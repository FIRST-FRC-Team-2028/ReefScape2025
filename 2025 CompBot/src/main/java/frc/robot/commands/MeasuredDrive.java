// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MeasuredDrive extends Command {
  Drivetrain drive;
  double xDist,yDist,rot,mag;
  boolean robotOriented;
  double targetDistance, drivenDistance, initialDist;
  PIDController controller;
  final static double CLOSE_ENOUGH = .5;  //inches^2
  /** Drive a specified distance. 
   * @param x distance forward in inches
   * @param y distance to the right in inches (optional, 0. by default)
   * @param rot degrees clockwise around up vector (optional, 0. by default)
   * @param robotOriented (optional, true by default)
  */
  public MeasuredDrive(Drivetrain drive, double x, double y, double rot, boolean robotOriented) {
    xDist = x;
    yDist = y;
    this.rot = rot;
    this.drive = drive;
    this.robotOriented = robotOriented;
    addRequirements(drive);
  }
  public MeasuredDrive(Drivetrain drive, double x){
    this(drive, x, 0., 0.,true);
  }
  public MeasuredDrive(Drivetrain drive, double x,double y){
    this(drive, x, y, 0.,true);
  }
  public MeasuredDrive(Drivetrain drive, double x,double y,boolean ro){
    this(drive, x, y, 0.,ro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetDistance = xDist*xDist+yDist*yDist;  // actually this is the square of it
    initialDist = drive.getModulePositions()[0].distanceMeters;
    controller = new PIDController(2.0/targetDistance/targetDistance, 0.,0.);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivenDistance = Units.metersToInches(drive.getModulePositions()[0].distanceMeters - initialDist);
    mag = controller.calculate(drivenDistance*drivenDistance, targetDistance);
    drive.driveComponent(mag*xDist, mag*yDist);
    SmartDashboard.putNumber("Magnitude*xDist", mag*yDist);
    SmartDashboard.putNumber("Magnitude", mag);
    SmartDashboard.putNumber("xDist", yDist);
    SmartDashboard.putNumber("Error Measured Drive", targetDistance-drivenDistance*drivenDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return Math.abs(targetDistance-drivenDistance)<CLOSE_ENOUGH*CLOSE_ENOUGH || mag <0.007;
  }
}
