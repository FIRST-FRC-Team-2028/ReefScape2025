// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  double smoothedXSpeed, smoothedYSpeed, smoothedTurningSpeed;
  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  Drivetrain drivetrain;
  /** Creates a new DriveCommand. */
  public DriveCommand(Drivetrain subsystem) {
    drivetrain = subsystem;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
     addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xLimiter.calculate(MathUtil.applyDeadband(-driverJoytick.getRawAxis(OIConstants.kDriverXAxis), OIConstants.kDeadband))*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond; // Negative values go forward
    double ySpeed = yLimiter.calculate(MathUtil.applyDeadband(-driverJoytick.getRawAxis(OIConstants.kDriverYAxis), OIConstants.kDeadband))*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // TODO limit acceleration as elevator is raised
    double turningSpeed = turningLimiter.calculate(MathUtil.applyDeadband(-driverJoytick.getRawAxis(OIConstants.kDriverRotAxis), OIConstants.kDeadband))*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    xSpeed *= 1. - (DriveConstants.kFineControlSpeed * driverJoytick.getRawAxis(OIConstants.kFineControlAxis))
                    + (DriveConstants.kFasterSpeed * driverJoytick.getRawAxis(OIConstants.kFastControlAxis));
    ySpeed *= 1. - (DriveConstants.kFineControlSpeed * driverJoytick.getRawAxis(OIConstants.kFineControlAxis))
                    + (DriveConstants.kFasterSpeed * driverJoytick.getRawAxis(OIConstants.kFastControlAxis));
    turningSpeed *= 1.
                    - (DriveConstants.kFineControlSpeed * driverJoytick.getRawAxis(OIConstants.kFineControlAxis))
                    + (DriveConstants.kFasterSpeed * driverJoytick.getRawAxis(OIConstants.kFastControlAxis));
    if(drivetrain.elevatorUp()){
    smoothedXSpeed = smoothedXSpeed + (xSpeed - smoothedXSpeed) * .04;
    smoothedYSpeed = smoothedYSpeed + (ySpeed - smoothedYSpeed) * .04;
    smoothedTurningSpeed = smoothedTurningSpeed + (turningSpeed - smoothedTurningSpeed) * .04;
    } else {
      smoothedXSpeed = smoothedXSpeed + (xSpeed - smoothedXSpeed) * .08;
      smoothedYSpeed = smoothedYSpeed + (ySpeed - smoothedYSpeed) * .08;
      smoothedTurningSpeed = smoothedTurningSpeed + (turningSpeed - smoothedTurningSpeed) * .08;
    }

    xSpeed = smoothedXSpeed;
    ySpeed = smoothedYSpeed;
    turningSpeed = smoothedTurningSpeed;
    
    ChassisSpeeds chassisSpeeds;
    
    if (!driverJoytick.getRawButton(OIConstants.kDriverRobotOrientedButton)) { //normal use
    // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                   xSpeed, ySpeed, turningSpeed, drivetrain.getRotation2d());
    } else {
    // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }
    drivetrain.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
