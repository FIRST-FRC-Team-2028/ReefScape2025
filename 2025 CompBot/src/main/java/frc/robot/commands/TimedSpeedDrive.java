// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TimedSpeedDrive extends Command {
  private final Drivetrain drivesubsytem;
  private Timer timer;
  private ChassisSpeeds chassisSpeeds;
  double time;
  double xSpeed;
  double ySpeed;
  double turningSpeed;
  /** Drives the robot for a set amount of time in robot orient
   * @param time to dirve
   * @param xSpeed speed positive is forwards and negative backwards
   * @param ySpeed speed positive is left and negative is right
   * @param turningSpeed positive is CCW and negative is CW
  */
  public TimedSpeedDrive(Drivetrain drivetrain, double time, double xSpeed, double ySpeed, double turningSpeed) {
    drivesubsytem = drivetrain;
    timer = new Timer();
    this.time = time;
    this.xSpeed=xSpeed;
    this.ySpeed=ySpeed;
    this.turningSpeed = turningSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivesubsytem.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    drivesubsytem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
