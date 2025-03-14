// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Handler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HandlerPosition extends Command {
  private final Handler handler;
  private final double target;
  double allowance;
  /** Pivot the Handler to position
   * @param target
   */
  public HandlerPosition(Handler handler, double target) {
    this.handler = handler;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    handler.moveHandler(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //handler.stopPivot();
  }

  // Returns true when position is close enough to target.
  @Override
  public boolean isFinished() {
    return true;
    /*return target - allowance < handler.getPivotPostition()       
           && target + allowance > handler.getPivotPostition();*/
  }
}
