// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Handler;


public class L1Shoot extends Command {
  Handler handler;
  Timer time;
  /** Uses the handler to shoot on L1 */
  public L1Shoot(Handler handler) {
    addRequirements(handler);
    time = new Timer();
    this.handler = handler;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    handler.lShoot(.5);
    handler.rShoot(.2);
    time.start();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
    handler.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.hasElapsed(5);
  }
}
