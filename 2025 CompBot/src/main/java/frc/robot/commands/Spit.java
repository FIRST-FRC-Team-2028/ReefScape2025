// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HandlerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Handler;


public class Spit extends Command {
  Handler handler;
  Elevator elevator;
  boolean L1;
  double position;
  Timer time;
  /** Uses the handler to shoot on L1 */
  public Spit(Handler handler) {
    addRequirements(handler);
    time = new Timer();
    this.handler = handler;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
    handler.Shoot(HandlerConstants.outputSpeed);
    

   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
    handler.stopWheels();
    handler.iDontHaveIt();
    time.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.hasElapsed(5);
  }
}
