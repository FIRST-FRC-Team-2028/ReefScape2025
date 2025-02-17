// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HandlerConstants;
import frc.robot.subsystems.Handler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunWheels extends Command {
  private final Handler handler;
  private final double speed;
  private final double runTime;
  private final boolean algae;
  Timer timer;
  
  /**@param handler Handler subsystem.
   * @param speed
   * Speed between -1 and 1.
   * Positive speed is input/output for coral and intake for algae.
   * Negative speed is output for algae
   * @param extraTime time after aquisition to continue running motor
   * @param algae true for algae, false for coral
   */
  public RunWheels(Handler handler, double speed, double extraTime, boolean algae) {
    this.handler = handler;
    this.speed = speed;
    this.runTime = extraTime;
    this.algae = algae;
    timer = new Timer();
    addRequirements(handler);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    handler.Shoot(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(algae && speed < 0){
      handler.Shoot(HandlerConstants.algaeHoldSpeed);
    } else handler.stopWheels();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (handler.doIHaveIt()){
      timer.start();
    }

    if (algae) {
      return false;
    }else {
      return handler.doIHaveIt() && timer.hasElapsed(runTime);
    }
  }
}
