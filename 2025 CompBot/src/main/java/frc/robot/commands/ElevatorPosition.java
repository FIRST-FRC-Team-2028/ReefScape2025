// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPosition extends Command {
  private final Elevator elevator;
  private double Destination;
 // private final double allowance = 0.15;
  /** closed loop set the elevator position.
   * @param Destination inches
   */
  public ElevatorPosition(Elevator elevator, double Destination) {
    this.elevator = elevator;
    this.Destination = Destination;
    addRequirements(elevator);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.PIDController(Destination);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //elevator.stopElevator();
  }

  // Returns true when position is close enough to target
  @Override
  public boolean isFinished() {
    return true; /*Destination - allowance < elevator.getPosition() 
           && elevator.getPosition() < Destination + allowance;*/
  }
}
