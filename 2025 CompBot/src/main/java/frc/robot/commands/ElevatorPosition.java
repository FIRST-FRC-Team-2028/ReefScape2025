// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPosition extends Command {
  private final Elevator elevator;
  private final double LeftDestination;
  private final double RightDestination;
  /** Drive elevator to a desired position - closed loop */
  public ElevatorPosition(Elevator elevator, double LeftDestination, double RightDestination) {
    this.elevator = elevator;
    this.LeftDestination = LeftDestination;
    this.RightDestination = RightDestination;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.PIDleft(LeftDestination);
    elevator.PIDright(RightDestination);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.Finished(LeftDestination, RightDestination);
  }
}
