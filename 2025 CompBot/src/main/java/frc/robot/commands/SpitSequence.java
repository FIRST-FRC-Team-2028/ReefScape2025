// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Handler;


public class SpitSequence extends SequentialCommandGroup {
  /** Sequence to move elevator to position
   *  and spit coral out */
  public SpitSequence(Handler handlerSubsystem, Elevator elevatorSubsystem, double positionP, double positionE) {

    addCommands(new InstantCommand(() -> elevatorSubsystem.PIDController(positionE)),
                new InstantCommand(() -> handlerSubsystem.moveHandler(positionP)),
                new Spit(handlerSubsystem));
  }
}
