// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HandlerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Handler;


public class SpitSequence extends SequentialCommandGroup {
  /** Sequence to move to position to spit out */
  public SpitSequence(Handler handlerSubsystem, Elevator elevatorSubsystem, double positionP, double positionE) {

    addCommands(  
                  Commands.parallel(
                    new ElevatorPosition(elevatorSubsystem, positionE),
                    new HandlerPosition(handlerSubsystem, positionP)    
                    ),
                  new RunWheels(handlerSubsystem, HandlerConstants.outputSpeed, 1, false));
  }
}
