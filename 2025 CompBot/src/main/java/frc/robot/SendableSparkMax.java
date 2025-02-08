// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.spark.SparkMax;


/** Make a SparkMax that can be displayed in the LiveWindow*/
public class SendableSparkMax 
extends SparkMax
implements Sendable
{
    /** create a new Sendable object for controlling a SparkMax Motor Controller */
    public SendableSparkMax(int id, MotorType motorType){
        super(id, motorType);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::disable);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    /* How do I make the associated encoder and closedloopcontroller also sendable
    class SendableEncoder 
    extends RelativeEncoder
    implements Sendable
    {
      SendableEncoder(){}
  
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        //builder.setSafeState(this::disable);
        builder.addDoubleProperty("Value", motor.getEncoder()::getPosition, this::setPosition);
      }
    } */
}
