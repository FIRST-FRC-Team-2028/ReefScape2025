// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Make the PID controller for a SparkMax accessible on LiveWindow */
public class SendableSparkPID 
   //extends SparkClosedLoopController
   implements Sendable
{
    SparkClosedLoopController controller;
    SparkMaxConfig mConfig;
    SendableSparkMax motor;
    double target;

    /** */
    public SendableSparkPID(SendableSparkMax motor){
    this.motor = motor;
    mConfig = new SparkMaxConfig();
    controller = motor.getClosedLoopController();
   }

   void p(double val){
    mConfig.closedLoop.p(val);
    motor.configure(mConfig, null, null);
   }
   void i(double val){
    mConfig.closedLoop.i(val);
     motor.configure(mConfig, null, null);
   }
   void d(double val){
    mConfig.closedLoop.d(val);
     motor.configure(mConfig, null, null);
   }
   void enable(boolean en){
    if(en)controller.setReference(target,ControlType.kPosition);
    else motor.stopMotor();
   }
   void sp(double val){     // this method does not ensure val is a valid set point
    target = val;
   }

   @Override
   public void initSendable(SendableBuilder builder){
       builder.setSmartDashboardType("PIDController");
       builder.addDoubleProperty("P", null, this::p);
       builder.addDoubleProperty("I", null, this::i);
       builder.addDoubleProperty("D", null, this::d);
       builder.addDoubleProperty("SetPoint", null, this::sp);
       builder.addBooleanProperty("Enable", null, this::enable);
   }
}
