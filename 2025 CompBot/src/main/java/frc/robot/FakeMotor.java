// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class FakeMotor
implements Sendable {
    PIDController PID = new PIDController(1, .3, .04);
    Encoder encoder = new Encoder();
    double Speed = 0.0;
    double Target = 0.0;
    double PIDOutput = 0.0;

    public FakeMotor(int dum1, MotorType dum2){}
    public void disable( ){}
    public double get(){
        return 999.;
    }

    public PIDController getClosedLoopController(){
        return PID;
    }

    public Encoder getEncoder(){
        return encoder;
    }

    public void setReference(double target, ControlType dummy) {
        Target = target;
        PIDOutput = PID.calculate(encoder.getPosition(), Target);
        set(PIDOutput);
    }

    public void set(double Speed) {
      this.Speed = Speed;
    }
    public void stopMotor(){
        Speed = 0;
    }

    public void update(){
        encoder.increment(Speed);
    }

    public void periodic() {
      PIDOutput = PID.calculate(encoder.getPosition(), Target);
      if (PID.atSetpoint()) {
          stopMotor();
      } else {
          set(PIDOutput);
      }
      System.out.println(Target + " " + PIDOutput);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::disable);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    public class Encoder{
        double counter = 0.;
        Encoder(){}
        public void setPosition(double val){
            counter = val;
        }
        
        public double getPosition(){
            return counter;
        }
        void increment(double speed){
            counter += speed * .01 * ElevatorConstants.EncoderConversionFactor; 
        }
    }
}
