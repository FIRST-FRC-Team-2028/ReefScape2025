// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Make the encoder for a SparkMax visible on LiveWindow */
public class SendableRelEncoder
implements RelativeEncoder, Sendable 
{
    RelativeEncoder self;
    public SendableRelEncoder(RelativeEncoder self){
        this.self = self;
    }
    public REVLibError setPosition(double pos){
        return null;
    }
    public double getPosition(){
        return self.getPosition();
    }
    public double getVelocity(){
        return self.getVelocity();
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Encoder");
        builder.addDoubleProperty("Speed", this::getVelocity, null);
        builder.addDoubleProperty("Distance", this::getPosition, this::setPosition);
    }
}
