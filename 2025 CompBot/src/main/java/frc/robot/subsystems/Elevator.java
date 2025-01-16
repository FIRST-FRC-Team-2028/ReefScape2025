// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
  private final SparkMax m_elevatorMotor;
  private final RelativeEncoder m_elevatorEncoder;
  private final SparkClosedLoopController m_ClosedLoopController;
  private double CurrentPosition = 0.0;
  private double Destination = 0;
  private double Vector = 0;

  public Elevator() {
    m_elevatorMotor = new SparkMax(57, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
    m_elevatorEncoder.setPosition(0.0);
    m_ClosedLoopController = m_elevatorMotor.getClosedLoopController();
    m_ClosedLoopController.setReference(5, ControlType.kPosition);
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop
        .p(1)
        .i(0)
        .d(0)
        .outputRange(0, 1);
  }

  @Override
  public void periodic() {
    CurrentPosition = m_elevatorEncoder.getPosition();
    SmartDashboard.putNumber("Position", CurrentPosition);
    System.out.println("To Stop: " + ((Destination - CurrentPosition) * (Vector)));
    System.out.println("Vector: " + Vector);
    Finished(Destination);
    // This method will be called once per scheduler run
  }


  public void PIDController() {
    
  }
  public void setElevatorSpeed(double speed) {
    m_elevatorMotor.set(speed);
  }
  public void stopElevator() {
    m_elevatorMotor.stopMotor();
  }
  public void Vroom(double Tim) {
    Destination = Tim;
    Vector = Math.copySign(1, (Destination - CurrentPosition));
    m_elevatorMotor.set(Math.copySign(.1, (Destination - CurrentPosition)));
  }
  public boolean Finished(double Destination) {
    if (((Destination - CurrentPosition) * (Vector)) <= 0) {
      stopElevator();
      return true;
    }
    return false;
  }
}