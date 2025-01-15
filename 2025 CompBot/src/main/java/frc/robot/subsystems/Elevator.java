// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
  private final SparkMax m_elevatorMotor;
  private final RelativeEncoder m_elevatorEncoder;
  private double CurrentPosition = 0.0;
  private double Destination = 0;
  private double Vector = 0;

  public Elevator() {
    m_elevatorMotor = new SparkMax(41, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
    m_elevatorEncoder.setPosition(0.0);

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

  public void setElevatorSpeed(double speed) {
    m_elevatorMotor.set(speed);
  }
  public void startElevator() {
    m_elevatorMotor.set(.1);
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