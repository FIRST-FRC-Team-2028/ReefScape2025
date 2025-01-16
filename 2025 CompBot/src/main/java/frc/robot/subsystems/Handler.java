// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Handler extends SubsystemBase {
  SparkMax coralShoot;
  SparkMax algaePivot;
  RelativeEncoder algaeEncoder;
  SparkClosedLoopController algaeController;
  /** Creates a new Handler. */
  public Handler() {
    coralShoot = new SparkMax(Constants.CANIDS.coralL, MotorType.kBrushless);
    algaePivot = new SparkMax(Constants.CANIDS.coralR, MotorType.kBrushless);
    algaeEncoder = algaePivot.getEncoder();
    algaeController = algaePivot.getClosedLoopController();
  }

  public void Shoot(double leftOutputSpeed){
    coralShoot.set(leftOutputSpeed);
  }
  
  
  public void stop(){
    Shoot(0);
  }

  public void algaeGrab(){

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
