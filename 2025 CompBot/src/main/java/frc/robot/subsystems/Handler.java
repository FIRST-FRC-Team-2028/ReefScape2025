// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Handler extends SubsystemBase {
  SparkMax coralL;
  SparkMax coralR;
  /** Creates a new Handler. */
  public Handler() {
    coralL = new SparkMax(Constants.CANIDS.coralL, MotorType.kBrushless);
    coralR = new SparkMax(Constants.CANIDS.coralR, MotorType.kBrushless);
    this.coralL = coralL;
    this.coralR = coralR;
  }

  public void lShoot(double leftOutputSpeed){
    coralL.set(leftOutputSpeed);
  }

  public void rShoot(double rightOutputSpeed){
    coralR.set(rightOutputSpeed);
  }
  
  public void stop(){
    lShoot(0);
    rShoot(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
