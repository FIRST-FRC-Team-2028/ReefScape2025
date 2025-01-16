// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

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
    SparkMaxConfig algaeConfig = new SparkMaxConfig(){};
      algaeConfig.closedLoop.pid(1.0,1.0,1.0);
  }    /*TODO: what kind of sensor might we want to help us control this subsystem 
    when Human eyes are not good enough, or quick enough?
    ie have we acquired a coral or algae,
       have we shot/deployed the coral or algae
       is the handler in the appropriate position
       can we help aim?
       */
     
  //TODO: Please add javadocs

  /**Run Handler to  shoot coral
   * @param leftOutputSpeed  how hard to spew (in what units or what range)
  */
  public void Shoot(double leftOutputSpeed){
    coralShoot.set(leftOutputSpeed);
  }
  
  
  public void stop(){
    Shoot(0);
  }

  public void algaeGrab(){

  } @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
