// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HandlerConstants;

public class Handler extends SubsystemBase {
  SparkMax coralShoot;
  SparkMax algaePivot;
  RelativeEncoder algaeEncoder;
  SparkClosedLoopController algaeController;
  DigitalInput grabSensor;
  double[] currentHist = {0.,0.,0.,0.,0.};
  int currP = 0;
  double avgCurrent = 0;
  boolean pivotSaftey = true;
  /** Creates a new Handler. */
  public Handler() {
    coralShoot = new SparkMax(Constants.CANIDS.coralL, MotorType.kBrushless);
    algaePivot = new SparkMax(Constants.CANIDS.coralR, MotorType.kBrushless);
    algaeEncoder = algaePivot.getEncoder();
    algaeController = algaePivot.getClosedLoopController();
    SparkMaxConfig algaeConfig = new SparkMaxConfig(){};
      algaeConfig.closedLoop.pid(HandlerConstants.algaeP,
                                 HandlerConstants.algaeI, 
                                 HandlerConstants.algaeD);
      grabSensor = new DigitalInput(HandlerConstants.grabSensorPort);
  }    
  /*TODO: what kind of sensor might we want to help us control this subsystem 
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
  
  /** Stops the coral shooter */
  public void stop(){
    Shoot(0);
  }

  public void algaeGrab(){

  } 
  
  public double getAlgaeCurrent(){
    return algaePivot.getOutputCurrent();
  }

  @Override
  public void periodic() {
    double currentCurrent = getAlgaeCurrent();

    avgCurrent += currentCurrent/5. - currentHist[currP]/5.;
    currentHist[currP] = currentCurrent;
    currP = (currP+1)%5;

    if(avgCurrent>HandlerConstants.algaeCurrentLimit){
      pivotSaftey = false;
    }
  }
  

}
