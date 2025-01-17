// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HandlerConstants;

public class Handler extends SubsystemBase {
  SparkMax coralShoot;
  SparkMax pivot;
  RelativeEncoder pivotEncoder;
  SparkClosedLoopController pivotController;
  DigitalInput grabSensor;
  double[] currentHist = {0.,0.,0.,0.,0.};
  int currP = 0;
  double avgCurrent = 0;
  boolean pivotSaftey = true;
  /** Creates a new Handler. */
  public Handler() {
    coralShoot = new SparkMax(Constants.CANIDS.coralL, MotorType.kBrushless);
    pivot = new SparkMax(Constants.CANIDS.coralR, MotorType.kBrushless);
    pivotEncoder = pivot.getEncoder();
    pivotController = pivot.getClosedLoopController();
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
   * @param outputSpeed  how hard to spew (in what units or what range)
  */
  public void Shoot(double outputSpeed){
    coralShoot.set(outputSpeed);
  }
  
  /** Stops the coral shooter */
  public void stop(){
    Shoot(0);
  }

  /** Pivots the handlers to the position to control an algae */
  public void grabPos(){
    if (pivotSaftey) {
      pivotController.setReference(HandlerConstants.usePos, ControlType.kPosition);
    }
  }
  /** Pivots the handlers to the position to score and grab coral 
   * 
  */
  public void restPos(){
    if (pivotSaftey) {
    pivotController.setReference(HandlerConstants.restPos, ControlType.kPosition);
    }
  }

  /** Runs the motor to grab 
   * @param outputSpeed how hard to spew (in what units or what range)
  */
  public void algaeGrab(double outputSpeed){
    Shoot(outputSpeed);

  }

  /** Gets the current of the motor that contols the handlers pivot
   * @return current of the Pivot motor
   */
  public double getAlgaeCurrent(){
    return pivot.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Algae Current limit", pivotSaftey);
    double currentCurrent = getAlgaeCurrent();

    avgCurrent += currentCurrent/5. - currentHist[currP]/5.;
    currentHist[currP] = currentCurrent;
    currP = (currP+1)%5;

    if(avgCurrent>HandlerConstants.algaeCurrentLimit){
      pivotSaftey = false;
    }
  }
  

}
