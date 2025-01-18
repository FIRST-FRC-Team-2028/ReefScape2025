// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
  double latestTarget;
  double[] currentHist = {0.,0.,0.,0.,0.};
  int currP = 0;
  double avgCurrent = 0;
  boolean pivotSaftey = true;

  double[] currentHista = {0.,0.,0.,0.,0.};
  int currPa = 0;
  double avgCurrenta = 0;
  boolean algaeCaptureCurrentLimit = true;
  /** Creates a new Handler. */
  public Handler() {
    coralShoot = new SparkMax(Constants.CANIDS.coralL, MotorType.kBrushless);
    pivot = new SparkMax(Constants.CANIDS.coralR, MotorType.kBrushless);
    pivotEncoder = pivot.getEncoder();
    pivotController = pivot.getClosedLoopController();
    grabSensor = new DigitalInput(HandlerConstants.grabSensorPort);
    SparkMaxConfig pivotConfig = new SparkMaxConfig(){};
      pivotConfig.closedLoop.pid(HandlerConstants.pivotP,
                                 HandlerConstants.pivotI, 
                                 HandlerConstants.pivotD);
      pivotConfig.encoder.positionConversionFactor(360);
      pivotConfig.softLimit.reverseSoftLimit(HandlerConstants.reverseSoftLimit);
      pivotConfig.softLimit.forwardSoftLimit(HandlerConstants.forwardSoftLimit);
      pivotConfig.idleMode(IdleMode.kBrake);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }    

     

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

  /** Pivots the handler to set position
   * 
   */
  public void moveHandler(double position){
    if(pivotSaftey){
    pivotController.setReference(position, ControlType.kPosition);
    }
  }

  /** Runs the motor to grab 
   * @param outputSpeed how hard to spew (in what units or what range)
  */
  public void algaeGrab(){
    if(algaeCaptureCurrentLimit){
    Shoot(HandlerConstants.grabAlgaeSpeed);
    }else Shoot(HandlerConstants.algaeHoldSpeed);
  }

  public void algaeShoot(){
    Shoot(HandlerConstants.algaeShootSpeed);
  }

  public void targetPivot(double target){
    if(pivotSaftey){
      pivotController.setReference(target, ControlType.kPosition);
      latestTarget = target;
    }
  }

  public void reTargetPivot(double adjustment){
    if (pivotSaftey){
      latestTarget += adjustment;
      pivotController.setReference(latestTarget, ControlType.kPosition);
    }
  }
  /** Gets the current of the motor that contols the handlers pivot
   * @return current of the Pivot motor
   */
  public double getPivotCurrent(){
    return pivot.getOutputCurrent();
  }

  public double getAlgaeCurrent(){
    return coralShoot.getOutputCurrent();
  }

  public void rePivot(){
    pivotSaftey = true;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Pivot Current limit", pivotSaftey);
    double currentCurrent = getPivotCurrent();

    avgCurrent += currentCurrent/5. - currentHist[currP]/5.;
    currentHist[currP] = currentCurrent;
    currP = (currP+1)%5;


    double currentCurrenta = getAlgaeCurrent();

    avgCurrenta += currentCurrenta/5. - currentHista[currPa]/5.;
    currentHista[currP] = currentCurrenta;
    currPa = (currPa+1)%5;

    if(avgCurrent>HandlerConstants.pivotCurrentLimit){
      pivotSaftey = false;
    }

    if(avgCurrenta>HandlerConstants.grabAlgaeCurrent){
      algaeCaptureCurrentLimit = false;
    }
  
  }
}
