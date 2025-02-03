// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
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
import frc.robot.SendableSparkMax;
import frc.robot.Constants.HandlerConstants;
import frc.robot.SendableRelEncoder;

public class Handler extends SubsystemBase {
  SendableSparkMax coralShoot;
  SendableSparkMax pivot;
  RelativeEncoder pivotEncoder;
  SparkClosedLoopController pivotController;
  DigitalInput grabSensor;
  SendableRelEncoder msrep;
  double latestTarget;
  double[] currentHist = {0.,0.,0.,0.,0.};
  int currP = 0;
  double avgCurrent = 0;
  boolean pivotSaftey = true;
  boolean doIHaveIt = false;

  double[] currentHista = {0.,0.,0.,0.,0.};
  int currPa = 0;
  double avgCurrenta = 0;
  boolean algaeCaptureCurrentLimit = true;
  /** Creates a new Handler. */
  public Handler() {
    coralShoot = new SendableSparkMax(Constants.CANIDS.wheels, MotorType.kBrushless);
    pivot = new SendableSparkMax(Constants.CANIDS.pivot, MotorType.kBrushless);
    pivotEncoder = pivot.getEncoder();
    pivotController = pivot.getClosedLoopController();
    grabSensor = new DigitalInput(HandlerConstants.grabSensorPort);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
      pivotConfig.closedLoop.pid(HandlerConstants.pivotP,
                                 HandlerConstants.pivotI, 
                                 HandlerConstants.pivotD);
      pivotConfig.encoder.positionConversionFactor(HandlerConstants.pivotEncoderConversionFactor);
      pivotConfig.softLimit.reverseSoftLimit(HandlerConstants.reverseSoftLimit);
      pivotConfig.softLimit.forwardSoftLimit(HandlerConstants.forwardSoftLimit);
      pivotConfig.idleMode(IdleMode.kBrake);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    addChild("Pivot", pivot);
    msrep = new SendableRelEncoder(pivotEncoder);
    addChild("Pivot Encoder", msrep);
    addChild("OutPut", coralShoot);
  }    

  /**
   * Runs the handler depending on the speed needed to grab the coral
   * @return true, If the handler has a coral ready to be scored
   */
  public boolean intake(){
    double speed;
    speed = HandlerConstants.grabCoralSpeed;
    // Runs the handler spitting motor until the sensor detects that it has a coral.
    // Then it switches to a lower speed until the other sensor detects that it has cleared the passive loader.
    if(grabSensor.get()){
      speed = HandlerConstants.grabCoralSpeed/2;
    }

    /* if(clearSensor.get()){
      speed = 0
      doIHaveIt = true;
    }
    */
    
    Shoot(speed);
    return doIHaveIt;
  }


  public boolean doIHaveIt(){
    return doIHaveIt;
  }

  public void iHaveIt(){
    doIHaveIt = true;
  }

  public void iDontHaveIt(){
    doIHaveIt = false;
  }

  public void moveHandlerSpeed(double hSpeed){
    pivot.set(hSpeed);
  }

  /**Run Handler to  shoot coral
   * @param outputSpeed  how hard to spew -1 to 1
  */
  public void Shoot(double outputSpeed){
    coralShoot.set(outputSpeed);
  }
  
  /** Stops the coral shooter */
  public void stopWheels(){
    coralShoot.stopMotor();
  }

  public void stopPivot(){
    pivot.stopMotor();
  }

  /** Pivots the handler to set position using closed-loop controller
   * 
   */
  public void moveHandler(double position){
    if(pivotSaftey){
    pivotController.setReference(position, ControlType.kPosition);
    }
  }

  /** Runs the motor to grab an algae*/
  public void algaeGrab(){
    if(algaeCaptureCurrentLimit){
    Shoot(HandlerConstants.grabAlgaeSpeed);
    }else Shoot(HandlerConstants.algaeHoldSpeed);
  }

  public void algaeShoot(){
    Shoot(HandlerConstants.algaeShootSpeed);
  }

  /** pivot the handler to position in closed loop 
   * @param target degrees
  */
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
/**get postition of the pivot encoder */
  public double getPivotPostition(){
    return pivotEncoder.getPosition();
  }
  
  /**Resets the pivot safety to true */
  public void rePivot(){
    pivotSaftey = true;
  }

  /** periodic constantly:
   * <ul>
   * <li> checks the sensor to determine if a coral is present
   *  and sets the state variable doIHaveIt accordingly;
   *  <li> monitors coralshoot motor current as a means to determine if an algae has been acquired;
   *  <li> monitors pivot motor current
   * </ul>
   */
  @Override
  public void periodic() {
    if (grabSensor.get()){
      iHaveIt();
    } else iDontHaveIt();
    SmartDashboard.putBoolean("Proximity Sensor", doIHaveIt());
    /*SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
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
    }*/
  
  }
}
