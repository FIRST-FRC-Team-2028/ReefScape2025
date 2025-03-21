// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SendableRelEncoder;
import frc.robot.Constants.HandlerConstants;
import frc.robot.SendableSparkMax;

public class Handler extends SubsystemBase {
  SendableSparkMax coralShoot;
  RelativeEncoder coralShootEncoder;
  SendableSparkMax pivot;
  RelativeEncoder pivotEncoder;
  SparkClosedLoopController pivotController;
  double latestTarget = 0;
  SendableRelEncoder msrep;
  SparkAnalogSensor grabsensor;
  double[] currentHist = {0.,0.,0.,0.,0.};
  int currP = 0;
  double avgCurrent = 0;
  boolean pivotSaftey = true;
  boolean doIHaveIt = false;
  SparkMaxConfig pivotConfig;

  double[] currentHista = {0.,0.,0.,0.,0.};
  int currPa = 0;
  double avgCurrenta = 0;
  boolean algaeCaptureCurrentLimit = true;
    /** Manipulates scoring elements: coral, and algae.
   * <p>Methods:<ul>
   * <li>intake - grab coral
   * <li>doIHaveIt - Spews doIHaveIt
   * <li>iHaveIt - sets doIHaveIt to true
   * <li>iDontHaveIt - set doIHaveIt to false
   * <li>moveHandlerSpeed - vbus control of pivot motor
   * <li>reTargetPivot - Nudges the PID target of the pivot motor
   * <li>Shoot - vbus control of shoot motor
   * <li>stop - Stops the algae and coral manipulator motor
   * <li>algaeGrab - vbus control of algae intake motor
   * <li>algaeShoot - vbus control to shoot algae
   * <li>getPivotCurrent - Gets the gets the current of the motor that controls the handlers pivot
   * <li>getAlgaeCurrent - Gets the current of the motor that controls the coral and algae manipulator
   * <li>rePivot - Resets the pivot safety to true
   * </ul>
   * </p> 
   */
  public Handler() {
    coralShoot = new SendableSparkMax(Constants.CANIDS.wheels, MotorType.kBrushless);
    coralShootEncoder = coralShoot.getEncoder();
    pivot = new SendableSparkMax(Constants.CANIDS.pivot, MotorType.kBrushless);
    pivotEncoder = pivot.getEncoder();
    pivotController = pivot.getClosedLoopController();
    grabsensor = coralShoot.getAnalog();
      pivotConfig = new SparkMaxConfig(){};
      pivotConfig.closedLoop.pid(HandlerConstants.pivotP,
                                 HandlerConstants.pivotI, 
                                 HandlerConstants.pivotD);
      pivotConfig.closedLoopRampRate(.5);
      pivotConfig.encoder.positionConversionFactor(HandlerConstants.pivotEncoderConversionFactor);
      pivotConfig.softLimit.reverseSoftLimit(HandlerConstants.reverseSoftLimit);
      pivotConfig.softLimit.reverseSoftLimitEnabled(true);
      pivotConfig.softLimit.forwardSoftLimitEnabled(true);
      pivotConfig.softLimit.forwardSoftLimit(HandlerConstants.forwardSoftLimit);
      pivotConfig.idleMode(IdleMode.kBrake);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig coralConfig = new SparkMaxConfig();
      coralConfig.inverted(true);
      coralConfig.idleMode(IdleMode.kBrake);
    coralShoot.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotEncoder.setPosition(0);
    addChild("Pivot", pivot);
    msrep = new SendableRelEncoder(pivotEncoder);
    addChild("Pivot Encoder", msrep);
    addChild("CoralShoot", coralShoot);
  }
  
  public void switchSL(boolean enabled){
    pivotConfig.softLimit.forwardSoftLimitEnabled(enabled);
    pivotConfig.softLimit.reverseSoftLimitEnabled(enabled);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, null);
    if (enabled) pivotEncoder.setPosition(0);
  }
  public void resetPivotEncoder(){
    pivotEncoder.setPosition(0);
  }

  /**
   * Runs the handler depending on the speed needed to grab the coral
   * @return true, If the handler has a coral ready to be scored
   */

    
  /**returns if the handler has a coral or not */
  public boolean doIHaveIt(){
    return doIHaveIt;
  }

  /** Tells the handler subsytem that it does NOT have a coral */
  public void iDontHaveIt(){
    doIHaveIt = false;
  }
  /**Tells handler subsytem that it does have a coral*/
  public void iHaveIt(){
    doIHaveIt=true;
  }  

  /**
  * vbus control control for the handler
  * @param hSpeed 
  */
  public void moveHandlerSpeed(double hSpeed){
    pivot.set(hSpeed);
  }

  /**Run Handler to  shoot coral
   * @param outputSpeed  how hard to spew (in what units or what range)
  */
  public void Shoot(double outputSpeed){
    coralShoot.set(outputSpeed);
  }
  
  /** Stops the algae and coral manipulator motor */
  public void stopWheels(){
    coralShoot.stopMotor();
  }

  /** Stops the Pivot motor */
  public void stopPivot(){
    pivot.stopMotor();
  }

  /** vbus control of algae intake motor*/
  /** Pivots the handler to set position using closed-loop controller
   * 
   */
  public void moveHandler(double position){
  //  if(pivotSaftey){
    pivotController.setReference(position, ControlType.kPosition);
    latestTarget = position;
 //   }
  }
  /** vbus control of algae intake motor */
  public void algaeGrab(){
    if(algaeCaptureCurrentLimit){
    Shoot(HandlerConstants.grabAlgaeSpeed);
    }else Shoot(HandlerConstants.algaeHoldSpeed);
  }

  /** vbus control to shoot algae */
  public void algaeShoot(){
    Shoot(HandlerConstants.algaeShootSpeed);
  }


  /**
   * Nudges the PID target of the pivot motor
   * @param adjustment PID nudge amount
   */
  public void reTargetPivot(double adjustment){
   // if (pivotSaftey){
      latestTarget += adjustment;
      pivotController.setReference(latestTarget, ControlType.kPosition);
   // }
  }

  /** Gets the current of the motor that contols the handlers pivot
   * @return current of the Pivot motor
   */
  public double getPivotCurrent(){
    return pivot.getOutputCurrent();
  }
  /**@return Position of the Pivot Motor in degrees */
  public double getPivotPostition(){
    return pivotEncoder.getPosition();
  }

  /** Gets the current of the motor that controls the coral and algae manipulator
   * @return current of the coral and algae manipulator
   */
  public double getAlgaeCurrent(){
    return coralShoot.getOutputCurrent();
  }
  
  /**Resets the pivot safety to true */
  public void rePivot(){
    pivotSaftey = true;
  }



  @Override
  public void periodic() {
 
    if (grabsensor.getPosition() > 2)
      iHaveIt();
    else iDontHaveIt();

    SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
    //SmartDashboard.putNumber("Algae Current", coralShoot.getOutputCurrent());
    //SmartDashboard.putNumber("Coral Temp", coralShoot.getMotorTemperature());
 //   SmartDashboard.putBoolean("Pivot Current limit", pivotSaftey);
    double currentCurrent = getPivotCurrent();

    avgCurrent += currentCurrent/5. - currentHist[currP]/5.;
    currentHist[currP] = currentCurrent;
    currP = (currP+1)%5;


    double currentCurrenta = getAlgaeCurrent();

    avgCurrenta += currentCurrenta/5. - currentHista[currPa]/5.;
    currentHista[currP] = currentCurrenta;
    currPa = (currPa+1)%5;

 /*   if(avgCurrent>HandlerConstants.pivotCurrentLimit){
      pivotSaftey = false;
    }
      */

    if(avgCurrenta>HandlerConstants.grabAlgaeCurrent){
      algaeCaptureCurrentLimit = false;
    }
  
  }
}
