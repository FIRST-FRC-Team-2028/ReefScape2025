// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.SendableRelEncoder;
import frc.robot.Constants;
import frc.robot.SendableSparkMax;
import frc.robot.SendableSparkPID;

public class Elevator extends SubsystemBase {
  
  private final SendableSparkMax m_elevatorMotorL;
  private final SendableSparkMax m_elevatorMotorR;
  private final RelativeEncoder m_elevatorEncoder;
  private final SendableRelEncoder msre;
  private final SparkClosedLoopController m_ClosedLoopController;
  private final SparkLimitSwitch m_rearLimitSwitch;
  private final SparkMaxConfig configL, configR;
 // private double Destination = 0;
  private double latestTarget = 3;
  double[] currentHist = {0.,0.,0.,0.,0.};
  int currP = 0;
  double avgCurrent = 0;
  boolean elevatorSaftey = true;

    /** Controls the height of the handler
     * <p>Methods:<ul>
     * <li>LSPressed - Gets if the limit switch is pressed
     * <li>resetLatestTarget - Resets the PID target to the current elevator position
     * <li>PIDController - Closed loop control of the elevator
     * <li>setElevatorSpeed - vbus control of the elevator
     * <li>stopElevator - stops the elevator motor
     * <li>getPosition - Gets the current position of the elevator
     * <li>setPosition - Set the position of the elevator encoder
     * <li>reTargetElevator - Adjusts the PID target on the elevator by a set value
     * <li>switchSL - disables/enables Softlimits on elevator motors, resets position to reverse SL
     */
    public Elevator() {
      m_elevatorMotorL = new SendableSparkMax(Constants.CANIDS.elevatorL, MotorType.kBrushless);
      m_elevatorMotorR = new SendableSparkMax(Constants.CANIDS.elevatorR, MotorType.kBrushless);
      m_elevatorEncoder = m_elevatorMotorL.getEncoder();
      m_ClosedLoopController = m_elevatorMotorL.getClosedLoopController();
      m_rearLimitSwitch = m_elevatorMotorL.getReverseLimitSwitch();
      configL = new SparkMaxConfig();
      configR = new SparkMaxConfig();
  
      configL.idleMode(IdleMode.kBrake)
            .inverted(false);
      configL.encoder.positionConversionFactor(ElevatorConstants.encoderConversionFactor);
      configL.softLimit.forwardSoftLimit(ElevatorConstants.softLimitForward)
                      .forwardSoftLimitEnabled(true)
                      .reverseSoftLimit(ElevatorConstants.softLimitReverse)
                      .reverseSoftLimitEnabled(true);
      configL.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
      //configL.closedLoop.velocityFF(ElevatorConstants.kFF);
      configR.follow(Constants.CANIDS.elevatorL, true);
      configL.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen)
                         .reverseLimitSwitchEnabled(true);
      m_elevatorMotorL.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_elevatorMotorR.configure(configR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
      m_elevatorEncoder.setPosition(3);
  
      addChild("Left (Leader)", m_elevatorMotorL);
      msre = new SendableRelEncoder(m_elevatorEncoder);
      addChild("Position", msre);
      addChild("PID", new SendableSparkPID(m_elevatorMotorL));
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
     if(m_rearLimitSwitch.isPressed()){
       m_elevatorEncoder.setPosition(3);
     }
      SmartDashboard.putNumber("Position", m_elevatorEncoder.getPosition());
      SmartDashboard.putNumber("Elevator Current L", m_elevatorMotorL.getOutputCurrent());
      SmartDashboard.putNumber("Elevator Temp L", m_elevatorMotorL.getMotorTemperature());
      SmartDashboard.putNumber("Elevator Temp R", m_elevatorMotorR.getMotorTemperature());
      

     double currentCurrent = m_elevatorMotorL.getOutputCurrent();
      avgCurrent += currentCurrent/5. - currentHist[currP]/5.;
    currentHist[currP] = currentCurrent;
    currP = (currP+1)%5;
    //System.out.println("Position: " + CurrentPosition);
  }

  /**
   * Gets if the limit switch is pressed
   * @return true if the limit switch is pressed
   */
  public boolean LSPressed(){
    return m_rearLimitSwitch.isPressed();
  }
  /** Resets the PID target to the current elevator position */
  public void resetLatestTarget(){
    latestTarget = getPosition(); 
  }

  /**Closed loop control of the elevator
   * 
   * @param target is the desired position (inches) for the elevator
   */
  public void PIDController(double target) {
    m_ClosedLoopController.setReference(target, ControlType.kPosition);
    latestTarget = target;
  }

  /** Run the elevator motor.
   * @param speed sets motor speed. Variable between -1, 1. Positive is up and negative is down.
  */
  public void setElevatorSpeed(double speed) {
    m_elevatorMotorL.set(speed);
  }

  /** Stop the elevator motor. */
  public void stopElevator() {
    m_elevatorMotorL.stopMotor();
  }

  /**Open-loop control of the elevator motor  
   * @param Tim is the target height
  */
  /*public void Vroom(double Tim) {
    Destination = Tim;
    m_elevatorMotorL.set(Math.copySign(.1, (Destination - CurrentPosition)));
  }*/

  /**Check whether elevator has attained the target height. Stop if it has. TODO NOT TRUE
   * 
   * @param Destination: Target Height
   * @return <b>True</b> if the motor is at the target height
   * <li><b>False</b> if the motor is not at the target height </li>
   */
 /* public boolean Finished(double Destination) {
    return getPosition() == Destination;
  }*/

  /**
   * Gets the current position of the elevator 
   * @return the current position of the elevator based on the encoder
   */
  public double getPosition(){
    return m_elevatorEncoder.getPosition();
  }

  /**
   * Set the position of the elevator encoder
   * @param newPose the new pose of the elevator encoder
   */
  public void setPosition(double newPose){
    m_elevatorEncoder.setPosition(newPose);
  }

  /**
   * Adjusts the PID target on the elevator by a set value
   * @param adjustment ammount to move the elevator
   */
  public void reTargetElevator(double adjustment){
    latestTarget += adjustment;
    PIDController(latestTarget);
    //m_ClosedLoopController.setReference(latestTarget, ControlType.kPosition);

  }

  /**disables/enables Softlimits on elevator motors, resets position to reverse SL
   * @param enabled whether or not the soft limits are enabled
  */
  public void switchSL(boolean enabled, boolean setPosition3){
    configL.softLimit.reverseSoftLimitEnabled(enabled);
    configL.softLimit.forwardSoftLimitEnabled(enabled);
    m_elevatorMotorL.configure(configL, ResetMode.kResetSafeParameters, null);    //Persist mode Null because can't persist while enabled
                                        // TODO: MrG asks do you want to override all original configs or just the SL?
    if (setPosition3) m_elevatorEncoder.setPosition(3.);
  }
  
}