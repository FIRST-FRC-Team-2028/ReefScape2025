// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.MediaSize.Engineering;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;
import frc.robot.SendableRelEncoder;
import frc.robot.SendableSparkMax;

public class Elevator extends SubsystemBase {
  
  private final SendableSparkMax m_elevatorMotorL;
  private final SendableSparkMax m_elevatorMotorR;
  private final RelativeEncoder m_elevatorEncoder;
  private final SendableRelEncoder msre;
  private final SparkClosedLoopController m_ClosedLoopController;
  private double CurrentPosition = 0.0;
  private double Destination = 0;
  private final double closeEnough = .2;
  private double speedOL = 0.;
  private SparkMaxConfig configL, configR;
  private final DigitalInput bottomSwitch;
  private double latestTarget;

  /**consists of two motors of which one is the leader.
   * <p>Lifts the handler up. Methods:
   *   <li>  PIDController - closedloop control to destination
   *   <li>  setElevatorSpeed - vbus control
   *   <li>  Vroom - open-loop to destination
   */
  public Elevator() {
    m_elevatorMotorL = new SendableSparkMax(Constants.CANIDS.elevatorL, MotorType.kBrushless);
    m_elevatorMotorR = new SendableSparkMax(Constants.CANIDS.elevatorR, MotorType.kBrushless);
    //m_elevatorMotorL = new SparkMax(Constants.CANIDS.elevatorL, MotorType.kBrushless);
    //m_elevatorMotorR = new SparkMax(Constants.CANIDS.elevatorR, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorMotorL.getEncoder();
    m_ClosedLoopController = m_elevatorMotorL.getClosedLoopController();
    configL = new SparkMaxConfig();
    configR = new SparkMaxConfig();

    configL.idleMode(IdleMode.kBrake)
          .inverted(false);
    configL.encoder.positionConversionFactor(ElevatorConstants.ENCODERCONVERSION);
    configL.softLimit.forwardSoftLimit(ElevatorConstants.SOFTLIMITFORWARD)
                    .forwardSoftLimitEnabled(false)
                    .reverseSoftLimit(ElevatorConstants.SOFTLIMITREVERSE)
                    .reverseSoftLimitEnabled(false);
    configL.closedLoop.pid(1.0, 0.0, 0.0);
    configR.follow(Constants.CANIDS.elevatorL, true);

    m_elevatorMotorL.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotorR.configure(configR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_elevatorEncoder.setPosition(0.);

    addChild("Left (Leader)", m_elevatorMotorL);
    msre = new SendableRelEncoder(m_elevatorEncoder);
    addChild("Position", msre);
    bottomSwitch = new DigitalInput(ElevatorConstants.bottomSwitchDIPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    CurrentPosition = m_elevatorEncoder.getPosition();
    SmartDashboard.putNumber("Position", CurrentPosition);
    //System.out.println("Position: " + CurrentPosition);
  }

  /**Closed loop control of the elevator
   * 
   * @param target is the desired position (inches) for the elevator
   */
  public void PIDController(double target) {
    m_ClosedLoopController.setReference(target, ControlType.kPosition);
    latestTarget = target;
  }

  /** Changes the target of the PID Controller by the adjustment
   * @param adjustment
   */
  public void nudge(double adjustment){
    latestTarget += adjustment;
    m_ClosedLoopController.setReference(latestTarget, ControlType.kPosition);
  }

  /** Run the elevator motor.
   * @param speed sets motor speed. Variable between -1, 1. Positive is up and negative is down.
  */
  public void setElevatorSpeed(double speed) {
    m_elevatorMotorL.set(speed);
  }

  
  public void forwardHO() {
    m_elevatorMotorL.set(.1);
  }

  
  public void backwardHO() {
    m_elevatorMotorL.set(-.1);
  }

  /** Stop the elevator motor. */
  public void stopElevator() {
    m_elevatorMotorL.stopMotor();
  }

  /**Open-loop control of the elevator motor  
   * @param Tim is the target height
  */
  public void Vroom(double Tim) {
    Destination = Tim;
    speedOL = Math.copySign(.1, (Destination - CurrentPosition));
    setElevatorSpeed(speedOL);
  }

  /**Check whether elevator has attained the target height. Stop if it has. TODO this be lie.
   * 
   * @param Destination: Target Height
   * @return <b>True</b> if the motor is at the target height
   * <li><b>False</b> if the motor is not at the target height </li>
   */
  public boolean Finished(double Destination) {
    return Math.abs(Destination-CurrentPosition)< closeEnough;
  }

  // Test mode methods
  /**Suspend and restore soft limits
   * @param
   */
  public void switchSL(boolean enabled){
    configL.softLimit.reverseSoftLimitEnabled(enabled);
    configL.softLimit.forwardSoftLimitEnabled(enabled);
    m_elevatorMotorL.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    if (enabled) m_elevatorEncoder.setPosition(0.);
  }
}