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
  
  private final SendableSparkMax m_elevatorMotorL, m_elevatorMotorR;
  private final RelativeEncoder m_elevatorEncoderL, m_elevatorEncoderR;
  private final SendableRelEncoder msre;
  private final SparkClosedLoopController m_ClosedLoopControllerL, m_ClosedLoopControllerR;
  private double CurrentPositionL = 0.0;
  private double CurrentPositionR = 0.0;
  private double Destination = 0;
  private final double closeEnough = .2;
  private double speedOL = 0.;
  private SparkMaxConfig configL, configR;
  private final DigitalInput bottomSwitch;
  private double latestTarget;

  /**Consists of two motors of which the Left Motor is the leader.
   * <p>Lifts the handler up. Methods:
   * @apiNote <b>PIDController</b> - closedloop control to destination
   *   <li><b>setElevatorSpeed</b> - vbus control
   *   <li><b>OpenLoopControl</b> - open-loop to destination
   *   <li><b>Finished</b> - open-loop check if finished
   */
  public Elevator() {
    m_elevatorMotorL = new SendableSparkMax(Constants.CANIDS.elevatorL, MotorType.kBrushless);
    m_elevatorMotorR = new SendableSparkMax(Constants.CANIDS.elevatorR, MotorType.kBrushless);
    //m_elevatorMotorL = new SparkMax(Constants.CANIDS.elevatorL, MotorType.kBrushless);
    //m_elevatorMotorR = new SparkMax(Constants.CANIDS.elevatorR, MotorType.kBrushless);
    m_elevatorEncoderL = m_elevatorMotorL.getEncoder();
    m_elevatorEncoderR = m_elevatorMotorR.getEncoder();
    m_ClosedLoopControllerL = m_elevatorMotorL.getClosedLoopController();
    m_ClosedLoopControllerR = m_elevatorMotorR.getClosedLoopController();
    configL = new SparkMaxConfig();
    configR = new SparkMaxConfig();

    configL.idleMode(IdleMode.kBrake)
           .inverted(false);
    // configL.encoder.positionConversionFactor(ElevatorConstants.EncoderConversionFactor);
    configL.softLimit.forwardSoftLimit(ElevatorConstants.SOFT_LIMIT_FORWARDl)
                     .forwardSoftLimitEnabled(true)
                     .reverseSoftLimit(ElevatorConstants.SOFT_LIMIT_REVERSEl)
                     .reverseSoftLimitEnabled(true);
    configL.closedLoop.pid(1.0, 
                           0.0, 
                           0.0);
    //configR.follow(Constants.CANIDS.elevatorL, true);
    configR.idleMode(IdleMode.kBrake)
           .inverted(true);
    // configL.encoder.positionConversionFactor(ElevatorConstants.EncoderConversionFactor);
    configR.softLimit.forwardSoftLimit(ElevatorConstants.SOFT_LIMIT_FORWARDr)
                     .forwardSoftLimitEnabled(true)
                     .reverseSoftLimit(ElevatorConstants.SOFT_LIMIT_REVERSEr)
                     .reverseSoftLimitEnabled(true);
    configR.closedLoop.pid(1.0, 
                           0.0, 
                           0.0);

    m_elevatorMotorL.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotorR.configure(configR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_elevatorEncoderL.setPosition(0); //3 in offset + 1/4 gear box
    m_elevatorEncoderR.setPosition(0);

    addChild("Left (Leader)", m_elevatorMotorL);
    msre = new SendableRelEncoder(m_elevatorEncoderL);
    addChild("Position", msre);
    bottomSwitch = new DigitalInput(ElevatorConstants.bottomSwitchDIPort);
  }

  @Override
  public void periodic() {
    CurrentPositionL = m_elevatorEncoderL.getPosition();
    CurrentPositionR = m_elevatorEncoderR.getPosition();
    SmartDashboard.putNumber("Left Position", m_elevatorEncoderL.getPosition());
    SmartDashboard.putNumber("Right Position", m_elevatorEncoderR.getPosition());
  }

  /**Closed loop control of the elevator
   * 
   * @param target is the desired position (inches) for the elevator
   */
  public void PIDController(double target) {
    m_ClosedLoopControllerL.setReference(target, ControlType.kPosition);
    latestTarget = target;
  }

  /** Changes the target of the PID Controller by the adjustment
   * @param adjustment
   */
  public void Nudge(double adjustment){
    latestTarget += adjustment;
    m_ClosedLoopControllerL.setReference(latestTarget, ControlType.kPosition);
  }

  /** Run the elevator motor.
   * @param speed sets motor speed. Variable between -1, 1. Positive is up and negative is down.
  */
  public void SetElevatorSpeedL(double speed) {
    m_elevatorMotorL.set(speed);
  }
  public void SetElevatorSpeedR(double speed) {
    m_elevatorMotorR.set(speed);
  }

  /** Stop the elevator motor. */
  public void StopElevator() {
    m_elevatorMotorL.stopMotor();
    m_elevatorMotorR.stopMotor();
  }

  /**Open-loop control of the elevator motor  
   * @param Target is the target height
  */
  public void OpenLoopControl(double Target) {
    Destination = Target;
    speedOL = Math.copySign(.1, (Destination - CurrentPositionL));
    //SetElevatorSpeed(speedOL);
  }

  
  public void PIDleft(double target) {
    m_ClosedLoopControllerL.setReference(target, ControlType.kPosition);
  }
  public void PIDright(double target) {
    m_ClosedLoopControllerR.setReference(target, ControlType.kPosition);
  }

  /**Check whether elevator has attained the target height. Stop if it has. TODO this be lie.
   * 
   * @param Destination: Target Height
   * @return <b>True</b> if the motor is at the target height
   * <li><b>False</b> if the motor is not at the target height </li>
   */
  public boolean Finished(double LeftDestination, double RightDestination) {
    boolean LeftCloseEnough = Math.abs(LeftDestination - CurrentPositionL) < closeEnough;
    boolean RightCloseEnough = Math.abs(RightDestination - CurrentPositionR) < closeEnough;
    return LeftCloseEnough && RightCloseEnough;
  }

  // Test mode methods
  /**Suspend and restore soft limits
   * @param
   */
  public void SwitchSL(boolean enabled){
    configL.softLimit.reverseSoftLimitEnabled(enabled);
    configL.softLimit.forwardSoftLimitEnabled(enabled);
    m_elevatorMotorL.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    if (enabled) m_elevatorEncoderL.setPosition(0.);
  }
}