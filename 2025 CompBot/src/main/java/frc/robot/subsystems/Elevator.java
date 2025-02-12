// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;
import frc.robot.SendableRelEncoder;
import frc.robot.SendableSparkMax;
import frc.robot.FakeMotor;
import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase {
  
  //private final SendableSparkMax m_elevatorMotor;
  //private final RelativeEncoder m_elevatorEncoder;
  private final FakeMotor m_elevatorMotorL, m_elevatorMotorR;
  private final FakeMotor.Encoder m_elevatorEncoder;
  //private final SendableRelEncoder msre;
  //private final SparkClosedLoopController m_ClosedLoopController;
  private final PIDController m_ClosedLoopController;
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
    //m_elevatorMotorL = new SendableSparkMax(Constants.CANIDS.elevator, MotorType.kBrushless);
    m_elevatorMotorL = new FakeMotor(Constants.CANIDS.elevatorL, MotorType.kBrushless);
    m_elevatorMotorR = new FakeMotor(Constants.CANIDS.elevatorR, MotorType.kBrushless);
    //m_elevatorMotorL = new SparkMax(Constants.CANIDS.elevator, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorMotorL.getEncoder();
    m_ClosedLoopController = m_elevatorMotorL.getClosedLoopController();
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
    configR.follow(Constants.CANIDS.elevatorL, true)
           .idleMode(IdleMode.kBrake);

    //m_elevatorMotor.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //m_elevatorMotor.configure(configR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_elevatorEncoder.setPosition(0); //3 in offset + 1/4 gear box

    addChild("Left (Leader)", m_elevatorMotorL);
    //msre = new SendableRelEncoder(m_elevatorEncoderL);
    //addChild("Position", msre);
    bottomSwitch = new DigitalInput(ElevatorConstants.bottomSwitchDIPort);
  }

  // Warning: this method does not run during testPeriodic()
  @Override
  public void periodic() {
    m_elevatorMotorL.update();  //  added for FakeMotor
    m_elevatorMotorL.periodic(); // added for FakeMotor
    SmartDashboard.putNumber("Left Position", m_elevatorEncoder.getPosition());
  }

  
  public double getElevatorPosition() {
    return m_elevatorEncoder.getPosition();
  }

  /**Closed loop control of the elevator
   * 
   * @param target is the desired position (inches) for the elevator
   */
  public void PIDController(double target) {
    //m_ClosedLoopController.setReference(target, ControlType.kPosition);
    m_elevatorMotorL.setReference(target, ControlType.kPosition);
    latestTarget = target;
  }

  /** Changes the target of the PID Controller by the adjustment
   * @param adjustment
   */
  public void Nudge(double adjustment){
    latestTarget += adjustment;
    //m_ClosedLoopControllerL.setReference(latestTarget, ControlType.kPosition);
    m_elevatorMotorL.setReference(latestTarget, ControlType.kPosition);
  }

  /** Run the elevator motor.
   * @param speed sets motor speed. Variable between -1, 1. Positive is up and negative is down.
  */
  public void SetElevatorSpeed(double speed) {
    m_elevatorMotorL.set(speed);
  }

  /** Stop the elevator motor. */
  public void StopElevator() {
    m_elevatorMotorL.stopMotor();
  }

  // Test mode methods
  /**Suspend and restore soft limits
   * @param
   */
  public void SwitchSL(boolean enabled){
    configL.softLimit.reverseSoftLimitEnabled(enabled);
    configL.softLimit.forwardSoftLimitEnabled(enabled);
    //m_elevatorMotorL.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    if (enabled) m_elevatorEncoder.setPosition(0.);
  }
}