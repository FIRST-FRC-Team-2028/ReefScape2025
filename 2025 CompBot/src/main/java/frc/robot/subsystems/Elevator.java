// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
  private final SparkMax m_elevatorMotorL;
  private final SparkMax m_elevatorMotorR;
  private final RelativeEncoder m_elevatorEncoder;
  private final SparkClosedLoopController m_ClosedLoopController;
  private double CurrentPosition = 0.0;
  private double Destination = 0;

  public Elevator() {
    m_elevatorMotorL = new SparkMax(Constants.CANIDS.elevatorL, MotorType.kBrushless);
    m_elevatorMotorR = new SparkMax(Constants.CANIDS.elevatorR, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorMotorL.getEncoder();
    m_ClosedLoopController = m_elevatorMotorL.getClosedLoopController();
    SparkMaxConfig configL = new SparkMaxConfig();
    SparkMaxConfig configR = new SparkMaxConfig();

    configL.idleMode(IdleMode.kBrake)
          .inverted(false);
    configL.encoder.positionConversionFactor(ElevatorConstants.ENCODERCONVERSION);
    configL.softLimit.forwardSoftLimit(ElevatorConstants.SOFTLIMITFORWARD)
                    .forwardSoftLimitEnabled(true)
                    .reverseSoftLimit(ElevatorConstants.SOFTLIMITREVERSE)
                    .reverseSoftLimitEnabled(true);
    configR.follow(Constants.CANIDS.elevatorL, true);

    m_elevatorMotorL.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotorR.configure(configR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   // config.closedLoop
   //    .p(1)  // 
   //     .i(0)
   //     .d(0);
        m_elevatorEncoder.setPosition(0.);  // initialize encoder position at startup
  }

  /** closed- loop control
   * @param position desired, inches
   */
  public void moveToPose(double position){
    m_ClosedLoopController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    CurrentPosition = m_elevatorEncoder.getPosition();
    SmartDashboard.putNumber("Position", CurrentPosition);
    System.out.println("Position: " + CurrentPosition);
  }

  /**Closed loop control of the elevator
   * TODO MrG says Calibrate the system before use:
   *   6 determine kp
   * 
   * @param target is the desired position (inches) for the elevator
   */
  public void PIDController(double target) {
    //m_ClosedLoopController.setReference(target,ControlType.kPosition);
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

  /** Resets the current position to 0 */
  public void resetPosition() {
    m_elevatorEncoder.setPosition(0.);
  }

  /**Open-loop control of the elevator motor  
   * @param Tim is the target height
  */
  public void Vroom(double Tim) {
    Destination = Tim;
    m_elevatorMotorL.set(Math.copySign(.1, (Destination - CurrentPosition)));
  }

  /**Check whether elevator has attained the target height. Stop if it has.
   * 
   * @param Destination: Target Height
   * @return <b>True</b> if the motor is at the target height
   * <li><b>False</b> if the motor is not at the target height </li>
   */
  public boolean Finished(double Destination) {
    return false;
  }
}