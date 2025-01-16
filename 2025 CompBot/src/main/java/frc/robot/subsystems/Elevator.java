// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SoftLimitConfig;

public class Elevator extends SubsystemBase {
  
  private final SparkMax m_elevatorMotor;
  private final RelativeEncoder m_elevatorEncoder;
  private final SparkClosedLoopController m_ClosedLoopController;
  private double CurrentPosition = 0.0;
  private double Destination = 0;
  private double Vector = 0;

  public Elevator() {
    m_elevatorMotor = new SparkMax(57, MotorType.kBrushless);
    // TODO make the CAN id a constant; see what Ethan did in Handler
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
    m_ClosedLoopController = m_elevatorMotor.getClosedLoopController();
    //m_ClosedLoopController.setReference(5, ControlType.kPosition);  MrG says do not start the PID controller here
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop
        .p(1)  // 
        .i(0)
        .d(0);  
        // TODO the motor can't go down?
        // TODO  make the motor use this configuration
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    CurrentPosition = m_elevatorEncoder.getPosition();
    SmartDashboard.putNumber("Position", CurrentPosition);
    System.out.println("Position: " + CurrentPosition);
    // System.out.println("To Stop: " + ((Destination - CurrentPosition) * (Vector)));
    // System.out.println("Vector: " + Vector);
    Finished(Destination);  //TODO Since periodic runs even outside any control loop,
                            //      hould it have the capability to shutdown the motor?
                            //  The answer is NO!
  }

  /**Closed loop control of the elevator
   * TODO MrG says Calibrate the system before use:
   *   1 ensure positive control make the elevator go up; if not, invert it (see config)
   *   2 find the range of motion of the mechanism
   *   3 set the encoder conversion factor (see config)
   *   4 pick a convenient position at which to reset 
   *       (Can this process be automated
   *        or must it be a manual setup step prior to each match?)
   *   5 set soft limits
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
    m_elevatorMotor.set(speed);
  }

  /** Stop the elevator motor. */
  public void stopElevator() {
    m_elevatorMotor.stopMotor();
  }

  /** Resets the current position to 0 */
  public void resetPosition() {
    CurrentPosition = m_elevatorEncoder.getPosition();
  }

  /**Open-loop control of the elevator motor  
   * @param Tim is the target height
  */
  public void Vroom(double Tim) {
    Destination = Tim;
    Vector = Math.copySign(1, (Destination - CurrentPosition));
    m_elevatorMotor.set(Math.copySign(.1, (Destination - CurrentPosition)));
  }

  /**Check whether elevator has attained the target height. Stop if it has.
   * 
   * @param Destination: Target Height
   * @return <b>True</b> if the motor is at the target height
   * <li><b>False</b> if the motor is not at the target height </li>
   */
  public boolean Finished(double Destination) {
    if (((Destination - CurrentPosition) * (Vector)) <= 0) {
      stopElevator();
      return false;
    }
    return false;
  }
}