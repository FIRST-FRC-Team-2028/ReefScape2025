// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private final PneumaticHub m_ControlModule;
  private final Solenoid k_BlueLight;
  private final Solenoid k_OrangeLight;
  boolean matchTimer=false;
  double matchTime;
  /** Creates a new Lights. */
  public Lights() {
    m_ControlModule = new PneumaticHub(1);
    k_BlueLight = m_ControlModule.makeSolenoid(8);
    k_OrangeLight = m_ControlModule.makeSolenoid(9);
    blueLight(true);
  }

  public void blueLight(boolean on) {
    k_BlueLight.set(on);
  }

  public void orangeLight(boolean on){
    k_OrangeLight.set(on);
  }

  public void startMatchTimer(){
    matchTimer = true;
  }
  public void stopMatchTimer(){
    matchTimer = false;
  }
  public double matchTime(){
    return matchTime;
  }
  public void setMatchTimer(double value){
    matchTime = value;
  }


  @Override
  public void periodic() {
    if (matchTimer){
      matchTime += 0.02;
    }
    if (matchTime > 90.){
      orangeLight(true);
    }
    else orangeLight(false);
    // This method will be called once per scheduler run
  }
}
