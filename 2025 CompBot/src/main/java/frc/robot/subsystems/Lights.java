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
  /** Controls the lights on the robot to be used as signals to drivers and human players
   * <p>Methods:<ul>
   * <li>blueLight - Controls the blue lights on top of the robot
   * <li>orangeLight - Controls the orange lights under the coral pan
   */
  public Lights() {
    m_ControlModule = new PneumaticHub(1);
    k_BlueLight = m_ControlModule.makeSolenoid(8);
    k_OrangeLight = m_ControlModule.makeSolenoid(9);
    blueLight(true);
  }

  /**
   * Controls the blue lights on top of the robot
   * @param on whether or not the lights are on
   */
  public void blueLight(boolean on) {
    k_BlueLight.set(on);
  }

  /**
   * Controls the orange lights under the coral pan
   * @param on whether or not the lights are on
   */
  public void orangeLight(boolean on){
    k_OrangeLight.set(on);
  }
  @Override
  public void periodic() {
   
  }
}
