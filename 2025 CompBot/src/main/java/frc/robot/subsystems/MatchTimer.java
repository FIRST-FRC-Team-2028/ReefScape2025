// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MatchTimer extends SubsystemBase {
  boolean matchTimer = false;
  double matchTime;
  /** Controls the Match Timer */
  public MatchTimer() {}


  /** Starts the munged match timer */
  public void startMatchTimer(){
    matchTimer = true;
  }
  
  /** Stops the munged match timer */
  public void stopMatchTimer(){
    matchTimer = false;
  }
  
  /**
   * Gets the munged match time
   * @return matchTime the munged match time
   */
  public double matchTime(){
    return matchTime;
  }

  /**
   * Sets the munged match time to a set value
   * @param value time to set the match timer
   */
  public void setMatchTimer(double value){
    matchTime = value;
  }
  @Override
  public void periodic() {
    if (matchTimer){
      matchTime += 0.02;
    }
    // This method will be called once per scheduler run
  }
}
