// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/** Table of Contents
 *  Availbilty of Subsystems
 *  ModuleConstants
 *  DriveConstants
 *  HandlerConstants
 *  ElevatorConstants
 *  CANIDS
 *  OIConstants
 *  CamConstants
 *  RobotConstants
 *  PathPlannerConstants
 *  FieldConstants
 */
public final class Constants {
  public static final boolean DRIVE_AVAILABLE = true;
  public static final boolean CAMERA_AVAILABLE = true;
  public static final boolean HANDLER_AVAILABLE = false;
  public static final boolean ELEVATOR_AVALIBLE = false;



  public static final class ModuleConstants {
    public static final int kDriveMotorCurrentLimit = 80;
    public static final int kTurningMotorCurrentLimit = 80;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
    public static final double kDriveMotorGearRatio = 1 / 5.36;
    public static final double kTurningMotorGearRatio = 1 / 18.75;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
   // Theoraticlly maybe; in practice from measurements
    //public static final double kDriveEncoderRot2Meter = Units.inchesToMeters(1)/41.2;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 42;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 42;
    public static final double kPTurning_Comp = 0.5;
    public static final double kDriveP = 0.1; // 2023 Competition Robot
    public static final double kDriveI = 0.0; // 2023 Competition Robot
    public static final double kDriveD = 0.0; // 2023 Competition Robot
    public static final double kDriveFF = 0.255; // 2023 Competition Robot

    public static final double kPTurning = 0.5;
    public static final double kTurningP = 0.75; // 2023 Competition Robot 
    public static final double kTurningI = 0.0; // 2023 Competition Robot
    public static final double kTurningD = 0.0; // 2023 Competition Robot

    public static final double kTurnGearRatio = 12.8; // 12.8 on project X
    public static final double kTurnPositionConversionFactor = 1.0 / kTurnGearRatio;

    // By default, the drive encoder in position mode measures rotations at the drive motor
    // Convert to meters at the wheel
    public static final double kDriveGearRatio = 6.75; // 6.75 on Project X
    public static final double kDrivePositionConversionFactor =
      (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

    // By default, the drive encoder in velocity mode measures RPM at the drive motor
    // Convert to meters per second at the wheel
    public static final double kDriveVelocityConversionFactor =
      kDrivePositionConversionFactor / 60.0;
    public static final double kRampRate = 1;
    public static final double kRampRateT = 0.75;
    public static final double AbsoluteSensorDiscontinuityPoint = 0.5; //1 is value between [0, 1] 0.5 is value between [-0.5, 0.5] 0 is value between [-1, 0] -CTRE Docs
  }

  public static class DriveConstants{
     //Defines the conventional order of the modules when in arrays
        public static final int Front_Left = 0;
        public static final int Front_Right = 1;
        public static final int Back_Left = 2;
        public static final int Back_Right = 3;

        // Distance between right and left wheels
        // Distance between front and back wheels
        public static final double kTrackWidth = Units.inchesToMeters(16.5);  //16.5     
        //
        public static final double kWheelBase = Units.inchesToMeters(22.5);   //22.5       
        // 
        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final SwerveDriveKinematics kDriveKinematics1 = new SwerveDriveKinematics(
//                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
  //              new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
    //            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      //          new Translation2d(-kWheelBase / 2,  kTrackWidth / 2));

                new Translation2d( kWheelBase / 2,  kTrackWidth / 2), // front left
                new Translation2d( kWheelBase / 2, -kTrackWidth / 2), // front right
                new Translation2d(-kWheelBase / 2,  kTrackWidth / 2), // back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right



                  

//    COMP BOT  FRONT                PRAC BOT    FRONT
//     +----------------------+        +----------------------+
//     | D11 S21      D12 S22 |        | D15 S25      D16 S26 |
//     | E31          E32     |        | E35          E36     |
//     |                      |        |                      |
//     |                      |        |                      |
//     |                      |        |                      |
//     |                      |        |                      |
//     | D13 S23      D14 S24 |        | D17 S27      D18 S28 |
//     | E33          E34     |        | E37          E38     |
//     +----------------------+        +----------------------+
//
//Front Left
        public static final int kFrontLeftDriveMotorPort        = 10;    // Module 1
        public static final int kFrontLeftTurningMotorPort      = 11;
        public static final int kFrontLeftAbsoluteEncoderPort   = 12;

//Front Right
        public static final int kFrontRightDriveMotorPort       = 20;    // Module 2
        public static final int kFrontRightTurningMotorPort     = 21;
        public static final int kFrontRightAbsoluteEncoderPort  = 22;

//Back Right
        public static final int kBackRightDriveMotorPort        = 30;    // Module 3
        public static final int kBackRightTurningMotorPort      = 31;
        public static final int kBackRightAbsoluteEncoderPort   = 32;

//Back Left
        public static final int kBackLeftDriveMotorPort         = 40;    // Module 4
        public static final int kBackLeftTurningMotorPort       = 41;
        public static final int kBackLeftAbsoluteEncoderPort    = 42;


        //Encoder Inversions
  public static final boolean kFrontLeftTurningEncoderReversed  = true;  //true
  public static final boolean kBackLeftTurningEncoderReversed   = true; //true
  public static final boolean kFrontRightTurningEncoderReversed = true;  //true
  public static final boolean kBackRightTurningEncoderReversed  = true;  //true

  public static final boolean kFrontLeftDriveEncoderReversed  = false;
  public static final boolean kBackLeftDriveEncoderReversed   = false;
  public static final boolean kFrontRightDriveEncoderReversed = false;
  public static final boolean kBackRightDriveEncoderReversed  = false;

  public static final boolean kFrontLeftDriveAbsoluteEncoderReversed  = true;
  public static final boolean kBackLeftDriveAbsoluteEncoderReversed   = true;
  public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
  public static final boolean kBackRightDriveAbsoluteEncoderReversed  = true;

  // Speed Limits
  public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * 2 * Math.PI;

  public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond /1.5;
  public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                                     kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
  public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.8;
  public static final double kptwist = .5;

  public static final double kFineControlSpeed = .5;
  public static final double kFasterSpeed = .4;

  public static final String kAbsEncoderMagnetOffsetKey = "kAbsEncoderMagnetOffsetKey";
  public static final double kDefaultAbsEncoderOffset = 0.0;

  // Units are meters per second
  public static final double kMaxTranslationalVelocity = 4.0; // 2023 Competion Robot // max 4.5

  // Units are radians per second
  public static final double kMaxRotationalVelocity = 5.0; // 2023 Competion Robot // max 5.0
  public static final double kRotateToZero = -2;
  public static final PIDConstants translationConstants = 
    new PIDConstants(ModuleConstants.kDriveP, ModuleConstants.kDriveI, ModuleConstants.kDriveD);
  public static final PIDConstants rotationConstants = 
    new PIDConstants(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);

  }

  public static class HandlerConstants{
    public static double pivotP = 1;
    public static double pivotI = 0;
    public static double pivotD = 0;
    public static final int grabSensorPort = 0;
    public static final double pivotCurrentLimit = 30;
    public static final double grabAlgaeSpeed = .5;
    public static final double grabAlgaeCurrent = 30;
    public static final double algaeHoldSpeed = .05;
    public static final double algaeShootSpeed = -0.5;
    public static final double outputSpeed = .5;
    public static final double grabCoralSpeed = .3;
    public static final double pivotEncoderConversionFactor = 1;


    //PID positions
    public static final double L4Position = 100;
    public static final double L2Position = 15;
    public static final double intake = 0;
    public static final double algae = 180;
    public static final double barge = -15;
    public static final double nudgeUp = 2;
    public static final double nudgeDown = -2;

    // Soft Limits
    public static final double forwardSoftLimit = 185;
    public static final double reverseSoftLimit = -20;
  }

  public final static class ElevatorConstants {
    public static final double SOFTLIMITFORWARD = 0.;  // all encoder values look backward
    public static final double SOFTLIMITREVERSE = -6.; //    but dumb REVRobotics API won't allow inversion
    public static final double L1 = -5.;
    public static double ENCODERCONVERSION = 7./286. ;  // inches : raw
  }

  public static class CANIDS {

    //Handler
    public static final int coralL = 50;
    public static final int coralR = 57;

    //Elevator
    public static final int elevatorL = 50;
    public static final int elevatorR = 57;  // TODO reset when hardware available
  }

  public static class OIConstants {
    public static final int kDriverControllerPort  =     0;
    public static final int kMechControllerPort =        1; 
    public static final int kMechControllerPort2 =       2;
    //Driver Axis (Includes triggers)
    public static final int kDriverYAxis =               0;
    public static final int kDriverXAxis =               1;
    public static final int kFineControlAxis =           2;
    public static final int kFastControlAxis =           3;
    public static final int kDriverRotAxis =             4;
    //Driver Buttons
    public static final int kResetGyro = 1;
    public static final int kFirstButton = 2;
    public static final int kSecondButton = 3;
    public static final int kThirdButton = 4;
    public static final int kpathfindTopCoralStation =   4;
    public static final int kDriverRobotOrientedButton = 6;

    //Gamemech Buttons
    public static final int kL1shoot =                   4;
    public static final int kL2shoot =                   3;
    public static final int kL3shoot =                   2;
    public static final int kL4shoot =                   1;
    public static final int kRePivot =                   5;
    public static final int kNudgeUp =                   6;
    public static final int kNudgeDown =                 7;

    //Gamemech2 Buttons

    public static final double kDeadband = 0.075;

  }

  public static class CamConstants {
      //public static final double camera_Height_Meters = Units.inchesToMeters(7.);
      //public static final double target_Height_Meters = Units.inchesToMeters(78.);

      public static final int followDistance = 1; //Meters
      public static final double camera_Height_Meters = Units.inchesToMeters(7);
      public static final double target_Height_Meters = Units.inchesToMeters(12);
      public static final double camera_Pitch_Radians = Units.degreesToRadians(1);
      public static final double tag_Follow_P = 1.75;
      public static final double tag_Follow_D = 0.5;
      public static final double drive_Range_Meters = 1;

  }

  public static final class RobotConstants {
    public static final double xcg = Units.inchesToMeters(0);  // from the geometric centroid
    public static final double kNominalVoltage = 12.0;
    public static final double kPeriod = TimedRobot.kDefaultPeriod;
    public static final double robotLength = Units.inchesToMeters(34.5); //inches
    public static final double robotWidth = Units.inchesToMeters(29.25) ; //inches
    public static final double handlerThickness = Units.inchesToMeters(6.); //inches
  }

  public static final class PathPlannerConstants {
    public static final boolean isCompetition = false;
    /*public static final PathConstraints pathConstraints = new PathConstraints(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                          DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                          DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                          DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);*/
    public static final PathConstraints pathConstraints = new PathConstraints(1,
                          0.5,
                          2,
                          1);
  }


}
