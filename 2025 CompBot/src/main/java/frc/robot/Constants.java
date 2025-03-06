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
  public static final boolean HANDLER_AVAILABLE = true;
  public static final boolean ELEVATOR_AVALIBLE = true;
  public static final boolean LIGHTS_AVALIBLE = true;



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
    public static final double kDriveP = 0.22; // 2025 Competition Robot
    public static final double kDriveI = 0.0; // 2025 Competition Robot
    public static final double kDriveD = 0.0; // 2025 Competition Robot
    public static final double kDriveFF = 0.255; // 2025 Competition Robot


    public static final double kTurningP = 2.3;//2.05; // 2025 Competition Robot   0.75
    public static final double kTurningI = 0.0; // 2025 Competition Robot
    public static final double kTurningD = 0.0; // 2025 Competition Robot

    public static final double kTurnGearRatio = 18.75; // 2025 Competion Robot Mark 4n ratio      12.8 on project X
    public static final double kTurnPositionConversionFactor = 1.0 / kTurnGearRatio;

    // By default, the drive encoder in position mode measures rotations at the drive motor
    // Convert to meters at the wheel
    public static final double kDriveGearRatio = 5.36; // 2025 Competion Robot            6.75 on Project X
    public static final double kDrivePositionConversionFactor =
      (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

    // By default, the drive encoder in velocity mode measures RPM at the drive motor
    // Convert to meters per second at the wheel
    public static final double kDriveVelocityConversionFactor =
      kDrivePositionConversionFactor / 60.0;
    public static final double kRampRate = 1;
    public static final double kRampRateT = 0.75; //0.75; 
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
        public static final double kTrackWidth = Units.inchesToMeters(20.75);  //16.5     2025COMP
        //
        public static final double kWheelBase = Units.inchesToMeters(26.75);   //22.5        2025COMP
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
//     | D10 S11      D20 S21 |        | D15 S25      D16 S26 |
//     | E11          E22     |        | E35          E36     |
//     |                      |        |                      |
//     |                      |        |                      |
//     |                      |        |                      |
//     |                      |        |                      |
//     | D40 S41      D30 S31 |        | D17 S27      D18 S28 |
//     | E42          E32     |        | E37          E38     |
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
    new PIDConstants(4.25, 0.1, 0);  
  public static final PIDConstants rotationConstants = 
    new PIDConstants(3., 0, 0);  

  }

  public static class HandlerConstants{

    public static final double pivotEncoderConversionFactor = 1/41.833;
    public static final double pivotCurrentLimit = 30;
    public static final double grabAlgaeSpeed = -.5;
    public static final double grabAlgaeCurrent = 30;
    public static final double algaeHoldSpeed = -.025;
    public static final double algaeShootSpeed = 0.5;
    public static final double outputSpeed = .75;
    public static final double grabCoralSpeed = .25;

    //PID
    public static final double pivotP = .6;
    public static final double pivotI = 0.00;
    public static final double pivotD = 0;


    //PID positions
    public static final double L1 = 0.9;
    public static final double L2 = 0;
    public static final double L3 = 0;
    public static final double L4 = 0.22;
    public static final double intake = 0;
    public static final double algaeL1 = 0.28;
    public static final double algaeL2 = .95;
    public static final double algaeL3 = .95;              
    public static final double algaeL4 = 0;
    public static final double algaeIntake = 1.;
    public static final double algae = .9;
    public static final double barge = 0.28;
    public static final double nudgeUp = .025;
    public static final double nudgeDown = -.025;

    public static final double kAlgaeFloor = 1;


    // Soft Limits
    public static final double forwardSoftLimit = 1;
    public static final double reverseSoftLimit = 0;
  }

  public final static class ElevatorConstants {
    public static final double softLimitForward = 65.;  // inches
    public static final double softLimitReverse = 3.; // inches

    public static double encoderConversionFactor = 50./124.238;  // inches : raw

    //public static double ENCODERCONVERSION = 7./286. ;  // inches : raw

    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0.02;
    public static final double kFFClimb = -0.;

    //PID Postiions
    public static final double kNudgeUpE = .25;
    public static final double kNudgeDownE = -.25;
    public static final double kBarge = 64.75;
    public static final double Intake = 3.;
    public static final double L1 = 23.;
    public static final double L2 = 12;
    public static final double L3 = 27.2;
    public static final double L4 = 60.3;
    public static final double algaeIntake = 3.;
    public static final double algaeL1 = 3.;
    public static final double algaeL2 = 24.;
    public static final double algaeL3 = 39.45;
    public static final double algaeL4 = 3.;


   // public static final double L2Algae = 20;
  }

  public static class CANIDS {

    //Handler
    public static final int wheels = 61;
    public static final int pivot = 60;

    //Elevator
    public static final int elevatorL = 50;
    public static final int elevatorR = 51;
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
    public static final int kRightCoralStation = 2;
    public static final int kLeftCoralStation = 3;
    public static final int kDriveToBarge = 4;
    public static final int kpathfindTopCoralStation =   4;
    public static final int kDriverRobotOrientedButton = 6;

    //Gamemech Buttons
    public static final int kIntake =                    5;
    public static final int kL1shoot =                   4;
    public static final int kL2shoot =                   3;
    public static final int kL3shoot =                   2;
    public static final int kL4shoot =                   1;
    public static final int kBarge =                     6;
    public static final int kAlgeaSwitch =               8;
    public static final int kNudgeUpE =                  9;
    public static final int kNudgeDownE =                10;
    public static final int kElevatorZero =              12;

    //Gamemech2 Buttons
    public static final int kNudgeUpP =                  1;
    public static final int kNudgeDownP =                2;
    public static final int kHandlerHalfExtend =         3;
    public static final int kOutCoral =                  5;
    public static final int kInCoral =                   6;
    public static final int kRetract =                   7;
    public static final int kHandlerExtend =             8;
    public static final int kAlgaeOut =                  9;
    public static final int kAlgaeIn =                   10;





    //Test Buttons
    public static final int RestSoftLimitsE =            1;     //Driver
    public static final int EnableSoftLimitsE =          2;     //Driver
    public static final int RestSoftLimitsP =            3;     //Driver
    public static final int EnableSoftLimitsP =          4;     //Driver

    public static final double kDeadband = 0.075;
    public static final double kSlewRateLimiter = 3.5;
    public static final double kTurnSlewRateLimiter = 5.;

  }

  public static class CamConstants {
      //public static final double camera_Height_Meters = Units.inchesToMeters(7.);
      //public static final double target_Height_Meters = Units.inchesToMeters(78.);

      public static final int followDistance = 1; //Meters
      public static final double camera_Height_Meters = Units.inchesToMeters(32);
      public static final double camera_Pitch_Radians = Units.degreesToRadians(1);
      public static final double cameraYawRadians = 2.29;
      public static final double tag_Follow_P = 1.75;
      public static final double tag_Follow_D = 0.5;
      public static final double drive_Range_Meters = 1;
      public static final double robotToCamX = Units.inchesToMeters(11);
      public static final double robotToCamY = Units.inchesToMeters(-3.5);
      public static final double robotToCamZ = Units.inchesToMeters(30.5);


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
    public static final boolean isCompetition = true;
    public static final PathConstraints pathConstraints = new PathConstraints(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                          DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                          DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                          DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    public static final double blueRightStationX = 1.;
    public static final double blueRightStationY = 1.;
    public static final double blueRightStationRot = 54;

    public static final double blueLeftStationX = 1.5;
    public static final double blueLeftStationY = 7.;
    public static final double blueLeftStationRot = -54.;

    public static final double redRightStationX = 16.;
    public static final double redRightStationY = 7.;
    public static final double redRightStationRot = -126.;   //54+180

    public static final double redLeftStationX = 16.;
    public static final double redLeftStationY = 1.;
    public static final double redLeftStationRot = 126.;     //-54+180

    public static final double coralStationEndVelocity = 0;

    public static final double blueBargeX = 7.5;
    public static final double blueBargeY = 6.2;
    public static final double blueBargeRot = 0.;

    public static final double redBargeX = 10.;
    public static final double redBargeY =1.9;
    public static final double redBargeRot = 180.;

    public static final double bargeEndVelocity = 0.;
  }


}
