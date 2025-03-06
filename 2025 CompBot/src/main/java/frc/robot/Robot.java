// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Handler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  //private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  //private Drivetrain drivetrain;
  double distance;
  double error;
  private PowerDistribution PDH;
  Thread m_visionThread;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    PDH =new PowerDistribution(1, ModuleType.kRev);
    enableLiveWindowInTest(true);
    PathfindingCommand.warmupCommand().schedule();
    m_visionThread = 
          new Thread(
              () -> {
                UsbCamera camera = CameraServer.startAutomaticCapture();
                camera.setResolution(480, 270);

                CvSink cvSink = CameraServer.getVideo();

                CvSource outputStream = CameraServer.putVideo("Marking lines", 480, 270);

                 Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                /*Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);*/
                // Give the output stream a new image to display
                Imgproc.line(mat, new Point(70, 0), new Point(116, 110), new Scalar(255, 255, 255), 10);
                Imgproc.line(mat, new Point(145, 168), new Point(183, 270), new Scalar(0, 255, 0), 10);
                
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("BatV", PDH.getVoltage());
   
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.getLights().stopMatchTimer();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.getLights().setMatchTimer(0);
    m_robotContainer.getLights().startMatchTimer();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().enable();
    m_robotContainer.configureButtonBindings();
    m_robotContainer.getLights().setMatchTimer(0);
    m_robotContainer.getLights().startMatchTimer();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //this.drivetrain = m_robotContainer.getDrivetrain();
      //this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      //this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      //this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
  }
  //double smoothedXSpeed = 0.;
  //double smoothedYSpeed = 0.;
  //double smoothedTurningSpeed = 0.;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*if (Constants.HANDLER_AVAILABLE && Constants.LIGHTS_AVALIBLE){
      m_robotContainer.getLights().blueLight(m_robotContainer.getHandler().doIHaveIt());
      }*/
      if (Constants.LIGHTS_AVALIBLE && Constants.ELEVATOR_AVALIBLE){
      m_robotContainer.getLights().blueLight(m_robotContainer.getElevator().getPosition() == 3);
      }
    /*if (Constants.DRIVE_AVAILABLE) {
            // 1. Get real-time joystick inputs
            double xSpeed = -driverJoytick.getRawAxis(OIConstants.kDriverXAxis); // Negative values go forward
            double ySpeed = -driverJoytick.getRawAxis(OIConstants.kDriverYAxis);
            double turningSpeed = -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis);

            // 2. Apply deadband
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
            // System.out.println("Deadband Applied");
            // System.out.println("X: " + String.format("%.3f", xSpeed)
            // + " Y: " + String.format("%.3f", ySpeed)
            // + " R: " + String.format("%.3f", turningSpeed));

             xSpeed *= 1. - (DriveConstants.kFineControlSpeed * driverJoytick.getRawAxis(OIConstants.kFineControlAxis))
                    + (DriveConstants.kFasterSpeed * driverJoytick.getRawAxis(OIConstants.kFastControlAxis));
            ySpeed *= 1. - (DriveConstants.kFineControlSpeed * driverJoytick.getRawAxis(OIConstants.kFineControlAxis))
                    + (DriveConstants.kFasterSpeed * driverJoytick.getRawAxis(OIConstants.kFastControlAxis));
            turningSpeed *= 1.
                    - (DriveConstants.kFineControlSpeed * driverJoytick.getRawAxis(OIConstants.kFineControlAxis))
                    + (DriveConstants.kFasterSpeed * driverJoytick.getRawAxis(OIConstants.kFastControlAxis));
            
            /*    if (driverJoytick.getRawButton(OIConstants.kFineControlAxis)) {
                    xSpeed *= 1. - (DriveConstants.kFineControlSpeed);
                    ySpeed *= 1. - (DriveConstants.kFineControlSpeed);
                    turningSpeed *= 1. - (DriveConstants.kFineControlSpeed);
                }
                if (driverJoytick.getRawButton(OIConstants.kFastControlAxis)) {
                    xSpeed *= 1. + (DriveConstants.kFasterSpeed);
                    ySpeed *= 1. + (DriveConstants.kFasterSpeed);
                    turningSpeed *= 1. + (DriveConstants.kFasterSpeed);
                }*/
            // Smooth driver inputs
            /*smoothedXSpeed = smoothedXSpeed + (xSpeed - smoothedXSpeed) * .08;
            smoothedYSpeed = smoothedYSpeed + (ySpeed - smoothedYSpeed) * .08;
            smoothedTurningSpeed = smoothedTurningSpeed + (turningSpeed - smoothedTurningSpeed) * .08;
            // System.out.println("Raw Joystick Values");
            // System.out.println("X: " + String.format("%.3f", xSpeed)
            // + " Y: " + String.format("%.3f", ySpeed)
            // + " R: " + String.format("%.3f", turningSpeed));

            // if (driverJoytick.getRawButton(OIConstants.BALANCE_AUGMENTER)) {
            // double augment = Math.sin(Math.toRadians(pigeon.getPitch()-1));
            // //System.out.println(augment);
            // smoothedXSpeed+=augment*.036;
            // }
            xSpeed = smoothedXSpeed;
            ySpeed = smoothedYSpeed;
            turningSpeed = smoothedTurningSpeed;

            // 3. Make the driving smoother
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                    * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            // System.out.println("Smoothing Applied");
            // System.out.println("X: " + String.format("%.3f", xSpeed)
            // + " Y: " + String.format("%.3f", ySpeed)
            // + " R: " + String.format("%.3f", turningSpeed));

            // System.out.println("=====================");
            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            //if (driverJoytick.getRawButton(OIConstants.kDriverRobotOrientedButtonIdx)) { //Following Minibot
            if (!driverJoytick.getRawButton(OIConstants.kDriverRobotOrientedButton)) { //normal use
                // Relative to field
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, turningSpeed, drivetrain.getRotation2d());
            } else {
                // Relative to robot
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            }
            // System.out.println("Chassis Speeds");
            // System.out.println("X: " + String.format("%.3f", xSpeed)
            // + " Y: " + String.format("%.3f", ySpeed)
            // + " R: " + String.format("%.3f", swerveSubsystem.getRotation2d()));

            // System.out.println("Encoder: " + frontleftsteerencoder.getPosition());

            drivetrain.drive(chassisSpeeds);
            // MrG recommends looking at swerveSubsystem.driveIt 

            // 5. Convert chassis speeds to individual module states
            // SwerveModuleState[] moduleStates =
            // DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            /* this should be done in the SwerveSubsystem */
            // SwerveModuleState[] moduleStates =
            // swerveSubsystem.chassis2ModuleStates(chassisSpeeds);

            // 6. Output each module states to wheels
            // swerveSubsystem.setModuleStates(moduleStates);

            // steps 4-6 should be accomplished by the swerve subsystem via a method such as
            // swerveSubsystem.driveit(xSpeed, ySpeed, turningSpeed, fieldoriented);
            // }

            // swerveSubsystem.reportStatesToSmartDashbd(moduleStates);
            //SmartDashboard.putNumber("Turning Speed", turningSpeed);*/
          //}*/
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().getActiveButtonLoop().clear();     // Clear active button triggers

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if(Constants.ELEVATOR_AVALIBLE){
      if (driverJoytick.getRawButtonPressed(OIConstants.RestSoftLimitsE)){
      m_robotContainer.getElevator().switchSL(false, false);
      }
      if (driverJoytick.getRawButtonPressed(OIConstants.EnableSoftLimitsE)){
        m_robotContainer.getElevator().switchSL(true, true);
      }
      if (m_robotContainer.getElevator().LSPressed()){
        m_robotContainer.getElevator().setPosition(3);
      }
    // TODO
    // Elevator tests:
    //   Leader/Follower
    //   direction
    //   initial known position
    //   encoder range
    //   softlimits
    }   
    
    if (Constants.HANDLER_AVAILABLE){
      if (driverJoytick.getRawButtonPressed(OIConstants.kRightCoralStation)){
        m_robotContainer.getHandler().switchSL(false);
      }
      if (driverJoytick.getRawButtonPressed(OIConstants.kRightCoralStation)){
        m_robotContainer.getHandler().switchSL(true);
      }
    }
  
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
