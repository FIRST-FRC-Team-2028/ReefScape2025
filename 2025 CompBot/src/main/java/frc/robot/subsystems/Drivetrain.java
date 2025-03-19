// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.RobotConstants;

public class Drivetrain extends SubsystemBase {
  static double kMaxSpeed = Constants.DriveConstants.kMaxTranslationalVelocity;
  static double kMaxAngularSpeed = Constants.DriveConstants.kMaxRotationalVelocity;
  private final SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;
  boolean elevatorUp;
  private RobotContainer m_RobotContainer;


  private final SwerveModule m_frontLeft =
      new SwerveModule(
          "FL",
          Constants.DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftAbsoluteEncoderPort);
  private final SwerveModule m_frontRight =
      new SwerveModule(
          "FR",
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightAbsoluteEncoderPort);
  private final SwerveModule m_backLeft =
      new SwerveModule(
          "BL",
          DriveConstants.kBackLeftDriveMotorPort,
          DriveConstants.kBackLeftTurningMotorPort,
          DriveConstants.kBackLeftAbsoluteEncoderPort);
  private final SwerveModule m_backRight =
      new SwerveModule(
          "BR",
          DriveConstants.kBackRightDriveMotorPort,
          DriveConstants.kBackRightTurningMotorPort,
          DriveConstants.kBackRightAbsoluteEncoderPort);

  private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
  private final Pigeon2 m_gyro = new Pigeon2(0);

  
  private final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),                                                                 
          m_backRight.getPosition()
         },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  /** Creates a new Drivetrain. */
  public Drivetrain(RobotContainer m_RobotContainer) {
    this.m_RobotContainer = m_RobotContainer;
    resetGyro();
    resetPoseEstimatorPose(new Pose2d(0, 0, getRotation2d()));
    for (SwerveModule module : modules) {
      module.resetDriveEncoder();
      module.initializeAbsoluteTurningEncoder();
      module.initializeRelativeTurningEncoder();

    }
    
    //Pathplanner Autobuilder
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPoseEstimatorPose, 
        this::resetPoseEstimatorPose, 
        this::getChassisSpeeds, 
        this::drive, 
        new PPHolonomicDriveController(
          DriveConstants.translationConstants,
          DriveConstants.rotationConstants
        ),
        config,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
      //System.out.println("IF I SEE THIS LINE THAT MEANS THAT THE TRY PART OF THE TRY CATCH IS WORKING AND THERE ISN'T AN ERROR INSIDE THE TRY");
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      e.printStackTrace();
    }
    
  
  }
  @Override
  public void periodic() {
    //updateOdometry();
    updatePoseEstimator();
    SmartDashboard.putNumber("Drive Current", getLoad());
    //SmartDashboard.putNumber("FL voltage", m_frontLeft.getAppliedVoltage());
    // This method will be called once per scheduler run
    //SmartDashboard.putString("Front Left Rotation Position", m_frontLeft.getRelativeTurningPosition().toString());
    //SmartDashboard.putNumber("FL Drive Encoder", m_frontLeft.getRelativeDrivePosition());
    //SmartDashboard.putNumber("FL Rotation Encoder", m_frontLeft.getRelativeTurningPosition().getDegrees());
  }




  public void BreakMode() {
    m_frontLeft.BreakMode();
    m_frontRight.BreakMode();
    m_backLeft.BreakMode();
    m_backRight.BreakMode();
  }

  public void CoastMode() {
    m_frontLeft.CoastMode();
    m_frontRight.CoastMode();
    m_backLeft.CoastMode();
    m_backRight.CoastMode();
  }

  public void resetGyro(){
    m_gyro.reset();
  }
  /**Set the heading on the gyro */
  public void setGyro(double heading){
    m_gyro.setYaw(heading);
  }

  public Pigeon2 getGyro(){
    return m_gyro;
  }

  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  /** return a Rotation2d representing the heading of the robot
     * described in radians clockwise from forward
     */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading().getDegrees());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.kPeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    
    setModuleStates(swerveModuleStates);
    
    //SmartDashboard.putNumber("Module Turning Real", m_frontLeft.getRelativeTurningPosition().getRotations());
    /*m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);*/
  }
  
  /** drive a specified speed (vector)
   * @param x x-component of the vector, positive is forward
   * @param y y-component of the vector, positive is right
   * @param rot rot-component of the vector, positive is clockwise arount up-vector (optional)
   * @param fieldOriented if true (optional, default is robot-oriented)
   */
  public void driveComponent(double x, double y, double rot, boolean fieldOriented){
    ChassisSpeeds chassisSpeeds;
    if (!fieldOriented) { //normal use
      chassisSpeeds = new ChassisSpeeds(x, y, rot);
    }else {
    // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                   x, y, rot, getRotation2d());
    }
    drive(chassisSpeeds);
  }
  /** drive a specified speed (vector)
   * @param x x-component of the vector, positive is forward
   * @param y y-component of the vector, positive is right
   * @param rot rot-component of the vector, positive is clockwise arount up-vector (optional)
   * @param fieldOriented if true (optional, default is robot-oriented)
   */
  public void driveComponent(double x, double y, double rot){
    driveComponent(x,y,rot,false);
  }
  /** drive a specified speed (vector) in the robot coordinate system
   * @param x x-component of the vector, positive is forward
   * @param y y-component of the vector, positive is right
   */
  public void driveComponent(double x, double y){
    driveComponent(x,y,0.,false);
  }
  /** drive a specified speed (vector) in the field coordinate system
   * @param x x-component of the vector, positive is forward
   * @param y y-component of the vector, positive is right
   * @param fieldOriented if true (optional, default is robot-oriented)
   */
  public void driveComponent(double x, double y, boolean ro){
    driveComponent(x,y,0.,ro);
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (SwerveModuleState state:desiredStates){
      state.angle= new Rotation2d(-state.angle.getRadians()); //The encoder won't let us invert it 
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
    //SmartDashboard.putNumber("Module Turnin Target", desiredStates[0].angle.getRotations());
  }
/**Updates the odometry location using swerve module position */
/*  public void updateOdometry() {                Replaced by m_poseEstimator 
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }*/
  int estimaterCounter = 0;
  public void updatePoseEstimator() {
    m_poseEstimator.update(m_gyro.getRotation2d(),
                          new SwerveModulePosition[] {                                  
                          m_frontLeft.getPosition(),
                          m_frontRight.getPosition(),
                          m_backLeft.getPosition(),
                          m_backRight.getPosition()
    });
    // Since Revlib wont let us invert the turn encoder, so we munged it, now we have to munge more to get the right orientation
    //Pose2d badY = m_poseEstimator.getEstimatedPosition();
    //m_poseEstimator.resetPose(new Pose2d(badY.getX(),-badY.getY(),badY.getRotation().times(-1)));  //MrG forces you to look at this to build
    if(Constants.CAMERA_AVAILABLE){
      if (m_RobotContainer.getApril().isPoseEstimated()) {

        //var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
        //var camPose = aprilTagFieldLayout.getTagPose(4).transformBy(camToTargetTrans.inverse());
        
        //SmartDashboard.putNumber("Counter", ++estimaterCounter);
       m_poseEstimator.addVisionMeasurement(
                  m_RobotContainer.getApril().getPose3d().toPose2d(), m_RobotContainer.getApril().estimatedPoseTime);
      }  
    }
    SmartDashboard.putNumber("Robot X Pos", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Robot Y Pos", m_poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Robot Rotation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
    };
  }
/** Position of robot with x and y in meters */
/*  public Pose2d getOdomentryPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetOdomentryPose(Pose2d pose) {                         Replaced by m_poseEstimator
    //System.out.println(pose);
    m_odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }*/ 

  public Pose2d getPoseEstimatorPose() {
    return m_poseEstimator.getEstimatedPosition();
  }
  public void resetPoseEstimatorPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }
  /**Contructs and runs a path to the given path name avoiding obsticals outlinned in navgrid.json. Uses the
   * normal constraints of the robot as path constraints.
   * @param pathname The name of the path file in the deploy/pathplanner/paths file.
   * Will only print out "FAIL" if the file name does not exist.
  */
  public Command pathfindToPath(String pathname) {
    PathPlannerPath path;
    try{
      path = PathPlannerPath.fromPathFile(pathname);
      return AutoBuilder.pathfindThenFollowPath(path, PathPlannerConstants.pathConstraints);
    } catch(Exception e){
      e.getStackTrace();
      return new InstantCommand(()->System.out.println("FAIL"));
    }
  }
  /**Contructs and runs a path to the given pose avoiding obsticals outlinned in navgrid.json
   * @param x The x cordinate of the target position 
   * @param y The y cordinate of the target position
   * @param rotation the Rotation 2d value of the target position in degrees
   * @param goalEndVelocity The velocity of the robot at the end of the path. 0 is required to stop at the target pose.
   * A value > 0 may be used to keep the robot up to speed for the driver to take over.
   */
  public Command pathfindToPose(double x, double y, double rotation, double goalEndVelocity) {
    Rotation2d rotation2d = new Rotation2d().fromDegrees(rotation);
    
    Pose2d targetPose = new Pose2d(x, y, rotation2d);
    System.out.println("working");
    return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.pathConstraints1, goalEndVelocity);
  }

  public Command pathfindToPose(double x, double y, Rotation2d rotation, double goalEndVelocity) {    
    Pose2d targetPose = new Pose2d(x, y, rotation);
    System.out.println(targetPose.toString());
    return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.pathConstraints1, goalEndVelocity);
  }

  /** returns the state, does the DriveTrain believe the elevator is up? */
  public boolean elevatorUp(){
    return elevatorUp;
  }
  /** sets the state whether the DriveTrain should drive as if the CG is elevated */
  public void elevatorPositionBoolean(boolean up){
    elevatorUp = up;
  }

  /** get average current load for the drive motors */
  public double getLoad(){
    double load=0.;
    for (SwerveModule mod: modules){
      load+=mod.getDriveLoad()*.25;
    }
    return load;
  }


  
}
