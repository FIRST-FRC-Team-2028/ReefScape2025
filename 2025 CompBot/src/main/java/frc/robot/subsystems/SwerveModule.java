// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.spark.config.ClosedLoopConfig;
//import com.revrobotics.spark.config.EncoderConfig;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.wpilibj.Preferences;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.RobotConstants;

public class SwerveModule {
  public final String moduleName;

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final SparkMaxConfig m_driveConfig;
  private final SparkMaxConfig m_turningConfig;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningRelativeEncoder;

  private final SparkClosedLoopController m_drivePIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private final CANcoder m_turningAbsEncoder;
  private final CANcoderConfiguration m_turningAbsEncoderConfig;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param moduleName Name of the module
   * @param driveMotorChannel CAN ID of the drive motor controller
   * @param turningMotorChannel CAN ID of the turning motor controller
   * @param turningAbsoluteEncoderChannel CAN ID of absolute encoder
   */
  public SwerveModule(
      String name,
      int driveMotorChannel,
      int turningMotorChannel,
      int turningAbsoluteEncoderChannel) {
    moduleName = name;

    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    //m_driveMotor.restoreFactoryDefaults();
    //m_turningMotor.restoreFactoryDefaults();
    
    m_driveConfig = new SparkMaxConfig();
    m_turningConfig = new SparkMaxConfig();
    m_driveConfig
        .idleMode(IdleMode.kBrake)
        //.closedLoopRampRate(ModuleConstants.kRampRate)
        //.openLoopRampRate(ModuleConstants.kRampRate)
        .smartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .inverted(false);
    m_driveConfig.encoder
        .velocityConversionFactor(ModuleConstants.kDriveVelocityConversionFactor)
        .positionConversionFactor(ModuleConstants.kDrivePositionConversionFactor);
    m_driveConfig.closedLoop
        .pidf(ModuleConstants.kDriveP, ModuleConstants.kDriveI, ModuleConstants.kDriveD, ModuleConstants.kDriveFF);
    
    m_turningConfig
        .idleMode(IdleMode.kBrake)
        //.closedLoopRampRate(ModuleConstants.kRampRateT)
        //.openLoopRampRate(ModuleConstants.kRampRateT)
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .smartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit)
        .inverted(false);
    m_turningConfig.encoder
        .positionConversionFactor(ModuleConstants.kTurnPositionConversionFactor);
        
    m_turningConfig.closedLoop
        .positionWrappingEnabled(true)
        .positionWrappingMaxInput(0.5)
        .positionWrappingMinInput(-0.5)
        .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);
        
        

    //m_driveMotor.setIdleMode(IdleMode.kBrake);
    //m_turningMotor.setIdleMode(IdleMode.kBrake);
    

    //m_driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);
    //m_turningMotor.setSmartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);
    //m_driveMotor.enableVoltageCompensation(RobotConstants.kNominalVoltage);
    //m_turningMotor.enableVoltageCompensation(RobotConstants.kNominalVoltage);

    //m_driveMotor.setInverted(false);
    //m_turningMotor.setInverted(false);

    m_drivePIDController = m_driveMotor.getClosedLoopController();
    m_turningPIDController = m_turningMotor.getClosedLoopController();

    //m_drivePIDController.setP(ModuleConstants.kDriveP);
    //m_drivePIDController.setI(ModuleConstants.kDriveI);
    //m_drivePIDController.setD(ModuleConstants.kDriveD);
    // m_drivePIDController.setIZone();
    //m_drivePIDController.setFF(ModuleConstants.kDriveFF);
    // m_drivePIDController.setOutputRange();

    //m_turningPIDController.setP(ModuleConstants.kTurningP);
    //m_turningPIDController.setI(ModuleConstants.kTurningI);
    //m_turningPIDController.setD(ModuleConstants.kTurningD);
    // m_turningPIDController.setIZone();
    // m_turningPIDController.setFF();
    // m_turningPIDController.setOutputRange();

    // m_turningPIDController.setSmartMotionMaxVelocity(
    //    ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, 0);
    // m_turningPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
    // m_turningPIDController.setSmartMotionMaxAccel(
    //    ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared, 0);
    // m_turningPIDController.setSmartMotionAllowedClosedLoopError(0.1, 0);

    // Limit the PID Controller's range to (-0.5, 0.5], with continuous wrapping
    //m_turningPIDController.positionWrappingEnabled(true);
    //m_turningPIDController.setPositionPIDWrappingMaxInput(0.5);
    //m_turningPIDController.setPositionPIDWrappingMinInput(-0.5);

    m_turningAbsEncoder = new CANcoder(turningAbsoluteEncoderChannel);
    m_turningAbsEncoderConfig = new CANcoderConfiguration();
    m_turningAbsEncoder.getConfigurator().refresh(m_turningAbsEncoderConfig);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningRelativeEncoder = m_turningMotor.getEncoder();

    //m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivePositionConversionFactor);
    //  m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveVelocityConversionFactor);

   // m_turningRelativeEncoder.setPositionConversionFactor(
   //     ModuleConstants.kTurnPositionConversionFactor);

    // m_driveMotor.burnFlash();
    // m_turningMotor.burnFlash();
    //m_turningMotor.setClosedLoopRampRate(ModuleConstants.kRampRateT);
    //m_driveMotor.setClosedLoopRampRate(ModuleConstants.kRampRate);
    //m_turningMotor.setOpenLoopRampRate(ModuleConstants.kRampRateT);
    //m_driveMotor.setOpenLoopRampRate(ModuleConstants.kRampRate);
    m_driveMotor.configure(m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  

  /**
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getRelativeTurningPosition());
  }

  /**
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getRelativeDrivePosition(), getRelativeTurningPosition().times(-1));
  }

  /**
   * @param desiredState The desired state for the module, with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    var encoderRotation = getRelativeTurningPosition();

    // Optimize the reference state to avoid spinning further than 90 degrees
    state.optimize( encoderRotation);
    
    
    

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    m_drivePIDController.setReference(
        state.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(
        state.angle.getRotations(), SparkMax.ControlType.kPosition);
  }

  public void BreakMode() {
    m_driveConfig.idleMode(IdleMode.kBrake);

  }

  public void CoastMode() {
    m_driveConfig.idleMode(IdleMode.kCoast);
  }

  public void resetDriveEncoder() {
    m_driveEncoder.setPosition(0.0);
  }

  /**
   * @return The relative drive position of the module
   */
  public double getRelativeDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  /**
   * @return The relative turning angle of the module
   */
  public Rotation2d getRelativeTurningPosition() {
    double relativePositionRotations = m_turningRelativeEncoder.getPosition();
    return Rotation2d.fromRotations(MathUtil.inputModulus(relativePositionRotations, -0.5, 0.5));
  }

  /**
   * @param waitPeriod Period to wait for up-to-date status signal value
   * @return The absolute turning angle of the module in rotations
   */
  public Rotation2d getAbsTurningPosition(double waitPeriod) {
    Angle absPositonRotations;
    
    if (waitPeriod > 0.0) {
      absPositonRotations =
          m_turningAbsEncoder.getAbsolutePosition().waitForUpdate(waitPeriod).getValue();  // rootations
    } else {
      absPositonRotations = m_turningAbsEncoder.getAbsolutePosition().getValue();
    }
    
    
    return Rotation2d.fromRotations(-absPositonRotations.magnitude());    //CANCoders are inverted of relative encoder
  }

  /**
   * Updates the relative turning encoder to match the absolute measurement of the module turning
   * angle.
   */
  public void initializeRelativeTurningEncoder() {
    m_turningRelativeEncoder.setPosition(getAbsTurningPosition(0.25).getRotations());
  }

  /** Initializes the magnetic offset of the absolute turning encoder */
  public void initializeAbsoluteTurningEncoder() {
  /*   double magnetOffsetFromCANCoder = getAbsTurningEncoderOffset().getRotations();
    Preferences.initDouble(
        moduleName + DriveConstants.AbsoluteEncoders.kAbsEncoderMagnetOffsetKey,
        magnetOffsetFromCANCoder);
    double magnetOffsetFromPreferences =
        Preferences.getDouble(
            moduleName + DriveConstants.AbsoluteEncoders.kAbsEncoderMagnetOffsetKey,
            magnetOffsetFromCANCoder);
    setAbsTurningEncoderOffset(magnetOffsetFromPreferences); */
  }

  /**
   * Decrements the offset of the absolute turning encoder to align the wheels with an alignment
   * device. To be performed upon a hardware change (e.g. when a swerve module or absolute turning
   * encoder has been swapped.)
   */
  /* public void zeroAbsTurningEncoderOffset() {
    m_turningAbsEncoder.getConfigurator().refresh(m_turningAbsEncoderConfig);

    Rotation2d magnetOffset = getAbsTurningEncoderOffset().minus(getAbsTurningPosition(0.25));
    Preferences.setDouble(
        moduleName + DriveConstants.AbsoluteEncoders.kAbsEncoderMagnetOffsetKey,
        magnetOffset.getRotations());
    setAbsTurningEncoderOffset(magnetOffset.getRotations());

    initializeRelativeTurningEncoder();
  } */

  /**
   * Sets the magnetic offset of the absolute turning encoder
   *
   * @param offset The magnetic offset in rotations
   */
  public void setAbsTurningEncoderOffset(double offset) {
    m_turningAbsEncoderConfig.MagnetSensor.MagnetOffset = offset;
    m_turningAbsEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ModuleConstants.AbsoluteSensorDiscontinuityPoint;
    //m_turningAbsEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    m_turningAbsEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_turningAbsEncoder.getConfigurator().apply(m_turningAbsEncoderConfig);
  }

  /**
   * @return The magnet offset of the module's absolute turning encoder
   */
  public Rotation2d getAbsTurningEncoderOffset() {
    return Rotation2d.fromRotations(m_turningAbsEncoderConfig.MagnetSensor.MagnetOffset);
  }

  /**
   * @return The name of the swerve module
   */
  public String getName() {
    return moduleName;
  }

  /*public void voltageDrive(double sysSpeed) {
    m_driveMotor.set(sysSpeed);
  }*/
  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }
  public double getVoltage() {
    return m_driveMotor.getBusVoltage();
  }
  public double getAppliedVoltage(){
    return m_driveMotor.getAppliedOutput();
  }
  public double getDriveLoad(){
    return m_driveMotor.getOutputCurrent();
  }
}
