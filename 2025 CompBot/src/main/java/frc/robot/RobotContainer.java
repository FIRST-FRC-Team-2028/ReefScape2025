// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.L1Shoot;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Handler;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain driveSubsystem;
  private final Handler handlerSubsystem;
  private final AprilCamera april;

  // Replace with CommandPS4Controller or CommandJoystick if needed
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick mechJoytick1 = new Joystick(OIConstants.kMechControllerPort);
    private final Joystick mechJoytick2 = new Joystick(OIConstants.kMechControllerPort2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.HANDLER_AVAILABLE){
      handlerSubsystem = new Handler();
    } else handlerSubsystem = null;
    if (Constants.DRIVE_AVAILABLE){
      driveSubsystem = new Drivetrain();
    } else driveSubsystem = null;
    if (Constants.CAMERA_AVAILABLE){
      april = new AprilCamera();
    } else april = null;

    // Configure the trigger bindings
    configureButtonBindings();
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverJoytick, OIConstants.kResetGyro)
      .onTrue(new InstantCommand(() -> driveSubsystem.resetGyro()));

    new JoystickButton(mechJoytick1, OIConstants.kL1shoot)
      .onTrue(new L1Shoot(handlerSubsystem));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public Drivetrain getDrivetrain(){
    return driveSubsystem;
  }

  public Handler getHandler(){
    return handlerSubsystem;
  }

  public AprilCamera getApril(){
    return april;
  }
}
