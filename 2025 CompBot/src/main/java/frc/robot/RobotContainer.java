// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HandlerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorPosition;
import frc.robot.commands.ElevatorVbusVariable;
import frc.robot.commands.Spit;
import frc.robot.commands.SpitSequence;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Handler;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain driveSubsystem;
  private final Elevator elevatorSubsystem;
  private final Handler handlerSubsystem;
  private final AprilCamera april;
  private final SendableChooser<Command> autoChooser;
  private final Field2d field;
  public static Command sequence;

  // Joysticks
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
    if (Constants.ELEVATOR_AVALIBLE){
      elevatorSubsystem = new Elevator();
    } else elevatorSubsystem = null;
    if (Constants.CAMERA_AVAILABLE){
      april = new AprilCamera();
    } else april = null;


    sequence = new SequentialCommandGroup(new ElevatorPosition(elevatorSubsystem, -150, -50)
                                 .andThen(new WaitCommand(.5))
                                 .andThen(new ElevatorPosition(elevatorSubsystem, -100, -100))
                                 .andThen(new WaitCommand(.5))
                                 .andThen(new ElevatorPosition(elevatorSubsystem, -50, -150))
                                 .andThen(new WaitCommand(2))
                                 .andThen(new ElevatorPosition(elevatorSubsystem, -1, -1)));



    if (Constants.DRIVE_AVAILABLE){
      driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem));
      field = new Field2d();
      field.setRobotPose(driveSubsystem.getPoseEstimatorPose());
      SmartDashboard.putData(field);

      /*PathPlannerLogging.setLogCurrentPoseCallback((pose)-> {
        field.setRobotPose(pose);
      });*/
      new EventTrigger("Raise Elevator L4").onTrue(Commands.print("Elevator at L4"));
      NamedCommands.registerCommand("Place Coral L4", Commands.print("I Placed It"));
      NamedCommands.registerCommand("Print Comp Auto 1", Commands.print("Comp Auto 1"));
      NamedCommands.registerCommand("Print Auto 1", Commands.print("Auto 1"));
      //autoChooser = AutoBuilder.buildAutoChooser();
      //If competition is true, only autos that start with comp will appear
      autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> PathPlannerConstants.isCompetition
          ? stream.filter(auto -> auto.getName().startsWith("comp"))
          : stream);
      SmartDashboard.putData("Auto Chooser", autoChooser);
    } else { autoChooser=null;
             field = null;
    }
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
  public void configureButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    if (Constants.ELEVATOR_AVALIBLE){

      new JoystickButton(driverJoytick, OIConstants.kThirdButton)
        .onTrue(sequence);

        /*
      new JoystickButton(driverJoytick, OIConstants.kFirstButton)
        .and(() -> elevatorSubsystem.getElevatorPosition() >= -35)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.SetElevatorSpeed(-.2)))
        .onFalse(new InstantCommand(() -> elevatorSubsystem.StopElevator()));

      new JoystickButton(driverJoytick, OIConstants.kFirstButton)
        .and(() -> elevatorSubsystem.getElevatorPosition() <= -35)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.SetElevatorSpeed(-.1)))
        .onFalse(new InstantCommand(() -> elevatorSubsystem.StopElevator()));
         */

      // Second Button (X) brings the elevator back down
      
      new JoystickButton(driverJoytick, OIConstants.kFirstButton)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.SetElevatorSpeedL(.1)))
        .onFalse(new InstantCommand(() -> elevatorSubsystem.StopElevator()));

      new JoystickButton(driverJoytick, OIConstants.kSecondButton)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.SetElevatorSpeedR(.1)))
        .onFalse(new InstantCommand(() -> elevatorSubsystem.StopElevator()));
    }
    

    if (Constants.DRIVE_AVAILABLE) {
      new JoystickButton(driverJoytick, OIConstants.kResetGyro)
        .onTrue(new InstantCommand(() -> driveSubsystem.resetGyro()));
      /*new JoystickButton(driverJoytick, OIConstants.kpathfindTopCoralStation)
        .onTrue(new InstantCommand(() -> {
          try {
            System.out.println("WORKING");
            driveSubsystem.pathfindToPath(PathPlannerPath.fromPathFile("To Tag 17"));
          } catch (Exception e) {
            e.printStackTrace();
          }
        }));*/
      new JoystickButton(driverJoytick, OIConstants.kpathfindTopCoralStation)
        .onTrue(driveSubsystem.pathfindToPath("To Tag 17"));

      new JoystickButton(driverJoytick, 3)
        .onTrue(new InstantCommand(() -> field.setRobotPose(driveSubsystem.getPoseEstimatorPose())));
    }

    if (Constants.HANDLER_AVAILABLE && Constants.ELEVATOR_AVALIBLE) {
      new JoystickButton(mechJoytick1, OIConstants.kL1shoot)
        .onTrue(new SpitSequence(handlerSubsystem, elevatorSubsystem, HandlerConstants.intake, 0));
      new JoystickButton(mechJoytick1, OIConstants.kL2shoot)
        .onTrue(new SpitSequence(handlerSubsystem, elevatorSubsystem, HandlerConstants.L2Position, 0));
      new JoystickButton(mechJoytick1, OIConstants.kL3shoot)
        .onTrue(new SpitSequence(handlerSubsystem, elevatorSubsystem, HandlerConstants.L2Position, 0));
      new JoystickButton(mechJoytick1, OIConstants.kL4shoot)
        .onTrue(new SpitSequence(handlerSubsystem, elevatorSubsystem, HandlerConstants.L4Position, 0));
    }

    if (Constants.HANDLER_AVAILABLE) {
        new JoystickButton(mechJoytick1, OIConstants.kRePivot)
          .onTrue(new InstantCommand(() -> handlerSubsystem.rePivot()));
        new JoystickButton(mechJoytick1, OIConstants.kNudgeUp)
          .onTrue(new InstantCommand(() -> handlerSubsystem.reTargetPivot(HandlerConstants.nudgeUp)));
        new JoystickButton(mechJoytick1, OIConstants.kNudgeDown)
          .onTrue(new InstantCommand(() -> handlerSubsystem.reTargetPivot(HandlerConstants.nudgeDown)));
        new JoystickButton(driverJoytick, OIConstants.kFirstButton)
          .onTrue(new InstantCommand(()-> handlerSubsystem.Shoot(.1)))
          .onFalse(new InstantCommand(()-> handlerSubsystem.stop()));
        new JoystickButton(driverJoytick, OIConstants.kSecondButton)
          .onTrue(new InstantCommand(()-> handlerSubsystem.Shoot(-.1)))
          .onFalse(new InstantCommand(()-> handlerSubsystem.stop()));
      }

      if (Constants.DRIVE_AVAILABLE && Constants.CAMERA_AVAILABLE){
        new JoystickButton(mechJoytick1, OIConstants.kMoveSide)
        .onTrue(new InstantCommand(() -> driveSubsystem.gamemechSwitchOn()));
        new JoystickButton(mechJoytick1, OIConstants.kMoveSide)
        .onFalse(new InstantCommand(() -> driveSubsystem.gamemechSwitchOff()));
      }

      
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
    return autoChooser.getSelected();
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

  public Elevator getElevator(){
    return elevatorSubsystem;
  }
}
