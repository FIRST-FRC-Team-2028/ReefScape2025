// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HandlerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.commands.RunWheels;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorPosition;
import frc.robot.commands.HandlerPosition;
import frc.robot.commands.SpitSequence; // Stuff using is Commented out
import frc.robot.commands.TimedDrive;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Handler;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final Lights lights;
  private final SendableChooser<Command> autoChooser;
  public boolean algae = false;
  public boolean blueDriverStation = true;
  

  // Joysticks
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick mechJoytick1 = new Joystick(OIConstants.kMechControllerPort);
    private final Joystick mechJoytick2 = new Joystick(OIConstants.kMechControllerPort2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  @SuppressWarnings("unused")
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
    if (Constants.LIGHTS_AVALIBLE){
      lights = new Lights();
    } else lights = null;

    if (Constants.DRIVE_AVAILABLE){
      driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem));
      if (Constants.ELEVATOR_AVALIBLE && Constants.HANDLER_AVAILABLE){
        new EventTrigger("Intake Wheels").onTrue(new RunWheels(handlerSubsystem, 0.25, 0.3, false));
        new EventTrigger("Raise L4").onTrue(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.L4)
        .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.L4)));
        NamedCommands.registerCommand( "Raise L3", new ElevatorPosition(elevatorSubsystem, ElevatorConstants.L3)
                                    .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.L3)));
        NamedCommands.registerCommand("Intake", new HandlerPosition(handlerSubsystem, HandlerConstants.intake)
                                    .andThen(new WaitCommand(.125))
                                    .andThen(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.Intake)));
        NamedCommands.registerCommand("Print Auto 1", Commands.print("Auto 1"));
        NamedCommands.registerCommand("Raise Elevator L4", new ElevatorPosition(elevatorSubsystem, ElevatorConstants.L4)
                                    .andThen(new WaitCommand(1))
                                    .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.L4)));
        NamedCommands.registerCommand("Shoot",new RunWheels(handlerSubsystem, HandlerConstants.outputSpeed, 1, false)
                                    .raceWith(new WaitCommand(.5)));
        
      }
      //autoChooser = AutoBuilder.buildAutoChooser();
      //If competition is true, only autos that start with comp will appear
      autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> PathPlannerConstants.isCompetition
          ? stream.filter(auto -> auto.getName().startsWith("Comp"))
          : stream);
      SmartDashboard.putData("Auto Chooser", autoChooser);
    } else autoChooser=null;
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      
    };
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
  @SuppressWarnings("unused")
  public void configureButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    
    new JoystickButton(mechJoytick1, OIConstants.kAlgeaSwitch)
    .onTrue(new InstantCommand(()-> algae = true))
    .onFalse(new InstantCommand(()-> algae = false));

    if (Constants.LIGHTS_AVALIBLE){
      /*new JoystickButton(driverJoytick, OIConstants.kBlueLight)
        .onTrue(new InstantCommand(()-> lights.blueLight(true)))
        .onFalse(new InstantCommand(()-> lights.blueLight(false)));*/
    }

    if (Constants.ELEVATOR_AVALIBLE){
      // if (!OIConstants.kCompleteSwitch){

      new JoystickButton(mechJoytick1, OIConstants.kNudgeUpE)
        .onTrue(new InstantCommand(()-> elevatorSubsystem.reTargetElevator(ElevatorConstants.kNudgeUpE)));

      new JoystickButton(mechJoytick1, OIConstants.kNudgeDownE)
        .onTrue(new InstantCommand(()-> elevatorSubsystem.reTargetElevator(ElevatorConstants.kNudgeDownE)));

        new JoystickButton(mechJoytick1, OIConstants.kIntake)
          .onTrue(new InstantCommand(() -> driveSubsystem.elevatorPositionBoolean(false)));
          //.andThen(new InstantCommand(()-> lights.blueLight(false))));

        new JoystickButton(mechJoytick1, OIConstants.kL3shoot)
          .onTrue(new InstantCommand(() -> driveSubsystem.elevatorPositionBoolean(true)));

        new JoystickButton(mechJoytick1, OIConstants.kL4shoot)
          .onTrue(new InstantCommand(() -> driveSubsystem.elevatorPositionBoolean(true)));

        new JoystickButton(mechJoytick1, OIConstants.kBarge)
          .onTrue(new InstantCommand(()->driveSubsystem.elevatorPositionBoolean(true)));

        new JoystickButton(mechJoytick1, 11)
          .onTrue(new InstantCommand(()-> elevatorSubsystem.setElevatorSpeed(-.2)))
          .onFalse(new InstantCommand(()->elevatorSubsystem.PIDController(elevatorSubsystem.getPosition())));

        new JoystickButton(mechJoytick1, OIConstants.kElevatorZero)
          .onTrue(new InstantCommand(()->elevatorSubsystem.switchSL(false, false)))
          .onTrue(new InstantCommand(()->elevatorSubsystem.setElevatorSpeed(-.5)))
          .onFalse(new InstantCommand(()->elevatorSubsystem.switchSL(true, false)))
          .onFalse(new InstantCommand(()->elevatorSubsystem.PIDController(elevatorSubsystem.getPosition())));
          //.onFalse(new InstantCommand(()->elevatorSubsystem.stopElevator()));

          /*new JoystickButton(mechJoytick1, 12)
          .onTrue(new InstantCommand(()->elevatorSubsystem.PIDController(elevatorSubsystem.getPosition())));*/
         
      //}
    }
    

    if (Constants.DRIVE_AVAILABLE) {
      //reset gyro
      new JoystickButton(driverJoytick, OIConstants.kResetGyro)
        .onTrue(new InstantCommand(() -> driveSubsystem.resetGyro()));
      //Pathplanner drive to blue, right coral station
      new JoystickButton(driverJoytick, OIConstants.kRightCoralStation).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.blueRightStationX, PathPlannerConstants.blueRightStationY, 
                                                 PathPlannerConstants.blueRightStationRot, PathPlannerConstants.coralStationEndVelocity));
      //Pathplanner drive to blue, left coral station
      new JoystickButton(driverJoytick, OIConstants.kLeftCoralStation).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.blueLeftStationX, PathPlannerConstants.blueLeftStationY, 
                                                 PathPlannerConstants.blueLeftStationRot, PathPlannerConstants.coralStationEndVelocity));
      //Pathplanner drive to red, right coral station
      new JoystickButton(driverJoytick, OIConstants.kRightCoralStation).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.redRightStationX, PathPlannerConstants.redRightStationY, 
                                                 PathPlannerConstants.redRightStationRot, PathPlannerConstants.coralStationEndVelocity));
      //Pathplanner drive to red, left coral station                     
      new JoystickButton(driverJoytick, OIConstants.kLeftCoralStation).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.blueLeftStationX, PathPlannerConstants.blueLeftStationY, 
                                                 PathPlannerConstants.blueLeftStationRot, PathPlannerConstants.coralStationEndVelocity));
      //Pathplanner drive to blue barge
      new JoystickButton(driverJoytick, OIConstants.kDriveToBarge).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.blueBargeX, PathPlannerConstants.blueBargeY, 
                                                 PathPlannerConstants.blueBargeRot, PathPlannerConstants.bargeEndVelocity));
      //Pathplanner drive to red barge
      new JoystickButton(driverJoytick, OIConstants.kDriveToBarge).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.redBargeX, PathPlannerConstants.redBargeY, 
                                                 PathPlannerConstants.redBargeRot, PathPlannerConstants.bargeEndVelocity));
      //Drive robot back enough to score on L4
      /*new JoystickButton(driverJoytick, OIConstants.kBackUp)
      .onTrue(new TimedDrive(driveSubsystem, 0.25, -0.5, 0, 0));*/
      

      /*new JoystickButton(driverJoytick, OIConstants.kpathfindTopCoralStation)
        .onTrue( driveSubsystem.pathfindToPath("Top Coral Station"));*/
      //new JoystickButton(driverJoytick, 2)
      //  .onTrue(driveSubsystem.pathfindToPose(1.25, 1, 0, 0));
    }

    if (Constants.HANDLER_AVAILABLE && Constants.ELEVATOR_AVALIBLE) {
      //Coral Intake
      new JoystickButton(mechJoytick1, OIConstants.kIntake).and(()->!getAlgae())
        .onTrue(new HandlerPosition(handlerSubsystem, HandlerConstants.intake)
        .andThen(new WaitCommand(.35))
        .andThen(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.Intake)));
      //Grabbing Algae off the floor
      new JoystickButton(mechJoytick1, OIConstants.kIntake).and(()->getAlgae())
        .onTrue(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.algaeIntake)
        .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.algaeIntake)));
      //Coral Onto Level 1
      new JoystickButton(mechJoytick1, OIConstants.kL1shoot).and(()->!getAlgae())
        .onTrue(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.L1)
        .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.L1)));
      //Algae Carrying position
      new JoystickButton(mechJoytick1, OIConstants.kL1shoot).and(()->getAlgae())
        .onTrue(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.algaeL1)
        .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.algaeL1)));
      //Coral onto level 2
      new JoystickButton(mechJoytick1, OIConstants.kL2shoot).and(()->!getAlgae())
        .onTrue(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.L2)
        .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.L2)));
      //Grab Algae between L2 and L3
      new JoystickButton(mechJoytick1, OIConstants.kL2shoot).and(()->getAlgae())
        .onTrue(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.algaeL2)
        .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.algaeL2)));
      //Coral onto level 3
      new JoystickButton(mechJoytick1, OIConstants.kL3shoot).and(()->!getAlgae())
        .onTrue(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.L3)
        .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.L3)));
      //Grab Algae between L3 and L4
      new JoystickButton(mechJoytick1, OIConstants.kL3shoot).and(()->getAlgae())
        .onTrue(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.algaeL3)
        .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.algaeL3)));
      //Coral onto Level 4
      new JoystickButton(mechJoytick1, OIConstants.kL4shoot)
        .onTrue(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.L4)
        .andThen(new WaitCommand(1.25))
        .andThen(new HandlerPosition(handlerSubsystem, HandlerConstants.L4))
        .andThen(new TimedDrive(driveSubsystem, 0.25, -0.5, 0, 0)));
      //Algae into barge
      new JoystickButton(mechJoytick1, OIConstants.kBarge)
        .onTrue(new HandlerPosition(handlerSubsystem, HandlerConstants.barge)
       // .andThen(new RunWheels(handlerSubsystem, HandlerConstants.grabAlgaeSpeed, 0.5, false))  //if not false then will run forever
        .andThen(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.kBarge)));


      /*new JoystickButton(mechJoytick1, OIConstants.kL1shoot)
        .onTrue(new SpitSequence(handlerSubsystem, elevatorSubsystem, HandlerConstants.intake, ElevatorConstants.L1));

      new JoystickButton(mechJoytick1, OIConstants.kL2shoot)
        .onTrue(new SpitSequence(handlerSubsystem, elevatorSubsystem, HandlerConstants.intake, ElevatorConstants.L2));

      new JoystickButton(mechJoytick1, OIConstants.kL3shoot)
        .onTrue(new SpitSequence(handlerSubsystem, elevatorSubsystem, HandlerConstants.intake, ElevatorConstants.L3));

      new JoystickButton(mechJoytick1, OIConstants.kL4shoot)
        .onTrue(new SpitSequence(handlerSubsystem, elevatorSubsystem, HandlerConstants.L4Position, ElevatorConstants.L4));
*/
    }

   if (Constants.HANDLER_AVAILABLE) {
        /*new JoystickButton(mechJoytick1, OIConstants.kRePivot)
          .onTrue(new InstantCommand(() -> handlerSubsystem.rePivot()));*/

        new JoystickButton(mechJoytick2, OIConstants.kNudgeUpP)
          .onTrue(new InstantCommand(() -> handlerSubsystem.reTargetPivot(HandlerConstants.nudgeUp)));

        new JoystickButton(mechJoytick2, OIConstants.kNudgeDownP)
          .onTrue(new InstantCommand(() -> handlerSubsystem.reTargetPivot(HandlerConstants.nudgeDown)));

        new JoystickButton(mechJoytick2, OIConstants.kInCoral)
          .whileTrue(new RunWheels(handlerSubsystem, HandlerConstants.grabCoralSpeed, 0.3, false)
          .andThen(new InstantCommand(()->lights.blueLight(true))));

        new JoystickButton(mechJoytick2, OIConstants.kAlgaeOut)
          .whileTrue(new RunWheels(handlerSubsystem, HandlerConstants.algaeShootSpeed, 0, true));

        new JoystickButton(mechJoytick2, OIConstants.kAlgaeIn)
          .whileTrue(new RunWheels(handlerSubsystem, HandlerConstants.grabAlgaeSpeed, 0, true));

        new JoystickButton(mechJoytick2, OIConstants.kHandlerHalfExtend)
          .onTrue(new HandlerPosition(handlerSubsystem, 0.5));

        new JoystickButton(mechJoytick2, OIConstants.kRetract)
          .onTrue(new HandlerPosition(handlerSubsystem, 0));

        new JoystickButton(mechJoytick2, OIConstants.kHandlerExtend)
          .onTrue(new HandlerPosition(handlerSubsystem, HandlerConstants.kAlgaeFloor));

        new JoystickButton(mechJoytick2, OIConstants.kOutCoral)
          .whileTrue(new RunWheels(handlerSubsystem, HandlerConstants.outputSpeed, 1, false));


    }

      
      // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    boolean MRG_WAS_NEVER_HERE = false;
    if (MRG_WAS_NEVER_HERE){
    /* How might the handler motion automatically wait move to a position 
     * of potential collision until the elevator has risen sufficiently?
     * Desired:
     *   send elevator on its way
     *   set trigger for handler to pivot only after elevator is high enough
     * Will the trigger remain active after the sequence is finished;
     * must the trigger be deactivated somehow
    */
    /*new JoystickButton(mechJoytick1, 777)
      .onTrue(Commands.parallel(new ElevatorPosition(elevatorSubsystem, 999.),
                                new InstantCommand(()->setHandlerTrigger(true)))
             .andThen(new InstantCommand(()->setHandlerTrigger(false)))
      );*/
    }

  }

  Command tHandle;
  /**activate/deactivate a trigger to run the handler when the elevator is high enough */
  void setHandlerTrigger(boolean activate){
    if(activate){
      tHandle =new HandlerPosition(handlerSubsystem, 8888.);
      new Trigger(()->{return elevatorSubsystem.getPosition() > 9999.5;}).onTrue(tHandle);
    }else {
      // remove the command
      tHandle.cancel();
    }
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

  public boolean getAlgae(){
    return algae;
  }
  public Lights getLights(){
    return lights;
  }
}
