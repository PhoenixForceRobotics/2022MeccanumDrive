// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.commands.drivebase.CycleCenterOfRotation;
import frc.robot.commands.drivebase.ForwardDrive;
import frc.robot.commands.drivebase.CycleCenterOfRotation.Direction;
import frc.robot.commands.drivebase.IndividualWheelDrive;
import frc.robot.commands.drivebase.MecanumDrive;
import frc.robot.commands.drivebase.WestcoastDrive;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Drivebase.CenterOfRotation;
import frc.robot.utils.PFRController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final Drivebase drivebase = new Drivebase();

  // The robot's controllers are defined here...
  private final PFRController operatorController = new PFRController(0);
  private final PFRController driverController = new PFRController(1);
  private final PFRController driverController2 = new PFRController(2);
  
  // The robot's commands are defined here...
  private final MecanumDrive mecanumDrive = new MecanumDrive(drivebase, driverController);
  private final WestcoastDrive westcoastDrive = new WestcoastDrive(drivebase, driverController);
  private final ForwardDrive forwardDrive = new ForwardDrive(drivebase, driverController);

  private final IndividualWheelDrive individualWheelDrive = new IndividualWheelDrive(drivebase, driverController, driverController2);
  private final CycleCenterOfRotation cycleUpCenterOfRotation = new CycleCenterOfRotation(drivebase, Direction.UP);
  private final CycleCenterOfRotation cycleDownCenterOfRotation = new CycleCenterOfRotation(drivebase, Direction.DOWN);
  
  // And the NetworkTable/NetworkTable/CommandChooser variables :)
  private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  private final SendableChooser<Command> drivebaseCommandChooser = new SendableChooser<>();;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    initializeListenersAndSendables();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  { 
    driverController.bButton().whenPressed(() -> drivebase.resetPosition(DrivebaseConstants.STARTING_POSE));
    driverController.aButton().whenPressed(() -> drivebase.resetPosition(DrivebaseConstants.WALL));
    
    driverController.dPadUpButton().whenPressed(cycleUpCenterOfRotation);
    driverController.dPadDownButton().whenPressed(cycleDownCenterOfRotation);
      
    driverController.dPadRightButton().whenPressed(() -> drivebase.setCenterOfRotation(CenterOfRotation.CENTER)); 
  }

  public void initializeListenersAndSendables()
  {
    NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault(); 
    
    // Main Tab
    mainTab.add("Drivebase Subsystem", drivebase); 
    mainTab.add("FL PID", drivebase.getFlWheel().getPIDController());
    mainTab.add("FR PID", drivebase.getFrWheel().getPIDController());
    mainTab.add("BL PID", drivebase.getBlWheel().getPIDController());
    mainTab.add("BR PID", drivebase.getBrWheel().getPIDController());

    // Add options for chooser
    drivebaseCommandChooser.addOption("Westcoast Drive", westcoastDrive);
    drivebaseCommandChooser.addOption("Mecanum Drive", mecanumDrive);
    drivebaseCommandChooser.setDefaultOption("Forward Drive", forwardDrive);
    drivebaseCommandChooser.addOption("Independent Wheel Control*", individualWheelDrive);

    // Places chooser on mainTab (where all configs are)
    mainTab.add(ShuffleboardConstants.DRIVEBASE_CHOOSER, drivebaseCommandChooser); 

    // Create listener for changes in command
    NetworkTableEntry commandChooserEntry = networkTableInstance.getEntry("/Shuffleboard/" + ShuffleboardConstants.DRIVEBASE_CHOOSER + "/selected"); // TODO: check if this is the right namespace
    commandChooserEntry.addListener(
      entry -> {
        drivebaseCommandChooser.getSelected().schedule();
      },
      EntryListenerFlags.kUpdate | EntryListenerFlags.kNew
    );
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;  
  }

  public void initializeTeleopCommands() {
    CommandScheduler.getInstance().cancelAll();
    drivebaseCommandChooser.getSelected().schedule();
  }

  public void teleopPeriodic() {
  
  }

  public MecanumDrive getMecanumDrive() {
      return mecanumDrive;
  }

  public WestcoastDrive getWestcoastDrive() {
      return westcoastDrive;
  }

  public IndividualWheelDrive getIndividualWheelDrive() {
      return individualWheelDrive;
  }

  public PFRController getDriverController() {
    return driverController;
  }

  public PFRController getOperatorController() {
      return operatorController;
  }

}
