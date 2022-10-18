// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.commands.drivebase.CycleCenterOfRotation;
import frc.robot.commands.drivebase.MeccanumDrive;
import frc.robot.commands.drivebase.CycleCenterOfRotation.Direction;
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
  private final PFRController operatorController = new PFRController(1);
  private final PFRController driverController = new PFRController(0);

  // and the robot's commands are defined here!
  private final MeccanumDrive meccanumDrive = new MeccanumDrive(drivebase, driverController);
  private final CycleCenterOfRotation cycleUpCenterOfRotation = new CycleCenterOfRotation(drivebase, Direction.UP);
  private final CycleCenterOfRotation cycleDownCenterOfRotation = new CycleCenterOfRotation(drivebase, Direction.DOWN);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverController.dPadUpButton().whenPressed(cycleUpCenterOfRotation);
    driverController.dPadDownButton().whenPressed(cycleDownCenterOfRotation);
    driverController.dPadRightButton().whenPressed(() -> drivebase.setCenterOfRotation(CenterOfRotation.CENTER));
    driverController.bButton().whenPressed(() -> drivebase.resetPosition(DrivebaseConstants.STARTING_POSE));
    driverController.aButton().whenPressed(() -> drivebase.resetPosition(DrivebaseConstants.WALL));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;  
  }

  public void teleopCommandInit() {
    meccanumDrive.initialize();
  }

  public MeccanumDrive getMeccanumDrive() {
      return meccanumDrive;
  }

  public PFRController getDriverController() {
    return driverController;
  }

  public PFRController getOperatorController() {
      return operatorController;
  }

}
