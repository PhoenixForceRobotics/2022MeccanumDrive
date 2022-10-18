package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class MeccanumDrive extends CommandBase {
    
    public enum FrameOfReference { ROBOT, FIELD; }

    private final Drivebase drivebase;
    private final PFRController driverController;
    private FrameOfReference frameOfReference;

    public MeccanumDrive(Drivebase drivebase, PFRController driverController)
    {
        this.drivebase = drivebase;
        this.driverController = driverController;
        frameOfReference = FrameOfReference.ROBOT;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        if(driverController.getAButtonPressed())
        {
            switch (frameOfReference) {
                case ROBOT:
                    frameOfReference = FrameOfReference.FIELD;
                    break;
                default:
                    frameOfReference = FrameOfReference.ROBOT;
                    break;
            }
        }

        if(frameOfReference == FrameOfReference.ROBOT)
        {    
            drivebase.setChassisSpeeds(
                Math.pow(driverController.getLeftX(), ControllerConstants.STICK_EXPONENTIAL_CURVE)  * DrivebaseConstants.MAX_LINEAR_VELOCITY, 
                Math.pow(driverController.getLeftY(), ControllerConstants.STICK_EXPONENTIAL_CURVE) * DrivebaseConstants.MAX_LINEAR_VELOCITY, 
                Math.pow(driverController.getRightX(), ControllerConstants.STICK_EXPONENTIAL_CURVE) * DrivebaseConstants.MAX_ANGULAR_VELOCITY
            );
        }
        else
        {
            drivebase.setFieldRelativeChassisSpeeds(
                Math.pow(driverController.getLeftX(), ControllerConstants.STICK_EXPONENTIAL_CURVE) * DrivebaseConstants.MAX_LINEAR_VELOCITY, 
                Math.pow(driverController.getLeftY(), ControllerConstants.STICK_EXPONENTIAL_CURVE) * DrivebaseConstants.MAX_LINEAR_VELOCITY, 
                Math.pow(driverController.getRightX(), ControllerConstants.STICK_EXPONENTIAL_CURVE) * DrivebaseConstants.MAX_ANGULAR_VELOCITY
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.stop();
    }
}