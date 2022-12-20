package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class MecanumDrive extends CommandBase {
    
    public enum FrameOfReference { ROBOT, FIELD; }

    private final Drivebase drivebase;
    private final PFRController driverController;
    private FrameOfReference frameOfReference;

    public MecanumDrive(Drivebase drivebase, PFRController driverController)
    {
        this.drivebase = drivebase;
        this.driverController = driverController;
        frameOfReference = FrameOfReference.ROBOT;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        if(driverController.getAButtonPressed()) 
        {
            // allows us to toggle frame of reference when button pressed
            if(frameOfReference == FrameOfReference.ROBOT) 
            {
                frameOfReference = FrameOfReference.FIELD;
            } else {
                frameOfReference = FrameOfReference.ROBOT;
            }
        }
        
        
        
        double xVelocity = Math.pow(driverController.getLeftX(), ControllerConstants.STICK_EXPONENTIAL_CURVE)  * DrivebaseConstants.MAX_LINEAR_VELOCITY;
        double yVelocity = Math.pow(driverController.getLeftY(), ControllerConstants.STICK_EXPONENTIAL_CURVE) * DrivebaseConstants.MAX_LINEAR_VELOCITY; 
        double angularVelocity = Math.pow(driverController.getRightX(), ControllerConstants.STICK_EXPONENTIAL_CURVE) * DrivebaseConstants.MAX_ANGULAR_VELOCITY;

        if(frameOfReference == FrameOfReference.ROBOT)
        {    
            drivebase.setChassisSpeeds(
                xVelocity,
                yVelocity,
                angularVelocity
            );
        }
        else // frame of reference must be field-relative
        {
            drivebase.setFieldRelativeChassisSpeeds(
                xVelocity, 
                yVelocity,
                angularVelocity    
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