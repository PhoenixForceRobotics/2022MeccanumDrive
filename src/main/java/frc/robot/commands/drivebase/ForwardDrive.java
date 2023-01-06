package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class ForwardDrive extends CommandBase {
    
    private final Drivebase drivebase;
    private final PFRController driverController;

    public ForwardDrive(Drivebase drivebase, PFRController driverController)
    {
        this.drivebase = drivebase;
        this.driverController = driverController;
        addRequirements(drivebase);
    }
    @Override
    public void initialize() {
        drivebase.setIndependentWheelControl(false);
    }

    @Override
    public void execute() {
        double linearVelocity = driverController.getLeftYSquared() * DrivebaseConstants.MAX_LINEAR_VELOCITY;        
        drivebase.setChassisSpeeds(
            linearVelocity, 
            0, // No horizontal velocity possible
            0 // No turning
        );
    }

}