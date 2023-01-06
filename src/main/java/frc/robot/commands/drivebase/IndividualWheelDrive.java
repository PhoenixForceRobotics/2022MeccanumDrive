package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class IndividualWheelDrive extends CommandBase{
    private final Drivebase drivebase;
    private final PFRController frontDriverController;
    private final PFRController backDriverController; // Need 4 y-axis to control each wheel individually

    public IndividualWheelDrive(Drivebase drivebase, PFRController frontDriverController, PFRController backDriverController)
    {
        this.drivebase = drivebase;
        this.frontDriverController = frontDriverController;
        this.backDriverController = backDriverController;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        drivebase.setIndependentWheelControl(true);
    }
    
    @Override
    public void execute() {
        // Find values for each wheel and apply them
        double flSpeed = frontDriverController.getLeftYSquared();
        double frSpeed = frontDriverController.getRightYSquared();
        double blSpeed = backDriverController.getLeftYSquared();
        double brSpeed = backDriverController.getRightYSquared();
        drivebase.setIndependentWheelPercentages(
            flSpeed, 
            frSpeed,
            blSpeed, 
            brSpeed
        );
    }

    @Override
    public void end(boolean interrupted) {
        //stop all wheels 
        drivebase.setIndependentWheelPercentages(0, 0, 0, 0);
    }
}
