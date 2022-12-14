package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class IndividualWheelControl extends CommandBase{
    private final Drivebase drivebase;
    private final PFRController frontDriverController;
    private final PFRController backDriverController; // Need 4 y-axis to control each wheel individually

    public IndividualWheelControl(Drivebase drivebase, PFRController frontDriverController, PFRController backDriverController)
    {
        this.drivebase = drivebase;
        this.frontDriverController = frontDriverController;
        this.backDriverController = backDriverController;
    }

    @Override
    public void execute() {
        // Find values for each wheel and apply them
        double flSpeed = Math.pow(frontDriverController.getLeftY(), ControllerConstants.STICK_EXPONENTIAL_CURVE);
        double frSpeed = Math.pow(frontDriverController.getRightY(), ControllerConstants.STICK_EXPONENTIAL_CURVE);
        double blSpeed = Math.pow(backDriverController.getLeftY(), ControllerConstants.STICK_EXPONENTIAL_CURVE);
        double brSpeed = Math.pow(backDriverController.getRightY(), ControllerConstants.STICK_EXPONENTIAL_CURVE);
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
