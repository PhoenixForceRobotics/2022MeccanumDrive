package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class WestcoastDrive extends CommandBase {
    private final Drivebase drivebase;
    private final PFRController driverController;

    public WestcoastDrive(Drivebase drivebase, PFRController driverController)
    {
        this.drivebase = drivebase;
        this.driverController = driverController;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // double linearVelocity = Math.pow(driverController.getLeftY(), ControllerConstants.STICK_EXPONENTIAL_CURVE) * DrivebaseConstants.MAX_LINEAR_VELOCITY;
        // double rotationalVelocity = Math.pow(driverController.getRightX(), ControllerConstants.STICK_EXPONENTIAL_CURVE) * DrivebaseConstants.MAX_ANGULAR_VELOCITY;
        
        // drivebase.setChassisSpeeds(
        //     linearVelocity, 
        //     0, // No horizontal velocity possible
        //     rotationalVelocity
        // );
        double xSpeed = Math.pow(driverController.getLeftY(), ControllerConstants.STICK_EXPONENTIAL_CURVE);
        double rotationSpeed = Math.pow(driverController.getRightX(), ControllerConstants.STICK_EXPONENTIAL_CURVE);
        WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(xSpeed, rotationSpeed, false);

        drivebase.setIndependentWheelPercentages(
            wheelSpeeds.left,
            wheelSpeeds.right,
            wheelSpeeds.left,
            wheelSpeeds.right
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.stop();
    }
}
