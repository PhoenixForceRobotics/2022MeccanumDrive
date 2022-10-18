package frc.robot.subsystems;

import java.text.DecimalFormat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.utils.Motor;

public class Drivebase extends SubsystemBase {

    public enum CenterOfRotation 
    { 
        FL_WHEEL(DrivebaseConstants.WHEEL_FL_LOCATION), 
        FR_WHEEL(DrivebaseConstants.WHEEL_FR_LOCATION), 
        BL_WHEEL(DrivebaseConstants.WHEEL_BL_LOCATION), 
        BR_WHEEL(DrivebaseConstants.WHEEL_BR_LOCATION), 
        CENTER(new Translation2d());

        private Translation2d location;

        private CenterOfRotation(Translation2d location)
        {
            this.location = location;
        }

        /** 
         * Where the center of rotation is relative to dead center of the robot
         */
        public Translation2d get()
        {
            return location;
        }
    
    }
    private Motor flWheel;
    private Motor frWheel;
    private Motor blWheel;
    private Motor brWheel;
    private Gyro gyro;
    private DigitalInput drivebaseSwitch; // The physical switch to toggle between 'Meccanum' and 'West-Coast'

    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;
    private ChassisSpeeds actualChassisSpeeds;
    private ChassisSpeeds desiredChassisSpeeds; // The Speeds we want to the set the robot to

    private CenterOfRotation centerOfRotation;

    private ShuffleboardTab tab;

    private DecimalFormat decimalFormatter;


    public Drivebase()
    {
        flWheel = new Motor(DrivebaseConstants.WHEEL_FL_PORT, DrivebaseConstants.WHEEL_FL_REVERSED, DrivebaseConstants.GEAR_RATIO, DrivebaseConstants.WHEEL_DIAMETER, DrivebaseConstants.POSITION_PID, DrivebaseConstants.VELOCITY_PID);
        frWheel = new Motor(DrivebaseConstants.WHEEL_FR_PORT, DrivebaseConstants.WHEEL_FR_REVERSED, DrivebaseConstants.GEAR_RATIO, DrivebaseConstants.WHEEL_DIAMETER, DrivebaseConstants.POSITION_PID, DrivebaseConstants.VELOCITY_PID);
        blWheel = new Motor(DrivebaseConstants.WHEEL_BL_PORT, DrivebaseConstants.WHEEL_BL_REVERSED, DrivebaseConstants.GEAR_RATIO, DrivebaseConstants.WHEEL_DIAMETER, DrivebaseConstants.POSITION_PID, DrivebaseConstants.VELOCITY_PID);
        brWheel = new Motor(DrivebaseConstants.WHEEL_BR_PORT, DrivebaseConstants.WHEEL_BR_REVERSED, DrivebaseConstants.GEAR_RATIO, DrivebaseConstants.WHEEL_DIAMETER, DrivebaseConstants.POSITION_PID, DrivebaseConstants.VELOCITY_PID);

        gyro = new ADXRS450_Gyro();

        drivebaseSwitch = new DigitalInput(0);
        
        kinematics = new MecanumDriveKinematics(
            DrivebaseConstants.WHEEL_FL_LOCATION, 
            DrivebaseConstants.WHEEL_FR_LOCATION, 
            DrivebaseConstants.WHEEL_BL_LOCATION, 
            DrivebaseConstants.WHEEL_BR_LOCATION
        );

        odometry = new MecanumDriveOdometry(
            kinematics, 
            gyro.getRotation2d(), 
            DrivebaseConstants.STARTING_POSE
        );

        desiredChassisSpeeds = new ChassisSpeeds();

        centerOfRotation = CenterOfRotation.CENTER;

        tab = Shuffleboard.getTab("Drivebase Subsystem");

        decimalFormatter = new DecimalFormat("##.##");
    }

    @Override
    public void periodic() {
        // Update the position of the robot
        MecanumDriveWheelSpeeds actualWheelSpeeds = new MecanumDriveWheelSpeeds(
            flWheel.getMetersPerSecond(),
            frWheel.getMetersPerSecond(),
            blWheel.getMetersPerSecond(),
            frWheel.getMetersPerSecond()
        );

        tab.add("Actual FL Wheel Velocity", decimalFormatter.format(actualWheelSpeeds.frontLeftMetersPerSecond) + " m/s");
        tab.add("Actual FR Wheel Velocity", decimalFormatter.format(actualWheelSpeeds.frontRightMetersPerSecond) + " m/s");
        tab.add("Actual BL Wheel Velocity", decimalFormatter.format(actualWheelSpeeds.rearLeftMetersPerSecond) + " m/s");
        tab.add("Actual BR Wheel Velocity", decimalFormatter.format(actualWheelSpeeds.rearRightMetersPerSecond) + " m/s");

        actualChassisSpeeds = kinematics.toChassisSpeeds(actualWheelSpeeds);

        tab.add("Actual X Velocity", decimalFormatter.format(actualChassisSpeeds.vxMetersPerSecond) + " m/s");
        tab.add("Actual Y Velocity", decimalFormatter.format(actualChassisSpeeds.vyMetersPerSecond) + " m/s");
        tab.add("Actual Rotational Velocity", decimalFormatter.format((actualChassisSpeeds.omegaRadiansPerSecond * 180 / Math.PI)) + " deg/s");

        Pose2d currentPose = odometry.update(gyro.getRotation2d(), actualWheelSpeeds); // Where we are on the field        
    
        // Updates the velocity
        MecanumDriveWheelSpeeds desiredWheelSpeeds =kinematics.toWheelSpeeds(desiredChassisSpeeds, centerOfRotation.get());

        desiredWheelSpeeds.desaturate(DrivebaseConstants.MAX_OBTAINABLE_WHEEL_VELOCITY);
        
        flWheel.setMetersPerSecond(desiredWheelSpeeds.frontLeftMetersPerSecond);
        frWheel.setMetersPerSecond(desiredWheelSpeeds.frontRightMetersPerSecond);
        blWheel.setMetersPerSecond(desiredWheelSpeeds.rearLeftMetersPerSecond);
        brWheel.setMetersPerSecond(desiredWheelSpeeds.rearRightMetersPerSecond);

        tab.add("Desired FL Wheel Velocity", desiredWheelSpeeds.frontLeftMetersPerSecond);
        tab.add("Desired FR Wheel Velocity", desiredWheelSpeeds.frontRightMetersPerSecond);
        tab.add("Desired BL Wheel Velocity", desiredWheelSpeeds.rearLeftMetersPerSecond);
        tab.add("Desired BR Wheel Velocity", desiredWheelSpeeds.rearRightMetersPerSecond);
    }
    
    public void setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds) {
        this.desiredChassisSpeeds = desiredChassisSpeeds;
    }

    public void setChassisSpeeds(double vx, double vy, double theta ) {
        desiredChassisSpeeds = new ChassisSpeeds(vx, vy, theta);
    }

    public void setFieldRelativeChassisSpeeds(ChassisSpeeds desiredChassisSpeeds) {
        setFieldRelativeChassisSpeeds(desiredChassisSpeeds.vxMetersPerSecond, desiredChassisSpeeds.vyMetersPerSecond, desiredChassisSpeeds.omegaRadiansPerSecond);
    }

    public void setFieldRelativeChassisSpeeds(double vx, double vy, double theta) {
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, theta, gyro.getRotation2d());
    }


    public void setCenterOfRotation(CenterOfRotation centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
    }

    public void resetPosition(Pose2d poseMeters) {
        odometry.resetPosition(poseMeters, gyro.getRotation2d());
    }

    public void stop() {
        desiredChassisSpeeds = new ChassisSpeeds();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public CenterOfRotation getCenterOfRotation() {
        return centerOfRotation;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }
}
