package frc.robot.subsystems;

import java.text.DecimalFormat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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
        FRONT_CENTER(DrivebaseConstants.FRONT_CENTER_LOCATION),
        CENTER(new Translation2d());

        private Translation2d location;

        private CenterOfRotation(Translation2d location)
        {
            this.location = location;
        }

        /** 
         * Where the center of rotation is relative to dead center of the robot
         * @return Translation2d of the position
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
    // private DigitalInput drivebaseSwitch; // The physical switch to toggle between 'Mecanum' and 'West-Coast'

    private MecanumDriveKinematics kinematics; // Everything we use to track the robot's location and behavior
    private MecanumDriveOdometry odometry;
    private ChassisSpeeds actualChassisSpeeds;
    private ChassisSpeeds desiredChassisSpeeds; // The Speeds we want to the set the robot to

    private CenterOfRotation centerOfRotation; // Where the mecanum drive will rotate around

    private final ShuffleboardTab drivebaseTab; // The shuffleboard tab we are using for TELEMETRY

    private boolean isIndependentWheelControl = false; // Whether we are setting the entire drivebase, or each wheel individually

    private double flSpeed, frSpeed, blSpeed, brSpeed = 0;

    private NetworkTableEntry actualFLVelocityEntry, actualFRVelocityEntry, actualBLVelocityEntry, actualBRVelocityEntry, actualXVelocityEntry, actualYVelocityEntry, actualRotationalVelocityEntry;
    private NetworkTableEntry desiredFLVelocityEntry, desiredFRVelocityEntry, desiredBLVelocityEntry, desiredBRVelocityEntry, desiredXVelocityEntry, desiredYVelocityEntry, desiredRotationalVelocityEntry;

    public Drivebase()
    {
        flWheel = new Motor(DrivebaseConstants.WHEEL_FL_PORT, DrivebaseConstants.WHEEL_FL_REVERSED, DrivebaseConstants.GEAR_RATIO, DrivebaseConstants.WHEEL_DIAMETER, DrivebaseConstants.POSITION_PID, DrivebaseConstants.VELOCITY_PID, DrivebaseConstants.FEEDFORWARD);
        frWheel = new Motor(DrivebaseConstants.WHEEL_FR_PORT, DrivebaseConstants.WHEEL_FR_REVERSED, DrivebaseConstants.GEAR_RATIO, DrivebaseConstants.WHEEL_DIAMETER, DrivebaseConstants.POSITION_PID, DrivebaseConstants.VELOCITY_PID, DrivebaseConstants.FEEDFORWARD);
        blWheel = new Motor(DrivebaseConstants.WHEEL_BL_PORT, DrivebaseConstants.WHEEL_BL_REVERSED, DrivebaseConstants.GEAR_RATIO, DrivebaseConstants.WHEEL_DIAMETER, DrivebaseConstants.POSITION_PID, DrivebaseConstants.VELOCITY_PID, DrivebaseConstants.FEEDFORWARD);
        brWheel = new Motor(DrivebaseConstants.WHEEL_BR_PORT, DrivebaseConstants.WHEEL_BR_REVERSED, DrivebaseConstants.GEAR_RATIO, DrivebaseConstants.WHEEL_DIAMETER, DrivebaseConstants.POSITION_PID, DrivebaseConstants.VELOCITY_PID, DrivebaseConstants.FEEDFORWARD);

        gyro = new ADXRS450_Gyro();

        // drivebaseSwitch = new DigitalInput(0);
        
        // Creates the kinematics 
        kinematics = new MecanumDriveKinematics(
            DrivebaseConstants.WHEEL_FL_LOCATION, 
            DrivebaseConstants.WHEEL_FR_LOCATION, 
            DrivebaseConstants.WHEEL_BL_LOCATION, 
            DrivebaseConstants.WHEEL_BR_LOCATION
        );

        // Creates the odometry
        odometry = new MecanumDriveOdometry(
            kinematics, 
            gyro.getRotation2d(), 
            DrivebaseConstants.STARTING_POSE
        );

        desiredChassisSpeeds = new ChassisSpeeds(); // completely empty chassis speeds (all 0's)

        centerOfRotation = CenterOfRotation.CENTER;

        drivebaseTab = Shuffleboard.getTab("Drivebase");


        // softwareDrivebaseSwitch = tab.add("Drivebase Switch", false).getEntry();

        actualFLVelocityEntry = drivebaseTab.add("Actual FL Wheel Velocity", 0).getEntry();
        actualFRVelocityEntry = drivebaseTab.add("Actual FR Wheel Velocity", 0).getEntry();
        actualBLVelocityEntry = drivebaseTab.add("Actual BL Wheel Velocity", 0).getEntry();
        actualBRVelocityEntry = drivebaseTab.add("Actual BR Wheel Velocity", 0).getEntry();
        
        actualXVelocityEntry = drivebaseTab.add("Actual X Velocity", 0).getEntry();
        actualYVelocityEntry = drivebaseTab.add("Actual Y Velocity", 0).getEntry();
        actualRotationalVelocityEntry = drivebaseTab.add("Actual Rotational Velocity", 0).getEntry();

        desiredFLVelocityEntry = drivebaseTab.add("Desired FL Wheel Velocity", 0).getEntry();
        desiredFRVelocityEntry = drivebaseTab.add("Desired FR Wheel Velocity", 0).getEntry();
        desiredBLVelocityEntry = drivebaseTab.add("Desired BL Wheel Velocity", 0).getEntry();
        desiredBRVelocityEntry = drivebaseTab.add("Desired BR Wheel Velocity", 0).getEntry();

        desiredXVelocityEntry = drivebaseTab.add("Desired X Velocity", 0).getEntry();
        desiredYVelocityEntry = drivebaseTab.add("Desired Y Velocity", 0).getEntry();
        desiredRotationalVelocityEntry = drivebaseTab.add("Desired Rotational Velocity", 0).getEntry();

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

        // Gives us the values that the chassis is moving at (velocities, directions)
        actualChassisSpeeds = kinematics.toChassisSpeeds(actualWheelSpeeds);

        // Updates where we are on the field        
        odometry.update(gyro.getRotation2d(), actualWheelSpeeds); 
    
        // Updates the velocities sent to each wheel's PID
        MecanumDriveWheelSpeeds desiredWheelSpeeds = kinematics.toWheelSpeeds(desiredChassisSpeeds, centerOfRotation.get());

        // Scales the values to prevent values from being too high
        desiredWheelSpeeds.desaturate(DrivebaseConstants.MAX_OBTAINABLE_WHEEL_VELOCITY);
        if(isIndependentWheelControl)
        {
            flWheel.set(flSpeed);
            frWheel.set(frSpeed);
            blWheel.set(blSpeed);
            brWheel.set(brSpeed);
        }
        else
        {
            flWheel.setMetersPerSecond(desiredWheelSpeeds.frontLeftMetersPerSecond);
            frWheel.setMetersPerSecond(desiredWheelSpeeds.frontRightMetersPerSecond);
            blWheel.setMetersPerSecond(desiredWheelSpeeds.rearLeftMetersPerSecond);
            brWheel.setMetersPerSecond(desiredWheelSpeeds.rearRightMetersPerSecond);
        }
        // Publishes the data to the Shuffleboard Tab
        actualFLVelocityEntry.setDouble(actualWheelSpeeds.frontLeftMetersPerSecond);
        actualFRVelocityEntry.setDouble(actualWheelSpeeds.frontRightMetersPerSecond);
        actualBLVelocityEntry.setDouble(actualWheelSpeeds.rearLeftMetersPerSecond);
        actualBRVelocityEntry.setDouble(actualWheelSpeeds.rearRightMetersPerSecond);
        
        actualXVelocityEntry.setDouble(actualChassisSpeeds.vxMetersPerSecond);
        actualYVelocityEntry.setDouble(actualChassisSpeeds.vyMetersPerSecond);
        actualRotationalVelocityEntry.setDouble(actualChassisSpeeds.omegaRadiansPerSecond * 180 / Math.PI);

        desiredFLVelocityEntry.setDouble(desiredWheelSpeeds.frontLeftMetersPerSecond);
        desiredFRVelocityEntry.setDouble(desiredWheelSpeeds.frontRightMetersPerSecond);
        desiredBLVelocityEntry.setDouble(desiredWheelSpeeds.rearLeftMetersPerSecond);
        desiredBRVelocityEntry.setDouble(desiredWheelSpeeds.rearRightMetersPerSecond);

        desiredXVelocityEntry.getDouble(desiredChassisSpeeds.vxMetersPerSecond);
        desiredFLVelocityEntry.getDouble(desiredChassisSpeeds.vyMetersPerSecond);
        desiredRotationalVelocityEntry.getDouble(desiredChassisSpeeds.omegaRadiansPerSecond * 180 / Math.PI);
        

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

    public void setIndependentWheelPercentages(double[] independentWheelPercentages) {
        flSpeed = independentWheelPercentages[0];
        frSpeed = independentWheelPercentages[1];
        blSpeed = independentWheelPercentages[2];
        brSpeed = independentWheelPercentages[3];
    }

    public void setIndependentWheelPercentages(double flSpeed, double frSpeed, double blSpeed, double brSpeed)
    {
        this.flSpeed = flSpeed;
        this.frSpeed = frSpeed;
        this.blSpeed = blSpeed;
        this.brSpeed = brSpeed;
    }
    /**
     * Sets the "proportational" variable for the PID in each motor
     * 
     * @param kp what to set each motor's "proportional" parameter to
     */
    public void setP(double kp)
    {
        flWheel.setVelocityP(kp);
        frWheel.setVelocityP(kp);
        blWheel.setVelocityP(kp);
        brWheel.setVelocityP(kp); 
    }

    /**
     * Sets the "derivative" variable (slope at one point) for the PID in each motor
     * 
     * @param kd what to set each motor's "proportional" parameter to
     */
    public void setD(double kd)
    {
        flWheel.setVelocityD(kd);
        frWheel.setVelocityD(kd);
        blWheel.setVelocityD(kd);
        brWheel.setVelocityD(kd);
    }

    /**
     * Sets the position of the robot to a particular position and rotation relative to the field
     * @param poseMeters The position on the field that your robot is at.
     */
    public void resetPosition(Pose2d poseMeters) {
        odometry.resetPosition(poseMeters, gyro.getRotation2d());
    }

    /**
     * Stops the robot
     */
    public void stop() {
        desiredChassisSpeeds = new ChassisSpeeds();
    }

    /**
     * sets current heading as the "zero"
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Sets where the robot will rotate
     * @return the enum value that contains position
     */
    public CenterOfRotation getCenterOfRotation() {
        return centerOfRotation;
    }

    /**
     * Returns the position of the robot on the field
     * @return The pose2d of the robot (in meters)
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * @return difference in angle since last reset
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public void setIndependentWheelControl(boolean isIndependentWheelControl) {
        this.isIndependentWheelControl = isIndependentWheelControl;
    }
    public boolean isIndependentWheelControl() {
        return isIndependentWheelControl;
    }

    public ShuffleboardTab getDrivebaseTab() {
        return drivebaseTab;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Velocity kP", flWheel.getPIDController()::getP, this::setP);
        builder.addDoubleProperty("Velocity kD", flWheel.getPIDController()::getD, this::setD);
    }
}
