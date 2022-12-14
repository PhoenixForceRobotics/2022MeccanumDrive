// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final class DrivebaseConstants {
        public static final double GEAR_RATIO = 72 / 12; // output / input
        public static final double WHEEL_DIAMETER = 0.1524; // In meters (6 in wheels)
        public static final PIDController VELOCITY_PID = new PIDController(1, 0, 0); // Controls feedback loop for maintaining velocity
        public static final PIDController POSITION_PID = new PIDController(1, 0, 0); // Controls feedback loop for maintaing position
        public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(12, 1); //TODO: actually characterize drivebase

        public static final int WHEEL_FL_PORT = 1;
        public static final int WHEEL_FR_PORT = 2;
        public static final int WHEEL_BL_PORT = 3;
        public static final int WHEEL_BR_PORT = 4;

        public static final boolean WHEEL_FL_REVERSED = false;
        public static final boolean WHEEL_FR_REVERSED = false;
        public static final boolean WHEEL_BL_REVERSED = false;
        public static final boolean WHEEL_BR_REVERSED = false;

        public static final Translation2d WHEEL_FL_LOCATION = new Translation2d(0.381, 0.381);
        public static final Translation2d WHEEL_FR_LOCATION = new Translation2d(0.381, -0.381);
        public static final Translation2d WHEEL_BL_LOCATION = new Translation2d(-0.381, 0.381);
        public static final Translation2d WHEEL_BR_LOCATION = new Translation2d(-0.381, -0.381);
        public static final Translation2d FRONT_CENTER_LOCATION = new Translation2d(0, 0.381); // Helpful for evassive manuvers

        public static final HolonomicDriveController HOLONOMIC_DRIVE_CONTROLLER = new HolonomicDriveController(
            VELOCITY_PID, // PID to control error in the x direction
            POSITION_PID, // PID to control error in the y direction
            new ProfiledPIDController(1, 0, 0, // PID to controller error in angle
                new TrapezoidProfile.Constraints(6.28, 3.14))
        );
        // Here, our rotation profile constraints were a max velocity
        // of 1 rotation per second and a max acceleration of 180 degrees
        // per second squared.

        public static final Pose2d STARTING_POSE = new Pose2d(
            0, 
            0, 
            new Rotation2d()
        );

        public static final Pose2d WALL = new Pose2d(
            -1,
            -1,
            new Rotation2d()
        );

        public static final double MAX_OBTAINABLE_WHEEL_VELOCITY = 1; // Absolute max speed of a single wheel (meters per second) *Used for desaturation*
        public static final double MAX_LINEAR_VELOCITY = 1; // Desired max speed of the chassis (meters per second)
        public static final double MAX_ANGULAR_VELOCITY = 1; // radians per second
    }

    public static final class ControllerConstants {
        public static final double AXIS_DEADZONE = 0.05;
        public static final int DPAD_UP = 0;
        public static final int DPAD_RIGHT = 90;
        public static final int DPAD_DOWN = 180;
        public static final int DPAD_LEFT = 270;

        public static final int STICK_EXPONENTIAL_CURVE = 2;
    }

    public static final class UtilConstants {
        public static final double FALCON_ENCODER_RESOLUTION = 2048;
        public static final int CLOSED_LOOP_SPEED_MS = 1; // in milliseconds
      }
}
