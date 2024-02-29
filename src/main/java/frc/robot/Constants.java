package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    /**
     * Once how much time, in seconds, to run the infrequent periodic procedure
     */
    public static final double INFREQUENT_PERIODIC_PERIOD = 0.03;
    public static final double STICK_DEADBAND = 0.1;

    public static final class Transforms {
        /**
         * Physical location of the camera on the robot, relative to the center of the robot. NEEDS TUNING
         */
        public static final Transform3d
                FRONT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(-0.31, -0.13, 0.27), new Rotation3d(0, Units.degreesToRadians(-25), 0)),
                ROBOT_TO_FRONT_CAMERA = FRONT_CAMERA_TO_ROBOT.inverse();
        public static final Transform3d
                REAR_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.11, Swerve.TRACK_WIDTH/2 - 0.05, 0.41),
                new Rotation3d(0, Units.degreesToRadians(-25), 180)),
                ROBOT_TO_REAR_CAMERA = REAR_CAMERA_TO_ROBOT.inverse();
        public static final Translation3d ROBOT_TO_PIVOT = new Translation3d(-0.275, 0, 0.285);

        public static final double SHOOTER_ARM_LENGTH = 0.49;
    }

    public static class VisionConstants {
        public static final String FRONT_CAMERA_NAME = "Front1937";
        public static final String REAR_CAMERA_NAME = "Rear1937";
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 11;
    }

    public static final class ShootingConstants {
        /**
         * An initial presumption to the slope of the path from the robot's
         * shooter to the virtual target
         *
         * Since the slope to the virtual target depends on the time of flight,
         * which itself depends on the slope to the virtual target, we provide
         * an initial guess.
         *
         * The value hereby is arbitrary and it depicts a common value for the
         * slope.
         */
        public static final double DEFAULT_SLOPE_TO_VIRTUAL_TARGET = 0.5;
        public static final int FLYWHEEL_MAX_RPM = 6400;
        /**
         * This table maps virtual shooter slopes to shooter orientations that actually achieve
         * the desired results, based on calibration and experimentation.
         * 
         * To obtain the samples, place the robot at some distance from the target, record the virtual
         * target slope given by the program as a key, and at an arbitrary pitch angle. Adjust it repeatedly
         * until the robot consistently scores with the current slope(i.e. from its current position). Then
         * move the robot to a different position and repeat until the table is complete. A good initial
         * angle is the arc-tangent of the virtual slope.
         */
        public static final InterpolatingTreeMap<Double, Rotation2d> SLOPE_TO_PITCH_MAP = new InterpolatingTreeMap<Double, Rotation2d>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
        
        static {
            SLOPE_TO_PITCH_MAP.put(0.94, Rotation2d.fromDegrees(84));
            SLOPE_TO_PITCH_MAP.put(0.87, Rotation2d.fromDegrees(78.5));
            SLOPE_TO_PITCH_MAP.put(0.77, Rotation2d.fromDegrees(76));
            SLOPE_TO_PITCH_MAP.put(0.67, Rotation2d.fromDegrees(73.5));
            SLOPE_TO_PITCH_MAP.put(0.52, Rotation2d.fromDegrees(65));
            SLOPE_TO_PITCH_MAP.put(0.41, Rotation2d.fromDegrees(61));
            SLOPE_TO_PITCH_MAP.put(0.36, Rotation2d.fromDegrees(58.8));
        }

        public static final InterpolatingTreeMap<Double, Double> SLOPE_TO_VELOCITY_MAP = new InterpolatingTreeMap<Double, Double>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

        static {
            SLOPE_TO_VELOCITY_MAP.put(0.87, 0.85 * FLYWHEEL_MAX_RPM);
            SLOPE_TO_VELOCITY_MAP.put(0.67, 0.86 * FLYWHEEL_MAX_RPM);
            SLOPE_TO_VELOCITY_MAP.put(0.52, 0.88 * FLYWHEEL_MAX_RPM);
            //SLOPE_TO_VELOCITY_MAP.put(0.436, 0.95 * FLYWHEEL_MAX_RPM);
            SLOPE_TO_VELOCITY_MAP.put(0.41, 0.90 * FLYWHEEL_MAX_RPM);
        }

        public static final InterpolatingTreeMap<Double, Double> SLOPE_TO_TIME_OF_FLIGHT_MAP = new InterpolatingTreeMap<Double, Double>(
                InverseInterpolator.forDouble(), Interpolator.forDouble());

        static {
            SLOPE_TO_TIME_OF_FLIGHT_MAP.put(0.87, 0.55);
            SLOPE_TO_TIME_OF_FLIGHT_MAP.put(0.67, 0.6);
            SLOPE_TO_TIME_OF_FLIGHT_MAP.put(0.52, 0.7);
            SLOPE_TO_TIME_OF_FLIGHT_MAP.put(0.41, 0.8);
        }

        public static final double MINIMUM_VIABLE_SLOPE = 0.45;
        public static final double MAXIMUM_VIABLE_SLOPE = 1.22;

        public static final double POSE_HISTORY_DURATION = 0.5;

        public static final int
                FLYWHEEL_LEFT_ID = 15,
                FLYWHEEL_RIGHT_ID = 16,
                PIVOT_ID = 1,
                PIVOT_CAN_CODER = 22,
                KICKER_ID = 8;
        public static final double PIVOT_ENCODER_OFFSET = 283.184;
        public static final double PIVOT_UP_P = 0.03;
        public static final double PIVOT_UP_FF = 0.000055;
        public static final double PIVOT_HIGH_P = 0.02;
        public static final double PIVOT_HIGH_D = 0.02;
        public static final double PIVOT_HIGH_FF = 0.000085;
        public static final double PIVOT_DOWN_P = 0.025;
        public static final double PIVOT_DOWN_FF = 0.0005;
        public static final float PIVOT_CONSTRAINT_DEGREES = 150;
        public static final CANSparkBase.SoftLimitDirection PIVOT_CONSTRAINT_DIRECTION = CANSparkBase.SoftLimitDirection.kForward;

        public static final double FLYWHEEL_FF = 0.000175;
        public static final double FLYWHEEL_RANGE_MIN = -1;
        public static final double FLYWHEEL_RANGE_MAX = 1;
        public static final double FLYWHEEL_P = 0.000919;
        public static final double FLYWHEEL_VELOCITY_TOLERANCE = 150;

        public static final double PIVOT_RANGE_MIN = -0.9;
        public static final double PIVOT_RANGE_MAX = 0.9;

        /** In seconds */
        public static final double SHOOTING_DELAY = 0.5;
        /** In seconds */
        public static final double POST_SHOOTING_DELAY = 0.25;
        public static final int SHOOTER_UTMOST_ANGLE = 220;
        public static final int SHOOTER_VERTICAL_ANGLE = 112;

        public static final double NOTE_RELEASE_VELOCITY = 5.5; //todo: CONFIGURE
        public static final Translation3d BLUE_TARGET_POSITION = new Translation3d(0.2, 5.5, 2.05);
        public static final Translation3d RED_TARGET_POSITION = new Translation3d(16.65, 5.5, 2.05);

        public static final double KICKER_SPEED_BACKWARDS = -0.5;
        public static final double KICKER_SPEED_FORWARD = 0.8;
        public static final double CONSIDERED_NOISELESS_THRESHOLD = 20;
    }

    public static final class Swerve {
        public static final double AZIMUTH_CONTROLLER_P = 16, AZIMUTH_CONTROLLER_I = 0,
                                   AZIMUTH_CONTROLLER_D = 2,
                                   AZIMUTH_CONTROLLER_TOLERANCE = Units.degreesToRadians(3.5);

        public static final int PIGEON_ID = 30;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE =
                COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.565;
        public static final double WHEEL_BASE = 0.615;
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

        /* SwerveSubsystem Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = false;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERT = CHOSEN_MODULE.canCoderInvert;

        /* SwerveSubsystem Current Limiting */
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KFF = 0;
        public static final double ANGLE_KP = 0.01;
        public static final double ANGLE_KI = 0;
        public static final double ANGLE_KD = 0;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.001;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double DRIVE_KS = 0; //(0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = 0; //(1.51 / 12);
        public static final double DRIVE_KA = 0; // (0.27 / 12);

        /* SwerveSubsystem Profiling Values */
        /**
         * Meters per Second
         */
        public static final double MAX_SPEED = 5; //TODO: This must be tuned to specific robot
        /**
         * Radians per Second
         */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final com.revrobotics.CANSparkBase.IdleMode ANGLE_NEUTRAL_MODE = CANSparkBase.IdleMode.kBrake;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Module0 {
            public static final int DRIVE_MOTOR_ID = 14;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CAN_CODER_ID = 18;//178.594-247.140
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(246.885);//247.140-178.594+180);//243.55);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Module1 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CAN_CODER_ID = 20;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(100.898);//283.799-358.24+180);//103.45);//;
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Module2 {
            public static final int DRIVE_MOTOR_ID = 13;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 19;//
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(190.459);//190.283-359.209);//188.23);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Module3 {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 21; //179.385-294.609
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(115.840);//294.609-179.385);//116.46);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        public static final boolean ANGLE_INVERT = true;
        public static final double VOLTAGE_COMP = 12.0;
        public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

        public static final HolonomicPathFollowerConfig holomonicPathFollowerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(1.366, 0.0, 0.0), // Translation PID constants
                new PIDConstants(AZIMUTH_CONTROLLER_P, AZIMUTH_CONTROLLER_I, AZIMUTH_CONTROLLER_D), // Rotation PID constants
                Constants.Swerve.MAX_SPEED, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig());
    //todo: shoot 60 angles. Joystick adjustable shooter angle
        public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
            public static final double MAX_SPEED_METERS_PER_SECOND = 3;
            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
            public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
            public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

            public static final double PX_CONTROLLER = 1;
            public static final double PY_CONTROLLER = 1;
            public static final double P_THETA_CONTROLLER = 1;

            public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
                    new TrapezoidProfile.Constraints(
                            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

            /**
             * We avoid steering the swerve wheels below a certain driving speed, for in-place turning
             * causes them to jitter. Thus, we hereby define the maximum driving speed that is
             * considered 'in-place'.
             */
            public static final double SWERVE_IN_PLACE_DRIVE_MPS = 0.1;
        }
    }

    public static final class NavigationConstants {
        public static final Pose2d AMPLIFIER_POSE = new Pose2d(1.83, 7.7, Rotation2d.fromDegrees(-90));
        public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(3.5, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
    }
}
