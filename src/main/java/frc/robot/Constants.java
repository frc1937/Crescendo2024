package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
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
    public static final double INFREQUENT_PERIODIC_PERIOD = 0.1;
    public static final double STICK_DEADBAND = 0.1;

    public static final class ChaseTagPIDConstants {
        public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
        public static final PIDController X_CONTROLLER = new PIDController(2, 0, 0);
        public static final PIDController Y_CONTROLLER = new PIDController(2, 0, 0);
        public static final ProfiledPIDController OMEGA_CONTROLLER = new ProfiledPIDController(4, 0, 0, OMEGA_CONSTRAINTS);

        static {
            X_CONTROLLER.setTolerance(0.2);
            Y_CONTROLLER.setTolerance(0.2);
            OMEGA_CONTROLLER.setTolerance(Units.degreesToRadians(3));
            OMEGA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        }
    }

    public static final class Transforms {
        /**
         * Physical location of the camera on the robot, relative to the center of the robot. NEEDS TUNING
         */
        public static final Transform3d
                CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.36, 0.39, 0.41), new Rotation3d()),
                ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse(),
                ROBOT_TO_PIVOT = new Transform3d(new Translation3d(-0.275, 0, 0.15), new Rotation3d());
    }

    public static class VisionConstants {
        public static final String CAMERA_NAME = "Ph1937";
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 11;
        public static final double INTAKE_SPEED = 0.8;
    }

    public static final class ShootingConstants {
        /**
         * This table maps theoretical (quasi) shooter slopes to shooter orientations that actually achieve
         * the desired results, based on calibration and experimentation.
         * <p>
         * To obtain the samples, place the robot at some distance from the target, record the theoretical
         * target slope given by the program as a key, and shoot with an arbitrary angle value. Shoot at an
         * arbitrary angle and adjust it repeatedly until the robot consistently scores with the current slope(i.e.
         * from its current position). Then move the robot to a different position and repeat until the table is
         * complete. A good initial angle is the arc-tangent of the theoretical slope.
         */
        public static final InterpolatingTreeMap<Double, Rotation2d> SLOPE_TO_SHOOTER_ROTATION_MAP = new InterpolatingTreeMap<Double, Rotation2d>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

        static {
            // TODO Currently, the values here are the initial guesses and are yet
            // to be calibrated
            SLOPE_TO_SHOOTER_ROTATION_MAP.put(0.2, Rotation2d.fromDegrees(15 + 11));
            SLOPE_TO_SHOOTER_ROTATION_MAP.put(0.37, Rotation2d.fromDegrees(60));
            //SLOPE_TO_SHOOTER_ROTATION_MAP.put(0.4, Rotation2d.fromDegrees(70)); //

        }

        public static final double POSE_HISTORY_DURATION = 0.5;

        public static final int
                FLYWHEEL_LEFT_ID = 15,
                FLYWHEEL_RIGHT_ID = 16,
                PIVOT_ID = 1,
                PIVOT_CAN_CODER = 22,
                KICKER_ID = 8;
        public static final double FLYWHEEL_MINIMUM_READY_RPM = 0.8*5600;
        public static final double PIVOT_ENCODER_OFFSET = 283.184;
        public static final double PIVOT_UP_P = 0.035;
        public static final double PIVOT_UP_FF = 0.000085;
        public static final float PIVOT_CONSTRAINT_DEGREES = 150;
        public static final CANSparkBase.SoftLimitDirection PIVOT_CONSTRAINT_DIRECTION = CANSparkBase.SoftLimitDirection.kForward;

        public static final double PIVOT_DOWN_P = 0.013_94;
        public static final double PIVOT_DOWN_FF = 0.00121;
        public static final double FLYWHEEL_FF = 0.000_065_955;
        public static final double FLYWHEEL_RANGE_MIN = -1;
        public static final double FLYWHEEL_RANGE_MAX = 1;
        public static final double FLYWHEEL_P = 0.001_159;

        public static final double PIVOT_RANGE_MIN = -0.9;
        public static final double PIVOT_RANGE_MAX = 0.9;

        public static final double FLYWHEEL_SPEED = 0.7;
        /** In seconds */
        public static final double SHOOTING_DELAY = 0.5;
        /** In seconds */
        public static final double POST_SHOOTING_DELAY = 0.4;

        public static final double NOTE_RELEASE_VELOCITY = 0.5; //todo: CONFIGURE
        public static final Translation3d TARGET_POSITION = new Translation3d(0.9, 0, 2);

    }

    public static final class Swerve {
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
        public static final double MAX_SPEED = 4.5; //TODO: This must be tuned to specific robot
        /**
         * Radians per Second
         */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final com.revrobotics.CANSparkBase.IdleMode ANGLE_NEUTRAL_MODE = com.revrobotics.CANSparkBase.IdleMode.kCoast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Module0 {
            public static final int DRIVE_MOTOR_ID = 14;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CAN_CODER_ID = 18;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(243.55);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Module1 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CAN_CODER_ID = 20;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(105.45);//281.51);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Module2 {
            public static final int DRIVE_MOTOR_ID = 13;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 19;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(188.23);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Module3 {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 21;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(113.46);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        public static final boolean ANGLE_INVERT = true;
        public static final double VOLTAGE_COMP = 12.0;
        public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

        public static final HolonomicPathFollowerConfig holomonicPathFollowerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(1.10, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.18, 0.0, 0.0), // Rotation PID constants
                Constants.Swerve.MAX_SPEED, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig());

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
}
